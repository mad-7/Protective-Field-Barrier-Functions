-- scanner_v2.lua -- one tick's scan, binned inside the simulator, published as one packet.
--
-- The contract this must satisfy is pfbnav/sim/scan_packet.py, which was written first
-- and is tested on synthetic geometry without a simulator. This file produces exactly the
-- float layout `encode_scan` documents; `decode_scan` refuses anything else.
--
-- Why server-side (measured 2026-09-20): reading the four vision sensors over the remote
-- API costs 14-17 ms per step in round trips alone, and the cost is latency, not data.
-- Binning here and returning ~1.7 kB once per tick removes eight round trips.
--
-- Loaded from lua/ at run time by the Python side (sim.createScript), never saved into the
-- scene: the scene stays pinned by its SHA-256 (hard rule 12), and the scanner is versioned
-- as source in git.
--
-- Frames: ranges and bearings are in the BODY frame -- +x chassis-forward, +y left,
-- bearings in (-pi, pi] -- about the body origin, because the certified field, the
-- footprint table and the PFB are all defined there. The body frame is `robot_ref`
-- rotated by `robot.yaw_offset` (pi/2: youBot_ref's +x is a quarter turn from
-- chassis-forward, as in v1), so every point read relative to `robot_ref` is rotated by
-- -yaw_offset before its bearing is taken. The first versions of this script skipped
-- that and reported every bearing 90 deg off (found 2026-09-21).
--
-- Validity (hard rule 5) comes from the Python side as a per-bin mask
-- (pfbnav/sim/coverage.py): a bin is valid only if its whole sector, from the footprint
-- edge out to scanners.coverage_depth beyond it, is seen by some sensor from where that
-- sensor actually is. The first version judged coverage by angle from the body origin, as
-- if every sensor sat there, and reported a blind zone beside the robot as free space
-- (2026-09-21). A covered bin with no return is valid at range_max (free space, measured).
--
-- The robot is rendered, except each sensor's OWN scanner model: each sensor sees the
-- chassis and the other scanners and is occluded by them as the renderer decides. A
-- return inside the footprint (+ self_filter_margin) is the robot and is dropped
-- (supervisor, 2026-09-21: model self-occlusion, filter self-returns). Rendering the own
-- model too (d7da848) blinded every sensor: all 512 pixels hit the pod's casing at
-- 0.0094 m, just past the 0.009 m near plane, every return fell inside the footprint, and
-- the coverage map read range_max at all 7,852 cells (SELFOCC_20260922T163041Z).

sim = require 'sim'

local CONFIG_SIGNAL = 'signal.pfbnav.scanner.config'
local SCAN_SIGNAL = 'signal.pfbnav.scan'
local STATUS_SIGNAL = 'signal.pfbnav.scanner.status'
local FORMAT_VERSION = 2   -- must equal pfbnav.sim.scan_packet.FORMAT_VERSION
local TWO_PI = 2 * math.pi

local cfg, robot, sensors, policy, filterStream
local cosYaw, sinYaw        -- rotation from robot_ref into the body frame: -yaw_offset
local footprintN, footprintStep

-- The footprint's extent (m) at a body-frame bearing (rad), interpolated from the table
-- the Python side sends: footprint_extent[k] is the extent at -180 + (k-1)*step degrees.
local function footprintExtent(bearing)
    local pos = (bearing * 180.0 / math.pi + 180.0) / footprintStep
    local i0 = math.floor(pos)
    local frac = pos - i0
    i0 = i0 % footprintN
    local i1 = (i0 + 1) % footprintN
    return cfg.footprint_extent[i0 + 1] * (1.0 - frac) + cfg.footprint_extent[i1 + 1] * frac
end

-- Rays for a sensor of this width: width x density, rounded to 4 decimals before the
-- ceiling exactly as pfbnav.sim.scanner.rays_for does.
local function raysFor(fovDeg)
    local exact = math.floor(fovDeg * cfg.rays_per_degree * 10000 + 0.5) / 10000
    return math.max(1, math.ceil(exact))
end

-- A point given relative to robot_ref, in the body frame.
local function toBody(x, y)
    return cosYaw * x - sinYaw * y, sinYaw * x + cosYaw * y
end

-- Bin index (1-based) of a bearing in a uniform binning of the full turn.
local function binOf(bearing, n)
    local i = math.floor((bearing + math.pi) / TWO_PI * n) + 1
    if i < 1 then i = 1 elseif i > n then i = n end
    return i
end

-- Centre bearing of bin i (1-based), matching pfbnav.sim.scan_packet.bin_bearings.
local function binCentre(i, n)
    return -math.pi + TWO_PI * (i - 0.5) / n
end

-- The scanner model a sensor belongs to: its ancestor just below the robot root, found
-- exactly as pfbnav.sim.scanner.Scanner.scanner_models finds it (by ancestry, not name).
local function ownModel(h, tree)
    local sensor = h
    local parent = sim.getObjectParent(h)
    while parent ~= tree and parent ~= -1 do
        h, parent = parent, sim.getObjectParent(parent)
    end
    if parent == -1 then
        error('scanner_v2: sensor ' .. sensor .. ' is not inside ' .. cfg.robot_tree)
    end
    return h
end

-- Everything except one subtree, as a collection. Created in sysCall_init, i.e. while the
-- simulation runs, so it is destroyed at stop with the script. A handle_all item covers
-- objects created later too: the diagnostic's post, created after its collections, was
-- rendered through them (SELFOCC_20260922T163041Z).
local function everythingExcept(root)
    local col = sim.createCollection(0)
    sim.addItemToCollection(col, sim.handle_all, -1, 0)
    sim.addItemToCollection(col, sim.handle_tree, root, 1)
    return col
end

local function newStream(n, mask)
    local s = {n = n, range = {}, bearing = {}, valid = {}}
    for i = 1, n do
        s.range[i] = cfg.range_max
        s.bearing[i] = binCentre(i, n)
        s.valid[i] = mask[i] == 1
    end
    return s
end

function sysCall_init()
    local raw = sim.getBufferProperty(sim.handle_scene, CONFIG_SIGNAL, {noError = true})
    if not raw then
        error('scanner_v2: no configuration at ' .. CONFIG_SIGNAL ..
              '; the Python side sets it before starting the simulation')
    end
    cfg = sim.unpackTable(raw)
    if cfg.yaw_offset == nil then
        error('scanner_v2: the configuration carries no yaw_offset; without it every '
              .. 'bearing is reported in robot_ref, a quarter turn from the body frame')
    end
    cosYaw, sinYaw = math.cos(-cfg.yaw_offset), math.sin(-cfg.yaw_offset)
    for _, key in ipairs({'policy_valid_mask', 'filter_valid_mask', 'footprint_extent',
                          'rays_per_degree', 'self_filter_margin'}) do
        if cfg[key] == nil then
            error('scanner_v2: the configuration carries no ' .. key)
        end
    end
    if #cfg.policy_valid_mask ~= cfg.policy_bins or #cfg.filter_valid_mask ~= cfg.filter_bins then
        error('scanner_v2: validity masks do not match the bin counts')
    end
    footprintN = #cfg.footprint_extent
    footprintStep = cfg.footprint_step_deg

    robot = sim.getObject(cfg.robot_ref)
    local tree = sim.getObject(cfg.robot_tree)

    sensors = {}
    for _, h in ipairs(sim.getObjectsInTree(tree, sim.sceneobject_visionsensor, 0)) do
        -- The main script must not also render these: we handle them, once, per tick.
        sim.setExplicitHandling(h, 1)
        local fov = sim.getObjectFloatParam(h, sim.visionfloatparam_perspective_angle)
        local wanted = cfg.resolution_x or raysFor(fov * 180.0 / math.pi)
        sim.setObjectInt32Param(h, sim.visionintparam_resolution_x, wanted)
        sim.setObjectFloatParam(h, sim.visionfloatparam_far_clipping, cfg.range_max)
        -- Render everything except this sensor's own scanner model: the rest of the robot
        -- occludes as it really would, and its returns are removed by the footprint filter
        -- below. The own model is the sensor's casing, not something it sees.
        local model = ownModel(h, tree)
        local renderSet = everythingExcept(model)
        sim.setObjectInt32Param(h, sim.visionintparam_entity_to_render, renderSet)
        local rendered = sim.getObjectInt32Param(h, sim.visionintparam_entity_to_render)
        if rendered ~= renderSet then
            error('scanner_v2: sensor ' .. h .. ' renders ' .. rendered .. ', not the set '
                  .. renderSet .. ' excluding its own model ' .. model)
        end

        local res = sim.getObjectInt32Param(h, sim.visionintparam_resolution_x)
        if res ~= wanted then
            error('scanner_v2: sensor ' .. h .. ' asked for ' .. wanted .. ' rays, has ' .. res)
        end
        local rows = sim.getObjectInt32Param(h, sim.visionintparam_resolution_y)
        -- Pixel i's horizontal offset on the image plane, as tan of its angle off the axis:
        -- tan(fov/2) * (2(i-0.5)/res - 1), negated when the pixel order is mirrored. The
        -- same arithmetic as pfbnav/sim/depth_geometry.pixel_tangents, which is tested;
        -- the convention (`pixel_mirror`) is set by the multi-wall check, not assumed --
        -- assuming it put a wall on a line tilted 60 deg (2026-09-21).
        local tans = {}
        local halfTan = math.tan(0.5 * fov)
        local sign = cfg.pixel_mirror and -1.0 or 1.0
        for i = 1, res do
            tans[i] = sign * halfTan * (2.0 * (i - 0.5) / res - 1.0)
        end
        sensors[#sensors + 1] = {handle = h, res = res, rows = rows, fov = fov, tans = tans,
                                 model = model}
    end
    if #sensors == 0 then
        error('scanner_v2: no vision sensors under ' .. cfg.robot_tree)
    end

    -- Each sensor's optical axis in the robot frame, in degrees. Reported because the
    -- depth-buffer question (perpendicular z or euclidean range) shows up as an error
    -- that grows with the angle off an axis, and a diagnosis needs the axes to plot it
    -- against (scripts/diag_scanner_check.py).
    local axes, rays, models = {}, {}, {}
    for i, s in ipairs(sensors) do
        rays[i] = s.res
        models[i] = s.model
        local m = sim.getObjectMatrix(s.handle, robot)
        local origin = sim.multiplyVector(m, {0, 0, 0})
        local ahead = sim.multiplyVector(m, {0, 0, 1})
        local bx, by = toBody(ahead[1] - origin[1], ahead[2] - origin[2])
        axes[i] = math.atan2(by, bx) * 180.0 / math.pi
    end

    sim.setBufferProperty(sim.handle_scene, STATUS_SIGNAL, sim.packTable({
        format_version = FORMAT_VERSION, sensors = #sensors,
        resolution_x = sensors[1].res, resolution_y = sensors[1].rows,
        rays_per_sensor = rays,
        fov_deg = sensors[1].fov * 180.0 / math.pi,
        sensor_axes_deg = axes,
        pixel_mirror = cfg.pixel_mirror and true or false,
        depth_is_euclidean = cfg.depth_is_euclidean and true or false,
        robot_rendered = true,
        render = 'everything except own scanner model',
        own_models = models,
        self_filter = 'footprint',
        yaw_offset = cfg.yaw_offset,
        frame = 'body',
    }))
end

local function record(stream, bearing, range)
    local i = binOf(bearing, stream.n)
    if range < stream.range[i] then
        stream.range[i] = range
        -- The bearing the range was measured at, which is what the filter stream needs;
        -- an empty bin keeps its centre.
        stream.bearing[i] = bearing
    end
end

function sysCall_sensing()
    -- sysCall_sensing runs after the step's physics, while sim.getSimulationTime() still
    -- reports the time the step started, so the scan of the post-physics world is stamped
    -- one period on (measured 2026-09-21; see `sense`).
    lastStateTime = sim.getSimulationTime() + sim.getSimulationTimeStep()
    sense(lastStateTime)
end

-- A re-render of the SAME tick, called by the client between steps (2026-10-23, the scan-fault
-- fix): the physics state has not changed, so a correct render is identical to the first, and the
-- packet keeps the tick's stamp -- by then the simulation clock has advanced, so it is not re-read.
function rescan()
    if lastStateTime == nil then
        error('scanner_v2: rescan before any sensing pass')
    end
    sense(lastStateTime)
    return lastStateTime
end

function sense(stateTime)
    policy = newStream(cfg.policy_bins, cfg.policy_valid_mask)
    filterStream = newStream(cfg.filter_bins, cfg.filter_valid_mask)

    for _, s in ipairs(sensors) do
        sim.handleVisionSensor(s.handle)
        local depth = sim.getVisionSensorDepth(s.handle, 1)      -- metres, along +z
        local d = sim.unpackFloatTable(depth)
        local m = sim.getObjectMatrix(s.handle, robot)           -- sensor -> robot frame

        -- Every return in the sensor frame, then ONE transform for all of them:
        -- sim.multiplyVector takes 3n values. At 512 rays per sensor a call per pixel cost
        -- ~10 ms a step (SCANRES, 2026-09-21); the arithmetic is the same either way.
        local flat, n = {}, 0
        for i = 1, s.res do
            local depthValue = d[i]
            if depthValue and depthValue < cfg.range_max - 1e-6 then   -- a return, not sky
                local t = s.tans[i]
                if cfg.depth_is_euclidean then
                    local norm = math.sqrt(1.0 + t * t)
                    flat[n + 1], flat[n + 2], flat[n + 3] =
                        depthValue * t / norm, 0, depthValue / norm
                else
                    flat[n + 1], flat[n + 2], flat[n + 3] = depthValue * t, 0, depthValue
                end
                n = n + 3
            end
        end
        if n > 0 then
            local p = sim.multiplyVector(m, flat)
            for k = 1, n, 3 do
                local x, y = toBody(p[k], p[k + 1])
                local range = math.sqrt(x * x + y * y)
                -- range_min is the self-filter; range_max matters because the far clipping
                -- plane is perpendicular to the axis, so an off-axis return can lie beyond
                -- range_max from the body origin. That is not a measurement inside the
                -- scanner's range: the bin stays free at range_max, which the decoder
                -- requires (a valid range above range_max is refused).
                local bearing = math.atan2(y, x)
                -- A return inside the footprint is the robot itself (it is rendered).
                local selfReturn = range <= footprintExtent(bearing) + cfg.self_filter_margin
                if not selfReturn and range >= cfg.range_min and range <= cfg.range_max then
                    record(policy, bearing, range)
                    record(filterStream, bearing, range)
                end
            end
        end
    end

    -- The packet, in pfbnav/sim/scan_packet.py's layout. Any change here changes
    -- FORMAT_VERSION there, and the decoder refuses the mismatch.
    -- The time of the STATE this scan describes, not of the pass that took it (`stateTime`,
    -- from sysCall_sensing or the tick being re-rendered). Measured 2026-09-21: the packet read
    -- back after sim.step() was exactly dt behind the client's clock, which the stamp corrects.
    local out = {FORMAT_VERSION, stateTime, policy.n, filterStream.n,
                 cfg.range_min, cfg.range_max}
    for i = 1, policy.n do out[#out + 1] = policy.range[i] end
    for i = 1, policy.n do out[#out + 1] = policy.valid[i] and 1.0 or 0.0 end
    for i = 1, filterStream.n do out[#out + 1] = filterStream.bearing[i] end
    for i = 1, filterStream.n do out[#out + 1] = filterStream.range[i] end
    for i = 1, filterStream.n do out[#out + 1] = filterStream.valid[i] and 1.0 or 0.0 end
    sim.setBufferProperty(sim.handle_scene, SCAN_SIGNAL, sim.packFloatTable(out))
end
