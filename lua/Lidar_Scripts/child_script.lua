sim=require'sim'

function sysCall_init()
    maxScanDistance=5
    showLines=false

    self=sim.getObject('..')
    visionSensors={sim.getObject("../sensor1"),sim.getObject("../sensor2")}
    local collection=sim.createCollection(0)
    sim.addItemToCollection(collection,sim.handle_all,-1,0)
    sim.addItemToCollection(collection,sim.handle_tree,self,1)
    sim.setObjectInt32Param(visionSensors[1],sim.visionintparam_entity_to_render,collection)
    sim.setObjectInt32Param(visionSensors[2],sim.visionintparam_entity_to_render,collection)
    sim.setObjectFloatParam(visionSensors[1],sim.visionfloatparam_far_clipping,maxScanDistance)
    sim.setObjectFloatParam(visionSensors[2],sim.visionfloatparam_far_clipping,maxScanDistance)
    red={1,0,0}
    lines=sim.addDrawingObject(sim.drawing_lines,1,0,-1,10000,red)

    -- ---- Pre-binning constants. MUST match Python (YouBotEnv.py): ----------
    --   N_BINS         = N_LIDAR_BINS         in Python
    --   LIDAR_MAX_RNG  = LIDAR_RANGE_MAX      in Python
    --   SELF_FILTER    = LIDAR_SELF_FILTER    (filter is now server-side)
    --   LIDAR_OFFSET_X = forward offset of the LiDAR body from youBot_ref;
    --                    translates hits into the robot-centre frame so the
    --                    env's SAFE_DIST / COLLISION_THRESH thresholds become
    --                    orientation-independent and align with ROBOT_RADIUS.
    N_BINS         = 36
    LIDAR_MAX_RNG  = 5.0
    SELF_FILTER    = 0.10
    LIDAR_OFFSET_X = 0.01
    TWO_PI         = 2*math.pi
end

function sysCall_cleanup()
    sim.removeDrawingObject(lines)
end

function sysCall_sensing()
    -- Initialise all bins to MAX_RNG so any untouched direction reads as "free".
    local bins={}
    for i=1,N_BINS do bins[i]=LIDAR_MAX_RNG end

    sim.addDrawingObjectItem(lines,nil)

    for i=1,2,1 do
        local r,t,u=sim.readVisionSensor(visionSensors[i])
        if u then
            -- sensor_to_self matrix (sensors are static; safe to compute once per outer iter)
            local sensorM=sim.getObjectMatrix(visionSensors[i])
            local relRefM=sim.getObjectMatrix(self)
            relRefM=sim.getMatrixInverse(relRefM)
            relRefM=sim.multiplyMatrices(relRefM,sensorM)

            -- Sensor origin in world (start of every visual ray for this sensor)
            local origin={0,0,0}
            origin=sim.multiplyVector(sensorM,origin)
            local lineSeg={origin[1],origin[2],origin[3],0,0,0}

            -- Walk every pixel of the depth buffer
            for j=0,u[2]-1,1 do
                for k=0,u[1]-1,1 do
                    local w=2+4*(j*u[1]+k)
                    local v={u[w+1],u[w+2],u[w+3],u[w+4]}   -- (x, y, z, dist) in sensor frame

                    -- Real hits only (skip max-distance "no hit" pixels)
                    if v[4]<maxScanDistance*0.9999 then
                        -- Pixel hit -> LiDAR-body frame
                        local p=sim.multiplyVector(relRefM,v)
                        -- LiDAR-body frame -> robot-centre frame (translate +0.30 m along LiDAR +X)
                        local x=p[1]+LIDAR_OFFSET_X
                        local y=p[2]
                        local rng=math.sqrt(x*x + y*y)
                        if rng>=SELF_FILTER then
                            local ang=math.atan2(y,x)
                            local idx=math.floor((ang+math.pi)/TWO_PI * N_BINS) + 1
                            if idx<1 then idx=1 end
                            if idx>N_BINS then idx=N_BINS end
                            if rng<bins[idx] then bins[idx]=rng end
                        end
                    end

                    -- Visual rays in CoppeliaSim (unchanged from default behaviour)
                    if showLines then
                        local p=sim.multiplyVector(sensorM,v)
                        lineSeg[4]=p[1]
                        lineSeg[5]=p[2]
                        lineSeg[6]=p[3]
                        sim.addDrawingObjectItem(lines,lineSeg)
                    end
                end
            end
        end
    end

    -- 36 floats (144 bytes), already in robot-centre frame and self-filtered.
    sim.setBufferSignal('youBot_lidar', sim.packFloatTable(bins))
end