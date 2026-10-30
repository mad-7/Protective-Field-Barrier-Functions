"""SAC-Lagrangian: the learner for L2 (ruled 2026-10-03).

    maximise   E[task return]                     (the unshaped reward: no proximity, no stop term)
    subject to E[protective stops TRIGGERED per episode] <= d

enforced by a Lagrange multiplier lambda >= 0 and a separate cost critic, on the same SB3 SAC with
every SAC hyperparameter shared with the other learned arms. Only the Lagrangian's own settings are
new (`lagrangian:` in `configs/experiments/L2_*.yaml`).

**The cost** is the environment's own per-step signal, `info["certified"]["stop_triggered"]`: 1 on
the step a protective stop is triggered, 0 otherwise -- the same event the reward's stop term charges
for L1 and the emulator counts, charged once per stop. A missing key raises: the cost is never
defaulted to 0. Nothing in `pfbnav/envs/` changes, so the frozen environment's identity does not move.

**The cost critic** Q_c(s, a) is one ContinuousCritic built like the reward critics (same net_arch,
optimiser, learning rate, tau, gamma), trained on

    y_c = c + gamma (1 - done) Q_c'(s', a'),   a' ~ pi(.|s')   (the SAME a' the reward target uses)

with no entropy term: entropy is a reward bonus, not a cost.

**The actor** minimises   E[ alpha log pi(a|s) - min_i Q_i(s, a) + lambda Q_c(s, a) ]   (lambda held
fixed within an update; unnormalised, as in Ha et al. 2020).

**The multiplier** is a PID-Lagrangian (Stooke, Achiam and Abbeel, ICML 2020; ruled 2026-10-27,
A28b), updated ONCE PER COMPLETED EPISODE k from c_bar_k, the EMPIRICAL mean of protective stops per
episode over the last `cost_window` = 20 completed episodes, against d:

    e_k = c_bar_k - d
    I_k = max(0, I_{k-1} + e_k)            (the integral, projected as in Stooke et al.'s
                                            Algorithm 2; I_0 = 0)
    lambda_k = min(lambda_max, max(0, K_P e_k + K_I I_k + K_D max(0, c_bar_k - c_bar_{k-1})))

with lambda_max = 100 and lambda_0 = 0. Until 20 episodes have completed, lambda is held at 0 and the
integral does not move (a mean over fewer is not the declared statistic); on the first update there
is no previous mean, so the derivative term is 0. The gains are one of three DECLARED candidates
(`PID_CANDIDATES`, A28b), chosen by the toy positive control: the first in declared order that
passes on seeds 0-3 (`scripts/validate_l2_toy.py`); if none passes, stop and report. The actor uses
the lambda in force for the whole episode; the cost critic still drives the policy gradient (lambda
Q_c in the actor's loss). Units: lambda in reward units per stop; e and c_bar in stops per episode;
K_P and K_D in (reward units per stop) per (stop per episode), K_I the same per summed episode.

**Why PID** (supervisor, 2026-10-27): projected dual ascent (A28a: lambda <- lambda + 3e-4 (c_bar -
d) on every gradient step) failed the toy control on 2 of 4 seeds at 5,000 steps. It met d on
average (0.86-1.06 stops per episode over episodes 250-1,250) but ran a limit cycle (period ~300-400
episodes, lambda 0.01-0.27), so its last iterate -- the policy that is evaluated -- depended on the
cycle's phase. Dual ascent is a pure integral controller; the proportional and derivative terms damp
it. (Earlier forms, all superseded on 2026-10-27: dual ascent on the cost critic's batch estimate
against d (1 - gamma^T) / ((1 - gamma) T); before that, once per finished episode on its measured
cost, eta 0.05, uncapped.) Units consistency with d (A28a) is kept: the statistic is stops per
episode, with no (1 - gamma^T) conversion. Each finished episode's cost is recorded beside the
multiplier in force during it (`dual_history`).

**The limit per 100 m (A30, ruled 2026-10-30).** L2's limit is restated as d = 1.0 protective stops
per 100 m driven (equivalent to 0.1 per episode at the training set's mean planned route length,
L_ref = 10.67 m). Over the window of the last 20 completed episodes the rate is
    rate = total stops / total distance driven x 100,
and the PID's error is scaled back to stops per episode at L_ref,
    e = (rate - d) x (L_ref / 100),
so the toy-validated gains keep their dynamics (the PID runs on rate x L_ref / 100 against
d x L_ref / 100: the same e, integral and derivative). Reason (ruled): unit consistency with H1
(stops per 100 m), and independence from episode length, which the training stall truncation
changes. A window with zero distance driven makes no update (recorded, `status: no_distance`).
The toy control keeps the per-episode statistic (`cost_unit: stops_per_episode`), as validated.

**The cost limit** (before A30) d = 0.1 protective stops per episode was fixed A PRIORI (ruled 2026-10-27,
superseding the 2026-10-03 selection on the tune set, which would have given L2 a per-arm tuning
budget no other learned arm has). L2 at d = 0.05 and d = 0.2 are an exploratory sensitivity, their
own method IDs (`L2-d005`, `L2-d020`, configs/methods.yaml), first in the cut order.

**With lambda held at 0 this is plain SB3 SAC, bit for bit** (`tests/test_sac_lagrangian.py`): the
training loop below is SB3 2.7.1's `SAC.train` with the cost lines added, and the cost critic is
built under a forked torch RNG so it consumes nothing the reward path draws. That test is what
catches this copy drifting from the library.

Single environment only (as `scripts/train.py` builds it): the buffer's cost column relies on it.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, NamedTuple

import numpy as np
import torch as th
from stable_baselines3 import SAC
from stable_baselines3.common.buffers import ReplayBuffer
from stable_baselines3.common.policies import ContinuousCritic
from stable_baselines3.common.utils import polyak_update
from torch.nn import functional as F

from pfbnav.control.sac import sac_kwargs
from pfbnav.training.record import RECORD_KEY

#: The cost's place in a step's `info`: the certified layer's trigger flag (`NavEnv.step`).
COST_KEYS = ("certified", "stop_triggered")


def step_cost(info: dict[str, Any]) -> float:
    """This step's cost: 1.0 if a protective stop was TRIGGERED on it, else 0.0.

    Reads `info["certified"]["stop_triggered"]`. Raises if it is absent or not a boolean: a step
    whose cost cannot be read must not be counted as free (hard rule 9).
    """
    flag = info[COST_KEYS[0]][COST_KEYS[1]]
    if not isinstance(flag, (bool, np.bool_)):
        raise TypeError(f"info['certified']['stop_triggered'] is {type(flag).__name__}, not bool")
    return 1.0 if flag else 0.0


def pid_step(mean_cost: float, previous_mean: float | None, integral: float, cost_limit: float,
             gains: tuple[float, float, float], cap: float) -> tuple[float, float]:
    """One PID-Lagrangian update (Stooke et al. 2020, Algorithm 2, capped): returns (lambda, the new
    integral).

        e = c_bar - d;   I <- max(0, I + e);
        lambda = min(cap, max(0, K_P e + K_I I + K_D max(0, c_bar - c_bar_prev)))

    `previous_mean` None (the first update) makes the derivative term 0. Costs in stops per episode.
    """
    k_p, k_i, k_d = (float(g) for g in gains)
    if min(k_p, k_i, k_d) < 0.0 or cap <= 0.0:
        raise ValueError("the PID gains are never negative and the cap is positive")
    if cost_limit < 0.0 or mean_cost < 0.0 or integral < 0.0 or (
            previous_mean is not None and previous_mean < 0.0):
        raise ValueError("the limit, a mean cost and the integral are never negative")
    error = float(mean_cost) - float(cost_limit)
    integral = max(0.0, float(integral) + error)
    rise = 0.0 if previous_mean is None else max(0.0, float(mean_cost) - float(previous_mean))
    return min(float(cap), max(0.0, k_p * error + k_i * integral + k_d * rise)), integral


def window_statistic(costs: list[float], distances: list[float], unit: str, cost_limit: float,
                     reference_length_m: float | None) -> tuple[float, float] | None:
    """(the statistic, the limit) the PID compares, in stops per episode.

    stops_per_episode (the toy control): (mean stops per episode, d).
    stops_per_100m (L2 since A30): rate = 100 x sum(stops) / sum(distance) over the window, scaled
    by L_ref / 100: (rate x L_ref / 100, d x L_ref / 100), so e = (rate - d) x L_ref / 100.
    None when the window's distance is zero (the rate is undefined; no update is made)."""
    if unit == PER_EPISODE:
        return float(np.mean(costs)), float(cost_limit)
    if unit != PER_100M:
        raise ValueError(f"unknown cost unit {unit}")
    if reference_length_m is None or reference_length_m <= 0.0:
        raise ValueError("L_ref must be positive")
    if len(distances) != len(costs):
        raise ValueError("one distance per episode in the window")
    total = float(np.sum(distances))
    if total <= 0.0:
        return None
    rate = 100.0 * float(np.sum(costs)) / total
    scale = float(reference_length_m) / 100.0
    return rate * scale, float(cost_limit) * scale


#: The three DECLARED (K_P, K_I, K_D) candidates (A28b, ruled 2026-10-27; declared before any toy run
#: with them), in their declared order. Derivation (docs/decision_log.md, 2026-10-27): K_I so that
#: the integral term ALONE reaches the oscillating run's multiplier level (0.1-0.3; its 90th
#: percentile 0.18-0.20) within ~100 episodes: the summed excess over its first 100 updates was
#: 88-116 stops per episode (seeds 0-3, mean ~104), so K_I = 0.2 / 104 ~ 2e-3. K_P so that a typical
#: excess (|e| median 0.9) moves lambda by half that level (0.1) or all of it (0.2) at once. K_D so
#: that the 95th-percentile rise of c_bar per episode (0.1) adds ~0.1; its mean rise 0.02 biases
#: lambda by ~0.02. Units: reward units per stop per (stop per episode).
PID_CANDIDATES = {1: (0.1, 2e-3, 0.0),        # PI
                  2: (0.1, 2e-3, 1.0),        # PID
                  3: (0.2, 2e-3, 1.0)}        # PID, the stronger proportional term

#: The ruled settings (2026-10-27, A28a/A28b): declared, not tuned.
RULED = {"multiplier_init": 0.0, "multiplier_max": 100.0, "cost_window": 20}
#: The two statistics: the toy control's (stops per episode, as validated) and L2's since A30.
PER_EPISODE, PER_100M = "stops_per_episode", "stops_per_100m"
#: A30 (ruled 2026-10-30): d in stops per 100 m driven. 1.0 is L2's; 0.5 and 2.0 are the exploratory
#: sensitivity, restated in proportion from 0.05 and 0.2 per episode (L2-d005, L2-d020).
RULED_LIMITS = {1.0: "L2", 0.5: "L2-d005", 2.0: "L2-d020"}


@dataclass(frozen=True)
class LagrangianSettings:
    """The `lagrangian:` block of an L2 config, validated."""

    cost_limit: float           # d, in `cost_unit`
    pid_candidate: int          # which of PID_CANDIDATES (A28b)
    multiplier_init: float      # lambda_0, reward units per stop
    multiplier_max: float       # lambda_max, reward units per stop
    cost_window: int            # completed episodes in the empirical mean compared with d
    cost_unit: str = "stops_per_episode"      # stops_per_100m for L2 since A30
    reference_length_m: float | None = None   # L_ref, m: the training set's mean planned route

    @classmethod
    def from_config(cls, cfg: dict[str, Any], *, ruled: bool = True) -> LagrangianSettings:
        """Refuses an unruled limit or multiplier setting, and any L2 config whose reward is not
        the unshaped task reward (ruled 2026-10-03: no proximity, no stop term). `ruled=False` is
        for the toy positive control alone, whose limit is its own (the multiplier settings are
        still the ruled ones). A `pid_candidate` still OPEN is refused: L2 does not train before the
        toy control has selected one (A28b)."""
        lag = cfg.get("lagrangian")
        if not lag:
            raise ValueError("the config has no `lagrangian:` block")
        d = lag.get("cost_limit")
        if d is None or d == "OPEN":
            raise ValueError("lagrangian.cost_limit is not set (ruled 2026-10-27: 0.1, a priori)")
        if ruled and float(d) not in RULED_LIMITS:
            raise ValueError(f"lagrangian.cost_limit {d} is not ruled: {sorted(RULED_LIMITS)} "
                             "stops per 100 m (1.0 for L2; 0.5, 2.0 the exploratory sensitivity)")
        unit = str(lag.get("cost_unit", PER_EPISODE))
        if unit not in (PER_EPISODE, PER_100M):
            raise ValueError(f"lagrangian.cost_unit {unit} is not one of {PER_EPISODE}, {PER_100M}")
        if ruled and unit != PER_100M:
            raise ValueError(f"lagrangian.cost_unit is {PER_100M} for L2 (A30, ruled 2026-10-30)")
        ref = lag.get("reference_length_m")
        if unit == PER_100M and (ref is None or float(ref) <= 0.0):
            raise ValueError("lagrangian.reference_length_m (L_ref, the training set's mean planned "
                             "route length) is required with stops per 100 m")
        for key, value in RULED.items():
            if float(lag[key]) != value:
                raise ValueError(f"lagrangian.{key} is declared {value} (ruled 2026-10-27), got "
                                 f"{lag[key]}")
        if "multiplier_lr" in lag:
            raise ValueError("lagrangian.multiplier_lr is retired (A28b): the multiplier is a "
                             "PID-Lagrangian, updated once per completed episode")
        candidate = lag.get("pid_candidate")
        if candidate is None or candidate == "OPEN":
            raise ValueError("lagrangian.pid_candidate is not selected: the toy control selects "
                             "the first declared candidate that passes (A28b, "
                             "scripts/validate_l2_toy.py)")
        if candidate not in PID_CANDIDATES:
            raise ValueError(f"lagrangian.pid_candidate {candidate} is not declared: "
                             f"{sorted(PID_CANDIDATES)}")
        reward = cfg["reward"]
        if float(reward["k_stop"]) != 0.0 or float(reward["k_proximity"]) != 0.0:
            raise ValueError("L2's reward is the unshaped task reward: reward.k_stop and "
                             "reward.k_proximity must be 0 (the stops are the constraint)")
        if cfg["filter"]["enabled"]:
            raise ValueError("L2 trains without the filter")
        return cls(cost_limit=float(d), pid_candidate=int(candidate),
                   multiplier_init=float(lag["multiplier_init"]),
                   multiplier_max=float(lag["multiplier_max"]),
                   cost_window=int(lag["cost_window"]), cost_unit=unit,
                   reference_length_m=None if ref is None else float(ref))


class CostReplayBufferSamples(NamedTuple):
    """SB3's `ReplayBufferSamples` with the transitions' costs."""

    observations: th.Tensor
    actions: th.Tensor
    next_observations: th.Tensor
    dones: th.Tensor
    rewards: th.Tensor
    discounts: th.Tensor | None
    costs: th.Tensor


class CostReplayBuffer(ReplayBuffer):
    """A replay buffer that also keeps each transition's `step_cost`. One environment only."""

    def __init__(self, buffer_size, observation_space, action_space, device="auto", n_envs=1,
                 optimize_memory_usage=False, handle_timeout_termination=True):
        if n_envs != 1 or optimize_memory_usage:
            raise ValueError("the cost buffer supports one environment, without memory optimisation")
        super().__init__(buffer_size, observation_space, action_space, device=device,
                         n_envs=n_envs, optimize_memory_usage=optimize_memory_usage,
                         handle_timeout_termination=handle_timeout_termination)
        self.costs = np.zeros((self.buffer_size, self.n_envs), dtype=np.float32)

    def add(self, obs, next_obs, action, reward, done, infos) -> None:
        self.costs[self.pos] = np.array([step_cost(info) for info in infos], dtype=np.float32)
        super().add(obs, next_obs, action, reward, done, infos)

    def _get_samples(self, batch_inds, env=None) -> CostReplayBufferSamples:
        samples = super()._get_samples(batch_inds, env=env)
        costs = self.to_torch(self.costs[batch_inds, 0].reshape(-1, 1))
        return CostReplayBufferSamples(*samples, costs=costs)


class SACLagrangian(SAC):
    """SB3 SAC with a cost critic and a Lagrange multiplier on the episode's stop count."""

    def __init__(self, policy, env, *, cost_limit: float | None = None,
                 pid_gains: tuple[float, float, float] | None = None,
                 multiplier_init: float = 0.0,
                 multiplier_max: float | None = None, cost_window: int | None = None,
                 cost_unit: str = "stops_per_episode", reference_length_m: float | None = None,
                 **kwargs):
        # Keyword defaults only because `SAC.load` re-creates the object with none of them and
        # restores them from the checkpoint before `_setup_model`, which validates them.
        if kwargs.get("replay_buffer_class", CostReplayBuffer) is not CostReplayBuffer:
            raise ValueError("SAC-Lagrangian needs its own replay buffer")
        kwargs["replay_buffer_class"] = CostReplayBuffer
        self.cost_limit = cost_limit
        self.pid_gains = None if pid_gains is None else tuple(float(g) for g in pid_gains)
        self.multiplier_max = multiplier_max
        self.cost_window = cost_window
        self.cost_unit = cost_unit
        self.reference_length_m = reference_length_m
        #: The distance driven in each of the last `cost_window` episodes, m (stops_per_100m only).
        self.recent_distances: list[float] = []
        #: The measured stops of the last `cost_window` completed episodes, oldest first.
        self.recent_costs: list[float] = []
        self.lagrange_multiplier = float(multiplier_init)
        #: The PID's integral of (c_bar - d), projected onto >= 0, and the previous window mean.
        self.pid_integral = 0.0
        self.previous_mean: float | None = None
        #: One entry per finished episode: its end step, measured cost, the multiplier in force
        #: during it, and (once the window is full) the window mean and the integral after it.
        self.dual_history: list[dict[str, Any]] = []
        super().__init__(policy, env, **kwargs)

    def _setup_model(self) -> None:
        super()._setup_model()
        if (self.cost_limit is None or self.pid_gains is None or self.multiplier_max is None
                or self.cost_window is None):
            raise ValueError("SAC-Lagrangian needs cost_limit, pid_gains, multiplier_max and "
                             "cost_window")
        self.pid_gains = tuple(float(g) for g in self.pid_gains)     # a checkpoint stores a list
        if (self.cost_limit < 0.0 or len(self.pid_gains) != 3 or min(self.pid_gains) < 0.0
                or self.cost_window < 1
                or not 0.0 <= self.lagrange_multiplier <= self.multiplier_max):
            raise ValueError("cost_limit >= 0, three gains >= 0, cost_window >= 1 and "
                             "0 <= multiplier <= multiplier_max")
        if self.n_envs != 1:
            raise ValueError("SAC-Lagrangian trains on one environment")
        kwargs = self.policy._update_features_extractor(self.policy.critic_kwargs, None)
        kwargs["n_critics"] = 1
        # Forked RNG: building the cost critic must not move the stream the reward path draws
        # from, or lambda = 0 would not reproduce plain SAC. Its own seed derives from the run's.
        with th.random.fork_rng(devices=[]):
            if self.seed is not None:
                th.manual_seed(int(np.random.SeedSequence([int(self.seed), 1]).generate_state(1)[0]))
            self.cost_critic = ContinuousCritic(**kwargs).to(self.device)
            self.cost_critic_target = ContinuousCritic(**kwargs).to(self.device)
        self.cost_critic_target.load_state_dict(self.cost_critic.state_dict())
        self.cost_critic_target.set_training_mode(False)
        self.cost_critic.optimizer = self.policy.optimizer_class(
            self.cost_critic.parameters(), lr=self.lr_schedule(1), **self.policy.optimizer_kwargs)
        self._episode_cost = 0.0

    # ------------------------------------------------------------------ the dual
    def _store_transition(self, replay_buffer, buffer_action, new_obs, reward, dones, infos):
        cost = step_cost(infos[0])
        super()._store_transition(replay_buffer, buffer_action, new_obs, reward, dones, infos)
        self._episode_cost += cost
        if dones[0]:
            distance = None
            if self.cost_unit == PER_100M:
                # The episode's own record (NavEnv's loop report, via EpisodeRecorder); a missing
                # distance raises -- never defaulted (hard rule 9).
                distance = float(infos[0][RECORD_KEY]["distance_m"])
            self._end_episode(self._episode_cost, distance)
            self._episode_cost = 0.0

    def _end_episode(self, episode_cost: float, distance_m: float | None = None) -> None:
        """Record the finished episode's measured cost beside the multiplier in force during it,
        keep it in the window of the last `cost_window`, and -- once the window is full -- make the
        PID update (A28b) for the next episode. An episode ending after `learning_starts` is a
        policy's."""
        entry = {"timestep": int(self.num_timesteps), "cost": episode_cost,
                 "counted": self.num_timesteps > self.learning_starts,
                 "multiplier": self.lagrange_multiplier}
        self.recent_costs = [*self.recent_costs, float(episode_cost)][-self.cost_window:]
        if self.cost_unit == PER_100M:
            if distance_m is None or distance_m < 0.0:
                raise ValueError(f"an episode's distance driven is required, got {distance_m}")
            entry["distance_m"] = float(distance_m)
            self.recent_distances = [*self.recent_distances, float(distance_m)][-self.cost_window:]
        if len(self.recent_costs) >= self.cost_window:
            update = window_statistic(self.recent_costs, self.recent_distances, self.cost_unit,
                                      self.cost_limit, self.reference_length_m)
            if update is None:
                entry["status"] = "no_distance"          # no update; lambda and I held
            else:
                mean, limit = update
                self.lagrange_multiplier, self.pid_integral = pid_step(
                    mean, self.previous_mean, self.pid_integral, limit, self.pid_gains,
                    self.multiplier_max)
                self.previous_mean = mean
                entry.update(window_mean=mean, integral=self.pid_integral)
                if self.cost_unit == PER_100M:
                    entry["window_rate_per_100m"] = mean * 100.0 / self.reference_length_m
        self.dual_history.append(entry)
        self.logger.record("lagrangian/episode_cost", episode_cost)
        self.logger.record("lagrangian/multiplier", self.lagrange_multiplier)

    # ------------------------------------------------------------------ the update
    def train(self, gradient_steps: int, batch_size: int = 64) -> None:
        """SB3 2.7.1's `SAC.train`, with the cost critic's update and lambda Q_c in the actor's loss
        (lambda moves only between episodes, in `_end_episode`). Every line not marked COST is the
        library's."""
        self.policy.set_training_mode(True)
        self.cost_critic.set_training_mode(True)                                    # COST
        optimizers = [self.actor.optimizer, self.critic.optimizer, self.cost_critic.optimizer]
        if self.ent_coef_optimizer is not None:
            optimizers += [self.ent_coef_optimizer]
        self._update_learning_rate(optimizers)

        ent_coef_losses, ent_coefs = [], []
        actor_losses, critic_losses, cost_critic_losses, cost_values = [], [], [], []
        for gradient_step in range(gradient_steps):
            multiplier = self.lagrange_multiplier             # COST: this episode's lambda
            replay_data = self.replay_buffer.sample(batch_size, env=self._vec_normalize_env)
            discounts = replay_data.discounts if replay_data.discounts is not None else self.gamma
            if self.use_sde:
                self.actor.reset_noise()

            actions_pi, log_prob = self.actor.action_log_prob(replay_data.observations)
            log_prob = log_prob.reshape(-1, 1)

            ent_coef_loss = None
            if self.ent_coef_optimizer is not None and self.log_ent_coef is not None:
                ent_coef = th.exp(self.log_ent_coef.detach())
                ent_coef_loss = -(self.log_ent_coef * (log_prob + self.target_entropy).detach()).mean()
                ent_coef_losses.append(ent_coef_loss.item())
            else:
                ent_coef = self.ent_coef_tensor
            ent_coefs.append(ent_coef.item())

            if ent_coef_loss is not None and self.ent_coef_optimizer is not None:
                self.ent_coef_optimizer.zero_grad()
                ent_coef_loss.backward()
                self.ent_coef_optimizer.step()

            with th.no_grad():
                next_actions, next_log_prob = self.actor.action_log_prob(
                    replay_data.next_observations)
                next_q_values = th.cat(self.critic_target(replay_data.next_observations,
                                                          next_actions), dim=1)
                next_q_values, _ = th.min(next_q_values, dim=1, keepdim=True)
                next_q_values = next_q_values - ent_coef * next_log_prob.reshape(-1, 1)
                target_q_values = (replay_data.rewards
                                   + (1 - replay_data.dones) * discounts * next_q_values)
                # COST: the same a', no entropy term.
                next_qc = self.cost_critic_target(replay_data.next_observations, next_actions)[0]
                target_qc = replay_data.costs + (1 - replay_data.dones) * discounts * next_qc

            current_q_values = self.critic(replay_data.observations, replay_data.actions)
            critic_loss = 0.5 * sum(F.mse_loss(current_q, target_q_values)
                                    for current_q in current_q_values)
            critic_losses.append(critic_loss.item())
            self.critic.optimizer.zero_grad()
            critic_loss.backward()
            self.critic.optimizer.step()

            # COST: the cost critic's own TD step.
            current_qc = self.cost_critic(replay_data.observations, replay_data.actions)[0]
            cost_critic_loss = 0.5 * F.mse_loss(current_qc, target_qc)
            cost_critic_losses.append(cost_critic_loss.item())
            self.cost_critic.optimizer.zero_grad()
            cost_critic_loss.backward()
            self.cost_critic.optimizer.step()

            q_values_pi = th.cat(self.critic(replay_data.observations, actions_pi), dim=1)
            min_qf_pi, _ = th.min(q_values_pi, dim=1, keepdim=True)
            qc_pi = self.cost_critic(replay_data.observations, actions_pi)[0]        # COST
            cost_values.append(qc_pi.mean().item())                                  # COST
            actor_loss = (ent_coef * log_prob - min_qf_pi + multiplier * qc_pi).mean()  # COST
            actor_losses.append(actor_loss.item())
            self.actor.optimizer.zero_grad()
            actor_loss.backward()
            self.actor.optimizer.step()

            if gradient_step % self.target_update_interval == 0:
                polyak_update(self.critic.parameters(), self.critic_target.parameters(), self.tau)
                polyak_update(self.batch_norm_stats, self.batch_norm_stats_target, 1.0)
                polyak_update(self.cost_critic.parameters(),                         # COST
                              self.cost_critic_target.parameters(), self.tau)

        self._n_updates += gradient_steps
        self.logger.record("train/n_updates", self._n_updates, exclude="tensorboard")
        self.logger.record("train/ent_coef", np.mean(ent_coefs))
        self.logger.record("train/actor_loss", np.mean(actor_losses))
        self.logger.record("train/critic_loss", np.mean(critic_losses))
        if len(ent_coef_losses) > 0:
            self.logger.record("train/ent_coef_loss", np.mean(ent_coef_losses))
        self.logger.record("train/cost_critic_loss", np.mean(cost_critic_losses))
        self.logger.record("train/cost_value", np.mean(cost_values))
        self.logger.record("lagrangian/multiplier", self.lagrange_multiplier)

    # ------------------------------------------------------------------ checkpoints
    def _excluded_save_params(self) -> list[str]:
        return [*super()._excluded_save_params(), "cost_critic", "cost_critic_target"]

    def _get_torch_save_params(self) -> tuple[list[str], list[str]]:
        state_dicts, variables = super()._get_torch_save_params()
        return [*state_dicts, "cost_critic", "cost_critic_target", "cost_critic.optimizer"], variables


def make_sac_lagrangian(env, cfg: dict[str, Any], seed: int, *,
                        tensorboard_log: str | None = None, ruled: bool = True) -> SACLagrangian:
    """A fresh L2 learner on `env`, seeded: the `training:` block exactly as `make_sac` reads it,
    plus the validated `lagrangian:` block."""
    lag = LagrangianSettings.from_config(cfg, ruled=ruled)
    return SACLagrangian("MlpPolicy", env, seed=int(seed), verbose=0,
                         tensorboard_log=tensorboard_log, cost_limit=lag.cost_limit,
                         cost_unit=lag.cost_unit, reference_length_m=lag.reference_length_m,
                         pid_gains=PID_CANDIDATES[lag.pid_candidate],
                         multiplier_init=lag.multiplier_init,
                         multiplier_max=lag.multiplier_max, cost_window=lag.cost_window,
                         **sac_kwargs(cfg["training"]))

