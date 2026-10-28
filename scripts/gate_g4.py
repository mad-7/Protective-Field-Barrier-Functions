"""Gate G4 -- a learned method's pilot seed, read against `preregistration/gates.yaml` (A15).

    python scripts/gate_g4.py --config configs/base.yaml --seed 0 \
        --train-run runs/train/L1_sac_shaped_seed9000000 \
        --eval-run runs/eval/L1_sac_shaped_<stamp>_seed0 \
        --zero-run runs/eval/zero_action_<stamp>_seed0 \
        --positive-run runs/eval/oracle_tracker_privileged_<stamp>_seed0

Reads produced files only; runs nothing. **Controls first** (ruled): if the zero-action policy does
not fail the checkpoint criterion with all timeouts, or the positive control does not pass it, the
gate is reported BROKEN and no pilot verdict is read. Every threshold is read from
`G4_learned_pilot`. **The positive control is the privileged oracle tracker** (ruled 2026-10-20,
pre-declared 2026-10-17; amendment A23): B1 deadlocks under the A20-A22 fields. B1 + recovery, under
the declared route cap, is read with `--b1-run` and RECORDED as a baseline finding, never judged.

**Consistency, checked before anything is scored:** the three evaluations ran on exactly G4's
checkpoint sets and on the frozen environment, and the pilot's training seed is one of G4's pilot
seeds (pilot seeds never enter H1 or H2). A mismatch is refused, not scored.

Writes `runs/gates/G4_<timestamp>_seed<seed>/summary.json`.

**Controls only** (before any pilot exists; ruled 2026-10-16: "then run the G4 controls"):

    python scripts/gate_g4.py --config configs/base.yaml --seed 0 --controls-only \
        --zero-run runs/eval/zero_action_<stamp>_seed0 \
        --positive-run runs/eval/oracle_tracker_privileged_<stamp>_seed0 \
        --b1-run runs/eval/B1_pure_pursuit_<stamp>_seed0

reads the two control evaluations with the same consistency checks and the same criteria, and
adds the classical recovery's statistics per episode (attempts, the outcome of each, deadlocks,
recovery distance and time), and B1's record when given. Writes `runs/gates/G4_CONTROLS_<timestamp>_seed<seed>/summary.json`.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

from pfbnav.config import load_config
from pfbnav.eval.g4 import (
    checkpoint_criterion,
    controls_verdict,
    g4_verdict,
    integrity_criterion,
    learning_criterion,
    recovery_statistics,
)
from pfbnav.utils.runio import create_run_dir

REPO_ROOT = Path(__file__).resolve().parents[1]
LOSS_COLUMNS = ("train/actor_loss", "train/critic_loss", "train/ent_coef_loss", "train/ent_coef")


def _json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _episodes(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]


def _losses(progress: Path) -> list[float]:
    """Every logged loss value; an empty cell is not a value, a 'nan' or 'inf' is."""
    if not progress.is_file():
        return [math.nan]                        # no loss log at all fails "finite losses"
    out = []
    with progress.open(encoding="utf-8") as f:
        for row in csv.DictReader(f):
            out += [float(row[c]) for c in LOSS_COLUMNS if row.get(c) not in (None, "")]
    return out


def _check_runs(g4: dict[str, Any], frozen: str, runs: dict[str, Path]) -> None:
    """Every evaluation ran on exactly G4's checkpoint sets and on the frozen environment."""
    want_sets = {Path(s).as_posix() for s in g4["checkpoint"]["sets"]}
    for name, run in runs.items():
        evaluated = _json(run / "evaluated.json")
        if set(evaluated["sets"]) != want_sets:
            raise ValueError(f"the {name} evaluation ran on {sorted(evaluated['sets'])}, "
                             f"not G4's sets {sorted(want_sets)}")
        if evaluated["environment"] != frozen:
            raise ValueError(f"the {name} evaluation did not run on the frozen environment")


def assess_controls(g4: dict[str, Any], frozen: str, zero: Path, positive: Path,
                    b1: Path | None = None) -> dict[str, Any]:
    """The two controls alone: CONTROLS SOUND or GATE BROKEN, with the recovery statistics; B1's
    run, when given, is checked for consistency and RECORDED, never judged (ruled 2026-10-20).
    Raises ValueError on an inconsistent input."""
    runs = {"zero": zero, "positive_control": positive, **({"b1": b1} if b1 else {})}
    _check_runs(g4, frozen, runs)
    ckpt = {k: float(g4["checkpoint"][k]) for k in ("min_success", "max_contact_rate")}
    summaries = {name: _json(run / "summary.json") for name, run in runs.items()}
    controls = controls_verdict(summaries["zero"]["overall"],
                                summaries["positive_control"]["overall"], **ckpt)
    recovery = {name: recovery_statistics(_episodes(run / "episodes.jsonl"))
                for name, run in runs.items() if name != "positive_control"}
    # Ruled 2026-10-16: any attempt the emulator would not release, although recovery judged its
    # direction permitted, stops the gate as a suspected recovery defect -- no verdict is read.
    defects = [d for rs in recovery.values() for d in rs["suspected_defects"]]
    verdict = ("STOP: SUSPECTED RECOVERY DEFECT" if defects
               else "CONTROLS SOUND" if controls["gate_sound"] else "GATE BROKEN")
    return {"verdict": verdict, "controls": controls, "recovery": recovery,
            "suspected_recovery_defects": defects,
            # Ruled 2026-10-19: expected zero in a static world; any is a construction gap.
            "at_rest_triggers": {name: {k: int(sm["overall"].get(k, 0)) for k in (
                "at_rest_triggers", "stationary_triggers", "creep_stops_ending_unrecoverable")}
                                 for name, sm in summaries.items()},
            "recorded_not_gated": {
                name: {"stops_per_100m": sm["overall"].get("stops_per_100m"),
                       "outcome_distribution": sm["overall"].get("shares"),
                       "per_stratum_success": {k: v.get("success_rate")
                                               for k, v in sm.get("per_stratum", {}).items()}}
                for name, sm in summaries.items()}}


def assess(g4: dict[str, Any], frozen: str, train: Path, pilot_eval: Path, zero: Path,
           positive: Path) -> dict[str, Any]:
    """The whole gate from four run directories. Raises ValueError on an inconsistent input."""
    _check_runs(g4, frozen, {"pilot": pilot_eval, "zero": zero, "positive_control": positive})
    seed = int(_json(train / "provenance.json")["seed"])
    if seed not in [int(s) for s in g4["pilot_training_seeds"]]:
        raise ValueError(f"training seed {seed} is not a G4 pilot seed {g4['pilot_training_seeds']}"
                         ": pilot seeds come from the pilot range only")

    ckpt = {k: float(g4["checkpoint"][k]) for k in ("min_success", "max_contact_rate")}
    summaries = {name: _json(run / "summary.json") for name, run in
                 (("pilot", pilot_eval), ("zero", zero), ("positive_control", positive))}
    controls = controls_verdict(summaries["zero"]["overall"],
                                summaries["positive_control"]["overall"], **ckpt)

    train_summary = _json(train / "summary.json")
    episodes = _episodes(train / "episodes.jsonl")
    environment = (_json(train / "environment.json")["environment"]["sha256"]
                   if (train / "environment.json").is_file() else None)
    integrity = integrity_criterion(
        completed=bool(train_summary.get("completed")),
        timesteps=int(train_summary.get("timesteps", 0)),
        total_timesteps=int(train_summary.get("total_timesteps", 0)) or 1,
        final_model=(train / "final_model.zip").is_file(),
        environment_sha256=environment, frozen_sha256=frozen,
        losses=_losses(train / "sb3" / "progress.csv"), episodes=episodes,
        error=train_summary.get("error"))
    learning = learning_criterion(episodes, **{k: float(v) for k, v in g4["learning"].items()})
    checkpoint = checkpoint_criterion(summaries["pilot"]["overall"], **ckpt)
    pilot = summaries["pilot"]
    resets = [float(e["reset_seconds"]) for e in episodes if "reset_seconds" in e]
    return {
        "verdict": g4_verdict(controls, integrity, learning, checkpoint),
        "controls": controls, "integrity": integrity, "learning": learning,
        "checkpoint": checkpoint, "training_seed": seed,
        "recorded_not_gated": {
            "steps_per_second": train_summary.get("steps_per_second"),
            "mean_reset_seconds": (sum(resets) / len(resets)) if resets else None,
            "stops_per_100m": pilot["overall"]["stops_per_100m"],
            "outcome_distribution": pilot["overall"]["shares"],
            "per_stratum_success": {s: v["success_rate"]
                                    for s, v in pilot.get("per_stratum", {}).items()},
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--controls-only", action="store_true",
                        help="read the two controls alone, before any pilot exists")
    for name in ("--zero-run", "--positive-run"):
        parser.add_argument(name, required=True)
    for name in ("--train-run", "--eval-run", "--b1-run"):
        parser.add_argument(name, default=None)
    args = parser.parse_args()
    if not args.controls_only and not (args.train_run and args.eval_run):
        raise SystemExit("--train-run and --eval-run are required unless --controls-only")
    cfg = load_config(args.config)
    gates = yaml.safe_load((REPO_ROOT / "preregistration" / "gates.yaml")
                           .read_text(encoding="utf-8"))
    if "G4_learned_pilot" not in gates:
        raise SystemExit("G4 is not preregistered (amendment A15); no verdict without it")
    if args.controls_only:
        report = assess_controls(gates["G4_learned_pilot"], cfg["env"]["frozen_code_sha256"],
                                 REPO_ROOT / args.zero_run, REPO_ROOT / args.positive_run,
                                 REPO_ROOT / args.b1_run if args.b1_run else None)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        run_dir = create_run_dir(REPO_ROOT / "runs" / "gates", f"G4_CONTROLS_{stamp}", args.seed,
                                 cfg)
        report["inputs"] = {"zero": args.zero_run, "positive_control": args.positive_run,
                            "b1_recorded": args.b1_run}
        (run_dir / "summary.json").write_text(json.dumps(report, indent=2, default=float),
                                              encoding="utf-8")
        print(f"G4 controls: {report['verdict']}")
        for name, n in report["at_rest_triggers"].items():
            print(f"at-rest triggers [{name}]: {n['at_rest_triggers']} (stationary "
                  f"{n['stationary_triggers']}; creep stops ending unrecoverable "
                  f"{n['creep_stops_ending_unrecoverable']})"
                  + ("  -- CONSTRUCTION GAP, report it" if n["stationary_triggers"] else "")
                  + ("  -- STOP AND REPORT" if n["creep_stops_ending_unrecoverable"] else ""))
        print(json.dumps(report["controls"], indent=1, default=float))
        for name, rs in report["recovery"].items():
            print(f"recovery [{name}]: {rs['episodes_with_recovery']}/{rs['episodes']} episodes, "
                  f"{rs['attempts']} attempts {rs['endings']}, deadlocks {rs['deadlocks']}, "
                  f"{rs['distance_m']:.2f} m, {rs['time_s']:.1f} s")
            times = rs["attempt_times_s"]
            if times:
                print(f"  attempt times: max {max(times):.1f} s, mean {sum(times) / len(times):.1f} s; "
                      f"cut short by the episode's end {rs['attempts_cut_short']}")
            for d in rs["suspected_defects"]:
                print(f"  SUSPECTED RECOVERY DEFECT: scenario {d['scenario_seed']} "
                      f"(index {d['index']}), attempt at step {d['start_step']}, heading "
                      f"{d['heading_deg']}")
        print(f"-> {run_dir.relative_to(REPO_ROOT).as_posix()}")
        return 0
    report = assess(gates["G4_learned_pilot"], cfg["env"]["frozen_code_sha256"],
                    *(REPO_ROOT / p for p in (args.train_run, args.eval_run, args.zero_run,
                                              args.positive_run)))
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "gates", f"G4_{stamp}", args.seed, cfg)
    report["inputs"] = {"train": args.train_run, "pilot_eval": args.eval_run,
                        "zero": args.zero_run, "positive_control": args.positive_run}
    (run_dir / "summary.json").write_text(json.dumps(report, indent=2, default=float),
                                          encoding="utf-8")
    print(f"G4: {report['verdict']}")
    print(json.dumps({k: report[k] for k in ("controls", "integrity", "learning", "checkpoint")},
                     indent=1, default=float))
    print(f"-> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
