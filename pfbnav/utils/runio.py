"""Run-directory management. Runs are never overwritten."""
from __future__ import annotations

import hashlib
import importlib.util
import json
import subprocess
from datetime import datetime, timezone
from pathlib import Path

import yaml


def git_state(repo: Path | None = None) -> dict[str, str | bool]:
    def _git(*args: str) -> str:
        return subprocess.run(["git", *args], cwd=repo, capture_output=True,
                              text=True, check=False).stdout.strip()
    return {"commit": _git("rev-parse", "HEAD") or "unknown",
            "dirty": bool(_git("status", "--porcelain"))}


REPO_ROOT = Path(__file__).resolve().parents[2]


class SceneMismatch(RuntimeError):
    """The scene file on disk is not the scene the configuration pins."""


def file_sha256(path: str | Path) -> str:
    """SHA-256 of a file's bytes, hex."""
    digest = hashlib.sha256()
    with Path(path).open("rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def host_identity(cfg: dict | None = None) -> dict:
    """Which machine produced a run: hostname, CPU, GPU, memory, OS, and whether shared.

    A timing figure is a property of the machine, not of the method: every throughput
    number in WP0 and WP1 came from the development laptop, while training runs on a
    separate GPU machine, and nothing in the runs said so (supervisor, 2026-09-20). G2's
    per-step budget and instance count are measured on the training host, so every run
    now names the host it ran on.

    `shared` is declared in the config (`host.shared`), not detected: whether someone else
    was using the machine is not something a process can see, and a contended run's
    timings are not comparable with an uncontended one's.
    """
    import os
    import platform
    import shutil

    info: dict = {
        "hostname": platform.node(),
        "os": platform.platform(),
        "python": platform.python_version(),
        "cpu_model": platform.processor() or platform.machine(),
        "logical_cores": os.cpu_count(),
        "physical_cores": None,
        "ram_total_gb": None,
        "gpu": None,
        "shared": (cfg or {}).get("host", {}).get("shared"),
    }
    if importlib.util.find_spec("psutil") is not None:
        import psutil
        info["physical_cores"] = psutil.cpu_count(logical=False)
        info["ram_total_gb"] = round(psutil.virtual_memory().total / 1024 ** 3, 1)
    # Ruled 2026-10-29: device, driver, CUDA and cuDNN versions in every run's provenance.
    info["gpu_driver"] = gpu_driver_version()
    info["cuda"] = None
    info["cudnn"] = None
    if importlib.util.find_spec("torch") is not None:
        import torch
        info["torch"] = torch.__version__
        info["gpu"] = (torch.cuda.get_device_name(0) if torch.cuda.is_available()
                       else "none available to torch")
        info["cuda"] = torch.version.cuda               # the CUDA torch was built with, or None
        info["cudnn"] = (torch.backends.cudnn.version() if torch.cuda.is_available() else None)
    elif shutil.which("nvidia-smi"):
        info["gpu"] = "nvidia-smi present, torch not installed"
    return info


def parse_driver_version(stdout: str) -> str | None:
    """The first GPU's driver version from `nvidia-smi --query-gpu=driver_version
    --format=csv,noheader` output, or None if it printed nothing."""
    lines = [line.strip() for line in stdout.splitlines() if line.strip()]
    return lines[0] if lines else None


def gpu_driver_version() -> str | None:
    """The NVIDIA driver version, None where there is no `nvidia-smi`, and a string saying so
    where it ran and failed: "no driver" and "the query failed" are not the same record."""
    import shutil

    if shutil.which("nvidia-smi") is None:
        return None
    out = subprocess.run(["nvidia-smi", "--query-gpu=driver_version", "--format=csv,noheader"],
                         capture_output=True, text=True, check=False)
    if out.returncode != 0:
        return f"nvidia-smi failed ({out.returncode}): {out.stderr.strip()[:200]}"
    return parse_driver_version(out.stdout)


def scene_provenance(cfg: dict, repo: Path = REPO_ROOT) -> dict[str, str]:
    """The configured scene's path and SHA-256, or an empty dict if the config has none.

    The scene carries physics and collision settings that live outside `configs/` --
    since 2026-09-18 which shapes count as collidable among them -- so a commit hash
    alone does not identify what was simulated. A missing scene file is an error.
    """
    scene = cfg.get("sim", {}).get("scene")
    if not scene:
        return {}
    path = repo / scene
    if not path.is_file():
        raise FileNotFoundError(f"configured scene {path} does not exist")
    digest = file_sha256(path)
    expected = cfg.get("sim", {}).get("scene_sha256")
    if expected is not None and digest != expected:
        raise SceneMismatch(
            f"{path} has SHA-256 {digest}, but sim.scene_sha256 is {expected}. The scene "
            "was changed without the config: scene edits are deliberate, read back, "
            "committed alone, and update sim.scene_sha256 in the same commit.")
    return {"scene": str(scene), "scene_sha256": digest}


def stamp_display(run_dir: str | Path, status: dict) -> None:
    """Merge a run's display status into its `provenance.json`, after the run.

    Every run records it, not only gates: the display refusals of 2026-09-20 are
    unexplained, and a count that rises with run length -- or becomes fatal -- is the
    trigger to stop and diagnose (supervisor). A count nobody keeps cannot show a trend.
    """
    path = Path(run_dir) / "provenance.json"
    meta = json.loads(path.read_text())
    meta["display"] = status
    path.write_text(json.dumps(meta, indent=2))


def stamp_learner_device(run_dir: str | Path, device: str) -> None:
    """Merge the device the learner was actually built on into a run's `provenance.json`: the
    configured `training.device` read back from the model (ruled 2026-10-29; hard rule 13)."""
    path = Path(run_dir) / "provenance.json"
    meta = json.loads(path.read_text())
    meta["learner_device"] = str(device)
    path.write_text(json.dumps(meta, indent=2))


def create_run_dir(root: str | Path, experiment: str, seed: int, cfg: dict) -> Path:
    """Create runs/<experiment>_seed<seed>/ and record config + provenance."""
    run_dir = Path(root) / f"{experiment}_seed{seed}"
    if run_dir.exists():
        raise FileExistsError(f"{run_dir} exists; refusing to overwrite a run.")
    scene = scene_provenance(cfg)   # before mkdir: a missing scene leaves no empty run
    run_dir.mkdir(parents=True)
    with (run_dir / "config.yaml").open("w") as f:
        yaml.safe_dump(cfg, f, sort_keys=False)
    # The display state changes the step's cost but must not change its result, so it
    # belongs with the provenance rather than in a timing note (supervisor, 2026-09-20).
    meta = {"seed": seed, "experiment": experiment,
            "created_utc": datetime.now(timezone.utc).isoformat(), **git_state(), **scene,
            "display_enabled": cfg.get("sim", {}).get("display_enabled"),
            "host": host_identity(cfg)}
    (run_dir / "provenance.json").write_text(json.dumps(meta, indent=2))
    return run_dir
