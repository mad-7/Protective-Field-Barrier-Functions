"""Training and evaluation accept `--display {config,on,off}` like the probe and gate scripts."""
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("script", ["train.py", "evaluate.py"])
def test_the_script_offers_the_display_flag(script):
    out = subprocess.run([sys.executable, str(REPO / "scripts" / script), "--help"],
                         capture_output=True, text=True, check=True, cwd=REPO).stdout
    assert "--display {config,on,off}" in out


def test_rendering_is_outside_the_frozen_environment():
    """Turning the display on changes `sim.display_enabled` only; the freeze does not move."""
    from pfbnav.config import load_config
    from pfbnav.envs.freeze import freeze_problem
    from pfbnav.utils.cli import apply_display
    cfg = apply_display(load_config(REPO / "configs" / "base.yaml"), "on")
    assert cfg["sim"]["display_enabled"] is True and freeze_problem(cfg) is None
