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


def test_the_display_flag_changes_only_the_display_setting():
    from pfbnav.config import load_config
    from pfbnav.utils.cli import apply_display
    base = load_config(REPO / "configs" / "base.yaml")
    cfg = apply_display(base, "on")
    assert cfg["sim"]["display_enabled"] is True
    assert {k: v for k, v in cfg["sim"].items() if k != "display_enabled"} == \
        {k: v for k, v in base["sim"].items() if k != "display_enabled"}
