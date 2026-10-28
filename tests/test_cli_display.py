"""Known-answer tests for the `--display` override (hard rule 6). Pure."""
import argparse
import copy
from pathlib import Path

import pytest

from pfbnav.config import load_config
from pfbnav.utils.cli import add_display_flag, apply_display

REPO = Path(__file__).resolve().parents[1]
CFG = load_config(REPO / "configs" / "base.yaml")


def parse(*argv):
    parser = argparse.ArgumentParser()
    add_display_flag(parser)
    return parser.parse_args(list(argv))


def test_the_default_defers_to_the_config():
    assert parse().display == "config"
    cfg = apply_display(CFG, "config")
    assert cfg is CFG, "'config' must not even copy: it changes nothing"


def test_on_and_off_override_the_config():
    assert apply_display(CFG, "on")["sim"]["display_enabled"] is True
    assert apply_display(CFG, "off")["sim"]["display_enabled"] is False


def test_the_override_does_not_write_through_to_the_caller_config():
    before = copy.deepcopy(CFG)
    apply_display(CFG, "on")
    assert CFG == before, "the override edited the config it was given"


def test_the_flag_accepts_only_its_three_choices():
    assert parse("--display", "on").display == "on"
    with pytest.raises(SystemExit):
        parse("--display", "sometimes")
    with pytest.raises(ValueError):
        apply_display(CFG, "sometimes")


def test_the_config_default_is_off_so_a_run_is_not_slowed_by_accident():
    """Rendering costs ~7 ms of a ~27 ms step; watching is opt-in, per run."""
    assert CFG["sim"]["display_enabled"] is False
