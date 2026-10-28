import pytest
import yaml

from pfbnav.config import load_config
from pfbnav.utils.runio import create_run_dir


def test_run_dir_refuses_overwrite(tmp_path):
    create_run_dir(tmp_path, "exp", 0, {"a": 1})
    with pytest.raises(FileExistsError):
        create_run_dir(tmp_path, "exp", 0, {"a": 1})


def test_config_inheritance(tmp_path):
    (tmp_path / "base.yaml").write_text(yaml.safe_dump({"x": {"a": 1, "b": 2}, "y": 3}))
    (tmp_path / "child.yaml").write_text(yaml.safe_dump({"defaults": "base.yaml", "x": {"b": 5}}))
    assert load_config(tmp_path / "child.yaml") == {"x": {"a": 1, "b": 5}, "y": 3}


def test_shipped_experiment_configs_resolve():
    cfg = load_config("configs/experiments/L5_sac_pfb_unshaped.yaml")
    assert cfg["filter"]["enabled"] is True
    assert cfg["reward"]["k_proximity"] == 0.0
    assert cfg["certified"]["enabled"] is True


def test_deceleration_limit_never_exceeds_the_braking_assumption():
    """D9: the PFB's stopping distance must be conservative by construction.

    `certified.a_brake` is what the barrier assumes the robot can shed speed at (the PFB reads the
    same key since WP2, ruled 2026-10-23). If the
    limiter let a controller decelerate harder than that, the barrier would still be
    correct -- but the moment `a_decel_max` rose above `a_brake`, a stop commanded through
    the limiter could out-run the distance the barrier reserved for it.
    """
    cfg = load_config("configs/base.yaml")
    assert cfg["actuation"]["a_decel_max"] <= cfg["certified"]["a_brake"]


def test_g1b_tracker_gains_are_committed():
    """gates.yaml requires G1b's controller gains to live in configs/base.yaml."""
    cfg = load_config("configs/base.yaml")
    assert cfg["tracking"]["k_cross_track"] > 0.0
    assert cfg["tracking"]["k_heading"] > 0.0


def test_provenance_records_the_scene_hash(tmp_path):
    from pfbnav.utils.runio import REPO_ROOT, file_sha256

    cfg = load_config("configs/base.yaml")
    run = create_run_dir(tmp_path, "exp", 0, cfg)
    meta = __import__("json").loads((run / "provenance.json").read_text())
    assert meta["scene"] == cfg["sim"]["scene"]
    assert meta["scene_sha256"] == file_sha256(REPO_ROOT / cfg["sim"]["scene"])
    assert len(meta["scene_sha256"]) == 64


def test_file_sha256_known_answer(tmp_path):
    from pfbnav.utils.runio import file_sha256

    path = tmp_path / "abc.txt"
    path.write_bytes(b"abc")
    assert file_sha256(path) == (
        "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad")


def test_a_missing_scene_is_an_error(tmp_path):
    cfg = {"sim": {"scene": "scenes/does_not_exist.ttt"}}
    with pytest.raises(FileNotFoundError):
        create_run_dir(tmp_path, "exp", 0, cfg)
    assert not (tmp_path / "exp_seed0").exists()


def test_scene_sha256_matches_the_config():
    """Standing rule (WP0 closure): the committed scene is the one the config pins."""
    from pfbnav.utils.runio import REPO_ROOT, file_sha256

    cfg = load_config("configs/base.yaml")
    assert file_sha256(REPO_ROOT / cfg["sim"]["scene"]) == cfg["sim"]["scene_sha256"], (
        "the scene file changed without sim.scene_sha256: scene edits are deliberate, read "
        "back, committed alone, and update the pinned hash in the same commit")


def test_a_run_refuses_a_scene_with_another_hash(tmp_path):
    from pfbnav.utils.runio import SceneMismatch

    scene = tmp_path / "scenes" / "s.ttt"
    scene.parent.mkdir()
    scene.write_bytes(b"not the pinned scene")
    cfg = {"sim": {"scene": "scenes/s.ttt", "scene_sha256": "0" * 64}}
    from pfbnav.utils.runio import scene_provenance
    with pytest.raises(SceneMismatch, match="sim.scene_sha256"):
        scene_provenance(cfg, repo=tmp_path)
