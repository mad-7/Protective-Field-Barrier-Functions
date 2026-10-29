"""Every run names the machine that produced it (supervisor, 2026-09-20).

A timing figure is a property of the host: WP0 and WP1's throughput numbers came from the
development laptop while training runs on a separate GPU machine, and nothing in the runs
recorded that. G2's per-step budget and instance count are measured on the training host,
so the host identity travels with every run.
"""
import json

from pfbnav.config import load_config
from pfbnav.utils.runio import (
    create_run_dir,
    host_identity,
    parse_driver_version,
    stamp_learner_device,
)

REQUIRED = ("hostname", "os", "cpu_model", "logical_cores", "physical_cores",
            "ram_total_gb", "gpu", "shared")


def test_host_identity_names_every_field_a_timing_is_read_against():
    info = host_identity(load_config("configs/base.yaml"))
    assert set(REQUIRED) <= set(info)
    assert isinstance(info["hostname"], str) and info["hostname"]
    assert info["logical_cores"] and info["logical_cores"] > 0


def test_shared_is_declared_not_detected():
    assert host_identity({"host": {"shared": True}})["shared"] is True
    assert host_identity({"host": {"shared": False}})["shared"] is False
    # A config that says nothing leaves it unknown rather than guessing "not shared".
    assert host_identity({})["shared"] is None
    assert host_identity()["shared"] is None


def test_gpu_is_reported_either_way():
    # Whichever machine this runs on, the field says something checkable rather than
    # being absent: "no GPU" and "nobody looked" must not be the same record.
    assert host_identity()["gpu"] is None or isinstance(host_identity()["gpu"], str)


def test_the_gpu_stack_is_recorded_either_way():
    """Ruled 2026-10-29: driver, CUDA and cuDNN versions in every run's provenance. The keys are
    always present; None means there is none on this host, not that nobody looked."""
    import torch

    info = host_identity()
    assert {"gpu_driver", "cuda", "cudnn", "torch"} <= set(info)
    assert info["cuda"] == torch.version.cuda
    if torch.cuda.is_available():
        assert info["cudnn"] == torch.backends.cudnn.version() and info["gpu_driver"]
    else:
        assert info["cudnn"] is None


def test_the_driver_version_is_parsed_from_nvidia_smi():
    assert parse_driver_version("617.14\n") == "617.14"
    assert parse_driver_version("  617.14  \n617.14\n") == "617.14"     # first GPU of two
    assert parse_driver_version("") is None and parse_driver_version("\n \n") is None


def test_the_learner_device_is_stamped_from_the_model(tmp_path):
    run = create_run_dir(tmp_path, "DEV", 0, load_config("configs/base.yaml"))
    stamp_learner_device(run, "cuda:0")
    meta = json.loads((run / "provenance.json").read_text())
    assert meta["learner_device"] == "cuda:0" and meta["seed"] == 0    # merged, nothing lost


def test_provenance_carries_the_host(tmp_path):
    cfg = load_config("configs/base.yaml")
    run = create_run_dir(tmp_path, "HOST", 0, cfg)
    meta = json.loads((run / "provenance.json").read_text())
    assert set(REQUIRED) <= set(meta["host"])
    assert meta["host"]["shared"] is False        # configs/base.yaml declares it
