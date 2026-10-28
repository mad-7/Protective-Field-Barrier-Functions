"""If preregistration is frozen, its files must match the recorded hashes."""
import hashlib
import json
from pathlib import Path

import pytest
import yaml

from pfbnav.utils.hashing import normalise_eol, text_sha256

PREREG = Path(__file__).resolve().parents[1] / "preregistration"
LOCK = PREREG / "LOCK.json"


def test_frozen_files_unchanged():
    if not LOCK.exists():
        pytest.skip("preregistration not frozen yet")
    lock = json.loads(LOCK.read_text(encoding="utf-8"))
    root = PREREG.parent
    for rel, digest in lock["sha256"].items():
        # POSIX paths and line-ending-normalised hashes, as scripts/freeze_prereg.py writes
        # them, so the lock verifies on the training host as well as on this laptop.
        assert "\\" not in rel, f"{rel}: a Windows path in the lock will not resolve on Linux"
        assert text_sha256(root / rel) == digest, rel


def test_the_generator_is_the_one_the_lock_pinned():
    """Ruled 2026-09-26 (Q2): the generator's code is part of a layout set's identity. After the
    freeze, a changed generator fails here until an amendment records why -- which is the
    point: every locked set was made by the pinned generator, and `verify()` would reject them
    all against a new one."""
    if not LOCK.exists():
        pytest.skip("preregistration not frozen yet")
    from pfbnav.world.layouts import generator_sha256
    from pfbnav.world.scenarios import scenarios_sha256
    lock = json.loads(LOCK.read_text(encoding="utf-8"))
    assert lock["generator_sha256"] == generator_sha256(), "the layout generator has changed"
    assert lock["scenarios_sha256"] == scenarios_sha256(), "the scenario module has changed"


def test_text_hashes_do_not_depend_on_line_endings(tmp_path):
    """Known answer (hard rule 6). The raw bytes of this repository's files differ between a
    CRLF checkout here and an LF one on the training host; the identity hashes must not."""
    lf = b"alpha\nbeta\n"
    (tmp_path / "lf").write_bytes(lf)
    (tmp_path / "crlf").write_bytes(b"alpha\r\nbeta\r\n")
    (tmp_path / "cr").write_bytes(b"alpha\rbeta\r")
    want = hashlib.sha256(lf).hexdigest()
    assert text_sha256(tmp_path / "lf") == want
    assert text_sha256(tmp_path / "crlf") == want
    assert text_sha256(tmp_path / "cr") == want


def test_normalising_does_not_hide_a_real_change(tmp_path):
    """Only line endings are normalised: a changed character or a trailing space is a changed
    file."""
    (tmp_path / "a").write_bytes(b"alpha\n")
    (tmp_path / "b").write_bytes(b"alpha \n")
    assert text_sha256(tmp_path / "a") != text_sha256(tmp_path / "b")
    assert normalise_eol(b"x\r\n") == normalise_eol(normalise_eol(b"x\r\n"))


class _StrictLoader(yaml.SafeLoader):
    """A loader that refuses duplicate mapping keys instead of taking the last silently."""


def _no_duplicates(loader, node, deep=False):
    seen = set()
    for key_node, _ in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in seen:
            raise yaml.constructor.ConstructorError(
                None, None, f"duplicate key {key!r}", key_node.start_mark)
        seen.add(key)
    return yaml.SafeLoader.construct_mapping(loader, node, deep=deep)


_StrictLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _no_duplicates)


def test_the_preregistration_has_no_duplicate_keys():
    """PyYAML takes the LAST of two identical keys without a word, so an edit that appends a
    key already present parses fine and reads as a contradiction. That happened to
    `positive_cases` on 2026-09-27 while committing A11: the file declared five cases and six
    at once, and only the load order decided which. In the preregistration specifically, an
    ambiguity a parser resolves by chance is not acceptable.
    """
    # Every YAML file, not a list of names: a new preregistration file (endpoints.yaml,
    # 2026-09-28) must not arrive unguarded because nobody remembered to add it here.
    for path in sorted(PREREG.glob("*.yaml")):
        yaml.load(path.read_text(encoding="utf-8"), Loader=_StrictLoader)


def test_the_preregistration_is_utf8_without_stray_nul_bytes():
    """PowerShell's `>>` writes UTF-16LE, which glued onto a UTF-8 file makes it part text and
    part binary -- it happened to AMENDMENTS.md on 2026-09-27. git then treats the file as
    binary and its history stops being readable as a diff."""
    for path in sorted(p for p in PREREG.iterdir() if p.is_file()):
        raw = path.read_bytes()
        assert b"\x00" not in raw, f"{path.name} contains NUL bytes; it is not UTF-8 text"
        raw.decode("utf-8")


# A value that marks a part of the plan still to be copied in. It may be COMMITTED -- a
# placeholder is honest about a gap -- but it may never be FROZEN: a lock over a placeholder
# would preregister the fact that something was missing rather than the thing itself.
PLACEHOLDER = "TO_BE_COPIED_FROM_THE_PLAN"


def test_the_preregistration_is_not_frozen_over_a_placeholder():
    """endpoints.yaml (2026-09-28) was drafted with the Holm family as a placeholder, because
    the family is not defined anywhere in this repository. Committing it that way is fine;
    freezing it that way would lock an empty confirmatory family."""
    if not LOCK.exists():
        pytest.skip("preregistration not frozen yet")
    for path in sorted(PREREG.glob("*.yaml")):
        assert PLACEHOLDER not in path.read_text(encoding="utf-8"), (
            f"{path.name} is frozen with a placeholder still in it")


def test_the_config_builds_what_a13_preregisters():
    """The generator reads configs/base.yaml; the preregistration is the authority. Once A13's
    placement rule is committed, the two must agree, or sets would be built by a rule nobody
    ruled (the build script refuses on the same check)."""
    path = PREREG / "scenario_sets.yaml"
    if not path.exists():
        pytest.skip("A13 (preregistration/scenario_sets.yaml) is not committed yet")
    from pfbnav.config import load_config
    from pfbnav.world.set_plan import placement_mismatches
    placement = yaml.safe_load(path.read_text(encoding="utf-8"))["unmapped_placement"]
    cfg = load_config(PREREG.parent / "configs" / "base.yaml")
    assert placement_mismatches(cfg, placement) == []
