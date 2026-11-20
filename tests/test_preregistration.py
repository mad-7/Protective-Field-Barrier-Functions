"""Hygiene of the preregistration files and of the text hashes. There is no lock: nothing here fails
because a file was edited."""
import hashlib
from pathlib import Path

import pytest
import yaml

from pfbnav.utils.hashing import normalise_eol, text_sha256

PREREG = Path(__file__).resolve().parents[1] / "preregistration"


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
