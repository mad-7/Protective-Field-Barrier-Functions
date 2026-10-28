"""The pure parts of the scene read-back: property comparison and value normalisation."""
import hashlib

from pfbnav.sim.scene_geometry import _plain, property_differences


def test_buffers_are_compared_by_hash_and_size():
    assert _plain(b"abc") == {"sha256": hashlib.sha256(b"abc").hexdigest(), "bytes": 3}
    assert _plain({"a": [b"x", 1.5]}) == {"a": [_plain(b"x"), 1.5]}


def test_differences_name_changed_added_and_removed_properties():
    a = {"friction": 0.5, "color": [1, 0, 0], "only_a": 1}
    b = {"friction": 0.5, "color": [1, 0, 1], "only_b": 2}
    assert property_differences(a, b) == ["color", "only_a", "only_b"]


def test_ignored_names_match_by_substring_case_insensitively():
    a = {"objectHandle": 10, "alias": "t0", "position": [0, 0], "friction": 0.5}
    b = {"objectHandle": 99, "alias": "t1", "position": [5, 5], "friction": 0.6}
    assert property_differences(a, b, ignore=("handle", "ALIAS", "position")) == ["friction"]
    assert property_differences(a, a) == []
