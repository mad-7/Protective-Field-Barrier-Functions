"""scripts/peek_training.py: blocks of episodes with a known success share."""
import importlib.util
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]


def _mod():
    spec = importlib.util.spec_from_file_location("peek_training",
                                                  REPO / "scripts" / "peek_training.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_blocks_in_episode_order_with_the_remainder_in_the_last():
    # 23 episodes, shuffled; successes exactly at episodes 20-22 (all in the last block).
    eps = [{"episode": i, "outcome": "success" if i >= 20 else "stall_truncated"}
           for i in reversed(range(23))]
    blocks = _mod().success_by_block(eps, 10)
    assert [b["n"] for b in blocks] == [2] * 9 + [5]
    assert blocks[0]["first"] == 0 and blocks[-1]["last"] == 22
    assert [b["success"] for b in blocks] == [0.0] * 9 + [0.6]
