# pfb-nav (v2)

Protective-Field Barrier functions for learned local planning on industrial mobile robots
(KUKA youBot, CoppeliaSim, ZeroMQ Remote API).

v1 results (A* + SAC/TD3 comparison) are reproducible from tag `v1-chapter-results`.

## Setup
```bash
python -m venv .venv && source .venv/bin/activate
pip install -r requirements-dev.txt   # pinned versions (training host: requirements.txt)
pip install -e . --no-deps
bash scripts/install_hooks.sh
pytest -m "not sim"
```

## Layout
- `configs/` experiment configs (YAML, `defaults:` inheritance)
- `pfbnav/safety/` PFB filter and certified-layer emulator (independent)
- `preregistration/` gate criteria and hypotheses (frozen by `scripts/freeze_prereg.py`)
- `scenarios/{train,tune,test}/` layout splits; `test/` is frozen
- `runs/` per-run outputs (never overwritten, not tracked)
- `docs/` decision log, migration map, weekly reports
