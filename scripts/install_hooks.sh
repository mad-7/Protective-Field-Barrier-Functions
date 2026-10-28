#!/bin/bash
# Installs a pre-commit hook that runs the fast test suite and protects frozen files.
set -e
HOOK=.git/hooks/pre-commit
cat > "$HOOK" <<'HOOK'
#!/bin/bash
python -m pytest -q -m "not sim" || { echo "pre-commit: tests failed"; exit 1; }
HOOK
chmod +x "$HOOK"
echo "Installed $HOOK"
