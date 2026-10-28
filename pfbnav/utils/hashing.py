"""Content hashes that survive a change of machine. Pure; no simulator.

**Why not `sha256(path.read_bytes())`** (found 2026-09-28, before the preregistration freeze).
This repository is checked out with `core.autocrlf = true` on the development laptop, so a
source file's working copy carries CRLF line endings while the committed blob -- which is what a
Linux training host checks out -- carries LF. Hashing raw bytes gave `world/layouts.py` the
identity `493fda...` here and `76a6b4...` there. A test set written and locked on this laptop
would then have failed `verify()` on the training host, reporting a changed generator where not
a line of logic had changed; and a `LOCK.json` written here would have failed its own freeze test
the first time a file was checked out elsewhere.

Line endings are normalised to LF before hashing, so the hash is a property of the text, not of
the checkout. Nothing else is normalised: a changed character, trailing space or blank line is a
changed file.
"""
from __future__ import annotations

import hashlib
from pathlib import Path


def normalise_eol(data: bytes) -> bytes:
    """CRLF and lone CR to LF. Idempotent; LF-only input is returned unchanged."""
    return data.replace(b"\r\n", b"\n").replace(b"\r", b"\n")


def text_sha256(path: str | Path) -> str:
    """SHA-256 of a text file's content with its line endings normalised to LF."""
    return hashlib.sha256(normalise_eol(Path(path).read_bytes())).hexdigest()
