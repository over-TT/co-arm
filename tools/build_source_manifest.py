#!/usr/bin/env python3
"""Build or verify the deterministic public software source manifest."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from check_repo import ROOT, SOURCE_MANIFEST, relative, release_files


def render_manifest() -> str:
    records: list[dict[str, object]] = []
    for path in release_files():
        name = relative(path)
        if not name.startswith("software/") or name == SOURCE_MANIFEST:
            continue
        payload = path.read_bytes()
        records.append(
            {
                "path": name,
                "bytes": len(payload),
                "sha256": hashlib.sha256(payload).hexdigest(),
            }
        )
    document = {
        "schema": "co-arm.source-manifest.v1",
        "root": "software",
        "files": records,
    }
    return json.dumps(document, indent=2, ensure_ascii=False) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--check",
        action="store_true",
        help="Fail if SOURCE_MANIFEST.json is absent or stale instead of rewriting it.",
    )
    arguments = parser.parse_args()
    path = ROOT / SOURCE_MANIFEST
    rendered = render_manifest()
    if arguments.check:
        try:
            current = path.read_text(encoding="utf-8")
        except OSError:
            print(f"Source manifest is missing: {relative(path)}")
            return 1
        if current != rendered:
            print(f"Source manifest is stale: {relative(path)}")
            return 1
        print(f"Source manifest is current: {relative(path)}")
        return 0
    path.write_text(rendered, encoding="utf-8", newline="\n")
    count = len(json.loads(rendered)["files"])
    print(f"Wrote {relative(path)} with {count} files.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
