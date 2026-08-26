from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from types import ModuleType

import pytest


def _load_checker() -> ModuleType:
    repository_root = Path(__file__).resolve().parents[4]
    path = repository_root / "tools" / "check_repo.py"
    spec = importlib.util.spec_from_file_location("co_arm_check_repo", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_public_source_index_has_unambiguous_root_and_resolves() -> None:
    checker = _load_checker()
    failures: list[str] = []

    checker.validate_source_index(failures)

    assert failures == []
    document = json.loads(
        (checker.ROOT / "software" / "SOURCE_INDEX.json").read_text(encoding="utf-8")
    )
    assert document["schema"] == "co-arm.source-index.v2"
    assert document["softwareRoot"] == "software"
    assert document["pathsRelativeTo"] == "softwareRoot"


def test_source_index_validator_consumes_repository_relative_software_root(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    checker = _load_checker()
    software = tmp_path / "software"
    software.mkdir()
    (software / "entry.py").write_text("pass\n", encoding="utf-8")
    (software / "SOURCE_INDEX.json").write_text(
        json.dumps(
            {
                "schema": "co-arm.source-index.v2",
                "canonicalEntry": "software/SOURCE_INDEX.json",
                "softwareRoot": ".",
                "pathsRelativeTo": "softwareRoot",
                "paths": {"entry": "software/entry.py"},
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setattr(checker, "ROOT", tmp_path)
    failures: list[str] = []

    checker.validate_source_index(failures)

    assert failures == ["source index softwareRoot does not resolve to software/"]
