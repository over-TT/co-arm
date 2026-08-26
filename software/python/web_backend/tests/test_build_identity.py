from __future__ import annotations

from pathlib import Path

from web_backend.build_identity import build_identity


def _write(root: Path, relative: str, value: str) -> None:
    path = root / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(value, encoding="utf-8")


def test_build_identity_covers_backend_frontend_source_and_dist(tmp_path: Path) -> None:
    _write(tmp_path, "python/web_backend/app.py", "app = 1\n")
    _write(tmp_path, "dashboard/src/App.tsx", "export const app = 1;\n")
    _write(tmp_path, "dashboard/dist/index.html", "<p>one</p>\n")
    _write(tmp_path, "dashboard/package.json", "{}\n")
    _write(tmp_path, "python/pyproject.toml", "[project]\nname='co-arm-stack'\n")
    _write(tmp_path, "python/arm_mcp/server.py", "TOOLS = 1\n")
    original = build_identity(tmp_path)

    _write(tmp_path, "dashboard/src/App.tsx", "export const app = 2;\n")
    source_changed = build_identity(tmp_path)
    _write(tmp_path, "dashboard/dist/index.html", "<p>two</p>\n")
    dist_changed = build_identity(tmp_path)
    _write(tmp_path, "python/web_backend/app.py", "app = 2\n")
    backend_changed = build_identity(tmp_path)
    _write(tmp_path, "python/arm_mcp/server.py", "TOOLS = 2\n")
    mcp_changed = build_identity(tmp_path)
    _write(tmp_path, "python/pyproject.toml", "[project]\nname='co-arm-stack-v2'\n")
    launch_input_changed = build_identity(tmp_path)

    assert len(
        {
            original,
            source_changed,
            dist_changed,
            backend_changed,
            mcp_changed,
            launch_input_changed,
        }
    ) == 6
