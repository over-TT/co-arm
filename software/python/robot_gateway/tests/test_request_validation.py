from __future__ import annotations

from fastapi import FastAPI, Request
from fastapi.testclient import TestClient
import pytest
from pydantic import Field

from robot_gateway.request_validation import MAX_REQUEST_BODY_BYTES, strict_body
from robot_gateway.strict_contract import StrictContract


class ExampleRequest(StrictContract):
    value: int = Field(strict=True, ge=0, le=10)


def _client() -> TestClient:
    app = FastAPI()

    @app.post("/strict")
    async def strict(request: Request) -> dict[str, int]:
        body = await strict_body(request, ExampleRequest)
        return {"value": body.value}

    return TestClient(app)


@pytest.mark.parametrize("content_type", ["application/json", "application/problem+json"])
def test_strict_body_accepts_json_media_types(content_type: str) -> None:
    response = _client().post(
        "/strict",
        content=b'{"value":4}',
        headers={"Content-Type": content_type},
    )
    assert response.status_code == 200
    assert response.json() == {"value": 4}


def test_strict_body_requires_json_content_type() -> None:
    response = _client().post(
        "/strict",
        content=b'{"value":4}',
        headers={"Content-Type": "text/plain"},
    )
    assert response.status_code == 415


@pytest.mark.parametrize("body", [b"{", b'{"value":NaN}', b'{"value":Infinity}'])
def test_strict_body_rejects_non_json_values(body: bytes) -> None:
    response = _client().post(
        "/strict",
        content=body,
        headers={"Content-Type": "application/json"},
    )
    assert response.status_code == 422


@pytest.mark.parametrize(
    "body",
    [b'{"value":true}', b'{"value":11}', b'{"value":4,"extra":1}'],
)
def test_strict_body_enforces_the_exact_contract(body: bytes) -> None:
    response = _client().post(
        "/strict",
        content=body,
        headers={"Content-Type": "application/json"},
    )
    assert response.status_code == 422


@pytest.mark.parametrize("length", ["-1", "not-a-number"])
def test_strict_body_rejects_invalid_content_length(length: str) -> None:
    response = _client().post(
        "/strict",
        content=b'{"value":4}',
        headers={"Content-Type": "application/json", "Content-Length": length},
    )
    assert response.status_code == 400


def test_strict_body_rejects_declared_oversize_payload() -> None:
    response = _client().post(
        "/strict",
        content=b'{"value":4}',
        headers={
            "Content-Type": "application/json",
            "Content-Length": str(MAX_REQUEST_BODY_BYTES + 1),
        },
    )
    assert response.status_code == 413


def test_strict_body_rejects_received_oversize_payload() -> None:
    response = _client().post(
        "/strict",
        content=b" " * (MAX_REQUEST_BODY_BYTES + 1),
        headers={"Content-Type": "application/json"},
    )
    assert response.status_code == 413
