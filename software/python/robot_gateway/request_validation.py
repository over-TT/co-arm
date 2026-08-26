"""Strict, size-bounded JSON request parsing shared by Arm HTTP routes."""

from __future__ import annotations

import json
from typing import NoReturn, TypeVar

from fastapi import HTTPException, Request
from pydantic import BaseModel, ValidationError


Contract = TypeVar("Contract", bound=BaseModel)
MAX_REQUEST_BODY_BYTES = 65_536


def _reject_non_json_number(value: str) -> NoReturn:
    raise ValueError(f"{value} is not a finite JSON number")


async def strict_body(request: Request, contract: type[Contract]) -> Contract:
    """Parse RFC-compliant JSON and return JSON-safe validation errors."""

    content_type = request.headers.get("content-type", "").split(";", 1)[0].strip().lower()
    if content_type != "application/json" and not content_type.endswith("+json"):
        raise HTTPException(status_code=415, detail="Content-Type must be application/json.")

    declared_length = request.headers.get("content-length")
    if declared_length is not None:
        try:
            parsed_length = int(declared_length)
        except ValueError as error:
            raise HTTPException(
                status_code=400,
                detail="Content-Length must be a non-negative integer.",
            ) from error
        if parsed_length < 0:
            raise HTTPException(
                status_code=400,
                detail="Content-Length must be a non-negative integer.",
            )
        if parsed_length > MAX_REQUEST_BODY_BYTES:
            raise HTTPException(
                status_code=413,
                detail=f"Request body exceeds the {MAX_REQUEST_BODY_BYTES}-byte limit.",
            )

    chunks: list[bytes] = []
    received = 0
    async for chunk in request.stream():
        received += len(chunk)
        if received > MAX_REQUEST_BODY_BYTES:
            raise HTTPException(
                status_code=413,
                detail=f"Request body exceeds the {MAX_REQUEST_BODY_BYTES}-byte limit.",
            )
        chunks.append(chunk)
    try:
        payload = json.loads(b"".join(chunks), parse_constant=_reject_non_json_number)
    except (json.JSONDecodeError, UnicodeDecodeError, ValueError) as error:
        raise HTTPException(
            status_code=422,
            detail="Request body must be strict JSON; NaN and Infinity are not allowed.",
        ) from error
    try:
        return contract.model_validate(payload)
    except ValidationError as error:
        safe_errors = [
            {
                "type": str(item["type"]),
                "loc": list(item["loc"]),
                "msg": str(item["msg"]),
            }
            for item in error.errors()
        ]
        raise HTTPException(status_code=422, detail=safe_errors) from error


__all__ = ["MAX_REQUEST_BODY_BYTES", "strict_body"]
