"""Shared fail-closed base model for the current Arm gateway contracts."""

from pydantic import BaseModel, ConfigDict


class StrictContract(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)


__all__ = ["StrictContract"]
