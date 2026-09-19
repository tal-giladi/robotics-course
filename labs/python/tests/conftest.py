"""Fixtures shared by the robotlab tests."""

from __future__ import annotations

import matplotlib

matplotlib.use("Agg")  # headless: must happen before pyplot is imported anywhere

import pytest  # noqa: E402

from robotlab.config import KarmelConfig, load_config  # noqa: E402
from robotlab.sim import DiffDriveParams, SensorParams  # noqa: E402


@pytest.fixture(scope="session")
def cfg() -> KarmelConfig:
    return load_config()


@pytest.fixture(scope="session")
def ideal(cfg: KarmelConfig) -> tuple[DiffDriveParams, SensorParams]:
    return DiffDriveParams.ideal(cfg), SensorParams.ideal(cfg)


@pytest.fixture(scope="session")
def realistic(cfg: KarmelConfig) -> tuple[DiffDriveParams, SensorParams]:
    return DiffDriveParams.realistic(cfg), SensorParams.realistic(cfg)
