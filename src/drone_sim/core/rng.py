"""Deterministic random streams.

Each subsystem draws from its own named stream derived from the run seed, so adding a subsystem or changing how
many numbers one draws never shifts another subsystem's numbers.
"""

from __future__ import annotations

import zlib

import numpy as np


def rng_stream(seed: int, name: str) -> np.random.Generator:
    return np.random.default_rng([seed, zlib.crc32(name.encode())])
