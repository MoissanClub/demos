#!/usr/bin/env python3
"""Compatibility launcher for the modular handshake controller."""

import asyncio

from handshake.controller import main


if __name__ == "__main__":
    try:
        raise SystemExit(asyncio.run(main()))
    except KeyboardInterrupt:
        raise SystemExit(130)
