"""Fail-safe BrainCo hand command worker for reviewed trace replay."""
from __future__ import annotations

import asyncio
import queue
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional, Sequence


EventCallback = Callable[[str, dict], None]


@dataclass
class _Command:
    positions: tuple[int, ...]
    reason: str
    completed: threading.Event
    retries: int = 1
    error: Optional[BaseException] = None


class BrainCoHandReplay:
    """Own one serial hand and reopen it on every normal shutdown path."""

    def __init__(
        self, sdk, port: str, baud_enum, slave_id: int, event: EventCallback,
        *, timeout_seconds: float = 2.0,
    ) -> None:
        self.sdk = sdk
        self.port = port
        self.baud_enum = baud_enum
        self.slave_id = int(slave_id)
        self.event = event
        self.timeout_seconds = float(timeout_seconds)
        self._commands: queue.Queue[Optional[_Command]] = queue.Queue()
        self._ready = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._error: Optional[BaseException] = None
        self._schedulers: list[threading.Thread] = []
        self.open_positions: Optional[tuple[int, ...]] = None

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("BrainCo replay worker already started")
        self._thread = threading.Thread(target=self._thread_main, daemon=True)
        self._thread.start()
        if not self._ready.wait(self.timeout_seconds + 1.0):
            raise RuntimeError("BrainCo replay worker initialization timed out")
        self.raise_if_failed()
        if self.open_positions is None:
            raise RuntimeError("BrainCo worker did not capture an open reference")
        self.open("initial_open", wait=True, retries=3)

    def _thread_main(self) -> None:
        try:
            asyncio.run(self._run())
        except BaseException as exc:
            self._error = exc
            self._ready.set()

    async def _run(self) -> None:
        context = await asyncio.wait_for(
            self.sdk.modbus_open(self.port, self.baud_enum), self.timeout_seconds
        )
        await asyncio.wait_for(
            context.set_finger_unit_mode(
                self.slave_id, self.sdk.FingerUnitMode.Normalized
            ),
            self.timeout_seconds,
        )
        observed_mode = await asyncio.wait_for(
            context.get_finger_unit_mode(self.slave_id), self.timeout_seconds
        )
        self.event("brainco_hand_unit_mode", {
            "requested": "Normalized",
            "observed": str(observed_mode),
        })
        initial_status = None
        for attempt in range(1, 4):
            try:
                initial_status = await asyncio.wait_for(
                    context.get_motor_status(self.slave_id), self.timeout_seconds
                )
                break
            except BaseException as exc:
                self.event("brainco_hand_open_reference_retry", {
                    "attempt": attempt,
                    "maximum_attempts": 3,
                    "error": f"{type(exc).__name__}: {exc}",
                })
                if attempt < 3:
                    await asyncio.sleep(0.1)
                else:
                    raise
        assert initial_status is not None
        initial_positions = tuple(int(value) for value in initial_status.positions)
        if len(initial_positions) != 6 or any(
            value < 0 or value > 10000 for value in initial_positions
        ):
            raise RuntimeError("BrainCo open reference is outside normalized bounds")
        self.open_positions = initial_positions
        self.event("brainco_hand_open_reference", {
            "positions": list(initial_positions),
            "basis": "fresh_measured_normalized_pose",
        })
        self._ready.set()
        try:
            while True:
                try:
                    command = self._commands.get_nowait()
                except queue.Empty:
                    await asyncio.sleep(0.005)
                    continue
                if command is None:
                    break
                for attempt in range(1, command.retries + 1):
                    try:
                        await asyncio.wait_for(
                            context.set_finger_positions_and_speeds(
                                self.slave_id, list(command.positions), [100] * 6
                            ),
                            self.timeout_seconds,
                        )
                        status = await asyncio.wait_for(
                            context.get_motor_status(self.slave_id), self.timeout_seconds
                        )
                        measured = list(getattr(status, "positions", []))
                        self.event("brainco_hand_command", {
                            "positions": list(command.positions),
                            "reason": command.reason,
                            "measured_positions": measured,
                            "attempt": attempt,
                        })
                        command.error = None
                        break
                    except BaseException as exc:
                        command.error = exc
                        self.event("brainco_hand_command_retry", {
                            "positions": list(command.positions),
                            "reason": command.reason,
                            "attempt": attempt,
                            "maximum_attempts": command.retries,
                            "error": f"{type(exc).__name__}: {exc}",
                        })
                        if attempt < command.retries:
                            await asyncio.sleep(0.1)
                if command.error is not None and attempt == command.retries:
                    self._error = command.error
                command.completed.set()
        finally:
            close = getattr(context, "close", None)
            if callable(close):
                result = close()
                if asyncio.iscoroutine(result):
                    await result

    def command(
        self, positions: Sequence[int], reason: str, *, wait: bool = False,
        retries: int = 1, ignore_existing_error: bool = False,
    ) -> None:
        if not ignore_existing_error:
            self.raise_if_failed()
        values = tuple(int(value) for value in positions)
        if len(values) != 6 or any(value < 0 or value > 10000 for value in values):
            raise ValueError("reviewed BrainCo command must contain six normalized positions in [0, 10000]")
        if not 1 <= retries <= 3:
            raise ValueError("BrainCo command retries must be between one and three")
        command = _Command(values, reason, threading.Event(), retries=retries)
        self._commands.put(command)
        if wait and not command.completed.wait(
            self.timeout_seconds * 2.0 * retries + 0.5 * (retries - 1)
        ):
            raise RuntimeError("BrainCo hand command completion timed out")
        if command.error is not None:
            raise RuntimeError(
                f"BrainCo hand command failed: {type(command.error).__name__}: {command.error}"
            )
        if not ignore_existing_error:
            self.raise_if_failed()

    def start_close_ramp(self, *, step: int = 50, period_seconds: float = 0.2) -> None:
        if self.open_positions is None:
            raise RuntimeError("BrainCo replay worker has no open reference")
        baseline = self.open_positions

        def schedule():
            for delta in range(step, 501, step):
                self.command(
                    tuple(min(10000, value + delta) for value in baseline),
                    "source_close_ramp",
                )
                time.sleep(period_seconds)
        thread = threading.Thread(target=self._guard_scheduler(schedule), daemon=True)
        self._schedulers.append(thread)
        thread.start()

    def schedule_open(self, delay_seconds: float) -> None:
        def schedule():
            time.sleep(delay_seconds)
            self.open("scaled_source_open", wait=True)
        thread = threading.Thread(target=self._guard_scheduler(schedule), daemon=True)
        self._schedulers.append(thread)
        thread.start()

    def _guard_scheduler(self, operation):
        def guarded():
            try:
                operation()
            except BaseException as exc:
                self._error = exc
        return guarded

    def raise_if_failed(self) -> None:
        if self._error is not None:
            raise RuntimeError(f"BrainCo replay failed: {type(self._error).__name__}: {self._error}")

    def open(self, reason: str, *, wait: bool = False, retries: int = 1,
             ignore_existing_error: bool = False) -> None:
        if self.open_positions is None:
            raise RuntimeError("BrainCo replay worker has no open reference")
        self.command(
            self.open_positions, reason, wait=wait, retries=retries,
            ignore_existing_error=ignore_existing_error,
        )

    def close(self) -> None:
        if self._thread is None:
            return
        final_error = None
        try:
            try:
                self.open(
                    "fail_safe_final_open", wait=True, retries=3,
                    ignore_existing_error=True,
                )
            except BaseException as exc:
                final_error = exc
        finally:
            self._commands.put(None)
            self._thread.join(self.timeout_seconds + 1.0)
            self._thread = None
        if final_error is not None:
            raise final_error
