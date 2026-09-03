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
    settle_tolerance: Optional[int] = None
    settle_timeout_seconds: float = 0.0
    error: Optional[BaseException] = None


class BrainCoHandReplay:
    """Own one serial hand and reopen it on every normal shutdown path."""

    VERIFIED_OPEN_POSITIONS = (1000, 510, 490, 490, 500, 500)

    def __init__(
        self, sdk, port: str, baud_enum, slave_id: int, event: EventCallback,
        *, timeout_seconds: float = 2.0,
        open_positions: Optional[Sequence[int]] = None,
        close_positions: Optional[Sequence[int]] = None,
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
        self._configured_open_positions = tuple(
            int(value) for value in (open_positions or self.VERIFIED_OPEN_POSITIONS)
        )
        self.close_positions = tuple(int(value) for value in (
            close_positions or (500, 500, 1000, 1000, 1000, 1000)
        ))
        for label, values in (("open", self._configured_open_positions),
                              ("close", self.close_positions)):
            if len(values) != 6 or any(value < 0 or value > 1000 for value in values):
                raise ValueError(f"BrainCo {label} pose must contain six values in [0, 1000]")

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
        self.command(
            self.open_positions, "initial_open", wait=True, retries=3,
            settle_tolerance=30, settle_timeout_seconds=8.0,
        )

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
            raise RuntimeError("BrainCo initial state is outside recoverable bounds")
        self.event("brainco_hand_initial_observed", {
            "positions": list(initial_positions),
        })
        self.open_positions = self._configured_open_positions
        self.event("brainco_hand_open_reference", {
            "positions": list(self.open_positions),
            "basis": "verified_robot_specific_normalized_pose",
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
                        finger_ids = (
                            self.sdk.FingerId.Thumb,
                            self.sdk.FingerId.ThumbAux,
                            self.sdk.FingerId.Index,
                            self.sdk.FingerId.Middle,
                            self.sdk.FingerId.Ring,
                            self.sdk.FingerId.Pinky,
                        )
                        transmitted_positions = [round(target / 10) for target in command.positions]
                        for finger_id, target in zip(finger_ids, transmitted_positions):
                            await asyncio.wait_for(
                                context.set_finger_position_with_millis(
                                    self.slave_id, finger_id, target, 500
                                ),
                                self.timeout_seconds,
                            )
                        status = await asyncio.wait_for(
                            context.get_motor_status(self.slave_id), self.timeout_seconds
                        )
                        measured = list(getattr(status, "positions", []))
                        if command.settle_tolerance is not None:
                            deadline = time.monotonic() + command.settle_timeout_seconds
                            while max(abs(a - b) for a, b in zip(
                                measured, command.positions
                            )) > command.settle_tolerance:
                                if time.monotonic() >= deadline:
                                    raise RuntimeError(
                                        "BrainCo hand did not settle to all six targets: "
                                        f"target={list(command.positions)}, measured={measured}"
                                    )
                                await asyncio.sleep(0.05)
                                status = await asyncio.wait_for(
                                    context.get_motor_status(self.slave_id),
                                    self.timeout_seconds,
                                )
                                measured = list(getattr(status, "positions", []))
                        self.event("brainco_hand_command", {
                            "positions": list(command.positions),
                            "per_finger_api_positions": transmitted_positions,
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
            close = getattr(self.sdk, "modbus_close", None)
            if callable(close):
                result = close(context)
                if asyncio.iscoroutine(result):
                    await result

    def command(
        self, positions: Sequence[int], reason: str, *, wait: bool = False,
        retries: int = 1, ignore_existing_error: bool = False,
        settle_tolerance: Optional[int] = None,
        settle_timeout_seconds: float = 0.0,
    ) -> None:
        if not ignore_existing_error:
            self.raise_if_failed()
        values = tuple(int(value) for value in positions)
        if len(values) != 6 or any(value < 0 or value > 1000 for value in values):
            raise ValueError("reviewed BrainCo command must contain six normalized positions in [0, 1000]")
        if not 1 <= retries <= 3:
            raise ValueError("BrainCo command retries must be between one and three")
        if settle_tolerance is not None and not 0 <= settle_tolerance <= 100:
            raise ValueError("BrainCo settle tolerance must be in [0, 100]")
        if settle_tolerance is not None and settle_timeout_seconds <= 0:
            raise ValueError("BrainCo settle timeout must be positive")
        command = _Command(
            values, reason, threading.Event(), retries=retries,
            settle_tolerance=settle_tolerance,
            settle_timeout_seconds=float(settle_timeout_seconds),
        )
        self._commands.put(command)
        if wait and not command.completed.wait(
            self.timeout_seconds * 2.0 * retries + 0.5 * (retries - 1)
            + command.settle_timeout_seconds
        ):
            raise RuntimeError("BrainCo hand command completion timed out")
        if command.error is not None:
            raise RuntimeError(
                f"BrainCo hand command failed: {type(command.error).__name__}: {command.error}"
            )
        if not ignore_existing_error:
            self.raise_if_failed()

    def start_close_ramp(self, *, steps: int = 10, period_seconds: float = 0.2) -> None:
        if self.open_positions is None:
            raise RuntimeError("BrainCo replay worker has no open reference")
        baseline = self.open_positions

        def schedule():
            for index in range(1, steps + 1):
                fraction = index / steps
                target = tuple(round(
                    start + fraction * (end - start)
                ) for start, end in zip(baseline, self.close_positions))
                self.command(
                    target,
                    "source_close_ramp",
                    wait=index == steps,
                    settle_tolerance=30 if index == steps else None,
                    settle_timeout_seconds=5.0 if index == steps else 0.0,
                )
                if index < steps:
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

    def wait_schedulers(self, timeout_seconds: float = 10.0) -> None:
        deadline = time.monotonic() + timeout_seconds
        for thread in self._schedulers:
            thread.join(max(0.0, deadline - time.monotonic()))
        if any(thread.is_alive() for thread in self._schedulers):
            raise RuntimeError("BrainCo scheduled command did not finish")
        self.raise_if_failed()

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
