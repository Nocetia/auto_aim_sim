"""Strict read-only consumer for the rm_aim_sim raw-frame mailbox."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import stat
import sys
import time
from typing import Any


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from tools.local_ipc import LocalClient  # noqa: E402


PROTOCOL = "rm_aim_sim/2"
MAXIMUM_JSON_BYTES = 65536
MINIMUM_FRAME_WIDTH = 160
MAXIMUM_FRAME_WIDTH = 3840
MINIMUM_FRAME_HEIGHT = 120
MAXIMUM_FRAME_HEIGHT = 2160


class FrameProtocolError(RuntimeError):
    """A matching frame was malformed, stale, incomplete, or unsafe."""


class ContextChangedError(FrameProtocolError):
    """The publisher/session/static capture context changed mid-request."""


class ContinuityError(RuntimeError):
    """A stream boundary requires the current episode to stop."""


def _finite_number(value: Any, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise FrameProtocolError(f"{field} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise FrameProtocolError(f"{field} must be finite")
    return result


def _bounded_integer(value: Any, minimum: int, maximum: int, field: str) -> int:
    numeric = _finite_number(value, field)
    if numeric != math.floor(numeric) or not minimum <= numeric <= maximum:
        raise FrameProtocolError(f"{field} must be an integer in {minimum}..{maximum}")
    return int(numeric)


def _numeric_tuple(value: Any, length: int, field: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != length:
        raise FrameProtocolError(f"{field} must contain exactly {length} numbers")
    return tuple(_finite_number(item, f"{field}[{index}]") for index, item in enumerate(value))


def frame_context(packet: dict[str, Any]) -> tuple[Any, ...]:
    """Return the same static observation context used by auto_aim_bridge."""

    try:
        camera = packet["camera"]
        publisher = packet["publisher_id"]
        session = packet["session_id"]
        paused = packet["paused"]
        enabled = packet["ipc"]["enabled"]
        if not isinstance(publisher, str) or not publisher or not isinstance(session, str) or not session:
            raise FrameProtocolError("publisher_id/session_id must be non-empty strings")
        if not isinstance(paused, bool) or not isinstance(enabled, bool):
            raise FrameProtocolError("paused/ipc.enabled must be booleans")
        return (
            publisher,
            session,
            json.dumps(
                packet["appearance"], sort_keys=True, separators=(",", ":"), allow_nan=False
            ),
            paused,
            enabled,
            json.dumps(packet["weapon"], sort_keys=True, separators=(",", ":"), allow_nan=False),
            _numeric_tuple(packet["gimbal"]["origin"], 3, "gimbal.origin"),
            _finite_number(camera["width"], "camera.width"),
            _finite_number(camera["height"], "camera.height"),
            _finite_number(camera["brightness_gain"], "camera.brightness_gain"),
            _numeric_tuple(camera["K"], 9, "camera.K"),
            _numeric_tuple(camera["distortion"], 5, "camera.distortion"),
            _numeric_tuple(camera["R_camera2gimbal"], 9, "camera.R_camera2gimbal"),
            _numeric_tuple(camera["t_camera2gimbal"], 3, "camera.t_camera2gimbal"),
        )
    except FrameProtocolError:
        raise
    except (KeyError, TypeError, ValueError) as error:
        raise FrameProtocolError(f"incomplete frame context: {error}") from error


def validate_frame_metadata(
    frame: dict[str, Any],
    request_id: str,
    requested_context: tuple[Any, ...],
    latest_state: dict[str, Any],
) -> tuple[int, int, int, str, int]:
    """Validate a matching frame and return width/height/channels/encoding/bytes."""

    if frame.get("protocol") != PROTOCOL or frame.get("type") != "frame":
        raise FrameProtocolError("matching packet is not an rm_aim_sim/2 frame")
    if frame.get("request_id") != request_id:
        raise FrameProtocolError("frame request_id does not match the pending request")
    latest_context = frame_context(latest_state)
    if frame_context(frame) != requested_context or latest_context != requested_context:
        raise ContextChangedError("publisher/session/static frame context changed")

    frame_sim_time = _finite_number(frame.get("sim_time"), "frame.sim_time")
    state_sim_time = _finite_number(latest_state.get("sim_time"), "state.sim_time")
    frame_age = state_sim_time - frame_sim_time
    if frame_sim_time < 0 or frame_age < -0.1 or frame_age > 0.2:
        raise FrameProtocolError("frame sim_time is stale or ahead of current state")
    tick = _bounded_integer(frame.get("tick"), 0, 2**53 - 1, "frame.tick")

    camera = frame.get("camera")
    if not isinstance(camera, dict):
        raise FrameProtocolError("frame.camera must be an object")
    camera_time = _finite_number(camera.get("timestamp"), "frame.camera.timestamp")
    camera_tick = _bounded_integer(camera.get("tick"), 0, 2**53 - 1, "frame.camera.tick")
    if abs(camera_time - frame_sim_time) > 1e-6 or camera_tick != tick:
        raise FrameProtocolError("camera metadata is not from the frame snapshot")
    width = _bounded_integer(
        camera.get("width"), MINIMUM_FRAME_WIDTH, MAXIMUM_FRAME_WIDTH, "frame.camera.width"
    )
    height = _bounded_integer(
        camera.get("height"), MINIMUM_FRAME_HEIGHT, MAXIMUM_FRAME_HEIGHT, "frame.camera.height"
    )
    encoding = frame.get("encoding")
    channels = {"rgb8": 3, "rgba8": 4}.get(encoding)
    if channels is None:
        raise FrameProtocolError("frame.encoding must be rgb8 or rgba8")
    expected_name = f"frame-{request_id}.raw"
    if frame.get("frame_file") != expected_name:
        raise FrameProtocolError("frame_file does not match request_id")
    expected_bytes = width * height * channels
    image_bytes = _bounded_integer(
        frame.get("image_bytes"), 0, MAXIMUM_FRAME_WIDTH * MAXIMUM_FRAME_HEIGHT * 4, "frame.image_bytes"
    )
    if image_bytes != expected_bytes:
        raise FrameProtocolError("image_bytes does not match dimensions and encoding")
    return width, height, channels, encoding, expected_bytes


class StrictFrameClient(LocalClient):
    """LocalClient extension with bridge-equivalent raw lifecycle checks.

    It deliberately has no command API usage and closes with send_stop=False.
    """

    def __init__(self, channel: str = "default", timeout: float = 8.0):
        super().__init__(channel, timeout=timeout)
        self._directory_fd = -1
        try:
            flags = os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC
            if hasattr(os, "O_NOFOLLOW"):
                flags |= os.O_NOFOLLOW
            self._directory_fd = os.open(self.directory, flags)
            info = os.fstat(self._directory_fd)
            if info.st_uid != os.getuid() or stat.S_IMODE(info.st_mode) != 0o700:
                raise PermissionError("IPC directory must belong to this UID with exact mode 0700")
        except Exception:
            super().close(send_stop=False)
            raise
        self._pending_request: str | None = None
        self._pending_context: tuple[Any, ...] | None = None
        self._state_received_at = 0.0
        self._request_counter = 0
        self._used_request_ids: set[str] = set()

    def _read_json_secure(self, name: str) -> dict[str, Any]:
        flags = os.O_RDONLY | os.O_CLOEXEC
        if hasattr(os, "O_NOFOLLOW"):
            flags |= os.O_NOFOLLOW
        try:
            descriptor = os.open(name, flags, dir_fd=self._directory_fd)
        except FileNotFoundError:
            return {}
        try:
            info = os.fstat(descriptor)
            if not stat.S_ISREG(info.st_mode) or not 0 <= info.st_size <= MAXIMUM_JSON_BYTES:
                raise FrameProtocolError(f"unsafe or oversized IPC JSON file: {name}")
            chunks: list[bytes] = []
            remaining = info.st_size
            while remaining:
                chunk = os.read(descriptor, remaining)
                if not chunk:
                    raise FrameProtocolError(f"incomplete IPC JSON read: {name}")
                chunks.append(chunk)
                remaining -= len(chunk)
        finally:
            os.close(descriptor)
        try:
            value = json.loads(b"".join(chunks))
        except (UnicodeDecodeError, json.JSONDecodeError):
            return {}
        return value if isinstance(value, dict) else {}

    def send(self, *args: Any, **kwargs: Any) -> None:
        raise RuntimeError("StrictFrameClient is read-only and cannot publish command.json")

    def _write_request(self, value: dict[str, str]) -> None:
        serialized = json.dumps(value, allow_nan=False, separators=(",", ":")).encode("utf-8")
        temporary = f"request.json.{self.token}.tmp"
        flags = os.O_CREAT | os.O_TRUNC | os.O_WRONLY | os.O_CLOEXEC
        if hasattr(os, "O_NOFOLLOW"):
            flags |= os.O_NOFOLLOW
        descriptor = os.open(temporary, flags, 0o600, dir_fd=self._directory_fd)
        try:
            offset = 0
            while offset < len(serialized):
                written = os.write(descriptor, serialized[offset:])
                if written <= 0:
                    raise OSError("incomplete request.json write")
                offset += written
        finally:
            os.close(descriptor)
        os.replace(
            temporary,
            "request.json",
            src_dir_fd=self._directory_fd,
            dst_dir_fd=self._directory_fd,
        )

    @staticmethod
    def _valid_state_envelope(value: dict[str, Any]) -> bool:
        try:
            publication = value["publication"]
            wall = value["wall_unix_sec"]
            return (
                value["protocol"] == PROTOCOL
                and value["type"] == "state"
                and isinstance(value["publisher_id"], str)
                and bool(value["publisher_id"])
                and isinstance(value["session_id"], str)
                and bool(value["session_id"])
                and isinstance(publication, int)
                and not isinstance(publication, bool)
                and publication >= 0
                and isinstance(wall, (int, float))
                and not isinstance(wall, bool)
                and math.isfinite(float(wall))
                and isinstance(value["paused"], bool)
                and isinstance(value["ipc"]["enabled"], bool)
            )
        except (KeyError, TypeError):
            return False

    def _accept_new_state(self, value: dict[str, Any], *, require_fresh_wall: bool) -> bool:
        if not self._valid_state_envelope(value):
            return False
        identity = (value["publisher_id"], value["publication"])
        if identity == self.last_publication:
            return False
        if require_fresh_wall:
            age = time.time() - float(value["wall_unix_sec"])
            if not math.isfinite(age) or age < -0.1 or age > 0.25:
                return False
        self.state = value
        self.last_publication = identity
        self._state_received_at = time.monotonic()
        return True

    def read_state(self, timeout: float = 3.0) -> dict[str, Any]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            value = self._read_json_secure("state.json")
            if self._accept_new_state(value, require_fresh_wall=True):
                return value
            time.sleep(0.002)
        raise TimeoutError("Waiting for fresh local simulator state")

    def _poll_state(self) -> None:
        value = self._read_json_secure("state.json")
        self._accept_new_state(value, require_fresh_wall=True)

    def request_frame(self) -> str:
        if self._pending_request is not None:
            raise RuntimeError("only one raw frame request may be pending")
        if self.state is None or time.monotonic() - self._state_received_at > 0.25:
            raise RuntimeError("read a fresh simulator state before requesting a frame")
        context = frame_context(self.state)
        request_id = f"{self.token}-{self._request_counter}"
        self._request_counter += 1
        if request_id in self._used_request_ids or len(request_id) > 80:
            raise RuntimeError("request_id uniqueness/length invariant failed")
        self._used_request_ids.add(request_id)
        self._pending_request = request_id
        self._pending_context = context
        try:
            self._write_request({"publisher_id": self.state["publisher_id"], "request_id": request_id})
        except Exception:
            self._pending_request = None
            self._pending_context = None
            raise
        return request_id

    def receive_frame(self, identity: str, timeout: float = 5.0) -> tuple[dict[str, Any], bytes]:
        if identity != self._pending_request or self._pending_context is None:
            raise RuntimeError("receive_frame must match the single pending request")
        deadline = time.monotonic() + timeout
        try:
            while time.monotonic() < deadline:
                self._poll_state()
                if self.state is None or frame_context(self.state) != self._pending_context:
                    raise ContextChangedError("frame context changed while request was pending")
                frame = self._read_json_secure("frame.json")
                if frame.get("request_id") != identity:
                    time.sleep(0.002)
                    continue
                _, _, _, _, expected_bytes = validate_frame_metadata(
                    frame, identity, self._pending_context, self.state
                )
                flags = os.O_RDONLY | os.O_CLOEXEC
                if hasattr(os, "O_NOFOLLOW"):
                    flags |= os.O_NOFOLLOW
                filename = f"frame-{identity}.raw"
                try:
                    descriptor = os.open(filename, flags, dir_fd=self._directory_fd)
                except FileNotFoundError as error:
                    raise FrameProtocolError("matching raw frame disappeared") from error
                try:
                    info = os.fstat(descriptor)
                    if not stat.S_ISREG(info.st_mode) or info.st_size != expected_bytes:
                        raise FrameProtocolError("raw file type/size does not match metadata")
                    data = bytearray(expected_bytes)
                    view = memoryview(data)
                    offset = 0
                    while offset < expected_bytes:
                        chunk = os.read(descriptor, expected_bytes - offset)
                        if not chunk:
                            raise FrameProtocolError("incomplete raw frame read")
                        view[offset : offset + len(chunk)] = chunk
                        offset += len(chunk)
                finally:
                    os.close(descriptor)
                return frame, bytes(data)
            raise TimeoutError("Waiting for matching raw frame; use graphical mode, not --headless")
        finally:
            # A failed request ID must never be retried: camera_stream marks it
            # consumed before capture and only accepts a new unique ID.
            self._pending_request = None
            self._pending_context = None

    def close(self, send_stop: bool = False) -> None:
        if self._directory_fd >= 0:
            os.close(self._directory_fd)
            self._directory_fd = -1
        super().close(send_stop=False)


class StreamContinuity:
    """Reject duplicate/out-of-order identifiers and explicit episode boundaries."""

    def __init__(self) -> None:
        self.publisher_id: str | None = None
        self.session_id: str | None = None
        self.context: tuple[Any, ...] | None = None
        self.last_tick: int | None = None
        self.last_sim_time: float | None = None
        self.requests: set[tuple[str, str]] = set()
        self.saved_image_digests: set[str] = set()

    def observe(self, frame: dict[str, Any]) -> str | None:
        publisher = frame.get("publisher_id")
        session = frame.get("session_id")
        request = frame.get("request_id")
        if not all(isinstance(value, str) and value for value in (publisher, session, request)):
            raise ContinuityError("frame identity is incomplete")
        context = frame_context(frame)
        tick = _bounded_integer(frame.get("tick"), 0, 2**53 - 1, "frame.tick")
        sim_time = _finite_number(frame.get("sim_time"), "frame.sim_time")
        if self.publisher_id is None:
            self.publisher_id, self.session_id, self.context = publisher, session, context
        elif publisher != self.publisher_id:
            raise ContinuityError("publisher changed; stop this episode and reconnect")
        elif session != self.session_id:
            raise ContinuityError("session changed; stop this episode and start a new one")
        elif context != self.context:
            raise ContinuityError("static capture context changed; stop this episode")

        request_key = (publisher, request)
        if request_key in self.requests:
            return "duplicate_request"
        self.requests.add(request_key)
        if self.last_tick is not None and tick < self.last_tick:
            raise ContinuityError("tick moved backwards")
        if self.last_sim_time is not None and sim_time < self.last_sim_time - 1e-9:
            raise ContinuityError("sim_time moved backwards")
        duplicate = None
        if tick == self.last_tick:
            duplicate = "duplicate_tick"
        elif self.last_sim_time is not None and sim_time <= self.last_sim_time + 1e-9:
            duplicate = "duplicate_sim_time"
        self.last_tick = tick
        self.last_sim_time = sim_time
        return duplicate

    def observe_saved_image(self, digest: str) -> bool:
        if digest in self.saved_image_digests:
            return False
        self.saved_image_digests.add(digest)
        return True
