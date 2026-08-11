"""Non-actuating tests for tracking, PBVS, and browser voice input."""

from concurrent.futures import ThreadPoolExecutor
import re
import socket
from types import SimpleNamespace
from urllib.request import Request, urlopen

import numpy as np
import pytest

from my_course_pkg.tasks.pbvs.control import (
    PBVSError,
    PositionWindowStabilityGate,
    bounded_camera_pbvs_target,
    masked_depth_centroid,
)
from my_course_pkg.tasks.tracking.motion_gate import (
    MOVING,
    STABILIZING,
    STABLE,
    MotionConfig,
    MotionStateMachine,
)
from my_course_pkg.tasks.voice_input.live_input import (
    record_browser_instruction,
    transcribe_audio_instruction,
)


def test_motion_gate_requires_continuous_stationary_window():
    gate = MotionStateMachine(
        MotionConfig(stable_duration_s=0.5, stable_frames=3)
    )

    assert gate.observe(1.0, moving=True) == MOVING
    assert gate.observe(1.1, moving=False) == STABILIZING
    assert gate.observe(1.3, moving=False) == STABILIZING
    assert gate.observe(1.6, moving=False) == STABLE
    assert gate.last_stop_stamp == 1.6


def test_position_gate_uses_fresh_samples_and_bounded_span():
    gate = PositionWindowStabilityGate(
        max_span_m=0.005,
        duration_s=0.5,
        minimum_samples=3,
    )

    assert gate.update(1.0, [0.0, 0.0, 0.0])[0] is False
    assert gate.update(1.0, [0.0, 0.0, 0.0])[0] is False
    assert gate.update(1.25, [0.001, 0.0, 0.0])[0] is False
    stable, span = gate.update(1.50, [0.002, 0.0, 0.0])

    assert stable is True
    assert span == pytest.approx(0.002)


def test_camera_pbvs_clamps_step_and_preserves_camera_error():
    command, error, should_command = bounded_camera_pbvs_target(
        [0.0, 0.0, 0.4],
        [0.10, 0.0, 0.6],
        [0.0, 0.0, 0.6],
        T_world_cv_camera=np.eye(4),
        T_workspace_world=np.eye(4),
        gain=1.0,
        deadband_m=0.001,
        max_step_m=0.02,
        workspace_min=[-1.0, -1.0, 0.0],
        workspace_max=[1.0, 1.0, 1.0],
        workspace_frame="world",
    )

    assert should_command is True
    np.testing.assert_allclose(error, [0.10, 0.0, 0.0])
    np.testing.assert_allclose(command, [0.02, 0.0, 0.4])


def test_camera_pbvs_fails_closed_outside_workspace():
    with pytest.raises(PBVSError, match="outside the configured workspace"):
        bounded_camera_pbvs_target(
            [0.99, 0.0, 0.4],
            [0.10, 0.0, 0.6],
            [0.0, 0.0, 0.6],
            T_world_cv_camera=np.eye(4),
            T_workspace_world=np.eye(4),
            gain=1.0,
            deadband_m=0.001,
            max_step_m=0.02,
            workspace_min=[-1.0, -1.0, 0.0],
            workspace_max=[1.0, 1.0, 1.0],
            workspace_frame="world",
        )


def test_masked_depth_centroid_uses_only_target_depth():
    depth = np.full((10, 12), 2.0, dtype=float)
    mask = np.zeros_like(depth, dtype=bool)
    mask[2:8, 3:9] = True
    depth[mask] = 1.0
    intrinsic = np.array(
        [[100.0, 0.0, 6.0], [0.0, 100.0, 5.0], [0.0, 0.0, 1.0]]
    )

    centroid = masked_depth_centroid(
        depth,
        mask,
        intrinsic,
        minimum_pixels=30,
    )

    np.testing.assert_allclose(centroid, [-0.005, -0.005, 1.0])


def _available_local_port():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        listener.bind(("127.0.0.1", 0))
        return int(listener.getsockname()[1])


def test_browser_microphone_server_accepts_one_audio_upload():
    port = _available_local_port()
    ready = []

    with ThreadPoolExecutor(max_workers=1) as executor:
        future = executor.submit(
            record_browser_instruction,
            1,
            port,
            5,
            ready.append,
            False,
        )
        for _ in range(100):
            if ready:
                break
            import time

            time.sleep(0.01)
        assert ready
        page = urlopen(ready[0], timeout=2).read().decode("utf-8")
        capture_path = re.search(r'fetch\("([^"]+)"', page).group(1)
        request = Request(
            ready[0] + capture_path,
            data=b"fake-webm-audio",
            headers={"Content-Type": "audio/webm"},
            method="POST",
        )
        assert urlopen(request, timeout=2).status == 204
        audio_path = future.result(timeout=3)

    try:
        assert audio_path.read_bytes() == b"fake-webm-audio"
    finally:
        audio_path.unlink(missing_ok=True)


def test_voice_transcription_uses_configured_model_without_key_in_argv(
    tmp_path,
    monkeypatch,
):
    audio_path = tmp_path / "voice.webm"
    audio_path.write_bytes(b"audio")
    captured = {}

    class FakeTranscriptions:
        def create(self, *, file, **kwargs):
            captured["payload"] = file.read()
            captured["kwargs"] = kwargs
            return SimpleNamespace(text="抓取苹果")

    class FakeOpenAI:
        def __init__(self, *, api_key, base_url):
            captured["api_key"] = api_key
            captured["base_url"] = base_url
            self.audio = SimpleNamespace(
                transcriptions=FakeTranscriptions()
            )

    monkeypatch.setenv("VOICE_API_KEY", "secret-test-key")
    monkeypatch.setenv("VOICE_BASE_URL", "https://voice.example/v1")
    monkeypatch.setattr("openai.OpenAI", FakeOpenAI)

    text = transcribe_audio_instruction(
        audio_path,
        model="kit.whisper-large-v3",
        language="zh",
    )

    assert text == "抓取苹果"
    assert captured["payload"] == b"audio"
    assert captured["kwargs"] == {
        "model": "kit.whisper-large-v3",
        "language": "zh",
    }
    assert captured["api_key"] == "secret-test-key"
