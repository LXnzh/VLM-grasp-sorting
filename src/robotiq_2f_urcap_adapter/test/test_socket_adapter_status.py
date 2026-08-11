import pytest

from robotiq_2f_urcap_adapter_socket.robotiq_2f_socket_adapter import (
    Robotiq2fSocketAdapter,
)


def test_is_active_returns_false_when_status_is_unavailable(monkeypatch):
    adapter = Robotiq2fSocketAdapter()
    monkeypatch.setattr(adapter, "get_gripper_variable", lambda variable: -1)

    assert adapter.is_active is False
