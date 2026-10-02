import ctypes
from types import SimpleNamespace

import pytest

from motorbridge.abi import Abi
from motorbridge.core import Motor
from motorbridge.errors import CallError


def make_motor(lib):
    motor = Motor.__new__(Motor)
    motor._ptr = 1
    motor._controller = SimpleNamespace(_ptr=1)
    motor._abi = SimpleNamespace(lib=lib)
    return motor


def test_feedback_sequence_preserves_the_full_counter():
    def read_sequence(_handle, out):
        ctypes.cast(out, ctypes.POINTER(ctypes.c_uint64))[0] = (1 << 40) + 7
        return 0

    motor = make_motor(SimpleNamespace(motor_handle_robstride_feedback_sequence=read_sequence))
    assert motor.robstride_feedback_sequence() == (1 << 40) + 7
    motor._ptr = None


def test_feedback_sequence_raises_on_native_error(monkeypatch):
    monkeypatch.setattr("motorbridge.core._err_text", lambda: "requires a RobStride motor")
    motor = make_motor(SimpleNamespace(motor_handle_robstride_feedback_sequence=lambda *_: -1))
    try:
        with pytest.raises(CallError, match="requires a RobStride motor"):
            motor.robstride_feedback_sequence()
    finally:
        motor._ptr = None


def test_feedback_sequence_raises_when_older_abi_has_no_counter():
    motor = make_motor(SimpleNamespace())
    try:
        with pytest.raises(CallError, match="requires a native library with feedback_sequence support"):
            motor.robstride_feedback_sequence()
    finally:
        motor._ptr = None


@pytest.mark.parametrize("has_sequence", [False, True])
def test_counter_binding_is_optional_and_uses_uint64(has_sequence):
    class Symbols:
        def __init__(self):
            self.symbols = {}

        def __getattr__(self, name):
            if name == "motor_handle_robstride_feedback_sequence" and not has_sequence:
                raise AttributeError(name)
            return self.symbols.setdefault(name, SimpleNamespace())

    abi = Abi.__new__(Abi)
    abi.lib = Symbols()
    abi._bind()
    assert abi.lib.motor_handle_get_state.restype is ctypes.c_int32
    if has_sequence:
        counter = abi.lib.motor_handle_robstride_feedback_sequence
        assert counter.argtypes == [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint64)]
        assert counter.restype is ctypes.c_int32
