#!/usr/bin/env python3
# coding: utf-8
"""smbus shim cho laptop (x86, không có I2C speech module trực tiếp).

dofbot_voice/scripts/*.py và notebook 02/03 dùng `import smbus` với
`smbus.SMBus(n)` để nói chuyện I2C addr 0x0f với module speech.
Trên laptop này `pip install smbus2` cung cấp module `smbus2`, không phải
`smbus`, nên import gốc luôn fail. Shim này:
  - nếu có smbus2 thật -> wrap nó (dùng bus I2C laptop nếu cần),
  - nếu không -> MOCK no-op để file runnable, các op I2C chỉ in cảnh báo.
"""
try:
    from smbus2 import SMBus as _RealSMBus  # type: ignore
    _HAS_REAL = True
except Exception:
    _RealSMBus = None
    _HAS_REAL = False


class SMBus:
    def __init__(self, bus=1):
        self.bus_no = bus
        self._real = None
        if _HAS_REAL:
            try:
                self._real = _RealSMBus(bus)
            except Exception as e:
                print(f"[smbus shim] real SMBus({bus}) unavailable: {e}, using MOCK")
                self._real = None
        else:
            print("[smbus shim] MOCK mode (smbus2 not installed)")

    def write_byte(self, addr, val):
        if self._real is not None:
            return self._real.write_byte(addr, val)
        return None

    def read_byte(self, addr):
        if self._real is not None:
            return self._real.read_byte(addr)
        return 0

    def write_i2c_block_data(self, addr, cmd, vals):
        if self._real is not None:
            return self._real.write_i2c_block_data(addr, cmd, vals)
        return None

    def read_i2c_block_data(self, addr, cmd, length=1):
        if self._real is not None:
            return self._real.read_i2c_block_data(addr, cmd, length)
        return [0] * length

    def write_byte_data(self, addr, cmd, val):
        if self._real is not None:
            return self._real.write_byte_data(addr, cmd, val)
        return None

    def read_byte_data(self, addr, cmd):
        if self._real is not None:
            return self._real.read_byte_data(addr, cmd)
        return 0

    def close(self):
        try:
            if self._real is not None:
                self._real.close()
        except Exception:
            pass
