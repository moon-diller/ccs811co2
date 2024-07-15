from typing import Optional, Union, Tuple, List, Any, ByteString
import struct
import RPi.GPIO as GPIO

class CO2MeterWrapper:

    def __init__(self, i2c) -> None:
        self._i2c = i2c
        self.init()

    def init(self) -> None:
        """Run the initialization commands."""
        print("co2: initialized")


class PinWrapper:
    def __init__(self, pin_id, mode=GPIO.OUT, value=0):
        self._pin_id = pin_id
        self._mode = mode
        GPIO.setup(self._pin_id, self._mode)
        self._value = value
    
    @property
    def value(self) -> int:
        """Return current pin value"""
        return self._value

    @value.setter
    def value(self, val: int) -> None:
        self._value = val
        GPIO.output(self._pin_id, GPIO.HIGH if self._value else GPIO.LOW)

class I2CWrapper:

    # def __init__(self, i2c_bus, i2c_dev_addr):
    #     self._pi = pigpio.pi()
    #     self._i2c_device = pi.i2c_open(i2c_bus, i2c_dev_addr)

    def __init__(self, pi, i2c_device):
        self._pi = pi
        self._i2c_device = i2c_device

    # def __enter__(self):
    #     return self

    # def __exit__(self, exc_type, exc_value, traceback):
    #     self._pi.i2c_close(i2c_device_handler)

    def write(
        self, addr: int, data: ByteString | None = None
    ) -> None:
        '''Writes data to addr using I2C interface'''
        print(f"i2c wr: addr={addr:x}, data={data}")
        if data is None:
            self._pi.i2c_write_device(self._i2c_device, struct.pack(">B", addr))
        elif len(data) == 1:
            data_byte = struct.unpack(">B", data)[0]
            # works with int only
            self._pi.i2c_write_byte_data(self._i2c_device, addr, data_byte)
        else:
            self._pi.i2c_write_i2c_block_data(self._i2c_device, addr, data)

    
    def read(self, addr: int, count: int = 1) -> ByteString:
        '''Reads data from addr using I2C interface'''
        print(f"i2c rd: addr={addr:x},", end="")
        data = None
        if count == 0:
            raise RuntimeError("Zero byte reading is not implemented")
        elif count == 1:
            data = pi.i2c_read_byte_data(self._i2c_device, addr)
            print(f"data={data:x}")
        else:
            (bytes, data) = pi.i2c_read_i2c_block_data(self._i2c_device, addr, count)
            if bytes < 0:
                raise RuntimeError("Got error while reading")
            print(f"data={[hex(i) for i in data] if data else None}")
        return data


class CO2MeterWrapper:
    _SW_RESET_ADDR = 0xFF
    _SW_RESET_VAL = b"\x11\xE5\x72\x8A"

    _HW_ID_ADDR = 0x20
    _HW_ID_VAL = 0x81

    _STATUS_ADDR = 0x0

    _APP_START_ADDR = 0xF4

    _MEAS_MODE_ADDR = 0x1
    _MEAS_MODE_1SEC = 0x01 << 4

    _ALG_RESULT_DATA_ADDR = 0x2
    def __init__(self, i2c) -> None:
        self._i2c = i2c
        self.reset()

    def reset(self) -> None:
        '''SW reset'''
        print(f"co2: SW RST")
        self._i2c.write(self._SW_RESET_ADDR, self._SW_RESET_VAL)
        time.sleep(0.5)

    def check_devid(self) -> bool:
        res = self._i2c.read(self._HW_ID_ADDR)
        print(f"co2: Expected HW_ID={self._HW_ID_VAL}, received HW_ID={res}")
        return res == self._HW_ID_VAL

    def read_status(self) -> int:
        res = self._i2c.read(self._STATUS_ADDR)
        print(f"co2: FW_MODE={(res >> 7) & 0x1:x}, APP_VALID={(res >> 4) & 0x1:x}, DATA_READY={(res >> 3) & 0x1:x}, ERROR={res & 0x1:x}")
        return res

    def set_app_mode(self):
        self._i2c.write(self._APP_START_ADDR)

    def set_meas_mode(self, mode_byte: int):
        print(f"co2: MEAS_MODE={mode_byte:x}")
        # send byte to reg MEAS_MODE_ADDR
        self._i2c.write(self._MEAS_MODE_ADDR, struct.pack(">B", mode_byte))

    def read_meas(self):
        data = self._i2c.read(self._ALG_RESULT_DATA_ADDR, 4)
        eco2 = data[0] << 8 | data[1]
        tvoc = data[2] << 8 | data[3]
        return eco2, tvoc

if __name__ == "__main__":
    import time
    import pigpio
    
    GPIO.setmode(GPIO.BCM)
    status_led = PinWrapper(26)

    i2c_device = None

    try: 
        status_led.value = 1
        pi = pigpio.pi()
        i2c_device = pi.i2c_open(1, 0x5a)
        i2c = I2CWrapper(pi, i2c_device)

        meter = CO2MeterWrapper(i2c)

        meter.check_devid()
        meter.read_status()
        meter.set_app_mode()
        status = meter.read_status()
        fw_mode = (status >> 7) & 0x1
        if not fw_mode:
            raise RuntimeError("wrong fw_mode:", fw_mode)
        
        meter.set_meas_mode(0x01 << 4)

        while True:
            time.sleep(0.5)
            res = meter.read_status()
            if (res >> 3) & 0x1:
                print("Data is ready!")
                eco2, tvoc = meter.read_meas()
                print(f"eco2={eco2}, tvoc={tvoc}")

    finally:
        print("Releasing resources...")
        status_led.value = 0
        if i2c_device is not None:
            pi.i2c_close(i2c_device)
            print("i2c bus closed...")
        GPIO.cleanup()

