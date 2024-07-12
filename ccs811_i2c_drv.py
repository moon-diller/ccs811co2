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


        # sw reset
        SW_RESET_ADDR = 0xFF
        pi.i2c_write_i2c_block_data(i2c_device, SW_RESET_ADDR, b"\x11\xE5\x72\x8A")
        time.sleep(0.5)

        # start
        HW_ID_ADDR = 0x20
        HW_ID_VAL = 0x81
        res = pi.i2c_read_byte_data(i2c_device, HW_ID_ADDR)
        print(f"Expected HW_ID={HW_ID_VAL:x}, received HW_ID={res:x}")

        STATUS_ADDR = 0x0
        res = pi.i2c_read_byte_data(i2c_device, STATUS_ADDR)
        print(f"FW_MODE={(res >> 7) & 0x1:x}, APP_VALID={(res >> 4) & 0x1:x}, DATA_READY={(res >> 3) & 0x1:x}, ERROR={res & 0x1:x}")

        APP_START_ADDR = 0xF4
        # pi.i2c_write_block_data(i2c_device, APP_START_ADDR, b"")
        pi.i2c_write_device(i2c_device, b"\xf4")

        STATUS_ADDR = 0x0
        res = pi.i2c_read_byte_data(i2c_device, STATUS_ADDR)
        print(f"FW_MODE={(res >> 7) & 0x1:x}, APP_VALID={(res >> 4) & 0x1:x}, DATA_READY={(res >> 3) & 0x1:x}, ERROR={res & 0x1:x}")
        fw_mode = (res >> 7) & 0x1
        if not fw_mode:
            raise RuntimeError("wrong fw_mode:", fw_mode)

        print(f"MEAS_MODE={0x01 << 4:x}")
        MEAS_MODE_ADDR = 0x1
        # send byte to reg MEAS_MODE_ADDR
        pi.i2c_write_byte_data(i2c_device, MEAS_MODE_ADDR, 0x01 << 4)


        ALG_RESULT_DATA_ADDR = 0x2
        while True:

            time.sleep(0.5)
            res = pi.i2c_read_byte_data(i2c_device, STATUS_ADDR)
            if (res >> 3) & 0x1:
                print("Data is ready!")
                (b, d) = pi.i2c_read_i2c_block_data(i2c_device, ALG_RESULT_DATA_ADDR, 4)
                if b >= 0:
                    # process data
                    eco2 = d[0] << 8 | d[1]
                    tvoc = d[2] << 8 | d[3]
                    print(f"eco2={eco2}, tvoc={tvoc}")
                else:
                    # process read failure
                    print("Unsuccessfull reading...")

    finally:
        print("Releasing resources...")
        status_led.value = 0
        if i2c_device is not None:
            pi.i2c_close(i2c_device)
            print("i2c bus closed...")
        GPIO.cleanup()

