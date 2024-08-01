from typing import Optional, Tuple, ByteString
import struct
import RPi.GPIO as GPIO
import math as m
from enum import Enum

class Logger:
    '''Logger with verbosity control'''
    class Verbosity(Enum):
        MIN = 1
        MED = 2
        MAX = 3

    def __init__(self, prefix: str, verbosity: "Logger.Verbosity" = Verbosity.MAX) -> None:
        '''Verbosity controls amount of messages to be printed. Higher verbosity => more messages.'''
        self._verbosity = verbosity
        self._prefix = prefix
    
    @property
    def verbosity(self) -> "Logger.Verbosity":
        """Return current verbosity"""
        return self._verbosity

    @verbosity.setter
    def verbosity(self, verbosity: "Logger.Verbosity") -> None:
        '''Changes logger min verbosity level'''
        self._verbosity = verbosity

    def info(self, *args, **kwargs) -> None:
        '''Logging an info message. Minumum message verbosity => message is always printed.
        Default message verbosity is MAX to limit amount of messages.'''
        message_verbosity = kwargs.pop('verbosity', Logger.Verbosity.MAX)
        if message_verbosity.value <= self._verbosity.value:
            if kwargs.pop('no_prefix', False):
                print(*args, **kwargs)
            else:
                print(f"{self._prefix} info: ", *args, **kwargs)

    def warning(self, *args, **kwargs) -> None:
        '''Logging a warning message'''
        raise NotImplementedError("Please Implement this method")
    
    def fatal(self, *args, **kwargs) -> None:
        '''Logging a fatal message'''
        raise NotImplementedError("Please Implement this method")


class OutPinWrapper:
    '''Output GPIO pin'''
    def __init__(self, pin_id, value=0):
        self._pin_id = pin_id
        self._mode = GPIO.OUT
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


class I2cDriver:
    '''I2C interace high-level driver'''

    def __init__(self, pi, i2c_device, logger: Logger):
        self._pi = pi
        self._i2c_device = i2c_device
        self._logger = logger

    def write(
        self, addr: int, data: Optional[ByteString] = None
    ) -> None:
        '''Writes data to addr using I2C interface'''
        self._logger.info(f"wr: addr={addr:x}, data={data}")
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
        self._logger.info(f"rd: addr={addr:x},", end="")
        data = None
        if count == 0:
            raise RuntimeError("Zero byte reading is not implemented")
        elif count == 1:
            
            # for some reason single transaction read doesn't work properly
            # might be speed issue
            # data = self._pi.i2c_read_byte_data(self._i2c_device, addr)

            # so have to use 2-transaction form
            self._pi.i2c_write_device(self._i2c_device, struct.pack(">B", addr))
            data = self._pi.i2c_read_byte(self._i2c_device)
            self._logger.info(f"data={data:x}", no_prefix=True)
        else:
            (bytes, data) = self._pi.i2c_read_i2c_block_data(self._i2c_device, addr, count)
            if bytes < 0:
                raise RuntimeError("Got error while reading")
            self._logger.info(f"data={[hex(i) for i in data] if data else None}", no_prefix=True)
        return data


class CO2Meter:
    '''CO2 Meter class that controls device via I2C interface'''
    SW_RESET_ADDR = 0xFF
    SW_RESET_VAL = b"\x11\xE5\x72\x8A"

    HW_ID_ADDR = 0x20
    HW_ID_VAL = 0x81

    STATUS_ADDR = 0x0
    STATUS_REG_FW_MODE_FIELD = (7,7)
    STATUS_REG_APP_VALID_FIELD = (4,4)
    STATUS_REG_DATA_READY_FIELD = (3,3)
    STATUS_REG_ERROR_FIELD = (0,0)

    APP_START_ADDR = 0xF4

    MEAS_MODE_ADDR = 0x1
    MEAS_MODE_REG_DRIVE_MODE_FIELD = (6,4)
    MEAS_MODE_REG_INT_DATARDY_FIELD = (3,3)
    MEAS_MODE_REG_INT_THRESH_FIELD = (2,2)
    DRIVE_MODE_1SEC = 0x01

    ALG_RESULT_DATA_ADDR = 0x2

    ERROR_ID_ADDR = 0xE0
    ERROR_ID_REG_WRITE_REG_INVALID_FIELD = (0,0)
    ERROR_ID_REG_READ_REG_INVALID_FIELD = (1,1)
    ERROR_ID_REG_MEASMODE_INVALID_FIELD = (2,2)
    ERROR_ID_REG_MAX_RESISTANCE_FIELD = (3,3)
    ERROR_ID_REG_HEATER_FAULT_FIELD = (4,4)
    ERROR_ID_REG_HEATER_SUPPLY_FIELD = (5,5)

    def __init__(self, i2c, logger: Logger) -> None:
        self._i2c = i2c
        self._logger = logger
        self.reset()

    def reset(self) -> None:
        '''SW reset'''
        self._logger.info(f"co2: SW RST")
        self._i2c.write(self.SW_RESET_ADDR, self.SW_RESET_VAL)
        time.sleep(0.5)

    def check_devid(self) -> bool:
        res = self._i2c.read(self.HW_ID_ADDR)
        self._logger.info(f"co2: Expected HW_ID={self.HW_ID_VAL}, received HW_ID={res}")
        return res == self.HW_ID_VAL

    def read_status(self) -> int:
        '''Returns status register value'''
        res = self._i2c.read(self.STATUS_ADDR)
        fw_mode = CO2Meter.get_field(res, self.STATUS_REG_FW_MODE_FIELD)
        app_valid = CO2Meter.get_field(res, self.STATUS_REG_APP_VALID_FIELD)
        data_ready = CO2Meter.get_field(res, self.STATUS_REG_DATA_READY_FIELD)
        error = CO2Meter.get_field(res, self.STATUS_REG_ERROR_FIELD)
        self._logger.info(f"co2: FW_MODE={fw_mode:x}, APP_VALID={app_valid:x}, DATA_READY={data_ready:x}, ERROR={error:x}")
        return res

    def check_errid(self) -> bool:
        res = self._i2c.read(self.ERROR_ID_ADDR)
        err_fields = [
            ("write_reg_invalid", CO2Meter.get_field(res, self.ERROR_ID_REG_WRITE_REG_INVALID_FIELD)),
            ("read_reg_invalid", CO2Meter.get_field(res, self.ERROR_ID_REG_READ_REG_INVALID_FIELD)),
            ("measmode_invalid", CO2Meter.get_field(res, self.ERROR_ID_REG_MEASMODE_INVALID_FIELD)),
            ("max_resistance", CO2Meter.get_field(res, self.ERROR_ID_REG_MAX_RESISTANCE_FIELD)),
            ("heater_fault", CO2Meter.get_field(res, self.ERROR_ID_REG_HEATER_FAULT_FIELD)),
            ("heater_supply", CO2Meter.get_field(res, self.ERROR_ID_REG_HEATER_SUPPLY_FIELD)),
        ]
        self._logger.info("co2:", end="")
        for name, val in err_fields:
            self._logger.info(f" {name}={val};", no_prefix=True, end="")
        self._logger.info()
        return res == 0

    def set_app_mode(self):
        '''Writes application mode'''
        self._i2c.write(self.APP_START_ADDR)
        time.sleep(0.1)

    def setmeas_mode(self, mode: int):
        '''Writes measurement mode to register'''
        meas_mode_regval = self._i2c.read(self.MEAS_MODE_ADDR)
        meas_mode_regval = CO2Meter.set_field(meas_mode_regval, self.MEAS_MODE_REG_DRIVE_MODE_FIELD, mode)
        self._logger.info(f"co2: MEAS_MODE={mode:x}")
        # send byte to reg MEAS_MODE_ADDR
        self._i2c.write(self.MEAS_MODE_ADDR, struct.pack(">B", meas_mode_regval))
        time.sleep(0.5)

    def read_meas(self):
        data = self._i2c.read(self.ALG_RESULT_DATA_ADDR, 4)
        eco2 = data[0] << 8 | data[1]
        tvoc = data[2] << 8 | data[3]
        return eco2, tvoc

    @staticmethod
    def get_field(regval:int, field: Tuple[int, int]) -> int:
        ''' Returns register field value'''
        mask = ((0x1 << (1 + field[0] - field[1])) - 1) << field[1]
        return (regval & mask) >> field[1]

    @staticmethod
    def set_field(regval:int, field: Tuple[int, int], fieldval:int) -> int:
        ''' Sets field value and return register value'''
        width = 1 + field[0] - field[1]
        if fieldval: assert((int(m.log2(fieldval)) + 1 <= width))
        mask = ((0x1 << width) - 1) << field[1]
        regval |= mask # set all ones in field
        regval &= (fieldval << field[1]) & mask # copy zeros from fieldval
        return regval



if __name__ == "__main__":
    import time
    import pigpio
    
    GPIO.setmode(GPIO.BCM)
    status_led = OutPinWrapper(26)

    i2c_device = None

    try: 
        status_led.value = 1
        pi = pigpio.pi()
        i2c_device = pi.i2c_open(1, 0x5a)
        i2c = I2cDriver(pi, i2c_device, logger=Logger("i2c"))

        meter = CO2Meter(i2c, logger=Logger("co2"))

        meter.check_devid()
        meter.read_status()
        meter.check_errid()
        meter.set_app_mode()
        
        status = meter.read_status()

        fw_mode = meter.get_field(status, meter.STATUS_REG_FW_MODE_FIELD)
        if not fw_mode:
            raise RuntimeError("wrong fw_mode:", fw_mode)
        if meter.get_field(status, meter.STATUS_REG_ERROR_FIELD):
            meter.check_errid()
            raise RuntimeError("got err bit")        

        meter.setmeas_mode(meter.DRIVE_MODE_1SEC)

        while True:
            time.sleep(1)
            status = meter.read_status()
            if meter.get_field(status, meter.STATUS_REG_ERROR_FIELD):
                meter.check_errid()
                raise RuntimeError("got err bit")
            if meter.get_field(status, meter.STATUS_REG_DATA_READY_FIELD):
                eco2, tvoc = meter.read_meas()
                print(f"eco2={eco2}, tvoc={tvoc}")

    finally:
        print("Releasing resources...")
        status_led.value = 0
        if i2c_device is not None:
            pi.i2c_close(i2c_device)
            print("i2c bus closed...")
        GPIO.cleanup()

