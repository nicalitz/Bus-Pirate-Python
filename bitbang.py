import serial
import time
import event_log


def _verify_connection(method):
    def decorated(instance, *args, **kwargs):
        if instance.com.is_open:
            response = method(instance, *args, **kwargs)
            return response
        else:
            instance.log.entry("ERROR", "No serial device configured")
    return decorated


class BitbangRaw(object):
    def __init__(self, port, baudrate, bytesize, parity, stopbits, timeout, xonxoff):
        # Configure serial port
        self.com = serial.Serial()
        self.com.port = port
        self.com.baudrate = baudrate
        self.com.bytesize = bytesize
        self.com.parity = parity
        self.com.stopbits = stopbits
        self.com.timeout = timeout
        self.com.xonxoff = xonxoff

        self.log = event_log.EventLog(datetime_format='ISO 8601')

        self.protocol = 'RAW'
        self.protocol_code = None
        self.protocol_echo = None
        self.delay = 0.05

    def connect(self):
        try:
            self.com.open()
            self.log.entry("INFO", "Connected to device at {0}".format(self.com.port))
        except serial.SerialException as error:
            self.log.entry("ERROR", "{0}".format(error))

    def disconnect(self):
        try:
            self.com.close()
            self.log.entry("INFO", "Disconnected from device at {0}".format(self.com.port))
        except serial.SerialException:
            self.log.entry("ERROR", "Failed to disconnect from device")

    @_verify_connection
    def reset(self):
        try:
            self.mode_binary()
            self.com.write(data=bytes([0b00001111]))
            time.sleep(self.delay)
            response = self.com.read(1)
            if response == b'\x01':
                self.log.entry("INFO", "BusPirate: Terminal mode configured")
            else:
                self.log.entry("WARNING", "Device could not be reset (terminal mode)")
        except serial.SerialTimeoutException:
            self.log.entry("ERROR", "Write operation timed out")

    @_verify_connection
    def mode_binary(self):
        for count in range(25):
            try:
                self.com.write(bytes([0b00000000]))
                time.sleep(self.delay)
                response = self.com.read_all()
                if response == b'BBIO1':
                    self.log.entry("INFO", "BusPirate: Binary mode configured ({0})".format(str(count+1)))
                    return
            except serial.SerialTimeoutException:
                self.log.entry("ERROR", "Write operation timed out")
        self.log.entry("ERROR", "Connection timeout. Device could not be configured")

    @_verify_connection
    def mode_protocol(self):
        try:
            self.com.write(self.protocol_code)
            time.sleep(self.delay)
            response = self.com.read_all()
            if response == self.protocol_echo:
                self.log.entry("INFO", "BusPirate: Configured for {0} communication".format(self.protocol))
            else:
                self.log.entry("ERROR", "Failed to configure device")
        except KeyError:
            self.log.entry("ERROR", "Specified protocol not implemented")
        except serial.SerialTimeoutException:
            self.log.entry("ERROR", "Write operation timed out")


class BitbangI2C(BitbangRaw):
    def __init__(self, port, baudrate, bytesize=8, parity='N', stopbits=1, timeout=3, xonxoff=0):
        super().__init__(port, baudrate, bytesize, parity, stopbits, timeout, xonxoff)
        self.protocol = 'I2C'
        self.protocol_code = bytes([0b00000010])
        self.protocol_echo = b'I2C1'

    @_verify_connection
    def configure_protocol(self):
        self.mode_binary()
        self.mode_protocol()

    @_verify_connection
    def configure_speed(self, speed):
        valid_speeds = {'5': 0b00, '50': 0b01, '100': 0b10, '400': 0b11}
        try:
            self.com.write(data=bytes([0b01100000+valid_speeds[str(speed)]]))
            time.sleep(self.delay)
            response = self.com.read_all()
            if response == b'\x01':
                self.log.entry("INFO", "BusPirate: I2C speed set to {0}KHz".format(str(speed)))
            else:
                self.log.entry("WARNING", "No confirmation message received from device")
        except KeyError:
            self.log.entry("ERROR", "Invalid speed setting")
            self.log.entry("INFO", "Valid speed setting are: {0}".format(valid_speeds.keys()))
        except serial.SerialTimeoutException:
            self.log.entry("ERROR", "Write operation timed out")

    @_verify_connection
    def configure_peripherals(self, vreg=False, pullups=False, aux=False, cs=False):
        try:
            command = 0b01000000 + 0b1000*vreg + 0b0100*pullups + 0b0010*aux + 0b0001*cs
            self.com.write(data=bytes([command]))
            time.sleep(self.delay)
            response = self.com.read_all()
            if response == b'\x01':
                self.log.entry("INFO", "BusPirate: Peripherals configured")
                self.log.entry("INFO", "V-Reg: {0}, Pull-ups: {1}, Aux: {2}, CS: {3}".format(str(vreg), str(pullups),
                                                                                             str(aux), str(cs)))
            else:
                self.log.entry("WARNING", "No confirmation message received from device")
        except ValueError:
            self.log.entry("ERROR", "Peripherals configuration should be boolean (ValueError)")
        except serial.SerialTimeoutException:
            self.log.entry("ERROR", "Write operation timed out")

    @_verify_connection
    def start_bit(self):
        self.com.write(data=bytes([0b00000010]))
        response = self.com.read(1)  # responds with 0x01 if successful

    @_verify_connection
    def stop_bit(self):
        self.com.write(data=bytes([0b00000011]))
        response = self.com.read(1)  # responds with 0x01 if successful

    @_verify_connection
    def ack_bit(self):
        self.com.write(data=bytes([0b00000110]))
        response = self.com.read(1)  # responds with 0x01 if successful

    @_verify_connection
    def nack_bit(self):
        self.com.write(data=bytes([0b00000111]))
        response = self.com.read(1)  # responds with 0x01 if successful

    @_verify_connection
    def read_byte(self):
        self.com.write(data=bytes([0b00000100]))
        response = self.com.read(1)
        return response

    @_verify_connection
    def write_bytes(self, data):
        try:
            if 1 <= len(data) <= 16:
                self.com.write(data=bytes([0b00010000 + (len(data)-1)]))
                response = self.com.read(1)  # responds with 0x01 if successful
                if response == b'\x01':
                    for i in range(len(data)):
                        self.com.write(data=bytes([data[i]]))
                        response = self.com.read(1)
                        if response == b'\x01': # nack received from slave
                            self.log.entry("ERROR", "NACK received. Transmission aborted")
                            return
                else:
                    self.log.entry("ERROR", "I2C write operation not")
            else:
                self.log.entry("ERROR", "Specify the number of bytes to write (1 - 16)")
        except:
            pass

    @_verify_connection
    def read(self, device_addr, register_addr, num_bytes, repeated_start=True):
        try:
            addr_write = device_addr << 1
            addr_read = (device_addr << 1) + 1

            self.start_bit()
            self.write_bytes([addr_write, register_addr])
            if repeated_start:
                self.start_bit()
            self.write_bytes([addr_read])

            data = []
            for i in range(num_bytes):
                data.append(self.read_byte())

            self.nack_bit()
            self.stop_bit()

            return data
        except:
            pass


if __name__ == "__main__":
    pass
