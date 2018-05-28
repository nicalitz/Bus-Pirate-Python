import serial
import time
import logging


def _verify_connection(method):
    """
    Decorator for methods of bitbang + derived classes. The following checks are performed in order:
      1) Serial port is open (and configured)
      2) Break condition is not set
    Exceptions are raised if the aforementioned check conditions are not met
    """
    def decorated(instance, *args, **kwargs):
        if instance.com.is_open:
            if not instance.com.break_condition:
                response = method(instance, *args, **kwargs)
                return response
            else:
                logging.error("Break condition active, no transmission possible.")
                raise serial.SerialException
        else:
            logging.error("Serial device not configured/connected.")
            raise serial.SerialException
    return decorated


class BitbangRaw(object):
    def __init__(self, port, baudrate, bytesize, parity, stopbits, timeout, xonxoff):
        """
        Configure serial, data logging and comms protocol parameters.

        :param port: serial/COM port
        :param baudrate: (typ. 115200)
        :param bytesize: (typ. 8-bit)
        :param parity: (typ. 'N' -> no parity check)
        :param stopbits: (typ. 1 stop bit)
        :param timeout: timeout period for serial comms (in seconds)
        :param xonxoff: xon-xoff flow control (typ. False)
        """
        try:
            # Configure serial port
            self.com = serial.Serial()
            self.com.port = port
            self.com.baudrate = baudrate
            self.com.bytesize = bytesize
            self.com.parity = parity
            self.com.stopbits = stopbits
            self.com.timeout = timeout
            self.com.xonxoff = xonxoff

            # Initialize attributes for protocol configuration
            self.protocol = 'RAW'
            self.protocol_code = None
            self.protocol_echo = None

            # Configure execution parameters
            self.delay = 0.05  # typical delay between bus pirate instructions
        except ValueError:
            logging.error("Serial port could not be configured. Initialization parameters are out of bounds.")
            raise
        except serial.SerialException:
            logging.error("Serial port could not be found/configured.")
            raise
        else:
            logging.info("Serial instance configured successfully.")

    def connect(self):
        """Open connection to serial device."""
        try:
            self.com.open()
            logging.info("Connected to serial device at {0}.".format(self.com.port))
        except serial.SerialException as error:
            logging.error("Failed to connect: {0}.".format(error))
            raise

    def disconnect(self):
        """Close connection to serial device."""
        try:
            self.com.close()
            logging.info("Disconnected from device at {0}.".format(self.com.port))
        except serial.SerialException as error:
            logging.error("Failed to disconnect: {0}.".format(error))
            raise

    @_verify_connection
    def reset(self):
        """
        [0000 1111 -> Reset Bus Pirate]
        Bus Pirate responds 0x01 and then performs a complete hardware reset.
        """
        try:
            self.mode_binary()  # reset to binary mode before attempting reset (seems necessary)
            self.com.write(bytes([0b00001111]))
            time.sleep(self.delay)
            response = self.com.read(1)
            if response == b'\x01':  # BP returns 0x01 before performing hardware reset
                logging.info("BusPirate: Terminal mode configured.")
                # wait one second and flush startup spam on comms interface (device info, etc.)
                time.sleep(1)
                self.com.read_all()
                return
            else:
                logging.error("Device could not be reset (terminal mode).")
                raise serial.SerialException
        except serial.SerialException:
            logging.error("Serial operation failed.")
            raise

    @_verify_connection
    def mode_binary(self):
        """
        [0000 0000 -> Reset (to binary mode)]
        Command resets the Bus Pirate into raw bitbang mode (from either terminal or other protocol modes). BP returns
        five byte bitbang version string "BBIOx".
        """
        try:
            for count in range(25):  # BP must receive 0x00 at least 20 times to enter raw bitbang mode (20 + 5 extra)
                self.com.write(bytes([0b00000000]))
                time.sleep(self.delay)
                response = self.com.read_all()
                if response == b'BBIO1':
                    logging.info("BusPirate: Binary mode configured ({0}).".format(str(count+1)))
                    return
        except serial.SerialException:
            logging.error("Serial operation failed.")
            raise
        else:
            logging.error("Connection timeout. Device could not be configured.")
            raise serial.SerialException

    @_verify_connection
    def mode_protocol(self):
        """
        [0000 xxxx -> Enter binary [xxx] mode]
        Configures the Bus Pirate for interfacing over a specific protocol. BP returns the pre-programmed protocol
        echo. This method relies on attributes configured by derived protocol classes.
        """
        try:
            self.com.write(self.protocol_code)
            time.sleep(self.delay)
            # TODO: Double check this read_all() call. Maybe rather use read(1)
            response = self.com.read_all()
            if response == self.protocol_echo:
                logging.info("BusPirate: Configured for {0} communication".format(self.protocol))
                return
            else:
                logging.error("Failed to configure device.")
                raise serial.SerialException
        except KeyError:
            logging.error("Specified protocol not implemented.")
            raise
        except serial.SerialException:
            logging.error("Serial operation failed.")
            raise


class BitbangI2C(BitbangRaw):
    def __init__(self, port, baudrate, bytesize=8, parity='N', stopbits=1, timeout=3, xonxoff=0):
        """
        Configure serial, data logging and comms protocol parameters.

        :param port: COM port
        :param baudrate: (typ. 115200)
        :param bytesize: (typ. 8-bit)
        :param parity: (typ. 'N' -> no parity check)
        :param stopbits: (typ. 1 stop bit)
        :param timeout: timeout period for serial comms (in seconds)
        :param xonxoff: xon-xoff flow control (typ. False)
        """
        try:
            super().__init__(port, baudrate, bytesize, parity, stopbits, timeout, xonxoff)

            # Initialize attributes for protocol configuration
            self.protocol = 'I2C'
            self.protocol_code = bytes([0b00000010])
            self.protocol_echo = b'I2C1'
        except Exception:  # pipe exception from parent class
            raise

    @_verify_connection
    def configure_protocol(self):
        """Configure BP for I2C interfacing."""
        try:
            self.mode_binary()  # reset BP to binary mode (seems necessary)
            self.mode_protocol()  # configure BP for interfacing over I2C
        except Exception:
            raise

    @_verify_connection
    def configure_speed(self, speed):
        """
        [0110 00xx -> Set I2C speed]
        Configure speed for I2C interfacing: 3=~400kHz, 2=~100kHz, 1=~50kHz, 0=~5kHz (updated in v4.2 firmware)
        BP responds 0x01 on success

        :param speed: I2C speed as string (in kHz)
        """
        valid_speeds = {'5': 0b00, '50': 0b01, '100': 0b10, '400': 0b11}  # valid I2C speeds (in kHz)
        try:
            # [0110 0000] + [0000 00xx] where [xx] = speed config
            self.com.write(bytes([0b01100000 + valid_speeds[str(speed)]]))
            time.sleep(self.delay)
            response = self.com.read(1)
            if response == b'\x01':  # responds 0x01 on success
                logging.info("BusPirate: I2C speed set to {0}KHz.".format(str(speed)))
                return
            else:
                logging.error("No confirmation message received from device.")
                raise serial.SerialException
        except KeyError:
            logging.error("Invalid I2C speed setting specified.")
            raise
        except serial.SerialException:
            logging.error("Serial operation failed.")
            raise

    @_verify_connection
    def configure_peripherals(self, vreg=False, pullups=False, aux=False, cs=False):
        """
        [0100 wxyz -> Configure peripherals. w=power, x=pullups, y=AUX, z=CS]
        Enable (True) and disable (False) BP peripherals and pins.

        :param vreg:    Configure voltage regulation (True=On; False=Off)
        :param pullups: Configure pull-up resistors (True=Enable; False=Disable)
        :param aux:     Set state of AUX pin [normal pin output] (True=3V3; False=GND)
        :param cs:      Set chip select pin (always follows HiZ pin configuration)
        """
        try:
            command = 0b01000000 + 0b1000*vreg + 0b0100*pullups + 0b0010*aux + 0b0001*cs
            self.com.write(bytes([command]))
            time.sleep(self.delay)
            response = self.com.read(1)
            if response == b'\x01': # responds 0x01 on success
                logging.info("BusPirate: Peripherals configured.")
                logging.info("V-Reg: {0}, Pull-ups: {1}, Aux: {2}, CS: {3}".format(str(vreg), str(pullups),
                                                                                   str(aux), str(cs)))
                return
            else:
                logging.error("No confirmation message received from device.")
                raise serial.SerialException
        except ValueError:
            logging.error("Peripherals configuration should be boolean.")
            raise
        except serial.SerialException:
            logging.error("Serial operation failed.")
            raise

    @_verify_connection
    def start_bit(self):
        """Send start bit."""
        try:
            self.com.write(bytes([0b00000010]))
            response = self.com.read(1)  # responds with 0x01 if successful
            if response == b'\x01':
                logging.debug("BP ACK: I2C start bit")
                return
            else:
                logging.error("Instruction not acknowledged by Bus Pirate.")
                raise serial.SerialException
        except serial.SerialException:
            logging.error("Serial operation failed.")
            raise

    @_verify_connection
    def stop_bit(self):
        """Send stop bit."""
        try:
            self.com.write(bytes([0b00000011]))
            response = self.com.read(1)  # responds with 0x01 if successful
            if response == b'\x01':
                logging.debug("BP ACK: I2C stop bit")
                return
            else:
                logging.error("Instruction not acknowledged by Bus Pirate.")
                raise serial.SerialException
        except serial.SerialException:
            logging.error("Serial operation failed.")
            raise

    @_verify_connection
    def ack_bit(self):
        """Send ACK bit."""
        try:
            self.com.write(bytes([0b00000110]))
            response = self.com.read(1)  # responds with 0x01 if successful
            if response == b'\x01':
                logging.debug("BP ACK: I2C ack bit")
                return
            else:
                logging.error("Instruction not acknowledged by Bus Pirate.")
                raise serial.SerialException
        except serial.SerialException:
            logging.error("Serial operation failed.")
            raise

    @_verify_connection
    def nack_bit(self):
        """Send NACK bit."""
        try:
            self.com.write(bytes([0b00000111]))
            response = self.com.read(1)  # responds with 0x01 if successful
            if response == b'\x01':
                logging.debug("BP ACK: I2C nack bit")
                return
            else:
                logging.error("Instruction not acknowledged by Bus Pirate.")
                raise serial.SerialException
        except serial.SerialException:
            logging.error("Serial operation failed.")
            raise

    @_verify_connection
    def read_byte(self):
        """[0000 0100 -> Read a byte from the I2C bus. Must be ACK/NACK'ed manually]"""
        try:
            self.com.write(bytes([0b00000100]))
            response = self.com.read(1)
            logging.debug("Byte read: [{0}]".format(response))
            return response
        except serial.SerialException:
            logging.error("Serial read operation failed.")
            raise

    @_verify_connection
    def write_bytes(self, data):
        """
        [0001 xxxx -> Bulk I2C write, send 1-16 bytes (0=1byte!)]

        :param data:    Data bytes to transmit over I2C as list (len 1-16)
        """
        try:
            if 1 <= len(data) <= 16:
                self.com.write(bytes([0b00010000 + (len(data)-1)]))
                response = self.com.read(1)  # responds with 0x01 if successful
                if response == b'\x01':
                    for i in range(len(data)):
                        self.com.write(bytes([data[i]]))
                        response = self.com.read(1)
                        if response == b'\x01': # nack received from slave
                            logging.error("NACK received. Transmission aborted.")
                            raise serial.SerialException
                else:
                    logging.error("I2C write operation not acknowledged by BP. Aborted.")
                    raise serial.SerialException
            else:
                logging.error("Specify the number of bytes to write within range: (1 - 16).")
                raise ValueError
        except serial.SerialException:
            logging.error("Serial operation failed.")
            raise

    @_verify_connection
    def read(self, device_addr, register_addr, num_bytes, repeated_start=True):
        """
        I2C read operation. (Master <-> Slave) flow as follow:
        M -> [S][dev_addr][W]     [reg_addr]     [RS][dev_addr][R]              [ACK] ...         [NACK][STOP]
        S ->                 [ACK]          [ACK]                 [ACK?][data_1]      ... [data_n]

        :param device_addr:     I2C device address
        :param register_addr:   Register address [from which to read bytes(s)]
        :param num_bytes:       Number of bytes to read from register address
        :param repeated_start:  Repeated start bit required (bool)
        """
        try:
            # configure write and read addresses
            addr_write = device_addr << 1
            addr_read = (device_addr << 1) + 1

            self.start_bit()
            self.write_bytes([addr_write, register_addr])
            if repeated_start:  # repeated start bit required
                self.start_bit()
            self.write_bytes([addr_read])

            data = []
            for i in range(num_bytes):
                data.append(self.read_byte())

            self.nack_bit()
            self.stop_bit()

            return data
        except Exception:
            raise

    @_verify_connection
    def write(self, device_addr, register_addr, data):
        """
        I2C write operation. (Master <-> Slave) flow as follow:
        M -> [S][dev_addr][W]     [reg_addr]     [data_1]      ... [data_n]     [STOP]
        S ->                 [ACK]          [ACK]        [ACK] ...         [ACK]

        :param device_addr:     I2C device address
        :param register_addr:   Register address [to write to]
        :param data:            Data to write [list]
        """
        try:
            # configure write address
            addr_write = device_addr << 1

            self.start_bit()
            self.write_bytes([addr_write, register_addr])
            self.write_bytes(data)
            self.stop_bit()
        except Exception:
            raise


if __name__ == "__main__":
    pass
