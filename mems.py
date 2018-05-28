import serial
import bitbang
import logging
import time


class MEMS:
    def __init__(self):
        log_format = '%(asctime)s [%(levelname)s] %(message)s'
        logging.basicConfig(format=log_format, filename='event_log.txt', level=logging.INFO)
        bp_i2c = bitbang.BitbangI2C(port='COM3', baudrate=115200)
        while True:
            try:
                bp_i2c.connect()
                bp_i2c.configure_protocol()
                bp_i2c.configure_speed(speed=100)
                bp_i2c.configure_peripherals(vreg=True, pullups=False)
                # print(bp_i2c.read(device_addr=0x21, register_addr=0x0C, num_bytes=1))
                bp_i2c.write(0x21, 0x0D, [0b00000011])
                bp_i2c.write(0x21, 0x13, [0b00100010])
                while True:
                    x_data = bp_i2c.read(0x21, 0x01, 1)[0]
                    x_data += bp_i2c.read(0x21, 0x02, 1)[0]
                    data = [(7.8125/1000)*int.from_bytes(x_data, byteorder='big', signed=True)]
                    y_data = bp_i2c.read(0x21, 0x03, 1)[0]
                    y_data += bp_i2c.read(0x21, 0x04, 1)[0]
                    data.append((7.8125/1000)*int.from_bytes(y_data, byteorder='big', signed=True))
                    z_data = bp_i2c.read(0x21, 0x05, 1)[0]
                    z_data += bp_i2c.read(0x21, 0x06, 1)[0]
                    data.append((7.8125/1000)*int.from_bytes(z_data, byteorder='big', signed=True))
                    print(data)
            except serial.SerialException:
                pass
            except KeyError:
                pass
            finally:
                bp_i2c.reset()
                bp_i2c.disconnect()


if __name__ == "__main__":
    mems = MEMS()
