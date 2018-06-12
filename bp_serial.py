import os
import logging
import bp_bitbang


class BusPirateSerial(object):
    def __init__(self):
        # create directory for data storage
        if not os.path.exists('data'):
            os.makedirs('data')

        # configure logging
        log_format = '%(asctime)s [%(levelname)s] %(message)s'
        logging.basicConfig(format=log_format, filename='event_log.txt', level=logging.INFO)

        # configure serial port for BP interfacing
        self.bp_i2c = bp_bitbang.BitbangI2C(port='COM4', baudrate=115200)

    def i2c_connect(self, speed, vreg, pullups):
        try:
            self.bp_i2c.connect()
            self.bp_i2c.configure_protocol()
            self.bp_i2c.configure_speed(speed=speed)
            self.bp_i2c.configure_peripherals(vreg=vreg, pullups=pullups)
        except:
            pass

    def i2c_disconnect(self):
        self.bp_i2c.reset()
        self.bp_i2c.disconnect()
