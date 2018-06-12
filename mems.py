import serial
import bitbang
import logging
from datetime import datetime, timedelta
import os
import csv
import time

class MEMS:
    def __init__(self):
        # create directory for data storage
        if not os.path.exists('data'):
            os.makedirs('data')

        # configure logging
        log_format = '%(asctime)s [%(levelname)s] %(message)s'
        logging.basicConfig(format=log_format, filename='event_log.txt', level=logging.INFO)

        # configure serial port for BP interfacing
        self.bp_i2c = bitbang.BitbangI2C(port='COM4', baudrate=115200)

        # gyro addresses
        self.nxp_addr = [0x20, 0x21]
        self.st_addr = [0x68, 0x69]

        # sensitivity values
        self.nxp_sensitivity = 0
        self.st_sensitivity = 0

    def i2c_connect(self):
        self.bp_i2c.connect()
        self.bp_i2c.configure_protocol()
        self.bp_i2c.configure_speed(speed=100)
        self.bp_i2c.configure_peripherals(vreg=True, pullups=False)

    def i2c_disconnect(self):
        self.bp_i2c.reset()
        self.bp_i2c.disconnect()

    # ============================================================================================================
    #  NXP Sensor
    # ============================================================================================================

    def nxp_id(self, device_addr):
        try:
            did = self.bp_i2c.read(device_addr, 0x0C, 1, True)[0]
            did_hex = hex(int.from_bytes(did, byteorder='big', signed=False))
            print("NXP device ID (0xD7): {0}".format(did_hex))
        except Exception:
            raise

    def nxp_configure(self, device_addr):
        try:
            # ------------------------------------------------------------
            # 0x0D: CTRL_REG0 configuration
            # ------------------------------------------------------------
            bandwidth = 0b00  # 00->high, 01->mid, 10/11->low
            spi_mode = 0b0  # 0->4-wire(default), 1->3-wire(MOSI for io)
            hpf_freq = 0b00  # 00->high, 01->mid-high, 10->mid-low, 11->low (high pass cutoff frequency)
            hpf_en = 0b0  # 0->disabled, 1->enabled (high pass filter enable)
            fs_range = 0b11  # 00->2000dps, 01->1000dps, 10->500dps, 11->250dps (full scale range)
            self.nxp_sensitivity = 62.5 / (2 ** fs_range)  # mdps/LSB (function of full scale range)

            ctrl_reg0 = [0x0D]
            ctrl_reg0.append((bandwidth << 6) + (spi_mode << 5) + (hpf_freq << 3) + (hpf_en << 2) + fs_range)

            self.bp_i2c.write(device_addr, ctrl_reg0[0], [ctrl_reg0[1]])

            # ------------------------------------------------------------
            # 0x13: CTRL_REG1 configuration
            # ------------------------------------------------------------
            sw_rst = 0b0  # 0->reset not triggered, 1->reset triggered (software reset)
            self_test = 0b0  # 0->self test disabled, 1->self test enabled
            data_rate = 0b000  # 000->800Hz, 001->400Hz, 010->200Hz, 011->100Hz, 100->50Hz, 101->25Hz, 110->12.5Hz
            active = 0b1  # A=0;R=0->Standby Mode, A=0;R=1->Ready Mode, A=1;R=x->Active Mode
            ready = 0b0  # See above

            ctrl_reg1 = [0x13]
            ctrl_reg1.append((sw_rst << 6) + (self_test << 5) + (data_rate << 2) + (active << 1) + ready)

            self.bp_i2c.write(device_addr, ctrl_reg1[0], [ctrl_reg1[1]])
        except Exception:
            raise

    def nxp_selftest(self, device_addr):
        try:
            ctrl_reg0_store = self.bp_i2c.read(device_addr, 0x0D, 1, True)[0]
            ctrl_reg0_store = int.from_bytes(ctrl_reg0_store, byteorder='big', signed=False)

            ctrl_reg1_store = self.bp_i2c.read(device_addr, 0x13, 1, True)[0]
            ctrl_reg1_store = int.from_bytes(ctrl_reg1_store, byteorder='big', signed=False)

            # active -> ready
            self.bp_i2c.write(device_addr, 0x13, [ctrl_reg1_store | 0b00000001])

            # configure max full scale range
            self.bp_i2c.write(device_addr, 0x0D, [ctrl_reg0_store & 0b11111100])

            # configure self test mode
            self.bp_i2c.write(device_addr, 0x13, [ctrl_reg1_store | 0b00100000])

            # self test
            fscale = self.nxp_sensitivity / 1000
            data = [int(x/fscale) for x in self.nxp_xyz(device_addr)]
            logging.info('NXP self test ({:x}) -> X:{:d}, Y:{:d}, Z:{:d}'.format(device_addr, abs(data[0]),
                                                                                 abs(data[1]), abs(data[2])))

            # revert to previous configuration
            self.bp_i2c.write(device_addr, 0x13, [ctrl_reg1_store | 0b00000001])
            self.bp_i2c.write(device_addr, 0x0D, [ctrl_reg0_store])
            self.bp_i2c.write(device_addr, 0x13, [ctrl_reg1_store])
        except Exception:
            raise


    def nxp_xyz(self, device_addr):
        try:
            fscale = self.nxp_sensitivity / 1000
            # read and process x-axis rate data
            x_data = self.bp_i2c.read(device_addr, 0x01, 1, True)[0]
            x_data += self.bp_i2c.read(device_addr, 0x02, 1, True)[0]
            data = [fscale * int.from_bytes(x_data, byteorder='big', signed=True)]
            # read and process y-axis rate data
            y_data = self.bp_i2c.read(device_addr, 0x03, 1, True)[0]
            y_data += self.bp_i2c.read(device_addr, 0x04, 1, True)[0]
            data.append(fscale * int.from_bytes(y_data, byteorder='big', signed=True))
            # read and process z-axis rate data
            z_data = self.bp_i2c.read(device_addr, 0x05, 1, True)[0]
            z_data += self.bp_i2c.read(device_addr, 0x06, 1, True)[0]
            data.append(fscale * int.from_bytes(z_data, byteorder='big', signed=True))
            # return rate data as [X, Y, Z]
            return data
        except Exception:
            raise

    def nxp_temp(self, device_addr):
        try:
            # read and process temp data
            temp = self.bp_i2c.read(device_addr, 0x12, 1, True)[0]
            data= [int.from_bytes(temp, byteorder='big', signed=True)]
            # return temp data as [temp(°C)]
            return data
        except Exception:
            raise

    # ============================================================================================================
    #  ST Sensor
    # ============================================================================================================

    def st_id(self, device_addr):
        try:
            did = self.bp_i2c.read(device_addr, 0x0F, 1, True)[0]
            did_hex = hex(int.from_bytes(did, byteorder='big', signed=False))
            print("ST device ID (0xD3): {0}".format(did_hex))
        except Exception:
            raise

    def st_configure(self, device_addr):
        try:
            # ------------------------------------------------------------
            # 0x20: CTRL_REG1 configuration
            # ------------------------------------------------------------
            data_rate = 0b00
            bandwidth = 0b00
            power = 0b1  # 0->power down, 1->normal or sleep
            z_en = 0b1  # 0->disabled, 1->enabled (z-axis)
            y_en = 0b1  # 0->disabled, 1->enabled (y-axis)
            x_en = 0b1  # 0->disabled, 1->enabled (x-axis)

            ctrl_reg1 = [0x20]
            ctrl_reg1.append((data_rate << 6) + (bandwidth << 4) + (power << 3) + (z_en << 2) + (y_en << 1) + x_en)

            self.bp_i2c.write(device_addr, ctrl_reg1[0], [ctrl_reg1[1]])

            # ------------------------------------------------------------
            # 0x21: CTRL_REG2 configuration
            # ------------------------------------------------------------
            hpf_mode = 0b00  # 00->normal, 01->ref for signal filtering; 10->normal, 11->auto reset on interrupt
            hpf_cutoff = 0b0000  # see datasheet

            ctrl_reg2 = [0x21]
            ctrl_reg2.append((hpf_mode << 4) + hpf_cutoff)

            self.bp_i2c.write(device_addr, ctrl_reg2[0], [ctrl_reg2[1]])

            # ------------------------------------------------------------
            # 0x22: CTRL_REG3 configuration
            # ------------------------------------------------------------
            int1_int = 0b0  # interrupt enable on INT1 pin (0->disable (default), 1->enable)
            int1_boot = 0b0  # boot status on INT1 pin (0->disable (default), 1->enable)
            int1_active = 0b0  # interrupt active configuration on INT1 (0->high (def), 1->low)
            pp_od = 0b0  # 0->push-pull, 1->open drain
            int2_drdy = 0b0  # data ready on int2 (0->disable (def), 1->enable)
            int2_wm = 0b0  # FIFO watermark interrupt on INT2 (0->disable (def), 1->enable)
            int2_orun = 0b0  # FIFO overrun interrupt on INT2 (0->disable (def), 1->enable)
            i2_empty = 0b0  # FIFO empty interrupt on INT2 (0->disable (def), 1->enable)

            ctrl_reg3 = [0x22]
            ctrl_reg3.append((int1_int << 7) + (int1_boot << 6) + (int1_active << 5) + (pp_od << 4) +
                             (int2_drdy << 3) + (int2_wm << 2) + (int2_orun << 1) + i2_empty)

            self.bp_i2c.write(device_addr, ctrl_reg3[0], [ctrl_reg3[1]])

            # ------------------------------------------------------------
            # 0x23: CTRL_REG4 configuration
            # ------------------------------------------------------------
            ble = 0b0  # 0->big endian (default), 1->little endian
            fs_rate = 0b00  # 00->245dps, 01->500dps, 10/11->2000dps
            self_test = 0b00  # 00->normal, 01->self-test 0(+), 10->n/a, 11->self-test 1(-)
            spi_sel = 0b0  # 0->4-wire (default), 1->3-wire

            if fs_rate == 0b00:
                self.st_sensitivity = 8.75
            elif fs_rate == 0b01:
                self.st_sensitivity = 17.5
            else:
                self.st_sensitivity = 70.0

            ctrl_reg4 = [0x23]
            ctrl_reg4.append((ble << 6) + (fs_rate << 4) + (self_test << 1) + spi_sel)
            # print(bin((ble << 6) + (fs_rate << 4) + (self_test << 1) + spi_sel))

            self.bp_i2c.write(device_addr, ctrl_reg4[0], [ctrl_reg4[1]])

            # ------------------------------------------------------------
            # 0x24: CTRL_REG5 configuration
            # ------------------------------------------------------------
            boot = 0b0  # reboot memory content (0->normal (default), 1->reboot memory content)
            fifo_en = 0b0  # fifo enable (0->disable (default), 1->enable)
            hpen = 0b0  # high pass filter enable (0->disabled (def), 1->enabled)
            int1 = 0b00  # see datasheet
            out = 0b00  # see datasheet

            ctrl_reg5 = [0x24]
            ctrl_reg5.append((boot << 7) + (fifo_en << 6) + (hpen << 4) + (int1 << 2) + out)

            self.bp_i2c.write(device_addr, ctrl_reg5[0], [ctrl_reg5[1]])
        except Exception:
            raise

    def st_selftest(self, device_addr):
        try:
            reg_store = self.bp_i2c.read(device_addr, 0x23, 1, True)[0]
            reg_store = int.from_bytes(reg_store, byteorder='big', signed=False)

            # self test positive
            reg_test_pos = reg_store | 0b00000010
            self.bp_i2c.write(device_addr, 0x23, [reg_test_pos])
            data = self.st_xyz(device_addr)
            logging.info('ST self test [+] ({:x}) -> X:{:.5f}, Y:{:.5f}, Z:{:.5f}'.format(device_addr, abs(data[0]),
                                                                                          abs(data[1]), abs(data[2])))
            # self test negative
            reg_test_pos = reg_store | 0b00000110
            self.bp_i2c.write(device_addr, 0x23, [reg_test_pos])
            data = self.st_xyz(device_addr)
            logging.info('ST self test [-] ({:x}) -> X:{:.5f}, Y:{:.5f}, Z:{:.5f}'.format(device_addr, abs(data[0]),
                                                                                          abs(data[1]), abs(data[2])))

            self.bp_i2c.write(device_addr, 0x23, [reg_store])
        except Exception:
            raise

    def st_xyz(self, device_addr):
        try:
            data_ready = 0
            while data_ready == 0:
                status_reg = self.bp_i2c.read(device_addr, 0x27, 1, True)[0]
                data_ready = (int.from_bytes(status_reg, byteorder='big', signed=False)) & 0b00001000

            data_read = [self.bp_i2c.read(device_addr, 0x28, 1)[0]]
            data_read.append(self.bp_i2c.read(device_addr, 0x29, 1, True)[0])
            data_read.append(self.bp_i2c.read(device_addr, 0x2A, 1, True)[0])
            data_read.append(self.bp_i2c.read(device_addr, 0x2B, 1, True)[0])
            data_read.append(self.bp_i2c.read(device_addr, 0x2C, 1, True)[0])
            data_read.append(self.bp_i2c.read(device_addr, 0x2D, 1, True)[0])

            fscale = (self.st_sensitivity / 1000)
            data = [fscale * int.from_bytes(data_read[1] + data_read[0], byteorder='big', signed=True)]
            data.append(fscale * int.from_bytes(data_read[3] + data_read[2], byteorder='big', signed=True))
            data.append(fscale * int.from_bytes(data_read[5] + data_read[4], byteorder='big', signed=True))

            return data
        except Exception:
            raise

    def st_temp(self, device_addr):
        try:
            # read and process temp data
            temp = self.bp_i2c.read(device_addr, 0x26, 1, True)[0]
            data = [int.from_bytes(temp, byteorder='big', signed=True)]
            # return temp data as [temp(°C)]
            return data
        except Exception:
            raise


if __name__ == "__main__":
    mems = MEMS()
    err_count = tuple([0] * 4)
    timestamp = datetime.strftime(datetime.now(), '%Y-%m-%d %H-%M-%S')
    with open(os.path.join('data', timestamp + '.csv'), 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=',')
        while True:
            try:
                timestamp_start = datetime.now()
                mems.i2c_connect()

                # connect to NXP sensor at I2C low
                try:
                    if err_count[0] < 25:
                        mems.nxp_configure(mems.nxp_addr[0])
                except Exception:
                    err_count[0] += 1
                    raise

                # connect to NXP sensor at I2C high
                try:
                    if err_count[1] < 25:
                        mems.nxp_configure(mems.nxp_addr[1])
                except Exception:
                    err_count[1] += 1
                    raise

                # connect to ST sensor at I2C low
                try:
                    if err_count[2] < 25:
                        mems.st_configure(mems.st_addr[0])
                except Exception:
                    err_count[2] += 1
                    raise

                # connect to ST sensor at I2C high
                try:
                    if err_count[3] < 25:
                        mems.st_configure(mems.st_addr[1])
                except Exception:
                    err_count[3] += 1
                    raise

                while True:
                    # print error count and do self check on all sensors every 10 min
                    if ((datetime.now()-timestamp_start).seconds/60) > 10.0:
                        print(err_count)

                        if err_count[0] < 25:
                            try:
                                mems.nxp_selftest(mems.nxp_addr[0])
                            except KeyboardInterrupt:
                                raise
                            except Exception:
                                pass

                        if err_count[1] < 25:
                            try:
                                mems.nxp_selftest(mems.nxp_addr[1])
                            except KeyboardInterrupt:
                                raise
                            except Exception:
                                pass

                        if err_count[2] < 25:
                            try:
                                mems.st_selftest(mems.st_addr[0])
                            except KeyboardInterrupt:
                                raise
                            except Exception:
                                pass

                        if err_count[3] < 25:
                            try:
                                mems.st_selftest(mems.st_addr[1])
                            except KeyboardInterrupt:
                                raise
                            except Exception:
                                pass

                        timestamp_start = datetime.now()

                    try:
                        for addr in range(2):
                            if err_count[addr] < 25:
                                data = mems.nxp_xyz(mems.nxp_addr[addr])
                                data += mems.nxp_temp(mems.nxp_addr[addr])
                                writer.writerow(['NXP{:d}'.format(addr)] + ["{:.5f}".format(x) for x in data])

                        for addr in range(2):
                            if err_count[addr + 2] < 25:
                                data = mems.st_xyz(mems.st_addr[addr])
                                data += mems.st_temp(mems.st_addr[addr])
                                writer.writerow(['ST{:d}'.format(addr)] + ["{:.5f}".format(x) for x in data])

                        time.sleep(6)
                    except Exception:
                        raise
            except (serial.SerialException, ValueError, KeyError):
                mems.i2c_disconnect()
            except KeyboardInterrupt:
                mems.i2c_disconnect()
                del mems
                break

        #     mems.nxp_configure(mems.nxp_addr[1])
        #     mems.st_configure(mems.st_addr[0])
        #     mems.st_configure(mems.st_addr[1])
        #
        #
        #
        #         writer = csv.writer(csvfile, delimiter=',')
        #         while True:
        #             print(datetime.strftime(datetime.now(), '%H-%M-%S'))
        #
        #             nxp_data = mems.nxp_xyz(mems.nxp_addr[0])
        #             nxp_data += mems.nxp_temp(mems.nxp_addr[0])
        #             print(["NXP[0]"] + ["{:.5f}".format(x) for x in nxp_data])
        #
        #             nxp_data = mems.nxp_xyz(mems.nxp_addr[1])
        #             nxp_data += mems.nxp_temp(mems.nxp_addr[1])
        #             print(["NXP[1]"] + ["{:.5f}".format(x) for x in nxp_data])
        #             #
        #             st_data = mems.st_xyz(mems.st_addr[0])
        #             st_data += mems.st_temp(mems.st_addr[0])
        #             print(["ST[0]"] + ["{:.5f}".format(x) for x in st_data])
        #             #
        #             st_data = mems.st_xyz(mems.st_addr[1])
        #             st_data += mems.st_temp(mems.st_addr[1])
        #             print(["ST[1]"] + ["{:.5f}".format(x) for x in st_data])
        #
        #             # mems.nxp_selftest(mems.nxp_addr[1])
        #
        #             time.sleep(5)
        #
        #             # writer.writerow(["{:.5f}".format(x) for x in (nxp_data + st_data)])
        # except (serial.SerialException, ValueError, KeyError):
        #     pass
        # except KeyboardInterrupt:
        #     mems.i2c_disconnect()
        #     del mems
        #     break
        # mems.i2c_disconnect()
