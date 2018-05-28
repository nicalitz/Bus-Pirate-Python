import serial.tools.list_ports
import bitbang

# print([port.device for port in serial.tools.list_ports.comports()])

bp_i2c = bitbang.BitbangI2C(port='COM3', baudrate=115200)
bp_i2c.connect()
bp_i2c.configure_protocol()
bp_i2c.configure_speed(speed=100)
bp_i2c.configure_peripherals(vreg=True, pullups=False)

# bp_i2c.start_bit()
# bp_i2c.write_bytes([0x42, 0x0C])
# bp_i2c.start_bit()
# bp_i2c.write_bytes([0x43])
# print(bp_i2c.read_byte())
# bp_i2c.nack_bit()
# bp_i2c.stop_bit()

print(bp_i2c.read(device_addr=0x21, register_addr=0x0C, num_bytes=1))

bp_i2c.reset()
bp_i2c.disconnect()