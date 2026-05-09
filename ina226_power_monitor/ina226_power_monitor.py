#!/usr/bin/env python3
# Copyright (c) 2026 Scott Horton
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

from ina226 import INA226
import smbus

# Implement the micropython methods for reading the I2C bus that are used by
# the INA226 package
class smbus_to_micropython_wrapper():
    def __init__(self, smb_obj):
        self.smb_obj = smb_obj

    # Only supports 2-byte buf sizes
    def writeto_mem(self, addr, reg, buf):
        #print(f"Write to INA226: reg: {hex(reg)} data: {hex(buf[0])} {hex(buf[1])}")
        self.smb_obj.write_i2c_block_data(addr, reg, [buf[0], buf[1]])

    def readfrom_mem_into(self, addr, reg, buf):
        value = self.smb_obj.read_word_data(addr, reg)
        buf[1] = (value >> 8) & 0xFF
        buf[0] = value & 0xFF
        #print(f"Read from INA226: reg: {hex(reg)} data: {hex(buf[0])} {hex(buf[1])}")

class PowerMonitor(Node):
    def __init__(self):
        super().__init__('ina226_power_monitor')

        self.declare_parameter('i2c_bus', int(os.getenv("ROBOT_I2C_BUS", 8)))
        self.declare_parameter('dev_addr', 0x41)
        self.declare_parameter('max_current', 6.0)
        self.declare_parameter('shunt_value', 0.1)
        self.declare_parameter('current_lsb', 0.0)
        self.declare_parameter('cal_value', 0)
        self.declare_parameter('update_period_sec', 5.0)
        self.declare_parameter('topic', '/battery/status')
        # voltage thresholds for % charge
        # For 3 values, v > val[0] => 100% to 75%
        #               v > val[1] => 75% to 50%
        #               v > val[1] => 50% to 25%
        #               else < 25%
        self.declare_parameter('charge_voltage_thresholds', [21.0, 19.0, 18.0, 17.0, 16.0])
        # If voltage less than this value, then assume on wall power (specific to elsabot)
        self.declare_parameter('ext_power_voltage_thresh', 15.1)

        i2c_bus = self.get_parameter('i2c_bus').get_parameter_value().integer_value
        dev_addr = self.get_parameter('dev_addr').get_parameter_value().integer_value
        max_current = self.get_parameter('max_current').get_parameter_value().double_value
        shunt_value = self.get_parameter('shunt_value').get_parameter_value().double_value
        current_lsb = self.get_parameter('current_lsb').get_parameter_value().double_value
        cal_value = self.get_parameter('cal_value').get_parameter_value().integer_value
        update_period_sec = self.get_parameter('update_period_sec').get_parameter_value().double_value
        topic = self.get_parameter('topic').get_parameter_value().string_value
        self.charge_voltage_thresholds = self.get_parameter('charge_voltage_thresholds').get_parameter_value().double_array_value
        self.ext_power_voltage_thresh = self.get_parameter('ext_power_voltage_thresh').get_parameter_value().double_value

        self.get_logger().info(f'Using i2c bus: {i2c_bus}, dev_addr: {hex(dev_addr)}, ' \
                               f'max_current: {max_current}, shut_value: {shunt_value}, ' \
                               f'current_lsb: {current_lsb}, cal_value: {cal_value}, ' \
                               f'update_period_sec: {update_period_sec}, topic: {topic}, ' \
                               f'charge_voltage_thresholds: {self.charge_voltage_thresholds} ' \
                               f'ext_power_voltage_thresh: {self.ext_power_voltage_thresh}')

        i2c = smbus.SMBus(i2c_bus)

        self.smbus_wrapper = smbus_to_micropython_wrapper(i2c)
        self.ina226 = INA226(self.smbus_wrapper, addr=dev_addr)

        if current_lsb == 0.0:
            current_lsb = None
        if cal_value == 0:
            cal_value = None
        self.ina226.calibrate(r_shunt_ohms=shunt_value, max_expected_amps=max_current, current_lsb_a=current_lsb, cal_value=cal_value)

        self.pub = self.create_publisher(BatteryState, topic, 10)
        self.timer = self.create_timer(update_period_sec, self.status_callback)

        #self.test_charge_calc()

    def status_callback(self):
        voltage = self.ina226.bus_voltage
        current = self.ina226.current
        self.get_logger().debug(f'Voltage: {voltage} V, Current: {current} A')

        msg = BatteryState()
        msg.voltage = voltage
        msg.current = current
        msg.percentage = self.voltage_to_charge_percent(voltage)

        self.pub.publish(msg) 

    def voltage_to_charge_percent(self, voltage):
        # ext_power_voltage_thresh assumes the external (wall power) is always
        # lower than lowest battery voltage
        if voltage < self.ext_power_voltage_thresh:
            # On external power
            self.get_logger().info(f'voltage: {voltage:.2f} (ext), precent: 1.0')
            return 1.0

        num_thresh = len(self.charge_voltage_thresholds)

        if voltage > self.charge_voltage_thresholds[0]:
            voltage = self.charge_voltage_thresholds[0]
        elif voltage < self.charge_voltage_thresholds[num_thresh - 1]:
            voltage = self.charge_voltage_thresholds[num_thresh - 1]

        bin_incr_percent = 1.0/(num_thresh - 1)

        percent = 1.0
        prev_thresh = self.charge_voltage_thresholds[0]
        for idx, thresh in enumerate(self.charge_voltage_thresholds):
            if voltage >= thresh:
                percent = 1.0 - idx*bin_incr_percent
                if prev_thresh != thresh:
                    percent += (voltage - thresh)/(prev_thresh - thresh)*bin_incr_percent
                break
            prev_thresh = thresh

        self.get_logger().info(f'voltage: {voltage:.2f}, precent charge: {percent:.2f} ')
        return percent

    def test_charge_calc(self):
        print('Test charge calc:')

        for v in range(220, 140, -2):
            voltage = v/10.0
            percent = self.voltage_to_charge_percent(voltage)

def main(args=None):
    rclpy.init(args=args)

    command_timeout = PowerMonitor()
    rclpy.spin(command_timeout)
    command_timeout.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()