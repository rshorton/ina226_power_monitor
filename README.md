# Power Monitor using TI INA226 

This package implements a power monitor for reporting the battery status.

It requires the package TI_INA226_micropython from https://github.com/elschopi/TI_INA226_micropython/blob/master/ina226.py
for reading the INA226 device.

The voltage and current are read using the INA226 and published to the specified topic.  Supported parameters:
   i2c_bus - Bus used to read the INA226 (default: 8)
   dev_addr - Device address of the INA226 (default: 0x41 - A0 jumpered to Vcc)
   max_current - Maximum expected current (default: 6A)
   shunt_value - Value of shunt resistor on the INA226
   update_period_sec - Delay between updates (default: 5 sec)
   topic - Topic to publish BatteryState sensor_msgs.msg (default: /battery/status)

