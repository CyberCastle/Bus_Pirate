#!/usr/bin/env python3
# encoding: utf-8
"""
Created by Piotr Gertz on 2020-01-30.
Copyright 2020 Piotr Gertz <(p)(i)(o)(t)(r)@piopawlu.net>

This file is part of pyBusPirate.

pyBusPirate is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

pyBusPirate is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with pyBusPirate.  If not, see <http://www.gnu.org/licenses/>.
"""
import sys, optparse
# import numpy as np
from time import sleep
from struct import unpack
from pyBusPirateLite.SPI import *

class BMECompA:
    dig = None

    def __init__(self, raw : bytes):
        self.dig = unpack("<HhhH8hxB", raw)

    def P(self, i : int):
        return self.dig[i + 2]

    def T(self, i : int):
        return self.dig[i - 1]

class BMECompB:
    dig = None

    def __init__(self, rawB : bytes):
        tmp = unpack("<BhBbBbb", rawB)

        self.dig = [
            tmp[0], # H1
            tmp[1], # H2
            tmp[2], # H3
            (tmp[3] << 4 | (tmp[4] & 0x0F)), # H4
            (tmp[5] << 4 | (tmp[4] >> 4)), # H5
            tmp[6] # H6
            ]

    def H(self, i : int):
        return self.dig[i-1]

class BME280:
    spi = None
    t_fine = 0

    def __init__(self, spi: SPI):
        self.spi = spi

    def begin(self):
        self.spi.CS_Low()

    def end(self):
        self.spi.CS_High()

    def read_reg8(self, reg_addr):
        return self.spi.bulk_trans(2, [reg_addr, 0])[1]

    def read_reg16(self, reg_addr):
        return self.spi.bulk_trans(3, [reg_addr, 0, 0])[1:]

    def write_reg8(self, reg_addr, data):
        self.spi.bulk_trans(2, [reg_addr & 0x7F, data])

    def read_data(self, addr, count):
        return self.spi.write_then_read_no_cs([addr], count)


    def calculateTemperature(self, raw : int, rawCompA : bytes) -> int:
        dig = BMECompA(rawCompA)

        var1 = ((((raw >> 3) - (dig.T(1) << 1))) * (dig.T(2))) >> 11
        var2 = (((((raw >> 4) - (dig.T(1))) * ((raw >> 4) - (dig.T(1)))) >> 12) * (dig.T(3))) >> 14

        self.t_fine = var1 + var2
    
        return (self.t_fine*5 + 128) >> 8

    def calculateHumidity(self, raw : int, rawCompB : bytes) -> int:
        dig = BMECompB(rawCompB)

        var_H = self.t_fine - 76800.0
        var_H = (raw - (dig.H(4) * 64.0 + dig.H(5) / 16384.0 * var_H)) * (dig.H(2) / 65536.0 * (1.0 + dig.H(6)/67108864.0 * var_H * (1.0 + dig.H(3)/67108864.0 * var_H)))
        var_H = var_H * (1.0 - dig.H(1) * var_H / 524288.0)

        humidity = var_H
        
        if humidity > 100.0: return 100.0
        elif humidity < 0.0: return 0.0
        
        return humidity
        
    def calculatePressure(self, raw : int, rawCompA : bytes) -> int:
        dig = BMECompA(rawCompA)

        pressure_min = 3000000
        pressure_max = 11000000
        pressure = 0

        var1 = (self.t_fine) - 128000
        var2 = var1 * var1 * dig.P(6)
        var2 = var2 + ((var1 * dig.P(5)) * 131072)
        var2 = var2 + ((dig.P(4)) * 34359738368)
        var1 = ((var1 * var1 * dig.P(3)) / 256) + (var1 * (dig.P(2) * 4096))
        var3 = 140737488355328
        var1 = (var3 + var1) * dig.P(1) / 8589934592

        if (var1 != 0):
            var4 = 1048576 - raw
            var4 = (((var4 * 2147483648) - var2) * 3125) / var1
            var1 = (dig.P(9) * (var4 / 8192) * (var4 / 8192)) / 33554432
            var2 = (dig.P(8) * var4) / 524288
            var4 = ((var4 + var1 + var2) / 256) + (dig.P(7) * 16)

            pressure = (((var4 / 2) * 100) / 128)

            if (pressure < pressure_min):
                pressure = pressure_min
            elif (pressure > pressure_max):
                pressure = pressure_max
        else:
            pressure = pressure_min
        
        return pressure / 100
    
""" enter binary mode """
def buspirate_test(buspirate_dev:str, filename:str):

    spi = SPI(buspirate_dev, 115200)

    print("Entering binmode: ")
    if spi.BBmode():
        print("OK.")
    else:
        print("failed.")
        sys.exit()

    print("Entering raw SPI mode: ")
    if spi.enter_SPI():
        print("OK.")
    else:
        print("failed.")
        sys.exit()

    print("Configuring SPI peripherals: ")
    if spi.cfg_pins(PinCfg.POWER | PinCfg.CS):
        print("OK.")
    else:
        print("failed.")
        sys.exit()

    sleep(0.5)

    print("Configuring SPI speed: ")
    if spi.set_speed(SPISpeed._125KHZ):
        print("OK.")
    else:
        print("failed.")
        sys.exit()
    print("Configuring SPI configuration: ")
    if spi.cfg_spi(SPICfg.CLK_EDGE | SPICfg.OUT_TYPE):
        print("OK.")
    else:
        print("failed.")
        sys.exit()
    spi.timeout(0.2)


    bme280 = BME280(spi)

    bme280.begin()

    print("Checking chip ID: ", end='')
    chipID = bme280.read_reg8(0xD0)

    if chipID == 0x60:
        print("OK")
    else:
        print(f"ERROR ({chipID} != 0x60")
        sys.exit()

    bme280.end()

    print("Starting conversion")
    bme280.begin()
    bme280.write_reg8(0xF2, 0x02)
    bme280.write_reg8(0xF4, 0b01001010)
    bme280.end()

    bme280.begin()
    print("Waiting for result.", end='')
    while bme280.read_reg8(0xF3) & 0x80:
        print('.',end='')
        sleep(0.01)
    print("OK")
    bme280.end()

    print("Reading results:", end='')

    bme280.begin()
    bres = bme280.read_data(0xF7, 8)
    bme280.end()

    if len(bres) == 8:
        print("OK")
    else:
        print("ERROR")
        sys.exit()

    raw_humidity = (bres[6] << 8) | bres[7]
    raw_pressure = (bres[0] << 12) | (bres[1] << 4) | (bres[2] >> 4)
    raw_temp = (bres[3] << 12) | (bres[4] << 4) | (bres[5] >> 4)

    print(f"Raw Humidity:{raw_humidity} Raw Pressure:{raw_pressure} Raw Temperature:{raw_temp}")

    bme280.begin()
    bcompA = bme280.read_data(0x88, 26) # 0x88 - 0xA1
    bme280.end()

    bme280.begin()
    bcompB = bme280.read_data(0xE1, 7) # 0xE1 - 0xE7
    bme280.end()

    if filename is not None:
        fp = open("bme280.bin","wb")
        fp.write(bres)
        fp.write(bcompA)
        fp.write(bcompB)
        fp.close()

    print("Reset Bus Pirate to user terminal: ", end='')
    if spi.resetBP():
        print("OK")
    else:
        print("ERROR")
        sys.exit()

    temperature = bme280.calculateTemperature(raw_temp, bcompA)
    pressure = bme280.calculatePressure(raw_pressure, bcompA)
    humidity = bme280.calculateHumidity(raw_humidity, bcompA[25:] + bcompB)

    print("Temperature: {:.2f}C".format(temperature/100.0))
    print("Pressure: {:.2f}hPa".format(pressure/100.0))
    print("RH: {:.2f}%".format(humidity))

def file_test(filename:str):

    bme280 = BME280(None)

    fp = open(filename, "rb")

    bres = fp.read(8)

    raw_humidity = (bres[6] << 8) | bres[7]
    raw_pressure = (bres[0] << 12) | (bres[1] << 4) | (bres[2] >> 4)
    raw_temp = (bres[3] << 12) | (bres[4] << 4) | (bres[5] >> 4)

    print(f"Raw Humidity:{raw_humidity} Raw Pressure:{raw_pressure} Raw Temperature:{raw_temp}")

    bcompA = fp.read(26) # 0x88 - 0xA1
    bcompB = fp.read(7) # 0xE1 - 0xE6
    fp.close()

    temperature = bme280.calculateTemperature(raw_temp, bcompA)
    pressure = bme280.calculatePressure(raw_pressure, bcompA)
    humidity = bme280.calculateHumidity(raw_humidity, bcompA[25:] + bcompB)
    
    print("Temperature: {:.2f}C".format(temperature/100.0))
    print("Pressure: {:.2f}hPa".format(pressure/100.0))
    print("RH: {:.2f}%".format(humidity))

if __name__ == '__main__':
    buspirate_test("/dev/tty.usbserial-pirate_1", None)
    #file_test("bme280.bin")