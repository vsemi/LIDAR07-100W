import numpy as np
import time
import signal
import sys

import serial
import serial.tools.list_ports
from time import sleep
import binascii

POLYNOM = 0x04C11DB7
CRC_INIT_VALUE = 0xFFFFFFFF

TIME_OUT = 200

def _calcCrc32Uint32(crc, data) :
    crc = crc ^ data
    for i in range(32) :
        if crc & 0x80 :
            crc = (crc << 1) ^ POLYNOM
        else :
            crc = crc << 1
    return(crc)

def _calcCrc32_32(data, size) :
    crc = CRC_INIT_VALUE
    for i in range(size) :
        crc = _calcCrc32Uint32(crc, data[i])
    return crc ^ XOR_VALUE

def calculateChecksum(data, size) :
    '''
    Calculate Checksum.

    Parameters
    ----------
    data : bytearray
        bytearray data.
    size : int
        size.
    '''
    return _calcCrc32_32(data, size)

def calculateChecksum(bytes, size) :
    '''
    Calculate Checksum.

    Parameters
    ----------
    data : bytearray
        bytearray data.
    size : int
        size.
    '''
    crc = CRC_INIT_VALUE

    for d in range(size) :
        b = bytes[d]
        crc ^= (b << 24)

        for i in range(8) :
            if crc & 0x80000000 != 0 :
                crc = (crc << 1) ^ POLYNOM
            else :
                crc <<= 1

    return crc

def setUint32LittleEndian(buffer, index, value) :
    '''
    set Uint32 Little Endian.

    Parameters
    ----------
    buffer : bytearray
        data array.
    index : int
        starting index.
    value : int
        value to be set.
    '''
    buffer[index] = value & 0xFF
    buffer[index+1] = (value >> 8) & 0xFF
    buffer[index+2] = (value >> 16) & 0xFF
    buffer[index+3] = (value >> 24) & 0xFF

def getUint16LittleEndian(array, index) :
    '''
    get Uint16 Little Endian.

    Parameters
    ----------
    array : bytearray
        data array.
    index : int
        starting index.
    '''
    byte0 = array[index]
    byte1 = array[index+1]
    value = (byte1 << 8) | byte0
    return value

def debug_array(msg, array) :    
    
    print("len(array)", len(array))
    ################################
    res_len = len(array)
    debeg_str= msg
    for i in range(res_len) :
        buf = "%x" % array[i]
        debeg_str += buf
    print(debeg_str)
    ################################
    

class Sensor :
    '''
    ToF Sensor class, access point to the ToF Sensor.

    To initiate an instance of Sensor object, call static method of Sensor.open(port=None).
    '''

    def __init__(self):
        self._ser = serial.Serial()

        self._ser.baudrate = 115200
        self._ser.timeout = 0.001
        self._ser.parity = serial.PARITY_NONE
        self._ser.stopbits = serial.STOPBITS_ONE
        self._ser.bytesize = serial.EIGHTBITS

    @staticmethod
    def open(port):
        '''
        open communication to the Sensor via serial port.

        Parameters
        ----------
        port : str
            serial port the Sensor connected. 

        Returns
        ----------
        Sensor
            An instance of Sensor object which connected to the ToF Sensor.

        Raises
        ----------
        Exception
            Error to open the Sensor.
        '''

        sensor = Sensor()

        return sensor._open(port)

    def close(self):
        '''
        close communication to the sensor.
        '''
        if self._ser.is_open :
            self._ser.close()

        print("Sensor disconnected.")

    def _open(self, port):

        self._ser.port = port

        if self._ser.is_open :
            raise Exception("Sensor is already opened!")

        try:
            self._ser.open()
        except:
            print("Failed to open serial port: ", port)

        print("Sensor connected at port: ", port)

        return self

    def readData(self, size) :        
        size_with_command = size + 8

        array = bytearray(0)
        count = 0
        while (len(array) < size_with_command) :
            len_pending = size_with_command - len(array)
            in_waiting = self._ser.in_waiting
            sz = max(len_pending, self._ser.in_waiting)
            
            buf = self._ser.read(sz)
            a = bytearray(buf)
            array.extend(a)

            count += 1
            if count > TIME_OUT : break
        
        # debug_array("RECEIVED: ", array)

        return array

    def writeData(self, data) :
        data_len = len(data)
        # print("write data length: ", data_len)

        dataWithChecksum = bytearray(data_len + 4)

        for i in range(data_len) :
            dataWithChecksum[i] = data[i]
        
        checksum = calculateChecksum(data, data_len)
        setUint32LittleEndian(dataWithChecksum, data_len, checksum)

        debug_array("SEND COMMAND: ", dataWithChecksum)

        return self._ser.write(dataWithChecksum)

    def getVersion(self):
        data = bytearray(6)
        data[0] = 0xF5
        data[1] = 0x43

        data[2] = 0x00
        data[3] = 0x00
        data[4] = 0x00
        data[5] = 0x00
        
        if self.writeData(data) > 0:
            array = self.readData(4)

            if len(array) == 12 :
                versionMajor = getUint16LittleEndian(array, 6)
                versionMinor = getUint16LittleEndian(array, 4)
                # print("versionMajor: ", versionMajor)
                # print("versionMinor: ", versionMinor)

                return (versionMajor, versionMinor)

    def setMode(self, mode):
        data = bytearray(6)
        data[0] = 0xF5
        data[1] = 0x61 | 0x80 #0xE1

        if mode :
            data[2] = 0x01
        else :
            data[2] = 0x00

        data[3] = 0x00
        data[4] = 0x00
        data[5] = 0x00
        
        if self.writeData(data) > 0:
            array = self.readData(4)

            if len(array) == 12 :
                print("Sensor mode set to: ", data[2])
                return True

    def setPeriod(self, period):
        data = bytearray(6)
        data[0] = 0xF5
        data[1] = 0xE2

        data[2] = period
        data[3] = 0x00
        data[4] = 0x00
        data[5] = 0x00
        
        if self.writeData(data) > 0:
            array = self.readData(4)

            if len(array) == 12 :
                print("Period set to: ", period)

                return True

    def stop(self):
        data = bytearray(6)
        data[0] = 0xF5
        data[1] = 0xE0

        data[2] = 0x00
        data[3] = 0x00
        data[4] = 0x00
        data[5] = 0x00
        
        if self.writeData(data) > 0:
            array = self.readData(16)

            if len(array) == 24 :
                print("Sensor stopped.")
                return True

    def start(self):
        data = bytearray(6)
        data[0] = 0xF5
        data[1] = 0xE0

        data[2] = 0x01
        data[3] = 0x00
        data[4] = 0x00
        data[5] = 0x00
        
        if self.writeData(data) > 0:
            return self.measure()

    def measure(self) :        
        array = self.readData(16)

        res_len = len(array)

        if res_len == 24 :
            distance    = getUint16LittleEndian(array, 4)
            temporature = getUint16LittleEndian(array, 6)
            amplitude   = getUint16LittleEndian(array, 8)

            return (distance, temporature, amplitude)
