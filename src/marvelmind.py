#!/usr/bin/env python
#
# marvelmind.py - small class for recieve and parse coordinates from Marvelmind mobile beacon by USB/serial port
# Written by Alexander Rudykh (awesomequality@gmail.com)
#
### Attributes:
#
#   adr - address of mobile beacon (from Dashboard) for data filtering. If it is None, every read data will be appended to buffer.
#       default: None
#
#   tty - serial port device name (physical or USB/virtual). It should be provided as an argument: 
#       /dev/ttyACM0 - typical for Linux / Raspberry Pi
#       /dev/tty.usbmodem1451 - typical for Mac OS X
#
#   baud - baudrate. Should be match to baudrate of hedgehog-beacon
#       default: 9600
#
#   maxvaluescount - maximum count of measurements of coordinates stored in buffer
#       default: 3
#
#   valuesUltrasoundPosition - buffer of measurements
#
#   debug - debug flag which activate console output    
#       default: False
#
#   pause - pause flag. If True, class would not read serial data
#
#   terminationRequired - If True, thread would exit from main loop and stop
#
#
### Methods:
#
#   __init__ (self, tty="/dev/ttyACM0", baud=9600, maxvaluescount=3, debug=False) 
#       constructor
#
#   print_position(self)
#       print last measured data in default format
#
#   position(self)
#       return last measured data in array [x, y, z, timestamp]
#
#   distances(self)
#       return raw distances in array [hedge, beacon0, distance0, beacon1, distance1, beacon2, distance2, beacon3, distance3, timestamp]
#
#   raw_imu(self)
#       return raw IMU (accelerometer, gyro, magnetometer) data in array [AX,AY,AZ, GX,GY,GZ, MX,MY,MZ, timestamp] 
#
#   imu_fusion(self)
#       return IMU fusion (position, quaternion, velocity, acceleration) data in array [X,Y,Z, QW,QX,QY,QZ, VX,VY,VZ, AX,AY,AZ, timestamp] 
#
#   telemetry(self)
#       return telemetry (battery voltage, RSSI) data in array [vbat, RSSI] 
#
#   quality(self)
#       return location quality data in array [address, quality] 
#
#   waypoint(self)
#       return last received waypoint in array [type, index, total_number, param1, param2, param3] 
#
#   stop(self)
#       stop infinite loop and close port
#
#
###

###
# Changes:
# lastValues -> valuesUltrasoundPosition
# recieveLinearDataCallback -> recieveUltrasoundPositionCallback
# lastImuValues -> valuesImuRawData
# recieveAccelerometerDataCallback -> recieveImuRawDataCallback
# mm and cm -> m
###

import serial
import struct
import collections
import time
from threading import Thread
from threading import Event
import math

CRC16_TABLE = (
0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040 )

def _crc16(arr, offset, size, table):
	if (len(arr)<offset+size):
		return -1
	n= 0
	wCRCWord = 0xFFFF
	while (n<size):
		nTemp = arr[n+offset] ^ wCRCWord
		wCRCWord = wCRCWord >> 8
		wCRCWord  = wCRCWord ^ table[nTemp & 0xff]
		n= n+1
	return wCRCWord

# Modbus RTU CRC-16
def crc16_mb(arr, offset, size):
	return _crc16(arr, offset, size, CRC16_TABLE)
	

class MarvelmindHedge (Thread):
    def __init__ (self, adr=None, tty="/dev/ttyACM0", baud=9600, maxvaluescount=3, debug=False, recieveUltrasoundPositionCallback=None, recieveImuRawDataCallback=None, recieveImuDataCallback=None, recieveUltrasoundRawDataCallback=None):
        self.tty = tty  # serial
        self.baud = baud  # baudrate
        self.debug = debug  # debug flag
        self._bufferSerialDeque = collections.deque(maxlen=255)  # serial buffer
        self._bufferSerialReply= bytearray(6)

        self.valuesUltrasoundPosition = collections.deque([[0]*6]*maxvaluescount, maxlen=maxvaluescount) # ultrasound position buffer
        self.recieveUltrasoundPositionCallback = recieveUltrasoundPositionCallback
        
        self.valuesImuRawData = collections.deque([[0]*10]*maxvaluescount, maxlen=maxvaluescount) # raw imu data buffer
        self.recieveImuRawDataCallback = recieveImuRawDataCallback

        self.valuesImuData = collections.deque([[0]*14]*maxvaluescount, maxlen=maxvaluescount) # processed imu data buffer
        self.recieveImuDataCallback = recieveImuDataCallback

        self.valuesUltrasoundRawData = collections.deque([[0]*5]*maxvaluescount, maxlen=maxvaluescount)
        self.recieveUltrasoundRawDataCallback = recieveUltrasoundRawDataCallback
        
        self.valuesTelemetryData = collections.deque([[0]*5]*maxvaluescount, maxlen=maxvaluescount)
        self.valuesQualityData = collections.deque([[0]*5]*maxvaluescount, maxlen=maxvaluescount)
        self.valuesWaypointData = collections.deque([[0]*16]*maxvaluescount, maxlen=maxvaluescount)


        self.pause = False
        self.terminationRequired = False

        self.positionUpdated= False;        
        self.distancesUpdated= False;
        self.rawImuUpdated= False;
        self.fusionImuUpdated= False;
        self.telemetryUpdated= False
        self.qualityUpdated= False
        self.waypointsUpdated= False
        
        self.adr = adr
        self.serialPort = None
        
        self.dataEvent= Event()
        Thread.__init__(self)

    def print_position(self):
        if (isinstance(self.position()[1], int)):
            print ("Hedge {:d}: X: {:d} m, Y: {:d} m, Z: {:d} m, Angle: {:d} at time T: {:.3f}".format(self.position()[0], self.position()[1], self.position()[2], self.position()[3], self.position()[4], self.position()[5]/1000.0))
        else:
            print ("Hedge {:d}: X: {:.3f}, Y: {:.3f}, Z: {:.3f}, Angle: {:d} at time T: {:.3f}".format(self.position()[0], self.position()[1], self.position()[2], self.position()[3], self.position()[4], self.position()[5]/1000.0))

    def position(self):
        self.positionUpdated= False
        return list(self.valuesUltrasoundPosition)[-1];
           
    def print_distances(self): 
        self.distancesUpdated= False
        rd= self.distances()
        print ("Distances: From:H{:d} to  B{:d}:{:.3f}, B{:d}:{:.3f}, B{:d}:{:.3f}, B{:d}:{:.3f}   at time T: {:.3f}".format(rd[0], rd[1], rd[2], rd[3], rd[4], rd[5], rd[6], rd[7], rd[8], rd[9]/1000.0))
        
    def distances(self):
        return list(self.valuesUltrasoundRawData)[-1];
        
    def print_raw_imu(self): 
        self.rawImuUpdated= False
        ri= self.raw_imu()
        print ("Raw IMU: AX:{:d}, AY:{:d}, AZ:{:d},   GX:{:d}, GY:{:d}, GZ:{:d},   MX:{:d}, MY:{:d}, MZ:{:d},   at time T: {:.3f}".format(ri[0], ri[1], ri[2], ri[3], ri[4], ri[5], ri[6], ri[7], ri[8], ri[9]/1000.0))
        
    def raw_imu(self):
        return list(self.valuesImuRawData)[-1];
        
    def print_imu_fusion(self): 
        self.fusionImuUpdated= False
        ifd= self.imu_fusion()
        print ("IMU fusion: X:{:.3f}, Y:{:.3f}, Z:{:.3f},   QW:{:.3f}, QX:{:.3f}, QY:{:.3f}, QZ:{:.3f},   VX:{:.3f}, VY:{:.3f}, VZ:{:.3f},   AX:{:.3f}, AY:{:.3f}, AZ:{:.3f},   at time T: {:.3f}".format(ifd[0], ifd[1], ifd[2], ifd[3], ifd[4], ifd[5], ifd[6], ifd[7], ifd[8], ifd[9], ifd[10], ifd[11], ifd[12], ifd[13]/1000.0))
        
    def imu_fusion(self):
        return list(self.valuesImuData)[-1];
        
    def print_telemetry(self): 
        self.telemetryUpdated= False
        td= self.telemetry()
        print ("Telemetry: Vbat: {:.3f}V, RSSI: {:d}dBm".format(td[0]/1000.0, td[1]))
        
    def telemetry(self):
        return list(self.valuesTelemetryData)[-1];
        
    def print_quality(self): 
        self.qualityUpdated= False
        qd= self.quality()
        print ("Quality: Address: {:d}, Quality: {:d}%".format(qd[0], qd[1]))
        
    def quality(self):
        return list(self.valuesQualityData)[-1];
        
    def print_waypoint(self): 
        self.waypointsUpdated= False
        wd= self.waypoint()
        print ("Movement: Type: {:d}, Index: {:d}, Total: {:d},  Param1: {:d}, Param2: {:d}, Param3: {:d}".format(wd[0], wd[1], wd[2], wd[3], wd[4], wd[5]))
        
    def waypoint(self):
        return list(self.valuesWaypointData)[-1];
        
    def replyWaypointRcvSuccess(self):
        if (self.adr is None):
            return
        self._bufferSerialReply[0]= self.adr
        self._bufferSerialReply[1]= 0x4a
        self._bufferSerialReply[2]= 0x01
        self._bufferSerialReply[3]= 0x02
		
        CRC_calcReply= crc16_mb(self._bufferSerialReply, 0, 4)
        self._bufferSerialReply[4]= CRC_calcReply & 0xff
        self._bufferSerialReply[5]= (CRC_calcReply>>8) & 0xff
        self.serialPort.write(self._bufferSerialReply)
    
    def stop(self):
        self.terminationRequired = True
        print ("stopping")

    def run(self):      
        while (not self.terminationRequired):
            if (not self.pause):
                try:
                    if (self.serialPort is None):
                        print ("Trying open serial port: {:s}".format(self.tty))
                        self.serialPort = serial.Serial(self.tty, self.baud, timeout=3)
                        print ("Serial port opened")
                    readChar = self.serialPort.read(1)
                    while (readChar is not None) and (readChar is not '') and (not self.terminationRequired):
                        self._bufferSerialDeque.append(readChar)
                        readChar = self.serialPort.read(1)
                        bufferList = list(self._bufferSerialDeque)
                        
                        strbuf = (b''.join(bufferList))

                        pktHdrOffset = strbuf.find(b'\xff\x47')
                        if (pktHdrOffset == -1):
                            pktHdrOffset = strbuf.find(b'\xff\x4a')
                        if (pktHdrOffset >= 0 and len(bufferList) > pktHdrOffset + 4 and pktHdrOffset<220):
#                           print(bufferList)
                            isMmMessageDetected = False
                            isCmMessageDetected = False
                            isRawImuMessageDetected = False
                            isImuMessageDetected = False
                            isDistancesMessageDetected = False
                            isTelemetryMessageDetected= False
                            isQualityMessageDetected= False
                            isWaypointsMessageDetected= False
                            pktHdrOffsetCm = strbuf.find(b'\xff\x47\x01\x00')
                            if (pktHdrOffsetCm == -1):
                                pktHdrOffsetMm = strbuf.find(b'\xff\x47\x11\x00')
                                if (pktHdrOffsetMm == -1):
                                    pktHdrOffsetRawImu = strbuf.find(b'\xff\x47\x03\x00')
                                    if (pktHdrOffsetRawImu == -1):
                                        pktHdrOffsetDistances = strbuf.find(b'\xff\x47\x04\x00')
                                        if (pktHdrOffsetDistances == -1):
                                            pktHdrOffsetImu = strbuf.find(b'\xff\x47\x05\x00')
                                            if (pktHdrOffsetImu == -1):
                                                pktHdrOffsetTelemetry = strbuf.find(b'\xff\x47\x06\x00')
                                                if (pktHdrOffsetTelemetry == -1):
                                                    pktHdrOffsetQuality= strbuf.find(b'\xff\x47\x07\x00')
                                                    if (pktHdrOffsetQuality == -1):
                                                        pktHdrOffsetWaypoints= strbuf.find(b'\xff\x4a\x01\x02')
                                                        if (pktHdrOffsetWaypoints != -1):
                                                            isWaypointsMessageDetected= True
                                                            if (self.debug): print ('Message with waypoints data was detected')
                                                    else:
                                                        isQualityMessageDetected= True
                                                        if (self.debug): print ('Message with quality data was detected')
                                                else:
                                                    isTelemetryMessageDetected= True
                                                    if (self.debug): print ('Message with telemetry data was detected')
                                            else:
                                                isImuMessageDetected = True
                                                if (self.debug): print ('Message with processed IMU data was detected')
                                        else:
                                            isDistancesMessageDetected = True
                                            if (self.debug): print ('Message with distances was detected') 
                                    else:
                                        isRawImuMessageDetected = True
                                        if (self.debug): print ('Message with raw IMU data was detected') 
                                else:
                                    isMmMessageDetected = True
                                    if (self.debug): print ('Message with US-position(mm) was detected')
                            else:
                                isCmMessageDetected = True
                                if (self.debug): print ('Message with US-position(cm) was detected') 								     
												
                            msgLen = ord(bufferList[pktHdrOffset + 4])
                            if (self.debug): print ('Message length: ', msgLen)
                            
                            self.dataEvent.set()

                            try:
                                if (len(bufferList) > pktHdrOffset + 4 + msgLen + 2):
                                    usnCRC16 = 0
                                    if (isCmMessageDetected):
                                        usnTimestamp, usnX, usnY, usnZ, usnAdr, usnAngle, usnCRC16 = struct.unpack_from ('<LhhhxBhxxH', strbuf, pktHdrOffset + 5)
                                        usnX = usnX/100.0
                                        usnY = usnY/100.0
                                        usnZ = usnZ/100.0
                                        usnAngle = 0b0000111111111111&usnAngle
                                    elif (isMmMessageDetected):
                                        usnTimestamp, usnX, usnY, usnZ, usnFlags, usnAdr, usnAngle, usnCRC16 = struct.unpack_from ('<LlllBBhxxH', strbuf, pktHdrOffset + 5)
                                        usnX = usnX/1000.0
                                        usnY = usnY/1000.0
                                        usnZ = usnZ/1000.0
                                        usnAngle = 0b0000111111111111&usnAngle
                                        if ((usnFlags&0x40)==0):
                                            self.adr= usnAdr
                                    elif (isRawImuMessageDetected):
                                        ax, ay, az, gx, gy, gz, mx, my, mz, timestamp, usnCRC16 = struct.unpack_from ('<hhhhhhhhhxxxxxxLxxxxH', strbuf, pktHdrOffset + 5)
                                    elif (isImuMessageDetected):
                                        x, y, z, qw, qx, qy, qz, vx, vy, vz, ax, ay, az, timestamp, usnCRC16 = struct.unpack_from ('<lllhhhhhhhhhhxxLxxxxH', strbuf, pktHdrOffset + 5)
                                    elif (isDistancesMessageDetected):
                                        HedgeAdr, b1, b1d, b2, b2d, b3, b3d, b4, b4d, timestamp,usnCRC16 = struct.unpack_from ('<BBlxBlxBlxBlxLxxxH', strbuf, pktHdrOffset + 5)
                                    elif (isTelemetryMessageDetected):
                                        vbat, rssi_dbm, usnCRC16 = struct.unpack_from ('<HbxxxxxxxxxxxxxH', strbuf, pktHdrOffset + 5)
                                    elif (isQualityMessageDetected):
                                        quality_addr, quality_per, usnCRC16 = struct.unpack_from ('<BBxxxxxxxxxxxxxxH', strbuf, pktHdrOffset + 5)
                                    elif (isWaypointsMessageDetected):
                                        mvmType, mvmIndex, mvmTotal, mvmParam1, mvmParam2, mvmParam3, usnCRC16 = struct.unpack_from ('<BBBhhhxxxH', strbuf, pktHdrOffset + 5)

                                    CRC_calc = crc16_mb(bytearray(strbuf), pktHdrOffset, msgLen+5)

                                    if CRC_calc == usnCRC16:
                                        if (isMmMessageDetected or isCmMessageDetected):
                                            self.positionUpdated= True
                                            value = [usnAdr, usnX, usnY, usnZ, usnAngle, usnTimestamp]
                                            if (self.adr == usnAdr or self.adr is None):
                                                self.valuesUltrasoundPosition.append(value)
                                                if (self.recieveUltrasoundPositionCallback is not None):
                                                    self.recieveUltrasoundPositionCallback()
                                        elif (isRawImuMessageDetected):
                                            self.rawImuUpdated= True
                                            value = [ax, ay, az, gx, gy, gz, mx, my, mz, timestamp]
                                            self.valuesImuRawData.append(value)
                                            if (self.recieveImuRawDataCallback is not None):
                                                self.recieveImuRawDataCallback()
                                        elif (isDistancesMessageDetected):
                                            value = [HedgeAdr, b1, b1d/1000.0, b2, b2d/1000.0, b3, b3d/1000.0, b4, b4d/1000.0, timestamp]
                                            self.valuesUltrasoundRawData.append(value)
                                            self.distancesUpdated= True
                                            if (self.recieveUltrasoundRawDataCallback is not None):
                                                self.recieveUltrasoundRawDataCallback()
                                        elif (isImuMessageDetected):
                                            self.fusionImuUpdated= True
                                            value = [x/1000.0, y/1000.0, z/1000.0, qw/10000.0, qx/10000.0, qy/10000.0, qz/10000.0, vx/1000.0, vy/1000.0, vz/1000.0, ax/1000.0,ay/1000.0,az/1000.0, timestamp]
                                            self.valuesImuData.append(value)
                                            if (self.recieveImuDataCallback is not None):
                                                self.recieveImuDataCallback()
                                        elif (isTelemetryMessageDetected):
                                            self.telemetryUpdated= True  
                                            value = [vbat, rssi_dbm]   
                                            self.valuesTelemetryData.append(value)  
                                        elif (isQualityMessageDetected):
                                            self.qualityUpdated= True  
                                            value = [quality_addr, quality_per] 
                                            self.valuesQualityData.append(value) 
                                        elif (isWaypointsMessageDetected):
                                            self.waypointsUpdated= True   
                                            value = [mvmType, mvmIndex, mvmTotal, mvmParam1, mvmParam2, mvmParam3] 
                                            self.valuesWaypointData.append(value) 
                                            self.replyWaypointRcvSuccess()                    
                                    else:
                                        if self.debug:
                                            print ('\n*** CRC ERROR')

                                    if pktHdrOffset == -1:
                                        if self.debug:
                                            print ('\n*** ERROR: Marvelmind USNAV beacon packet header not found (check modem board or radio link)')
                                        continue
                                    elif pktHdrOffset >= 0:
                                        if self.debug:
                                            print ('\n>> Found USNAV beacon packet header at offset %d' % pktHdrOffset)
                                    for x in range(0, pktHdrOffset + msgLen + 7):
                                        self._bufferSerialDeque.popleft()
                            except struct.error:
                                print ('smth wrong')
                except OSError:
                    print ('*** ERROR: OS error (possibly serial port is not available)')
                    time.sleep(3)
                except serial.SerialException:
                    print ('*** ERROR: serial port error (possibly beacon is reset, powered down or in sleep mode). Restarting reading process...')
                    self.serialPort = None
                    time.sleep(3)
            else: 
                time.sleep(1)
    
        if (self.serialPort is not None):
            self.serialPort.close()
