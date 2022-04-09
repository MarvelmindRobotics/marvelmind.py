# marvelmind.py #

marvelmind.py includes small python class based on threading.Thread for receiving and parsing coordinates data from Marvelmind mobile beacon by USB/serial port.
example.py is an example of use.
Written by Alexander Rudykh.
Support: info@marvelmind.com

[Download](https://github.com/MarvelmindRobotics/marvelmind.py/archive/master.zip)

## Attributes: ##

**adr** - address of mobile beacon (from Dashboard) for data filtering. If it is None, every read data will be appended to buffer.

*Default value: None*


**tty** - serial port device name (physical or USB/virtual). It should be provided as an argument: 

  * /dev/ttyACM0 - typical for Linux

  * /dev/tty.usbmodem1451 - typical for Mac OS X


**baud** - baudrate. Should be match to baudrate of hedgehog-beacon

*Default value: 9600*


**maxvaluescount** - maximum count of measurements of coordinates stored in buffer

*Default value: 3*


**valuesUltrasoundPosition** - buffer of US measured data (address of device, x, y, z (meters), angle (1/10 degree), timestamp (seconds)): [adr, x, y, z, ang, timestamp]

**valuesImuRawData** - buffer of IMU raw measures (accelerometer, gyroscope, compass)

**valuesImuData** - buffer of IMU and US based measures (position, angular position (quaternion), velocities, accelerations): [x, y, z, qw, qx, qy, qz, vx, vy, vz, ax, ay, az, timestamp]


**debug** - debug flag which activate console output	

*Default value: False*


**pause** - pause flag. If True, class would not read serial data


**terminationRequired** - If True, thread would exit from main loop stop


## Methods: ##

**position(self)**
Return last measured data in array [x, y, z, timestamp]

**distances(self)**
Return raw distances in array [hedge, beacon0, distance0, beacon1, distance1, beacon2, distance2, beacon3, distance3, timestamp]

**raw_imu(self)**
Return raw IMU (accelerometer, gyro, magnetometer) data in array [AX,AY,AZ, GX,GY,GZ, MX,MY,MZ, timestamp] 

**imu_fusion(self)**
Return IMU fusion (position, quaternion, velocity, acceleration) data in array [X,Y,Z, QW,QX,QY,QZ, VX,VY,VZ, AX,AY,AZ, timestamp] 

**telemetry(self)**
Return telemetry (battery voltage, RSSI) data in array [vbat, RSSI] 

**quality(self)**
Return location quality data in array [address, quality] 

**waypoint(self)**
Return last received waypoint in array [type, index, total_number, param1, param2, param3] 


**run(self)**
Main loop

**stop(self)**
Request to stop main loop and close serial port

**print_position(self)**
Print last measured position in default format

**print_distances(self)**
Print last measured raw distances in default format

**print_raw_imu(self)**
Print last raw IMU data (accelerometer, gyro, magnetometer) in default format

**print_imu_fusion(self)**
Print last IMU fusion data (position, orientation quaternion, velocity, acceleration) in default format

**print_raw_imu(self)**
Print last raw IMU data (accelerometer, gyro, magnetometer) in default format

**print_telemetry(self)**
Print telemetry data (battery voltage, RSSI) in default format

**print_quality(self)**
Print positioning quality data (percents) in default format

**print_waypoint(self)**
Print last received waypoint in default format

