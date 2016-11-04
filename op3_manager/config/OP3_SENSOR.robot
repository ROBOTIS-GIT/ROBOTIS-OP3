[ port info ]
# PORT NAME  | BAUDRATE  | DEFAULT JOINT
/dev/ttyUSB0 | 1000000   | open-cr

[ device info ]
# TYPE    | PORT NAME    | ID  | MODEL          | PROTOCOL | DEV NAME       | BULK READ ITEMS
dynamixel | /dev/ttyUSB0 | 1   | XM-430         | 2.0      | head_pan       | present_position
dynamixel | /dev/ttyUSB0 | 2   | XM-430         | 2.0      | head_tilt      | present_position
sensor    | /dev/ttyUSB0 | 200 | OPEN-CR        | 2.0      | open-cr        | button, gyro_z, gyro_y, gyro_x, acc_x, acc_y, acc_z  