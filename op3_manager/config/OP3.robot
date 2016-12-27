[ port info ]
# PORT NAME  | BAUDRATE  | DEFAULT JOINT
/dev/ttyUSB1 | 2000000   | r_sho_pitch
/dev/ttyUSB0 | 1000000   | cm-740

[ device info ]
# TYPE    | PORT NAME    | ID  | MODEL          | PROTOCOL | DEV NAME       | BULK READ ITEMS
dynamixel | /dev/ttyUSB1 | 1   | XM-430         | 2.0      | r_sho_pitch    | present_position
dynamixel | /dev/ttyUSB1 | 2   | XM-430         | 2.0      | l_sho_pitch    | present_position
dynamixel | /dev/ttyUSB1 | 3   | XM-430         | 2.0      | r_sho_roll     | present_position
dynamixel | /dev/ttyUSB1 | 4   | XM-430         | 2.0      | l_sho_roll     | present_position
dynamixel | /dev/ttyUSB1 | 5   | XM-430         | 2.0      | r_el           | present_position
dynamixel | /dev/ttyUSB1 | 6   | XM-430         | 2.0      | l_el           | present_position
dynamixel | /dev/ttyUSB1 | 7   | XM-430         | 2.0      | r_hip_yaw      | present_position
dynamixel | /dev/ttyUSB1 | 8   | XM-430         | 2.0      | l_hip_yaw      | present_position
dynamixel | /dev/ttyUSB1 | 9   | XM-430         | 2.0      | r_hip_roll     | present_position
dynamixel | /dev/ttyUSB1 | 10  | XM-430         | 2.0      | l_hip_roll     | present_position
dynamixel | /dev/ttyUSB1 | 11  | XM-430         | 2.0      | r_hip_pitch    | present_position
dynamixel | /dev/ttyUSB1 | 12  | XM-430         | 2.0      | l_hip_pitch    | present_position
dynamixel | /dev/ttyUSB1 | 13  | XM-430         | 2.0      | r_knee         | present_position
dynamixel | /dev/ttyUSB1 | 14  | XM-430         | 2.0      | l_knee         | present_position
dynamixel | /dev/ttyUSB1 | 15  | XM-430         | 2.0      | r_ank_pitch    | present_position
dynamixel | /dev/ttyUSB1 | 16  | XM-430         | 2.0      | l_ank_pitch    | present_position
dynamixel | /dev/ttyUSB1 | 17  | XM-430         | 2.0      | r_ank_roll     | present_position
dynamixel | /dev/ttyUSB1 | 18  | XM-430         | 2.0      | l_ank_roll     | present_position
dynamixel | /dev/ttyUSB1 | 19  | XM-430         | 2.0      | head_pan       | present_position
dynamixel | /dev/ttyUSB1 | 20  | XM-430         | 2.0      | head_tilt      | present_position
sensor    | /dev/ttyUSB0 | 200 | CM-740         | 1.0      | cm-740         | button, gyro_z, gyro_y, gyro_x, acc_x, acc_y, acc_z, present_voltage  
