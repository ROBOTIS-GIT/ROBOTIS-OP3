[ control info ]
control_cycle = 8   # milliseconds

[ port info ]
# PORT NAME  | BAUDRATE  | DEFAULT JOINT
/dev/U2D2    | 2000000   | r_sho_pitch

[ device info ]
# TYPE    | PORT NAME    | ID  | MODEL          | PROTOCOL | DEV NAME       | BULK READ ITEMS
dynamixel | /dev/U2D2    | 1   | XM-430         | 2.0      | r_sho_pitch    | present_position
dynamixel | /dev/U2D2    | 2   | XM-430         | 2.0      | l_sho_pitch    | present_position
dynamixel | /dev/U2D2    | 3   | XM-430         | 2.0      | r_sho_roll     | present_position
dynamixel | /dev/U2D2    | 4   | XM-430         | 2.0      | l_sho_roll     | present_position
dynamixel | /dev/U2D2    | 5   | XM-430         | 2.0      | r_el           | present_position
dynamixel | /dev/U2D2    | 6   | XM-430         | 2.0      | l_el           | present_position
dynamixel | /dev/U2D2    | 7   | XM-430         | 2.0      | r_hip_yaw      | present_position
dynamixel | /dev/U2D2    | 8   | XM-430         | 2.0      | l_hip_yaw      | present_position
dynamixel | /dev/U2D2    | 9   | XM-430         | 2.0      | r_hip_roll     | present_position
dynamixel | /dev/U2D2    | 10  | XM-430         | 2.0      | l_hip_roll     | present_position
dynamixel | /dev/U2D2    | 11  | XM-430         | 2.0      | r_hip_pitch    | present_position
dynamixel | /dev/U2D2    | 12  | XM-430         | 2.0      | l_hip_pitch    | present_position
dynamixel | /dev/U2D2    | 13  | XM-430         | 2.0      | r_knee         | present_position
dynamixel | /dev/U2D2    | 14  | XM-430         | 2.0      | l_knee         | present_position
dynamixel | /dev/U2D2    | 15  | XM-430         | 2.0      | r_ank_pitch    | present_position
dynamixel | /dev/U2D2    | 16  | XM-430         | 2.0      | l_ank_pitch    | present_position
dynamixel | /dev/U2D2    | 17  | XM-430         | 2.0      | r_ank_roll     | present_position
dynamixel | /dev/U2D2    | 18  | XM-430         | 2.0      | l_ank_roll     | present_position
dynamixel | /dev/U2D2    | 19  | XM-430         | 2.0      | head_pan       | present_position
dynamixel | /dev/U2D2    | 20  | XM-430         | 2.0      | head_tilt      | present_position
sensor    | /dev/U2D2    | 200 | OPEN-CR        | 2.0      | open-cr        | button, present_voltage, gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, roll, pitch, yaw
