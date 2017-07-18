[ control info ]
control_cycle = 8   # milliseconds

[ port info ]
# PORT NAME  | BAUDRATE   | DEFAULT JOINT
/dev/ttyUSB0 | 1000000    | r_sho_pitch
/dev/ttyUSB1 | 1000000    | cm-740

[ device info ]
# TYPE    | PORT NAME    | ID  | MODEL          | PROTOCOL | DEV NAME      | BULK READ ITEMS
dynamixel | /dev/ttyUSB0 | 1   | MX-28          | 1.0      | r_sho_pitch   | present_position
dynamixel | /dev/ttyUSB0 | 2   | MX-28          | 1.0      | l_sho_pitch   | present_position
dynamixel | /dev/ttyUSB0 | 3   | MX-28          | 1.0      | r_sho_roll    | present_position
dynamixel | /dev/ttyUSB0 | 4   | MX-28          | 1.0      | l_sho_roll    | present_position
dynamixel | /dev/ttyUSB0 | 5   | MX-28          | 1.0      | r_el          | present_position
dynamixel | /dev/ttyUSB0 | 6   | MX-28          | 1.0      | l_el          | present_position
dynamixel | /dev/ttyUSB0 | 7   | MX-28          | 1.0      | r_hip_yaw     | present_position
dynamixel | /dev/ttyUSB0 | 8   | MX-28          | 1.0      | l_hip_yaw     | present_position
dynamixel | /dev/ttyUSB0 | 9   | MX-28          | 1.0      | r_hip_roll    | present_position
dynamixel | /dev/ttyUSB0 | 10  | MX-28          | 1.0      | l_hip_roll    | present_position
dynamixel | /dev/ttyUSB0 | 11  | MX-28          | 1.0      | r_hip_pitch   | present_position
dynamixel | /dev/ttyUSB0 | 12  | MX-28          | 1.0      | l_hip_pitch   | present_position
dynamixel | /dev/ttyUSB0 | 13  | MX-28          | 1.0      | r_knee        | present_position
dynamixel | /dev/ttyUSB0 | 14  | MX-28          | 1.0      | l_knee        | present_position
dynamixel | /dev/ttyUSB0 | 15  | MX-28          | 1.0      | r_ank_pitch   | present_position
dynamixel | /dev/ttyUSB0 | 16  | MX-28          | 1.0      | l_ank_pitch   | present_position
dynamixel | /dev/ttyUSB0 | 17  | MX-28          | 1.0      | r_ank_roll    | present_position
dynamixel | /dev/ttyUSB0 | 18  | MX-28          | 1.0      | l_ank_roll    | present_position
dynamixel | /dev/ttyUSB0 | 19  | MX-28          | 1.0      | head_pan      | present_position
dynamixel | /dev/ttyUSB0 | 20  | MX-28          | 1.0      | head_tilt     | present_position
sensor    | /dev/ttyUSB1 | 200 | CM-740         | 1.0      | cm-740        | button, present_voltage, gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z
