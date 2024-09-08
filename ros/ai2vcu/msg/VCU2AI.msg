bool    handshake_receive_bit

bool    res_go_signal

uint8   AS_OFF=1
uint8   AS_READY=2
uint8   AS_DRIVING=3
uint8   AS_EMERGENCY_BRAKE=4
uint8   AS_FINISHED=5
uint8   as_state

uint8   AMI_NOT_SELECTED=0
uint8   AMI_ACCELERATION=1
uint8   AMI_SKIDPAD=2
uint8   AMI_AUTOCROSS=3
uint8   AMI_TRACK_DRIVE=4
uint8   AMI_STATIC_INSPECTION_A=5
uint8   AMI_STATIC_INSPECTION_B=6
uint8   AMI_AUTONOMOUS_DEMO=7
uint8   ami_state

float32 steer_angle_deg

float32 brake_press_f_pct

float32 brake_press_r_pct

float32 fl_wheel_speed_rpm

float32 fr_wheel_speed_rpm

float32 rl_wheel_speed_rpm

float32 rr_wheel_speed_rpm

uint16  fl_pulse_count

uint16  fr_pulse_count

uint16  rl_pulse_count

uint16  rr_pulse_count
