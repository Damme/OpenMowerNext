# Worx mainboard status, published by the worx_hardware ros2_control plugin.
std_msgs/Header header

bool link_ok                 # a board message arrived within link_timeout
bool emergency               # emergency latched (/worx/emergency, or lift with lift_emergency)
bool motors_enabled          # /worx/motors_enabled (motor_state below is the firmware's, SETSPEED even when disabled)
bool collision               # bump latched: forward blocked, blade off (see worx-hardware docs)
bool lift                    # Lift input active (last Digital report)
uint32 bumps                 # bumps detected since start; a change = new bump
string motor_state           # firmware motorState (MOTORREQ_ENABLE / _DISABLE / _SETSPEED)
string power_state           # firmware powerState (e.g. Charging)

float32 battery_voltage      # V
float32 battery_current      # A (charge current)
float32 battery_temperature  # degC
bool in_charger

int32 left_pwm_cmd           # last PWM sent
int32 right_pwm_cmd
int32 mow_pwm_cmd
bool manual_mow                # blade switched on by hand (/worx/manual_mow), runs at manual_mow_pwm
int32 manual_mow_pwm         # worx_hardware parameter manual_mow_pwm: blade PWM, the sign is the direction
int32 manual_mow_max_pwm     # limit for |manual_mow_pwm|
int32[3] motor_pwm           # reported by the board: left, right, mow
int32[3] motor_current       # reported by the board: left, right, mow
int32 mow_pulses             # blade pulses per report (~180 while mowing)
int32 board_emergency        # MotorPulse "Emergancy" (-1 = not reported)
int32 block_forward          # MotorPulse "BlockForward" (-1 = not reported)

# Digital inputs: raw value, and "active" after applying digital_inverted.
string[] digital_names
int32[] digital_raw
bool[] digital_active

string[] analog_names
int32[] analog_values
string[] boundary_names      # perimeter wire signal
int32[] boundary_values

uint64 rx_messages
uint64 rx_dropped
uint32 tick_glitches
