import sys
import unitree_arm_interface
import time
import numpy as np
import serial
import datetime
import sys
import select

# command line flags
dummy = False
reverse_arms = False
time_dialation = 1 #Useful because sim doesn't run at 100% speed
if '--dummy' in sys.argv[1:]:
    print('Launching in DUMMY mode. use `dline [left arm pitch], [torso pitch], [right arm pitch]` to set angles')
    dummy = True
    time_dialation = 0.5

if '--reverse' in sys.argv[1:]:
    print('launching in arms reversed mode. Right operator arm will now drive left robot arm and vice-versa.')
    reverse_arms = True

np.set_printoptions(precision=3, suppress=True)

print("Press ctrl+\ to quit process.")

# neutral position
idle_pos = np.asarray([0.0,0.0,1.5,0.15,0.5,0.225])

# Go to set postitions for transition into/out of flat
def goto_forward():
    arm0.labelRun("forward")
    if not dummy:
        arm1.labelRun("forward")
def goto_flat():
    arm0.labelRun("startFlat")
    if not dummy:
        arm1.labelRun("startFlat")

#exponential moving average parameter
ema_alpha = 0.1

cartesian_cmds_moving_average = {True:np.zeros([7]), False:np.zeros([7])}

def control_loop_single(arm, arm_angle, offset_from_accel, strength, flip_y, log=False):
    # Determine goal position
    arm_influence = determine_arm_influence(arm_angle)
    target_pos = idle_pos + [0,0,0,0,-arm_influence*0.75,arm_influence]
    if flip_y:
        target_pos = target_pos * [-1,1,-1,1,-1,1]
    target_pos = target_pos + np.concatenate(([0,0,0], offset_from_accel))

    # Determine unsmoothed velocity
    offset_from_target = target_pos - get_end_posture(arm)
    cartesian_cmd_instantaneous = np.concatenate((offset_from_target * strength, [0]))

    # Calculate smoothed velocity and command arms
    cartesian_cmds_moving_average[flip_y] = \
        (cartesian_cmds_moving_average[flip_y] * (1-ema_alpha)) + \
        (cartesian_cmd_instantaneous * ema_alpha)

    arm.cartesianCtrlCmd(cartesian_cmds_moving_average[flip_y], 0.05, 1)
    
    if log:
        print('flip:', flip_y, ' arm_influence:', arm_influence, 'offset_from_accel:', offset_from_accel,
        end = ' ')

i = 0

# Control both arms
def control_loop_both(arm_angles, torso_accel, strength):
    global i
    log = i == 0
    offset_from_accel = -np.asarray(np.clip(torso_accel,-1,1)) / 5
    control_loop_single(arm0, arm_angles[0], offset_from_accel, strength, False, log)
    if not dummy:
        control_loop_single(arm1, arm_angles[1], offset_from_accel, strength, True, log)

    if log:
        print('')
    i = (i + 1)% 500

# Calculate the given arms end effector position
def get_end_posture(arm):
    return unitree_arm_interface.homoToPosture(
        arm._ctrlComp.armModel.forwardKinematics(
            arm.lowstate.getQ(), 6)
    )

# Formula for angle -> displacement calculation
def determine_arm_influence(angle):
    return -min(abs(angle) / 275, 0.3)

# arm config
# IF YOU GET AN ERROR ON THIS LINE, LOOK AT THE README!!!!!!!!!!
ARM_1_UDP = unitree_arm_interface.UDPPort("127.0.0.1", 8073, 8074, unitree_arm_interface.RECVSTATE_LENGTH, unitree_arm_interface.BlockYN.NO, 500000)
arm1 =  unitree_arm_interface.ArmInterface()
arm1._ctrlComp.udp = ARM_1_UDP
arm0 = unitree_arm_interface.ArmInterface()

# Activate both arms
arm1.loopOn()
arm0.loopOn()

# If we are not in DUMMY mode, activate serial
ser = None
if not dummy:
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0, rtscts=True)
serial_line_buffer = b''
most_recent_serial_line = "0, 0, 0, 0, 0"
most_recent_serial_time = datetime.datetime(year=1970,month=1,day=1)

# state machine
mode = "flat"

while True:
    try:
        # Handle incomming commands:
        if select.select([sys.stdin], [], [], 0.0)[0]: 
            command = sys.stdin.readline().strip()
            if command == mode:
                print("Already in mode ", mode)
            else:
                if command == "flat":
                    if mode != "flat":
                        goto_forward()
                        goto_flat()

                    mode = "flat"

                elif command == "idle":
                    if mode == "flat":
                        goto_forward()
                    mode = "idle"

                elif command == "run":
                    if mode != "idle":
                        print("Run mode must be accessed from idle mode")
                    else:
                        mode = "run"

                elif command == "report":
                    print("Most recent serial line:", most_recent_serial_line)

                elif 'dline' in command and dummy:
                    most_recent_serial_line = command[len('dline'):]
                    print('updated dummy serial to ', most_recent_serial_line)

                else:
                    print("Unrecognised command:", command)

        # Handle inbound serial:
        if dummy:
            most_recent_serial_time = datetime.datetime.now()
        elif ser.inWaiting():
            serial_line_buffer += ser.read(ser.inWaiting())
            if b'\n' in serial_line_buffer:
                lines = serial_line_buffer.split(b'\n')
                if len(lines) > 2:
                    serial_line_buffer = lines[-1]
                    most_recent_serial_line = lines[-2].decode('ascii').strip(', \r\n')
                    if most_recent_serial_line == "HOLD" and mode == "run":
                        print("Switching to idle due to HOLD command")
                        mode = "idle"
                    most_recent_serial_time = datetime.datetime.now()

        # Apply per cycle instructions if needed (run, idle):
        if mode == "run":
            if \
                (datetime.datetime.now() - most_recent_serial_time) > datetime.timedelta(seconds=0.1) \
                or most_recent_serial_line.count(',') != 4:

                print("Switching to idle due to loss of communication with microcontroller")
                mode = "idle"
            
            serial_split = [float(x) for x in most_recent_serial_line.split(",")]

            left_angle, right_angle = serial_split[0], serial_split[4]
            
            if reverse_arms:
                left_angle, right_angle = right_angle, left_angle
                
            control_loop_both([left_angle, right_angle], serial_split[1:4], 20)

        elif mode == "idle":
            control_loop_both([0, 0], [0, 0, -1], 2.5)
        time.sleep(arm0._ctrlComp.dt / time_dialation)
    except Exception as e:
        # Handle exceptions and continue control loop in idle
        print("Returning to idle due to exception:", e)
        mode = "idle"