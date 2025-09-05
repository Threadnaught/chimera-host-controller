import sys
import unitree_arm_interface
import time
import numpy as np
import serial
import datetime
import sys
import select

armState = unitree_arm_interface.ArmFSMState

np.set_printoptions(precision=2, suppress=True)

print("Press ctrl+\ to quit process.")

#roll, pitch, yaw, x, y, z
idle_pos = np.asarray([-0.1,-0.4,-0.25,0.25,0.3,0.4])

def goto_forward():
    arm0.loopOn()
    arm0.labelRun("forward")
    arm1.loopOn()
    arm1.labelRun("forward")


def goto_flat():
    arm0.loopOn()
    arm0.labelRun("startFlat")
    arm1.loopOn()
    arm1.labelRun("startFlat")

i = 0

#Low pass filter to avoid the shakes
joint_cmds_moving_average = {True:np.zeros([7]), False:np.zeros([7])}

arm_target_qs = {False:np.zeros([6]), True:np.zeros([6])}
arm_target_qds = {False:np.zeros([6]), True:np.zeros([6])}

def approach_z_offset_single(arm, desired_z_offset, strength, flip_y):
    global i, joint_cmds_moving_average
    flipped_idle_pos = idle_pos
    if flip_y:
        flipped_idle_pos = flipped_idle_pos * [-1,1,-1,1,-1,1]

    offset_from_target = (flipped_idle_pos + [0,0,0,0,0,desired_z_offset]) - get_end_posture(arm)

    unfiltered_target_qd = arm._ctrlComp.armModel.solveQP(
        offset_from_target * strength,
        arm.lowstate.getQ(),
        arm._ctrlComp.dt
    )

    new_target_qd = (arm_target_qds[flip_y] * 0.9925) + (unfiltered_target_qd * 0.0075)
    new_target_qd = np.clip(new_target_qd, -2, 2)

    qdd = (new_target_qd - arm_target_qds[flip_y]) / arm._ctrlComp.dt
    
    arm_target_qds[flip_y] = new_target_qd
    arm_target_qs[flip_y] += arm_target_qds[flip_y] * arm._ctrlComp.dt
    
    tau = arm._ctrlComp.armModel.inverseDynamics(arm.lowstate.getQ(), arm_target_qds[flip_y], qdd, np.zeros(6))
    
    arm.setArmCmd(arm_target_qs[flip_y], arm_target_qds[flip_y], tau)
    arm.sendRecv()

    i = (i+1) % 500
    if i == 0 or i == 1:
        print('flip:', flip_y, ' target offset:', offset_from_target,
        end = '\n' if flip_y else ' ')


def approach_z_offset_both(desired_z_offsets, strength):
    approach_z_offset_single(arm0, desired_z_offsets[0], strength, False)
    approach_z_offset_single(arm1, desired_z_offsets[1], strength, True)


def get_end_posture(arm):
    return unitree_arm_interface.homoToPosture(
        arm._ctrlComp.armModel.forwardKinematics(
            arm.lowstate.getQ(), 6)
    )
  
def determine_z_offset(angle):
    return -min(abs(angle) / 350, 0.3)

# arm config
ARM_1_UDP = unitree_arm_interface.UDPPort("127.0.0.1", 8073, 8074, unitree_arm_interface.RECVSTATE_LENGTH, unitree_arm_interface.BlockYN.NO, 500000)
arm1 =  unitree_arm_interface.ArmInterface()
arm1._ctrlComp.udp = ARM_1_UDP

arm0 = unitree_arm_interface.ArmInterface()

arm1.loopOn()
arm0.loopOn()

# serial config
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0, rtscts=True)
serial_line_buffer = b''
most_recent_serial_line = ""
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
                    arm_target_qs = {False:arm0.lowstate.getQ(), True:arm1.lowstate.getQ()}
                    arm0.setFsmLowcmd()
                    arm1.setFsmLowcmd()
                    mode = "idle"

                elif command == "run":
                    if mode != "idle":
                        print("Run mode must be accessed from idle mode")
                    else:
                        mode = "run"

                elif command == "report":
                    print("Most recent serial line:", most_recent_serial_line)

                else:
                    print("Unrecognised command:", command)

        # Handle inbound serial:
        if ser.inWaiting():
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

        # Apply per cycle instructions if needed:
        if mode == "run":
            if \
                (datetime.datetime.now() - most_recent_serial_time) > datetime.timedelta(seconds=0.1) \
                or most_recent_serial_line.count(',') != 2:

                print("Switching to idle due to loss of communication with microcontroller")
                mode = "idle"
            
            serial_split = [float(x) for x in most_recent_serial_line.split(",")]
            
            approach_z_offset_both([
                determine_z_offset(serial_split[0]),
                determine_z_offset(serial_split[2])
            ], 5)

        elif mode == "idle":
            approach_z_offset_both([0, 0], 2.5)

        time.sleep(1/500)
    except Exception as e:
        print("Returning to idle due to exception:", e)
        mode = "idle"