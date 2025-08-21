import sys
import unitree_arm_interface
import time
import numpy as np
import serial
import datetime
import sys
import select

np.set_printoptions(precision=3, suppress=True)

print("Press ctrl+\ to quit process.")

idle_pos = np.asarray([-0.1,-0.4,-0.25,0.2,0.2,0.55])


def goto_forward():
    arm0.labelRun("forward")
    arm1.labelRun("forward")

def goto_flat():
    arm0.labelRun("startFlat")
    arm1.labelRun("startFlat")


def approach_z_offset_single(arm, desired_z_offset, strength, flip_y):
    flipped_idle_pos = idle_pos
    if flip_y:
        flipped_idle_pos = flipped_idle_pos * [1,1,1,1,-1,1]

    offset_from_target = (flipped_idle_pos + [0,0,0,0,0,desired_z_offset]) - get_end_posture(arm)
    vel_inc_gripper = np.concatenate((offset_from_target * strength, [0]))
    arm.cartesianCtrlCmd(vel_inc_gripper, 0.05, 1)

def approach_z_offset_both(desired_z_offsets, strength):
    approach_z_offset_single(arm0, desired_z_offsets[0], strength, False)
    approach_z_offset_single(arm1, desired_z_offsets[1], strength, True)


def get_end_posture(arm):
    return unitree_arm_interface.homoToPosture(
        arm._ctrlComp.armModel.forwardKinematics(
            arm.lowstate.getQ(), 6)
    )
  

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
            
            angle = float(most_recent_serial_line.split(",")[2])
            desired_z_offset = -min(abs(angle) / 600, 0.2)

            # print(desired_z_offset)
            
            approach_z_offset_both([-0.05, -0.05], 2.5)

        elif mode == "idle":
            approach_z_offset_both([0, 0], 5)

        time.sleep(1/500)
    except Exception as e:
        print("Returning to idle due to exception:", e)
        mode = "idle"