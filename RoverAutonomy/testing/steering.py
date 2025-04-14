from pymavlink import mavutil
import time

TURN_DELAY = .5 #seconds

# Constants for RC values
RC_NEUTRAL = 1500
RC_MIN = 1000
RC_MAX = 2000

# Connection parameters
PORT = "/dev/ttyACM0"  # Adjust as per your CubePilot's port
BAUDRATE = 115200

 # Connect to the CubePilot Rover
print("Connecting to CubePilot Rover...")
master = mavutil.mavlink_connection(PORT, baud=BAUDRATE)

# Wait for heartbeat
print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"Connected to system {master.target_system}, component {master.target_component}")

def send_rc_command(left_motor, right_motor, propeller_1=1500, propeller_2=1500):
    """
    Sends RC override commands to the specified channels.
    """
    # Enforce throttle limits
    left_motor = max(min(left_motor, RC_MAX), RC_MIN)
    right_motor = max(min(right_motor, RC_MAX), RC_MIN)
    propeller_1 = max(min(propeller_1, RC_MAX), RC_MIN)
    propeller_2 = max(min(propeller_2, RC_MAX), RC_MIN)

    # Send RC override command
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        left_motor,  # Channel 1 - Left motor
        right_motor, # Channel 3 - Right motor
        propeller_1, # Channel 4 - Propeller 1
        0,           # Channel 5 - Unused
        propeller_2, # Channel 6 - Propeller 2
        0, 0, 0, 0   # Channels 7 to 10 - Unused
    )

def is_armed():
    msg = master.recv_match(type="HEARTBEAT", blocking=True)
    return msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

def arm_vehicle():
    print("Arming rover...")

    master.mav.param_set_send(
    master.target_system, master.target_component,
    b'ARMING_CHECK',
    float(0),
    mavutil.mavlink.MAV_PARAM_TYPE_INT32)

    master.arducopter_arm()
    time.sleep(1)
    while not is_armed():
        print("Waiting for arming...")
        time.sleep(1)
    send_rc_command(RC_NEUTRAL, RC_NEUTRAL)
    print("Rover armed!")

def disarm_vehicle():
    print("Disarming rover...")
    master.arducopter_disarm()
    time.sleep(1)
    while is_armed():
        print("Waiting for disarm...")
        time.sleep(1)
    print("Rover disarmed.")

def test():
#    for i in range(11, 21):
#        send_rc_command(i*100, 3100-i*100)
#        print(i*100)
#        time.sleep(2)

    send_rc_command(1800, 1200)
    time.sleep(5)

#    print("Forward")
#    send_rc_command(RC_NEUTRAL+200, RC_NEUTRAL+200)
#    time.sleep(2)
    
#    send_rc_command(RC_NEUTRAL, RC_NEUTRAL)
#    time.sleep(5)

#    print("Right")
#    send_rc_command(RC_NEUTRAL-200, RC_NEUTRAL+200)
#    time.sleep(2)

#    send_rc_command(RC_NEUTRAL, RC_NEUTRAL)
#    time.sleep(5)

  #  print("Left")
 #   send_rc_command(RC_NEUTRAL+200, RC_NEUTRAL-200)
#    time.sleep(2)

arm_vehicle()
time.sleep(5)
test()
disarm_vehicle()
