# Existing imports and setup
from pymavlink import mavutil
import time

baud_rate = 115200
port = 'COM5'

master = mavutil.mavlink_connection(port, baud=baud_rate)
master.wait_heartbeat()

def request_message(message_id):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0, message_id, 0, 0, 0, 0, 0, 0
    )

def set_home_location(lat, lon, alt=0):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        0, 0, 0, 0, 0, lat, lon, alt
    )
    print(f"Home location set: lat={lat}, lon={lon}, alt={alt}")

def set_home_to_current_location():
    print("Fetching current GPS location...")
    gps_msg = master.recv_match(type='GPS_RAW_INT', blocking=True, condition='fix_type>=3', timeout=10)
    if gps_msg:
        lat, lon, alt = gps_msg.lat / 1e7, gps_msg.lon / 1e7, gps_msg.alt / 1000
        set_home_location(lat, lon, alt)
        return (lat, lon, alt)
    else:
        raise Exception("Unable to obtain GPS fix.")

def send_loiter_waypoint(seq, lat, lon, loiter_time=30, alt=0):
    master.mav.mission_item_send(
        master.target_system,
        master.target_component,
        seq,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
        0, 1,
        loiter_time, 0, 0, 0,
        lat, lon, alt
    )

def upload_mission(waypoints, loiter_time=30):
    print("Uploading mission waypoints...")
    master.mav.mission_clear_all_send(master.target_system, master.target_component)
    time.sleep(1)
    master.mav.mission_count_send(master.target_system, master.target_component, len(waypoints))
    for seq, (name, coords) in enumerate(waypoints.items()):
        send_loiter_waypoint(seq, coords['lat'], coords['lon'], loiter_time, coords.get('alt', 0))
        print(f"Waypoint '{name}' sent: loiter {loiter_time}s at lat={coords['lat']}, lon={coords['lon']}")

def arm_vehicle():
    print("Arming vehicle...")
    master.arducopter_arm()
    master.motors_armed_wait()
    print("Vehicle armed.")

def disarm_vehicle():
    print("Disarming vehicle...")
    master.arducopter_disarm()
    master.motors_disarmed_wait()
    print("Vehicle disarmed.")

def set_mode(mode_name):
    if mode_name not in master.mode_mapping():
        raise Exception(f"Mode '{mode_name}' not found.")
    mode_id = master.mode_mapping()[mode_name]
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
    print(f"Mode set to '{mode_name}'.")

# ------------------- MAIN SCRIPT -------------------

current_loc = set_home_to_current_location()

waypoints = {
    "Waypoint_A": {"lat": 33.47379904133127, "lon": -88.79123624264602},
    "Waypoint_B": {"lat": 33.47385012053299, "lon": -88.7914942079917}
}

upload_mission(waypoints, loiter_time=60)  # Loiter 60 seconds per waypoint

arm_vehicle()
set_mode("AUTO")

master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_MISSION_START,
    0, 0, 0, 0, 0, 0, 0, 0
)
print("Mission started.")

# Estimate time needed (waypoints + loiter)
mission_duration = len(waypoints) * 90  # travel + loiter buffer
time.sleep(mission_duration)

set_mode("RTL")
print("Returning to home waypoint.")
time.sleep(300)  # 5 mins to return safely

disarm_vehicle()
