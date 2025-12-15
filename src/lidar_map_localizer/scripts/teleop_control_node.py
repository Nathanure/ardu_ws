#!/usr/bin/env python3
import time
from pymavlink import mavutil
from pynput import keyboard


# 1) CONNECT & HEARTBEAT

the_connection = mavutil.mavlink_connection('udpin:127.0.0.1:14550')
the_connection.wait_heartbeat()
print(f"Heartbeat from sys:{the_connection.target_system} comp:{the_connection.target_component}")

# 2) GUIDED MODE --> ARM --> TAKEOFF

the_connection.set_mode_apm('GUIDED')
print("Mode -> GUIDED")
the_connection.arducopter_arm()
print("Arming motors...")
time.sleep(2)

takeoff_alt = 1.0
the_connection.mav.command_long_send(
    the_connection.target_system,
    the_connection.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,        # confirm
    0,0,0,0,  # unused
    0,0,      # lat, lon ignored
    takeoff_alt
)
print(f"Taking off to {takeoff_alt} m...")
time.sleep(5)

# 3) VELOCITY + YAW_RATE HOLD (mask=1479) 

MASK_VEL_YAWRATE = 1479
FRAME_BODY_OFFSET_NED = mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED  # 9

def set_velocity_body(vx, vy, vz, yaw_rate=0):
    """
    Send body-frame (forward, right, down) velocity.
    vx: forward (m/s)
    vy: right (m/s)
    vz: down (m/s)
    yaw_rate: (rad/s)
    """
    the_connection.mav.set_position_target_local_ned_send(
        0,
        the_connection.target_system,
        the_connection.target_component,
        FRAME_BODY_OFFSET_NED,
        MASK_VEL_YAWRATE,
        0,0,0,        # ignore position
        vx, vy, vz,   # desired m/s
        0,0,0,        # ignore acceleration
        0,            # yaw ignored
        yaw_rate      # yaw_rate in rad/s
    )

# 4) KEYBOARD LISTENER

# Velocity increments per key press
speed_increment = 1.0    # m/s for linear movement
yaw_increment = 0.35     # rad/s for yaw (approximately 20 deg/s)

# Current velocity values
x_val = 0.0      # forward/backward
y_val = 0.0      # left/right strafe
z_val = 0.0      # up/down
yaw_val = 0.0    # yaw rotation

# Track last key to prevent repeats
last_key = None

def on_press(key):
    global x_val, y_val, z_val, yaw_val, last_key
    
    try:
        cmd = key.char.lower()
        
        # Prevent key repeat (only register once per press)
        if cmd == last_key:
            return
        last_key = cmd
        
        # Toggle/add velocities based on key press
        if   cmd == 'w': x_val += speed_increment   # Forward +1.0 m/s
        elif cmd == 's': x_val -= speed_increment   # Backward -1.0 m/s
        elif cmd == 'a': y_val -= speed_increment   # Left -1.0 m/s
        elif cmd == 'd': y_val += speed_increment   # Right +1.0 m/s
        elif cmd == 'z': z_val -= speed_increment   # Up +1.0 m/s
        elif cmd == 'x': z_val += speed_increment   # Down -1.0 m/s
        elif cmd == 'q': yaw_val -= yaw_increment   # Yaw Left
        elif cmd == 'e': yaw_val += yaw_increment   # Yaw Right
        elif cmd == 'r': # Reset all to zero
            x_val = 0.0
            y_val = 0.0
            z_val = 0.0
            yaw_val = 0.0
            print("*** RESET ALL VELOCITIES TO ZERO ***")
        
        print(f"X: {x_val:.2f}   Y: {y_val:.2f}   Z: {z_val:.2f}   Yaw: {yaw_val:.2f}")
        
    except AttributeError:
        pass

def on_release(key):
    global last_key
    # Clear last_key on release to allow same key to be pressed again
    try:
        if key.char.lower() == last_key:
            last_key = None
    except AttributeError:
        pass

listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

print("→ Ready: W/A/S/D = strafe, Z/X = down/up, Q/E = yaw, R = RESET. Ctrl-C to exit.")
print("→ Each key press adds/subtracts velocity. Press R to stop all movement.")

# 5) MAIN LOOP
try:
    while True:
        # Continuously send the current velocity commands
        # Note: z_val is negated because NED convention (down is positive)
        set_velocity_body(x_val, y_val, -z_val, yaw_val)
        
        time.sleep(0.05)  # Send at 20Hz

except KeyboardInterrupt:
    print("\nExiting...")
    set_velocity_body(0, 0, 0, 0)
    listener.stop()