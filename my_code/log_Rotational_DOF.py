import math
from dronekit import connect, VehicleMode, Command
import time
from pymavlink import mavutil

def arm_and_takeoff( aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)

def send_velocity(vx, vy, vz):
    """
    Send velocity command to the vehicle.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        vx, vy, vz, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # Send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

vehicle = connect("127.0.0.1:14550", wait_ready=True)
takeoff_altitude = 10
arm_and_takeoff(takeoff_altitude)

i = 0
file_path = 'logs.txt'
with open(file_path, 'w') as file:
    while True:
        if i > 30:
            break

        curr_time = time.time()        
        send_velocity(15, 0, 0)
        pitch = (180/math.pi)*(vehicle.attitude.pitch)
        roll = (180/math.pi)*(vehicle.attitude.roll)
        yaw = (180/math.pi)*(vehicle.attitude.yaw)
        file.write(str(i) + '\t' + 'Pitch = ' + str(pitch) + '\t' + 'roll = ' + str(roll) + '\t' + 'yaw = ' + str(yaw) + '\t' + 'Time : ' + str(curr_time) + '\n')
        i += 1
        print(i)
        time.sleep(1)

vehicle.mode = VehicleMode("RTL")