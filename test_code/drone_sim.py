import time

print "Start simulator (SITL)"
import dronekit_sitl
sitl = dronekit_sitl.start_default()
connection_string = sitl.connection_string()

# Import DroneKit-Python
from dronekit import connect, VehicleMode

# Connect to the Vehicle.
print("Connecting to vehicle on: %s" % (connection_string,))
vehicle = connect(connection_string, wait_ready=True)

# Get some vehicle attributes (state)
print "Get some vehicle attribute values:"
print " GPS: %s" % vehicle.gps_0
print " Battery: %s" % vehicle.battery
print " Last Heartbeat: %s" % vehicle.last_heartbeat
print " Is Armable?: %s" % vehicle.is_armable
print " System status: %s" % vehicle.system_status.state
print " Mode: %s" % vehicle.mode.name    # settable

print "##############"

print "Autopilot Firmware version: %s" % vehicle.version
print "Autopilot capabilities (supports ftp): %s" % vehicle.capabilities.ftp
print "Global Location: %s" % vehicle.location.global_frame
print "Global Location (relative altitude): %s" % vehicle.location.global_relative_frame
print "Local Location: %s" % vehicle.location.local_frame    #NED
print "Attitude: %s" % vehicle.attitude
print "Velocity: %s" % vehicle.velocity
print "GPS: %s" % vehicle.gps_0
print "Groundspeed: %s" % vehicle.groundspeed
print "Airspeed: %s" % vehicle.airspeed
print "Gimbal status: %s" % vehicle.gimbal
print "Battery: %s" % vehicle.battery
print "EKF OK?: %s" % vehicle.ekf_ok
print "Last Heartbeat: %s" % vehicle.last_heartbeat
print "Rangefinder: %s" % vehicle.rangefinder
print "Rangefinder distance: %s" % vehicle.rangefinder.distance
print "Rangefinder voltage: %s" % vehicle.rangefinder.voltage
print "Heading: %s" % vehicle.heading
print "Is Armable?: %s" % vehicle.is_armable
print "System status: %s" % vehicle.system_status.state
print "Mode: %s" % vehicle.mode.name    # settable
print "Armed: %s" % vehicle.armed    # settable

# def arm_and_takeoff(aTargetAltitude):
#     """
#     Arms vehicle and fly to aTargetAltitude.
#     """
#
#     print "Basic pre-arm checks"
#     # Don't try to arm until autopilot is ready
#     while not vehicle.is_armable:
#         print " Waiting for vehicle to initialise..."
#         time.sleep(1)
#
#     print "Arming motors"
#     # Copter should arm in GUIDED mode
#     vehicle.mode    = VehicleMode("GUIDED")
#     vehicle.armed   = True
#
#     # Confirm vehicle armed before attempting to take off
#     while not vehicle.armed:
#         print " Waiting for arming..."
#         time.sleep(1)
#
#     print "Taking off!"
#     vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude
#
#     # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
#     #  after Vehicle.simple_takeoff will execute immediately).
#     while True:
#         print " Altitude: ", vehicle.location.global_relative_frame.alt
#         #Break and return from function just below target altitude.
#         if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
#             print "Reached target altitude"
#             break
#         time.sleep(1)
#
# arm_and_takeoff(20)

# Close vehicle object before exiting script
vehicle.close()

# Shut down simulator
sitl.stop()
print("Completed")
