try:
    import time
    from dronekit import connect, VehicleMode

    # connect to the Vehicle.
    print "Connecting to vehicle on: /dev/ttyS0"
    vehicle = connect("/dev/ttyS0", wait_ready=True)

    vehicle.channels.overrides = {}

    # disable some pre arm checks: (needed for the drone to actually start)
    vehicle.parameters["ARMING_CHECK"] = 2 + 8 + 4096 + 16 + 32 + 64 + 128 + 256 + 512 + 2048 # (no compass)
    #vehicle.parameters["ARMING_CHECK"] = 2 + 16 + 32 + 64 + 128 + 256 + 512 + 2048 # (no compass, no gps)
    print vehicle.parameters["ARMING_CHECK"]

    # print all(?) vehicle attributes:
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





    print "Arming motors"
    vehicle.mode    = VehicleMode("ALT_HOLD")
    vehicle.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print " Waiting for arming..."
        time.sleep(1)

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    counter = 0
    while True:
        print "##################################################################"
        print "##################################################################"
        counter += 1
        print counter

        print vehicle.channels

        print " Ch1 (Roll): %s" % vehicle.channels["1"]
        print " Ch2 (Pitch): %s" % vehicle.channels["2"]
        print " Ch3 (Throttle): %s" % vehicle.channels["3"]
        print " Ch4 (Yaw): %s" % vehicle.channels["4"]

        if counter > 10:
            print "$$$$$$$$$$$$$$$$$$$"
            print "setting pitch!"
            print "$$$$$$$$$$$$$$$$$$$"
            vehicle.channels.overrides["2"] = 1600

        time.sleep(1)



    # Close vehicle object before exiting script
    vehicle.close()
except:
    print "clear all channel overrides"
    vehicle.channels.overrides = {}
