## DroneKit tutorial


In this tutorial, [DroneKit](https://github.com/dronekit/dronekit-python) will be used to control a vehicle, specifically by connecting to a simulated APM2 autopilot, from a companion computer (e.g. Raspberry PI, common box, etc.). The steps described in here are almost exactly as it will be done with a real vehicle. 

To execute Python commands please use a Python console, a script or a Jupyter notebook 

#### ArduPilot Software In the Loop (SITL)
---

Start ArduPilot SITL with the following command:

```
sim_vehicle.py -v <vehicleType> --map --console --out <IP>:<PORT> -l <location> 
where
    <vehicleType> = <ArduCopter|AntennaTracker|APMrover2|ArduSub|ArduPlane>
    <location> = latitude,longitude,absolute-altitude,heading (optional)
    
    
Example: 

sim_vehicle.py -v ArduCopter --map --console --out 127.0.0.1:14550 --out 127.0.0.1:14551 -l 2.148971,-73.944397,0,270

```
The following sections asume the usage of the previous example


#### Connect to the vehicle 
---

Use connect function from dronekit

```
from dronekit import connect
vehicle = connect('127.0.0.1:14550',wait_ready = True)
```

The connect function returns an object of class _dronekit.Vehicle_, which can be used to interact with the autopilot

#### Check the vehicle is armable
---

Before attempting arming, check the vehicle is armable, i.e. it has passed pre-arming tests, e.g. compass calibration, etc

```
print(vehicle.is_armable)
```

If True, the vehicle can be armed

#### Arm the vehicle
---

To proceed with arming, the vehicle is to be set in GUIDED mode previously

```
from dronekit import VehicleMode
vehicle.mode = VehicleMode('GUIDED')
vehicle.armed = True
```

#### Take off
---

This instructions apply for flying vehicles e.g. ArduCopter, etc. Set TARGET_ALTITUDE to desired value 

```
vehicle.simple_takeoff(TARGET_ALTITUDE)
```

If using Python's interactive console execute arming and taking off instruction in one line, i.e.

```
vehicle.armed = True; vehicle.simple_takeoff(TARGET_ALTITUDE)
```

#### Checking vehicle altitude
---

To check the vehicle's current altitude, use its location.global_relative_frame.alt, for example

```
while True:
  if abs(TARGET_ALTITUDE - vehicle.location.global_relative_frame.alt) <= 1:
    print('Selected altitude was reached')
    break
  else:
    print('Current altitude: '+ str(vehicle.location.global_relative_frame.alt))
  
```

#### Check which information provides the vehicle object

Code taken from [DroneKit Vehicle state example](https://github.com/dronekit/dronekit-python/blob/master/examples/vehicle_state/vehicle_state.py)

```
print("\nGet all vehicle attribute values:")
print(" Autopilot Firmware version: %s" % vehicle.version)
print("   Major version number: %s" % vehicle.version.major)
print("   Minor version number: %s" % vehicle.version.minor)
print("   Patch version number: %s" % vehicle.version.patch)
print("   Release type: %s" % vehicle.version.release_type())
print("   Release version: %s" % vehicle.version.release_version())
print("   Stable release?: %s" % vehicle.version.is_stable())
print(" Autopilot capabilities")
print("   Supports MISSION_FLOAT message type: %s" % vehicle.capabilities.mission_float)
print("   Supports PARAM_FLOAT message type: %s" % vehicle.capabilities.param_float)
print("   Supports MISSION_INT message type: %s" % vehicle.capabilities.mission_int)
print("   Supports COMMAND_INT message type: %s" % vehicle.capabilities.command_int)
print("   Supports PARAM_UNION message type: %s" % vehicle.capabilities.param_union)
print("   Supports ftp for file transfers: %s" % vehicle.capabilities.ftp)
print("   Supports commanding attitude offboard: %s" % vehicle.capabilities.set_attitude_target)
print("   Supports commanding position and velocity targets in local NED frame: %s" % vehicle.capabilities.set_attitude_target_local_ned)
print("   Supports set position + velocity targets in global scaled integers: %s" % vehicle.capabilities.set_altitude_target_global_int)
print("   Supports terrain protocol / data handling: %s" % vehicle.capabilities.terrain)
print("   Supports direct actuator control: %s" % vehicle.capabilities.set_actuator_target)
print("   Supports the flight termination command: %s" % vehicle.capabilities.flight_termination)
print("   Supports mission_float message type: %s" % vehicle.capabilities.mission_float)
print("   Supports onboard compass calibration: %s" % vehicle.capabilities.compass_calibration)
print(" Global Location: %s" % vehicle.location.global_frame)
print(" Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
print(" Local Location: %s" % vehicle.location.local_frame)
print(" Attitude: %s" % vehicle.attitude)
print(" Velocity: %s" % vehicle.velocity)
print(" GPS: %s" % vehicle.gps_0)
print(" Gimbal status: %s" % vehicle.gimbal)
print(" Battery: %s" % vehicle.battery)
print(" EKF OK?: %s" % vehicle.ekf_ok)
print(" Last Heartbeat: %s" % vehicle.last_heartbeat)
print(" Rangefinder: %s" % vehicle.rangefinder)
print(" Rangefinder distance: %s" % vehicle.rangefinder.distance)
print(" Rangefinder voltage: %s" % vehicle.rangefinder.voltage)
print(" Heading: %s" % vehicle.heading)
print(" Is Armable?: %s" % vehicle.is_armable)
print(" System status: %s" % vehicle.system_status.state)
print(" Groundspeed: %s" % vehicle.groundspeed)    # settable
print(" Airspeed: %s" % vehicle.airspeed)    # settable
print(" Mode: %s" % vehicle.mode.name)    # settable
print(" Armed: %s" % vehicle.armed)    # settable
```

#### Return home
---

To return home and land (in case of flying vehicles) set the mode to RTL

```
vehicle.mode = VehicleMode("RTL")
```

#### Move to selected location
---

Using the class _LocationGlobalRelative_ is possible to command the vehicle to move to a particular location, for example:

```
from dronekit import LocationGlobalRelative
location = LocationGlobalRelative(2.148970999989359, -73.94421721134503, TARGET_ALTITUDE)
vehicle.simple_goto(location) ## With default (autopilot) groundspeed and airspeed
vehicle.simple_goto(location, groundspeed=10) ## With groundspeed of 10 [m/s]
vehicle.simple_goto(location, airspeed=10) ## With airspeed of 10 [m/s]
```




