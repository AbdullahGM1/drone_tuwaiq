

# Inside the terminal for direct connect through the PX4 terminal -- PX4 terminal 
#  
# commander takeoff 
# commander arm
# commander disarm
# commander land

###############################################################################################################################################

# MavAPI - Interface for connection to the drone (Mavsdk) with my program! - Important 

import asyncio # to add the process Asynchronous with entire process to separating the connection the steps with other process 

from mavsdk import System  # Import the function for the drone system (API)

from mavsdk.mission import MissionItem, MissionPlan ## To get the Mission Item class & Mission Plan -> Dive more to the classes and there finctions!!!

# AsyncIO - (Synchronous & Asynchronous):
# Used for the mavlink for connect function to keep running in the background [Asynchronous] & [Synchronous] used for ccamera feed -- 

async def main():  # Define the async function , why do we use async function? 

    drone = System() # define the drone from the sdk 

    # Connecting to the PX4 (drone)

    print("Connecting to the drone .....")

    await drone.connect(system_address="udpin://0.0.0.0:14540")  #Connect using alternative port

    # or 
    # await drone.connect(system_address="udpin://0.0.0.0:14541")
    #

    print("Drone is connected")

    ##### Arming 

    print("Arming the drone")

    await drone.action.arm() ## Check the action class to see the functions (command+Click the action class) - Important to add arm first

    print ("The drone is armed")
    
    # Wait for vehicle to be ready for takeoff
    print("Waiting for vehicle to be ready for takeoff...")
    async for is_ready in drone.telemetry.armed():
        if is_ready:
            break
    
    # Check if vehicle is in correct mode
    print("Setting takeoff mode...")
    await drone.action.set_takeoff_altitude(10.0)

    #### Takeoff

    print("Taking off ...")

    await drone.action.takeoff()
    await asyncio.sleep(10) # To wait for the takeoff


    ## Assign home location (Getting the home Location)
    print("Getting ailtitude ht home location ...")
    home = await anext(drone.telemetry.home()) # We can take this as (0,0,0) values, we can add them values to them to move as below

    absolute_altitude = home.absolute_altitude_m
    absolute_latitude = home.latitude_deg
    absolute_longitude = home.longitude_deg

    flying_alt = absolute_altitude + 5.0 

    print(f"Flying Alt is : {flying_alt}m")

    ## Go to location (GPS) coordinates  (Understand the GPS coordinates [Latitiude] & [Longtitude])
    # Go to function takes (4) values - (latitude, longitude, altitude, & Yaw) 


    print("Going to target 1 location...")
    await drone.action.goto_location(absolute_latitude + 10 * 1e-5, absolute_longitude, flying_alt, 0) # Zero is the (Yaw)
    await asyncio.sleep(10) ## We need to add await to ensure that the task is completed!

    print("Going to target 2 location...")
    await drone.action.goto_location(absolute_latitude + 10 * 1e-5, absolute_longitude + 10 * 1e-5, flying_alt, 0)
    await asyncio.sleep(10)

    print("Going to target 3 location...")
    await drone.action.goto_location(absolute_latitude, absolute_longitude + 10 * 1e-5, flying_alt, 0)
    await asyncio.sleep(10)

    print("Going to target 4 location...")
    await drone.action.goto_location(absolute_latitude, absolute_longitude, flying_alt, 0) ## Going to home location
    await asyncio.sleep(10)

     ## Landing 
    print("Landing the drone")
    await drone.action.land()
    print("The drone is landed")

    await asyncio.sleep(15) #Waiting to start the mission 

     ## Mission (Sequence actions (series)) -> Upload Mission, Pause Mission, Start Mission, abd etc ... #Check the mission class!!!
     # The mission takes (alot) of verables.  + Check the image
     # For the mission uses relative alt 

     #First we need to to arm 

    print("Arming drone...")
    await drone.action.arm()
    await asyncio.sleep(5)
    print("Drone Armed")

    home = await anext(drone.telemetry.home()) # We don't need to use abs alit, mission uses relative alt!!! 

    absolute_latitude = home.latitude_deg
    absolute_longitude = home.longitude_deg

    flying_alt = 5.0
    print(f"Flight altitude is: {flying_alt}")

    # Define mission waypoints
    mission_items = [
        MissionItem(absolute_latitude + 10 * 1e-5, absolute_longitude , flying_alt, 5.0, True,
                    float('nan'), float('nan'), MissionItem.CameraAction.NONE,
                    float('nan'), float('nan'), float('nan'), float('nan'),
                    float('nan'),MissionItem.VehicleAction.NONE),
        MissionItem( absolute_latitude + 10 * 1e-5 , absolute_longitude + 10 * 1e-5  , flying_alt, 5.0, True,
                    float('nan'), float('nan'), MissionItem.CameraAction.NONE,
                    float('nan'), float('nan'), float('nan'), float('nan'),
                    float('nan'), MissionItem.VehicleAction.NONE),

        MissionItem(absolute_latitude, absolute_longitude + 10 * 1e-5, flying_alt, 5.0, True,
                    float('nan'), float('nan'), MissionItem.CameraAction.NONE,
                    float('nan'), float('nan'), float('nan'), float('nan'),
                    float('nan'), MissionItem.VehicleAction.NONE),

        MissionItem(absolute_latitude, absolute_longitude, flying_alt, 5.0, True,
                    float('nan'), float('nan'), MissionItem.CameraAction.NONE,
                    float('nan'), float('nan'), float('nan'), float('nan'),
                    float('nan'), MissionItem.VehicleAction.NONE)
    ]

    mission_plan = MissionPlan(mission_items)

    print("Uploading mission...")
    await drone.mission.upload_mission(mission_plan)
    await asyncio.sleep(2)

    print("Starting mission...")
    await drone.mission.start_mission()
    await asyncio.sleep(2)

    #Folowing the progress:

    async for mission_progress in drone.mission.mission_progress(): 
        print(f"Mission progress: {mission_progress.current}/{mission_progress.total}")
        if mission_progress.current == mission_progress.total:
            print("-- Mission completed!")
            break

    print("Landing...")
    await drone.action.land()
    await asyncio.sleep(10)



asyncio.run(main()) # run the asyncio function 

