def reward_function(on_track, x, y, distance_from_center, car_orientation, progress, steps, throttle, steering, track_width, waypoints, is_left_of_center, closest_waypoint):

    '''
    @on_track (boolean) :: The vehicle is off-track if the front of the vehicle is outside of the white
    lines

    @x (float range: [0, 1]) :: Fraction of where the car is along the x-axis. 1 indicates
    max 'x' value in the coordinate system.

    @y (float range: [0, 1]) :: Fraction of where the car is along the y-axis. 1 indicates
    max 'y' value in the coordinate system.

    @distance_from_center (float [0, track_width/2]) :: Displacement from the center line of the track
    as defined by way points

    @car_orientation (float: [-3.14, 3.14]) :: yaw of the car with respect to the car's x-axis in
    radians (-180 degrees, 180 degrees)

    @progress (float: [0,1]) :: % of track complete

    @steps (int) :: numbers of steps completed. One step is one move by the car

    @throttle :: (float) 0 to 1 (0 indicates stop, 1 max throttle)

    @steering :: (float) -1 to 1 (-1 is right, 1 is left)

    @track_width (float) :: width of the track (> 0)

    @waypoints (ordered list) :: list of waypoint in order; each waypoint is a set of coordinates
    (x,y,yaw) that define a turning point. Defines center.

    @closest_waypoint (int) :: index of the closest waypoint (0-indexed) given the car's x,y
    position as measured by the eucliedean distance
    
    @is_left_of_center (bool) :: is left of center

    @@output: @reward (float [-1e5, 1e5])
    '''


    '''
    Ideas:
    Incentivize throttle on straight aways by looking ahead on yaws to detect straight aways
    Incentivize being on the left side being 3/8ths from the wall
    
    '''
    import math
    from statistics import mean

    ##########
    # Settings
    ##########
    # Min / Max Reward
    REWARD_MIN = -1e5
    REWARD_MAX = 1e5
    # Define the Area each side of the center that the card can use.
    # Later version might consider adjust this so that it can hug corners
    CENTER_LANE = track_width * .25
    HALF_TRACK = track_width / 2

    ABS_STEERING_THRESHOLD = .85

    ####################
    # Locations on track
    ####################

    # Set Base Reward
    if not on_track: # Fail them if off Track
        reward = REWARD_MIN
        return reward
    elif progress == 1:
        reward = REWARD_MAX
        return reward
    else:        # we want the vehicle to continue making progress
        reward = REWARD_MAX * progress
    
    #Check is Turning
    correction= 0
    waypoint_yaw = waypoints[closest_waypoint][-1]
    next_waypoint_yaw = waypoints[min(closest_waypoint+1, len(waypoints)-1)][-1]
    next_next_waypoint_yaw = waypoints[min(closest_waypoint+2, len(waypoints)-1)][-1]
    if(abs(waypoint_yaw - next_waypoint_yaw) > math.radians(2)):
        correction +=.5
    if(abs(next_next_waypoint_yaw - next_waypoint_yaw) > math.radians(2)):
        correction +=.5
    
        ##########
        # On straight
        ##########
    if correction == 0:
        if throttle != 1:
            reward *= 1 - throttle;
        if abs(steering) >.1:
            reward *= steering;
        if is_left_of_center:
            if (distance_from_center >0 and distance_from_center< track_width/4):
                reward += 50000
            else:
                reward -= 50000
        else:
            if (distance_from_center >0 and distance_from_center <track_width/8):
                reward +=10000
            elif(distance_from_center>track_width/8 and distance_from_center<track_width*3/8):
                #Do nothing
            else:
                reward -=50000
                
    ##########
    # Around Curve
    ##########
    else:
        if abs(steering) > .5 and abs(steering > throttle):
            reward *= 1 - (steering - throttle)  
            
        if abs(car_orientation - next_waypoint_yaw) >= math.radians(10):
            reward *= 1 - (abs(car_orientation - next_waypoint_yaw) / 180)
        elif abs(car_orientation - next_waypoint_yaw) < math.radians(10) and abs(steering) > ABS_STEERING_THRESHOLD:    # penalize if stearing to much
            reward *= ABS_STEERING_THRESHOLD / abs(steering)
        else:
            reward *= 1 + (10 - (abs(car_orientation - next_waypoint_yaw) / 10))   
            


    # Add penalty for wrong direction
    



    
    # Add penalty if throttle exsides the steering else add reward


    # make sure reward value returned is within the prescribed value range.
    reward = max(reward, REWARD_MIN)
    reward = min(reward, REWARD_MAX)

    return float(reward)
