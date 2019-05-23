def reward_function(params):

    '''
    @on_track (boolean) :: The vehicle is off-track if the front of the vehicle is outside of the white
    lines

    @x (float range: [0, 1]) :: Fraction of where the car is along the x-axis. 1 indicates
    max 'x' value in the coordinate system.

    @y (float range: [0, 1]) :: Fraction of where the car is along the y-axis. 1 indicates
    max 'y' value in the coordinate system.

    @distance_from_center (float [0, track_width/2]) :: Displacement from the center line of the track
    as defined by way points

    @heading (float: [-180, 180]) :: yaw of the car with respect to the car's x-axis in
    radians (-180 degrees, 180 degrees)

    @progress (float: [0,1]) :: % of track complete

    @steps (int) :: numbers of steps completed. One step is one move by the car

    @speed :: (float) 0 to c In m/s

    @steering_angle :: (float) -30 to 30 (-30 is right, 30 is left)

    @track_width (float) :: width of the track (> 0)

    @waypoints (ordered list) :: list of waypoint in order; each waypoint is a set of coordinates
    (x,y,yaw) that define a turning point. Defines center.

    @closest_waypoints (int,int) :: index of the closest waypoint (0-indexed) given the car's x,y
    position as measured by the eucliedean distance
    
    @is_left_of_center (bool) :: is left of center

    @@output: @reward (float [-1e5, 1e5])
    '''
    on_track = params['all_wheels_on_track']
    x = params["x"]
    y = params["y"]
    distance_from_center = params["distance_from_center"]
    heading = params["heading"]
    progress = params["progress"]
    steps = params["steps"]
    throttle = params["speed"]
    steering = params["steering_angle"] /30
    track_width = params["track_width"]
    waypoints = params["waypoints"]
    is_left_of_center = params["is_left_of_center"]
    closest_waypoints = params["closest_waypoints"]
    '''
    Ideas:
    Incentivize throttle on straight aways by looking ahead on yaws to detect straight aways
    Incentivize being on the left side being 3/8ths from the wall
    With this model we only use positive reenforcement
    '''
    import math
    from statistics import mean
    SPEED_MAX = 5
    CURVING_SPEED_MAX=3
    ##########
    # Settings
    ##########
    # Min / Max Reward
    REWARD_MIN = .01
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
        reward = REWARD_MAX * max(progress, REWARD_MIN)
    
    #Check is Turning
    correction= 0
    next_next_point = waypoints[min(closest_waypoints[1]+1, len(waypoints)-1)]
    next_point = waypoints[closest_waypoints[1]]
    prev_point = waypoints[closest_waypoints[0]]  
    
    track_direction = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0]) 
    # Convert to degree
    track_direction = math.degrees(track_direction)  
    
    track_direction_next = math.atan2(next_next_point[1] - next_point[1], next_next_point[0] - next_point[0]) 
    # Convert to degree
    track_direction_next = math.degrees(track_direction)
  
    if(abs(track_direction) > 3):
        correction +=.5
    if(abs(track_direction_next) > 3):
        correction +=.5
    
    ##########
    # On straight
    ##########
    if correction == 0:
        if speed != SPEED_MAX:
            reward *= max(speed/SPEED_MAX,.01);
        if abs(steering) >.1:
            reward *= max(1-abs(steering),.01);
        if is_left_of_center:
            if (distance_from_center >0 and distance_from_center< track_width/4):
                reward *= 1.4
            else:
                reward *= .9
        else:
            if (distance_from_center >0 and distance_from_center <track_width/8):
                reward *= 1.1
            elif(distance_from_center>track_width/8 and distance_from_center<track_width*3/8):
                pass
            else:
                reward *=.9
                
    ##########
    # Around Curve
    ##########
    else:
        if abs(steering) > .5 and abs(steering) > speed/(CURVING_SPEED_MAX*2):
            reward *= max((1 - (steering - speed/(CURVING_SPEED_MAX*2)),.01))
        # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radia
    
        # Cacluate the difference between the track direction and the heading direction of the car
        direction_diff = abs(track_direction - heading)
        reward *= 1-direction_diff/30


    # make sure reward value returned is within the prescribed value range.
    reward = max(reward, REWARD_MIN)
    reward = min(reward, REWARD_MAX)

    return float(reward)
