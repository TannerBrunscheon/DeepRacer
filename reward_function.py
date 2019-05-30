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
    speed = params["speed"]
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
        return REWARD_MIN
    elif progress == 1:
        #the lap is complete.  if we use more steps than TARGET_STEPS, the reward is lower
        return REWARD_MAX * TARGET_STEPS / steps
    elif steps == 0:
        #assuming that the car starts on a straight track, set the base reward to be the maximum
        #we will adjust things later on
        reward = REWARD_MAX
    elif steps > 0:        # we want the vehicle to continue making progress
        reward = REWARD_MAX * (params["progress"] / params["steps"]) * 100
        
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
  
    if( abs(track_direction-track_direction_next) > 3):
        correction =True
    else:
        correction =False
    
    ##########
    # On straight
    ##########
    if not correction:
        if speed != SPEED_MAX:
            reward *= (speed/SPEED_MAX)**3
        if abs(steering) >.1:
            reward *= (1/(abs(steering)+1))**20;
        if distance_from_center < track_width/4:
            #the position is good, so bump the reward up 30%
            reward = reward * 1.3
        elif distance_from_center < track_width/3:
            #getting closer to the edge, so only increase the reward a bit
            reward = reward * 1.10
        elif distance_from_center < track_width/2:
            #getting close to the edge, so decrease the reward a bit
            reward = reward * 0.9
        else:
            #too close to the edge?  really discourage this position
            reward = reward * .01
                
    ##########
    # Around Curve
    ##########
    else:
        if abs(steering) > .75 and abs(steering) > speed/(CURVING_SPEED_MAX*2):
            reward *= max((1 - (steering - speed/(CURVING_SPEED_MAX*2)),.01))
        # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radia
    
        # Cacluate the difference between the track direction and the heading direction of the car
        direction_diff = abs(track_direction - heading)
        reward *= 1-((direction_diff**2/10)/10)


    # make sure reward value returned is within the prescribed value range.
    reward = max(reward, REWARD_MIN)
    reward = min(reward, REWARD_MAX)

    return float(reward)
