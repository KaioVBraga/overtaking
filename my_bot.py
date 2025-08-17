# my_bot.py

def drive(car_state):
    """
    Produces driving commands in response to the car's current state.
    
    Args:
        car_state: A dictionary containing the car's sensor readings.
        
    Returns:
        A dictionary of commands for steering, acceleration, and braking.
    """
    
    # --- Steering Control ---
    # A simple proportional controller to steer toward the track center.
    # The 'angle' is the angle between the car's direction and the track's axis.
    # The 'track_pos' is the car's distance from the track center.
    steer_command = car_state['angle'] * 0.5 + car_state['track_pos'] * 0.1
    
    # --- Speed Control ---
    # If the car is pointing straight, accelerate. Otherwise, brake gently.
    if abs(car_state['angle']) < 0.2:  # Angle is in radians
        accel_command = 0.7
        brake_command = 0.0
    else:
        # Slow down for turns
        accel_command = 0.3
        brake_command = 0.1

    # --- Command Dictionary ---
    return {
        'steer': steer_command,
        'accel': accel_command,
        'brake': brake_command,
    }