import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from log import logger # Assuming you have a logger.py file

class TorcsDriver:
    def __init__(self):
        # --- Preserve original state variables ---
        self.log_car_state_count = 0
        self.log_car_control_count = 0
        self.steer_count = 0
        self.last_steer = 0.0

        # --- Initialize the Fuzzy Control System ---
        self._setup_fuzzy_system()

    def _setup_fuzzy_system(self):
        """
        Defines and compiles the fuzzy logic controller using scikit-fuzzy.
        This is called only once when the driver is created.
        """
        # --- Antecedents (Inputs) ---
        track_pos = ctrl.Antecedent(np.arange(-1.5, 1.51, 0.1), 'track_pos')
        angle = ctrl.Antecedent(np.arange(-np.pi, np.pi + 0.1, 0.1), 'angle')
        speed_x = ctrl.Antecedent(np.arange(0, 301, 1), 'speed_x')

        # --- Consequents (Outputs) ---
        steer = ctrl.Consequent(np.arange(-1, 1.01, 0.1), 'steer')
        accel = ctrl.Consequent(np.arange(0, 1.01, 0.1), 'accel')

        # --- Membership Functions for Inputs ---
        track_pos['far_left']   = fuzz.trimf(track_pos.universe, [-1.5, -1.5, -0.7])
        track_pos['left']       = fuzz.trimf(track_pos.universe, [-1.0, -0.4, 0.0])
        track_pos['center']     = fuzz.trimf(track_pos.universe, [-0.3, 0.0, 0.3])
        track_pos['right']      = fuzz.trimf(track_pos.universe, [0.0, 0.4, 1.0])
        track_pos['far_right']  = fuzz.trimf(track_pos.universe, [0.7, 1.5, 1.5])

        angle['sharp_left']  = fuzz.trimf(angle.universe, [-np.pi, -0.8, -0.3])
        angle['left']        = fuzz.trimf(angle.universe, [-0.5, -0.2, 0.0])
        angle['straight']    = fuzz.trimf(angle.universe, [-0.15, 0.0, 0.15])
        angle['right']       = fuzz.trimf(angle.universe, [0.0, 0.2, 0.5])
        angle['sharp_right'] = fuzz.trimf(angle.universe, [0.3, 0.8, np.pi])
        
        speed_x['slow'] = fuzz.trimf(speed_x.universe, [0, 30, 80])
        speed_x['medium'] = fuzz.trimf(speed_x.universe, [60, 120, 180])
        speed_x['fast'] = fuzz.trimf(speed_x.universe, [150, 220, 300])

        # --- Membership Functions for Outputs ---
        steer['hard_left']   = fuzz.trimf(steer.universe, [-1.0, -0.8, -0.5])
        steer['left']        = fuzz.trimf(steer.universe, [-0.6, -0.3, 0.0])
        steer['zero']        = fuzz.trimf(steer.universe, [-0.1, 0.0, 0.1])
        steer['right']       = fuzz.trimf(steer.universe, [0.0, 0.3, 0.6])
        steer['hard_right']  = fuzz.trimf(steer.universe, [0.5, 0.8, 1.0])

        accel['slow']   = fuzz.trimf(accel.universe, [0.0, 0.2, 0.4])
        accel['medium'] = fuzz.trimf(accel.universe, [0.3, 0.5, 0.7])
        accel['fast']   = fuzz.trimf(accel.universe, [0.6, 0.8, 1.0])

        # --- Fuzzy Rules (Improved for Cornering) ---
        rules = [
            # --- Steering Rules ---
            # Basic position correction
            ctrl.Rule(track_pos['far_left'], steer['hard_right']),
            ctrl.Rule(track_pos['left'], steer['right']),
            ctrl.Rule(track_pos['center'], steer['zero']),
            ctrl.Rule(track_pos['right'], steer['left']),
            ctrl.Rule(track_pos['far_right'], steer['hard_left']),
            
            # Angle correction (overrides position if angle is bad)
            ctrl.Rule(angle['sharp_left'], steer['hard_right']),
            ctrl.Rule(angle['sharp_right'], steer['hard_left']),

            # ** NEW: Combination rules for better cornering **
            # If we are centered but pointing wrong, correct the angle
            ctrl.Rule(track_pos['center'] & angle['left'], steer['right']),
            ctrl.Rule(track_pos['center'] & angle['right'], steer['left']),

            # --- Acceleration Rules ---
            # Go fast on straights, but slow down if speed is already high
            ctrl.Rule(angle['straight'] & (speed_x['slow'] | speed_x['medium']), accel['fast']),
            ctrl.Rule(angle['straight'] & speed_x['fast'], accel['medium']),
            
            # Moderate speed in slight turns
            ctrl.Rule(angle['left'] | angle['right'], accel['medium']),
            
            # Go slow in sharp turns
            ctrl.Rule(angle['sharp_left'] | angle['sharp_right'], accel['slow']),

            # ** NEW: Safety rule **
            # If we are far off track, slow down to regain control
            ctrl.Rule(track_pos['far_left'] | track_pos['far_right'], accel['slow'])
        ]

        # --- Create and store the simulation ---
        fuzzy_control = ctrl.ControlSystem(rules)
        self.fuzzy_simulation = ctrl.ControlSystemSimulation(fuzzy_control)

    # --- Preserved Logging Methods ---
    def log_car_state(self, car_state):
        self.log_car_state_count = (self.log_car_state_count % 100) + 1
        if self.log_car_state_count == 1 and car_state:
            car_meaningful_state = {
                'angle': car_state.get('angle'),
                'damage': car_state.get('damage'),
                'distRaced': car_state.get('distRaced'),
                'fuel': car_state.get('fuel'),
                'gear': car_state.get('gear'),
                'trackPos': car_state.get('trackPos')
            }
            logger.info(f"Car meaningful state: {car_meaningful_state}\n")

    def log_car_control(self, car_control):
        self.log_car_control_count = (self.log_car_control_count % 100) + 1
        if self.log_car_control_count == 1 and car_control:
            logger.info(f"Car control: {car_control}\n\n")

    # --- Main Drive Method (Internal Logic Changed) ---
    def drive(self, car_state): 
        if not car_state:
            return dict(accel=0, brake=0, gear=1, steer=0)

        self.log_car_state(car_state)
        
        # 1. Provide crisp inputs to the fuzzy system
        current_speed = car_state.get('speedX', 0.0)
        self.fuzzy_simulation.inputs({
            'track_pos': car_state.get('trackPos', 0.0),
            'angle': car_state.get('angle', 0.0),
            'speed_x': current_speed
        })

        # 2. Compute the fuzzy logic output
        self.fuzzy_simulation.compute()
        fuzzy_output = self.fuzzy_simulation.output

        # 3. Use fuzzy outputs and simple rules to build the final control dictionary
        
        # ** NEW: Dynamic steering based on speed **
        # Reduce the amount of steering at higher speeds to prevent instability
        steer_lock = 0.77 - (current_speed / 300)
        steer_command = fuzzy_output['steer'] * steer_lock
        
        # Smooth the steering to prevent jerky movements
        steer_command = (self.last_steer * 0.4) + (steer_command * 0.6)
        self.last_steer = steer_command
        
        # Simple braking logic
        brake_command = 0.0
        if current_speed > 100 and abs(car_state.get('angle', 0.0)) > 0.3:
            brake_command = 0.2

        car_control = dict(
            accel = fuzzy_output['accel'],
            brake = brake_command,
            gear = 1, # Simple gear logic
            steer = steer_command,
        )

        self.log_car_control(car_control)

        return car_control
