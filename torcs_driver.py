# torcs_driver.py
# An improved TORCS driver with a more advanced fuzzy logic controller for better cornering.
# Author: Gemini
# Date: 2025-08-18

import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from log import logger

class TorcsDriver:
    def __init__(self):
        # --- Preserve original state variables ---
        self.log_count = 0 # Counter for the new rule logger
        self.log_ticks = 50
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
        track_pos = ctrl.Antecedent(np.arange(-2, 2.1, 0.1), 'track_pos')
        angle = ctrl.Antecedent(np.arange(-np.pi, np.pi + 0.1, 0.1), 'angle')
        speed_x = ctrl.Antecedent(np.arange(0, 301, 1), 'speed_x')

        # --- Consequents (Outputs) ---
        steer = ctrl.Consequent(np.arange(-1, 1.01, 0.1), 'steer')
        # accel = ctrl.Consequent(np.arange(0, 1.01, 0.1), 'accel')

        # --- Membership Functions for Inputs (Widened for stability) ---
        # track_pos['far_left']   = fuzz.trimf(track_pos.universe, [-2.0, -2.0, -0.8])
        # track_pos['left']       = fuzz.trimf(track_pos.universe, [-1.2, -0.5, 0.0])
        # track_pos['center']     = fuzz.trimf(track_pos.universe, [-0.4, 0.0, 0.4])
        # track_pos['right']      = fuzz.trimf(track_pos.universe, [0.0, 0.5, 1.2])
        # track_pos['far_right']  = fuzz.trimf(track_pos.universe, [0.8, 2.0, 2.0])

        track_pos['far_right']   = fuzz.trimf(track_pos.universe, [-2.0, -2.0, -0.8])
        track_pos['right']       = fuzz.trimf(track_pos.universe, [-1.2, -0.5, 0.0])
        track_pos['center']     = fuzz.trimf(track_pos.universe, [-0.4, 0.0, 0.4])
        track_pos['left']      = fuzz.trimf(track_pos.universe, [0.0, 0.5, 1.2])
        track_pos['far_left']  = fuzz.trimf(track_pos.universe, [0.8, 2.0, 2.0])

        angle['sharp_left']  = fuzz.trimf(angle.universe, [-np.pi, -1.0, -0.4])
        angle['left']        = fuzz.trimf(angle.universe, [-0.6, -0.3, 0.0])
        angle['straight']    = fuzz.trimf(angle.universe, [-0.2, 0.0, 0.2])
        angle['right']       = fuzz.trimf(angle.universe, [0.0, 0.3, 0.6])
        angle['sharp_right'] = fuzz.trimf(angle.universe, [0.4, 1.0, np.pi])


        # angle['sharp_right']  = fuzz.trimf(angle.universe, [-np.pi, -1.0, -0.4])
        # angle['right']        = fuzz.trimf(angle.universe, [-0.6, -0.3, 0.0])
        # angle['straight']    = fuzz.trimf(angle.universe, [-0.2, 0.0, 0.2])
        # angle['left']       = fuzz.trimf(angle.universe, [0.0, 0.3, 0.6])
        # angle['sharp_left'] = fuzz.trimf(angle.universe, [0.4, 1.0, np.pi])
        
        speed_x['slow'] = fuzz.trimf(speed_x.universe, [0, 40, 90])
        speed_x['medium'] = fuzz.trimf(speed_x.universe, [70, 130, 190])
        speed_x['fast'] = fuzz.trimf(speed_x.universe, [160, 230, 301])

        # --- Membership Functions for Outputs ---
        # steer['hard_left']   = fuzz.trimf(steer.universe, [-1.0, -0.8, -0.5])
        # steer['left']        = fuzz.trimf(steer.universe, [-0.6, -0.3, 0.0])
        # steer['zero']        = fuzz.trimf(steer.universe, [-0.1, 0.0, 0.1])
        # steer['right']       = fuzz.trimf(steer.universe, [0.0, 0.3, 0.6])
        # steer['hard_right']  = fuzz.trimf(steer.universe, [0.5, 0.8, 1.0])

        steer['hard_right']   = fuzz.trimf(steer.universe, [-1.0, -0.8, -0.5])
        steer['right']        = fuzz.trimf(steer.universe, [-0.6, -0.3, 0.0])
        steer['zero']        = fuzz.trimf(steer.universe, [-0.1, 0.0, 0.1])
        steer['left']       = fuzz.trimf(steer.universe, [0.0, 0.3, 0.6])
        steer['hard_left']  = fuzz.trimf(steer.universe, [0.5, 0.8, 1.0])

        # accel['slow']   = fuzz.trimf(accel.universe, [0.0, 0.2, 0.4])
        # accel['medium'] = fuzz.trimf(accel.universe, [0.3, 0.6, 0.8])
        # accel['fast']   = fuzz.trimf(accel.universe, [0.7, 0.9, 1.0])

        # --- Fuzzy Rules (Rewritten for Stability and Cornering) ---
        rules = [
            # --- Steering Rules ---
            # High-priority rules to prevent spinning out
            ctrl.Rule(angle['sharp_left'], steer['hard_right']),
            ctrl.Rule(angle['sharp_right'], steer['hard_left']),

            # Combination rules for stable cornering
            ctrl.Rule(track_pos['left'] & angle['left'], steer['right']),
            ctrl.Rule(track_pos['right'] & angle['right'], steer['left']),
            ctrl.Rule(track_pos['center'] & angle['left'], steer['right']),
            ctrl.Rule(track_pos['center'] & angle['right'], steer['left']),
            
            # Basic position correction when angle is straight
            ctrl.Rule(track_pos['far_left'] & angle['straight'], steer['hard_right']),
            ctrl.Rule(track_pos['left'] & angle['straight'], steer['right']),
            ctrl.Rule(track_pos['center'] & angle['straight'], steer['zero']),
            ctrl.Rule(track_pos['right'] & angle['straight'], steer['left']),
            ctrl.Rule(track_pos['far_right'] & angle['straight'], steer['hard_left']),

            # --- Acceleration Rules ---
            # Go fast on straights
            # ctrl.Rule(angle['straight'] & (speed_x['slow'] | speed_x['medium']), accel['fast']),
            
            # # Maintain speed on straights if already fast
            # ctrl.Rule(angle['straight'] & speed_x['fast'], accel['medium']),
            
            # # Moderate speed in slight turns
            # ctrl.Rule(angle['left'] | angle['right'], accel['medium']),
            
            # # Go slow in sharp turns
            # ctrl.Rule(angle['sharp_left'] | angle['sharp_right'], accel['slow']),

            # # Safety rule: if far off track, slow down
            # ctrl.Rule(track_pos['far_left'] | track_pos['far_right'], accel['slow'])
        ]

        # --- Create and store the simulation ---
        fuzzy_control = ctrl.ControlSystem(rules)
        self.fuzzy_simulation = ctrl.ControlSystemSimulation(fuzzy_control)

    # --- Preserved Logging Methods ---
    def log_car_state(self, car_state):
        if self.log_count == 1 and car_state:
            car_meaningful_state = {
                'angle': car_state.get('angle'),
                'damage': car_state.get('damage'),
                'distRaced': car_state.get('distRaced'),
                'fuel': car_state.get('fuel'),
                'gear': car_state.get('gear'),
                'trackPos': car_state.get('trackPos')
            }
            logger.info(f"Car meaningful state: {car_meaningful_state}")

    def log_car_control(self, car_control):
        if self.log_count == 1 and car_control:
            logger.info(f"Car control: {car_control}\n\n")

    def log_active_rules(self):
        if self.log_count == 1:
            active_rules = []
            # Correctly iterate through the control system's rules
            for rule in self.fuzzy_simulation.ctrl.rules:
                # The 'antecedent_activation' attribute holds the final float value
                activation = rule.antecedent_activation
                if activation > 0.1:  # Only log rules with significant activation
                    active_rules.append(f"  - Rule: '{rule.label}' -> Activation: {activation:.2f}")
            
            if active_rules:
                logger.info("Active Fuzzy Rules:\n" + "\n".join(active_rules) + "\n")

    # --- Main Drive Method (Internal Logic Changed) ---
    def drive(self, car_state):
        self.log_count = (self.log_count % self.log_ticks) + 1

        if not car_state:
            logger.info("Empty car_state\n")
            return dict(accel=0, brake=0, gear=1, steer=0)

        
        # 1. Provide crisp inputs to the fuzzy system
        current_speed = car_state.get('speedX', 0.0)
        track_pos = car_state.get('trackPos', 0.0)
        angle = car_state.get('angle', 0.0)

        self.fuzzy_simulation.inputs({
            'track_pos': track_pos,
            'angle': angle,
            # 'speed_x': current_speed
        })

        # 2. Compute the fuzzy logic output
        self.fuzzy_simulation.compute()
        fuzzy_output = self.fuzzy_simulation.output
        
        # Log the active rules for debugging
        # self.log_active_rules()

        # 3. Use fuzzy outputs and simple rules to build the final control dictionary
        
        # Dynamic steering based on speed
        steer_lock = 0.8 - (current_speed / 400)
        steer_command = fuzzy_output['steer'] if fuzzy_output['steer'] != None else 0
        steer_command = steer_command * max(steer_lock, 0.2)
        
        # Smooth the steering to prevent jerky movements
        steer_command = (self.last_steer * 0.5) + (steer_command * 0.5)
        self.last_steer = steer_command
        
        # Dynamic braking logic
        brake_command = 0.0
        if abs(angle) > 0.4 and current_speed > 90: # Brake harder in fast, sharp turns
            brake_command = 0.3
        elif abs(track_pos) > 1.0: # Brake if off track
            brake_command = 0.2
            
        car_control = dict(
            # accel = fuzzy_output['accel'],
            accel = 1,
            brake = brake_command,
            gear = 1, # Simple gear logic
            steer = steer_command,
        )
        

        self.log_car_state(car_state)
        self.log_car_control(car_control)

        return car_control