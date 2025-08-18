# scr_client.py
# A simple Python client to connect to a TORCS scr_server bot.
# Author: Gemini
# Date: 2025-08-17

import socket
import sys
import time
from log import logger

class TorcsDriver:
    def __init__(self):
        self.log_car_state_count = 0
        self.log_car_control_count = 0
        self.steer_count = 0

    def log_car_state(self, car_state):
        self.log_car_state_count = self.log_car_state_count % 100 + 1

        if(self.log_car_state_count == 1 and len(car_state) > 0):
            # logger.info(f"Car state: {car_state}\n\n")

            car_meaningful_state = {}
            car_meaningful_state['angle'] = car_state['angle']
            car_meaningful_state['damage'] = car_state['damage']
            car_meaningful_state['distRaced'] = car_state['distRaced']
            car_meaningful_state['fuel'] = car_state['fuel']
            car_meaningful_state['gear'] = car_state['gear']
            car_meaningful_state['trackPos'] = car_state['trackPos']

            logger.info(f"Car meaninful state: {car_meaningful_state}\n")

    def log_car_control(self, car_control):
        self.log_car_control_count = self.log_car_control_count % 100 + 1

        if(self.log_car_control_count == 1 and len(car_control) > 0):
            logger.info(f"Car control: {car_control}\n\n")
    
    def get_steer_v1(self,car_state):
        self.steer_count = self.steer_count % 1000 + 1

        if(self.steer_count < 333):
            return -0.25

        if(self.steer_count < 666):
            return 0

        return 0.25
    
    def get_steer_v2(self, car_state):
        if(car_state['angle'] < -5):
            return 0.25

        if(car_state['angle'] > 5):
            return -0.25

    def get_steer(self, car_state):
        self.steer_count = self.steer_count % 1000 + 1

        if(car_state['trackPos'] > 1.75):
            return -0.25
        
        if(car_state['trackPos'] > 1.15):
            return -0.1

        if(car_state['trackPos'] < 0.75):
            return 0.25
        
        if(car_state['trackPos'] < 0.15):
            return 0.15

        return 0
        
    
    def get_gear(self, car_state):
        return 1
    
    def get_brake(self, car_state):
        return 0
    
    def get_accel(self, car_state):
        # return 1
        return 0.5
    
    def drive(self, car_state): 
        if(len(car_state) < 1):
            return dict(accel = 0, brake = 0, gear=0, steer=0)

        self.log_car_state(car_state)
                
        car_control = dict(
            accel = self.get_accel(car_state),
            brake = self.get_brake(car_state),
            gear = self.get_gear(car_state),
            steer = self.get_steer(car_state),
        )

        self.log_car_control(car_control)

        return car_control
    
