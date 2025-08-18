# scr_client.py
# A simple Python client to connect to a TORCS scr_server bot.
# Author: Gemini
# Date: 2025-08-17

import socket
import sys
import time
from torcs_driver import TorcsDriver
from log import logger

class TorcsClient:
    """
    Client to connect to a TORCS scr_server and control a car.
    """
    def __init__(self, host='localhost', port=3001):
        self.host = host
        self.port = port
        self.sock = None
        self.driver = TorcsDriver()
        self.log_car_state_count = 0
        self.log_car_control_count = 0
        self.steer_count = 0

    def connect(self):
        """Creates a UDP socket and connects to the server."""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except socket.error as msg:
            logger.error(f"Error: could not create socket: {msg}")
            sys.exit(1)
        
        # The scr_server doesn't send a confirmation, so we just assume connection.
        logger.info(f"Socket created. Ready to send to {self.host}:{self.port}")

    def send_init_request(self):
        """Sends a correctly formatted initialization string to the server."""
        # The init string must define the angles for the car's sensors.
        # A standard set of 19 sensors, from -90 to +90 degrees.
        angles = ' '.join(str(i) for i in range(-90, 91, 10))
        init_str = f"SCR(init {angles})"
        
        try:
            self.sock.sendto(init_str.encode(), (self.host, self.port))
            logger.info("Initialization request sent.")
        except socket.error as msg:
            logger.error(f"Error: failed to send init request: {msg}")
            sys.exit(1)

    def parse_server_message(self, message):
        """Parses a string of sensor data from the server."""
        state = {}
        # The message is a string of key-value pairs in parentheses
        # e.g., (angle -0.00152959)(trackPos 0.001223)...
        parts = message.strip().replace(')(', ' ').replace('(', '').replace(')', '').split()
        
        # This creates a dictionary like {'angle': -0.0015, 'trackPos': 0.0012, ...}
        # It handles lists like 'track' and 'opponents' by creating sub-lists.
        i = 0
        while i < len(parts):
            key = parts[i]
            # Check if the next part is a value or another key (for lists)
            if i + 1 < len(parts) and not parts[i+1].isalpha():
                values = []
                i += 1
                while i < len(parts) and not parts[i].isalpha():
                    try:
                        values.append(float(parts[i]))
                    except ValueError:
                        break # Not a float, must be the next key
                    i += 1
                state[key] = values[0] if len(values) == 1 else values
            else:
                i += 1 # Key with no value
        return state

    def format_control_command(self, car_control):
        """Formats driving commands into a string for the server."""
        return f"(accel {car_control['accel']})(brake {car_control['brake']})(gear {car_control['gear']})(steer {car_control['steer']})"
    
    def drive_loop(self):
        """The main loop that receives data, decides action, and sends command."""
        logger.info("Starting drive loop. Press Ctrl+C to exit.")
        self.send_init_request()
        
        while True:
            try:
                # Receive data from the server
                message, addr = self.sock.recvfrom(1024)
                message = message.decode()

                # print(f"Server response: {message}")

                # The server sends '***shutdown***' or '***restart***' to end the race
                if message == "***shutdown***" or message == "***restart***":
                    logger.info(f"Server message: {message}. Exiting.")
                    break

                # Parse the sensor data
                car_state = self.parse_server_message(message)

                car_control = self.driver.drive(car_state)

                # Format and send the command
                command = self.format_control_command(car_control)
                self.sock.sendto(command.encode(), (self.host, self.port))

            except socket.error as msg:
                logger.error(f"Socket error: {msg}")
                break
            except KeyboardInterrupt:
                logger.info("User interrupted. Shutting down.")
                break
        
        self.sock.close()
        logger.info("Connection closed.")