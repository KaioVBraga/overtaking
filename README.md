# TORCS Fuzzy Logic Python Bot

This project provides a Python client and an AI driver to control a car in The Open Racing Car Simulator (TORCS). The driver uses a fuzzy logic controller, implemented with the scikit-fuzzy library, to make real-time decisions for steering and acceleration, aiming to keep the car on the track.

This system communicates with a special TORCS bot called scr_server over a network socket, allowing the Python code to run as a separate process from the game itself.

## Features

- Fuzzy Logic Controller: The car's driving logic is not based on hardcoded if/else statements but on linguistic rules (e.g., "IF the car is far left THEN steer hard right"), providing smoother and more human-like control.
- Modular Design: The client (scr_client.py) is separate from the driver logic (torcs_driver.py), making it easy to experiment with and tune the AI's behavior without modifying the network code.
- Real-time Control: Connects to a running TORCS instance and controls the car live.

## Prerequisites

Before you can run this bot, you need to have the following software installed and configured:

- Python 3: Along with the pip package manager.
- scikit-fuzzy: The core library for the fuzzy logic controller.
- pip install scikit-fuzzy numpy
- A Patched Version of TORCS: The standard version of TORCS available in most package managers will not work. You must compile TORCS from a specific source code repository that includes the scr_server bot.

## Installation

The most complex part of the setup is installing the correct version of TORCS. This project was developed and tested against the version from the fmirus/torcs-1.3.7 repository on GitHub.

A complete, step-by-step guide on how to compile and install this version of TORCS on Ubuntu can be found in the Canvas document we have been working on. The key steps from that guide are:

- Install all required build dependencies (build-essential, libplib-dev, freeglut3-dev, etc.).
- Clone the repository: git clone https://github.com/fmirus/torcs-1.3.7.git.
- Run ./configure, make, sudo make install, and sudo make datainstall inside the source directory.

Refer to the guide for detailed commands and workarounds for common compilation issues.

## How to Run

### Start TORCS and Prepare the Race:

- Launch the game from your terminal: /usr/local/bin/torcs.
- In the main menu, navigate to Race -> Quick Race -> Configure Race.
- In the "Player List" on the right, add one of the scr_server drivers (e.g., scr_server 1) to the "Selected Players" list on the left.
- Click Accept, and then click New Race.

The game will now freeze on the starting grid, and the terminal where you launched TORCS will display a message like Waiting for request on port 3001. This is the correct behavior.

### Run the Python Client:

- Open a new terminal window.
- Navigate to the directory of this repo.

- Run the client script:

```
  python3 main.py
```

The Python script will connect to the waiting scr_server, and the race will begin immediately. The car will be controlled by the fuzzy logic defined in torcs_driver.py. To stop the bot, press Ctrl+C in the terminal where the script is running.
Tuning the Bot

The "brain" of the bot is located in the \_setup_fuzzy_system method within the torcs_driver.py file. You can easily change the car's behavior by modifying:

- Membership Functions: Adjust the ranges of the fuzzy sets (e.g., make the "center" of the track wider or narrower).
- Fuzzy Rules: Add, remove, or modify the rules to handle different situations, such as taking corners at different speeds or recovering from being far off-center.
