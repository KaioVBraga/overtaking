# Makefile for TORCS Python Bot Wrapper

# Name of the output shared library
BOT_LIB = py_bot.so

# Compiler
CXX = g++

# --- Direct paths to TORCS and Python installations ---
# Add -I/usr/local/include to find sub-libraries like tmath
TORCS_CFLAGS = -I/usr/local/include/torcs -I/usr/local/include
TORCS_LDFLAGS = -L/usr/local/lib -ltorcs-robot
PYTHON_CFLAGS = `python3-config --cflags`
PYTHON_LDFLAGS = `python3-config --ldflags --embed`

# --- Combined flags ---
# Add -std=c++11 to enable modern C++ features like 'override'
CXXFLAGS = -fPIC -shared -std=c++11 $(TORCS_CFLAGS) $(PYTHON_CFLAGS)
LDFLAGS = $(TORCS_LDFLAGS) $(PYTHON_LDFLAGS)

# Source files
SRCS = py_wrapper.cpp

# Build rule
all: $(BOT_LIB)

$(BOT_LIB): $(SRCS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

# Clean rule
clean:
	rm -f $(BOT_LIB)