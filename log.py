import logging
import sys

# Create logger
logger = logging.getLogger("my_app")
logger.setLevel(logging.DEBUG)  # Minimum level to capture

# Formatter (shared for both handlers)
formatter = logging.Formatter("%(asctime)s - %(levelname)s - %(message)s")

# File handler
file_handler = logging.FileHandler("app.log", mode="a")
file_handler.setFormatter(formatter)

# Stdout handler
stream_handler = logging.StreamHandler(sys.stdout)
stream_handler.setFormatter(formatter)

# Avoid adding handlers multiple times if this file is imported repeatedly
if not logger.hasHandlers():
    logger.addHandler(file_handler)
    logger.addHandler(stream_handler)