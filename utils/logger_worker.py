import os
import time
import logging
import re

LOG_WSL_TO_WINDOWS_FILE = False

formatter = logging.Formatter('%(asctime)s - [%(name)s] - %(levelname)s - %(message)s')

console_handler = logging.StreamHandler()
console_handler.setFormatter(formatter)

file_handler = logging.FileHandler("windows_combined.log", mode='a', encoding='utf-8')
file_handler.setFormatter(formatter)

win_logger = logging.getLogger("WIN_APP")
win_logger.setLevel(logging.DEBUG)
win_logger.propagate = False
win_logger.addHandler(console_handler)
win_logger.addHandler(file_handler)

# 2. WSL Client Logger
wsl_logger = logging.getLogger("WSL_APP")
wsl_logger.setLevel(logging.DEBUG)
wsl_logger.propagate = False
wsl_logger.addHandler(console_handler)

if LOG_WSL_TO_WINDOWS_FILE:
    wsl_logger.addHandler(file_handler)


ERROR_LEVEL_RE = re.compile(r"(?:^|\s-\s|\[)(?:ERROR|CRITICAL)(?:\]|\s-|$)")


def log_wsl_line(line: str):
    """
    Log WSL output as DEBUG by default, but keep real errors visible as ERROR.
    """
    message = line.strip()
    if not message:
        return

    if ERROR_LEVEL_RE.search(message):
        wsl_logger.error(message)
    else:
        wsl_logger.debug(message)


def tail_linux_logs(log_path: str):
    """
    Continuously reads the Linux log file (like 'tail -f'),
    handles file rotation, and injects lines into the WSL logger.
    """
    # Wait for the Linux script to create the file
    while not os.path.exists(log_path):
        time.sleep(0.5)

    current_file = open(log_path, 'r', encoding='utf-8')
    
    # Uncomment the next line if you want to ignore old logs and only print new ones on startup
    # current_file.seek(0, os.SEEK_END)

    while True:
        line = current_file.readline()
        
        if not line:
            # Check if file was rotated by WSL (size became smaller than our current read position)
            try:
                if os.path.getsize(log_path) < current_file.tell():
                    wsl_logger.debug("Log rotation detected. Reopening file.")
                    current_file.close()
                    current_file = open(log_path, 'r', encoding='utf-8')
                    continue
            except OSError:
                pass # Prevent crash if file is temporarily inaccessible during rotation
            
            # Wait for new content to be written
            time.sleep(0.1)
            continue
        
        # Inject the raw Linux line into the WSL logger
        log_wsl_line(line)
