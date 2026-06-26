# ******************************************************************************
# *  LOGGER UTILS
# *
# *  @file  logger_utils.py
# *  @brief Logger utility.
# *
# *  @author Antonin CHAUVET
# *  @date   12/05/2025
# ******************************************************************************
import logging
import logging.config
import os
from datetime import date
from pathlib import Path


# ======================
# Filter
# ======================

class LevelFilter(logging.Filter):
    def __init__(self, levels):
        super().__init__()
        if isinstance(levels, int):
            self.levels = {levels}
        else:
            self.levels = set(levels)

    def filter(self, record):
        return record.levelno in self.levels


BASE_DIR = Path(__file__).resolve().parent
LOG_DIR = str(BASE_DIR.parents[1]) + os.sep + 'logs'
os.makedirs(LOG_DIR, exist_ok=True)

# Get today's date
today = date.today()

# Format the date (ex: YYYY-MM-DD)
dateStr = today.strftime("%Y-%m-%d")

logFile = f"robot_{dateStr}.log"
logFilePath = os.path.join(LOG_DIR, logFile)

log_file_path = os.path.join(LOG_DIR, logFilePath)


def createLogger(name: str, console_levels=logging.DEBUG, file_levels=logging.DEBUG):
    logger = logging.getLogger(name)

    if not logger.handlers:  # avoids duplicates
        logger.setLevel(logging.DEBUG)
        logger.propagate = False

        formatter = logging.Formatter('%(asctime)s | %(levelname)s | %(name)s | '
                                      'file: %(filename)s | function: %(funcName)s | '
                                      'line: %(lineno)d %(message)s')

        # Console
        streamHandler = logging.StreamHandler()
        streamHandler.setFormatter(formatter)
        streamHandler.addFilter(LevelFilter(console_levels))
        logger.addHandler(streamHandler)

        # File
        fileHandler = logging.FileHandler(logFilePath)
        fileHandler.setFormatter(formatter)
        fileHandler.addFilter(LevelFilter(file_levels))
        logger.addHandler(fileHandler)

    return logger
