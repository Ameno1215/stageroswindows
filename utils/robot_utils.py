# ******************************************************************************
# *  ROBOt UTILS
# *
# *  @file  robot_utils.py
# *  @brief Applications utility.
# *
# *  @author Antonin CHAUVET
# *  @date   12/05/2025
# ******************************************************************************
import sys
import os

c_currentFile = os.path.abspath(__file__)
c_projectRootPath = os.path.abspath(os.path.join(os.path.dirname(c_currentFile), "..", ".."))
if not c_projectRootPath in sys.path:
    sys.path.append(c_projectRootPath)

from lib.robotcontroller.constants.robot_constants import RobotConstants

def getErrorMessageFromErrorCode(errorCode: int) -> str:
    """
    Retrieves the error message based on the error code.
    :param errorCode: the error code.
    :return: the error message.
    """
    errorCodeStr = str(errorCode)
    return RobotConstants.ERROR_MESSAGES_DICT[errorCodeStr]


def getLimitsCoordinatesFromString(limitsCoordinatesStr: str, extMode: int) -> {}:
    """
    Get limits coordinates in 1/10 mm from string coordinates.
    :param limitsCoordinatesStr: the string coordinates.
    :param extMode: 0 if extMode is not activate, 1 if it is
    :return: a dictionary contenaining coordiantes in 1/10 mm with key:
        - xMin,
        - xMax,
        - yMin,
        - yMax,
        - zMin.
        None in case of error.
        """
    coordinatesArray = limitsCoordinatesStr.split(',')
    if len(coordinatesArray) == 5:
        coordinatesDict = {RobotConstants.X_MIN: int(coordinatesArray[0].strip()),
                           RobotConstants.X_MAX: int(coordinatesArray[1].strip()),
                           RobotConstants.Y_MIN: int(coordinatesArray[2].strip()),
                           RobotConstants.Y_MAX: int(coordinatesArray[3].strip()),
                           RobotConstants.Z_MIN: int(coordinatesArray[4].strip())}

        if not extMode:
            coordinatesDict[RobotConstants.X_MIN] = int(coordinatesDict[RobotConstants.X_MIN]) * 10
            coordinatesDict[RobotConstants.X_MAX] = int(coordinatesDict[RobotConstants.X_MAX]) * 10
            coordinatesDict[RobotConstants.Y_MIN] = int(coordinatesDict[RobotConstants.Y_MIN]) * 10
            coordinatesDict[RobotConstants.Y_MAX] = int(coordinatesDict[RobotConstants.Y_MAX]) * 10
            coordinatesDict[RobotConstants.Z_MIN] = int(coordinatesDict[RobotConstants.Z_MIN]) * 10

        return coordinatesDict
    else:
        return None


def checkLimits(limits1: dict, limits2: dict) -> int:
    """
    Compare limits.
    :param limits1: first limts
    :param limits2: second limits
    :return: 0 if the limits are equal otherwise false.
    """
    if limits1[RobotConstants.X_MIN] != limits2[RobotConstants.X_MIN]:
        return -1
    if limits1[RobotConstants.X_MAX] != limits2[RobotConstants.X_MAX]:
        return -1
    if limits1[RobotConstants.Y_MIN] != limits2[RobotConstants.Y_MIN]:
        return -1
    if limits1[RobotConstants.Y_MAX] != limits2[RobotConstants.Y_MAX]:
        return -1
    if limits1[RobotConstants.Z_MIN] != limits2[RobotConstants.Z_MIN]:
        return -1

    return 0

