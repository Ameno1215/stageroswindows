# ******************************************************************************
# *  ROBOT PARAMETERS MANAGER
# *
# *  @file  robot_parameters_manager.py
# *  @brief Manages the file containing the robot parameters and allows them to be extracted.
# *
# *  @author Antonin CHAUVET
# *  @date   12/05/2026
# ******************************************************************************
import json
import sys
import os

c_currentFile = os.path.abspath(__file__)
c_projectRootPath = os.path.abspath(os.path.join(os.path.dirname(c_currentFile), "..", ".."))
if not c_projectRootPath in sys.path:
    sys.path.append(c_projectRootPath)
from lib.robotcontroller.utils.logger_utils import createLogger
from lib.robotcontroller.constants.robot_constants import RobotConstants
from pathlib import Path


class RobotParametersManager:

    def __init__(self):
        self.logger = createLogger(__name__)
        baseDir = Path(__file__).resolve().parent
        self.fileParamsPath = str(
            baseDir.parents[1]) + os.sep + 'robotcontroller' + os.sep + 'resources' + os.sep + 'robot_parameters.json'
        self.jsonData = None

        self.__loadFileParams()

    def __loadFileParams(self):

        try:
            with open(self.fileParamsPath, 'r') as jsonFile:
                # Load the JSON content from the file
                self.jsonData = json.load(jsonFile)

        except FileNotFoundError as err:
            print(f"File {self.fileParamsPath} not found: {err}")
            self.logger.error(f"File {self.fileParamsPath} not found: {err}")
        except json.JSONDecodeError as err:
            print(f"Error while decoding json file {self.fileParamsPath}: {err}")
            self.logger.error(f"Error while decoding json file {self.fileParamsPath}: {err}")

    def getLimits(self) -> {}:
        return self.jsonData[RobotConstants.POSITION][RobotConstants.LIMITS]

    def getIpAddress(self) -> str:
        return self.jsonData[RobotConstants.IP_ADDRESS]
    
    def getRobotSupported(self) -> list[str]:
        return self.jsonData[RobotConstants.ROBOT_SUPPORTED]
    
    def getRosRobotSupported(self) -> list[str]:
        return self.jsonData[RobotConstants.ROS_ROBOT_SUPPORTED]

    def getRobotNameToRosMapping(self, robotName: str) -> str:
        robotNames = self.getRobotSupported()
        rosNames = self.getRosRobotSupported()
        if len(robotNames) != len(rosNames):
            raise ValueError(
                "robot_supported and ros_robot_supported must have the same length"
            )
        mapping = dict(zip(robotNames, rosNames))
        if robotName not in mapping:
            raise KeyError(f"Unknown robot name: {robotName!r}. Supported: {list(mapping)}")
        return mapping[robotName]

    def getHomeJoint(self) -> {}:
        return self.jsonData[RobotConstants.POSITION][RobotConstants.HOME][RobotConstants.JOINT]

    def getHomeCartesian(self) -> {}:
        return self.jsonData[RobotConstants.POSITION][RobotConstants.HOME][RobotConstants.CARTESIAN]

    def getSpeed(self) -> {}:
        return self.jsonData[RobotConstants.SPEED]

    def getToolParam(self) -> int:
        return self.jsonData[RobotConstants.TOOL]

    def getZSafePosition(self):
        return self.jsonData[RobotConstants.POSITION][RobotConstants.Z_SAFE]
