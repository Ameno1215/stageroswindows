#******************************************************************************
#*  ROBOT APPLICATION CONSTANTS
#*
#*  @file  robot_constants.py
#*  @brief Applications constants.
#*
#*  @author Antonin CHAUVET
#*  @date   12/05/2026
#******************************************************************************
from dataclasses import dataclass

@dataclass(frozen=True)
class RobotConstants:
    # json parameters constants
    POSITION = "positions"
    IP_ADDRESS = "ip_address"
    ROBOT_SUPPORTED = "robot_supported"
    ROS_ROBOT_SUPPORTED = "ros_robot_supported"
    
    HOME = "home"
    JOINT = "joint"
    CARTESIAN = "cartesian"
    X_POSITION = "x_position"
    Y_POSITION = "y_position"
    Z_POSITION = "z_position"
    X_ROTATION = "x_rotation"
    Y_ROTATION = "y_rotation"
    Z_ROTATION = "z_rotation"
    JOINT_1 = "joint_1"
    JOINT_2 = "joint_2"
    JOINT_3 = "joint_3"
    JOINT_4 = "joint_4"
    JOINT_5 = "joint_5"
    JOINT_6 = "joint_6"
    LIMITS = "limits"
    X_MIN = "x_min"
    X_MAX = "x_max"
    Y_MIN = "y_min"
    Y_MAX = "y_max"
    Z_MIN = "z_min"
    Z_MAX = "z_max"
    SPEED = "speed"
    DEFAULT_SPEED = "default_speed"
    MAX_SPEED = "max_speed"
    SLOW_SPEED = "slow_speed"
    MIN_SPEED = "min_speed"
    DEFAULT_ACCELERATION = "default_acceleration"
    MAX_ACCELERATION = "max_acceleration"
    MIN_ACCELERATION = "min_acceleration"
    TOOL = "tool"
    Z_SAFE = "z_safe"

    # RoboX error codes
    ERR_OK = "0"
    ERR_COMMERROR = "1"
    ERR_ROBOT = "2"
    ERR_LIMIT = "3"
    ERR_TOOLPARAM = "4"
    ERR_VACUUM = "5"
    ERR_SOFT = "6"
    ERR_TIMEOUT = "7"
    ERR_NOT_CALIBRATED = "8"
    ERR_STOP = "9"
    ERR_ABORT = "10"
    ERR_OSOLETE = "11"
    ERR_NOT_IMPLEMENTED = "12"

    # RoboX specific error codes
    ERR_ROBOT_MOVE = "100"
    ERR_ROBOT_PARAM = "101"
    ERR_ROBOT_SYSTEM = "102"
    ERR_ROBOT_SOFT = "103"
    ERR_ROBOT_LIMITS = "104"
    ERR_ROBOT_EMERGENCY_STOP = "105"
    ERR_ROBOT_INIT = "106"

    # Mapping error codes - error messages
    ERROR_MESSAGES_DICT = {
        ERR_OK: "No error.",
        ERR_COMMERROR: "A communication error occurred.",
        ERR_ROBOT: "The robot aborted the command, or did not answer before time-out expired. This may happen in case of invalid coordinates.",
        ERR_LIMIT: "An error occurred during the call of the “GetLimits” function, or, the given parameters are not correct.",
        ERR_TOOLPARAM: "An error occurred during the call of the “GetToolParam” function, or the given parameters to a function are not correct.",
        ERR_VACUUM: "An error occurred during the call of the “GetVaccum” function.",
        ERR_SOFT: "An internal error occurred in the software. Most of the time, when trying to send back an out parameter.",
        ERR_TIMEOUT: "A timeout has occurred during movement.",
        ERR_NOT_CALIBRATED: "A movement was requested before the arm was calibrated.",
        ERR_STOP: "The Stop button was pushed or the door opened.",
        ERR_ABORT: "The previous command was aborted.",
        ERR_OSOLETE: "The called function is obsolete.",
        ERR_NOT_IMPLEMENTED: "The called function is not implemented.",
        ERR_ROBOT_MOVE: "Movement error: the target cannot be reached by the robot.",
        ERR_ROBOT_PARAM: "Parameter error: the given parameters are not correct.",
        ERR_ROBOT_SYSTEM: "System error returned by the robot. Please contact support.",
        ERR_ROBOT_SOFT: "Soft error returned by the robot. Please contact support.",
        ERR_ROBOT_LIMITS: "Limits error: the robot has reached the predefined limits.",
        ERR_ROBOT_EMERGENCY_STOP: "Emergency stop error.",
        ERR_ROBOT_INIT: "Initialization error: the robot cannot be powered or initialized. Please contact support."
    }


