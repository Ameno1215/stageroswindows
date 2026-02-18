import requests


class DensoRobotClient:
    def __init__(self, base_url="http://localhost:8000", timeout=60.0):
        """
        Initializes the Denso client.
        
        Examples:
            robot = DensoRobotClient("http://localhost:8000")
        
        Args:
            base_url (str): The URL of the wsl_ros_bridge server.
            timeout (float): Default HTTP request timeout in seconds.
        """
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout


    def health(self):
        """
        Checks if the bridge server is online.

        Examples:
            ret = robot.health()

        Returns:
            dict: {"ok": True} if the server responds.
        """
        r = requests.get(f"{self.base_url}/health", timeout=self.timeout)
        r.raise_for_status()
        return r.json()

    def init_robot(self, model="vs060", planning_group="arm", velocity_scale=0.1, accel_scale=0.1):
        """
        Initializes the robot on the ROS side (MoveIt). Must be called once at startup.

        Examples:
            ret = robot.init_robot(model="vs060", planning_group="arm", velocity_scale=0.1, accel_scale=0.1)

        Args:
            model (str): Robot model name (e.g., "vs060", "cobotta").
            planning_group (str): MoveIt planning group name (e.g., "arm").
            velocity_scale (float): Initial velocity scaling factor (0.0 to 1.0).
            accel_scale (float): Initial acceleration scaling factor (0.0 to 1.0).

        Returns:
            dict: Initialization result (success, message).
        """
        payload = {
            "model": model,
            "planning_group": planning_group,
            "velocity_scale": float(velocity_scale),
            "accel_scale": float(accel_scale),
        }
        r = requests.post(f"{self.base_url}/init", json=payload, timeout=self.timeout)
        r.raise_for_status()
        return r.json()

    def set_scaling(self, velocity_scale, accel_scale):
        """
        Updates velocity and acceleration scaling factors for future movements.

        Examples:
            ret = robot.set_scaling(velocity_scale=0.5, accel_scale=0.5)

        Args:
            velocity_scale (float): Velocity factor (0.0 to 1.0).
            accel_scale (float): Acceleration factor (0.0 to 1.0).

        Returns:
            dict: Update status.
        """
        payload = {
            "velocity_scale": float(velocity_scale),
            "accel_scale": float(accel_scale),
        }
        r = requests.post(f"{self.base_url}/scaling", json=payload, timeout=self.timeout)
        r.raise_for_status()
        return r.json()
   
    def goto_joint(self, joints, execute=True):
        """
        Commands a movement to a target joint configuration.

        Examples:
            #### Move all 6 axes to 0.0 radians
            ret = robot.goto_joint([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], execute=True)

        Args:
            joints (list[float]): List of target angles in radians (must match the number of axes).
            execute (bool): If True, physically moves the robot. If False, only plans.

        Returns:
            dict: Movement status (success, message).
        """
        payload = {"joints": [float(x) for x in joints], "execute": bool(execute)}
        current_timeout = 120.0 if execute else self.timeout
    
        r = requests.post(f"{self.base_url}/goto_joint", json=payload, timeout=current_timeout)
        r.raise_for_status()
        return r.json()

    def goto_pose(self, frame_id, x, y, z, qx, qy, qz, qw, execute=True):
        """
        Commands a movement to a target Cartesian pose (Position + Orientation).

        Examples:
            ret = robot.goto_pose(
                    frame_id="base_link",
                    x=0.4, y=0.0, z=0.4,
                    qx=0.0, qy=1.0, qz=0.0, qw=0.0,
                    execute=True
                    )

        Args:
            frame_id (str): Reference frame (e.g., "base_link" or "world").
            x, y, z (float): Target position in meters.
            qx, qy, qz, qw (float): Target orientation (Quaternion).
            execute (bool): If True, physically moves the robot.

        Returns:
            dict: Movement status.
        """
        payload = {
            "frame_id": frame_id,
            "position": {"x": float(x), "y": float(y), "z": float(z)},
            "orientation": {"x": float(qx), "y": float(qy), "z": float(qz), "w": float(qw)},
            "execute": bool(execute),
        }
        current_timeout = 120.0 if execute else self.timeout

        r = requests.post(f"{self.base_url}/goto_pose", json=payload, timeout=self.timeout)
        r.raise_for_status()
        return r.json()
    
    def goto_euler_world(self, x, y, z, rx, ry, rz, frame_id="base_link", execute=True):
        """
        Moves the robot to an absolute position with Euler orientation (Extrinsic XYZ).
        
        Examples:
            ret = robot.goto_euler_world(x=0.5, y=0.0, z=0.4, rx=3.14/2, ry=0, rz=0.0)

        Args:
            x, y, z (float): Target position (meters)
            rx, ry, rz (float): Target rotation in radians (World Fixed Axes)
            frame_id (str): Reference frame ("base_link" or "world")
            execute (bool): Execute or just schedule
        """
        payload = {
            "frame_id": frame_id,
            "x": float(x), "y": float(y), "z": float(z),
            "rx": float(rx), "ry": float(ry), "rz": float(rz),
            "execute": bool(execute)
        }
        # longer timeout for execution since it may take time to move, while planning should be quick
        current_timeout = 120.0 if execute else self.timeout
        r = requests.post(f"{self.base_url}/goto_euler_world", json=payload, timeout=current_timeout)
        r.raise_for_status()
        return r.json()

    def move_relative_tool(self, dx, dy, dz, drx, dry, drz, execute=True):
        """
        Moves the robot RELATIVELY to its tool (Airplane Mode).

        Examples:
            ret = robot.move_relative_tool(dx=0, dy=0, dz=0.1, drx=0, dry=0, drz=0)

        Args:
        dx, dy, dz (float): Movement in meters (X=Forward/Backward, Y=Left/Right, Z=Top/Bottom of the tool)
        drx, dry, drz (float): Rotation in radians around the tool axes
        execute (bool): Execute or simply plan
        """
        payload = {
            "frame_id": "ignored", # The frame is ignored because it is relative.
            "x": float(dx), "y": float(dy), "z": float(dz),
            "rx": float(drx), "ry": float(dry), "rz": float(drz),
            "execute": bool(execute)
        }
        current_timeout = 120.0 if execute else self.timeout
        r = requests.post(f"{self.base_url}/move_relative_tool", json=payload, timeout=current_timeout)
        r.raise_for_status()
        return r.json()
    
    def move_relative_world(self, dx, dy, dz, drx, dry, drz, execute=True):
        """
        Moves the robot RELATIVELY to the WORLD (Crane).
        
        Examples:
            ret = robot.move_relative_world(dx=0, dy=0, dz=0.1, drx=0, dry=0, drz=0, execute=True)
        Args:
            dx, dy, dz (float): Movement in meters on the world axes (X, Y, Z)
            drx, dry, drz (float): Rotation in radians around the fixed world axes (RPY)
            execute (bool): Execute or just plan
        """
        payload = {
            "frame_id": "world",
            "x": float(dx), "y": float(dy), "z": float(dz),
            "rx": float(drx), "ry": float(dry), "rz": float(drz),
            "execute": bool(execute)
        }
        current_timeout = 120.0 if execute else self.timeout
        r = requests.post(f"{self.base_url}/move_relative_world", json=payload, timeout=current_timeout)
        r.raise_for_status()
        return r.json()
    
    def get_joint_state(self):
        """
        Retrieves the current joint angles of the robot.

        Examples:
            ret = robot.get_joint_state()

        Returns:
            dict: Contains the 'joints' list with angles in radians.
        """
        r = requests.get(f"{self.base_url}/state/joints", timeout=self.timeout)
        r.raise_for_status()
        return r.json()
    
    def get_current_pose(self, frame_id=None, child_frame_id=None, output_format="quaternion"):
        """
        Retrieves the current Cartesian pose (Position + Orientation).
        Uses TF2 to calculate the transform between two frames.

        Examples:
            #### 1. Get standard ROS Pose (Quaternion) - Default
            ret = robot.get_current_pose(output_format="quaternion")

            #### 2. Get Euler Angles (easier to read)
            ret = robot.get_current_pose(output_format="euler")

            #### 3. Get Both formats
            ret = robot.get_current_pose(output_format="both")

        Args:
            frame_id (str, optional): The reference frame (Origin). Defaults to "world" or "base_link".
            child_frame_id (str, optional): The target frame. Defaults to End-Effector.
            output_format (str, optional): The desired orientation format.
                - "quaternion": Returns x, y, z, w (Standard).
                - "euler": Returns rx, ry, rz (Radians, Fixed XYZ).
                - "both": Returns a dictionary containing both formats.

        Returns:
            dict: The pose with the requested orientation format.
        
        Raises:
            ValueError: If output_format is not 'quaternion', 'euler', or 'both'.
        """
        # Construct URL parameters
        params = {}
        if frame_id:
            params["frame_id"] = frame_id
        if child_frame_id:
            params["child_frame_id"] = child_frame_id

        # The server returns EVERYTHING (pos + quaternion + euler)
        r = requests.get(f"{self.base_url}/state/pose", params=params, timeout=self.timeout)
        r.raise_for_status()
        raw_data = r.json()

        # If the request failed on the server side, return the error immediately
        if not raw_data.get("success"):
            return raw_data
        
        # Build the base response
        result = {
            "success": raw_data["success"],
            "message": raw_data["message"],
            "frame_id": raw_data["frame_id"],
            "child_frame_id": raw_data["child_frame_id"],
            "position": raw_data["position"]
        }

        # Select the orientation format explicitly
        if output_format == "euler":
            result["orientation"] = raw_data["orientation_euler"]
        
        elif output_format == "quaternion":
            result["orientation"] = raw_data["orientation_quat"]
            
        elif output_format == "both":
            result["orientation"] = {
                "quaternion": raw_data["orientation_quat"],
                "euler": raw_data["orientation_euler"]
            }
        
        else: 
            # RAISE ERROR instead of default behavior
            raise ValueError(f"Invalid output_format '{output_format}'. Must be 'quaternion', 'euler', or 'both'.")

        return result
