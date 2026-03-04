import requests


class MotionRobotClient:
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
        self.model = None


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

    
    def init_robot(self, model="vs060", planning_group="arm", velocity_scale=0.1, accel_scale=0.1, planning_time=5.0, planning_attempts=10, allow_replanning=True, planner_id="PRMstar"):
        """
        Initializes the robot on the ROS side (MoveIt). Must be called once at startup.

        Examples:
            ret = robot.init_robot(
                model="vp5243", 
                planning_group="arm", 
                velocity_scale=0.2, 
                planning_time=10.0,
                planner_id="RRTstar"
            )

        Args:
            model (str): Robot model name (e.g., "vs060", "vp5243").
            planning_group (str): MoveIt planning group name (e.g., "arm").
            velocity_scale (float): Global velocity scaling factor (0.0 to 1.0).
            accel_scale (float): Global acceleration scaling factor (0.0 to 1.0).
            planning_time (float): Maximum time (in seconds) allowed for the solver to compute the path.
            planning_attempts (int): Number of solver attempts (with different random seeds) before failing.
            allow_replanning (bool): If True, MoveIt will attempt to replan a path on the fly if an obstacle appears.
            planner_id (str): Identifier of the OMPL planning algorithm to use. 
                Here are the most relevant choices:
                
                -- Optimizing Planners (Smooth and short trajectories) --
                * "RRTstar"   : Excellent for smooth and direct movements. It uses the entire 'planning_time' to refine and shorten the path as much as possible. No more useless contortions!
                * "PRMstar"   : Very powerful in confined environments or with many obstacles (like your virtual cage). It pre-calculates a roadmap of possible movements.
                * "FMT"       : (Fast Marching Tree) A modern algorithm, very fast to converge towards an optimal solution without making detours.
                
                -- Fast Planners (First found path = validated) --
                * "RRTConnect": MoveIt's default algorithm. Ultra-fast (often < 0.1s), but very erratic. It can cause the robot to make large detours or strange wrist rotations.
                * "BiTRRT"    : A good compromise. It is fast like RRTConnect, but incorporates a slight notion of optimization to avoid overly absurd movements.

        Returns:
            dict: Initialization result containing 'success' (bool) and 'message' (str).
        """
        payload = {
            "model": model,
            "planning_group": planning_group,
            "velocity_scale": float(velocity_scale),
            "accel_scale": float(accel_scale),
            "planning_time": float(planning_time),
            "planning_attempts": int(planning_attempts),
            "allow_replanning": bool(allow_replanning),
            "planner_id": str(planner_id)
        }
        r = requests.post(f"{self.base_url}/init", json=payload, timeout=self.timeout)
        r.raise_for_status()
        self.get_solver()
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

    def move_joints(self, joints, is_relative=False, execute=True):
        """
        Commands a movement to target joint angles.

        Args:
            joints (list[float]): List of target angles in radians (must match the number of axes).
            is_relative (bool): If True, adds the angles to the current position. If False, goes to absolute angles.
            execute (bool): If True, physically moves the robot. If False, only plans.
        """
        payload = {
            "joints": [float(x) for x in joints], 
            "is_relative": bool(is_relative),
            "execute": bool(execute)
        }
        current_timeout = 120.0 if execute else self.timeout
    
        r = requests.post(f"{self.base_url}/move_joints", json=payload, timeout=current_timeout)
        r.raise_for_status()
        return r.json()

    def move_to_pose(self, x, y, z, r1, r2, r3, r4=0.0, rotation_format="RPY", reference_frame="WORLD", is_relative=False, cartesian_path=False, execute=True):
        """
        The universal function for point-to-point Cartesian movement.

        Examples:
            # 1. Absolute Move in World (Euler)
            robot.move_to_pose(0.5, 0.0, 0.4, 3.14/2, 0.0, 0.0, rotation_format="RPY", reference_frame="WORLD")

            # 2. Relative Move in Tool frame (Fly-by-wire: advance 10cm on Z)
            robot.move_to_pose(0.0, 0.0, 0.10, 0.0, 0.0, 0.0, rotation_format="RPY", reference_frame="TOOL", is_relative=True, cartesian_path=True)

            # 3. Absolute Move with Quaternion
            robot.move_to_pose(0.4, 0.0, 0.4, 0.0, 1.0, 0.0, 0.0, rotation_format="QUAT")

        Args:
            x, y, z (float): Translation.
            r1, r2, r3, r4 (float): Rotation (r4 is ignored if format is RPY).
            rotation_format (str): "RPY" (Roll, Pitch, Yaw) or "QUAT" (x, y, z, w).
            reference_frame (str): "WORLD" or "TOOL".
            is_relative (bool): True = Delta from current pos, False = Absolute target.
            cartesian_path (bool): True = Strict straight line, False = Fluid joint-space path.
            execute (bool): Execute or simply plan.
        """
        payload = {
            "x": float(x), "y": float(y), "z": float(z),
            "r1": float(r1), "r2": float(r2), "r3": float(r3), "r4": float(r4),
            "rotation_format": str(rotation_format),
            "reference_frame": str(reference_frame),
            "is_relative": bool(is_relative),
            "cartesian_path": bool(cartesian_path),
            "execute": bool(execute)
        }
        current_timeout = 120.0 if execute else self.timeout

        r = requests.post(f"{self.base_url}/move_to_pose", json=payload, timeout=current_timeout)
        r.raise_for_status()
        return r.json()

    def move_waypoints(self, waypoints, rotation_format="RPY", reference_frame="WORLD", is_relative=False, cartesian_path=True, execute=True):
        """
        Moves the robot through a list of points without stopping.

        Examples:
            points = [
                {"x": 0.1, "y": 0.0, "z": 0.0, "r1": 0.0, "r2": 0.0, "r3": 0.0}, # +10cm X
                {"x": 0.0, "y": 0.1, "z": 0.0, "r1": 0.0, "r2": 0.0, "r3": 0.0}, # then +10cm Y
            ]
            robot.move_waypoints(points, is_relative=True, reference_frame="TOOL")

        Args:
            waypoints (list[dict]): List of dictionaries with keys x, y, z, r1, r2, r3, (r4).
            rotation_format (str): "RPY" or "QUAT" for all points.
            reference_frame (str): "WORLD" or "TOOL".
            is_relative (bool): If True, Point 1 is relative to start, Point N is relative to Point N-1.
            cartesian_path (bool): True = straight lines between points.
            execute (bool): Execute or simply plan.
        """
        payload = {
            "waypoints": waypoints,
            "rotation_format": str(rotation_format),
            "reference_frame": str(reference_frame),
            "is_relative": bool(is_relative),
            "cartesian_path": bool(cartesian_path),
            "execute": bool(execute)
        }
        current_timeout = 120.0 if execute else self.timeout

        r = requests.post(f"{self.base_url}/move_waypoints", json=payload, timeout=current_timeout)
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
    
    def get_current_pose(self, frame_id=None, child_frame_id=None, output_format="euler"):
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
    

    def move_to_home(self):
        home_position = []
        if self.model == "vs060":
            home_position = [0.0, 0.0, 1.57, 0.0, 1.57, 0.0]
        if self.model == "vp5243":
            home_position = [0.0, 0.0, 1.57, 1.57, 0.0]
        return self.move_joints(home_position, is_relative=False)

    def set_virtual_cage(self, enable=True, front=0.8, back=0.8, left=0.8, right=0.8, top=1.2, bottom=0.0, r=0.0, g=0.6, b=1.0, a=0.15):
        """
        Enables or disables a virtual collision cage around the robot.
        Distances are measured in meters from the world's zero point.

        Args:
        enable(bool): Enables or disables the cage.
        front(float): Maximum distance forward (+X).
        back(float): Maximum distance backward (-X).
        left(float): Maximum distance left (+Y).
        right(float): Maximum distance right (-Y).
        top(float): Maximum height (+Z).
        bottom(float): Maximum depth (-Z).
        r, g, b(float): Color of the cage in RGB (0.0 to 1.0).
        a(float): Alpha (transparency) of the cage (0.0 to 1.0).
        """
        payload = {
            "enable": bool(enable),
            "front": float(front), "back": float(back),
            "left": float(left), "right": float(right),
            "top": float(top), "bottom": float(bottom),
            "r": float(r), "g": float(g), "b": float(b), "a": float(a)
        }
        r = requests.post(f"{self.base_url}/set_virtual_cage", json=payload, timeout=self.timeout)
        r.raise_for_status()
        return r.json()

    def get_solver(self):
        """
        Retrieves the currently active Kinematic Solver (IK) used by MoveIt.

        Examples:
            ret = robot.get_solver()
            print(f"Active solver: {ret.get('solver')}")
            # Output: Active solver: pick_ik

        Returns:
            dict: Contains 'success', 'solver' (short name), and 'full_plugin_name'.
        """
        r = requests.get(f"{self.base_url}/state/solver", timeout=self.timeout)
        r.raise_for_status()
        return r.json()

    def move_approach(self, x, y, z, r1, r2, r3, r4=0.0, rotation_format="RPY", z_offset=0.1, cartesian_path=False, execute=True):
        """
        Asks the Linux ROS server to calculate and execute an approach position above an object.
        
        Args:
            x, y, z (float): Position of the object (final target).
            r1, r2, r3, r4 (float): Desired orientation of the tool.
            rotation_format (str): "RPY" or "QUAT".
            z_offset (float): Retreat distance in meters (e.g., 0.1 for 10 cm above).
            cartesian_path (bool): True = straight line, False = joint space path.
            execute (bool): True = execute motion, False = plan only.
            
        Returns:
            dict: The response from the motion server.
        """
        payload = {
            "x": float(x), "y": float(y), "z": float(z),
            "r1": float(r1), "r2": float(r2), "r3": float(r3), "r4": float(r4),
            "rotation_format": str(rotation_format),
            "z_offset": float(z_offset),
            "cartesian_path": bool(cartesian_path),
            "execute": bool(execute)
        }
        current_timeout = 120.0 if execute else self.timeout

        r = requests.post(f"{self.base_url}/move_approach", json=payload, timeout=current_timeout)
        r.raise_for_status()
        return r.json()
    
    def manage_box(self, box_id, x=0.0, y=0.0, z=0.0, r1=0.0, r2=0.0, r3=0.0, r4=0.0, rotation_format="RPY", size_x=0.1, size_y=0.1, size_z=0.1, action="ADD"):
        """
        Adds or removes a collision box in MoveIt.
        If adding, the coordinates provided should be the TOP SURFACE center of the box, 
        where the robot will grasp. The box center will be automatically calculated downward.

        Args:
            box_id (str): Unique name for the object (e.g., "target_cube").
            x, y, z (float): Position of the grasp point.
            r1, r2, r3, r4 (float): Orientation of the grasp point.
            size_x, size_y, size_z (float): Dimensions of the box in meters.
            action (str): "ADD" to spawn the box, "REMOVE" to delete it.
        """
        payload = {
            "box_id": str(box_id),
            "x": float(x), "y": float(y), "z": float(z),
            "r1": float(r1), "r2": float(r2), "r3": float(r3), "r4": float(r4),
            "rotation_format": str(rotation_format),
            "size_x": float(size_x), "size_y": float(size_y), "size_z": float(size_z),
            "action": str(action).upper()
        }
        r = requests.post(f"{self.base_url}/manage_box", json=payload, timeout=self.timeout)
        r.raise_for_status()
        return r.json()