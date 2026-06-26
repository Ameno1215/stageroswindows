import requests

class RobotErrorException(Exception):
    """Raised when the motion server reports a failed operation."""


class RobotController:
    def __init__(self, base_url="http://localhost:8000", sim=True, timeout=60.0):
        """
        Initializes the motion client (HTTP bridge to the ROS 2 motion_server).

        Examples:
            robot = MotionRobotClient("http://localhost:8000")
            robot = MotionRobotClient("http://localhost:8000", sim=False, timeout=120.0)

        Args:
            base_url (str): The URL of the wsl_ros_bridge server.
            sim (bool): True = simulation mode.
                False = real hardware. Forwarded to the server on init_robot().
            timeout (float): Default HTTP request timeout in seconds.
        """
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout
        self.model = None
        self.sim = sim
        self.session = requests.Session()
        self.session.trust_env = False

    def _check(self, result: dict) -> dict:
        """Raises RuntimeError if the motion server returned a failure."""
        if not result.get("success"):
            raise RuntimeError(f"Motion server failure: {result.get('message', 'unknown error')}")
        return result

    def health(self):
        """
        Checks if the bridge server is online.

        Examples:
            ret = robot.health()

        Returns:
            dict: {"ok": True} if the server responds.
        """
        r = self.session.get(f"{self.base_url}/health", timeout=self.timeout)
        r.raise_for_status()
        return r.json()

    def init_robot(self, model="vs060", velocity_scale=0.1, accel_scale=0.1, planning_time=5.0, planning_attempts=10, allow_replanning=True, planner_id="RRTConnect"):
        """
        Initializes the robot on the ROS side (MoveIt). Must be called once at startup.

        Examples:
            ret = robot.init_robot(
                model="vp5243",
                velocity_scale=0.2,
                planning_time=10.0,
                planner_id="RRTConnect"
            )

        Args:
            model (str): Robot model name (e.g., "vs060", "vp5243").
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
            "sim": self.sim,
            "velocity_scale": float(velocity_scale),
            "accel_scale": float(accel_scale),
            "planning_time": float(planning_time),
            "planning_attempts": int(planning_attempts),
            "allow_replanning": bool(allow_replanning),
            "planner_id": str(planner_id)
        }
        r = self.session.post(f"{self.base_url}/init", json=payload, timeout=self.timeout)
        r.raise_for_status()
        self.get_solver()
        self.model = model
        return self._check(r.json())

    def set_scaling(self, velocity_scale, accel_scale, cartesian_speed=0.0):
        """
        Updates the velocity / acceleration scaling factors for future movements.

        Three independent knobs:
        - velocity_scale : joint-space velocity scaling (OMPL / move_joints / move_to_pose).
        - accel_scale    : acceleration scaling (all moves).
        - cartesian_speed: velocity scaling for Cartesian (Pilz LIN/Sequence) moves only,
                            i.e. move_to_pose(cartesian_path=True) and move_waypoints.
                            0.0 = use velocity_scale for Cartesian moves too.

        Examples:
            # Same speed everywhere
            robot.set_scaling(velocity_scale=0.5, accel_scale=0.5)

            # Fast joint moves, but slow precise straight-line Cartesian moves
            robot.set_scaling(velocity_scale=0.8, accel_scale=0.5, cartesian_speed=0.15)

        Args:
            velocity_scale (float): Joint velocity factor (0.0 to 1.0).
            accel_scale (float): Acceleration factor (0.0 to 1.0).
            cartesian_speed (float): Cartesian velocity factor (0.0 to 1.0).
                0.0 means "fall back to velocity_scale" for Cartesian moves.

        Returns:
            dict: Update status ('success', 'message').
        """
        payload = {
            "velocity_scale": float(velocity_scale),
            "accel_scale": float(accel_scale),
            "cartesian_speed": float(cartesian_speed),
        }
        r = self.session.post(f"{self.base_url}/scaling", json=payload, timeout=self.timeout)
        r.raise_for_status()
        return self._check(r.json())
        
    def get_scaling(self):
        """
        Retrieves the current velocity and acceleration scaling factors.

        Examples:
            ret = robot.get_scaling()
            print(f"Velocity: {ret['velocity_scale']}, Accel: {ret['accel_scale']}")

        Returns:
            dict: Contains 'success' (bool), 'message' (str), 'velocity_scale' (float), and 'accel_scale' (float).
        """
        r = self.session.get(f"{self.base_url}/scaling", timeout=self.timeout)
        r.raise_for_status()
        return self._check(r.json())

    def move_joints(self, joints, joint_constraints=None, angle_format="RAD", is_relative=False, execute=True):
        """
        Commands a movement to target joint angles (joint-space planning).

        Examples:
            # Absolute joint target (radians)
            robot.move_joints([0.0, 0.0, 1.57, 0.0, 1.57, 0.0])

            # Relative offset in degrees
            robot.move_joints([10, 0, 0, 0, 0, 0], angle_format="DEG", is_relative=True)

            # With a joint constraint restricting the planning amplitude
            robot.move_joints(
                [0.3, 0.0, 1.57, 0.0, 1.3, -0.5],
                joint_constraints=[{"joint_name": "joint_2", "min": -0.3, "max": 0.3}],
            )

        Args:
            joints (list[float]): List of target angles in radians or in degrees (must match the number of axes).
            joint_constraints (list[dict], optional): List of joint constraints. Each dict:
                {"joint_name": str, "min": float, "max": float, "relative": bool}.
                Bounds use the same angle_format as `joints`. If "relative" is True, the
                [min, max] window is applied around the current joint value.
            angle_format (str): Angle format, "RAD" or "DEG".
            is_relative (bool): If True, adds the angles to the current position. If False, goes to absolute angles.
            execute (bool): If True, physically moves the robot. If False, only plans.

        Returns:
            dict: Contains 'success' (bool) and 'message' (str).
        """
        payload = {
            "joints": [float(x) for x in joints],
            "joint_constraints": joint_constraints or [],
            "angle_format": str(angle_format),
            "is_relative": bool(is_relative),
            "execute": bool(execute)
        }
        current_timeout = 120.0 if execute else self.timeout

        r = self.session.post(f"{self.base_url}/move_joints", json=payload, timeout=current_timeout)
        r.raise_for_status()
        return self._check(r.json())

    def move_to_pose(self, x, y, z, r1, r2, r3, r4=0.0, joint_constraints=None, rotation_format="RPY", angle_format="RAD", reference_frame="WORLD", is_relative=False, cartesian_path=False, execute=True):
        """
        The universal function for point-to-point Cartesian movement.

        cartesian_path=False -> the goal pose is solved with IK and reached with a
        free-form joint-space path (OMPL). cartesian_path=True -> the TCP follows a
        strict straight line (Pilz LIN); joint_constraints are then IGNORED.

        Examples:
            # 1. Absolute Move in World (Euler)
            robot.move_to_pose(0.5, 0.0, 0.4, 3.14/2, 0.0, 0.0, rotation_format="RPY", reference_frame="WORLD")

            # 2. Relative Move in Tool frame (Fly-by-wire: advance 10cm on Z, straight line)
            robot.move_to_pose(0.0, 0.0, 0.10, 0.0, 0.0, 0.0, rotation_format="RPY", reference_frame="TOOL", is_relative=True, cartesian_path=True)

            # 3. Absolute Move with Quaternion
            robot.move_to_pose(0.4, 0.0, 0.4, 0.0, 1.0, 0.0, 0.0, rotation_format="QUAT")

        Args:
            x, y, z (float): Translation.
            r1, r2, r3, r4 (float): Rotation (r4 is ignored if format is RPY).
            joint_constraints (list[dict], optional): List of joint constraints. Each dict:
                {"joint_name": str, "min": float, "max": float, "relative": bool}.
                Bounds use `angle_format`. IGNORED when cartesian_path=True.
            rotation_format (str): "RPY" (Roll, Pitch, Yaw) or "QUAT" (x, y, z, w).
            angle_format (str): "DEG" for degrees or "RAD" for radians.
            reference_frame (str): "WORLD" or "TOOL".
            is_relative (bool): True = Delta from current pos, False = Absolute target.
            cartesian_path (bool): True = Strict straight line (Pilz LIN), False = Fluid joint-space path.
            execute (bool): Execute or simply plan.

        Returns:
            dict: Contains 'success' (bool) and 'message' (str).
        """
        payload = {
            "x": float(x), "y": float(y), "z": float(z),
            "r1": float(r1), "r2": float(r2), "r3": float(r3), "r4": float(r4),
             "joint_constraints": joint_constraints or [],
            "rotation_format": str(rotation_format),
            "reference_frame": str(reference_frame),
            "angle_format": str(angle_format),
            "is_relative": bool(is_relative),
            "cartesian_path": bool(cartesian_path),
            "execute": bool(execute)
        }
        current_timeout = 120.0 if execute else self.timeout

        r = self.session.post(f"{self.base_url}/move_to_pose", json=payload, timeout=current_timeout)
        r.raise_for_status()
        return self._check(r.json())

    def move_waypoints(self, waypoints,
                    rotation_format=None, angle_format=None,
                    is_relative=None, reference_frame=None,
                    cartesian_path=True, blend_radius=0.01, path_tolerance=0.05, execute=True):
        """
        Moves the robot through a list of points without stopping.

        Per-point fields (rotation_format, angle_format, is_relative, reference_frame)
        can be set INSIDE each waypoint dict, OR globally via the function arguments.
        If a global argument is given (not None), it OVERRIDES that field for ALL
        waypoints. If it is left as None, each waypoint keeps its own value (or a
        default: RPY / RAD / is_relative=False / reference_frame="WORLD").

        cartesian_path, blend_radius, path_tolerance and execute are global-only.

        Examples:
            # 1) Everything specified INSIDE each waypoint (no global args)
            points = [
                {"x": 0.1, "y": 0.0, "z": 0.0, "r1": 0.0, "r2": 0.0, "r3": 0.0,
                "is_relative": True, "reference_frame": "WORLD",
                "rotation_format": "RPY", "angle_format": "DEG"},
                {"x": 0.0, "y": 0.1, "z": 0.0, "r1": 0.0, "r2": 0.0, "r3": 0.0,
                "is_relative": True, "reference_frame": "WORLD",
                "rotation_format": "RPY", "angle_format": "DEG"},
            ]
            robot.move_waypoints(points)

            # 2) Everything specified GLOBALLY (the global values overwrite every point)
            points = [
                {"x": 0.1, "y": 0.0, "z": 0.0, "r1": 0.0, "r2": 0.0, "r3": 0.0},
                {"x": 0.0, "y": 0.1, "z": 0.0, "r1": 0.0, "r2": 0.0, "r3": 0.0},
            ]
            robot.move_waypoints(
                points,
                rotation_format="RPY", angle_format="DEG",
                is_relative=True, reference_frame="WORLD",
                cartesian_path=True, blend_radius=0.01)

        Args:
            waypoints (list[dict]): Points with keys x, y, z, r1, r2, r3, (r4) and
                optionally is_relative, reference_frame, rotation_format, angle_format.
            rotation_format (str|None): "RPY" or "QUAT". If set, overrides every point.
            angle_format (str|None): "RAD" or "DEG". If set, overrides every point.
            is_relative (bool|None): If set, overrides every point.
            reference_frame (str|None): "WORLD" or "TOOL". If set, overrides every point.
            cartesian_path (bool): True = straight lines (global).
            blend_radius (float): Corner blend in m, Cartesian only (cartesian_path=True).
                Rounds the corner between consecutive LIN segments.
                0.0          -> stop dead at each waypoint (most precise, jerky)
                0.005-0.02   -> recommended: smooth chaining for fine trajectories
                0.03-0.05    -> large sweeping moves, heavily rounded corners
                MUST stay below half the shortest segment length, else the Pilz
                Sequence planning fails.
            path_tolerance (float): TOTG corner rounding in rad, joint-space only
                (cartesian_path=False). How much the re-timing may deviate from the
                planned path to smooth segment junctions. Joint-space analogue of
                blend_radius.
                0.001-0.01   -> follows the path almost exactly: safest vs.
                                collisions, but more jerk at corners
                0.05         -> default / recommended: good fluidity vs. fidelity
                0.1-0.2      -> smoother/faster but deviates more from the path
                Server clamps any value > 0 to [0.001, 0.5].
                The final trajectory is re-checked for collisions before execution,
                so a too-large value is refused rather than executed blindly.
            execute (bool): Execute or just plan (global).

        Returns:
            dict: Contains 'success' (bool) and 'message' (str).
        """
        # Per-point fields: global override (if not None) else default-of-last-resort.
        per_point_overrides = {
            "rotation_format": rotation_format,
            "angle_format": angle_format,
            "is_relative": is_relative,
            "reference_frame": reference_frame,
        }
        per_point_defaults = {
            "rotation_format": "RPY",
            "angle_format": "RAD",
            "is_relative": False,
            "reference_frame": "WORLD",
        }

        resolved_waypoints = []
        for wp in waypoints:
            item = dict(wp)  # copy so we never mutate the caller's dicts
            for key, default in per_point_defaults.items():
                if per_point_overrides[key] is not None:
                    item[key] = per_point_overrides[key]   # global wins
                elif key not in item:
                    item[key] = default                     # neither given -> default
                # else: keep the value already present in the waypoint
            resolved_waypoints.append(item)

        payload = {
            "waypoints": resolved_waypoints,
            "cartesian_path": bool(cartesian_path),
            "blend_radius": float(blend_radius),
            "path_tolerance": float(path_tolerance),
            "execute": bool(execute),
        }
        current_timeout = 120.0 if execute else self.timeout

        r = self.session.post(f"{self.base_url}/move_waypoints", json=payload, timeout=current_timeout)
        r.raise_for_status()
        return self._check(r.json())

    def get_joint_state(self):
        """
        Retrieves the current joint angles of the robot.

        Examples:
            ret = robot.get_joint_state()

        Returns:
            dict: Contains the 'joints' list with angles in radians.
        """
        r = self.session.get(f"{self.base_url}/state/joints", timeout=self.timeout)
        r.raise_for_status()
        return self._check(r.json())

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
        r = self.session.get(f"{self.base_url}/state/pose", params=params, timeout=self.timeout)
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
        """
        Moves the robot back to its predefined "home" joint configuration.

        For safety, if the TCP is currently low (z < 0.05 m in world), it first
        lifts straight up by 10 cm (Cartesian) to avoid dragging through obstacles,
        then performs an absolute joint move to the home configuration.

        The home configuration depends on the model:
            - "vp5243": [0.0, 0.0, 1.57, 1.57, 0.0]
            - any other model: [0.0, 0.0, 1.57, 0.0, 1.57, 0.0]

        Examples:
            robot.move_to_home()

        Returns:
            dict: Contains 'success' (bool) and 'message' (str) from the final joint move.
        """
        home_position = []
        if self.get_current_pose()["position"]["z"] < 0.05:
            print("Robot is in a low position, moving up first to avoid collisions...")
            self.move_to_pose(0.0, 0.0, 0.1, 0.0, 0.0, 0.0, rotation_format="RPY", is_relative=True, cartesian_path=True)

        if self.model == "vp5243":
            home_position = [0.0, 0.0, 1.57, 1.57, 0.0]
        else:
            home_position = [0.0, 0.0, 1.57, 0.0, 1.57, 0.0]
        return self.move_joints(home_position, is_relative=False)

    def set_virtual_cage(self, enable=True, front=0.8, back=0.8, left=0.8, right=0.8, top=1.2, bottom=0.0, r=0.0, g=0.6, b=1.0, a=0.15):
        """
        Enables or disables a virtual collision cage around the robot.
        Distances are measured in meters from the world's zero point.

        Examples:
            robot.set_virtual_cage(enable=True, front=0.6, back=0.6, top=1.0)
            robot.set_virtual_cage(enable=False)

        Args:
            enable (bool): Enables or disables the cage.
            front (float): Maximum distance forward (+X).
            back (float): Maximum distance backward (-X).
            left (float): Maximum distance left (+Y).
            right (float): Maximum distance right (-Y).
            top (float): Maximum height (+Z).
            bottom (float): Maximum depth (-Z).
            r, g, b (float): Color of the cage in RGB (0.0 to 1.0).
            a (float): Alpha (transparency) of the cage (0.0 to 1.0).

        Returns:
            dict: Contains 'success' (bool) and 'message' (str).
        """
        payload = {
            "enable": bool(enable),
            "front": float(front), "back": float(back),
            "left": float(left), "right": float(right),
            "top": float(top), "bottom": float(bottom),
            "r": float(r), "g": float(g), "b": float(b), "a": float(a)
        }
        r = self.session.post(f"{self.base_url}/set_virtual_cage", json=payload, timeout=self.timeout)
        r.raise_for_status()
        return self._check(r.json())

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
        r = self.session.get(f"{self.base_url}/state/solver", timeout=self.timeout)
        r.raise_for_status()
        return self._check(r.json())

    def move_approach(self, x, y, z, r1, r2, r3, r4=0.0, joint_constraints=None, angle_format="RAD", rotation_format="RPY", z_offset=0.1, cartesian_path=False, execute=True):
        """
        Asks the ROS server to compute and execute an approach position above an object.

        Examples:
            robot.move_approach(0.45, 0.08, 0.12, 3.1416, 0.0, -2.478, z_offset=0.12)

        Args:
            x, y, z (float): Position of the object (final target).
            r1, r2, r3, r4 (float): Desired orientation of the tool.
            joint_constraints (list[dict], optional): Joint constraints. Each dict:
                {"joint_name": str, "min": float, "max": float, "relative": bool}.
                IGNORED when cartesian_path=True.
            angle_format (str): "RAD" or "DEG".
            rotation_format (str): "RPY" or "QUAT".
            z_offset (float): Retreat distance in meters (e.g., 0.1 for 10 cm above).
            cartesian_path (bool): True = straight line, False = joint space path.
            execute (bool): True = execute motion, False = plan only.

        Returns:
            dict: The response from the motion server ('success', 'message').
        """
        payload = {
            "x": float(x), "y": float(y), "z": float(z),
            "r1": float(r1), "r2": float(r2), "r3": float(r3), "r4": float(r4),
            "joint_constraints": joint_constraints or [],
            "angle_format": str(angle_format),
            "rotation_format": str(rotation_format),
            "z_offset": float(z_offset),
            "cartesian_path": bool(cartesian_path),
            "execute": bool(execute)
        }
        current_timeout = 120.0 if execute else self.timeout

        r = self.session.post(f"{self.base_url}/move_approach", json=payload, timeout=current_timeout)
        r.raise_for_status()
        return self._check(r.json())

    def compute_approach_pose(self, x, y, z, r1, r2, r3, r4=0.0,
                            rotation_format="RPY", angle_format="RAD",
                            reference_frame="WORLD", z_offset=0.1):
        """
        Computes the approach pose for a given target WITHOUT moving the robot.
        The backoff is applied along the target tool's local Z-axis (same
        convention as move_approach). The returned pose is always in the WORLD
        frame, so it can be fed directly into move_to_pose / move_to_pose_via_joint.

        Examples:
            # Target in world frame (degrees for readability)
            res = robot.compute_approach_pose(
                0.5, 0.0, 0.3, 180, 0, 0,
                rotation_format="RPY", angle_format="DEG", z_offset=0.1
            )
            pos = res["position"]
            quat = res["orientation_quat"]

            # Target expressed relative to the current tool pose
            res = robot.compute_approach_pose(
                0.0, 0.0, 0.05, 0, 0, 0,
                reference_frame="TOOL", z_offset=0.1
            )

        Args:
            x, y, z (float): Target position (final grasp point, meters).
            r1, r2, r3, r4 (float): Target orientation (r4 ignored if RPY).
            rotation_format (str): "RPY" or "QUAT".
            angle_format (str): "RAD" or "DEG" (only applies to RPY).
            reference_frame (str): "WORLD" or "TOOL". If "TOOL", the target is
                expressed relative to the current end-effector pose and is
                resolved server-side using the current robot state.
            z_offset (float): Backoff distance along the target tool's local
                Z-axis, in meters.

        Returns:
            dict: {
                "success", "message", "frame_id",
                "position":          {"x", "y", "z"},
                "orientation_quat":  {"x", "y", "z", "w"},
                "orientation_euler": {"rx", "ry", "rz"},
                "z_axis":            {"x", "y", "z"}
            }
        """
        payload = {
            "x": float(x), "y": float(y), "z": float(z),
            "r1": float(r1), "r2": float(r2), "r3": float(r3), "r4": float(r4),
            "rotation_format": str(rotation_format),
            "angle_format": str(angle_format),
            "reference_frame": str(reference_frame),
            "z_offset": float(z_offset),
        }
        r = self.session.post(f"{self.base_url}/compute_approach",
                            json=payload, timeout=self.timeout)
        r.raise_for_status()
        return self._check(r.json())

    def manage_box(self, box_id, x=0.0, y=0.0, z=0.0, r1=0.0, r2=0.0, r3=0.0, r4=0.0, rotation_format="RPY", size_x=0.1, size_y=0.1, size_z=0.1, r=0.8, g=0.8, b=0.8, a=1.0, action="ADD", enable_collision=True):
        """
        Adds or removes a collision box in MoveIt.
        If adding, the coordinates provided should be the TOP SURFACE center of the box,
        where the robot will grasp. The box center will be automatically calculated downward.

        Examples:
            robot.manage_box("target_cube", x=0.4, y=0.0, z=0.1, size_x=0.05, size_y=0.05, size_z=0.05)
            robot.manage_box("target_cube", action="REMOVE")

        Args:
            box_id (str): Unique name for the object (e.g., "target_cube").
            x, y, z (float): Position of the grasp point (top surface center).
            r1, r2, r3, r4 (float): Orientation of the grasp point.
            rotation_format (str): "RPY" (Roll, Pitch, Yaw) or "QUAT" (x, y, z, w).
            size_x, size_y, size_z (float): Dimensions of the box in meters.
            r, g, b (float): RGB color values from 0.0 to 1.0 (default is light gray).
            a (float): Alpha/transparency from 0.0 (invisible) to 1.0 (solid).
            action (str): "ADD" to spawn the box, "REMOVE" to delete it.
            enable_collision (bool): If True the box is a real collision obstacle; if
                False it is drawn as a visual-only marker (no effect on planning).

        Returns:
            dict: Contains 'success' (bool) and 'message' (str).
        """
        payload = {
            "box_id": str(box_id),
            "x": float(x), "y": float(y), "z": float(z),
            "r1": float(r1), "r2": float(r2), "r3": float(r3), "r4": float(r4),
            "rotation_format": str(rotation_format),
            "size_x": float(size_x), "size_y": float(size_y), "size_z": float(size_z),
            "r": float(r), "g": float(g), "b": float(b), "a": float(a),
            "action": str(action).upper(),
            "enable_collision": bool(enable_collision)
        }
        r = self.session.post(f"{self.base_url}/manage_box", json=payload, timeout=self.timeout)
        r.raise_for_status()
        return self._check(r.json())

    def manage_mesh(self, mesh_id, mesh_path="", x=0.0, y=0.0, z=0.0, r1=0.0, r2=0.0, r3=0.0, r4=0.0, rotation_format="RPY", scale_x=1.0, scale_y=1.0, scale_z=1.0, r=0.8, g=0.8, b=0.8, a=1.0, action="ADD"):
        """
        Adds or removes a 3D mesh (STL or DAE file) as a collision object in MoveIt/RViz.
        The mesh is positioned relative to the world frame.

        Examples:
            # 1. Add an STL file located on the absolute file system
            robot.manage_mesh(
                mesh_id="my_obstacle",
                mesh_path="file:///home/user/workspace/models/obstacle.stl",
                x=0.5, y=0.0, z=0.0,
                r1=0.0, r2=0.0, r3=1.57, rotation_format="RPY",
                action="ADD"
            )

            # 2. Add an STL file located inside a ROS package
            robot.manage_mesh(
                mesh_id="my_obstacle",
                mesh_path="package://my_robot_description/meshes/obstacle.stl",
                x=0.5, y=0.0, z=0.0,
                action="ADD"
            )

            # 3. Remove the mesh from the environment
            robot.manage_mesh(mesh_id="my_obstacle", action="REMOVE")

        Args:
            mesh_id (str): Unique identifier for the collision object (e.g., "motor_casing").
            mesh_path (str): URI pointing to the 3D file. MUST start with "file://" for absolute paths or "package://" for ROS packages. Ignored if action is "REMOVE".
            x, y, z (float): Position of the mesh's origin in the world frame.
            r1, r2, r3, r4 (float): Orientation of the mesh.
            rotation_format (str): "RPY" (Roll, Pitch, Yaw) or "QUAT" (x, y, z, w).
            scale_x, scale_y, scale_z (float): Scaling factors for the mesh along its X, Y, and Z axes (default is 1.0).
            r, g, b (float): RGB color values from 0.0 to 1.0 (default is light gray).
            a (float): Alpha/transparency from 0.0 (invisible) to 1.0 (solid).
            action (str): "ADD" to spawn/update the mesh, "REMOVE" to delete it from the scene.

        Returns:
            dict: The response from the motion server containing 'success' and 'message'.
        """
        payload = {
            "mesh_id": str(mesh_id),
            "mesh_path": str(mesh_path),
            "x": float(x), "y": float(y), "z": float(z),
            "r1": float(r1), "r2": float(r2), "r3": float(r3), "r4": float(r4),
            "rotation_format": str(rotation_format),
            "scale_x": float(scale_x), "scale_y": float(scale_y), "scale_z": float(scale_z),
            "r": float(r), "g": float(g), "b": float(b), "a": float(a),
            "action": str(action).upper()
        }

        r = self.session.post(f"{self.base_url}/manage_mesh", json=payload, timeout=self.timeout)
        r.raise_for_status()
        return self._check(r.json())

    def clear_environment(self):
        """
        Removes all collision objects (boxes, meshes, cage walls) from the
        planning scene, keeping only the robot and any attached tool objects.

        Examples:
            robot.clear_environment()

        Returns:
            dict: Contains 'success' (bool) and 'message' (str) listing removed objects.
        """
        r = self.session.post(f"{self.base_url}/clear_environment", timeout=self.timeout)
        r.raise_for_status()
        return self._check(r.json())

    def move_to_pose_via_joint(self, x, y, z, r1, r2, r3, r4=0.0, joint_constraints=None, rotation_format="RPY", angle_format="RAD", reference_frame="WORLD", is_relative=False, execute=True):
        """
        Moves to a Cartesian pose by first solving Inverse Kinematics, then planning
        and executing in joint space. Useful when you want a free-form joint-space
        trajectory to a known Cartesian goal (avoids Cartesian path constraints).
        Always joint-space, so joint_constraints are always honored.

        Examples:
            # Absolute pose in world frame (RPY)
            robot.move_to_pose_via_joint(0.5, 0.0, 0.4, 3.14, 0.0, 0.0)

            # Relative move in tool frame
            robot.move_to_pose_via_joint(0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                                        reference_frame="TOOL", is_relative=True)

            # With a joint constraint (e.g. keep base near 0)
            robot.move_to_pose_via_joint(
                0.25, 0.13, 0.30, 3.1416, 0.0, -1.5708,
                joint_constraints=[{"joint_name": "joint_1", "min": -0.5, "max": 0.5}],
            )

        Args:
            x, y, z (float): Translation target.
            r1, r2, r3, r4 (float): Rotation (r4 ignored if RPY).
            joint_constraints (list[dict], optional): Joint constraints. Each dict:
                {"joint_name": str, "min": float, "max": float, "relative": bool}.
                Bounds use `angle_format`.
            rotation_format (str): "RPY" or "QUAT".
            angle_format (str): "RAD" or "DEG".
            reference_frame (str): "WORLD" or "TOOL".
            is_relative (bool): True = delta from current pose, False = absolute.
            execute (bool): True = execute, False = plan only.

        Returns:
            dict: Contains 'success' (bool) and 'message' (str).
        """
        payload = {
            "x": float(x), "y": float(y), "z": float(z),
            "r1": float(r1), "r2": float(r2), "r3": float(r3), "r4": float(r4),
            "joint_constraints": joint_constraints or [],
            "rotation_format": str(rotation_format),
            "angle_format": str(angle_format),
            "reference_frame": str(reference_frame),
            "is_relative": bool(is_relative),
            "cartesian_path": False,
            "execute": bool(execute)
        }
        current_timeout = 120.0 if execute else self.timeout

        r = self.session.post(f"{self.base_url}/move_to_pose_via_joint", json=payload, timeout=current_timeout)
        r.raise_for_status()
        return self._check(r.json())

    def set_servo_on(self, enable: bool):
        """
        Enables or disables the robot motors.

        Examples:
            robot.set_servo_on(True)   # Motors ON
            robot.set_servo_on(False)  # Motors OFF

        Args:
            enable (bool): True = motors ON, False = motors OFF.

        Returns:
            dict: Contains 'success' (bool) and 'message' (str).
        """
        payload = {"enable": bool(enable)}
        r = self.session.post(f"{self.base_url}/set_servo_on", json=payload, timeout=self.timeout)
        r.raise_for_status()
        return self._check(r.json())

    def pump_grab(self):
        """
        Activates the vacuum pump + valve to grab an object.
        (Rob6x: valve in series with pump, both must be ON)

        Examples:
            robot.pump_grab()

        Returns:
            dict: Contains 'success' (bool) and 'message' (str).
        """
        r = self.session.post(f"{self.base_url}/pump/grab", timeout=self.timeout)
        r.raise_for_status()
        return self._check(r.json())

    def pump_release(self):
        """
        Deactivates the vacuum pump + valve to release an object.

        Examples:
            robot.pump_release()

        Returns:
            dict: Contains 'success' (bool) and 'message' (str).
        """
        r = self.session.post(f"{self.base_url}/pump/release", timeout=self.timeout)
        r.raise_for_status()
        return self._check(r.json())

    def pump_is_grabbed(self):
        """
        Checks if the vacuum sensor detects an object is grabbed.

        Examples:
            result = robot.pump_is_grabbed()
            if result["grabbed"]:
                print("Object is held!")

        Returns:
            dict: Contains 'success' (bool), 'grabbed' (bool), and 'message' (str).
        """
        r = self.session.get(f"{self.base_url}/pump/is_grabbed", timeout=self.timeout)
        r.raise_for_status()
        return r.json()

    def start_trace(self):
        """
        Starts the continuous TCP path trace.

        The server samples the real tool-tip position (via TF) and publishes it as a
        marker, so the actual travelled path is shown live in RViz, regardless of the
        motion type (joint, Cartesian/Pilz, waypoints, manual jog).

        Examples:
            robot.start_trace()
            robot.move_waypoints(points)
            robot.stop_trace()

        Returns:
            dict: Contains 'success' (bool) and 'message' (str).
        """
        r = self.session.post(f"{self.base_url}/trace/start", timeout=self.timeout)
        r.raise_for_status()
        return self._check(r.json())

    def stop_trace(self):
        """
        Stops sampling the TCP path. The existing trace stays displayed in RViz;
        call clear_trace() to erase it.

        Examples:
            robot.stop_trace()

        Returns:
            dict: Contains 'success' (bool) and 'message' (str).
        """
        r = self.session.post(f"{self.base_url}/trace/stop", timeout=self.timeout)
        r.raise_for_status()
        return self._check(r.json())

    def clear_trace(self):
        """
        Clears the recorded TCP trace and erases its marker in RViz.

        Examples:
            robot.clear_trace()

        Returns:
            dict: Contains 'success' (bool) and 'message' (str).
        """
        r = self.session.post(f"{self.base_url}/trace/clear", timeout=self.timeout)
        r.raise_for_status()
        return self._check(r.json())