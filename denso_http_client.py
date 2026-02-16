import requests


class DensoRobotClient:
    def __init__(self, base_url="http://localhost:8000", timeout=60.0):
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout

    def health(self):
        r = requests.get(f"{self.base_url}/health", timeout=self.timeout)
        r.raise_for_status()
        return r.json()

    def init_robot(self, model="vs060", planning_group="arm", velocity_scale=0.1, accel_scale=0.1):
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
        payload = {
            "velocity_scale": float(velocity_scale),
            "accel_scale": float(accel_scale),
        }
        r = requests.post(f"{self.base_url}/scaling", json=payload, timeout=self.timeout)
        r.raise_for_status()
        return r.json()

    def goto_joint(self, joints, execute=True):
        payload = {"joints": [float(x) for x in joints], "execute": bool(execute)}
        r = requests.post(f"{self.base_url}/goto_joint", json=payload, timeout=self.timeout)
        r.raise_for_status()
        return r.json()

    def goto_pose(self, frame_id, x, y, z, qx, qy, qz, qw, execute=True):
        payload = {
            "frame_id": frame_id,
            "position": {"x": float(x), "y": float(y), "z": float(z)},
            "orientation": {"x": float(qx), "y": float(qy), "z": float(qz), "w": float(qw)},
            "execute": bool(execute),
        }
        r = requests.post(f"{self.base_url}/goto_pose", json=payload, timeout=self.timeout)
        r.raise_for_status()
        return r.json()
    
    def get_joint_state(self):
        r = requests.get(f"{self.base_url}/state/joints", timeout=self.timeout)
        r.raise_for_status()
        return r.json()

    def get_current_pose(self):
        r = requests.get(f"{self.base_url}/state/pose", timeout=self.timeout)
        r.raise_for_status()
        return r.json()

