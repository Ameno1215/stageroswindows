# Pilotage d’un robot DENSO (VS060) en simulation depuis Windows
## via ROS 2 Humble + MoveIt 2 (WSL) et Python (Windows)

Ce document décrit **pas à pas** comment :
- lancer la **simulation** du robot DENSO VS060 sous **WSL (Ubuntu + ROS 2 Humble)**,
- lancer le **serveur de mouvement MoveIt**,
- lancer un **bridge HTTP**,
- **commander le robot depuis Windows en Python** (init, mouvement, vitesse).

---

## 1. Architecture générale

Windows (Python)
→ HTTP (requests)
→ WSL Ubuntu
→ FastAPI / Uvicorn (bridge)
→ ROS 2 Humble
→ denso_motion_control (C++ / MoveIt 2)
→ Simulation Gazebo / RViz

---

## 2. Pré-requis

### Côté WSL
- Ubuntu (WSL2 recommandé)
- ROS 2 Humble
- Workspace compilé (`denso_ros2_ws`)
- Packages :
  - denso_robot_bringup
  - denso_robot_moveit_config
  - denso_motion_control

### Côté Windows
- Python 3.10+
- Environnement virtuel (`venv`)
- Module `requests`

---

## 3. Préparation de l’environnement WSL

À faire dans **chaque terminal WSL** :

```bash
cd ~/workspace/denso_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Optionnel (souvent nécessaire sous WSL) :

```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

---

## 4. Lancer la simulation DENSO + MoveIt

### Terminal WSL #1

```bash
ros2 launch denso_robot_bringup denso_robot_bringup.launch.py model:=vs060 sim:=true
```

---

## 5. Lancer le serveur de mouvement

### Terminal WSL #2

```bash
ros2 launch denso_motion_control motion_server.launch.py model:=vs060 sim:=true
```

---

## 6. Lancer le bridge HTTP

### Installation (une seule fois)

```bash
python3 -m pip install --user fastapi uvicorn pydantic
```

### Lancement

```bash
cd ~/workspace
source /opt/ros/humble/setup.bash
source ~/workspace/denso_ros2_ws/install/setup.bash
uvicorn wsl_ros_bridge:app --host 0.0.0.0 --port 8000
```

Test :

```bash
curl http://localhost:8000/health
```

---

## 7. Préparer l’environnement Windows

```powershell
.\venv\Scripts\activate
python -m pip install requests
```

---

## 8. Client Python Windows

### denso_http_client.py

```python
import requests

class DensoRobotClient:
    def __init__(self, base_url="http://localhost:8000"):
        self.base_url = base_url.rstrip("/")

    def init_robot(self, model="vs060", planning_group="arm", velocity_scale=0.1, accel_scale=0.1):
        return requests.post(f"{self.base_url}/init", json={
            "model": model,
            "planning_group": planning_group,
            "velocity_scale": velocity_scale,
            "accel_scale": accel_scale
        }).json()

    def set_scaling(self, velocity_scale, accel_scale):
        return requests.post(f"{self.base_url}/scaling", json={
            "velocity_scale": velocity_scale,
            "accel_scale": accel_scale
        }).json()

    def goto_joint(self, joints):
        return requests.post(f"{self.base_url}/goto_joint", json={
            "joints": joints,
            "execute": True
        }).json()

    def goto_pose(self, frame_id, x, y, z, qx, qy, qz, qw):
        return requests.post(f"{self.base_url}/goto_pose", json={
            "frame_id": frame_id,
            "position": {"x": x, "y": y, "z": z},
            "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
            "execute": True
        }).json()
```

---

## 9. Exemple d’utilisation

```python
from denso_http_client import DensoRobotClient

robot = DensoRobotClient()

robot.init_robot()
robot.set_scaling(0.5, 0.5)
robot.goto_joint([1.57, 0, 1.57, 0, 1.57, 0])
robot.goto_pose("base_link", -0.1, 0.4, 0.4, 0.707, -0.707, 0, 0)
```

---

## 10. Passage au robot réel

Changer uniquement le bringup :

```bash
ros2 launch denso_robot_bringup denso_robot_bringup.launch.py model:=vs060 sim:=false
```

Le code Windows reste identique.

---

Fin du document.
