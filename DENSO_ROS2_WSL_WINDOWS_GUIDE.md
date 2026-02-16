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

### Terminal WSL #3

```bash
cd ~/workspace
source venv/bin/activate
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

### Terminal windows

```powershell
.\venv\Scripts\activate
python ./test.py
```
