## Clone, build és source
Doosan M1013 driver:
```
git clone https://github.com/DoosanRobotics/doosan-robot2.git
```

RG-2 mefogó driver:
```
git clone https://github.com/takuya-ki/onrobot-rg.git
```

```
colcon build
```

```
source install/setup.bash
```

## Launch
```
ros2 launch onrobot_eye_integration pickup_and_putdown_bringup.py
```

## Mozgatás paranccsal (bringup kell hozzá)
```
ros2 service call /dsr01/motion/move_joint dsr_msgs2/srv/MoveJoint "{pos: [<cords>], vel: 20.0, acc: 50.0, time: 0.0, mode: 0, radius: 0.0, blend_type: 0, sync_type: 0}"
```

## Külön-külön indítás

### 1. Terminál: API
```
uvicorn src.onrobot_eye_integration.onrobot_eye_integration.api:app --reload
```

### 2. Terminál: Bringup
```
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real model:=m1013 host:=192.168.10.2
```

### 3. Terminál: Node
```
ros2 run onrobot_eye_integration onrobot_eye_node
```

### 4. Terminál: Program
```
ros2 run onrobot_eye_integration onrobot_pickup
```

### 5. Terminál: Kamerakép mentése
```
ros2 run onrobot_eye_integration take_a_picture
```

### 6. Terminál: Topic híd
```
ros2 run domain_bridge domain_bridge /home/arnl/kevin_ws/src/onrobot_eye_integration/onrobot_eye_integration/config/topic_bridge.yaml
```
