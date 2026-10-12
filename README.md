## Build és source
```
colcon build --packages-select onrobot_eye_integration
```
```
source install/setup.bash
```

## Launch
```
ros2 launch onrobot_eye_integration pickup_and_putdown_bringup.py
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
python3 src/onrobot_eye_integration/onrobot_eye_integration/take_a_picture.py
```