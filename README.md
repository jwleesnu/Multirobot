multirobot simulator

based on turtlesim

창하나에서

```bash
ros2 run turtlesim turtlesim_node
```

다른 창에서

```bash
export TURTLEBOT3_MODEL=burger && ros2 run turtlebot3_teleop teleop_keyboard
```

혹시 거북이 추가하려면

```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 5.5, y: 9, theta: 1.57}"
```

csv로 로깅하는 노드 실행하려면

```bash
ros2 run turtlebot3_log pose_logger
```
