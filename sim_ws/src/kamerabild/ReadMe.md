### Wie die C# Klassen verwenden? In Unity
Lidar2DSensorROS muss auf ein Objekt oder leeres Objekt welches als Sensor dienen sollen in Unity gezogen werden.
Am Besten auch noch [ROS2_for_Unity](https://github.com/RobotecAI/ros2-for-unity) von Robotec in den Assets Ordner einbinden.

### In ROS2 Humble
[Kamerabild](./) Ordner in ```src``` in den ROS2 Humble Workspace kopieren

```
colcon build
source install/setup.bash
ros2 run kamerabild lidar1
```

zum Visualisieren in ```rviz2``` die RVIZ-Konfiguration öffnen.
