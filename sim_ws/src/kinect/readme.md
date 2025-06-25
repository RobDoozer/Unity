# Kinect Sensor-Modell in Unity implementieren und in RVIZ ausführen

### C# Klassen in Unity
[KinectSensorFrequency.cs]() auf ein leeres Objekt als Komponente hinzufügen
Diesem leeren Objekt gehört nun eine Kamera zugeordnet, diese sollte untergeordnet 
dem leeren Kinect-Objekt zugeordnet werden, damit dieses sich mitbewegt
Den AudioListener dieser Kamera bitte entfernen.
	
  Zudem müssen Parameter wie z.B. Blickfeld selbst eingestellt werden...

### ROS2 Humble Paket: ```kinect``` nur optional
Das Paket [kinect](./) in den ```src``` Ordner des 
ROS Workspaces kopieren.

```
colcon build --packages-select kinect
source install/setup.bash
ros2 run kinect multi_kinect.launch.py
``` 


### Für ein fertiges Unity Projekt oder zum überprüfen der Einstellungen den Ordner 'Labyrinth_fertig' in Unity öffnen
