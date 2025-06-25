Lidar2DSensorROS simuliert die Daten eines 2D-Lidars in Unity und sendet die Daten über einen einstellbaren Port an ROS

KinectRaycastSensorFrequenzy simuliert einen Kinect in Unity und sendet die Daten über einstellbare Ports an ROS (ein Port für Tiefenbild, ein Port für RGB-Daten)

ROS2MultiBoolListener empfängt in Unity die 16-Boolean-Werte, die in ROS aus dem Auswertealgorithmus nach der RTAB-Map kommen

Rollstuhl_aktuell ist für den Rollstuhl in Unity bestimmt (u.a. Steuerung des Rollstuhls und der Ampeln)

SteuerArrayTalker gibt die in Unity durch die Pfeiltasten eingegebenen Fahrbefehle an ROS weiter

freigegebeneFahrbefehlEmpfänger empfängt von ROS das Array nach der Entscheidungsbridge, das liefert, ob und wie gefahren werden darf (oder auch nicht)

KollisionsManager erkennt Kontakt des Körpers, dem das Skript zugeordnet ist (muss Collider UND Rigidbody haben) und anderen Objekten mit Collideer (zwingend notwendig). Bei Kollision wird ein frei wählbares Objekt rot eingefärbt.

Rollstuhl und Rollstuhl sind alte Skripts
