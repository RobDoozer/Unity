using UnityEngine;
using ROS2;
using std_msgs.msg;

public class FahrbefehlAnzeigen : MonoBehaviour
{
    // ROS-Komponenten
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private ISubscription<Float32MultiArray> fahrbefehlSubscription;

    // Interne Variablen für die beiden Werte
    private float bewegung = 0f; // Index 0 → vor/zurück
    private float rotation = 0f; // Index 1 → rechts/links

    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
        if (ros2Unity == null)
        {
            Debug.LogError("ROS2UnityComponent fehlt!");
        }
    }

    void Update()
    {
        if (ros2Node == null && ros2Unity != null && ros2Unity.Ok())
        {
            ros2Node = ros2Unity.CreateNode("fahrbefehl_listener_node");

            fahrbefehlSubscription = ros2Node.CreateSubscription<Float32MultiArray>(
                "/freigegebener_fahrbefehl",
                msg =>
                {
                    if (msg.Data.Length >= 2)
                    {
                        bewegung = msg.Data[0];
                        rotation = msg.Data[1];
                        Debug.Log($"[ROS] Fahrbefehl empfangen: Bewegung = {bewegung}, Rotation = {rotation}");
                    }
                    else
                    {
                        Debug.LogWarning("Empfangene Float32MultiArray hat weniger als 2 Werte.");
                    }
                });
        }
    }

    // Öffentliche Getter-Methoden für externen Zugriff
    public float GetBewegung()
    {
        return bewegung;
    }

    public float GetRotation()
    {
        return rotation;
    }
}