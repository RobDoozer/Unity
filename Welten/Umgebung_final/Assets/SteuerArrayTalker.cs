using UnityEngine;
using ROS2;
using std_msgs.msg;

public class SteuerArrayTalker : MonoBehaviour
{
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private IPublisher<Float32MultiArray> commandPublisher;

    private Rollstuhl rollstuhlScript;

    private float sendInterval = 1f / 125f; // 125 Hz = alle 0.008 Sekunden wird das Array an ROS gesendet
    private float sendTimer = 0f;

    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
        rollstuhlScript = GetComponent<Rollstuhl>();

        if (rollstuhlScript == null)
        {
            Debug.LogError("Rollstuhl-Komponente nicht gefunden!");
        }
    }

    void Update()
    {
        if (!ros2Unity.Ok()) return;

        if (ros2Node == null)
        {
            ros2Node = ros2Unity.CreateNode("steuer_array_talker_node");
            commandPublisher = ros2Node.CreatePublisher<Float32MultiArray>("/controller_commands");
        }

        sendTimer += Time.deltaTime;

        if (sendTimer >= sendInterval)
        {
            sendTimer = 0f;

            if (rollstuhlScript != null)
            {
                int[] aktuellerZustand = rollstuhlScript.Steuerbefehle_Unity;

                Float32MultiArray msg = new Float32MultiArray
                {
                    Data = new float[2]
                };

                msg.Data[0] = aktuellerZustand[0];
                msg.Data[1] = aktuellerZustand[1];

                commandPublisher.Publish(msg);

               // Debug.Log($"[ROS2 125Hz] Publish: Bewegung: {msg.Data[0]}, Rotation: {msg.Data[1]}");
            }
        }
    }

    public float GetSendFrequency()
    {
        return 1f / sendInterval;
    }
}