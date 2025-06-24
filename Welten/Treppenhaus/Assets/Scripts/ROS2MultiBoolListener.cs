using System;
using UnityEngine;
using ROS2;
using std_msgs.msg;

public class ROS2MultiBoolListener : MonoBehaviour
{
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;

    private ISubscription<Bool>[] boolSubscriptions = new ISubscription<Bool>[16];
    private bool[] shouldRotate = new bool[16];

    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
    }

    void Update()
    {
        if (ros2Node == null && ros2Unity.Ok())
        {
            ros2Node = ros2Unity.CreateNode("ROS2UnityMultiBoolListenerNode");

            for (int i = 0; i < 16; i++)
            {
                int index = i; // Closure-Schutz
                string topicName = $"/bool_topic_{index + 1}_unity";

                boolSubscriptions[index] = ros2Node.CreateSubscription<Bool>(
                    topicName,
                    msg =>
                    {
                        Debug.Log($"[{topicName}] Empfangen: {msg.Data}");
                        shouldRotate[index] = msg.Data;
                    }
                );
            }
        }
    }

    // Zugriff per Index (0–15)
    public bool GetShouldRotate(int index)
    {
        if (index >= 0 && index < 16)
        {
            return shouldRotate[index];
        }
        return false;
    }

    // Einzelne Getter 0-basiert
    public bool GetShouldRotate0() => shouldRotate[0];
    public bool GetShouldRotate1() => shouldRotate[1];
    public bool GetShouldRotate2() => shouldRotate[2];
    public bool GetShouldRotate3() => shouldRotate[3];
    public bool GetShouldRotate4() => shouldRotate[4];
    public bool GetShouldRotate5() => shouldRotate[5];
    public bool GetShouldRotate6() => shouldRotate[6];
    public bool GetShouldRotate7() => shouldRotate[7];
    public bool GetShouldRotate8() => shouldRotate[8];
    public bool GetShouldRotate9() => shouldRotate[9];
    public bool GetShouldRotate10() => shouldRotate[10];
    public bool GetShouldRotate11() => shouldRotate[11];
    public bool GetShouldRotate12() => shouldRotate[12];
    public bool GetShouldRotate13() => shouldRotate[13];
    public bool GetShouldRotate14() => shouldRotate[14];
    public bool GetShouldRotate15() => shouldRotate[15];
}