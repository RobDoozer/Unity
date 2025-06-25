using UnityEngine;
using System.Net.Sockets;
using System.Text;
using System.Globalization;
using System.Collections;
using System.Collections.Generic;
using System.Net;

/// <summary>
/// Simuliert kontinuierliche 2D-LiDAR-Daten (XZ-Ebene, Rotation um lokale Y-Achse).
/// Sendet bei 360* eine UDP-Nachricht im Format: "Winkel,Reichweite\n"
/// </summary>
/// 
/// //detection layer muss default sein in Unity
public class Lidar2DSensorROS : MonoBehaviour
{
    [Header("Sensor-Einstellungen")]
    public float rotationSpeed = 3600f;
    public float angleStep = 0.72f;
    public float maxRange = 10f;
    public LayerMask detectionLayers = ~0;

    [Header("ROS TCP-Ziel")]
    public string rosIp = "127.0.0.1";
    public int Port;

    public delegate void OnRayMeasured(float angleDeg, float range);
    public event OnRayMeasured OnMeasurement;

    private float accumulatedRotation = 0f;
    private float currentScanAngle = 0f;
    private UdpClient udpClient;
    private List<string> batchData = new List<string>();

    void Start()
    {
        udpClient = new UdpClient();

        try
        {
            udpClient.Connect(rosIp, Port); // IP des ROS-PCs
        }
        catch (SocketException)
        {
            Debug.LogWarning($"Port {Port} inaktiv oder nicht erreichbar.");
            // Optional: udpClient = null;  // falls gewünscht, um später zu prüfen
        }

        //Application.targetFrameRate = 240;
        //StartCoroutine(LidarLoop());         
    }

    private IEnumerator LidarLoop()
    {
        float angleStepLocal = angleStep;  // sicherstellen, dass Zwischenschritte exakt laufen
        float scanAngle = 0f;

        WaitForSecondsRealtime wait = new WaitForSecondsRealtime(0.0002f); // 5000 Hz

        while (true)
        {
            MeasureAndSend(scanAngle);
            scanAngle += angleStepLocal;
            if (scanAngle >= 360f) scanAngle -= 360f;

            yield return wait;
        }
    }

    void Update()
    {
        //float deltaRotation = rotationSpeed * Time.deltaTime;
        
        float deltaRotation = 5000 * 0.72f * Time.deltaTime;  // 5000 Hz * 0,72°
        // Debug.Log($"Frame Time: {Time.deltaTime:F4} s");
        accumulatedRotation += deltaRotation;

        while (accumulatedRotation >= angleStep)
        {
            MeasureAndSend(currentScanAngle);
            currentScanAngle += angleStep;
            if (currentScanAngle >= 360f) currentScanAngle -= 360f;
            accumulatedRotation -= angleStep;
        }
    }

    void MeasureAndSend(float angleDeg)
    {
        if (udpClient == null)
            return; // Kein UDP-Client verfügbar, Daten nicht senden

        Vector3 origin = transform.position;
        Quaternion localRotation = Quaternion.Euler(0f, -angleDeg, 0f);
        Vector3 localDirection = localRotation * Vector3.forward;
        Vector3 worldDirection = transform.TransformDirection(localDirection);

        RaycastHit hit;
        float range = maxRange;
        if (Physics.Raycast(origin, worldDirection, out hit, maxRange, detectionLayers))
        {
            range = hit.distance;
        }

        string data = $"{angleDeg:F1},{range:F3}";
        batchData.Add(data);

        if (batchData.Count >= 500)
        {
            string combinedData = string.Join("\n", batchData) + "\n";
            byte[] bytes = Encoding.ASCII.GetBytes(combinedData);
            udpClient.Send(bytes, bytes.Length);
            batchData.Clear();
           // Debug.Log(combinedData);
        }

        Debug.DrawLine(origin, origin + worldDirection * range, Color.green, 0.05f);

        OnMeasurement?.Invoke(angleDeg, range);
    }

    void OnApplicationQuit()
    {
        if (udpClient != null)
        {
            udpClient.Close();
        }
    }
}
