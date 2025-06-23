using UnityEngine;
using Unity.Jobs;
using Unity.Collections;
using System.Net.Sockets;
using System.Threading.Tasks;

public class KinectRaycastSensorFrequenzy : MonoBehaviour
{
    [Header("Kameraeinstellungen")]
    public Camera sensorCamera;

    [Header("Auflösung")]
    public int width = 512;
    public int height = 424;

    [Header("ROS TCP-Ziel")]
    public string rosIp = "127.0.0.1";
    public int depthPort;
    public int rgbPort;

    [Header("Abtastfrequenz")]
    [Tooltip("Abtastfrequenz in Hertz (z. B. 10 = 10 Abtastungen/Sekunde)")]
    public float abtastfrequenzHz = 10f;

    private float[] depthData;
    private byte[] rgbData;
    private float nextSampleTime = 0f;

    private NativeArray<RaycastCommand> rayCommands;
    private NativeArray<RaycastHit> rayResults;

    private bool isRunning = false;

    void Start()
    {
        if (!Application.isPlaying) return;

        isRunning = true;

        if (sensorCamera == null)
            sensorCamera = Camera.main;

        sensorCamera.enabled = false;

        int totalRays = width * height;
        depthData = new float[totalRays];
        rgbData = new byte[totalRays * 3];

        rayCommands = new NativeArray<RaycastCommand>(totalRays, Allocator.Persistent);
        rayResults = new NativeArray<RaycastHit>(totalRays, Allocator.Persistent);

        nextSampleTime = Time.time;
    }

    void OnDestroy()
    {
        isRunning = false;

        if (rayCommands.IsCreated) rayCommands.Dispose();
        if (rayResults.IsCreated) rayResults.Dispose();
    }

    void Update()
    {
        if (!Application.isPlaying || !isRunning) return;

        if (Time.time >= nextSampleTime)
        {
            nextSampleTime = Time.time + (1f / abtastfrequenzHz);
            ScheduleAndCollectRaycasts();
        }
    }

    void ScheduleAndCollectRaycasts()
    {
        if (!Application.isPlaying || !isRunning) return;

        int totalRays = width * height;

        CameraData camData = new CameraData(sensorCamera);

        var rayGenJob = new RayGenerationJob
        {
            width = width,
            height = height,
            camData = camData,
            rayCommands = rayCommands
        };
        JobHandle genHandle = rayGenJob.Schedule(totalRays, 64);
        genHandle.Complete();

        JobHandle rayHandle = RaycastCommand.ScheduleBatch(rayCommands, rayResults, 64);
        rayHandle.Complete();

        ProcessHitsOnMainThread();

        SendDepthData(rosIp, depthPort);
        SendRgbData(rosIp, rgbPort);
    }

    void ProcessHitsOnMainThread()
    {
        for (int i = 0; i < rayResults.Length; i++)
        {
            RaycastHit hit = rayResults[i];

            if (hit.collider != null)
            {
                depthData[i] = Mathf.Round(hit.distance * 100f) / 100f;

                Color color = Color.black;
                Renderer renderer = hit.collider.GetComponent<Renderer>();
                if (renderer != null && renderer.material != null)
                {
                    color = renderer.material.color;
                }

                rgbData[i * 3 + 0] = (byte)(Mathf.Clamp01(color.r) * 255f);
                rgbData[i * 3 + 1] = (byte)(Mathf.Clamp01(color.g) * 255f);
                rgbData[i * 3 + 2] = (byte)(Mathf.Clamp01(color.b) * 255f);
            }
            else
            {
                depthData[i] = 0f;
                rgbData[i * 3 + 0] = 0;
                rgbData[i * 3 + 1] = 0;
                rgbData[i * 3 + 2] = 0;
            }
        }
    }

    void SendDepthData(string ip, int port)
    {
        if (!Application.isPlaying || !isRunning) return;

        byte[] bytes = new byte[depthData.Length * sizeof(float)];
        System.Buffer.BlockCopy(depthData, 0, bytes, 0, bytes.Length);
        SendTcpAsync(ip, port, bytes);
    }

    void SendRgbData(string ip, int port)
    {
        if (!Application.isPlaying || !isRunning) return;

        SendTcpAsync(ip, port, rgbData);
    }

    void SendTcpAsync(string ip, int port, byte[] data)
    {
        if (!Application.isPlaying || !isRunning) return;

        _ = Task.Run(() =>
        {
            if (!isRunning) return;

            try
            {
                using (TcpClient client = new TcpClient(ip, port))
                using (NetworkStream stream = client.GetStream())
                {
                    stream.Write(data, 0, data.Length);
                }
            }
            catch (SocketException ex)
            {
                Debug.LogWarning($"TCP-Verbindung fehlgeschlagen (Port {port}): {ex.Message}");
            }
        });
    }

    struct CameraData
    {
        public Vector3 position;
        public Quaternion rotation;
        public float fov;
        public float aspect;
        public float nearClipPlane;

        public CameraData(Camera cam)
        {
            position = cam.transform.position;
            rotation = cam.transform.rotation;
            fov = cam.fieldOfView;
            aspect = cam.aspect;
            nearClipPlane = cam.nearClipPlane;
        }

        public Ray ViewportPointToRay(float u, float v)
        {
            Vector3 forward = rotation * Vector3.forward;
            Vector3 right = rotation * Vector3.right;
            Vector3 up = rotation * Vector3.up;

            float nearHeight = 2f * nearClipPlane * Mathf.Tan(fov * 0.5f * Mathf.Deg2Rad);
            float nearWidth = nearHeight * aspect;

            Vector3 point = position + forward * nearClipPlane
                            + right * ((u - 0.5f) * nearWidth)
                            + up * ((v - 0.5f) * nearHeight);

            Vector3 dir = (point - position).normalized;
            return new Ray(position, dir);
        }
    }

    struct RayGenerationJob : IJobParallelFor
    {
        public int width;
        public int height;
        public CameraData camData;
        public NativeArray<RaycastCommand> rayCommands;

        public void Execute(int index)
        {
            int x = width - 1 - (index % width);    // x gespiegelt
            int y = index / width;
            float u = (float)x / (width - 1);
            float v = (float)y / (height - 1);

            Ray ray = camData.ViewportPointToRay(u, v);
            rayCommands[index] = new RaycastCommand(ray.origin, ray.direction, 10f);
        }
    }
}
