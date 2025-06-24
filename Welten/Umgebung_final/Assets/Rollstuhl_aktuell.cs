using UnityEngine;
using System.Net.Sockets;
using System.Text;
using System.Threading.Tasks;

public class Rollstuhl : MonoBehaviour
{
    public float Speed;
    public float SpeedRotation;

    private ROS2MultiBoolListener rosListener;

    [Header("TCP Odometry Einstellungen")]
    public string rosIp = "127.0.0.1";
    public int odomPort = 5005;
    public float odomSendRateHz = 10f;

    [Header("Fahrbefehl (von ROS)")]
    public FahrbefehlAnzeigen fahrbefehlAnzeigen;

    [Header("Ampelanzeigen")]
    public GameObject AnzeigeVorwärts;
    public GameObject AnzeigeRückwärts;
    public GameObject AnzeigeRechts;
    public GameObject AnzeigeLinks;

    public GameObject[] visualIndicators = new GameObject[16];
    public int[] Steuerbefehle_Unity = new int[2]; // 0 = Bewegung, 1 = Rotation

    private float odomSendInterval;
    private float odomTimer;

    private bool bewegungErlaubt = true; // Steuerung über Tasten 0 und 1

    void Start()
    {
        rosListener = GetComponent<ROS2MultiBoolListener>();
        if (rosListener == null)
        {
            Debug.LogError("ROS2MultiBoolListener-Komponente nicht gefunden!");
        }

        odomSendInterval = 1f / odomSendRateHz;
        odomTimer = 0f;
    }

    void Update()
    {
        // Tasteneingaben zur Bewegungssperre
        if (Input.GetKeyDown(KeyCode.Alpha0))
        {
            bewegungErlaubt = false;
            Debug.Log("🚫 Bewegung GESPERRT durch Ziffer 0");
        }
        else if (Input.GetKeyDown(KeyCode.Alpha1))
        {
            bewegungErlaubt = true;
            Debug.Log("✅ Bewegung FREIGEGEBEN durch Ziffer 1");
        }

        UpdateSteuerbefehle();
        UpdateVisualIndicators();
        UpdateAmpelFarbLogik();

        if (bewegungErlaubt)
        {
            Movement(); // Nur wenn erlaubt
        }

        odomTimer += Time.deltaTime;
        if (odomTimer >= odomSendInterval)
        {
            SendOdometryToROS();
            odomTimer = 0f;
        }
    }

    void UpdateSteuerbefehle()
    {
        Steuerbefehle_Unity[0] = Input.GetKey(KeyCode.UpArrow) ? 1 :
                                  Input.GetKey(KeyCode.DownArrow) ? -1 : 0;

        Steuerbefehle_Unity[1] = Input.GetKey(KeyCode.RightArrow) ? 1 :
                                  Input.GetKey(KeyCode.LeftArrow) ? -1 : 0;
    }

    void Movement()
    {
        if (fahrbefehlAnzeigen == null) return;

        float bewegung = fahrbefehlAnzeigen.GetBewegung();
        float rotation = fahrbefehlAnzeigen.GetRotation();

        if (bewegung == 1f)
        {
            transform.Translate(Vector3.back * Speed * Time.deltaTime);
        }
        else if (bewegung == -1f)
        {
            transform.Translate(Vector3.forward * Speed * Time.deltaTime);
        }

        if (rotation != 0f)
        {
            Vector3 axis = rotation > 0 ? Vector3.up : Vector3.down;
            transform.Rotate(axis, Mathf.Abs(rotation) * SpeedRotation * Time.deltaTime);
        }
    }

    void UpdateAmpelFarbLogik()
    {
        if (fahrbefehlAnzeigen == null) return;

        float bewegung = fahrbefehlAnzeigen.GetBewegung();
        float rotation = fahrbefehlAnzeigen.GetRotation();

        bool tasteVor = Steuerbefehle_Unity[0] == 1;
        bool tasteZurück = Steuerbefehle_Unity[0] == -1;
        bool tasteRechts = Steuerbefehle_Unity[1] == 1;
        bool tasteLinks = Steuerbefehle_Unity[1] == -1;

        UpdateAmpelFarbe(AnzeigeVorwärts, tasteVor, bewegung == 1 && bewegungErlaubt);
        UpdateAmpelFarbe(AnzeigeRückwärts, tasteZurück, bewegung == -1 && bewegungErlaubt);
        UpdateAmpelFarbe(AnzeigeRechts, tasteRechts, rotation == 1 && bewegungErlaubt);
        UpdateAmpelFarbe(AnzeigeLinks, tasteLinks, rotation == -1 && bewegungErlaubt);
    }

    void UpdateAmpelFarbe(GameObject anzeige, bool tasteAktiv, bool rosBefehlAktiv)
    {
        if (anzeige == null) return;

        Renderer renderer = anzeige.GetComponent<Renderer>();
        if (renderer != null)
        {
            // Neu: rot wenn Taste gedrückt aber kein freigegebener Befehl (oder Bewegung gesperrt),
            // grün wenn Befehl aktiv, grau sonst
            if (tasteAktiv && (!rosBefehlAktiv || !bewegungErlaubt))
                renderer.material.color = Color.red;
            else if (rosBefehlAktiv && bewegungErlaubt)
                renderer.material.color = Color.green;
            else
                renderer.material.color = Color.gray;
        }
    }

    void UpdateVisualIndicators()
    {
        if (rosListener == null) return;

        bool[] values = new bool[16]
        {
            rosListener.GetShouldRotate0(), rosListener.GetShouldRotate1(),
            rosListener.GetShouldRotate2(), rosListener.GetShouldRotate3(),
            rosListener.GetShouldRotate4(), rosListener.GetShouldRotate5(),
            rosListener.GetShouldRotate6(), rosListener.GetShouldRotate7(),
            rosListener.GetShouldRotate8(), rosListener.GetShouldRotate9(),
            rosListener.GetShouldRotate10(), rosListener.GetShouldRotate11(),
            rosListener.GetShouldRotate12(), rosListener.GetShouldRotate13(),
            rosListener.GetShouldRotate14(), rosListener.GetShouldRotate15()
        };

        for (int i = 0; i < visualIndicators.Length && i < values.Length; i++)
        {
            if (visualIndicators[i] != null)
            {
                Renderer renderer = visualIndicators[i].GetComponent<Renderer>();
                if (renderer != null)
                {
                    renderer.material.color = values[i] ? Color.red : Color.green;
                }
            }
        }
    }

    void SendOdometryToROS()
    {
        var odom = new OdometryData
        {
            x = transform.position.x,
            y = transform.position.y,
            z = transform.position.z,
            qx = transform.rotation.x,
            qy = transform.rotation.y,
            qz = transform.rotation.z,
            qw = transform.rotation.w
        };

        string json = JsonUtility.ToJson(odom);
        byte[] data = Encoding.UTF8.GetBytes(json);
        SendTcpAsync(rosIp, odomPort, data);
    }

    void SendTcpAsync(string ip, int port, byte[] data)
    {
        _ = Task.Run(() =>
        {
            try
            {
                using (TcpClient client = new TcpClient(ip, port))
                using (NetworkStream stream = client.GetStream())
                {
                    stream.Write(data, 0, data.Length);
                }
            }
            catch (SocketException) { }
        });
    }

    [System.Serializable]
    public class OdometryData
    {
        public float x, y, z;
        public float qx, qy, qz, qw;
    }

    public void OnTriggerEnter(Collider other)
    {
        Debug.Log("Zusammenstoß mit Objekt");
    }
}
