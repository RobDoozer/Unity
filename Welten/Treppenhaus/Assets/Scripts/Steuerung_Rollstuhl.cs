using UnityEngine;

public class Steuerung_Rollstuhl : MonoBehaviour
{
    public float Speed = 5f;
    public float SpeedRotation = 100f;

    void Update()
    {
        // Vorwärts-/Rückwärtsbewegung (↑/↓)
        float vorwärts = Input.GetAxis("Vertical") * Speed * Time.deltaTime;
        transform.Translate(Vector3.back * vorwärts);

        // Rotation (←/→)
        float rotation = Input.GetAxis("Horizontal") * SpeedRotation * Time.deltaTime;
        transform.Rotate(Vector3.up * rotation);
    }
}
