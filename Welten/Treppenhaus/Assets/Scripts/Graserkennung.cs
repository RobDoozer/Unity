using UnityEngine;

public class Graserkennung : MonoBehaviour
{
    public Material targetMaterial; // Das gesuchte Material im Inspector zuweisen

    void OnCollisionEnter(Collision collision)
    {
        Renderer rend = collision.gameObject.GetComponent<Renderer>();

        if (rend != null)
        {
            Material currentMat = rend.material;

            if (currentMat == targetMaterial)
            {
                Debug.Log("Kollision mit Objekt, das das gesuchte Material hat: " + collision.gameObject.name);
            }
            else
            {
                Debug.Log("Kollision, aber Material stimmt nicht überein.");
            }
        }
        else
        {
            Debug.Log("Kein Renderer am kollidierten Objekt gefunden.");
        }
    }
}

