using UnityEngine;

public class Steinerkennung : MonoBehaviour
{
    public Material targetMaterial; // Weisen Sie dies im Inspector zu

    void OnCollisionEnter(Collision collision)
    {
        GameObject other = collision.gameObject;
        Debug.Log("Kollision mit: " + other.name);

        Renderer rend = other.GetComponent<Renderer>();
        if (rend == null)
        {
            // Prüfe ob Renderer in Kindobjekten liegt
            rend = other.GetComponentInChildren<Renderer>();
        }

        if (rend != null)
        {
            Material currentMat = rend.sharedMaterial;

            Debug.Log("Materialname im Objekt: " + currentMat.name);
            Debug.Log("Gesuchtes Material: " + targetMaterial.name);

            // Material direkt vergleichen
            if (currentMat == targetMaterial)
            {
                Debug.Log("✅ Exaktes Material erkannt: " + currentMat.name);
            }
            // Oder Namen vergleichen, weil Unity Materialinstanzen zur Laufzeit erstellt
            else if (currentMat.name.StartsWith(targetMaterial.name))
            {
                Debug.Log("✅ Materialname stimmt überein (Instance): " + currentMat.name);
            }
            else
            {
                Debug.Log("❌ Material stimmt nicht überein.");
            }
        }
        else
        {
            Debug.Log("❌ Kein Renderer (auch nicht im Kind) gefunden.");
        }
    }
}

