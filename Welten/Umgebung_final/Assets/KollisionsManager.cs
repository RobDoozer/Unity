using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class KollisionsManager : MonoBehaviour
{
    [Header("Zielobjekt (bleibt rot bei Kollision)")]
    public GameObject targetToColor;

    [Header("Blinkfrequenz in Sekunden")]
    [Range(0.1f, 2f)]
    public float blinkFrequency = 0.25f;

    private Dictionary<GameObject, Coroutine> activeCoroutines = new Dictionary<GameObject, Coroutine>();
    private Dictionary<GameObject, Color> originalColors = new Dictionary<GameObject, Color>();

    private Renderer targetRenderer;
    private Color originalTargetColor = Color.green; // Standardfarbe ist grün

    void Start()
    {
        if (targetToColor != null && targetToColor.TryGetComponent(out targetRenderer))
        {
            targetRenderer.material.color = originalTargetColor;
        }
        else
        {
            Debug.LogWarning("Kein Zielobjekt gesetzt oder Renderer fehlt!");
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        GameObject other = collision.gameObject;

        if (IsCollisionFromBelow(collision)) return;

        // Zielobjekt rot färben
        if (targetRenderer != null)
        {
            targetRenderer.material.color = Color.red;
        }

        // Blink-Coroutine starten
        if (other.TryGetComponent(out Renderer otherRenderer) && !activeCoroutines.ContainsKey(other))
        {
            // Ursprungsfarbe merken
            if (!originalColors.ContainsKey(other))
            {
                originalColors[other] = otherRenderer.material.color;
            }

            Coroutine c = StartCoroutine(BlinkCoroutine(otherRenderer));
            activeCoroutines.Add(other, c);
        }
    }

    private void OnCollisionExit(Collision collision)
    {
        GameObject other = collision.gameObject;

        if (IsCollisionFromBelow(collision)) return;

        // Zielobjekt-Farbe auf grün zurücksetzen
        if (targetRenderer != null)
        {
            targetRenderer.material.color = originalTargetColor;
        }

        // Coroutine stoppen und Farbe zurücksetzen
        if (activeCoroutines.TryGetValue(other, out Coroutine coroutine))
        {
            StopCoroutine(coroutine);

            if (other.TryGetComponent(out Renderer otherRenderer) && originalColors.TryGetValue(other, out Color originalColor))
            {
                otherRenderer.material.color = originalColor;
            }

            activeCoroutines.Remove(other);
            originalColors.Remove(other);
        }
    }

private bool IsCollisionFromBelow(Collision collision)
{
    foreach (ContactPoint contact in collision.contacts)
    {
        Vector3 toContact = contact.point - transform.position;

        // Prüfe: Kontaktpunkt unterhalb + Normale zeigt deutlich nach oben
        bool below = toContact.y < -0.1f; // Toleranz: 0.1f
        bool normalPointsUp = Vector3.Dot(contact.normal, Vector3.up) > 0.5f;

        if (below && normalPointsUp)
        {
            return true;
        }
    }

    return false;
}


    private IEnumerator BlinkCoroutine(Renderer renderer)
    {
        bool toggle = false;

        while (true)
        {
            renderer.material.color = toggle ? Color.red : Color.black;
            toggle = !toggle;
            yield return new WaitForSeconds(blinkFrequency);
        }
    }
}
