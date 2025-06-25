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

    private Renderer targetRenderer;
    private Color originalTargetColor = Color.green;

    private Dictionary<GameObject, Coroutine> activeCoroutines = new Dictionary<GameObject, Coroutine>();
    private Dictionary<GameObject, Color> originalColors = new Dictionary<GameObject, Color>();

    private int activeCollisions = 0;

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
        if (IsCollisionFromBelow(collision)) return;

        activeCollisions++;

        UpdateTargetColor();

        GameObject other = collision.gameObject;

        // Nur wenn Zielobjekt ROT ist, sollen andere Objekte blinken
        if (IsTargetRed() && other.TryGetComponent(out Renderer otherRenderer) && !activeCoroutines.ContainsKey(other))
        {
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
        if (IsCollisionFromBelow(collision)) return;

        activeCollisions = Mathf.Max(0, activeCollisions - 1);

        UpdateTargetColor();

        GameObject other = collision.gameObject;

        // Wenn das Zielobjekt wieder grün ist, soll das Blinken aufhören
        if (!IsTargetRed() && activeCoroutines.TryGetValue(other, out Coroutine coroutine))
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

    private void UpdateTargetColor()
    {
        if (targetRenderer != null)
        {
            targetRenderer.material.color = (activeCollisions > 0) ? Color.red : originalTargetColor;

            // Falls auf grün zurückgesetzt, ALLE Blink-Coroutines stoppen
            if (activeCollisions == 0)
            {
                StopAllBlinking();
            }
        }
    }

    private bool IsTargetRed()
    {
        return targetRenderer != null && targetRenderer.material.color == Color.red;
    }

    private void StopAllBlinking()
    {
        foreach (var entry in activeCoroutines)
        {
            GameObject obj = entry.Key;
            Coroutine coroutine = entry.Value;

            StopCoroutine(coroutine);

            if (obj.TryGetComponent(out Renderer renderer) && originalColors.TryGetValue(obj, out Color originalColor))
            {
                renderer.material.color = originalColor;
            }
        }

        activeCoroutines.Clear();
        originalColors.Clear();
    }

    private bool IsCollisionFromBelow(Collision collision)
    {
        foreach (ContactPoint contact in collision.contacts)
        {
            Vector3 localPoint = transform.InverseTransformPoint(contact.point);
            Vector3 worldNormal = contact.normal;

            bool isBelow = localPoint.y < -0.1f;
            bool normalPointsUp = Vector3.Dot(worldNormal, Vector3.up) > 0.7f;

            if (isBelow && normalPointsUp)
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
