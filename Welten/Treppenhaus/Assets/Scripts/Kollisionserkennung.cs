using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CollisionColorChanger : MonoBehaviour
{
    private Dictionary<GameObject, Coroutine> activeColorCoroutines = new Dictionary<GameObject, Coroutine>();

    private void OnCollisionEnter(Collision collision)
    {
        GameObject other = collision.gameObject;

        // Prüfe, ob das Objekt einen Renderer hat
        if (other.TryGetComponent<Renderer>(out Renderer renderer))
        {
            // Starte die Coroutine, wenn sie nicht schon läuft
            if (!activeColorCoroutines.ContainsKey(other))
            {
                Coroutine colorCoroutine = StartCoroutine(ChangeColorCoroutine(renderer));
                activeColorCoroutines.Add(other, colorCoroutine);
            }
        }
    }

    private void OnCollisionExit(Collision collision)
    {
        GameObject other = collision.gameObject;

        if (activeColorCoroutines.TryGetValue(other, out Coroutine coroutine))
        {
            StopCoroutine(coroutine);

            // Setze die Farbe beim Verlassen zurück (optional)
            if (other.TryGetComponent<Renderer>(out Renderer renderer))
            {
                renderer.material.color = Color.white;
            }

            activeColorCoroutines.Remove(other);
        }
    }

    private IEnumerator ChangeColorCoroutine(Renderer renderer)
    {
        bool toggle = false;

        while (true)
        {
            renderer.material.color = toggle ? Color.red : Color.black;
            toggle = !toggle;
            yield return new WaitForSeconds(0.25f);
        }
    }
}
