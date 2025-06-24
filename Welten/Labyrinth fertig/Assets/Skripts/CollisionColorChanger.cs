using UnityEngine;

public class CollisionColorChanger : MonoBehaviour
{
    [Tooltip("Dieses Objekt wird eingefärbt (z.B. ein GameObject mit Renderer).")]
    public GameObject targetObject;

    private Renderer targetRenderer;
    private int collisionCount = 0;

    private void Start()
    {
        if (targetObject != null)
        {
            targetRenderer = targetObject.GetComponent<Renderer>();
            if (targetRenderer != null)
            {
                SetColor(Color.green); // Anfangszustand
            }
        }
        else
        {
            Debug.LogWarning("Kein Zielobjekt zugewiesen!");
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        collisionCount++;
        UpdateColor();
    }

    private void OnCollisionExit(Collision collision)
    {
        collisionCount = Mathf.Max(0, collisionCount - 1);
        UpdateColor();
    }

    private void UpdateColor()
    {
        if (targetRenderer != null)
        {
            if (collisionCount > 0)
                SetColor(Color.red);
            else
                SetColor(Color.green);
        }
    }

    private void SetColor(Color color)
    {
        targetRenderer.material.color = color;
    }
}
