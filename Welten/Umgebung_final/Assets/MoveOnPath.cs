using UnityEngine;

public class MoveOnPath : MonoBehaviour
{
    public Transform[] points;
    public float speed = 2f;
    public float rotationSpeed = 5f;
    private int currentPointIndex = 0;

    private bool isWaiting = false;
    private float waitTime = 15f;
    private float waitTimer = 0f;

    void Update()
    {
        if (points.Length == 0 || isWaiting) 
        {
            HandleWait();
            return;
        }

        Transform target = points[currentPointIndex];
        Vector3 direction = (target.position - transform.position).normalized;
        transform.position = Vector3.MoveTowards(transform.position, target.position, speed * Time.deltaTime);

        if (Vector3.Distance(transform.position, target.position) < 0.005f)
        {
            isWaiting = true;
            waitTimer = 0f;
        }
    }

    void HandleWait()
    {
        if (!isWaiting) return;

        waitTimer += Time.deltaTime;
        if (waitTimer >= waitTime)
        {
            isWaiting = false;
            currentPointIndex = (currentPointIndex + 1) % points.Length;
        }
    }
}


