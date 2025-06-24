using UnityEngine;

public class MoveOnPathRotation : MonoBehaviour
{
    public Transform[] points;
    public float speed = 2f;
    public float rotationSpeed = 5f;

    private int currentPointIndex = 0;

    void Update()
    {
        if (points.Length == 0) return;

        Transform target = points[currentPointIndex];
        Vector3 direction = (target.position - transform.position).normalized;
        transform.position = Vector3.MoveTowards(transform.position, target.position, speed * Time.deltaTime);

        if (direction != Vector3.zero)
        {
            Quaternion toRotation = Quaternion.LookRotation(direction);
            transform.rotation = Quaternion.Slerp(transform.rotation, toRotation, rotationSpeed * Time.deltaTime);
        }

        if (Vector3.Distance(transform.position, target.position) < 0.1f)
        {
            currentPointIndex = (currentPointIndex + 1) % points.Length;
        }
    }
}