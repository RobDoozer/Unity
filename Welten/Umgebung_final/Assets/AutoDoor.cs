using UnityEngine;
using System.Collections;

public class AutoDoor : MonoBehaviour
{
    public float openAngle = 90f;
    public float openCloseSpeed = 2f;
    public float interval = 15f;

    private Quaternion closedRotation;
    private Quaternion openRotation;
    private bool isOpen = false;

    void Start()
    {
        closedRotation = transform.rotation;
        openRotation = Quaternion.Euler(transform.eulerAngles + new Vector3(0, openAngle, 0));
        StartCoroutine(DoorRoutine());
    }

    IEnumerator DoorRoutine()
    {
        while (true)
        {
            isOpen = !isOpen;
            Quaternion targetRotation = isOpen ? openRotation : closedRotation;
            float t = 0f;
            Quaternion startRotation = transform.rotation;

            while (t < 1f)
            {
                t += Time.deltaTime * openCloseSpeed;
                transform.rotation = Quaternion.Slerp(startRotation, targetRotation, t);
                yield return null;
            }

            yield return new WaitForSeconds(interval);
        }
    }
}
