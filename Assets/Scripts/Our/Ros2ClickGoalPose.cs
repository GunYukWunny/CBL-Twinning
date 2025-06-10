using UnityEngine;

public class Ros2ClickGoalPose : MonoBehaviour
{
    public Ros2GoalPoser goalSender;

    void Start()
    {
        if (goalSender == null)
        {
            goalSender = FindObjectOfType<Ros2GoalPoser>();
            if (goalSender == null)
            {
                Debug.LogError("Ros2GoalPoser not assigned or found in scene.");
            }
        }
    }

    void OnMouseDown()
    {
        if (!this.isActiveAndEnabled)
        {
            return;
        }

        if (Camera.main == null || goalSender == null) return;

        Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
        if (Physics.Raycast(ray, out RaycastHit hit, 100f))
        {
            if (hit.collider.gameObject == gameObject)
            {
                goalSender.SendGoalPose(hit.point);
                Debug.Log("Sent click goal" + gameObject.name);
            }
        }
    }
}

