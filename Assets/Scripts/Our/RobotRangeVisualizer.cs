using UnityEngine;

public class RobotRangeVisualizer : MonoBehaviour
{
    [Header("Dependencies")]
    public Ros2PathPlanner pathPlanner;

    [Header("Range Settings")]
    public LineRenderer rangeCirclePrefab;
    public int circleSegments = 60;
    public float circleLineWidth = 0.1f;
    public Color circleColor = new Color(0, 0, 1, 0.3f);

    private LineRenderer activeRangeCircle;
    public bool showRangeCircle = true;

    void Start()
    {
        if (pathPlanner == null)
        {
            Debug.LogError("Path Planner not assigned in RobotRangeVisualizer!", this);
            enabled = false;
            return;
        }
        if (pathPlanner.robotTransform == null)
        {
            Debug.LogError("Robot Transform not assigned in Path Planner, required by RobotRangeVisualizer!", this);
            enabled = false;
            return;
        }
        if (pathPlanner.batteryStatus == null)
        {
            Debug.LogWarning("Battery Status UI not assigned in Path Planner. Range visualization will not be dynamic.", this);
        }
        if (rangeCirclePrefab == null)
        {
            Debug.LogError("Range Circle Prefab not assigned in RobotRangeVisualizer!", this);
            enabled = false;
            return;
        }

        InitializeRangeCircle();
    }

    void Update()
    {
        UpdateRangeCircle();
    }

    private void InitializeRangeCircle()
    {
        activeRangeCircle = Instantiate(rangeCirclePrefab, pathPlanner.robotTransform.position, Quaternion.identity);
        activeRangeCircle.transform.parent = pathPlanner.robotTransform;
        activeRangeCircle.gameObject.SetActive(showRangeCircle);
        SetupLineRenderer(activeRangeCircle);
    }

    private void SetupLineRenderer(LineRenderer line)
    {
        line.useWorldSpace = false;
        line.startWidth = circleLineWidth;
        line.endWidth = circleLineWidth;
        line.startColor = circleColor;
        line.endColor = circleColor;
    }

    private void UpdateRangeCircle()
    {
        if (activeRangeCircle == null || pathPlanner.robotTransform == null) return;

        float range = (float)((pathPlanner.batteryStatus?.batteryPercentage ?? 0) * pathPlanner.batteryToMeterConstant);
        GenerateCirclePoints(activeRangeCircle, range);
        activeRangeCircle.gameObject.SetActive(showRangeCircle);
    }

    private void GenerateCirclePoints(LineRenderer line, float radius)
    {
        line.positionCount = circleSegments + 1;
        float angleStep = 360f / circleSegments;
        for (int i = 0; i <= circleSegments; i++)
        {
            float angle = Mathf.Deg2Rad * (i * angleStep);
            float x = radius * Mathf.Cos(angle);
            float z = radius * Mathf.Sin(angle);
            line.SetPosition(i, new Vector3(x, 0.01f, z));
        }
    }
    public void ToggleRangeCircleVisibility()
    {
        showRangeCircle = !showRangeCircle;

        if (activeRangeCircle != null)
        {
            activeRangeCircle.gameObject.SetActive(showRangeCircle);
        }

        Debug.Log("Range Circle visibility set to: " + showRangeCircle);
    }

    void OnDestroy()
    {
        if (activeRangeCircle != null)
        {
            Destroy(activeRangeCircle.gameObject);
        }
    }
}