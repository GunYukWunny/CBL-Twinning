using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class Ros2PathPlanner : MonoBehaviour
{
    [Header("Dependencies")]
    public Transform robotTransform;
    public Camera sceneCamera;
    public Ros2GoalPoser externalSender;

    [Header("Path Rendering")]
    public LineRenderer lineRendererPrefab;
    public GameObject goalMarkerPrefab;
    public LayerMask clickableLayers;

    [Header("Behavior Settings")]
    public float waypointThreshold = 0.5f;
    public float resendInterval = 2.0f;
    public float pathRecheckInterval = 5.0f;

    [Header("UI")]
    public GUIStyle guiStyle;

    private NavMeshPath currentPath;
    private int currentWaypointIndex = 0;
    private bool pathActive = false;
    private Vector3 currentGoalPoint;

    private LineRenderer activeLineRenderer;
    private List<GameObject> activeMarkers = new List<GameObject>();

    private string statusMessage = "";
    private float statusTime = 0f;
    private float messageDuration = 3f;

    private float resendTimer = 0f;
    private float pathRecheckTimer = 0f;

    void Start()
    {
        currentPath = new NavMeshPath();
        guiStyle.fontSize = 24;
        guiStyle.normal.textColor = Color.white;
    }

    void Update()
    {
        HandleClick();
        HandlePathRecheck();

        if (pathActive && currentWaypointIndex < currentPath.corners.Length)
        {
            Vector3 target = currentPath.corners[currentWaypointIndex];
            Vector3 flatRobot = new Vector3(robotTransform.position.x, 0, robotTransform.position.z);
            Vector3 flatTarget = new Vector3(target.x, 0, target.z);
            float distance = Vector3.Distance(flatRobot, flatTarget);

            resendTimer += Time.deltaTime;
            if (resendTimer >= resendInterval)
            {
                externalSender.SendGoalPose(target);
                resendTimer = 0f;
            }

            if (distance <= waypointThreshold)
            {
                currentWaypointIndex++;
                if (currentWaypointIndex < currentPath.corners.Length)
                {
                    externalSender.SendGoalPose(currentPath.corners[currentWaypointIndex]);
                    resendTimer = 0f;
                }
                else
                {
                    pathActive = false;
                    ShowMessage("Destination Reached!");
                    ClearPathVisuals();
                }
            }
        }
    }

    private void HandleClick()
    {
        if (Input.GetMouseButtonDown(0))
        {
            Ray ray = sceneCamera.ScreenPointToRay(Input.mousePosition);
            if (Physics.Raycast(ray, out RaycastHit hit, 100f, clickableLayers))
            {
                Vector3 clickedPoint = hit.point;
                currentGoalPoint = clickedPoint;

                AttemptPathCalculation(robotTransform.position, clickedPoint);
            }
        }
    }

    private void HandlePathRecheck()
    {
        if (pathActive)
        {
            pathRecheckTimer += Time.deltaTime;
            if (pathRecheckTimer >= pathRecheckInterval)
            {
                pathRecheckTimer = 0f;
                NavMeshPath checkPath = new NavMeshPath();
                if (currentPath.corners.Length > currentWaypointIndex &&
                    NavMesh.CalculatePath(robotTransform.position, currentPath.corners[currentPath.corners.Length - 1], NavMesh.AllAreas, checkPath) &&
                    checkPath.status == NavMeshPathStatus.PathComplete)
                {
                    if (currentPath.status != NavMeshPathStatus.PathComplete)
                    {
                         AttemptPathCalculation(robotTransform.position, currentGoalPoint);
                    }
                }
                else
                {
                    AttemptPathCalculation(robotTransform.position, currentGoalPoint);
                }
            }
        }
    }

    private void AttemptPathCalculation(Vector3 startPoint, Vector3 endPoint)
    {
        NavMeshPath newPath = new NavMeshPath();
        if (NavMesh.CalculatePath(startPoint, endPoint, NavMesh.AllAreas, newPath) &&
            newPath.status == NavMeshPathStatus.PathComplete &&
            newPath.corners.Length > 1)
        {
            currentPath = newPath;
            currentWaypointIndex = 0;
            pathActive = true;
            resendTimer = 0f;

            ShowMessage("Path Found!");
            externalSender.SendGoalPose(currentPath.corners[currentWaypointIndex]);

            DrawPath();
            DrawGoalMarkers();
        }
        else
        {
            pathActive = false;
            ClearPathVisuals();
            ShowMessage("Target Not Reachable!");
        }
    }

    private void ClearPathVisuals()
    {
        currentPath.ClearCorners();
        if (activeLineRenderer != null)
        {
            Destroy(activeLineRenderer.gameObject);
            activeLineRenderer = null;
        }
        foreach (var marker in activeMarkers)
            Destroy(marker);
        activeMarkers.Clear();
    }

    private void DrawPath()
    {
        if (activeLineRenderer != null)
        {
            Destroy(activeLineRenderer.gameObject);
            activeLineRenderer = null;
        }

        if (currentPath.corners.Length < 2) return;

        activeLineRenderer = Instantiate(lineRendererPrefab);
        activeLineRenderer.positionCount = currentPath.corners.Length;
        activeLineRenderer.SetPositions(currentPath.corners);
    }

    private void DrawGoalMarkers()
    {
        foreach (var marker in activeMarkers)
            Destroy(marker);
        activeMarkers.Clear();

        foreach (var point in currentPath.corners)
        {
            GameObject marker = Instantiate(goalMarkerPrefab, point + Vector3.up * 2, Quaternion.identity);
            marker.transform.localScale = Vector3.one * 0.2f;
            activeMarkers.Add(marker);
        }
    }

    private void OnGUI()
    {
        if (Time.time - statusTime < messageDuration)
        {
            GUI.Label(new Rect(Screen.width - 400, 20, 380, 50), statusMessage, guiStyle);
        }
    }

    private void ShowMessage(string message)
    {
        statusMessage = message;
        statusTime = Time.time;
    }
}
