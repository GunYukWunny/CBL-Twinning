using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.BuiltinInterfaces;

public class Ros2GoalPoser : MonoBehaviour
{
    [Header("ROS Goal Publisher Settings")]
    public ROSConnection rosConnection;
    public string goalTopicName = "/goal_pose";

    private bool publisherRegistered = false;

    void Start()
    {
        if (rosConnection == null)
        {
            rosConnection = ROSConnection.GetOrCreateInstance();
        }

        TryRegisterPublisher();
    }

    void Update()
    {
        if (!publisherRegistered && rosConnection != null && !rosConnection.HasConnectionError)
        {
            TryRegisterPublisher();
        }
    }

    private void TryRegisterPublisher()
    {
        try
        {
            rosConnection.RegisterPublisher<PoseStampedMsg>(goalTopicName);
            Debug.Log("Registered ROS2 goal publisher.");
            publisherRegistered = true;
        }
        catch (System.Exception ex)
        {
            Debug.LogError($"ROS Publisher registration failed: {ex.Message}");
        }
    }

    public void SendGoalPose(Vector3 unityWorldPosition)
    {
        if (!publisherRegistered || rosConnection == null || rosConnection.HasConnectionError)
        {
            Debug.LogWarning("Cannot send goal pose: ROS connection not ready.");
            return;
        }

        // Convert Unity position to ROS (X = Z, Y = -X)
        Vector3 rosPosition = new Vector3(unityWorldPosition.z, -unityWorldPosition.x, 0.0f);
        Quaternion rosOrientation = Quaternion.identity;

        PoseStampedMsg goalPose = new PoseStampedMsg();

        double timeNow = Time.realtimeSinceStartupAsDouble;
        goalPose.header.stamp = new TimeMsg((int)timeNow, (uint)((timeNow - (int)timeNow) * 1e9));
        goalPose.header.frame_id = "map";

        goalPose.pose.position.x = rosPosition.x;
        goalPose.pose.position.y = rosPosition.y;
        goalPose.pose.position.z = rosPosition.z;

        goalPose.pose.orientation.x = rosOrientation.x;
        goalPose.pose.orientation.y = rosOrientation.y;
        goalPose.pose.orientation.z = rosOrientation.z;
        goalPose.pose.orientation.w = rosOrientation.w;

        rosConnection.Publish(goalTopicName, goalPose);
        Debug.Log($"Published goal to ROS: X={rosPosition.x}, Y={rosPosition.y}");

        // Visual feedback
        GameObject marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        marker.transform.position = unityWorldPosition;
        marker.transform.localScale = Vector3.one * 0.2f;
        marker.GetComponent<Renderer>().material.color = Color.blue;
        Destroy(marker, 5f);
    }
}

