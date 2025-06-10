using UnityEngine;
using Unity.Robotics.Visualizations;

public class Ros2RobotSync : MonoBehaviour
{
    // very hardcoded way of doing this but for the purpouses of a POC it works

    public GameObject baseFootprintReference;
    public GameObject baseFootprintGameObject;

    public Vector3 baseFootprintOffset;


    public GameObject leftWheelReference;
    public GameObject leftWheelGameObject;

    public GameObject rightWheelReference;
    public GameObject rightWheelGameObject;


    void Start()
    {
        baseFootprintReference = GetObjectReferenceByName("base_footprint");
        rightWheelReference = GetObjectReferenceByName("wheel_right_link");
        leftWheelReference = GetObjectReferenceByName("wheel_left_link");
    }

    void Update()
    {
        if (baseFootprintReference == null)
        {
            baseFootprintReference = GetObjectReferenceByName("base_footprint");
        }
        else
        {
            baseFootprintGameObject.transform.rotation = baseFootprintReference.transform.rotation;
            baseFootprintGameObject.transform.position = baseFootprintReference.transform.position + baseFootprintOffset;
        }




        if (rightWheelReference == null)
        {
            rightWheelReference = GetObjectReferenceByName("wheel_right_link");
        }
        else
        {
            rotateWheel(ref rightWheelGameObject, ref rightWheelReference);
            foreach (Transform child in rightWheelGameObject.transform)
            {
                child.localRotation = Quaternion.identity;
            }
        }
        if (leftWheelReference == null)
        {
            leftWheelReference = GetObjectReferenceByName("wheel_left_link");
        }
        else
        {
            rotateWheel(ref leftWheelGameObject, ref leftWheelReference);
            foreach (Transform child in rightWheelGameObject.transform)
            {
                child.localRotation = Quaternion.identity;
            }
        }
    }

    private GameObject GetObjectReferenceByName(string name)
    {
        GameObject foundObject = GameObject.Find(name);
        if (foundObject == null)
        {
            Debug.LogWarning($"GameObject with name '{name}' not found!");
        }
        return foundObject;
    }

    private void rotateWheel(ref GameObject wheel, ref GameObject referenceWheel)
    {
        Vector3 referenceEuler = referenceWheel.transform.rotation.eulerAngles;

        Vector3 currentEuler = wheel.transform.rotation.eulerAngles;

        Vector3 newEuler = new Vector3(referenceEuler.x, 0, 0);

        wheel.transform.localRotation = Quaternion.Euler(newEuler);
    }
}

