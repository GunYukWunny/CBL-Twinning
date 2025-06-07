using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class BatteryStatusUI : MonoBehaviour
{
    ROSConnection ros;
    public string batteryTopic = "/battery_state";

    public Slider batteryBar;
    public Text batteryText;

    private float batteryPercentage = 100f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<BatteryStateMsg>(batteryTopic, BatteryCallback);
    }

    void BatteryCallback(BatteryStateMsg msg)
    {
        batteryPercentage = msg.percentage * 100f;
    }

    void Update()
    {
        batteryBar.value = batteryPercentage;
        batteryText.text = $"Battery: {batteryPercentage:F1}%";
    }
} 

