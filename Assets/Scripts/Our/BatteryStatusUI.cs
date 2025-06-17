using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using System;

public class BatteryStatusUI : MonoBehaviour
{
    ROSConnection ros;
    public string batteryTopic = "/battery_state";

    public Slider batteryBar;
    public Text batteryText;
    public Text textMeshProGUI;

    [Range(0, 100)]
    public float batteryPercentage = 100f;

    public Boolean overwriteBattery;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<BatteryStateMsg>(batteryTopic, BatteryCallback);
    }

    void BatteryCallback(BatteryStateMsg msg)
    {
        if (!overwriteBattery)
        {
            batteryPercentage = msg.percentage * 100f;
        }

    }

    void Update()
    {
        if (overwriteBattery)
        {
            batteryPercentage = batteryBar.value;
            batteryText.text = $"Battery: {batteryPercentage:F1}%";
        }
        else
        {
            batteryBar.value = batteryPercentage;
            batteryText.text = $"Battery: {batteryPercentage:F1}%";
        }
    }
}
