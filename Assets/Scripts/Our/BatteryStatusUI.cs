using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using System;

public class BatteryStatusUI : MonoBehaviour
{
    private ROSConnection ros;
    public string batteryTopic = "/battery_state";

    public Slider batteryBar;
    public Text batteryText;

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
        }
        else
        {
            batteryBar.value = batteryPercentage;
        }

        batteryText.text = "Battery: " + Math.Floor(batteryPercentage) + " % ";
    }

    public void ToggleOverwriteBattery()
    {
        overwriteBattery = !overwriteBattery;
        Debug.Log("Overwrite Battery toggled to: " + overwriteBattery);
    }
}

