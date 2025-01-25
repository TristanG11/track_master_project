import React, { useState, useEffect } from "react";
import ROSLIB from "roslib";
import ros from "../common/ROSConnection";
import BatteryGauge from "react-battery-gauge";

const BatteryStatus = () => {
  const [batteryStatus, setBatteryStatus] = useState({
    voltage: 0,
    current: 0,
    charge_level: 0,
    charging: false,
  });

  useEffect(() => {
    // Subscribe to the ROS topic for battery status
    const batteryTopic = new ROSLIB.Topic({
      ros: ros,
      name: "/battery_status",
      messageType: "msg_utils/BatteryStatus",
    });

   // Update the local state with the received data
    batteryTopic.subscribe((message) => {
      setBatteryStatus({
        voltage: message.voltage,
        current: message.current,
        charge_level: message.charge_level,
        charging: message.charging,
      });
    });

    // Cleanup the subscription when the component unmounts
    return () => {
      batteryTopic.unsubscribe();
    };
  }, []);

  return (
    <div>
      <h2>Battery Status</h2>
      {/* Display the battery gauge */}
      <BatteryGauge
        value={batteryStatus.charge_level} // Charge level (percentage)
        charging={batteryStatus.charging} // Charging indicator
        size={200} // Gauge size
        customization={{
          batteryMeter: {
            fill: batteryStatus.charge_level > 20 ? "green" : "red", // Green color if > 20%, red otherwise
            lowBatteryValue: 20, // Low battery threshold
            lowBatteryFill: "red",
          },
          readingText: {
            fontSize: 16, // Font size
            showPercentage: true, // Display percentage
            lowBatteryColor: "red", // Text color if battery is low
          },
        }}
      />
      {/* Additional information */}
      <p>Voltage: {batteryStatus.voltage} V</p>
      <p>Current: {batteryStatus.current} A</p>
      <p>
        Charging: {batteryStatus.charging ? "Yes" : "No"}
      </p>
    </div>
  );
};

export default BatteryStatus;
