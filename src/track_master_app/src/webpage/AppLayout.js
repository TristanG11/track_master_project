import React from "react";
import JoystickControl from "./joystick/JoystickControl";
import MotorsStatus from "./motors_status/MotorsStatus";
import BatteryStatus from "./battery_status/batteryStatus";
import GNSSDisplay from "./gnss/GNSSDisplay";
import LidarVisualization from "./lidar/LidarVisualization";
import MotorPIDTuner from "./test/MotorPIDTuner";
import MotorFeedback from "./test/MotorFeedback";
const AppLayout = () => {
  return (
    <div style={{ padding: "20px" }}>
      <h1 style={{ textAlign: "center" }}>Robot Interface</h1>

      {/* Disposition côte à côte */}
      <div
        style={{
          display: "flex",
          justifyContent: "space-around",
          alignItems: "flex-start",
          gap: "20px",
        }}
      >
        <JoystickControl />
        <MotorsStatus />
        <BatteryStatus />
      </div>

      {/* GNSS et LiDAR en dessous */}
      <div style={{ marginTop: "30px" }}>
        <GNSSDisplay />
      </div>
      <div style={{ marginTop: "30px" }}>
        <LidarVisualization />
      </div>

      {/* Fenêtre de test PID */}
      <div style={{ marginTop: "30px" }}>
        <MotorPIDTuner />
      </div>

      <div style={{ marginTop: "30px" }}>
        <MotorFeedback />
      </div>
    </div>
  );
};

export default AppLayout;
