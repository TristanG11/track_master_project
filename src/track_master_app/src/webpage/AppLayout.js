import React from "react";
import JoystickControl from "./joystick/JoystickControl";
import MotorsStatus from "./motors_status/MotorsStatus";
import BatteryStatus from "./battery_status/BatteryStatus";
import GNSSDisplay from "./gnss/GNSSDisplay";
import LidarVisualization from "./lidar/LidarVisualization";
import MotorPIDTuner from "./test/MotorPIDTuner";
import MotorFeedback from "./test/MotorFeedback";
import CameraView from "./camera/CameraView";

const AppLayout = () => {
  return (
    <div style={{ padding: "20px", maxWidth: "1200px", margin: "0 auto" }}>
      <h1 style={{ textAlign: "center" }}>Track Master Interface</h1>

      <div
        style={{
          display: "flex",
          flexWrap: "nowrap",
          justifyContent: "flex-start",
          alignItems: "flex-start",
          gap: "20px",
          marginBottom: "30px",
        }}
      >
        <JoystickControl style={{ flex: "0 0 150px", minWidth: "100px" }} />
        <BatteryStatus style={{ flex: "0 0 150px", minWidth: "120px" }} />
      </div>

      {/* GNSS et LiDAR / caméra */}
      <div
        style={{
          display: "flex",
          flexWrap: "wrap",        // MODIF : rend le layout adaptatif
          justifyContent: "space-around",
          alignItems: "flex-start",
          gap: "20px",
          marginTop: "20px",
          marginBottom: "10px",
        }}
      >
        <CameraView style={{ flex: "1 1 300px", minWidth: "200px" }} />
        <LidarVisualization style={{ flex: "1 1 300px", minWidth: "200px" }} />
      </div>

      {/* GNSS display */}
      <div style={{ marginTop: "30px" }}>
        <GNSSDisplay />
      </div>

      {/*moteurs / batterie */}
      <div
        style={{
          display: "flex",
          flexWrap: "wrap",          // MODIF : permet aux éléments de passer en colonne sur mobile
          justifyContent: "space-around",
          alignItems: "flex-start",
          gap: "20px",
        }}
      >
        <MotorsStatus style={{ flex: "1 1 200px", minWidth: "150px" }} />
      </div>

      {/* Feedback moteur */}
      <div style={{ marginTop: "30px" }}>
        <MotorFeedback />
      </div>

      {/* PID test */}
      <div style={{ marginTop: "30px" }}>
        <MotorPIDTuner />
      </div>*/
    </div>
  );
};

export default AppLayout;
