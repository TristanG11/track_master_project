import React from "react";
import JoystickControl from "./joystick/JoystickControl";
import MotorsStatus from "./motors_status/MotorsStatus";
import BatteryStatus from "./battery_status/BatteryStatus";
import GNSSDisplay from "./gnss/GNSSDisplay";
import LidarVisualization from "./lidar/LidarVisualization";
import MotorPIDTuner from "./test/MotorPIDTuner";
import MotorFeedback from "./test/MotorFeedback";
import CameraView from "./camera/CameraView"
const AppLayout = () => {
  return (
    <div style={{ padding: "20px" }}>
      <h1 style={{ textAlign: "center" }}>Track Master Interface</h1>

      {/* Side-by-side layout */}
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

      <div style={{ marginTop: "30px" }}>
        <MotorFeedback />
      </div>
      {/* PID test window */}
      {
      <div style={{ marginTop: "30px" }}>
        <MotorPIDTuner />
      </div> 
      }


      {/* GNSS and LiDAR below */}
      
      <div style={{
          display: "flex",
          justifyContent: "space-around",
          alignItems: "flex-start",
          gap: "20px",
          marginTop: "20px",
          marginBottom: "10px",
        }}>
        
        <LidarVisualization />
        <CameraView />
      </div>

      <div style={{ marginTop: "30px" }}>
        <GNSSDisplay />
      </div>
    

      

      
    </div>
  );
};

export default AppLayout;
