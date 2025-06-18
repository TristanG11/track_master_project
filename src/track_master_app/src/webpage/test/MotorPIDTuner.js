import React, { useState } from "react";
import ROSLIB from "roslib";
import ros from "../common/ROSConnection";

const MotorPIDTuner = () => {
  const [kp, setKp] = useState({ fl: 0, fr: 0, rl: 0, rr: 0 });
  const [ki, setKi] = useState({ fl: 0, fr: 0, rl: 0, rr: 0 });
  const [kd, setKd] = useState({ fl: 0, fr: 0, rl: 0, rr: 0 });
  const [mode, setMode] = useState("Tune"); // "Tune" ou "Send"

  const topic = new ROSLIB.Topic({
    ros: ros,
    name: "/pid_gains",
    messageType: "msg_utils/FourMotorsPid",
  });

  const handleSend = () => {
    const pidMessage = {
      motor_front_left: { kp: kp.fl, ki: ki.fl, kd: kd.fl },
      motor_front_right: { kp: kp.fr, ki: ki.fr, kd: kd.fr },
      motor_rear_left: { kp: kp.rl, ki: ki.rl, kd: kd.rl },
      motor_rear_right: { kp: kp.rr, ki: ki.rr, kd: kd.rr },
    };

    topic.publish(pidMessage);
    console.log("PID gains sent:", pidMessage);
  };

  const handleModeToggle = () => {
    const newMode = mode === "Tune" ? "Send" : "Tune";
    setMode(newMode);

    if (newMode === "Send") {
      handleSend();
    }

    console.log(`Mode switched to: ${newMode}`);
  };

  const renderSlider = (label, motor, min, max, step, value, setValue) => (
    <div style={{ marginBottom: "20px" }}>
      <label>
        {label} ({value[motor].toFixed(2)})
      </label>
      <input
        type="range"
        min={min}
        max={max}
        step={step}
        value={value[motor]}
        onChange={(e) =>
          setValue((prev) => ({ ...prev, [motor]: parseFloat(e.target.value) }))
        }
        disabled={mode === "Send"}
      />
    </div>
  );

  const motors = ["fl", "fr", "rl", "rr"]; // Les noms des moteurs

  return (
    <div style={{ padding: "20px", backgroundColor: "#f4f4f4" }}>
      <h2>Motor PID Tuner</h2>
      <div style={{ display: "flex", justifyContent: "space-between" }}>
        {motors.map((motor) => (
          <div key={motor} style={{ margin: "0 20px" }}>
            <h3>{motor.toUpperCase()}</h3>
            {renderSlider("Kp", motor, 0, 1500, 0.5, kp, setKp)}
            {renderSlider("Ki", motor, 0, 120, 0.05, ki, setKi)}
            {renderSlider("Kd", motor, 0, 20, 0.05, kd, setKd)}
          </div>
        ))}
      </div>
      <button
        onClick={handleModeToggle}
        style={{
          padding: "10px 20px",
          backgroundColor: mode === "Send" ? "red" : "green",
          color: "white",
          border: "none",
          borderRadius: "5px",
          cursor: "pointer",
          marginTop: "20px",
        }}
      >
        {mode === "Tune" ? "Send" : "Tune"}
      </button>
    </div>
  );
};

export default MotorPIDTuner;
