import React, { useState, useEffect } from "react";
import { Joystick } from "react-joystick-component";
import ros from "../common/ROSConnection";
import ROSLIB from "roslib";

const JoystickControl = () => {
  const [joystickState, setJoystickState] = useState({ x: 0, y: 0 });
  const [linearSpeed, setLinearSpeed] = useState(0.0);
  const [angularSpeed, setAngularSpeed] = useState(0.0);
  const [useAppJoystick, setUseAppJoystick] = useState(true); // Définit si on utilise le joystick de l'application

  // Paramètres de conversion
  const maxLinearSpeed = 0.5;
  const minLinearSpeed = -0.3;
  const maxAngularSpeed = 0.5;
  const minAngularSpeed = -0.5;
  const maxJoy = 1.0; // Échelle maximale de l'entrée du joystick
  const deadzoneThreshold = 0.05; // La valeur de la zone morte (ici 0)

  const joyToLin = (joyVal) => {
    if (joyVal >= deadzoneThreshold) {
      const coeff = (maxLinearSpeed - 0.0) / (maxJoy - deadzoneThreshold);
      const d = 0.0 - coeff * deadzoneThreshold;
      return coeff * joyVal + d;
    } else if (joyVal <= -deadzoneThreshold) {
      const coeff = (0.0 + maxLinearSpeed) / (-deadzoneThreshold + maxJoy);
      const d = 0.0 + coeff * deadzoneThreshold;
      return coeff * joyVal + d;
    } else {
      return 0.0;
    }
  };

  const joyToAng = (joyVal) => {
    if (joyVal >= deadzoneThreshold) {
      const coeff = (maxAngularSpeed - 0.0) / (maxJoy - deadzoneThreshold);
      const d = 0.0 - coeff * deadzoneThreshold;
      return coeff * joyVal + d;
    } else if (joyVal <= -deadzoneThreshold) {
      const coeff = (-0.0 - minAngularSpeed) / (-deadzoneThreshold + maxJoy);
      const d = -0.0 + coeff * deadzoneThreshold;
      return coeff * joyVal + d;
    } else {
      return 0.0;
    }
  };

  const handleMove = (event) => {
    setJoystickState({ x: event.x, y: event.y });
  };

  const handleStop = () => {
    setJoystickState({ x: 0, y: 0 });
  };

  // Publier la commande pour basculer entre manette PS4 et joystick app
  const publishUseAppJoystick = (value) => {
    const useAppJoystickTopic = new ROSLIB.Topic({
      ros: ros,
      name: "/use_app_joystick",
      messageType: "std_msgs/Bool",
    });

    const message = new ROSLIB.Message({
      data: value,
    });

    useAppJoystickTopic.publish(message);
    console.log(`Published /use_app_joystick: ${value}`);
  };

  const toggleJoystickMode = () => {
    setUseAppJoystick((prev) => {
      const newValue = !prev;
      publishUseAppJoystick(newValue); // Publie la nouvelle valeur uniquement ici
      return newValue;
    });
  };

  useEffect(() => {
    if (useAppJoystick) {
      const interval = setInterval(() => {
        const linear = joyToLin(joystickState.y);
        const angular = -joyToAng(joystickState.x);

        setLinearSpeed(linear.toFixed(2));
        setAngularSpeed(angular.toFixed(2));

        // Publier la commande de vitesse
        const cmdVelTopic = new ROSLIB.Topic({
          ros: ros,
          name: "/diff_drive_controller/cmd_vel_unstamped",
          messageType: "geometry_msgs/Twist",
        });

        const twist = new ROSLIB.Message({
          linear: { x: linear, y: 0.0, z: 0.0 },
          angular: { x: 0.0, y: 0.0, z: angular },
        });

        cmdVelTopic.publish(twist);
      }, 10); // Publier toutes les 10 ms

      return () => clearInterval(interval); // Nettoyer l'intervalle à la désactivation
    }
  }, [joystickState, useAppJoystick]); // Dépendances : joystickState, useAppJoystick

  return (
    <div>
      <h2>Joystick Control</h2>
      <button
        onClick={toggleJoystickMode}
        style={{
          padding: "10px 20px",
          backgroundColor: useAppJoystick ? "green" : "red",
          color: "white",
          border: "none",
          borderRadius: "5px",
          cursor: "pointer",
          marginBottom: "10px",
        }}
      >
        {useAppJoystick ? "Switch to PS4 Controller" : "Switch to App Joystick"}
      </button>

      {useAppJoystick && (
        <div>
          <Joystick
            size={100}
            baseColor="lightgray"
            stickColor="blue"
            move={handleMove}
            stop={handleStop}
          />
          <p>Linear Speed: {linearSpeed}</p>
          <p>Angular Speed: {angularSpeed}</p>
        </div>
      )}
    </div>
  );
};

export default JoystickControl;
