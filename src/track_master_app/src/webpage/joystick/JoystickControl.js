import React, { useState, useEffect } from "react";
import { Joystick } from "react-joystick-component";
import ros from "../common/ROSConnection";
import ROSLIB from "roslib";

const JoystickControl = () => {
  const [joystickState, setJoystickState] = useState({ x: 0, y: 0 });
  const [linearSpeed, setLinearSpeed] = useState(0.0);
  const [angularSpeed, setAngularSpeed] = useState(0.0);
  const [cmdType, setCmdType] = useState(null); // Selected control type
  const [loading, setLoading] = useState(true); //

  // Conversion parameters
  const maxLinearSpeed = 0.5;
  const minLinearSpeed = -0.5;
  const maxAngularSpeed = 2.0;
  const minAngularSpeed = -2.0;
  const maxJoy = 1.0; // Maximum scale of joystick input
  const deadzoneThreshold = 0.05; // Deadzone value (here 0)

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

  // Publish the control type commands

    const cmdTypeTopic = new ROSLIB.Topic({
      ros: ros,
      name: "/cmd_type",
      messageType: "std_msgs/String",
    });

  const publishCmdType = (value) => {
    const message = new ROSLIB.Message({
      data: value,
    });

    cmdTypeTopic.publish(message);
    //console.log(`Published /cmd_type: ${value}`);
  };

  const handleCmdTypeChange = (event) => {
    const selectedType = event.target.value;
    setCmdType(selectedType);
    publishCmdType(selectedType);
  };

  useEffect(() => {

    if (!cmdType) return;
    const interval = setInterval(() => {
     publishCmdType(cmdType);
    }, 1000);
    return () => clearInterval(interval);
  }, [cmdType]);

  useEffect(() => {
    const service = new ROSLIB.Service({
      ros: ros,
      name: "/app_initializer/get_cmd_type",
      serviceType: "track_master_debug/srv/GetCmdType",
    });

    const request = new ROSLIB.ServiceRequest({});

    service.callService( request, (result) =>{
      console.log("Service /get_cmd_type result:", result);
      if (result && result.cmd_type && result.cmd_type !== "")
      {
        setCmdType(result.cmd_type);
      }
      else {
        setCmdType("app_joystick");
      }
      setLoading(false);
      clearTimeout(timeout); 
    });

    const timeout = setTimeout(() => {
      if (loading) {
        setCmdType("app_joystick");
        setLoading(false);
      }
    }, 5000);

  }, []);




  useEffect(() => {
    if (cmdType === "app_joystick") {
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
          queue_size: 10, // Taille de la file d'attente
          
          qos: {
            durability: "transient_local", // Set durability to match the subscriber
            reliability: "reliable", // Use reliable communication
          },
        });

        const twist = new ROSLIB.Message({
          linear: { x: linear, y: 0.0, z: 0.0 },
          angular: { x: 0.0, y: 0.0, z: angular },
        });

        cmdVelTopic.publish(twist);
      }, 10); // Publish every 10 ms

      return () => clearInterval(interval); // Clear the interval when deactivating
    }
  }, [joystickState, cmdType]); // Dependencies: joystickState, cmdType

  if (loading || cmdType === null) {
    return <p>Chargement du mode de commande...</p>;
  }
  return (
    <div>
      <h2>Joystick Control</h2>

      <select
        value={cmdType}
        onChange={handleCmdTypeChange}
        style={{
          padding: "10px",
          marginBottom: "10px",
          borderRadius: "5px",
        }}
      >
        <option value="app_joystick">App Joystick</option>
        <option value="ps4_controller">PS4 Controller</option>
        <option value="test">test</option>
        <option value="voice_command">Voice Command</option>
      </select>

      {cmdType === "app_joystick" && (
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
