import React, { useState, useEffect } from "react";
import ROSLIB from "roslib";
import ros from "../common/ROSConnection";
import CarDiagram from "../common/CarDiagram";

const CarStatus = () => {
  const [motorStatus, setMotorStatus] = useState({
    motor_front_left: { speed: 0, voltage: 0, current: 0 },
    motor_front_right: { speed: 0, voltage: 0, current: 0 },
    motor_rear_left: { speed: 0, voltage: 0, current: 0 },
    motor_rear_right: { speed: 0, voltage: 0, current: 0 },
  });

  useEffect(() => {
    const motorStatusTopic = new ROSLIB.Topic({
      ros: ros,
      name: "/cmd_vel_feedback",
      messageType: "msg_utils/FourMotorsFeedback",
    });

    motorStatusTopic.subscribe((message) => {
      setMotorStatus({
        motor_front_left: {
          speed: message.motor_front_left.speed,
          //voltage: message.motor_front_left.voltage,
          //current: message.motor_front_left.current,
        },
        motor_front_right: {
          speed: message.motor_front_right.speed,
          voltage: message.motor_front_right.voltage,
          current: message.motor_front_right.current,
        },
        motor_rear_left: {
          speed: message.motor_rear_left.speed,
          //voltage: message.motor_rear_left.voltage,
          //current: message.motor_rear_left.current,
        },
        motor_rear_right: {
          speed: message.motor_rear_right.speed,
          //voltage: message.motor_rear_right.voltage,
          //current: message.motor_rear_right.current,
        },
      });
    });

    return () => {
      motorStatusTopic.unsubscribe();
    };
  }, []);

  return <CarDiagram motorStatus={motorStatus} />;
};

export default CarStatus;
