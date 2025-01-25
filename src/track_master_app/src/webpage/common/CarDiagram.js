import React from "react";
import { Stage, Layer, Rect, Circle, Text } from "react-konva";

const CarDiagram = ({ motorStatus }) => {
  return (
    <Stage width={500} height={300}>
      <Layer>
        {/* Chassis */}
        <Rect
          x={175}
          y={100}
          width={75}
          height={150}
          stroke="black"
          strokeWidth={5}
          cornerRadius={10}
          fill="#d3d3d3"
        />

        {/* Wheels */}
        <Circle x={175} y={100} radius={10} fill="black" />
        <Circle x={250} y={100} radius={10} fill="black" />
        <Circle x={175} y={250} radius={10} fill="black" />
        <Circle x={250} y={250} radius={10} fill="black" />

        {/* Informations for each wheel */}
        <Text x={30} y={80} text={`FL Speed: ${motorStatus.motor_front_left.speed} m/s`} />
        <Text x={30} y={100} text={`FL Voltage: ${motorStatus.motor_front_left.voltage} V`} />
        <Text x={30} y={120} text={`FL Current: ${motorStatus.motor_front_left.current} A`} />

        <Text x={300} y={80} text={`FR Speed: ${motorStatus.motor_front_right.speed} m/s`} />
        <Text x={300} y={100} text={`FR Voltage: ${motorStatus.motor_front_right.voltage} V`} />
        <Text x={300} y={120} text={`FR Current: ${motorStatus.motor_front_right.current} A`} />

        <Text x={30} y={240} text={`RL Speed: ${motorStatus.motor_rear_left.speed} m/s`} />
        <Text x={30} y={260} text={`RL Voltage: ${motorStatus.motor_rear_left.voltage} V`} />
        <Text x={30} y={280} text={`RL Current: ${motorStatus.motor_rear_left.current} A`} />

        <Text x={300} y={240} text={`RR Speed: ${motorStatus.motor_rear_right.speed} m/s`} />
        <Text x={300} y={260} text={`RR Voltage: ${motorStatus.motor_rear_right.voltage} V`} />
        <Text x={300} y={280} text={`RR Current: ${motorStatus.motor_rear_right.current} A`} />
      </Layer>
    </Stage>
  );
};

export default CarDiagram;
