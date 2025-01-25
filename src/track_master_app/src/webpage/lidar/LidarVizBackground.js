import React from "react";
import { Stage, Layer, Circle, Line, Text, Arrow } from "react-konva";

const LidarVizBackground = ({ lidarPoints = [] }) => {
  const lidarRange = 12; // Maximum LiDAR range in meters
  const circleSteps = [2, 4, 6, 8, 10, 12]; // Circle distances in meters
  const scale = 17; // Scale to convert meters to pixels (1m = 17px)
  const canvasSize = 450; // Canvas size (450x450)
  const center = canvasSize / 2; // Center coordinates

  return (
    <div style={{ backgroundColor: "black", padding: "20px" }}>
      <Stage width={canvasSize} height={canvasSize}>
        <Layer>
          {/* Concentric circles */}
          {circleSteps.map((distance, index) => (
            <Circle
              key={index}
              x={center}
              y={center}
              radius={distance * scale} // Distance multiplied by scale
              stroke="white"
              strokeWidth={1}
              dash={[5, 5]} // Dashed line style
            />
          ))}

          {/* Axes (crossed lines at the center) */}
          <Line
            points={[center, 0, center, canvasSize]} // Vertical line
            stroke="white"
            strokeWidth={1}
          />
          <Line
            points={[0, center, canvasSize, center]} // Horizontal line
            stroke="white"
            strokeWidth={1}
          />

          {/* Labels for the circles */}
          {circleSteps.map((distance, index) => (
            <Text
              key={`label-${index}`}
              x={center + distance * scale + 5} // Offset to the right of the circle
              y={center - 10} // Aligned at the horizontal axis level
              text={`${distance}m`}
              fontSize={10}
              fill="white"
            />
          ))}

          {/* Central circle to represent the origin */}
          <Circle x={center} y={center} radius={3} fill="red" />

          {/* Direction indicator (forward and right) */}
          <Arrow
            points={[center, center, center, center - 50]} // Line pointing upward
            pointerLength={10}
            pointerWidth={10}
            fill="green"
            stroke="green"
            strokeWidth={1}
          />
          <Arrow
            points={[center, center, center + 50, center]} // Line pointing right
            pointerLength={10}
            pointerWidth={10}
            fill="blue"
            stroke="blue"
            strokeWidth={1}
          />

          {/* LiDAR points */}
          {lidarPoints.map((point, index) => (
            <Circle
              key={index}
              x={center + point.x * scale} // Convert coordinates
              y={center - point.y * scale} // Convert coordinates
              radius={1} // Point size
              fill="yellow" // Point color
            />
          ))}
        </Layer>
      </Stage>
    </div>
  );
};

export default LidarVizBackground;
