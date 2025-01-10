import React from "react";
import { Stage, Layer, Circle, Line, Text } from "react-konva";

const LidarVizBackground = ({ lidarPoints = []  }) => {
  const lidarRange = 12; // Portée maximale du LiDAR en mètres
  const circleSteps = [0.05, 2, 4, 6, 8, 10, 12]; // Distances des cercles en mètres
  const scale = 30; // Échelle pour convertir les mètres en pixels (1m = 30px)
  const canvasSize = 800; // Taille du canvas (800x800)
  const center = canvasSize / 2; // Coordonnées du centre

  return (
    <div style={{ backgroundColor: "black", padding: "20px" }}>
      <Stage width={canvasSize} height={canvasSize}>
        <Layer>
          {/* Cercles concentriques */}
          {circleSteps.map((distance, index) => (
            <Circle
              key={index}
              x={center}
              y={center}
              radius={distance * scale} // Distance multipliée par l'échelle
              stroke="white"
              strokeWidth={0.5}
            />
          ))}

          {/* Axes (lignes croisées au centre) */}
          <Line
            points={[center, 0, center, canvasSize]} // Ligne verticale
            stroke="white"
            strokeWidth={0.5}
          />
          <Line
            points={[0, center, canvasSize, center]} // Ligne horizontale
            stroke="white"
            strokeWidth={0.5}
          />

          {/* Petits labels pour indiquer les distances */}
          {circleSteps.map((distance, index) => (
            <Text
              key={`label-${index}`}
              x={center + distance * scale + 5} // Décalé à droite du cercle
              y={center - 5} // Aligné au niveau de l'axe horizontal
              text={`${distance}m`}
              fontSize={12}
              fill="white"
            />
          ))}

          {/* Cercle central pour représenter l'origine */}
          <Circle x={center} y={center} radius={5} fill="red" />

          {/* Points du LiDAR */}
          {lidarPoints.map((point, index) => (
            <Circle
              key={index}
              x={center + point.x * scale} // Conversion des coordonnées
              y={center - point.y * scale} // Conversion des coordonnées
              radius={2} // Taille des points
              fill="red"
            />
          ))}
        </Layer>
      </Stage>
    </div>
  );
};

export default LidarVizBackground;
