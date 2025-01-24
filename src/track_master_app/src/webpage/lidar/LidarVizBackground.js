import React from "react";
import { Stage, Layer, Circle, Line, Text, Arrow } from "react-konva";

const LidarVizBackground = ({ lidarPoints = [] }) => {
  const lidarRange = 12; // Portée maximale du LiDAR en mètres
  const circleSteps = [2, 4, 6, 8, 10, 12]; // Distances des cercles en mètres
  const scale = 17; // Échelle pour convertir les mètres en pixels (1m = 30px)
  const canvasSize = 450; // Taille du canvas (800x800)
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
              strokeWidth={1}
              dash={[5, 5]} // Style en pointillé
            />
          ))}

          {/* Axes (lignes croisées au centre) */}
          <Line
            points={[center, 0, center, canvasSize]} // Ligne verticale
            stroke="white"
            strokeWidth={1}
          />
          <Line
            points={[0, center, canvasSize, center]} // Ligne horizontale
            stroke="white"
            strokeWidth={1}
          />

          {/* Labels pour les cercles */}
          {circleSteps.map((distance, index) => (
            <Text
              key={`label-${index}`}
              x={center + distance * scale + 5} // Décalé à droite du cercle
              y={center - 10} // Aligné au niveau de l'axe horizontal
              text={`${distance}m`}
              fontSize={10}
              fill="white"
            />
          ))}

          {/* Cercle central pour représenter l'origine */}
          <Circle x={center} y={center} radius={3} fill="red" />

          {/* Indicateur de direction (avant et droite) */}
          <Arrow
            points={[center, center, center, center - 50]} // Ligne pointant vers le haut
            pointerLength={10}
            pointerWidth={10}
            fill="green"
            stroke="green"
            strokeWidth={1}
          />
          <Arrow
            points={[center, center, center + 50, center]} // Ligne pointant vers la droite
            pointerLength={10}
            pointerWidth={10}
            fill="blue"
            stroke="blue"
            strokeWidth={1}
          />

          {/* Points du LiDAR */}
          {lidarPoints.map((point, index) => (
            <Circle
              key={index}
              x={center + point.x * scale} // Conversion des coordonnées
              y={center - point.y * scale} // Conversion des coordonnées
              radius={1} // Taille des points
              fill="yellow" // Couleur des points
            />
          ))}
        </Layer>
      </Stage>
    </div>
  );
};

export default LidarVizBackground;
