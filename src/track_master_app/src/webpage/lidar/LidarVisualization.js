import React, { useState, useEffect } from "react";
import ROSLIB from "roslib";
import ros from "../common/ROSConnection";
import LidarVizBackground from "./LidarVizBackground";

const LidarVisualization = () => {
  const [lidarPoints, setLidarPoints] = useState([]);

  useEffect(() => {
    // Souscription au topic LiDAR
    const lidarTopic = new ROSLIB.Topic({
      ros: ros,
      name: "/scan", // Topic des données LiDAR
      messageType: "sensor_msgs/LaserScan",
    });

    lidarTopic.subscribe((message) => {
      const points = [];
      const angleMin = message.angle_min;
      const angleIncrement = message.angle_increment;

      // Conversion des données en coordonnées cartésiennes (x, y)
      message.ranges.forEach((range, index) => {
        if (range >= message.range_min && range <= message.range_max && isFinite(range)) {
          const angle = angleMin + index * angleIncrement;
          const x = range * Math.cos(angle); // Conversion en mètres
          const y = range * Math.sin(angle);
          points.push({ x, y });
        }
      });

      setLidarPoints(points);
    });

    // Nettoyage lors du démontage du composant
    return () => {
      lidarTopic.unsubscribe();
    };
  }, []);

  return (
    <div>
      {/* Passe les points au composant de fond */}
      <LidarVizBackground lidarPoints={lidarPoints} />
    </div>
  );
};

export default LidarVisualization;
