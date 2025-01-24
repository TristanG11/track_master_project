import React, { useState, useEffect } from "react";
import ROSLIB from "roslib";
import ros from "../common/ROSConnection";
import LidarVizBackground from "./LidarVizBackground";
import styles from './LidarVisualization.module.css'; // Ajoute le chemin correct au fichier CSS

const LidarVisualization = () => {
  const [lidarPoints, setLidarPoints] = useState([]);

  useEffect(() => {
    const lidarTopic = new ROSLIB.Topic({
      ros: ros,
      name: "/scan",
      messageType: "sensor_msgs/LaserScan",
    });

    const updateLidarPoints = (message) => {
      const points = [];
      const angleMin = message.angle_min;
      const angleIncrement = message.angle_increment;

      message.ranges.forEach((range, index) => {
        if (range >= message.range_min && range <= message.range_max && isFinite(range)) {
          const angle = angleMin + index * angleIncrement;
          const x = range * Math.cos(angle);
          const y = range * Math.sin(angle);
          points.push({ x, y });
        }
      });

      setLidarPoints(points);
    };

    lidarTopic.subscribe(updateLidarPoints);

    return () => {
      lidarTopic.unsubscribe();
    };
  }, []);

  return (
    <div className={styles.lidarContainer}>
      <h2>Lidar Visualization</h2>
      <LidarVizBackground lidarPoints={lidarPoints} />
    </div>
  );
};

export default LidarVisualization;
