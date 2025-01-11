import React, { useState, useEffect } from "react";
import {
  Chart as ChartJS,
  LineElement,
  PointElement,
  LinearScale,
  CategoryScale,
  Title,
  Tooltip,
  Legend,
} from "chart.js";
import { Line } from "react-chartjs-2";
import ROSLIB from "roslib";
import ros from "../common/ROSConnection";

// Enregistrer les composants nécessaires pour Chart.js
ChartJS.register(LineElement, PointElement, LinearScale, CategoryScale, Title, Tooltip, Legend);

const MotorFeedback = () => {
  // États pour stocker les données des graphiques
  const [data, setData] = useState({
    fl: { labels: [], datasets: [{ data: [] }, { data: [] }] },
    fr: { labels: [], datasets: [{ data: [] }, { data: [] }] },
    rl: { labels: [], datasets: [{ data: [] }, { data: [] }] },
    rr: { labels: [], datasets: [{ data: [] }, { data: [] }] },
  });

  // États pour gérer le temps de réponse
  const [responseTimes, setResponseTimes] = useState({
    fl: null,
    fr: null,
    rl: null,
    rr: null,
  });

  const [lastDesiredSpeed, setLastDesiredSpeed] = useState({
    fl: null,
    fr: null,
    rl: null,
    rr: null,
  });

  const [startTime, setStartTime] = useState({
    fl: null,
    fr: null,
    rl: null,
    rr: null,
  });

  useEffect(() => {
    // Configurer le topic ROS
    const topic = new ROSLIB.Topic({
      ros: ros,
      name: "/motor_feedback",
      messageType: "msg_utils/FourMotorsFeedback",
    });

    // Abonnement au topic
    topic.subscribe((message) => {
      const time = new Date().toLocaleTimeString();

      // Récupérer les données des moteurs
      const motors = {
        fl: message.motor_front_left,
        fr: message.motor_front_right,
        rl: message.motor_rear_left,
        rr: message.motor_rear_right,
      };

      Object.keys(motors).forEach((motorKey) => {
        const motor = motors[motorKey];
        const desiredSpeed = motor.desired_speed;
        const trueSpeed = motor.speed;

        // Si la vitesse désirée change, réinitialiser le calcul
        if (desiredSpeed !== lastDesiredSpeed[motorKey]) {
          setLastDesiredSpeed((prev) => ({ ...prev, [motorKey]: desiredSpeed }));
          setStartTime((prev) => ({ ...prev, [motorKey]: Date.now() }));
          setResponseTimes((prev) => ({ ...prev, [motorKey]: null })); // Réinitialiser le temps de réponse
        }

        // Si le temps de réponse n'a pas encore été calculé
        if (responseTimes[motorKey] === null && desiredSpeed > 0) {
          const targetSpeed = desiredSpeed * 0.95; // 95% de desired_speed
          if (Math.abs(trueSpeed - targetSpeed) <= 0.05 * desiredSpeed) {
            const t1 = Date.now();
            setResponseTimes((prev) => ({
              ...prev,
              [motorKey]: ((t1 - startTime[motorKey]) / 1000).toFixed(2),
            })); // Temps en secondes
          }
        }

        // Mettre à jour les données pour le graphique
        setData((prev) => ({
          ...prev,
          [motorKey]: {
            labels: [...prev[motorKey].labels.slice(-19), time],
            datasets: [
              {
                label: "Desired Speed",
                data: [...(prev[motorKey].datasets[0]?.data || []).slice(-19), desiredSpeed],
                borderColor: "red",
                backgroundColor: "rgba(255, 0, 0, 0.5)",
              },
              {
                label: "True Speed",
                data: [...(prev[motorKey].datasets[1]?.data || []).slice(-19), trueSpeed],
                borderColor: "blue",
                backgroundColor: "rgba(0, 0, 255, 0.5)",
              },
            ],
          },
        }));
      });
    });

    return () => topic.unsubscribe(); // Nettoyer l'abonnement
  }, [responseTimes, lastDesiredSpeed, startTime]);

  const options = {
    responsive: true,
    plugins: {
      legend: {
        position: "top",
      },
      title: {
        display: true,
        text: "Motor Feedback",
      },
    },
    scales: {
      x: {
        type: "category",
        title: {
          display: true,
          text: "Time",
        },
      },
      y: {
        title: {
          display: true,
          text: "Speed (rad/s)",
        },
      },
    },
  };

  return (
    <div style={{ padding: "20px" }}>
      <h2>Motor Feedback</h2>
      <div style={{ display: "flex", flexWrap: "wrap", gap: "20px" }}>
        {["fl", "fr", "rl", "rr"].map((motorKey) => (
          <div key={motorKey} style={{ flex: "1 1 45%" }}>
            <h3>{motorKey.toUpperCase()} Motor</h3>
            <Line data={data[motorKey]} options={options} />
            <p>
              Response Time (95%):{" "}
              {responseTimes[motorKey] !== null ? `${responseTimes[motorKey]} sec` : "N/A"}
            </p>
          </div>
        ))}
      </div>
    </div>
  );
};

export default MotorFeedback;
