import React, { useState, useEffect, useMemo } from "react";
import ROSLIB from "roslib";
import ros from "../common/ROSConnection";
import BatteryGauge from "react-battery-gauge";

/**
 * Composant React pour afficher le statut des batteries et gérer le bouton d'arrêt d'urgence
 */
const PowerStatusDisplay = () => {
  // --- État local pour les tensions mesurées par le robot
  const [powerVoltage, setPowerVoltage] = useState(0);       // Tension batterie power
  const [logicalVoltage, setLogicalVoltage] = useState(0);   // Tension batterie logical

  // --- État local pour refléter l'état d'urgence exécuté par le robot
  const [statusEmergency, setStatusEmergency] = useState(false);

  // --- État local pour piloter le bouton (ce qu'on publie)
  const [buttonPressed, setButtonPressed] = useState(false);

  /**
   * Publisher ROS pour envoyer la commande d'arrêt d'urgence
   * Utilisation de useMemo pour ne créer le topic qu'une fois
   */
  const emergencyTopic = useMemo(
    () => new ROSLIB.Topic({
      ros,
      name: "/emergency_stop_cmd",
      messageType: "std_msgs/Bool",
    }),
    []
  );

  /**
   * Subscription ROS au topic /power_status pour récupérer :
   * - power_bqtt_voltage
   * - logical_batt_voltage
   * - emergency_button_pressed
   */
  useEffect(() => {
    // Création du topic de souscription
    const powerStatusTopic = new ROSLIB.Topic({
      ros,
      name: "/power_status",
      messageType: "msg_utils/PowerStatus",
    });

    // Callback exécuté à la réception de chaque message
    powerStatusTopic.subscribe((msg) => {
      setPowerVoltage(msg.power_bqtt_voltage);
      setLogicalVoltage(msg.logical_batt_voltage);
      setStatusEmergency(msg.emergency_button_pressed);
      // Ne pas toucher à buttonPressed ici : bouton et état robot séparés
    });

    // Nettoyage de la souscription lorsque le composant est démonté
    return () => {
      powerStatusTopic.unsubscribe();
    };
  }, []);

  /**
   * Convertit une tension en pourcentage de batterie
   * @param {number} v - tension mesurée
   * @param {number} min - tension minimale (0% batterie)
   * @param {number} max - tension maximale (100% batterie)
   * @returns {number} pourcentage (0 à 100)
   */
  const voltageToPercent = (v, min = 3.0, max = 4.2) =>
    Math.min(100, Math.max(0, ((v - min) / (max - min)) * 100));

  /**
   * Gestionnaire du clic sur le bouton d'arrêt d'urgence
   * - Inverse l'état buttonPressed
   * - Publie une seule fois la nouvelle valeur sur /emergency_stop_cmd
   */
  const handleEmergencyToggle = () => {
    const next = !buttonPressed;
    emergencyTopic.publish(new ROSLIB.Message({ data: next }));
    setButtonPressed(next);
  };

  return (
    <div>
      {/* Titre général */}
      <h2>Battery Status</h2>

      {/* Affichage des deux jauges de batterie côte à côte */}
      <div style={{ display: "flex", justifyContent: "space-around" }}>
        {/* Jauge pour la batterie Power */}
        <div>
          <h3>Power Battery</h3>
          <BatteryGauge
            value={voltageToPercent(powerVoltage)}
            charging={false}        // pas de charge détectée ici
            size={150}
            customization={{
              batteryMeter: {
                fill: powerVoltage > 3.7 ? "green" : "red",
                lowBatteryValue: 20,
                lowBatteryFill: "red",
              },
              readingText: {
                fontSize: 14,
                showPercentage: true,
              },
            }}
          />
          {/* Affichage de la valeur numérique de la tension */}
          <p>Voltage: {powerVoltage.toFixed(2)} V</p>
        </div>

        {/* Jauge pour la batterie Logical */}
        <div>
          <h3>Logical Battery</h3>
          <BatteryGauge
            value={voltageToPercent(logicalVoltage)}
            charging={false}
            size={150}
            customization={{
              batteryMeter: {
                fill: logicalVoltage > 3.7 ? "green" : "red",
                lowBatteryValue: 20,
                lowBatteryFill: "red",
              },
              readingText: {
                fontSize: 14,
                showPercentage: true,
              },
            }}
          />
          <p>Voltage: {logicalVoltage.toFixed(2)} V</p>
        </div>
      </div>

      {/* Section du bouton d'urgence et indicateur d'état robot */}
      <div style={{ marginTop: 20, display: "flex", alignItems: "center" }}>
        {/* Bouton qui pilote buttonPressed */}
        <button onClick={handleEmergencyToggle}>
          {buttonPressed ? "EMERGENCY STOP" : "RUN MODE"}
        </button>

        {/* Indicateur visuel de l'état d'urgence du robot */}
        <span
          style={{
            display: "inline-block",
            marginLeft: 10,
            width: 16,
            height: 16,
            borderRadius: "50%",
            backgroundColor: statusEmergency ? "red" : "green",
          }}
          title={
            statusEmergency
              ? "Bot is in EMERGENCY STOP"
              : "Bot is in RUN MODE"
          }
        />
      </div>
    </div>
  );
};

export default PowerStatusDisplay;
