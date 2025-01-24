import React, { useState, useEffect } from "react";
import ROSLIB from "roslib";
import ros from "../common/ROSConnection";
import BatteryGauge from "react-battery-gauge";

const BatteryStatus = () => {
  const [batteryStatus, setBatteryStatus] = useState({
    voltage: 0,
    current: 0,
    charge_level: 0,
    charging: false,
  });

  useEffect(() => {
    // Souscription au topic ROS pour la batterie
    const batteryTopic = new ROSLIB.Topic({
      ros: ros,
      name: "/battery_status",
      messageType: "msg_utils/BatteryStatus",
    });

    // Mise à jour de l'état local avec les données reçues
    batteryTopic.subscribe((message) => {
      setBatteryStatus({
        voltage: message.voltage,
        current: message.current,
        charge_level: message.charge_level,
        charging: message.charging,
      });
    });

    // Nettoyage de la souscription lors du démontage du composant
    return () => {
      batteryTopic.unsubscribe();
    };
  }, []);

  return (
    <div>
      <h2>Battery Status</h2>
      {/* Affichage de la jauge de batterie */}
      <BatteryGauge
        value={batteryStatus.charge_level} // Niveau de charge (pourcentage)
        charging={batteryStatus.charging} // Indicateur de charge
        size={200} // Taille de la jauge
        customization={{
          batteryMeter: {
            fill: batteryStatus.charge_level > 20 ? "green" : "red", // Couleur verte si > 20%, rouge sinon
            lowBatteryValue: 20, // Seuil de batterie faible
            lowBatteryFill: "red",
          },
          readingText: {
            fontSize: 16, // Taille de la police
            showPercentage: true, // Affiche le pourcentage
            lowBatteryColor: "red", // Couleur du texte si batterie faible
          },
        }}
      />
      {/* Informations supplémentaires */}
      <p>Voltage: {batteryStatus.voltage} V</p>
      <p>Current: {batteryStatus.current} A</p>
      <p>
        Charging: {batteryStatus.charging ? "Yes" : "No"}
      </p>
    </div>
  );
};

export default BatteryStatus;
