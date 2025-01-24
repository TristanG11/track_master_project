import React, { useState, useEffect } from "react";
import { MapContainer, TileLayer, Marker, Popup } from "react-leaflet";
import "leaflet/dist/leaflet.css";
import ROSLIB from "roslib";
import ros from "../common/ROSConnection"; // Assure-toi que ros est bien configuré.
import L from "leaflet"

const GNSSDisplay = () => {
  // État pour la position GNSS
  const [gnssPosition, setGnssPosition] = useState({
    lat: 45.8566, // Coordonnées initiales (exemple : Paris)
    lng: 5.3522,
  });

  const customIcon = new L.Icon({
    iconUrl: "https://leafletjs.com/examples/custom-icons/leaf-green.png", // URL de l'image
    iconSize: [38, 38], // Taille de l'icône
    iconAnchor: [22, 38], // Point d'ancrage
    popupAnchor: [0, -40], // Position du popup
  });

  useEffect(() => {
    // Souscription au topic GNSS
    const gnssTopic = new ROSLIB.Topic({
      ros: ros,
      name: "/gnss/fix", // Assure-toi que le topic est correct
      messageType: "sensor_msgs/NavSatFix", // Message standard GNSS
    });

    // Callback pour mettre à jour la position
    gnssTopic.subscribe((message) => {
      setGnssPosition({
        lat: message.latitude,
        lng: message.longitude,
      });
      console.log()
    });

    // Nettoyage lors du démontage du composant
    return () => {
      gnssTopic.unsubscribe();
    };
  }, []);

  return (
    <MapContainer
      center={[gnssPosition.lat, gnssPosition.lng]} // Centre de la carte sur la position GNSS
      zoom={13}
      style={{ height: "400px", width: "100%" }}
      icon={customIcon}
    >
      
      {/* Fond de carte OpenStreetMap */}
      <TileLayer
        url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
        attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
      />
      {/* Marqueur pour afficher la position GNSS */}
      <Marker position={[gnssPosition.lat, gnssPosition.lng]}>
        <Popup>
          GNSS Position<br />
          Latitude: {gnssPosition.lat.toFixed(10)} <br />
          Longitude: {gnssPosition.lng.toFixed(10)}
        </Popup>
      </Marker>
    </MapContainer>
  );
};

export default GNSSDisplay;
