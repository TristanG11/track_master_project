import React, { useState, useEffect } from "react";
import { MapContainer, TileLayer, Marker, Popup } from "react-leaflet";
import "leaflet/dist/leaflet.css";
import ROSLIB from "roslib";
import ros from "../common/ROSConnection"; // Make sure ROS is properly configured.
import L from "leaflet";

const GNSSDisplay = () => {
  // State for GNSS position
  const [gnssPosition, setGnssPosition] = useState({
    lat: 45.8566, // Initial coordinates (example: Paris)
    lng: 5.3522,
  });

  const customIcon = new L.Icon({
    iconUrl: "https://leafletjs.com/examples/custom-icons/leaf-green.png", // URL of the icon image
    iconSize: [38, 38], // Icon size
    iconAnchor: [22, 38], // Anchor point
    popupAnchor: [0, -40], // Popup position
  });

  useEffect(() => {
    // Subscribe to the GNSS topic
    const gnssTopic = new ROSLIB.Topic({
      ros: ros,
      name: "/gnss/fix", // Make sure the topic is correct
      messageType: "sensor_msgs/NavSatFix", // Standard GNSS message
    });

    // Callback to update the position
    gnssTopic.subscribe((message) => {
      setGnssPosition({
        lat: message.latitude,
        lng: message.longitude,
      });
    });

    // Cleanup when the component unmounts
    return () => {
      gnssTopic.unsubscribe();
    };
  }, []);

  return (
    <MapContainer
      center={[gnssPosition.lat, gnssPosition.lng]} // Center the map on the GNSS position
      zoom={13}
      style={{ height: "400px", width: "100%" }}
      icon={customIcon}
    >
      {/* OpenStreetMap background */}
      <TileLayer
        url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
        attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
      />
      {/* Marker to display the GNSS position */}
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
