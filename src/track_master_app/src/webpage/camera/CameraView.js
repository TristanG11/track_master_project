import React, { useState, useEffect } from "react";
import ROSLIB from "roslib";
import ros from "../common/ROSConnection";
import styles from './CameraView.module.css';

const CameraView = () => {
  const [imageSrc, setImageSrc] = useState(null);

  useEffect(() => {
    const cameraTopic = new ROSLIB.Topic({
      ros: ros,
      name: "/camera/image_raw/compressed",
      messageType: "sensor_msgs/CompressedImage",
    });

    const handleImageMessage = (message) => {
      let base64String;

      if (typeof message.data === "string") {
        // message.data est déjà du base64
        base64String = message.data;
      } else if (message.data instanceof Array) {
        // message.data est un array d'octets → convertir en base64
        const binaryStr = String.fromCharCode(...message.data);
        base64String = btoa(binaryStr);
      } else if (message.data instanceof Uint8Array) {
        // message.data est un Uint8Array
        let binaryStr = "";
        for (let i = 0; i < message.data.length; i++) {
          binaryStr += String.fromCharCode(message.data[i]);
        }
        base64String = btoa(binaryStr);
      } else {
        console.error("Type de message.data inconnu :", typeof message.data);
        return;
      }

      setImageSrc("data:image/jpeg;base64," + base64String);
    };

    cameraTopic.subscribe(handleImageMessage);

    return () => {
      cameraTopic.unsubscribe();
    };
  }, []);

  return (
    <div className={styles.cameraContainer}>
      <h2>Camera View</h2>
      {imageSrc ? (
        <img src={imageSrc} alt="Camera Feed" className={styles.cameraImage} />
      ) : (
        <p>Loading camera feed...</p>
      )}
    </div>
  );
};

export default CameraView;
