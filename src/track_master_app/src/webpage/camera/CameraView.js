import React, { useState, useEffect } from "react";
import ROSLIB from "roslib";
import ros from "../common/ROSConnection";

const CameraView = () => {
  const [imageSrc, setImageSrc] = useState(null);

  useEffect(() => {
    // Souscription au topic de la caméra
    const cameraTopic = new ROSLIB.Topic({
      ros: ros,
      name: "/camera/image_raw", // Nom du topic
      messageType: "sensor_msgs/Image",
    });

    const handleImageMessage = (message) => {
        const { height, width, encoding } = message;
      
        // Convertir la chaîne en tableau d'octets
        const dataArray = new Uint8Array(message.data.length);
        for (let i = 0; i < message.data.length; i++) {
          dataArray[i] = message.data.charCodeAt(i);
        }
      
        console.log("Converted data length:", dataArray.length);
      
        const canvas = document.createElement("canvas");
        const context = canvas.getContext("2d");
        canvas.width = width;
        canvas.height = height;
      
        const imageData = context.createImageData(width, height);
      
        // Gestion des données selon l'encodage (par ex., rgb8)
        if (encoding === "rgb8") {
          for (let i = 0; i < dataArray.length; i += 3) {
            const pixelIndex = i / 3;
            imageData.data[pixelIndex * 4] = dataArray[i]; // Rouge
            imageData.data[pixelIndex * 4 + 1] = dataArray[i + 1]; // Vert
            imageData.data[pixelIndex * 4 + 2] = dataArray[i + 2]; // Bleu
            imageData.data[pixelIndex * 4 + 3] = 255; // Alpha (opaque)
          }
        } else if (encoding === "bgr8") {
          for (let i = 0; i < dataArray.length; i += 3) {
            const pixelIndex = i / 3;
            imageData.data[pixelIndex * 4] = dataArray[i + 2]; // Rouge
            imageData.data[pixelIndex * 4 + 1] = dataArray[i + 1]; // Vert
            imageData.data[pixelIndex * 4 + 2] = dataArray[i]; // Bleu
            imageData.data[pixelIndex * 4 + 3] = 255; // Alpha (opaque)
          }
        } else {
          console.error("Unsupported encoding:", encoding);
          return;
        }
        console.log("Encoding:", encoding);
        console.log("Data length (expected):", width * height * 3); // Pour rgb8 ou bgr8
        console.log("Data length (received):", dataArray.length);
        console.log("First 10 bytes:", dataArray.slice(0, 10)); // Vérifiez les premières données

      
        // Mettre les données sur le canvas et générer l'image
        context.putImageData(imageData, 0, 0);
        setImageSrc(canvas.toDataURL());
      };
      

    cameraTopic.subscribe(handleImageMessage);

    return () => {
      cameraTopic.unsubscribe();
    };
  }, []);

  return (
    <div>
      <h2>Camera View</h2>
      {imageSrc ? (
        <img
          src={imageSrc}
          alt="Camera Feed"
          style={{ maxWidth: "100%", border: "2px solid white" }}
        />
      ) : (
        <p>Loading camera feed...</p>
      )}
    </div>
  );
};

export default CameraView;
