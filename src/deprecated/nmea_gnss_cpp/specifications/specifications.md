Voici une **spécification fonctionnelle** tirée du code donné :

---

### **Contexte et objectif**
Le nœud `nmea_gnss_node` est un nœud ROS2 en Python qui lit des données GNSS via une liaison série, les décode pour extraire des informations de localisation et de vitesse, puis publie ces données sur des topics ROS2.

---

### **Spécifications fonctionnelles**

1. **Description des fonctionnalités**
   - **Lecture des données GNSS** :
     - Le nœud se connecte à un module GNSS via un port série configuré.
     - Les données sont lues en continu à intervalle régulier (toutes les 0.1 secondes).
   - **Décodage des messages NMEA** :
     - Les messages GNSS de type `GGA` (position) et `VTG` (cap/vitesse) sont analysés.
     - Les données extraites incluent :
       - Latitude, longitude, altitude.
       - Qualité du signal GNSS.
       - Cap vrai (heading) et vitesse en m/s.
   - **Publication des données** :
     - Les données GNSS sont publiées sur les topics suivants :
       - `/gnss/fix` (type `sensor_msgs/NavSatFix`).
       - `/gnss/heading_vel` (type `GpsVelocityHeading`).
   - **Gestion des erreurs** :
     - Le nœud loggue les erreurs liées à la lecture du port série ou au décodage des messages GNSS.

2. **Interfaces**
   - **Topics publiés** :
     - `/gnss/fix` :
       - Message : `sensor_msgs/NavSatFix`.
       - Données : latitude, longitude, altitude, qualité du signal.
     - `/gnss/heading_vel` :
       - Message : `GpsVelocityHeading`.
       - Données : cap vrai (heading), vitesse en m/s, validité des données.
   - **Paramètres configurables** :
     - `gnss/module_port` (par défaut : `/dev/ttyUSB0`) : chemin du port série du module GNSS.
     - `gnss/baudrate` (par défaut : `38400`) : débit en bauds pour la communication série.

3. **Scénarios d’utilisation**
   - **Cas nominal** :
     - Le module GNSS est correctement connecté et envoie des données valides.
     - Les données de localisation et de vitesse sont publiées sur les topics ROS2.
   - **Cas d’erreur** :
     - Si la communication série échoue, un message d’erreur est loggué.
     - Si les messages NMEA sont invalides ou incomplets, le système les ignore.

4. **Contraintes**
   - Fonctionne avec des modules GNSS compatibles NMEA (messages `GGA` et `VTG`).
   - Nécessite une connexion série fonctionnelle avec un débit configurable.

5. **Critères de validation**
   - Les messages `GGA` valides sont publiés sur `/gnss/fix` avec les données de position correctes.
   - Les messages `VTG` valides sont publiés sur `/gnss/heading_vel` avec les données de cap et vitesse correctes.
   - Les erreurs de communication ou de décodage sont correctement logguées.
