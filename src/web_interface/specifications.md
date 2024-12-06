# **Web Interface Specifications for Track Master Robot**

## **1. Introduction**
The web interface is designed to provide an intuitive way to monitor and control the Track Master robot. This interface will offer real-time visualization of the robot's status, diagnostics, and control functionalities. The system will run on the robot's onboard computer and be accessible via a web browser.

---

## **2. Functionalities**

### **2.1 Visualization**
- **Robot Representation:**
  - A square graphic will represent the robot's chassis, including its wheels.
  - A vector will indicate the robot's speed and heading (derived from odometry).
  - Display the GPS coordinates of the robot in real-time.

- **Camera Feed:**
  - Display a live feed from the robot's onboard camera.

### **2.2 Control**
- **Joystick:**
  - A virtual joystick to send velocity commands to the robot by publishing messages on the `/cmd/Vel` topic.
  - Controls:
    - Linear velocity (forward/backward).
    - Angular velocity (rotation).

### **2.3 Diagnostics**
- **Error Flags:**
  - Display a list of errors or issues detected by the robot with appropriate flags.
  - Include timestamps for when each error occurred.

- **Wi-Fi Data Rate:**
  - Show the current Wi-Fi throughput in Mbps or Kbps.

- **Battery Status:**
  - Display the battery level as a percentage.
  - Include a visual indicator (e.g., color-coded bar: green for high, red for low).

---

## **3. Interface Layout**
### **3.1 Main Screen**
1. **Robot Visualization Panel:**
   - A top-view representation of the robot with:
     - A square for the chassis.
     - Rectangles or circles for the wheels.
     - A vector arrow for speed and direction.

2. **Joystick Control Panel:**
   - Positioned below or to the side of the visualization panel.
   - Real-time publishing to `/cmd/Vel` based on joystick input.

3. **GPS Information:**
   - Display GPS coordinates (latitude and longitude) in real-time.

4. **Camera Feed:**
   - A video stream window for the robot's camera.

### **3.2 Diagnostics Menu**
A dropdown or collapsible menu providing:
- **Error List:**
  - Display a list of errors with their timestamps and error flags.
- **Wi-Fi Throughput:**
  - Show current data rates.
- **Battery Level:**
  - A battery status bar with percentage.
- **Additional Diagnostics:**
  - Placeholder for future diagnostic data.

---

## **4. Technical Requirements**

### **4.1 Frontend**
- **Framework:** React.js or Vue.js for a responsive and interactive user interface.
- **Visualization Library:** Use D3.js or Canvas API to draw the robot and vectors dynamically.
- **Video Streaming:** Integration with WebRTC or MJPEG for the live camera feed.
- **Joystick Implementation:** Use an existing joystick library (e.g., `nipple.js`) for intuitive control.

### **4.2 Backend**
- **Language:** Python (FastAPI or Flask).
- **ROS2 Integration:**
  - Use `rclpy` or `rosbridge` to communicate with the robot's topics.
  - Subscribe to:
    - `/odom` for odometry data (speed and vector).
    - `/gps/fix` for GPS data.
    - `/battery_state` for battery status.
    - `/diagnostics` for robot diagnostics.
  - Publish to `/cmd/Vel` for joystick commands.

### **4.3 Hosting**
- **Onboard Hosting:**
  - The webpage will be served from the robot's onboard computer (e.g., Raspberry Pi).
  - Accessible over the robot's Wi-Fi or mobile hotspot.

---

## **5. Constraints**
1. **Hardware Limitations:**
   - Limited processing power and memory on the Raspberry Pi.
   - Optimize for lightweight frameworks and libraries.
2. **Network:**
   - Ensure low latency for joystick commands over Wi-Fi.
   - Handle potential bandwidth limitations for the camera feed.

---

## **6. Future Evolution**
- Add logging for all sent joystick commands and received diagnostic data.
- Enhance error reporting with visual alerts for critical issues.
- Integrate map-based visualization for the robot's real-time GPS path.
- Support multiple robots in a unified dashboard.

---

## **7. Glossary**
- **Odometry:** Data used to estimate the robot's position and velocity based on its wheel encoders.
- **Joystick:** A virtual control element used for sending velocity commands to the robot.
- **Diagnostics:** Reports on the robot's internal state and detected errors.
