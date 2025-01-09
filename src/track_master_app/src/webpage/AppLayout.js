import React from "react";
import JoystickControl from "./joystick/JoystickControl";


const AppLayout = () => {
    return (
      <div>
        <h1>Robot Interface</h1>
        <JoystickControl />
      </div>
    );
  };
  
  export default AppLayout;