import ROSLIB from "roslib"

const urlBrigde =process.env.REACT_APP_ROSBRIDGE_URL;
const ros = new ROSLIB.Ros({
    url: urlBrigde,
});


ros.on("connection", () => {
    console.log("Connected to rosbridge");
});

ros.on("error", (error) => {
    console.error("Error connecting to rosbridge:", error);
});

ros.on("close", () => {
    console.log("Connection to rosbridge closed");
});

export default ros;

