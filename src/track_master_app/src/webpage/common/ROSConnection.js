import ROSLIB from "roslib"


const ros = new ROSLIB.Ros({
    url: "ws://localhost:9090",
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

