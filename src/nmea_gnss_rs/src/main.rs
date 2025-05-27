use diagnostic_msgs::msg::{DiagnosticArray, DiagnosticStatus, KeyValue};
use geometry_msgs::msg::{PoseWithCovarianceStamped, TwistWithCovarianceStamped};
use map_3d::{geodetic2enu, Ellipsoid::WGS84};
use msg_utils::msg::GpsVelocityHeading;
use msg_utils::msg::SerialPorts;
use nmea_parser::{gnss::GgaQualityIndicator, NmeaParser, ParsedMessage};
use rclrs::MandatoryParameter;
use rust_utils::serial::*;
use sensor_msgs::msg::NavSatFix;
use std::sync::atomic::AtomicBool;
use std::sync::Arc;
use std::{
    io::{BufRead, BufReader},
    time::Duration,
};
use parking_lot::FairMutex;
fn main() {
    let context = rclrs::Context::new(std::env::args()).unwrap();
    let node = rclrs::create_node(&context, "nmea_gnss_node_rs").unwrap();
    let diag_name: &'static str = "nmea_gnss_node_rs";

    let baud_rate: MandatoryParameter<i64> = node
        .declare_parameter("baud_rate")
        .default(38400)
        .mandatory()
        .unwrap();

    let lat0 = node
        .declare_parameter("lat0")
        .default(45.1884999)
        .mandatory()
        .unwrap();

    let lon0 = node
        .declare_parameter("lon0")
        .default(5.7588211)
        .mandatory()
        .unwrap();

    let alt0 = node
        .declare_parameter("alt0")
        .default(0.0)
        .mandatory()
        .unwrap();

    let gnss_fix_topic: MandatoryParameter<Arc<str>> = node
        .declare_parameter("gnss_fix_topic")
        .default(Arc::from("/gnss/fix"))
        .mandatory()
        .unwrap();

    let heading_vel_topic: MandatoryParameter<Arc<str>> = node
        .declare_parameter("heading_vel_topic")
        .default(Arc::from("/gnss/heading_vel"))
        .mandatory()
        .unwrap();

    let twist_topic: MandatoryParameter<Arc<str>> = node
        .declare_parameter("twist_topic")
        .default(Arc::from("/gnss/twist"))
        .mandatory()
        .unwrap();

    let pose_topic: MandatoryParameter<Arc<str>> = node
        .declare_parameter("pose_topic")
        .default(Arc::from("/gnss/pose"))
        .mandatory()
        .unwrap();

    let gnss_fix_pub = node
        .create_publisher::<NavSatFix>(&gnss_fix_topic.get(), rclrs::QoSProfile::default())
        .unwrap();
    let heading_vel_pub = node
        .create_publisher::<GpsVelocityHeading>(
            &heading_vel_topic.get(),
            rclrs::QoSProfile::default(),
        )
        .unwrap();

    let twist_pub = node
        .create_publisher::<TwistWithCovarianceStamped>(
            &twist_topic.get(),
            rclrs::QoSProfile::default(),
        )
        .unwrap();

    let pose_pub = node
        .create_publisher::<PoseWithCovarianceStamped>(
            &pose_topic.get(),
            rclrs::QoSProfile::default(),
        )
        .unwrap();

    let diag_publisher = node
        .create_publisher::<DiagnosticArray>("/diagnostics", rclrs::QoSProfile::default())
        .unwrap();

    let mut pose_msg = PoseWithCovarianceStamped::default();

    let disconnected_flag = Arc::new(AtomicBool::new(true));
    //Diagnostics part :
    let (diag_tx, diag_rx) = std::sync::mpsc::channel::<DiagnosticStatus>();
    // Thread to process diagnostic messages
    {
        let node = node.clone();
        let mut diag_array = DiagnosticArray::default();
        std::thread::spawn(move || loop {
            while let Ok(diag) = diag_rx.try_recv() {
                diag_array.status.push(diag.clone());
            }
            let now = node.get_clock().now().to_ros_msg().unwrap();
            diag_array.header.stamp.sec = now.sec;
            diag_array.header.stamp.nanosec = now.nanosec;
            if !diag_array.status.is_empty() {
                diag_publisher.publish(diag_array.clone()).unwrap();
            }
            diag_array.status.clear();
            std::thread::sleep(Duration::from_millis(1000));
        });
    };

    let serial_port: Arc<parking_lot::lock_api::Mutex<parking_lot::RawFairMutex, Option<Box<dyn serialport::SerialPort + 'static>>>>= Arc::new(parking_lot::FairMutex::new(None));

    let port_name: Arc<parking_lot::lock_api::Mutex<parking_lot::RawFairMutex, String>> = Arc::new(FairMutex::new(String::new()));

    let _subscription = node
        .create_subscription::<SerialPorts, _>(
            "/available_serial_ports",
            rclrs::QoSProfile::default(),
            {
                let mut last_name = String::new();
                let port_name = Arc::clone(&port_name);
                move |msg: SerialPorts| {
                    if last_name != msg.gnss_port {
                        if let Some(mut name) = port_name.try_lock_for(Duration::from_millis(100)) {
                            *name = msg.gnss_port.clone();
                            last_name = msg.gnss_port.clone();
                        }
                    }
                }
            },
        )
        .unwrap();

    spawn_reconnection_thread(
        serial_port.clone(),
        disconnected_flag.clone(),
        port_name.clone(),
        diag_tx.clone(),
        baud_rate.get(),
        diag_name.to_string(),
    );

    std::thread::spawn({
        let lat0 = lat0.get();
        let alt0 = alt0.get();
        let lon0 = lon0.get();

        let node = node.clone();
        let mut parser = NmeaParser::new();
        let serial_port: Arc<parking_lot::lock_api::Mutex<parking_lot::RawFairMutex, Option<Box<dyn serialport::SerialPort + 'static>>>>= serial_port.clone();
        move || {
            let disconnected_flag = disconnected_flag.clone();
            let diag_tx = diag_tx.clone();
            loop {
                let mut line = String::new();
                let mut serial_error: Option<std::io::Error> = None;

                if let Some(mut port_lock) = serial_port.try_lock_for(Duration::from_millis(10)) {
                    if let Some(ref mut port) = *port_lock {
                        println!("will cre");
                        let mut reader = BufReader::new(port);
                        match reader.read_line(&mut line) {
                            Ok(n) if n > 0 => {
                                match parser.parse_sentence(&line) {
                                    Ok(result) => {
                                        match result {
                                            ParsedMessage::Gga(gga) => {
                                                let mut fix_msg: NavSatFix = NavSatFix::default();

                                                fix_msg.header.stamp.sec = node
                                                    .get_clock()
                                                    .now()
                                                    .to_ros_msg()
                                                    .unwrap()
                                                    .sec;
                                                fix_msg.header.stamp.nanosec = node
                                                    .get_clock()
                                                    .now()
                                                    .to_ros_msg()
                                                    .unwrap()
                                                    .nanosec;
                                                // Convert latitude and longitude
                                                fix_msg.latitude = gga.latitude.unwrap_or(0.0);
                                                fix_msg.longitude = gga.longitude.unwrap_or(0.0);

                                                // Convert altitude
                                                fix_msg.altitude = gga.altitude.unwrap_or(0.0);

                                                // Fill the status
                                                fix_msg.status.status = match gga.quality {
                                                    GgaQualityIndicator::Invalid => -1, // STATUS_NO_FIX
                                                    GgaQualityIndicator::GpsFix => 0, // STATUS_FIX
                                                    GgaQualityIndicator::DGpsFix => 2, // STATUS_DGPS_FIX
                                                    _ => -1, // Default to no fix
                                                };

                                                // Satellite count as additional diagnostic info
                                                fix_msg.status.service = 1; // SERVICE_GPS

                                                // Fill position covariance
                                                fix_msg.position_covariance = [
                                                    gga.hdop.unwrap_or(1.0),
                                                    0.0,
                                                    0.0,
                                                    0.0,
                                                    gga.hdop.unwrap_or(1.0),
                                                    0.0,
                                                    0.0,
                                                    0.0,
                                                    gga.hdop.unwrap_or(1.0),
                                                ];
                                                fix_msg.position_covariance_type =
                                                    sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

                                                println!(
                                                    "Publishing NavSatFix: latitude={}, longitude={}, altitude={}",
                                                    fix_msg.latitude, fix_msg.longitude, fix_msg.altitude
                                                );

                                                /*if is_first_pose_fetched {
                                                    // convertir en coordonnées enu la position actuelle
                                                }*/

                                                // convert to enu

                                                let (x, y, z) = geodetic2enu(
                                                    fix_msg.latitude.to_radians(),
                                                    fix_msg.longitude.to_radians(),
                                                    fix_msg.altitude,
                                                    lat0,
                                                    lon0,
                                                    alt0,
                                                    WGS84,
                                                );

                                                pose_msg.header.frame_id =
                                                    std::string::String::from("map");
                                                pose_msg.pose.pose.position.x = x;
                                                pose_msg.pose.pose.position.y = y;
                                                pose_msg.pose.pose.position.z = z;
                                                // Publish the message
                                                gnss_fix_pub.publish(fix_msg).unwrap();
                                            }
                                            ParsedMessage::Vtg(vtg) => {
                                                let mut heading_vel_msg =
                                                    GpsVelocityHeading::default();
                                                let mut twist_msg =
                                                    TwistWithCovarianceStamped::default();
                                                if vtg.cog_true.is_some()
                                                    && vtg.sog_kph.is_some()
                                                    && vtg.cog_magnetic.is_some()
                                                {
                                                    heading_vel_msg.valid = true;
                                                    heading_vel_msg.heading = vtg.cog_true.unwrap(); // Heading en degrés
                                                    heading_vel_msg.velocity =
                                                        vtg.sog_kph.unwrap() / 3.6;
                                                // Conversion de km/h en m/s
                                                } else {
                                                    heading_vel_msg.valid = false;
                                                    heading_vel_msg.heading =
                                                        vtg.cog_true.unwrap_or(0.0); // Si None, utilise 0.0 par défaut
                                                    heading_vel_msg.velocity =
                                                        vtg.sog_kph.unwrap_or(0.0) / 3.6;
                                                }

                                                twist_msg.header.frame_id =
                                                    std::string::String::from("map");
                                                twist_msg.header.stamp.nanosec = node
                                                    .get_clock()
                                                    .now()
                                                    .to_ros_msg()
                                                    .unwrap()
                                                    .nanosec;
                                                twist_msg.header.stamp.sec = node
                                                    .get_clock()
                                                    .now()
                                                    .to_ros_msg()
                                                    .unwrap()
                                                    .sec;

                                                twist_msg.twist.twist.linear.x =
                                                    heading_vel_msg.velocity;

                                                // orientation
                                                pose_msg.header.stamp.sec =
                                                    twist_msg.header.stamp.sec;
                                                pose_msg.header.stamp.nanosec =
                                                    twist_msg.header.stamp.nanosec;

                                                heading_vel_pub.publish(heading_vel_msg).unwrap();
                                                twist_pub.publish(twist_msg).unwrap();
                                                pose_pub.publish(&pose_msg).unwrap();
                                            }
                                            _ => {
                                                println!(
                                                    "Unrecognized NMEA sentence : {:?}",
                                                    result
                                                );
                                            }
                                        }
                                    }
                                    Err(e) => {
                                        eprintln!("Error while parsing NMEA sentence: {}", e);
                                    }
                                }
                            }
                            Ok(_) => {
                                println!("No data, waiting...");
                            }
                            Err(e) => {
                                //
                                eprintln!("Read error: {}", e);
                                serial_error = Some(e);
                                //break;
                            }
                        }
                    }
                }
                if let Some(e) = serial_error {
                    handle_serial_error(e.into(), &diag_tx, diag_name, &disconnected_flag);
                }
            }
        }
    });

    rclrs::spin(node).unwrap();
}
