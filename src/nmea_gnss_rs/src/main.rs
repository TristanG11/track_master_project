use std::io::{BufRead, BufReader};
use sensor_msgs::msg::NavSatFix;
use rclrs::MandatoryParameter;
use serialport;
use nmea_parser::{NmeaParser, ParsedMessage, gnss::GgaQualityIndicator};
use msg_utils::msg::GpsVelocityHeading;
use geometry_msgs::msg::{PoseWithCovarianceStamped, TwistWithCovarianceStamped};
use map_3d::{Ellipsoid::WGS84,geodetic2enu};
use std::sync::Arc;

fn main() {

    let context = rclrs::Context::new(std::env::args()).unwrap();
    let node = rclrs::create_node(&context, "nmea_gnss_node_rs").unwrap();

    let serial_port:  MandatoryParameter<Arc<str>> =
        node.declare_parameter("serial_port").default(Arc::from("/dev/ttyUSB0")).mandatory().unwrap();

    let baud_rate: MandatoryParameter<i64> =
        node.declare_parameter("baud_rate").default(38400).mandatory().unwrap();

    let lat0 =
        node.declare_parameter("lat0").default(45.1884999).mandatory().unwrap();

    let lon0=
        node.declare_parameter("lon0").default(5.7588211).mandatory().unwrap();

    let alt0 =
        node.declare_parameter("alt0").default(0.0).mandatory().unwrap();

    let gnss_fix_topic: MandatoryParameter<Arc<str>> =
        node.declare_parameter("gnss_fix_topic").default(Arc::from("/gnss/fix")).mandatory().unwrap();

    let heading_vel_topic: MandatoryParameter<Arc<str>>  =
        node.declare_parameter("heading_vel_topic").default(Arc::from("/gnss/heading_vel")).mandatory().unwrap();

    let twist_topic: MandatoryParameter<Arc<str>>  =
        node.declare_parameter("twist_topic").default(Arc::from("/gnss/twist")).mandatory().unwrap();

    let pose_topic: MandatoryParameter<Arc<str>> =
        node.declare_parameter("pose_topic").default(Arc::from("/gnss/pose")).mandatory().unwrap();

    let gnss_fix_pub = node
        .create_publisher::<NavSatFix>(&gnss_fix_topic.get(), rclrs::QoSProfile::default())
        .unwrap();
    let heading_vel_pub = node
        .create_publisher::<GpsVelocityHeading>(&heading_vel_topic.get(), rclrs::QoSProfile::default())
        .unwrap();

    let twist_pub = node
        .create_publisher::<TwistWithCovarianceStamped>(&twist_topic.get(), rclrs::QoSProfile::default())
        .unwrap();

    let pose_pub = node
        .create_publisher::<PoseWithCovarianceStamped>(&pose_topic.get(), rclrs::QoSProfile::default())
        .unwrap();

    let mut pose_msg = PoseWithCovarianceStamped::default();
    let port = serialport::new(& *serial_port.get(), baud_rate.get() as u32)
        .timeout(std::time::Duration::from_secs(2))
        .open()
        .expect("Failed to open serial port");

        let (lat0, lon0, alt0) = (lat0.get().to_radians(), lon0.get().to_radians(), alt0.get());
        println!(
            "Anchor coordinates: lat0={}, lon0={}, alt0={}",
            lat0, lon0, alt0
        );
        
    let mut parser = NmeaParser::new();
    let mut reader = BufReader::new(port);

    loop {
        let mut line = String::new();
        match reader.read_line(&mut line) {
            Ok(n) if n > 0 => {
                match parser.parse_sentence(&line) {
                    Ok(result) => {
                        match result {
                            ParsedMessage::Gga(gga) => {
                                let mut fix_msg: NavSatFix = NavSatFix::default();

                                fix_msg.header.stamp.sec = node.get_clock().now().to_ros_msg().unwrap().sec;
                                fix_msg.header.stamp.nanosec = node.get_clock().now().to_ros_msg().unwrap().nanosec;
                                // Convert latitude and longitude
                                fix_msg.latitude = gga.latitude.unwrap_or(0.0);
                                fix_msg.longitude = gga.longitude.unwrap_or(0.0);

                                // Convert altitude
                                fix_msg.altitude = gga.altitude.unwrap_or(0.0);

                                // Fill the status
                                fix_msg.status.status = match gga.quality {
                                    GgaQualityIndicator::Invalid => -1, // STATUS_NO_FIX
                                    GgaQualityIndicator::GpsFix => 0,   // STATUS_FIX
                                    GgaQualityIndicator::DGpsFix => 2,  // STATUS_DGPS_FIX
                                    _ => -1,                // Default to no fix
                                };

                                // Satellite count as additional diagnostic info
                                fix_msg.status.service = 1; // SERVICE_GPS

                                // Fill position covariance
                                fix_msg.position_covariance = [
                                    gga.hdop.unwrap_or(1.0), 0.0, 0.0,
                                    0.0, gga.hdop.unwrap_or(1.0), 0.0,
                                    0.0, 0.0, gga.hdop.unwrap_or(1.0),
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

                                let (x,y,z) = geodetic2enu(fix_msg.latitude.to_radians(), fix_msg.longitude.to_radians(), fix_msg.altitude, lat0, lon0, alt0, WGS84);

                                pose_msg.header.frame_id = std::string::String::from("map");
                                pose_msg.pose.pose.position.x = x;
                                pose_msg.pose.pose.position.y = y;
                                pose_msg.pose.pose.position.z = z;
                                // Publish the message
                                gnss_fix_pub.publish(fix_msg).unwrap();
                            }
                            ParsedMessage::Vtg(vtg) => {
                                let mut heading_vel_msg = GpsVelocityHeading::default();
                                let mut twist_msg = TwistWithCovarianceStamped::default();
                                if vtg.cog_true.is_some() && vtg.sog_kph.is_some() && vtg.cog_magnetic.is_some() {
                                    heading_vel_msg.valid = true;
                                    heading_vel_msg.heading = vtg.cog_true.unwrap(); // Heading en degrés
                                    heading_vel_msg.velocity = vtg.sog_kph.unwrap() / 3.6; // Conversion de km/h en m/s
                                } else {
                                    heading_vel_msg.valid = false;
                                    heading_vel_msg.heading = vtg.cog_true.unwrap_or(0.0); // Si None, utilise 0.0 par défaut
                                    heading_vel_msg.velocity = vtg.sog_kph.unwrap_or(0.0) / 3.6;
                                }

                                twist_msg.header.frame_id = std::string::String::from("map");
                                twist_msg.header.stamp.nanosec = node.get_clock().now().to_ros_msg().unwrap().nanosec;
                                twist_msg.header.stamp.sec = node.get_clock().now().to_ros_msg().unwrap().sec;
                                
                                twist_msg.twist.twist.linear.x = heading_vel_msg.velocity;

                                // orientation 
                                pose_msg.header.stamp.sec = twist_msg.header.stamp.sec ;
                                pose_msg.header.stamp.nanosec = twist_msg.header.stamp.nanosec;

                                heading_vel_pub.publish(heading_vel_msg).unwrap();
                                twist_pub.publish(twist_msg).unwrap();
                                pose_pub.publish(&pose_msg).unwrap();
                            }
                            _ => {
                                println!("Unrecognized NMEA sentence : {:?}",result);
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
                eprintln!("Read error: {}", e);
                break;
            }
        }
    }

    println!("End of program.");
}


