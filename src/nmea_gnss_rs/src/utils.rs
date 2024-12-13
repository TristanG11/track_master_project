
#[derive(Debug)]
struct GGAData {
    time_utc: Option<String>,
    latitude: Option<f64>,
    longitude: Option<f64>,
    fix_quality: u8,
    num_satellites: u8,
    horizontal_dilution: Option<f64>,
    altitude: Option<f64>,
    altitude_units: Option<String>,
    geoidal_separation: Option<f64>,
    geoidal_units: Option<String>,
    differential_age: Option<f64>,
    differential_station: Option<u16>,
}

