def parse_gga(nmea_sentence):
    """
    Parse a NMEA GGA message and return a dictionary of the extracted data.
    """
    fields = nmea_sentence.strip().split(',')

    if len(fields) < 15:
        raise ValueError("Invalid GGA sentence: not enough fields")
    # Extract the data
    service = get_service_flags(fields[0])
    time_utc = fields[1] if fields[1] else None
    latitude = get_lat(fields[2], fields[3]) if fields[2] and fields[3] else None
    longitude = get_lon(fields[4], fields[5]) if fields[4] and fields[5] else None
    fix_quality = int(fields[6]) if fields[6].isdigit() else 0
    num_satellites = int(fields[7]) if fields[7].isdigit() else 0
    altitude = float(fields[9]) if fields[9] else 0.0
   

    return {
        'time_utc': time_utc,
        'latitude': latitude,
        'longitude': longitude,
        'fix_quality': fix_quality,
        'service': service,
        'num_satellites': num_satellites,
        'altitude': altitude,
    }

def parse_vtg(nmea_sentence):
    """
    Parse an NMEA VTG sentence and return a dictionary of the extracted data.
    """
    fields = nmea_sentence.strip().split(',')
    print(fields)
    
    # Validate sentence format
    if len(fields) < 8 :
        raise ValueError("Invalid VTG sentence: not enough fields")
    
    # Extract data with default handling for missing fields
    true_course = float(fields[1]) if fields[1] else None
    magnetic_course = float(fields[3]) if fields[3] else None
    speed_knots = float(fields[5]) if fields[5] else None
    speed_kmh = float(fields[7]) if fields[7] else None
    

    return {
        'true_course': true_course,         # Course relative to true north (degrees)
        'magnetic_course': magnetic_course, # Course relative to magnetic north (degrees)
        'speed_knots': speed_knots,         # Ground speed in knots
        'speed_ms': speed_kmh/3.6,             # Ground speed in kilometers per hour
    }

def parse_rmc(nmea_sentence):
    """
    Parse a NMEA RMC message and return a dictionary of the extracted data.
    """
    fields = nmea_sentence.strip().split(',')

    # Extract the data
    time_utc = fields[1]
    status = fields[2]  # A = valid, V = invalid
    latitude = get_lat(fields[3], fields[4])
    longitude = get_lon(fields[5], fields[6])
    print(fields[5])
    speed = fields[7]  # Speed in knots
    date = fields[9]  # Date in DDMMYY format

    return {
        'time_utc': time_utc,
        'status': status,
        'latitude': latitude,
        'longitude': longitude,
        'speed': float(speed) if speed else 0.0,
        'date': date,
    }

def map_gga_to_status(fix_quality):
    if fix_quality == 0:
        return -1  # STATUS_NO_FIX
    elif fix_quality == 1:
        return 0  # STATUS_FIX
    elif fix_quality in [2, 4, 5]:
        return 1  # STATUS_SBAS_FIX or better
    else:
        return -1  # Unknown or no fix

def get_service_flags(gnss_systems):
    service = 0
    if 'GP' in gnss_systems:
        service |= 1  # SERVICE_GPS
    if 'GN' in gnss_systems:
        service |= 2  # SERVICE_GLONASS
    if 'BD' in gnss_systems or 'COMPASS' in gnss_systems:
        service |= 4  # SERVICE_COMPASS
    if 'GL' in gnss_systems:
        service |= 8  # SERVICE_GALILEO
    return service


def get_lat(value, direction):
    """
    Convert a NMEA latitude value to decimal degrees.
    Example: 4807.038,N → 48.1173
    """
    if not value or direction not in ['S', 'N']:
        return None
    
    degrees = int(value[:2])  # First 2 characters for degrees
    minutes = float(value[2:])  # Remaining characters for minutes
    decimal = degrees + minutes / 60  # Convert to decimal degrees

    return decimal if direction == 'N' else -decimal  # Apply direction


def get_lon(value, direction):
    """
    Convert a NMEA longitude value to decimal degrees.
    Example: 01131.324,E → 11.5221
    """
    if not value or direction not in ['W', 'E']:
        return None
    
    degrees = int(value[:3])  # First 3 characters for degrees
    minutes = float(value[3:])  # Remaining characters for minutes
    decimal = degrees + minutes / 60  # Convert to decimal degrees

    return decimal if direction == 'E' else -decimal  # Apply direction
