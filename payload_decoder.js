function Decoder(bytes, port) {
  // Decode an uplink message from a buffer
  // (array) of bytes to an object of fields.
  //value = decoded<<16>>16;

  // if (port === 1) decoded.led = bytes[0];

  //Batterie
  var batterie = 0.0;
  var in_min = 0;
  var out_min = 2.0;
  var out_max = 4.0;
  var in_max = 0.0;
  
  if (port === 1) {
    in_max = 255;
    batterie = ( bytes[0] - in_min ) * ( ( out_max - out_min) / ( in_max - in_min ) ) + out_min;
    batterie = batterie.toFixed(2);
  } else if (port === 2) {
    in_max = 65535;
    batterie = ( ( (bytes[0] << 8 ) + bytes[1]) - in_min ) * ( ( out_max - out_min) / ( in_max - in_min ) ) + out_min;
    batterie = batterie.toFixed(3);
  }
  
  if (batterie < out_min) {
		batterie = out_min;
	} else if (batterie > out_max) {
		batterie = out_max; 
	}
	
	//Temperature
	var temperature = 0.0;
	out_min = -40.0;
  out_max = 85.0;
  
	if (port === 1) {
	  in_max = 250;
    temperature = ( bytes[1] - in_min ) * ( ( out_max - out_min) / ( in_max - in_min ) ) + out_min;
    temperature = temperature.toFixed(1);
  } else if (port === 2) {
    in_max = 65535;
    temperature = ( ( (bytes[2] << 8 ) + bytes[3]) - in_min ) * ( ( out_max - out_min) / ( in_max - in_min ) ) + out_min;
    temperature = temperature.toFixed(2);
  }
  
  if (temperature < out_min) {
		temperature = out_min;
	} else if (temperature > out_max) {
		temperature = out_max; 
	}
	
	//Humidity
	var humidity = 0.0
	out_min = 0.0;
  out_max = 100.0;
  
  if (port === 1) {
    in_max = 200;
    humidity = ( bytes[2] - in_min ) * ( ( out_max - out_min) / ( in_max - in_min ) ) + out_min;
    humidity = humidity.toFixed(1);
  } else if (port === 2) {
    humidity = ( ( (bytes[4] << 8 ) + bytes[5]) - in_min ) * ( ( out_max - out_min) / ( in_max - in_min ) ) + out_min;
    humidity = humidity.toFixed(2);
  }
  
  if (humidity < out_min) {
		humidity = out_min;
	} else if (humidity > out_max) {
		humidity = out_max; 
	}
	
	//Pressure
	var pressure = 950.0;
	out_min = 950.0;
  out_max = 1050.0;
  
  if (port === 1) {
    pressure = ( bytes[3] - in_min ) * ( ( out_max - out_min) / ( in_max - in_min ) ) + out_min;
    pressure = pressure.toFixed(1);
  } else if (port === 2) {
    pressure = ( ( (bytes[6] << 8 ) + bytes[7]) - in_min ) * ( ( out_max - out_min) / ( in_max - in_min ) ) + out_min;
    pressure = pressure.toFixed(2);
  } 
  
  if (pressure < out_min) {
		pressure = out_min;
	} else if (pressure > out_max) {
		pressure = out_max; 
	}

	
	var text = "";
	if (port === 10) {
    var sensor_status = bytes[0];
    
    if ( sensor_status !== 0) {
      sensor_status = sensor_status - 256; //negative number
    }
    
    switch (sensor_status) {
    case 0:
            text = "BME280_OK";
            break;
        
        case -1:
            text = "BME280_E_NULL_PTR";
            break;
        
        case -2:
            text = "BME280_E_DEV_NOT_FOUND";
            break;
        
        case -3:
            text = "BME280_E_INVALID_LEN";
            break;
        
        case -4:
            text = "BME280_E_COMM_FAIL";
            break;
        
        case -5:
            text = "BME280_E_SLEEP_MODE_FAIL";
            break;
        
        default:
            text = "Unknown status code: ";
            text = text.concat(sensor_status);
            break;
    }
	  return {
	    sensor_status: text
	  };
	} else {
    return {
      batterie: batterie,
      temperature: temperature,
      humidity:humidity,
      pressure: pressure
    };
	}
}

