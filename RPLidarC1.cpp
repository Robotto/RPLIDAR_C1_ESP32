/*
 * RPLidarC1 Arduino Library Implementation for ESP32
 * Based on the Python rplidarc1 library functionality
 */

#include "RPLidarC1.h"

RPLidarC1::RPLidarC1(HardwareSerial* serial) {
    _serial = serial;
    _scanning = false;
    _timeout_ms = 1000;
}

bool RPLidarC1::begin(uint32_t baudrate, uint32_t timeout_ms, int rxpin, int txpin) {
    _timeout_ms = timeout_ms;
    
    if (!_serial) {
        return false;
    }
    
    _serial->begin(baudrate,SERIAL_8N1,rxpin,txpin);
    delay(100);
    
    // Flush any existing data
    _flush_serial();
    
    // Try to get device info to verify connection
    reset();
    //RPLidarDeviceInfo info;
    RPLidarHealth health;
    return get_health(&health);
}

void RPLidarC1::end() {
    stop_scan();
    if (_serial) {
        _serial->end();
    }
}

bool RPLidarC1::get_health(RPLidarHealth* health) {
    if (!health || !_send_command(RPLIDAR_CMD_GET_HEALTH)) {
        return false;
    }
    
    uint32_t response_size;
    if (!_wait_response_header(RPLIDAR_ANS_TYPE_DEVHEALTH, &response_size)) {
        Serial.println("in get_health(): _wait_response_header failed!");
        return false;
    }
    
    if (response_size != 3) {
        Serial.print("in get_health(): calculated response size !=3 but instead= "); Serial.println(response_size);
        return false;
    }
    
    uint8_t buffer[response_size];
    if (!_read_response_data(buffer, response_size)) {
        Serial.println("in get_health(): didn't get expected number of bytes. (3), even though response header said they would come");
        return false;
    }
    
    health->status = buffer[0];
    health->error_code = (buffer[2] << 8) | buffer[1];
    
    return true;
}
bool RPLidarC1::start_scan(bool force) {
    uint8_t cmd = force ? RPLIDAR_CMD_FORCE_SCAN : RPLIDAR_CMD_SCAN;
    
    if (!_send_command(cmd)) {
        return false;
    }
    
    uint32_t response_size;
    if (!_wait_response_header(RPLIDAR_ANS_TYPE_MEASUREMENT, &response_size)) {
        return false;
    }
    
    _scanning = true;
    return true;
}

bool RPLidarC1::stop_scan() {
    if (!_send_command(RPLIDAR_CMD_STOP)) {
        return false;
    }
    
    _scanning = false;
    delay(10);
    _flush_serial();
    return true;
}

bool RPLidarC1::is_scanning() {
    return _scanning;
}

bool RPLidarC1::get_measurement(RPLidarMeasurement* measurement) {
    if (!measurement || !_scanning) {
        return false;
    }
    
    uint8_t buffer[5];
    uint32_t start_time = millis();
    
    while (millis() - start_time < _timeout_ms) {
        if (_serial->available() >= 5) {
            _serial->readBytes(buffer, 5);
            if (_parse_measurement_node(buffer, measurement)) {
                measurement->timestamp = millis();
                return true;
            }
        }
        delay(1);
    }
    
    return false;
}

int RPLidarC1::get_measurements(RPLidarMeasurement* measurements, int max_count, uint32_t timeout_ms) {
    if (!measurements || max_count <= 0 || !_scanning) {
        return 0;
    }
    
    int count = 0;
    uint32_t start_time = millis();
    
    while (count < max_count && (millis() - start_time) < timeout_ms) {
        if (get_measurement(&measurements[count])) {
            count++;
        }
    }
    
    return count;
}

bool RPLidarC1::reset() {
    Serial.print("Resetting Lidar... ");
    if (!_send_command(RPLIDAR_CMD_RESET)) {
        Serial.println("Send failed!");
        return false;
    }
    
    delay(1000); // Wait for reset to complete
    _scanning = false;
    _flush_serial();
    Serial.println("Sent!");
    return true;
}

bool RPLidarC1::disconnect() {
    stop_scan();
    return true;
}

void RPLidarC1::set_timeout(uint32_t timeout_ms) {
    _timeout_ms = timeout_ms;
}

bool RPLidarC1::is_connected() {
        RPLidarHealth health;
    return get_health(&health);
}

// Private methods implementation

bool RPLidarC1::_send_command(uint8_t cmd, const uint8_t* payload, uint8_t payload_size) {
    if (!_serial) {
        return false;
    }
    
    uint8_t checksum = 0;
    uint8_t header[2] = {RPLIDAR_SYNC_BYTE1, cmd};
    /*
    // Calculate checksum
    checksum ^= RPLIDAR_SYNC_BYTE1;
    checksum ^= cmd;
    
    if (payload && payload_size > 0) {
        checksum ^= payload_size;
        for (uint8_t i = 0; i < payload_size; i++) {
            checksum ^= payload[i];
        }
    }
    */
    // Send command
    _serial->write(header, 2);
    /*
    if (payload && payload_size > 0) {
        _serial->write(payload_size);
        _serial->write(payload, payload_size);
    }
    
    _serial->write(checksum);
    */
    _serial->flush();
    Serial.print("TX: ");
    Serial.print(header[0],HEX);
    Serial.print(":");
    Serial.print(header[1],HEX);
    Serial.println();
    //if(command== RESET: Wait longer
    return true;
}

bool RPLidarC1::_wait_response_header(uint8_t expected_type, uint32_t* response_size) {
    if (!_serial || !response_size) {
        return false;
    }
    
    uint32_t start_time = millis();
    uint8_t header[7]={0,0,0,0,0,0,0};
    uint8_t pointer=0;
    while (millis() - start_time < _timeout_ms) {

        //if (_serial->available()) Serial.print(_serial->available()); Serial.println(" bytes waiting in rx buffer...");
        if (_serial->available()) {
            header[pointer]=_serial->read();
            pointer++;
            Serial.print("Pointer = "); Serial.print(pointer); Serial.print(" ");
            Serial.print("Buffer: ");
            for(int i = 0; i<7;i++) { Serial.print(header[i],HEX); if(i<6) Serial.print(":"); else Serial.println();}
            
        }
        //if (_serial->available() >= 7) {
        if (pointer == 7) {

            Serial.println("Got 7 bytes!");


            //_serial->readBytes(header, 7);
            // Check sync bytes
            if (header[0] == RPLIDAR_SYNC_BYTE1 && header[1] == RPLIDAR_SYNC_BYTE2) {
                Serial.println("Response header OK!");
                // Check response type
                if(expected_type == RPLIDAR_ANS_TYPE_DEVHEALTH){
                
                if ((header[2]) == RPLIDAR_ANS_TYPE_DEVHEALTH) {
                    Serial.println("Got Expected Health Packet Header!");
                    *response_size = 3;
                    return true;
                    }
                else{
                    Serial.println("Size calc failed!");
                    return false;
                }
                }
                else if(expected_type == RPLIDAR_ANS_TYPE_MEASUREMENT ){
                    if(header[6] == RPLIDAR_ANS_TYPE_MEASUREMENT) return true;
                    else return false;
                }
                else 
                {
                    Serial.print("Got unexpected response... Expected ");
                    Serial.print(expected_type,HEX);
                    Serial.print(" but got ");
                    Serial.print(header[3],HEX);
                    Serial.print(" / ");
                    Serial.print(header[6],HEX);
                    
                    
                    return false;
                }
            }
        }
        delay(1);
    }
    Serial.print("_wait_response_header() timed out... with ");
    Serial.print(Serial.available());
    Serial.println(" bytes in Serial buffer");
    

    return false;
}

bool RPLidarC1::_read_response_data(uint8_t* buffer, uint32_t size) {
    if (!_serial || !buffer || size == 0) {
        return false;
    }
    
    uint32_t start_time = millis();
    
    while (millis() - start_time < _timeout_ms) {
        if (_serial->available() >= size) {
            _serial->readBytes(buffer, size);
            return true;
        }
        delay(1);
    }
    
    return false;
}

void RPLidarC1::_flush_serial() {
    if (_serial) {
        while (_serial->available()) {
            _serial->read();
        }
    }
}

bool RPLidarC1::_parse_measurement_node(const uint8_t* buffer, RPLidarMeasurement* measurement) {
    if (!buffer || !measurement) {
        return false;
    }
    
    // C1 uses a different measurement format
    // Byte 0: [S][!S][C][6-bit quality]
    // Byte 1-2: Distance (LSB first) in 0.25mm units
    // Byte 3-4: Angle (LSB first) in 0.01 degree units
    
    uint8_t quality_and_flags = buffer[0];
    uint16_t distance_raw = (buffer[2] << 8) | buffer[1];
    uint16_t angle_raw = (buffer[4] << 8) | buffer[3];
    
    measurement->start_flag = (quality_and_flags & 0x01) != 0;
    measurement->quality_flag = (quality_and_flags & 0x02) == 0; // Inverted for C1
    measurement->quality = (quality_and_flags >> 2) & 0x3F;
    
    // Convert to physical units (C1 specific scaling)
    measurement->distance = distance_raw * 0.25f; // 0.25mm units
    measurement->angle = angle_raw * 0.01f; // 0.01 degree units
    
    // Validate angle range
    if (measurement->angle >= 360.0f) {
        measurement->angle -= 360.0f;
    }
    
    return true;
}



// Utility functions

void RPLidarC1::print_health(RPLidarHealth* health) {
    Serial.println("=== RPLidar Health ===");
    Serial.print("Status: ");
    Serial.println(get_health_status_string(health->status));
    Serial.print("Error Code: 0x");
    Serial.println(health->error_code, HEX);
}

const char* RPLidarC1::get_health_status_string(uint8_t status) {
    switch (status) {
        case RPLIDAR_STATUS_OK: return "OK";
        case RPLIDAR_STATUS_WARNING: return "WARNING";
        case RPLIDAR_STATUS_ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}