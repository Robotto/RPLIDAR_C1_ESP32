/*
 * RPLidarC1 Arduino Library for ESP32
 * Based on the Python rplidarc1 library functionality
 * 
 * This library provides functionality for communicating with RPLidar C1 laser scanners
 * 
 * Converted from Python library https://github.com/dsaadatmandi/rplidarc1
 * Version: 0.0.1
 */

#ifndef RPLIDARC1_H
#define RPLIDARC1_H

#include <Arduino.h>
#include <HardwareSerial.h>

// RPLidar Protocol Constants
#define RPLIDAR_SYNC_BYTE1          0xA5
#define RPLIDAR_SYNC_BYTE2          0x5A

// Command Codes
#define RPLIDAR_CMD_STOP            0x25
#define RPLIDAR_CMD_RESET           0x40
#define RPLIDAR_CMD_SCAN            0x20
#define RPLIDAR_CMD_EXPRESS_SCAN    0x82
#define RPLIDAR_CMD_FORCE_SCAN      0x21
#define RPLIDAR_CMD_GET_INFO        0x50
#define RPLIDAR_CMD_GET_HEALTH      0x52
#define RPLIDAR_CMD_GET_SAMPLERATE  0x59

#define RPLIDAR_CMD_GET_ACC_BOARD_FLAG 0xFF

// Response Types
#define RPLIDAR_ANS_TYPE_DEVHEALTH  0x03
#define RPLIDAR_ANS_TYPE_MEASUREMENT 0x81

// Health Status
#define RPLIDAR_STATUS_OK           0x0
#define RPLIDAR_STATUS_WARNING      0x1
#define RPLIDAR_STATUS_ERROR        0x2


// Data structures
struct RPLidarMeasurement {
    bool start_flag;
    bool quality_flag;
    float angle;        // in degrees
    float distance;     // in mm
    uint8_t quality;
    uint32_t timestamp;
};

struct RPLidarHealth {
    uint8_t status;
    uint16_t error_code;
};

class RPLidarC1 {
private:
    HardwareSerial* _serial;
    bool _scanning;
    uint32_t _timeout_ms;
    
    // Internal methods
    bool _send_command(uint8_t cmd, const uint8_t* payload = nullptr, uint8_t payload_size = 0);
    bool _wait_response_header(uint8_t expected_type, uint32_t* response_size);
    bool _read_response_data(uint8_t* buffer, uint32_t size);
    void _flush_serial();
    uint16_t _calculate_checksum(const uint8_t* data, uint8_t len);
    bool _parse_measurement_node(const uint8_t* buffer, RPLidarMeasurement* measurement);
    
public:
    // Constructor
    RPLidarC1(HardwareSerial* serial);
    
    // Initialization
    bool begin(uint32_t baudrate = 115200, uint32_t timeout_ms = 1000, int rxpin = 16, int txpin=17);
    void end();
    
        // Device information
    bool get_health(RPLidarHealth* health);
    
    // Scanning control
    bool start_scan(bool force = false);
    bool stop_scan();
    bool is_scanning();
    
    // Data acquisition
    bool get_measurement(RPLidarMeasurement* measurement);
    int get_measurements(RPLidarMeasurement* measurements, int max_count, uint32_t timeout_ms = 1000);
    
    // System control
    bool reset();
    bool disconnect();
    
    // Utility functions
    void set_timeout(uint32_t timeout_ms);
    bool is_connected();
  //  void print_device_info(const RPLidarDeviceInfo& info);
    void print_health(RPLidarHealth* health);
    const char* get_health_status_string(uint8_t status);
};

#endif // RPLIDARC1_H