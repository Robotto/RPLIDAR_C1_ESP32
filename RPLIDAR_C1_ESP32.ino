/*
 * RPLidarC1 Arduino Library Example
 * 
 * This example demonstrates how to use the RPLidarC1 library with ESP32
 * to communicate with an RPLidar C1 laser scanner.
 * 
 * Hardware connections:
 * - RPLidar VCC -> 5V
 * - RPLidar GND -> GND  
 * - RPLidar TX -> GPIO 33 (RX2)
 * - RPLidar RX -> GPIO 26 (TX2)
 * 
 * Note: RPLidar C1 uses UART-based motor control (no separate motor pin needed)
 * Make sure your ESP32 can provide enough current for the RPLidar!
 */

#define RXpin 33
#define TXpin 26

#include "RPLidarC1.h"
HardwareSerial C1UART(2);

// Create RPLidar instance using Serial2 (C1 uses UART motor control)
RPLidarC1 lidar(&C1UART);
RPLidarHealth health;

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("RPLidarC1 Arduino Library Example");
  Serial.println("==================================");

  // Initialize the lidar
  Serial.println("Initializing RPLidar...");
  if (!lidar.begin(460800, 4000, RXpin, TXpin)) {
    Serial.println("Failed to initialize RPLidar!");
    Serial.println("Check connections and power supply.");
    while (1) {
      delay(1000);
    }
  }

  Serial.println("RPLidar initialized successfully!");


  // Check health status
  
  if (lidar.get_health(&health)) {
    lidar.print_health(&health);

    if (health.status != RPLIDAR_STATUS_OK) {
      Serial.println("Warning: RPLidar is not in good health!");
      if (health.status == RPLIDAR_STATUS_ERROR) {
        Serial.println("Attempting to reset...");
        lidar.reset();
        delay(2000);
      }
    }
  } else {
    Serial.println("Failed to get health status");
  }

  
  
  // Start scanning
  Serial.println("Starting scan...");
  if (lidar.start_scan()) {
    Serial.println("Scan started successfully");
  } else {
    Serial.println("Failed to start scan");
    while (1) {
      delay(1000);
    }
  }

  Serial.println("Reading measurements...");
  Serial.println("Format: Angle(deg) Distance(mm) Quality StartFlag");
  Serial.println("================================================");
}

bool run = true;
void loop() {
  RPLidarMeasurement measurement;

  // Get a single measurement
  if(run){
  if (lidar.get_measurement(&measurement)) {
    // Print measurement data
    Serial.print("Angle: ");
    Serial.print(measurement.angle, 2);
    Serial.print("°  Distance: ");
    Serial.print(measurement.distance, 1);
    Serial.print("mm  Quality: ");
    Serial.print(measurement.quality);
    Serial.print("  Start: ");
    Serial.print(measurement.start_flag ? "Y" : "N");
    Serial.print("  Time: ");
    Serial.println(measurement.timestamp);

    // Optional: Filter out invalid measurements (C1 specific ranges)
    if (measurement.distance < 5.0 || measurement.distance > 12000.0) {
      Serial.println("  -> Invalid distance reading");
    }

    if (measurement.quality < 15) {  // C1 has different quality scaling
      Serial.println("  -> Low quality reading");
    }

  } else {
    Serial.println("Failed to get measurement");
    delay(10);
  }
  }

  static unsigned long last_health_check = 0;
  if (millis() - last_health_check > 30000) {
    last_health_check = millis();

    RPLidarHealth health;
    if (lidar.get_health(&health)) {
      if (health.status != RPLIDAR_STATUS_OK) {
        Serial.println("\nWarning: Health status changed!");
        
      }
      lidar.print_health(&health);
    }
  }

  if(millis()>10000 && run) 
  {
    run = false;
    Serial.println("Done after 10 seconds..");
    lidar.stop_scan();
    lidar.reset();
   // while(1) {delay(10);}
  }
  delay(1);  // Small delay to prevent overwhelming the serial output
}
