#include <TinyGPS++.h>    // Include the GPS parsing library
#include <HardwareSerial.h> // For using UART1 with ESP32
#include <WiFi.h>         // Include Wi-Fi library for ESP32
#include <BlynkSimpleEsp32.h> // Include Blynk library

// Blynk Template and Auth Token
#define BLYNK_TEMPLATE_ID "TMPL6jZxHMXjf"
#define BLYNK_TEMPLATE_NAME "hub"
#define BLYNK_AUTH_TOKEN "h90AI4s_JC1AYzTuO1iFnSZqm-QWsjp5"

// Wi-Fi Credentials
char ssid[] = "AHHSN Shop 2.4GHz_2";      // Replace with your Wi-Fi SSID
char pass[] = "03727124";                 // Replace with your Wi-Fi Password

// Ultrasonic Sensor Pins
#define TRIG_PIN 5  // GPIO pin for Trigger
#define ECHO_PIN 18 // GPIO pin for Echo

// LED Pin
#define LED_PIN 2   // GPIO pin for LED (use onboard LED or an external one)

// GPS Configuration
TinyGPSPlus gps;
HardwareSerial GPS_Serial(1); // Use UART1 for GPS (RX=16, TX=17)

// Variables for duplicate data filtering
float lastLatitude = 0.0, lastLongitude = 0.0;
float lastAltitude = 0.0;

// Blynk Virtual Pins
#define VIRTUAL_DISTANCE V0
#define VIRTUAL_LATITUDE V1
#define VIRTUAL_LONGITUDE V2
#define VIRTUAL_ALTITUDE V3
#define VIRTUAL_LED V4

void setup() {
  Serial.begin(115200); // Initialize Serial Monitor at 115200 baud
  GPS_Serial.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17 for GPS

  pinMode(TRIG_PIN, OUTPUT); // Set TRIG_PIN as OUTPUT
  pinMode(ECHO_PIN, INPUT);  // Set ECHO_PIN as INPUT
  pinMode(LED_PIN, OUTPUT);  // Set LED_PIN as OUTPUT

  // Initialize Blynk and connect to Wi-Fi
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  Serial.println("Ultrasonic Sensor, GPS Module, and Blynk Integration Test");
}

void loop() {
  Blynk.run(); // Keep Blynk connected

  // Ultrasonic Sensor Readings
  long duration;
  float distance;
  float percentage = 0; // Variable to store the percentage level

  // Trigger Ultrasonic Sensor
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read Echo
  duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate Distance in cm
  distance = duration * 0.034 / 2;

  // Calculate the percentage based on distance
  if (distance < 10.0) {
    percentage = 100; // Maximum level
  } else if (distance <= 100.0) {
    percentage = 100 - ((distance - 10) / 90.0) * 100;
    percentage = max(0.0f, percentage); // Ensure the percentage does not go below 0
  } else {
    percentage = 0; // Minimum level
  }

  // Send the percentage to Blynk
  Blynk.virtualWrite(VIRTUAL_DISTANCE, percentage);

  // Turn on the LED if the distance is less than 10 cm
  if (distance < 10.0) {
    digitalWrite(LED_PIN, HIGH); // Turn LED ON
    Blynk.virtualWrite(VIRTUAL_LED, 255); // Turn ON virtual LED
  } else {
    digitalWrite(LED_PIN, LOW); // Turn LED OFF
    Blynk.virtualWrite(VIRTUAL_LED, 0); // Turn OFF virtual LED
  }

  // Print Ultrasonic Sensor Data to Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(" cm, Level: ");
  Serial.print(percentage);
  Serial.println("%");

  // GPS Readings
  while (GPS_Serial.available() > 0) {
    char c = GPS_Serial.read();
    gps.encode(c); // Parse the GPS data

    if (gps.location.isUpdated()) {
      float currentLat = gps.location.lat();
      float currentLng = gps.location.lng();
      float currentAlt = gps.altitude.meters();

      // Check for new GPS data to avoid duplicates
      if (currentLat != lastLatitude || currentLng != lastLongitude || currentAlt != lastAltitude) {
        Serial.print("Location: ");
        Serial.print("Lat: ");
        Serial.print(currentLat, 6);
        Serial.print(", Lng: ");
        Serial.println(currentLng, 6);

        Serial.print("Altitude: ");
        Serial.print(currentAlt);
        Serial.println(" m");

        // Send GPS Data to Blynk
        Blynk.virtualWrite(VIRTUAL_LATITUDE, currentLat);
        Blynk.virtualWrite(VIRTUAL_LONGITUDE, currentLng);
        Blynk.virtualWrite(VIRTUAL_ALTITUDE, currentAlt);

        // Update last values
        lastLatitude = currentLat;
        lastLongitude = currentLng;
        lastAltitude = currentAlt;
      }
    }
  }

  // Delay before next loop iteration
  delay(1000); // 1-second delay
}