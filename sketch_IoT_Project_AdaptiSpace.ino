#include <Servo.h>          // Include Servo library
#include <SPI.h>            // Required for WiFiNINA
#include <ArduinoBearSSL.h>
#include <ArduinoECCX08.h>
#include <ArduinoMqttClient.h>
#include <WiFiNINA.h> // change to #include <WiFi101.h> for MKR1000
#include <ArduinoJson.h>    // For creating JSON payloads
#include <Wire.h>           // Required for I2C communication (if using I2C sensors)
#include "arduino_secrets.h"

// Pin Definitions
#define led 4
#define servoPin 1
#define phoRes A0
#define echoPin 6
#define trigPin 7

// DC Motor Pinouts
#define input2 3
#define input1 2
#define enablePin 5 // PWM pin for motor speed control

// Sensor Variables
int lightlevel = 0;
int DARK_THRESHOLD = 600; // Threshold for darkness (adjust based on your photoresistor)
int maxRange = 5;         // Maximum distance for occupancy in cm
int minRange = 2;         // Minimum distance for occupancy in cm
long duration, distance;

// Estimated Energy Usage Variables (Relative Units)
const int LED_EST_COST = 5;      // Relative cost when LED is on
const int FAN_EST_COST = 50;     // Relative cost when Fan is on (at full speed)
const int SERVO_EST_COST = 20;   // Relative cost when Servo is active (moving or holding position)
int estimatedEnergyUsage = 0;    // Total estimated energy usage

// System State
bool isOccupied = false;
Servo myServo;
int currentServoAngle = 0; // To track servo position for energy estimation

/////// Enter your sensitive data in arduino_secrets.h
const char ssid[]        = SECRET_SSID;
const char pass[]        = SECRET_PASS;
const char broker[]      = SECRET_BROKER;
const char* certificate  = SECRET_CERTIFICATE;

WiFiClient    wifiClient;            // Used for the TCP socket connection
BearSSLClient sslClient(wifiClient); // Used for SSL/TLS connection, integrates with ECC508
MqttClient    mqttClient(sslClient);

// MQTT topics for Device Shadow
// Replace "YOUR_THING_NAME" with the actual name of your AWS IoT Thing
const char shadowUpdateTopic[] = "$aws/things/YOUR_THING_NAME/shadow/update";
// const char shadowDeltaTopic[] = "$aws/things/YOUR_THING_NAME/shadow/update/delta"; // For receiving desired state changes

unsigned long lastPublishMillis = 0;
const long publishInterval = 15000; // Publish sensor data to shadow every 15 seconds

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!ECCX08.begin()) {
    Serial.println("No ECCX08 present!");
    while (1);
  }

  // Pin setup
  pinMode(led, OUTPUT);
  pinMode(servoPin, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  myServo.attach(servoPin);

  // DC Motor Pinouts setup
  pinMode(input2, OUTPUT);
  pinMode(input1, OUTPUT);
  pinMode(enablePin, OUTPUT); // Ensure enable pin is set as OUTPUT

  // Initialize outputs
  analogWrite(led, 0); // Ensure LED is off initially
  myServo.write(0);    // Set servo to initial position
  currentServoAngle = 0; // Initialize tracked angle
  fan_off();           // Ensure fan is off initially

  // Set a callback to get the current time
  // used to validate the servers certificate
  ArduinoBearSSL.onGetTime(getTime);

  // Set the ECCX08 slot to use for the private key
  // and the accompanying public certificate for it
  sslClient.setEccSlot(0, certificate);

  // Optional, set the client id used for MQTT,
  // each device that is connected to the broker
  // must have a unique client id. The MQTTClient will generate
  // a client id for you based on the millis() value if not set
  //
  // mqttClient.setId("clientId");

  // Set the message callback, this function is
  // called when the MQTTClient receives a message
  mqttClient.onMessage(onMessageReceived);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  if (!mqttClient.connected()) {
    // MQTT client is disconnected, connect
    connectMQTT();
  }

  // Perform sensor checks
  occupancy_check();
  light_check();

  // Control outputs based on sensor readings
  illumination();

  // Calculate estimated energy usage
  calculateEstimatedEnergy();

  // poll for new MQTT messages and send keep alives
  mqttClient.poll();

  // Publish sensor data periodically to the AWS IoT Device Shadow
  if (millis() - lastPublishMillis > publishInterval) {
    lastPublishMillis = millis(); // Update last publish time
    publishShadowUpdate(); // Publish the current state to the device shadow
  }
}

unsigned long getTime() {
  // get the current time from the WiFi module  
  return WiFi.getTime();
}

// Function to check for occupancy using the ultrasonic sensor
void occupancy_check() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck >= 200) { // Check every 200 milliseconds
    // Trigger the ultrasonic sensor
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Measure the duration of the echo pulse
    duration = pulseIn(echoPin, HIGH, 30000); // Timeout after 30ms

    // Calculate the distance in cm
    distance = duration / 58.2;

    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    // Determine occupancy based on distance thresholds
    isOccupied = (distance >= minRange && distance <= maxRange);

    lastCheck = millis(); // Update last check time
  }
}

// Function to read the photoresistor level
void light_check() {
  lightlevel = analogRead(phoRes);  // Read photoresistor value
  Serial.print("Light Level: ");
  Serial.println(lightlevel);
}


// Function to control LED and fan based on occupancy and light level
void illumination() {
  if (isOccupied) {
    if (lightlevel > DARK_THRESHOLD) {
      // Dark conditions - turn on LED and stop fan
      fan_off();
      myServo.write(90); // Example servo position for dark
      currentServoAngle = 90;
      analogWrite(led, 255); // LED turned on (assuming analogWrite for brightness control, otherwise use digitalWrite(led, HIGH))
    }
    else {
      // Bright conditions - turn off LED and run fan
      fan_on();
      myServo.write(0); // Example servo position for bright
      currentServoAngle = 0;
      analogWrite(led, 0); // LED turned off
    }
  }
  else {
    // Room empty - turn off LED and fan
    fan_off();
    analogWrite(led, 0);
    myServo.write(0); // Return servo to initial position
    currentServoAngle = 0;
  }
}

// Function to turn the fan on
void fan_on() {
  analogWrite(enablePin, 255); // Set motor speed (PWM value 0-255)
  digitalWrite(input1, HIGH); // Set direction
  digitalWrite(input2, LOW);
  Serial.println("Fan ON");
}

// Function to turn the fan off
void fan_off() {
  analogWrite(enablePin, 0); // Stop the motor
  digitalWrite(input1, LOW);
  digitalWrite(input2, LOW);
  Serial.println("Fan OFF");
}

// Function to calculate estimated energy usage based on component states
void calculateEstimatedEnergy() {
  estimatedEnergyUsage = 0;

  // Add cost if LED is on
  if (analogRead(led) > 0) { // Check if LED PWM value is greater than 0
    estimatedEnergyUsage += LED_EST_COST;
  }

  // Add cost if Fan is on
  if (analogRead(enablePin) > 0) { // Check if motor enable pin PWM value is greater than 0
    estimatedEnergyUsage += FAN_EST_COST; // Assuming full speed cost
    // If using variable speed, you could scale the cost based on analogWrite(enablePin) value
    // estimatedEnergyUsage += map(analogRead(enablePin), 0, 255, 0, FAN_EST_COST);
  }

  // Add cost if Servo is not at its initial position (indicating it's holding or moving)
  // This is a simplification; a more accurate way would track movement
  if (currentServoAngle != 0) { // Assuming 0 is the idle/initial position
     estimatedEnergyUsage += SERVO_EST_COST;
  }
   // A more complex approach would involve tracking if the servo is actively moving

  Serial.print("Estimated Energy Usage: ");
  Serial.println(estimatedEnergyUsage);
}

// Function to publish the current state to the AWS IoT Device Shadow
void publishShadowUpdate() {
  // Create a JSON payload for the Device Shadow update
  StaticJsonDocument<256> doc; // Adjust size based on your payload

  // Structure for reporting state to the shadow
  JsonObject state = doc.createNestedObject("state");
  JsonObject reported = state.createNestedObject("reported");

  reported["timestamp"] = millis(); // Timestamp in milliseconds since board started
  reported["light_level"] = lightlevel;
  reported["is_occupied"] = isOccupied;
  reported["distance_cm"] = distance;

  // Include estimated energy usage
  reported["estimated_energy_usage"] = estimatedEnergyUsage;

  // Optional: Include the current state of outputs
  reported["led_status"] = (analogRead(led) > 0); // Assuming analogWrite(led, 255) is HIGH
  reported["fan_status"] = (analogRead(enablePin) > 0); // Assuming analogWrite(enablePin, 255) is ON
  reported["servo_angle"] = currentServoAngle;


  String jsonString;
  serializeJson(doc, jsonString); // Serialize JSON document to a string

  Serial.print("Publishing shadow update to topic: ");
  Serial.println(shadowUpdateTopic);
  Serial.println(jsonString);

  // Publish the JSON message to the shadow update topic
  mqttClient.beginMessage(shadowUpdateTopic);
  mqttClient.print(jsonString);
  mqttClient.endMessage();

  Serial.println("Shadow update published.");
}

void onMessageReceived(int messageSize) {
    Serial.print("Received message on topic: ");
    Serial.print(mqttClient.messageTopic());
    Serial.print(". Payload: ");

    // Read the message payload
    String messagePayload = "";
    while (mqttClient.available()) {
      messagePayload += (char)mqttClient.read();
    }
    Serial.println(messagePayload);

    // Parse the JSON message to check for desired state changes
    StaticJsonDocument<256> doc; // Adjust size
    DeserializationError error = deserializeJson(doc, messagePayload);

    if (error) {
      Serial.print("deserializeJson() failed: ");
      Serial.println(error.c_str());
      return;
    }

    // Example: Check for desired fan state
    if (doc.containsKey("state") && doc["state"].containsKey("desired")) {
      JsonObject desired = doc["state"]["desired"];
      if (desired.containsKey("fan_status")) {
        bool desiredFanStatus = desired["fan_status"];
        if (desiredFanStatus) {
          fan_on();
        } else {
          fan_off();
        }
        // Acknowledge the desired state by reporting the new state
        publishShadowUpdate(); // Report the updated state including fan_status
      }
      // Add similar logic for desired LED or servo state
      if (desired.containsKey("servo_angle")) {
         int desiredAngle = desired["servo_angle"];
         myServo.write(desiredAngle);
         currentServoAngle = desiredAngle; // Update tracked angle
         publishShadowUpdate(); // Report the updated state including servo_angle
      }
      if (desired.containsKey("led_status")) {
         bool desiredLedStatus = desired["led_status"];
         if (desiredLedStatus) {
             analogWrite(led, 255); // Or digitalWrite(led, HIGH)
         } else {
             analogWrite(led, 0); // Or digitalWrite(led, LOW)
         }
         publishShadowUpdate(); // Report the updated state including led_status
      }
   }
 }


void connectWiFi() {
  Serial.print("Attempting to connect to SSID: ");
  Serial.print(ssid);
  Serial.print(" ");

  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    // failed, retry
    Serial.print(".");
    delay(5000);
  }
  Serial.println();

  Serial.println("You're connected to the network");
  Serial.println();
}

void connectMQTT() {
  Serial.print("Attempting to MQTT broker: ");
  Serial.print(broker);
  Serial.println(" ");

  while (!mqttClient.connect(broker, 8883)) {
    // failed, retry
    Serial.print(".");
    delay(5000);
  }
  Serial.println();

  Serial.println("You're connected to the MQTT broker");
  Serial.println();
}

message.txt
13 KB