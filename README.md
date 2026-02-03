# IoT Smart Room & AWS Cloud Integration

## Project Overview

This project was developed for the CPE 4903 IoT course. It’s a smart room controller that handles lighting, ventilation, and security using an Arduino MKR WiFi 1010. The system detects if someone is in the room and checks the light levels to decide whether to turn on the LED or run the fan.

The coolest part is the AWS integration—the board stays synced with an AWS IoT Device Shadow, so it can report energy usage data to the cloud and receive remote commands to override the hardware.

## Features

* **Automatic Lighting & Fan:** Uses an ultrasonic sensor for occupancy and a photoresistor for light. If you're in the room and it's dark, the light comes on. If it's bright, the fan runs instead.
* **Energy Tracking:** I wrote a custom function to estimate power consumption based on which components (LED, Fan, Servo) are currently active.
* **AWS IoT Connectivity:** The system connects via MQTT over SSL/TLS. It publishes updates every 15 seconds and listens for "desired" state changes from the cloud.
* **Hardware Security:** Uses the onboard ECCX08 crypto-chip to handle secure authentication with AWS.

## Hardware Used

* **Microcontroller:** Arduino MKR WiFi 1010
* **Sensors:** HC-SR04 Ultrasonic, Photoresistor
* **Actuators:** DC Motor (Fan), Servo Motor (Vent), LED
* **Other:** L293D Motor Driver

## How the Code Works

1. **Sensing:** The code constantly pings the ultrasonic sensor to check for a "person" within 2cm to 5cm.
2. **Logic:** If occupied, it checks the `lightlevel`. If the value is above the `DARK_THRESHOLD`, it triggers the "Dark" state.
3. **Cloud Sync:** Every 15 seconds, a JSON payload is created using `ArduinoJson` and sent to the AWS shadow topic.
4. **Remote Control:** The `onMessageReceived` function parses incoming MQTT messages. If you change a setting in the AWS console, the hardware responds instantly.

---

**Course:** CPE 4903 - IoT Systems

**Author:** Anthony Nguyen
