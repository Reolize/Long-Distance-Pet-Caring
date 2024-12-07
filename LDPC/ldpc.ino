#include <ESP32Servo.h> // Use ESP32Servo instead of Servo
#include "HX711.h"
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <PubSubClient.h> // MQTT library

#define TRIG_PIN 13 // HC-SR04 trigger pin
#define ECHO_PIN 14 // HC-SR04 echo pin
#define BUTTON_PIN 15 // Pin for manual feed button
#define SERVO_PIN 12 // Pin for servo motor
#define DISTANCE_THRESHOLD 5 // Pet detection threshold in cm

HX711 scale; // Initialize the load cell

Servo feederServo;

long duration;
int distance;
const int servoOpenAngle = 90; // Angle to dispense food
const int servoCloseAngle = 0; // Angle to close feeder

// Scheduling variables
int scheduledHour[2] = {9, 18}; // Default scheduled times (9 AM, 6 PM)
int scheduledMinute[2] = {0, 0}; // Default scheduled minutes
bool isScheduledFeed[2] = {false, false}; // Whether to feed at scheduled times

// WiFi and NTP setup for time management
const char* ssid = "Twenty-three";
const char* password = "23232323";

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 25200, 60000); // Adjust timezone offset

// Timing variables
unsigned long lastAutoFeedTime = 0;
unsigned long lastPetDetectTime = 0;
unsigned long lastButtonCheckTime = 0;
unsigned long buttonDebounceDelay = 1000; // Debounce delay for button
unsigned long petDetectInterval = 5000;  // Interval to check for pet again
unsigned long autoFeedInterval = 60000;   // Interval to prevent multiple auto-feeds

// MQTT setup
const char* mqttServer = "20.205.20.59"; // Replace with MQTT broker IP
const int mqttPort = 1883;
const char* mqttUser = ""; // Optional for local brokers
const char* mqttPassword = ""; // Optional for local brokers
const char* feedTopic = "petfeeder/manualFeed"; // MQTT topic for manual feed
const char* scheduleTopic = "petfeeder/schedule";

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  Serial.begin(115200);
  
  // Set up pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Set up servo
  feederServo.attach(SERVO_PIN);
  feederServo.write(servoCloseAngle); // Keep feeder closed initially

  // Initialize load cell
  scale.begin(19, 21); // DOUT, SCK pins for HX711
  scale.set_scale(1000.f); // Calibration factor (adjust based on your setup)
  scale.tare(); // Reset the scale

  // Initialize WiFi and NTP
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);  // Start Wi-Fi
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }
  Serial.println(" Connected!");

  // Initialize MQTT
  client.setServer(mqttServer, mqttPort);
  client.setCallback(mqttCallback);

  timeClient.begin();
}

// Timing variable for distance printing
unsigned long lastDistancePrintTime = 0; // Last time the distance was printed
const unsigned long distancePrintInterval = 1000; // Interval for printing distance (in milliseconds)

// In your loop function
void loop() {
    if (!client.connected()) {
        reconnectMQTT();
    }
    client.loop();

    unsigned long currentMillis = millis();

    // Reconnect to WiFi if disconnected
    if (WiFi.status() != WL_CONNECTED) {
        Serial.print("Reconnecting to WiFi...");
        WiFi.begin(ssid);
        delay(1000); // Allow some time for reconnection attempt
    }

    timeClient.update();

    int hours = timeClient.getHours();
    int minutes = timeClient.getMinutes();

    // Check for scheduled auto-feed times
    for (int i = 0; i < 2; i++) {
        if (isScheduledFeed[i] && (hours == scheduledHour[i] && minutes == scheduledMinute[i]) &&
            (currentMillis - lastAutoFeedTime >= autoFeedInterval)) {
            autoFeed();
            lastAutoFeedTime = currentMillis; // Reset auto-feed timer
        }
    }

    // Check manual feed button with debounce
    if (digitalRead(BUTTON_PIN) == LOW && (currentMillis - lastButtonCheckTime >= buttonDebounceDelay)) {
        Serial.println("Manual feed activated via button.");
        feedPet();
        lastButtonCheckTime = currentMillis; // Reset button debounce timer
    }

    // Pet detection with HC-SR04, timed with millis()
    if (currentMillis - lastPetDetectTime >= petDetectInterval) {
        digitalWrite(TRIG_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN, LOW);

        duration = pulseIn(ECHO_PIN, HIGH);
        distance = duration * 0.034 / 2;

        // Print the detected distance every second
        /**if (currentMillis - lastDistancePrintTime >= distancePrintInterval) {
            Serial.print("Distance detected: ");
            Serial.print(distance);
            Serial.println(" cm");
            lastDistancePrintTime = currentMillis; // Update last printed time
        } **/

        if (distance > 0 && distance <= DISTANCE_THRESHOLD) {
            Serial.println("Pet detected for auto-feed.");
            feedPet();
            lastPetDetectTime = currentMillis; // Reset pet detection timer
        }
    }
}


void feedPet() {
  feederServo.write(servoOpenAngle); // Open feeder
  unsigned long feedStartTime = millis();
  while (millis() - feedStartTime < 1000) {
    // Perform other tasks or check for interrupts
    delay(10); // Short delay to prevent blocking
  }
  feederServo.write(servoCloseAngle); // Close feeder
}


void autoFeed() {
  Serial.println("Auto-feeding at scheduled time.");
  feedPet();
}

// MQTT callback function to handle incoming messages
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Convert payload to string
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  // Check if the message is a manual feed command
  if (String(topic) == feedTopic && message == "FEED") {
    Serial.println("Manual feed activated via MQTT.");
    feedPet();
  }

  // Schedule update command
  if (String(topic) == scheduleTopic) {
    // Parse the message to get scheduled times (expected format: "HH,MM;HH,MM")
    int firstCommaIndex = message.indexOf(',');
    int secondCommaIndex = message.indexOf(';');
    int thirdCommaIndex = message.indexOf(',', secondCommaIndex + 1);
    
    if (firstCommaIndex != -1 && secondCommaIndex != -1 && thirdCommaIndex != -1) {
      scheduledHour[0] = message.substring(0, firstCommaIndex).toInt();
      scheduledMinute[0] = message.substring(firstCommaIndex + 1, secondCommaIndex).toInt();
      scheduledHour[1] = message.substring(secondCommaIndex + 1, thirdCommaIndex).toInt();
      scheduledMinute[1] = message.substring(thirdCommaIndex + 1).toInt();
      isScheduledFeed[0] = true; // Enable the first schedule
      isScheduledFeed[1] = true; // Enable the second schedule
      Serial.printf("Scheduled feeding times updated to %02d:%02d and %02d:%02d\n", 
                    scheduledHour[0], scheduledMinute[0], scheduledHour[1], scheduledMinute[1]);
    }
  }
}

// Reconnect to MQTT
void reconnectMQTT() {
  static unsigned long lastAttempt = 0;
  unsigned long now = millis();
  if (now - lastAttempt > 5000) { // Try every 5 seconds
    lastAttempt = now;
    while (!client.connected()) {
      Serial.print("Connecting to MQTT...");
      if (client.connect("ESP32Client", mqttUser, mqttPassword)) {
        Serial.println("connected");
        client.subscribe(feedTopic);
        client.subscribe(scheduleTopic);
      } else {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        delay(2000);
      }
    }
  }
}
