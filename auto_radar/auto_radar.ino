#include <WiFi.h>
#include <WiFiClient.h>
#include <ESP32Servo.h>

// Servo Pin Definitions
#define panServoPin 18
#define tiltServoPin 17
#define panServoPinCam 16
#define tiltServoPinCam 4


#include <NewPing.h>

// Ultrasonic Sensor Pin Definitions
#define TRIGGER_PIN1 25
#define ECHO_PIN1 33
#define TRIGGER_PIN2 26
#define ECHO_PIN2 27

// Ultrasonic Distance Variables
unsigned long dis_1 = 0;
unsigned long dis_2 = 0;
unsigned long threshold = 40;  // Example threshold for distance
int angle = 0;

String data = " ";
/////////////////////////

const char* ssid = "Ghuzy";
const char* password = "01066709440@AHMAG";

WiFiServer server(12345);

bool sweeping;
// Maximum distance we want to ping for (in centimeters)
#define MAX_DISTANCE 800

// Sensor object array
NewPing sonar[2] = {
  NewPing(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE),  // Each sensor's trigger pin, echo pin, and max distance to ping
  NewPing(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE)
};

// Servo objects
Servo panServo;
Servo tiltServo;
Servo panServoCam;
Servo tiltServoCam;

// Initial positions and step sizes for servo movement
int panPosition = 180;  // Initial middle position for pan
int tiltPosition = 90;  // Initial middle position for tilt
int panPositioncam = 90;
int tiltPositioncam = 90;
int panStep = 1;      // Step size for pan servo
int tiltStep = 5;     // Step size for tilt servo
int panStepcam = 1;   // Step size for pan servo
int tiltStepcam = 5;  //Step size for tilt servo

void setupServos() {
  // Allocate timers for each servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Standard 50 hz servo setup
  panServo.setPeriodHertz(100);
  tiltServo.setPeriodHertz(100);
  panServoCam.setPeriodHertz(100);
  tiltServoCam.setPeriodHertz(100);

  // Attach the servos to their respective pins
  panServo.attach(panServoPin, 500, 2500);
  tiltServo.attach(tiltServoPin, 500, 2500);
  panServoCam.attach(panServoPinCam, 500, 2500);
  tiltServoCam.attach(tiltServoPinCam, 500, 2500);

  // Write initial positions to servos
  panServo.write(panPosition);
  tiltServo.write(tiltPosition);
  panServoCam.write(panPositioncam);
  tiltServoCam.write(tiltPositioncam);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  setupServos();
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.print("ip");
  Serial.println(WiFi.localIP());
  server.begin();
  Serial.println("Server started");
}

void loop() {
  // Sweep the tilt servo and read distances
  WiFiClient client = server.available();
  if (client) {
    Serial.println("Client connected");

    while (client.connected()) {
      if (client.available()) {
        data = client.readStringUntil('\n');
        Serial.println("Received: " + data);
        // Process the received data here
      }
      ultrasonicRadar();
      radar();
      if (sweeping == 1) {
        sweepServo(tiltServo, tiltPosition, tiltStep, 0, 180);
      }
    }
  }
  client.stop();
  Serial.println("Client disconnected");
}

void sweepServo(Servo& servo, int& position, int& step, int minPos, int maxPos) {
  position += step;
  if (position <= minPos || position >= maxPos) {
    step = -step;      // Reverse direction at the boundaries
    position += step;  // Correct overshoot
  }
  servo.write(position);
  angle = position;
  Serial.print("angle = ");
  Serial.println(position);
  delay(50);  // Delay for smoother servo movement
}

void ultrasonicRadar() {
  dis_1 = sonar[0].ping_cm();
  dis_2 = sonar[1].ping_cm();
  Serial.print("dis_1 = ");
  Serial.println(dis_1);
  Serial.print("dis_2 = ");
  Serial.println(dis_2);
}

void radar() {
  // Implement radar logic if needed
  if (dis_2 <= threshold) {
    Serial.print("air defense found something at degree: ");
    Serial.println(angle);
    Serial.print("at dist_1: ");
    Serial.println(dis_1);
    Serial.print("at dist_2: ");
    Serial.println(dis_2);
    //panServoCam.write(panPosition);'
    sweeping = 0;
    RADAR_CAM_postion();
  } else {
    Serial.println("air defense found NOthing");
    sweeping = 1;
  }
}
void RADAR_CAM_postion() {
  int ANGLE = angle;
  if (ANGLE > tiltPositioncam) {
    for (int i = tiltPositioncam; i <= ANGLE; i++) {
      tiltServoCam.write(i);
      delay(20);
    }
  } else {
    for (int i = tiltPositioncam; i >= ANGLE; i--) {
      tiltServoCam.write(i);
      delay(20);
    }
  }
  delay(5000);
  while (data != "None") {
  }
}