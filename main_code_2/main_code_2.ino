#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include "index.h"
////////////////////////////////////////////
#define CMD_STOP 0
#define CMD_FORWARD 10
#define CMD_BACKWARD 30
#define CMD_LEFT 20
#define CMD_RIGHT 40
#define CMD_Cam_left 70
#define CMD_Cam_right 80
#define CMD_Cam_up 50
#define CMD_Cam_down 60
#define shoot 90
#define Realese 110


//#define ENA_PIN 14  // The ESP32 pin GPIO14 connected to the ENA pin L298N
#define IN1_PIN 23  // The ESP32 pin GPIO27 connected to the IN1 pin L298N
#define IN2_PIN 22  // The ESP32 pin GPIO26 connected to the IN2 pin L298N
#define IN3_PIN 21  // The ESP32 pin GPIO25 connected to the IN3 pin L298N
#define IN4_PIN 19  // The ESP32 pin GPIO33 connected to the IN4 pin L298N
//#define ENB_PIN 32  // The ESP32 pin GPIO32 connected to the ENB pin L298N

////////////////////////////////////////////////////////////////////////////////////////////////
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
unsigned long threshold = 25;  // Example threshold for distance
int angle = 0;

bool sweeping;
bool AUTO;
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
///////////////////////////////////
#define inductive_1 39
#define inductive_2 36
#define buzzer 14
//////////////////////////////////////
#define laser_pin 32

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
//////////////////////////////////////////////////////////////////
const char* ssid = "the";                  // CHANGE IT
const char* password = "the2004*#";  // CHANGE IT

AsyncWebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);  // WebSocket server on port 81

void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
      }
      break;
    case WStype_TEXT:
      //Serial.printf("[%u] Received text: %s\n", num, payload);
      String angle = String((char*)payload);
      int command = angle.toInt();
      Serial.print("command: ");
      Serial.println(command);
      if (command % 2 == 0) {
        AUTO = 0;
        switch (command) {
          case CMD_STOP:
            Serial.println("Stop");
            CAR_stop();
            shoot_off();
            break;
          case CMD_FORWARD:
            Serial.println("Move Forward");
            CAR_moveForward();
            break;
          case CMD_BACKWARD:
            Serial.println("Move Backward");
            CAR_moveBackward();
            break;
          case CMD_LEFT:
            Serial.println("Turn Left");
            CAR_turnLeft();
            break;
          case CMD_RIGHT:
            Serial.println("Turn Right");
            CAR_turnRight();
            break;
          case CMD_Cam_left:
            Serial.println("CAM Turn left");
            cam_left();
            break;
          case CMD_Cam_right:
            Serial.println("CAM Turn Right");
            cam_right();
            break;
          case CMD_Cam_up:
            Serial.println("CAM Turn upt");
            cam_up();
            break;
          case CMD_Cam_down:
            Serial.println("CAM Turn down");
            cam_down();
            break;
          case shoot:
            Serial.println("shoot");
            Shoot_on();
            break;
          case Realese:
            Serial.println("Realese");
            break;
          default:
            Serial.println("Unknown command");
        }
      } else {
        AUTO = 1;
        command--;
        switch (command) {
          case CMD_STOP:
            Serial.println("Stop");
            CAR_stop();
            shoot_off();
            break;
          case CMD_FORWARD:
            Serial.println("Move Forward");
            CAR_moveForward();
            break;
          case CMD_BACKWARD:
            Serial.println("Move Backward");
            CAR_moveBackward();
            break;
          case CMD_LEFT:
            Serial.println("Turn Left");
            CAR_turnLeft();
            break;
          case CMD_RIGHT:
            Serial.println("Turn Right");
            CAR_turnRight();
            break;
          case shoot:
            Serial.println("shoot");
            Shoot_on();
            ;
            break;
          case Realese:
            Serial.println("Realese");
            break;
          default:
            Serial.println("Unknown command");
        }
      }
      break;
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  setupServos();
  pinMode(laser_pin, OUTPUT);
  //pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  //pinMode(ENB_PIN, OUTPUT);
  pinMode(inductive_1, INPUT);
  pinMode(inductive_2, INPUT);
  pinMode(buzzer, OUTPUT);
  //digitalWrite(ENA_PIN, HIGH);  // set full speed
  //digitalWrite(ENB_PIN, HIGH);  // set full speed

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Initialize WebSocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  // Serve a basic HTML page with JavaScript to create the WebSocket connection
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    Serial.println("Web Server: received a web page request");
    String html = HTML_CONTENT;  // Use the HTML content from the servo_html.h file
    request->send(200, "text/html", html);
  });

  server.begin();
  Serial.print("ESP32 Web Server's IP address: ");
  Serial.println(WiFi.localIP());
  delay(5000);
}

void loop() {
  // put your main code here, to run repeatedly:
  webSocket.loop();
  mine_detector();
  ultrasonicRadar();
  radar();
  if (sweeping == 1) {
    sweepServo(tiltServo, tiltPosition, tiltStep, 0, 180);
    if (AUTO == 1) {
      sweepServo(tiltServoCam, tiltPosition, tiltStep, 0, 180);
    }
  }
}
void sweepServo(Servo& servo, int& position, int& step, int minPos, int maxPos) {
  position += step;
  if (position <= minPos || position >= maxPos) {
    step = -step;      // Reverse direction at the boundaries
    position += step;  // Correct overshoot
  }
  servo.write(position);
  angle = position;
  /*Serial.print("angle = ");
  Serial.println(position);*/
  delay(50);  // Delay for smoother servo movement
}

void ultrasonicRadar() {
  dis_1 = sonar[0].ping_cm();
  dis_2 = sonar[1].ping_cm();
  /*Serial.print("dis_1 = ");
  Serial.println(dis_1);
  Serial.print("dis_2 = ");
  Serial.println(dis_2);*/
}
void RELEASE() {
}
void Shoot_on() {
  digitalWrite(laser_pin, HIGH);
  Serial.println("laser_on");
}
void shoot_off() {
  digitalWrite(laser_pin, LOW);
  Serial.println("laser_off");
}
void radar() {
  // Implement radar logic if needed
  if (dis_2 <= threshold) {
    /*Serial.print("air defense found something at degree: ");
    Serial.println(angle);
    Serial.print("at dist_1: ");
    Serial.println(dis_1);
    Serial.print("at dist_2: ");
    Serial.println(dis_2);*/
    digitalWrite(buzzer, HIGH);
    delay(100);
    sweeping = 0;
  } else {
    //Serial.println("air defense found NOthing");
    sweeping = 1;
    digitalWrite(buzzer, LOW);
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
}
void cam_home() {
  int ANGLE = angle;
  if (ANGLE > tiltPositioncam) {
    for (int i = ANGLE; i >= tiltPositioncam; i--) {
      tiltServoCam.write(i);
      delay(20);
    }
  } else {
    for (int i = ANGLE; i <= tiltPositioncam; i++) {
      tiltServoCam.write(i);
      delay(20);
    }
  }
}
void mine_detector() {
  int sensorValue_1 = digitalRead(inductive_1);
  int sensorValue_2 = digitalRead(inductive_2);
  if (sensorValue_1 == 0 && sensorValue_2 == 0) {
    Serial.println("Object");
    digitalWrite(buzzer, HIGH);
  } else {
    Serial.println("nothing detected");
    digitalWrite(buzzer, LOW);
  }
}
void CAR_moveForward() {
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);
}

void CAR_moveBackward() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);
}

void CAR_turnLeft() {
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
}

void CAR_turnRight() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);
}

void CAR_stop() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
}
void cam_up() {
  int position;
  position = max(0, panServoCam.read() - 5);
  panServoCam.write(position);
}
void cam_down() {
  int position;
  position = max(0, panServoCam.read() + 5);
  panServoCam.write(position);
}
void cam_left() {
  int position;
  position = max(0, tiltServoCam.read() + 5);
  tiltServoCam.write(position);
}
void cam_right() {
  int position;
  position = max(0, tiltServoCam.read() - 5);
  tiltServoCam.write(position);
}