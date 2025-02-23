#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include "index.h"
#include <ESP32Servo.h>


////////////////////////////////////////////
#define CMD_STOP 0
#define CMD_FORWARD 10
#define CMD_BACKWARD 30
#define CMD_LEFT 20
#define CMD_RIGHT 40

//#define ENA_PIN 14  // The ESP32 pin GPIO14 connected to the ENA pin L298N
#define IN1_PIN 23  // The ESP32 pin GPIO27 connected to the IN1 pin L298N
#define IN2_PIN 22  // The ESP32 pin GPIO26 connected to the IN2 pin L298N
#define IN3_PIN 21  // The ESP32 pin GPIO25 connected to the IN3 pin L298N
#define IN4_PIN 19  // The ESP32 pin GPIO33 connected to the IN4 pin L298N
//#define ENB_PIN 32  // The ESP32 pin GPIO32 connected to the ENB pin L298N

//////////////////////////////////////////////////////
// Servo Pin Definitions
#define panServoPin  18
#define tiltServoPin 17

// Servo objects
Servo panServo;
Servo tiltServo;

// Sweep state variables
//bool sweepHorizontal = false;
//bool sweepVertical = false;
int panPosition = 180; // Initial middle position
int tiltPosition = 90; // Initial middle position
int panStep = 1;  // Step size for pan servo
int tiltStep = 1; // Step size for tilt servo
/////////////////////////
const char* ssid = "Ghuzy";     // CHANGE IT
const char* password = "01066709440@AHMAG";  // CHANGE IT

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
      switch (command) {
        case CMD_STOP:
          Serial.println("Stop");
          CAR_stop();
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
        default:
          Serial.println("Unknown command");
      }
      
      break;
  }
}
void setupServos() {
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
  panServo.setPeriodHertz(50);    // standard 50 hz servo
  tiltServo.setPeriodHertz(50);    // standard 50 hz servo
  panServo.attach(panServoPin, 500, 2500);
  tiltServo.attach(tiltServoPin, 500, 2500);
  panServo.write(panPosition);
  tiltServo.write(tiltPosition);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  setupServos();
    //pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  //pinMode(ENB_PIN, OUTPUT);

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
}

void loop() {
  // put your main code here, to run repeatedly:
  sweepServo(panServo, panPosition, panStep, 90, 180);
  sweepServo(tiltServo, tiltPosition, tiltStep, 0, 180);
  webSocket.loop();
  //SWEEP_RADAR();
}

void sweepServo(Servo& servo, int& position, int& step, int minPos, int maxPos) {
  position += step;
  if (position <= minPos || position >= maxPos) {
    step = -step; // Reverse direction at the boundaries
    position += step; // Correct for overshoot
  }
  servo.write(position);
  delay(20); // Delay for a smoother movement
}

/*void SWEEP_RADAR(){
  for(int pos_tilt=0; pos_tilt<=180; pos_tilt+=20){
    panServo.write(pos_tilt);
    for(int pos_pan=180; pos_pan>=90; pos_pan-=5){
      tiltServo.write(pos_pan);
      //delay(5);
    }
    for(int pos_pan=90; pos_pan<=180; pos_pan +=5){
      tiltServo.write(pos_pan);
      //delay(5);
    }
  }
  for(int pos_tilt=180; pos_tilt>=0; pos_tilt-=20){
    panServo.write(pos_tilt);
    for(int pos_pan=180; pos_pan>=90; pos_pan-=5){
      tiltServo.write(pos_pan);
      //delay(5);
    }
    for(int pos_pan=90; pos_pan<=180; pos_pan +=5){
      tiltServo.write(pos_pan);
      //delay(5);
    }
  }
}*/

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