#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

// Network settings
const char* ssid = "Ghuzy";
const char* password = "01066709440@AHMAG";

// Motor Pin Definitions
const int motor1Pin1 = 27;
const int motor1Pin2 = 26;
const int motor2Pin1 = 25;
const int motor2Pin2 = 33;

// Servo Pin Definitions
const int panServoPin = 14;
const int tiltServoPin = 12;

// Servo objects
Servo panServo;
Servo tiltServo;

// Sweep state variables
bool sweepHorizontal = false;
bool sweepVertical = false;
int panPosition = 90; // Initial middle position
int tiltPosition = 90; // Initial middle position
int panStep = 1;  // Step size for pan servo
int tiltStep = 1; // Step size for tilt servo

// Web server running on port 80
WebServer server(80);

void setup() {
  Serial.begin(115200);
  setupMotors();
  setupServos();
  setupWiFi();
  setupServer();
  Serial.print("Got IP: ");  Serial.println(WiFi.localIP());
}

void setupMotors() {
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
}

void setupServos() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  panServo.attach(panServoPin, 500, 2500);
  tiltServo.attach(tiltServoPin, 500, 2500);
  panServo.write(panPosition);
  tiltServo.write(tiltPosition);
}

void setupWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi..");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to the WiFi network");
}

void setupServer() {
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", controlPage());
  });

  server.on("/control", HTTP_GET, []() {
    String command = server.arg("command");
    handleCommand(command);
    server.send(204); // No content to send back
  });

  server.begin();
}

String controlPage() {
  String html = "<html><head><style>";
  html += "button { background-color: #4CAF50; color: white; border: none; padding: 15px 32px; text-align: center;";
  html += "text-decoration: none; display: inline-block; font-size: 16px; margin: 4px 2px; cursor: pointer;}";
  html += "button:hover { background-color: #45a049;}";
  html += "</style></head><body><h1>Motor & Servo Control</h1>";
  html += "<p>Use the buttons or WASD keys on your keyboard for motors, arrow keys for servos.</p>";
  html += "<button onclick=\"sendCommand('forward')\">Forward</button>";
  html += "<button onclick=\"sendCommand('left')\">Left</button>";
  html += "<button onclick=\"sendCommand('right')\">Right</button>";
  html += "<button onclick=\"sendCommand('reverse')\">Reverse</button>";
  html += "<button onclick=\"sendCommand('stop')\" style='background-color: #f44336;'>Stop</button><br>";
  html += "<button onclick=\"sendCommand('panLeft')\">Pan Left</button>";
  html += "<button onclick=\"sendCommand('panRight')\">Pan Right</button>";
  html += "<button onclick=\"sendCommand('tiltUp')\">Tilt Up</button>";
  html += "<button onclick=\"sendCommand('tiltDown')\">Tilt Down</button><br>";
  html += "<button onclick=\"sendCommand('neutral')\">Reset Position</button>";
  html += "<button onclick=\"sendCommand('sweepHorizontal')\">Sweep Horizontal</button>";
  html += "<button onclick=\"sendCommand('sweepVertical')\">Sweep Vertical</button><br>";
  html += "<script>";
  html += "document.body.addEventListener('keydown', function(event) {";
  html += "  const key = event.key.toLowerCase();";
  html += "  if(key === 'w') sendCommand('forward');";
  html += "  else if(key === 's') sendCommand('reverse');";
  html += "  else if(key === 'a') sendCommand('left');";
  html += "  else if(key === 'd') sendCommand('right');";
  html += "  else if(key === 'x') sendCommand('stop');";
  html += "  else if(key === 'arrowleft') sendCommand('panLeft');";
  html += "  else if(key === 'arrowright') sendCommand('panRight');";
  html += "  else if(key === 'arrowup') sendCommand('tiltUp');";
  html += "  else if(key === 'arrowdown') sendCommand('tiltDown');";
  html += "});";
  html += "function sendCommand(command) {";
  html += "  fetch(`/control?command=${command}`);";
  html += "}";
  html += "</script>";
  html += "</body></html>";
  return html;
}

void handleCommand(String command) {
  Serial.println(command);
  if (command == "stop") {
    sweepHorizontal = false;
    sweepVertical = false;
  } else if (command == "sweepHorizontal") {
    sweepHorizontal = !sweepHorizontal; // Toggle sweeping
  } else if (command == "sweepVertical") {
    sweepVertical = !sweepVertical; // Toggle sweeping
  } else if (command == "neutral") {
    panServo.write(90);
    tiltServo.write(90);
  } else {
    manualControl(command);
  }
}

void manualControl(String command) {
  int position;
  if (command == "panLeft") {
    position = max(0, panServo.read() - 5);
    panServo.write(position);
  } else if (command == "panRight") {
    position = min(180, panServo.read() + 5);
    panServo.write(position);
  } else if (command == "tiltUp") {
    position = min(180, tiltServo.read() + 5);
    tiltServo.write(position);
  } else if (command == "tiltDown") {
    position = max(0, tiltServo.read() - 5);
    tiltServo.write(position);
  }
}

void loop() {
  if (sweepHorizontal) {
    sweepServo(panServo, panPosition, panStep, 0, 180);
  }
  if (sweepVertical) {
    sweepServo(tiltServo, tiltPosition, tiltStep, 0, 180);
  }
  server.handleClient();
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
