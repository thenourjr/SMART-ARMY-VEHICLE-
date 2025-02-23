#define inductive_1 39
#define inductive_2 36
#define buzzer 14

void setup() {
    pinMode(inductive_1, INPUT);
    pinMode(inductive_2, INPUT);
    pinMode(buzzer, OUTPUT);
    Serial.begin(115200);
}
 
void loop() {
    int sensorValue_1 = digitalRead(inductive_1);
    int sensorValue_2 = digitalRead(inductive_2);
    if(sensorValue_1== 0 || sensorValue_2 == 0){ 
        Serial.println("Object");
        digitalWrite(buzzer , HIGH);
        delay(500);
    }
    else {
      Serial.println("nothing detected");
      digitalWrite(buzzer , HIGH);
    }
  }