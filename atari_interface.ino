char command;
int duration;

void setup() {
  pinMode(3, OUTPUT);  // Serve
  pinMode(4, OUTPUT);  // Left
  pinMode(5, OUTPUT);  // Right
  Serial.begin(115200);
}

void loop() {
  digitalWrite(5, HIGH);
  digitalWrite(4, HIGH);
  digitalWrite(3, HIGH);
    
  if(Serial.available()) {
    command = Serial.read();
    duration = Serial.parseInt();
    
    if (command == 'R') {
       digitalWrite(5, LOW);
       delay(duration);
       digitalWrite(5, HIGH);
       Serial.write("R");
    }
    else if (command == 'L') {
       digitalWrite(4, LOW);
       delay(duration);
       digitalWrite(4, HIGH);
       Serial.write("L");
    }
    else if (command == 'S') {
       digitalWrite(3, LOW);
       delay(duration);
       digitalWrite(3, HIGH);
       Serial.write("S");
    }
    else {
       Serial.write("X");
    }
  }
}
