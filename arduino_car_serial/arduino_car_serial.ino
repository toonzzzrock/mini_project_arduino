// Motor pins
const int LF  = 3,  LB  = 4;
const int RF  = 6,  RB  = 5;
const int BLF = 8,  BLB = 9;
const int BRF = 10, BRB = 11;

const int S = 200;  // base speed

int isOpen = 0, angle = 0;

// buffer for incoming bytes
char buf[64];
uint8_t idx = 0;

void setup() {
  Serial.begin(9600);  // RX from ESP8266
  Serial.println("booted arduino");
  // motor pins
  int pins[] = {LF, LB, RF, RB, BLF, BLB, BRF, BRB};
  for (int i = 0; i < 8; i++) pinMode(pins[i], OUTPUT);
}

void setMotor(int fwdPin, int revPin, int speed) {
  if (speed >= 0) {
    analogWrite(fwdPin, constrain(speed, 0, 255));
    analogWrite(revPin, 0);
  } else {
    analogWrite(fwdPin, 0);
    analogWrite(revPin, constrain(-speed, 0, 255));
  }
}

void drive(int leftSpeed, int rightSpeed) {
  setMotor(LF, LB, leftSpeed);
  setMotor(BLF, BLB, leftSpeed);
  setMotor(RF, RB, rightSpeed);
  setMotor(BRF, BRB, rightSpeed);
}

void loop() {
  // read bytes until newline
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      buf[idx] = '\0';          // terminate
      String line = String(buf);
      line.trim();
      idx = 0;                  // reset for next
      Serial.println(line);

      // parse "isOpen,angle"
      int comma = line.indexOf(',');
      if (comma >= 0) {
        isOpen = line.substring(0, comma).toInt();
        angle  = line.substring(comma + 1).toInt();
      }

      // movement logic
      if (isOpen == 0) {
        drive(0, 0);
      } else {
        angle = constrain(angle, -180, 180);
        float rad = radians(angle);
        // Forward/backward with differential for small turning
        int base = S * cos(rad);
        int delta = S * sin(rad);  // turning adjustment

        int leftSpeed  = constrain(base + delta, -255, 255);
        int rightSpeed = constrain(base - delta, -255, 255);

        drive(leftSpeed, rightSpeed);

      }


    } else if (idx < sizeof(buf) - 1) {
      buf[idx++] = c;
    } else {
      // overflow guard
      idx = 0;
    }
  }
}
