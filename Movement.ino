#define M1A 7
#define M1B 6
#define M2A 10
#define M2B 9

void setup() {
  // put your setup code here, to run once:
  pinMode(M1A, OUTPUT);
  pinMode(M2A, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2B, OUTPUT);
}

void loop() {

  // Serial.println("Testing Forward (fast decay): ");
  // analogWrite(M1A, 200);
  // analogWrite(M1B, 0);

  // analogWrite(M2A, 200);
  // analogWrite(M2B, 0);

  // Serial.println("Testing Backwards (fast decay): ");

  // analogWrite(M1A, 0);
  // analogWrite(M1B, 200);
  
  // analogWrite(M2A, 0);
  // analogWrite(M2B, 200);


  // Serial.println("Testing Backwards (slow decay): ");

  // analogWrite(M1A, 255);
  // analogWrite(M1B, 127);
  
  // analogWrite(M2A, 255);
  // analogWrite(M2B, 127);

  Serial.println("Testing Backwards (slow decay): ");
  analogWrite(M1A, 127);
  analogWrite(M1B, 255);

  analogWrite(M2A, 127);
  analogWrite(M2B, 255);

}
