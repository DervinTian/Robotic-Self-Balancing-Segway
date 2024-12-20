// Define the pins for the encoder
#define ENCODER_A 2
#define ENCODER_B 3
volatile uint8_t encoderALast;
volatile uint8_t encoderBLast;

// Define constants
const float WHEEL_RADIUS = 0.045;  // Radius of the wheel in meters
const float GEAR_RATIO = 50.0;  // Gear ratio of the encoder
const int ENCODER_PULSE_PER_REVOLUTION = 28;  // Number of encoder pulses per revolution
const float COUNTS_PER_REVOLUTION = ENCODER_PULSE_PER_REVOLUTION * GEAR_RATIO; // Pulses per revolution
// Define the variables for the encoder
volatile int16_t encoder_countsAB = 0;
volatile int16_t encoder_countsCD = 0;

volatile unsigned long last_time = 0;
volatile float position_change = 0.0;
volatile float angular_velocity = 0.0;
volatile float angle = 0.0;
void setup() {
  // Set the encoder pins as inputs and enable pull-up resistors
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  // Attach interrupts to the encoder pins
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoder_isr, CHANGE);
  // Initialize serial communication
  Serial.begin(9600);
}

void loop() {
  // Calculate the elapsed time since the last encoder pulse
  unsigned long current_time = micros();
   // Calculate the time difference since the last interrupt
  float dt_interrupt = (float)(current_time - last_time) / 1000000;
  // Calculate the change in the position of the encoder in radians
  position_change = 2 * PI * encoder_counts / COUNTS_PER_REVOLUTION;
  // Reset encoder counts
  encoder_counts = 0;
  // Calculate the angular velocity
  angular_velocity = position_change / dt_interrupt;
  // Calculate the angle
  angle += position_change;
  // Print results
  Serial.print("Angular velocity: ");
  Serial.print(angular_velocity);
  Serial.print(" rad/s, Angle: ");
  Serial.print(angle);
  Serial.println(" radians");
  // Update last time
  last_time = current_time;
}

void encoder_isr() {
  uint8_t encoderAState = digitalRead(ENCODER_A);
  uint8_t encoderBState = digitalRead(ENCODER_B);

  if (encoderALast*encoderBLast) { //State 11
    encoder_counts -= (encoderAState - encoderBState); 
  } else if (encoderALast && (encoderALast^encoderBLast)) { //State 10
    encoder_counts += encoderAState*encoderBState - (1-encoderAState|encoderBState); 
  } else if (encoderBLast && (encoderBLast^encoderALast)) { //State 01
    encoder_counts += (1-encoderAState|encoderBState) - encoderAState*encoderBState;
  } else { //State 00
    encoder_counts += encoderAState - encoderBState; 
  }

  encoderALast = encoderAState;
  encoderBLast = encoderBState;
}
