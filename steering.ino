#include "ServoInput/ServoInput_impl.h"

#define OUTPUT_LIMIT 0.8
#define OUTPUT_LIMIT_LOW 0.1

#define RAW_LEAST_SIGNIFICANT_DELTA 5
#define RAW_MID_ENCODER_VALUE 120
#define RAW_AMP_ENCODER_VALUE 120
#define RAW_RIGHT_ENCODER_VALUE RAW_MID_ENCODER_VALUE + (RAW_AMP_ENCODER_VALUE / 2)
#define RAW_LEFT_ENCODER_VALUE RAW_MID_ENCODER_VALUE - (RAW_AMP_ENCODER_VALUE / 2)

#if RAW_LEFT_ENCODER_VALUE < 0
#error Too much left (ZCP)
#elif RAW_RIGHT_ENCODER_VALUE > 1023
#error Too much right (ZCP)
#endif

#if RAW_LEAST_SIGNIFICANT_DELTA > RAW_AMP_ENCODER_VALUE
#error Too much delta
#endif

const int CLOCK_PIN = 8; // Green Pin
const int DATA_PIN = 7; // White Pin
const int CS_PIN = 6; // Yellow Pin
const int BIT_COUNT = 10; // 10 Bit Mode

const int DRIVER_RE = 4;
const int DRIVER_LE = 5;
const int DRIVER_RPWM = 9;
const int DRIVER_LPWM = 10;

// Steering Setup
const int SteeringSignalPin = 3;   // MUST be interrupt-capable!
const int SteeringPulseMin = 1000;  // microseconds (us)
const int SteeringPulseMax = 2000;  // Ideal values for your servo can be found with the "Calibration" example
ServoInputPin<SteeringSignalPin> steering(SteeringPulseMin, SteeringPulseMax);

void setup()
{
  // motor setup
  pinMode(DRIVER_RE, OUTPUT);
  pinMode(DRIVER_LE, OUTPUT);
  digitalWrite(DRIVER_RE, LOW);
  digitalWrite(DRIVER_LE, LOW);
  motor_setup();

  // setup encoder pins
  pinMode(DATA_PIN, INPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CLOCK_PIN, HIGH);
  digitalWrite(CS_PIN, HIGH);

  // pwm input
  steering.attach();

  Serial.begin(9600);

  delay(1000);
}

void loop()
{
  bool failure = false;

  uint32_t strizh = 0;
  if (steering.available())
  {
    strizh = steering.getPulse();
  }
  else
  {
    failure = true;
  }
  
  int32_t encoder = readPosition();
  int32_t desired_pos = convertServoToPosition(strizh);
  int32_t current_pos = encoder;

  if (encoder < 0)
  {
    Serial.print("No encoder!\r\n");
    failure = true;
  }
  else
  {
    Serial.print("Current pos: ");
    Serial.println(current_pos);
  }

  if (strizh == 0)
  {
    Serial.print("No flight controller!\r\n");
    failure = true;
  }
  else
  {
    Serial.print("Desired value: ");
    Serial.println(desired_pos);
  }

  if (failure)
  {
    motor_setPWM(0.0);
    Serial.println("FAILURE!");
    return;
  }

  // Everything is OK - Let's spin the motor
  if (abs(current_pos - desired_pos) > RAW_LEAST_SIGNIFICANT_DELTA)
  {
    // motor needs to be spinned
    if (desired_pos - current_pos < 0)
    {
      // needs left spin
      Serial.println("LEFT SPIN!");
      motor_setPWM(-OUTPUT_LIMIT);
    }
    else if (desired_pos - current_pos > 0)
    {
      // needs right spin
      Serial.println("RIGHT SPIN!");
      motor_setPWM(OUTPUT_LIMIT);
    }
  }
  else
  {
    // don't spin
    Serial.println("NO SPIN!");
    motor_setPWM(0.0);
  }


  Serial.println("\r\n");
}

// param: servo microseconds value (from ~1000 to ~2000)
// return value: position from RAW_LEFT_ENCODER_VALUE to RAW_RIGHT_ENCODER_VALUE
uint32_t convertServoToPosition(uint32_t servous)
{
  if (servous < 500 || servous > 2500)
  {
    return RAW_MID_ENCODER_VALUE;  
  }

  if (servous <= 1000)
  {
    return RAW_LEFT_ENCODER_VALUE;
  }
  else if (servous >= 2000)
  {
    return RAW_RIGHT_ENCODER_VALUE;
  }

  uint32_t retval = round(RAW_LEFT_ENCODER_VALUE + (servous - 1000.0) * RAW_AMP_ENCODER_VALUE / 1000.0);

  if (retval <= RAW_LEFT_ENCODER_VALUE)
  {
    retval = RAW_LEFT_ENCODER_VALUE + 1;
  }
  else if (retval >= RAW_RIGHT_ENCODER_VALUE)
  {
    retval = RAW_RIGHT_ENCODER_VALUE - 1;
  }

  return retval;
}

//read the current angular position from RAW_LEFT_ENCODER_VALUE to RAW_RIGHT_ENCODER_VALUE
int32_t readPosition()
{
  // Read the same position data twice to check for errors
  uint32_t sample1 = shiftIn(DATA_PIN, CLOCK_PIN, CS_PIN, BIT_COUNT);
  uint32_t sample2 = shiftIn(DATA_PIN, CLOCK_PIN, CS_PIN, BIT_COUNT);

  delayMicroseconds(20); // Clock must be high for 20 microseconds before a new sample can be taken

  const unsigned long c_errorThreshold = 20;

  if (sample1 == 0 || sample2 == 0) return -1.0;

  if (sample1 > sample2) {
    if (sample1 - sample2 > c_errorThreshold)
      return -1.0;
  }

  if (sample1 < sample2) {
    if (sample2 - sample1 > c_errorThreshold)
      return -1.0;
  }

  sample1 = int32_t((sample1 + sample2) * 0.5);

  Serial.print("Raw encoder data: ");
  Serial.println(sample1);

  if (sample1 > RAW_RIGHT_ENCODER_VALUE)
    sample1 = RAW_RIGHT_ENCODER_VALUE;
  else if (sample1 < RAW_LEFT_ENCODER_VALUE)
    sample1 = RAW_LEFT_ENCODER_VALUE;

  return sample1;
}

//read in a byte of data from the digital input of the board.
unsigned long shiftIn(const int data_pin, const int clock_pin, const int cs_pin, const int bit_count)
{
  unsigned long data = 0;

  digitalWrite(cs_pin, LOW);
  for (int i = 0; i < bit_count; i++)
  {
    data <<= 1;
    digitalWrite(clock_pin, LOW);
    delayMicroseconds(1);
    digitalWrite(clock_pin, HIGH);
    delayMicroseconds(1);

    data |= digitalRead(data_pin);
  }
  digitalWrite(cs_pin, HIGH);
  return data;
}

const uint16_t TIMER1_PERIOD = 512;
void motor_setPWM(float value)
{
  if (value > OUTPUT_LIMIT) value = OUTPUT_LIMIT;
  if (value < -OUTPUT_LIMIT) value = -OUTPUT_LIMIT;

  if (value > 0 && value < OUTPUT_LIMIT_LOW)
    value = 0;

  if (value < 0 && value > -OUTPUT_LIMIT_LOW)
    value = 0;

  TCCR1A = 0 | _BV(COM1A1) | _BV(COM1B1);

  if (value > OUTPUT_LIMIT_LOW)
  {
    OCR1B = 0;
    digitalWrite(DRIVER_LE, HIGH);
    OCR1A = uint16_t(value * TIMER1_PERIOD);
    digitalWrite(DRIVER_RE, HIGH);
  }
  else if (value < -OUTPUT_LIMIT_LOW)
  {
    OCR1A = 0;
    digitalWrite(DRIVER_RE, HIGH);
    OCR1B = uint16_t(-value * TIMER1_PERIOD);
    digitalWrite(DRIVER_LE, HIGH);
  }
  else
  {
    digitalWrite(DRIVER_RE, LOW);
    digitalWrite(DRIVER_LE, LOW);
    OCR1A = 0;
    OCR1B = 0;
  }
}

void motor_setup()
{
  // Set pins low by default
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);

  // Stop timer before configuring
  TCCR1B = 0;

  // 16.11.1 TCCR1A – Timer/Counter1 Control Register A
  TCCR1A = 0 | _BV(COM1A1) | _BV(COM1B1);
  TCCR1B = _BV(WGM13); // set mode as phase and frequency correct pwm

  // Set TOP value
  ICR1 = TIMER1_PERIOD;

  motor_setPWM(0.0);

  // 14.4.3 DDRB – The Port B Data Direction Register
  DDRB =
    0
    | (1 << DDB1)  // PB1 (aka OC1A) as output - pin 9 on Arduino Uno
    | (1 << DDB2)  // PB2 (aka OC1B) as output - pin 10 on Arduino Uno
    ;

  // Start the timer with no prescaler
  TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
}