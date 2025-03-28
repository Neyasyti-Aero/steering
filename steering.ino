/*   Tested for CALT HAE28 Absolute Rotary Encoder
 *   
 *   Encoder Red (Power +)  <-> Arduino +5V
 *   Encoder Black (GND)    <-> Arduino GND
 *   Encoder Green (Clock)   <-> Arduino Pin 5
 *   Encoder White (DATA)   <-> Arduino Pin 6
 *   Encoder Yellow (CS)    <-> Arduino Pin 7
 */

 #define RAW_RIGHT_ENCODER_VALUE 203
 // nuetral ~120
 #define RAW_LEFT_ENCODER_VALUE 42
 #define RAW_MID_ENCODER_VALUE ((RAW_RIGHT_ENCODER_VALUE + RAW_LEFT_ENCODER_VALUE) / 2)
 #define RAW_AMP_ENCODER_VALUE (RAW_RIGHT_ENCODER_VALUE - RAW_LEFT_ENCODER_VALUE)
 
const int CLOCK_PIN = 13; // Green Pin
const int DATA_PIN = 12; // White Pin
const int CS_PIN = 10; // Yellow Pin
const int BIT_COUNT = 10; // 10 Bit Mode

const int PWM_IN = 14; // PWM IN

const int DRIVER_RE = 4;
const int DRIVER_LE = 5;
const int DRIVER_RPWM = 7;
const int DRIVER_LPWM = 6;

void setup()
{
  //setup our pins
  pinMode(DATA_PIN, INPUT_PULLUP);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  pinMode(PWM_IN, INPUT_PULLUP);
  //give some default values
  digitalWrite(CLOCK_PIN, HIGH);
  digitalWrite(CS_PIN, HIGH);
  Serial.begin(9600);
}

void loop()
{
  uint32_t strizh = readServo(40000);
  int32_t encoder = readPosition();
  uint32_t desired_pos = convertServoToPosition(strizh);
  uint32_t current_pos = encoder;
  if (encoder >= -0.5 || strizh != 0)
  {
    Serial.print("Desired: ");
    Serial.println(desired_pos);
    Serial.print("Current: ");
    Serial.println(current_pos);
    Serial.println("\n");
  }
  else
  {
    Serial.print(".");
  }
  delay(100);
}

// read servo value
uint32_t readServo(uint32_t timeoutMicroseconds)
{
  uint32_t data = 0;
  uint32_t timeus = 0;

  // skip current 1 state
  while (digitalRead(PWM_IN))
  {
    delayMicroseconds(1);
    timeus++;
    if (timeus > timeoutMicroseconds)
      return 0;
  }

  // wait for new 1 state
  while (digitalRead(PWM_IN) == 0)
  {
    delayMicroseconds(1);
    timeus++;
    if (timeus > timeoutMicroseconds)
      return 0;
  }

  // read
  while (digitalRead(PWM_IN))
  {
    delayMicroseconds(3);
    data++;
  }

  return 25*data/4;
}

// param: servo microseconds value (from ~1000 to ~2000)
// return value: position from 0 to 10000
uint32_t convertServoToPosition(uint32_t servous)
{
  if (servous < 500 || servous > 2500)
    return 5000;
  
  if (servous <= 1000)
    return 0;
  else if (servous >= 2000)
    return 10000;
  
  return (servous - 1000) * 10;
}

//read the current angular position from 0 to 10000 (from max left to max right)
// 5000 => neutral
int32_t readPosition()
{
  // Read the same position data twice to check for errors
  uint32_t sample1 = shiftIn(DATA_PIN, CLOCK_PIN, CS_PIN, BIT_COUNT);
  uint32_t sample2 = shiftIn(DATA_PIN, CLOCK_PIN, CS_PIN, BIT_COUNT);

  delayMicroseconds(50); // Clock must be high for 20 microseconds before a new sample can be taken

  if (sample1 != sample2)
  {
    return -1.0;
  }

  Serial.print("Raw encoder data: ");
  Serial.println(sample1);

  if (sample1 > RAW_RIGHT_ENCODER_VALUE)
    sample1 = RAW_RIGHT_ENCODER_VALUE;
  else if (sample1 < RAW_LEFT_ENCODER_VALUE)
    sample1 = RAW_LEFT_ENCODER_VALUE;
  
  int32_t retval = (((sample1 & 0x0FFF) - RAW_LEFT_ENCODER_VALUE) * 10000) / RAW_AMP_ENCODER_VALUE;

  // clamp to 0...10000
  if (retval > 10000)
    retval = 10000;
  else if (retval < 0)
    retval = 0;

  return retval;
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