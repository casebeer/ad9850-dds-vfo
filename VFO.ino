#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>
#include <AD9850.h>

#define DEBUG

//#include <ClickEncoder.h>
//#include <TimerOne.h>

#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

#define ACCELERATION 1.2
#define DECELERATION .99
#define MAX_ACCELERATION 100000
#define QUIESCENCE_MAX 2500

#define FREQUENCY_MAX 50000000

#define W_CLK_PIN 5
#define FQ_UD_PIN 6
#define DATA_PIN 7
#define RESET_PIN 8
#define DDS_CLOCK_FREQ 125000000

LiquidCrystal_PCF8574 lcd(0x27);  // set the LCD address to 0x27 for a 16 chars and 2 line display
Encoder myEnc(2, 3);

double acceleration = 1.0;
uint32_t frequency;
int8_t encoderDirection;

void setup() {
  int error;
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("LCD...");

  frequency = 440;

  //encoder = new ClickEncoder(2, 3);
  //encoder->setAccelerationEnabled(true);
  
  // put your main code here, to run repeatedly:
  while (! Serial);

  Serial.println("Dose: check for LCD");

  // See http://playground.arduino.cc/Main/I2cScanner
  Wire.begin();
  Wire.beginTransmission(0x27);
  error = Wire.endTransmission();
  Serial.print("Error: ");
  Serial.print(error);

  if (error == 0) {
    Serial.println(": LCD found.");

  } else {
    Serial.println(": LCD not found.");
  } // if

  lcd.begin(16, 2); // initialize the lcd
  lcd.setBacklight(64);
  lcd.home();
  lcd.clear();
  lcd.print(" AD9850 DDS VFO");

  DDS.begin(W_CLK_PIN, FQ_UD_PIN, DATA_PIN, RESET_PIN);
  DDS.calibrate(DDS_CLOCK_FREQ);

  setFrequency(frequency);
}

void setFrequency(uint32_t f) {
  char buf[31];
  format_frequency(buf, 31, f);
  lcd.setCursor(0,1);
  lcd.write(buf);
  DDS.setfreq(f, 0);
}

uint32_t timer = QUIESCENCE_MAX;
double timerSmooth = QUIESCENCE_MAX;
void loop() {
  int32_t encoderValue;
  timer++;
  
  encoderValue = myEnc.read();
  myEnc.write(0);

  //if (encoderValue != 0 )
  //Serial.println(encoderValue);

  if (encoderValue > 1 || encoderValue < -1) {
    encoderValue = 0;
  }

  // encoder debouncing; ignore inputs for 10% of the moving average inter-input time
  if (timer < timerSmooth / 10 && timer < QUIESCENCE_MAX) {
    return;
  }
  if ( encoderValue != 0) {
    timerSmooth = .2 * timer + (1-.2) * timerSmooth;
    
    if (encoderValue != encoderDirection) {
      // reset acceleration when changing direction
      //acceleration *= .1;
    }
    frequency += encoderValue * int32_t(acceleration);
    if (frequency > FREQUENCY_MAX) {
      frequency = (FREQUENCY_MAX + encoderValue * int32_t(acceleration)) % FREQUENCY_MAX;
    }

    setFrequency(frequency);

    // -1 or +1
    encoderDirection = encoderValue;
#ifdef DEBUG
  
    Serial.print("Accel: ");
    Serial.print(acceleration);
    Serial.print("\tEnc: ");
    Serial.print(encoderValue);
    Serial.print("\tLoops: ");
    Serial.print(timer);
    Serial.print("\t");
    Serial.print(timerSmooth);
    Serial.print("\n");
                
    /*
    Serial.print(encoderValue);
    Serial.print("\tnewValue (accelerated): ");
    Serial.print(encoderValue * int32_t(acceleration));
    Serial.print("\tfrequency: ");
    Serial.print(frequency);
    Serial.println("");
    */
#endif
    timer = 0;


  }
  //acceleration *= encoderValue == 0 ? DECELERATION : ACCELERATION;
  double ts = timerSmooth / 100., 
    ts3 = ts * ts * ts;
  acceleration = double(100000.0) / (ts3 * ts3 * ts);
  if (acceleration > 1) {
    //Serial.print("Accleration: ");
    //Serial.print(acceleration);
    //Serial.print("\n");
  }
  if (acceleration < 1) {
    acceleration = 1;
  } else if (acceleration > MAX_ACCELERATION) {
    acceleration = MAX_ACCELERATION;
  }
  
  
  
}

#define Hz(f) ((uint16_t)(f % 1000))
#define kHz(f) ((uint16_t)((f % 1000000) / 1000))
#define mHz(f) ((uint16_t)(f / 1000000))

char* format_frequency(char *buf, int len, uint32_t f) {
  if (f < 1000000) {
    if (f < 1000) {
      snprintf(buf, len, " %11d Hz ", Hz(f)); 
    } else {
      snprintf(buf, len, " %7d %03d Hz ", kHz(f), Hz(f)); 
    }
  } else {
    snprintf(buf, len, " %3d %03d %03d Hz ", mHz(f), kHz(f), Hz(f)); 
  }
  return buf;
}
