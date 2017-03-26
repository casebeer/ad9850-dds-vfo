#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>

#define DEBUG

//#include <ClickEncoder.h>
//#include <TimerOne.h>

#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

#define ACCELERATION 1.2
#define DECELERATION .9999
#define MAX_ACCELERATION 100000

#define FREQUENCY_MAX 50000000

LiquidCrystal_PCF8574 lcd(0x27);  // set the LCD address to 0x27 for a 16 chars and 2 line display
Encoder myEnc(2, 3);

//ClickEncoder *encoder;
int32_t newValue;
double acceleration = 1.0;
uint32_t value;

void timerIsr() {
  //encoder->service();
}


void setup() {
  int error;
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("LCD...");

  value = 0;
  newValue = 0;

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
}

void loop() {
  char buf[31];
  
  newValue = myEnc.read();
  myEnc.write(0);

  if (newValue > 1 || newValue < -1) {
    newValue = 0;
  }
  
  if ( newValue != 0 ) { //newValue != value) {
    value += newValue * int32_t(acceleration);
    if (value > FREQUENCY_MAX) {
      value = (FREQUENCY_MAX + newValue * int32_t(acceleration)) % FREQUENCY_MAX;
    }
    format_frequency(buf, 31, value);
    lcd.setCursor(0,1);
    lcd.write(buf);

#ifdef DEBUG
  
    Serial.print("Acceleration: ");
    Serial.print(acceleration);
    Serial.print("\tnewValue (raw): ");
    Serial.print(newValue);
    Serial.print("\tnewValue (accelerated): ");
    Serial.print(newValue * int32_t(acceleration));
    Serial.print("\tvalue: ");
    Serial.print(value);
    Serial.println("");
  
#endif
  }
  acceleration *= newValue == 0 ? DECELERATION : ACCELERATION;
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
