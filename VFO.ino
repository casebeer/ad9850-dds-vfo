#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>
#include <AD9850.h>

#define DEBUG

#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

#define MAX_ACCELERATION 100000
#define QUIESCENCE_MAX 2500

#define FREQUENCY_MAX 50000000

#define W_CLK_PIN 5
#define FQ_UD_PIN 6
#define DATA_PIN 7
#define RESET_PIN 8
#define DDS_CLOCK_FREQ 125000000

#ifdef DEBUG
#define PRINT_DEBUG() {\
  Serial.print("Accel: ");\
  Serial.print(acceleration);\
  Serial.print("\tEnc: ");\
  Serial.print(encoderValue);\
  Serial.print("\tLoops: ");\
  Serial.print(timer);\
  Serial.print("\t");\
  Serial.print(uint32_t(timerSmooth) / 1000);\
  Serial.print("\tRPM ");\
  Serial.print(int(60 * double(1000000.) / (20 * timerSmooth)));\
  Serial.print("\n");\
}
#endif

// set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_PCF8574 lcd(0x27);

// Use hardware interrupt pins D2 and D3 on the Pro Mini
Encoder FrequencyEncoder(2, 3);

double acceleration = 1.0;
uint32_t frequency = 440;
uint32_t timer = QUIESCENCE_MAX;
double timerSmooth = QUIESCENCE_MAX;
unsigned long last_us  = micros();

void setup() {
  int error;
  
  Serial.begin(115200);
  Serial.println("LCD...");
  
  while (! Serial);

  Serial.println("Checking for LCD...");

  // See http://playground.arduino.cc/Main/I2cScanner
  Wire.begin();
  Wire.beginTransmission(0x27);
  error = Wire.endTransmission();
  
  if (error == 0) {
    Serial.println("LCD found.");
  } else {
    Serial.print("Error ");
    Serial.print(error);
    Serial.println(": LCD not found.");
  }

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

inline double calculate_acceleration(double timerSmooth) {
    //double acceleration = double(100000.0) / pow(timerSmooth / 100., 7);
    //double acceleration = double(2142835.0) / timerSmooth - 7142;
    //double acceleration = double(3333300.0) / (timerSmooth / 1000) - 33332;
    double rate = double(1000.0) / timerSmooth;
    double rpm = 60 * double(1000000.) / (20 * timerSmooth);
    double acceleration = exp(0.000996455 * pow(rpm, 2) - 0.016707 * rpm + 0.13860); 
    
    if (acceleration < 1) {
      acceleration = 1;
    } else if (acceleration > MAX_ACCELERATION) {
      acceleration = MAX_ACCELERATION;
    }
    return acceleration;
}

void loop() {
  int32_t encoderValue;

  // increment timer; avoid overflows
  timer = micros() - last_us;
  
  encoderValue = FrequencyEncoder.read();
  FrequencyEncoder.write(0);

  // Ignore inputs with absolute value > 1 
  // We're reading the encoder fast enough that these are all noise
  if (encoderValue > 1 || encoderValue < -1) {
    encoderValue = 0;
  }

  // encoder debouncing; ignore inputs for 10% of the moving average inter-input time
  if (encoderValue != 0) {
      timerSmooth = .1 * timer + (1-.1) * timerSmooth;
      last_us = micros();
  }
  
  if (timer >= timerSmooth / 10 || timer >= QUIESCENCE_MAX) {
      
    // base acceleration on the speed of turning, as determined by debounced
    // inter-encoder-input timings
    acceleration = calculate_acceleration(timerSmooth);
    
    if (encoderValue != 0) {
#ifdef DEBUG
      PRINT_DEBUG();
#endif
      //timer = 0;
      
      frequency += encoderValue * int32_t(acceleration);
  
      if (frequency > FREQUENCY_MAX) {
        frequency = (FREQUENCY_MAX + encoderValue * int32_t(acceleration)) % FREQUENCY_MAX;
      }
  
      setFrequency(frequency);
    }  
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
