#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <PID_v1.h>
#include <ESP32Encoder.h>

// which analog pin to connect
#define THERMISTORPIN 4     
#define SSR_PIN 2
// resistance at 25 degrees C
#define THERMISTORNOMINAL 100000      
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25   
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 10
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
#define SERIESRESISTOR 99000
// how many speps per degree
#define ENCODER_RESOLUTION 2.0
int samples[NUMSAMPLES];

const static float CALIBRATION = -2.3;

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);

ESP32Encoder encoder;

//Define Variables we'll be connecting to
// output = active time of SSR / 10ms
// 1 -> 10ms = 1 zero crossing
double setpoint, input, output;
//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
// how long is one period of SSR control in ms
int WindowSize = 500;
unsigned long windowStartTime;
//timer stuff
//hw_timer_t *timer = NULL;
//portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
//volatile uint8_t interruptCounter = 0;

// update the display output
void updateDisplay() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.print(input);
  display.println(F(" C"));
  display.print(F("set:"));
  display.println(setpoint);
  display.print(F("PID:"));
  display.print(output);
  display.display();
}

//// set the output appropriately
//void IRAM_ATTR onTimer() {
////  portENTER_CRITICAL_ISR(&timerMux);
////  interruptCounter++;
////  portEXIT_CRITICAL_ISR(&timerMux);
//  if interruptCounter == threshold {
//    
//  }
// 
//}

void setup(void) {
  
  Serial.begin(115200);
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  pinMode(SSR_PIN, OUTPUT);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    // also probably set output low
    digitalWrite(SSR_PIN, LOW);
    for(;;); // Don't proceed, loop forever
  }
  display.display();
  delay(500);
  display.clearDisplay();
  display.setTextSize(2);             
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  // set initial PID values
  input = 25.0;
  setpoint = 105.0;
  myPID.SetOutputLimits(0, 100);
  myPID.SetMode(AUTOMATIC);
  analogSetAttenuation(ADC_11db);
  //start the timer (timer freq = 80Mhz -> 2nd arg prescaler)
  //1us per timer tick
//  timer = timerBegin(0, 80, true);
//  timerAttachInterrupt(timer, &onTimer, true);
//  // alarm freq = 100hz
//  timerAlarmWrite(timer, 10000, true);
//  timerAlarmEnable(timer);

  // encoder stuff
  // Enable the weak pull up resistors
  ESP32Encoder::useInternalWeakPullResistors=UP;
  // Attache pins for use as encoder pins
  encoder.attachHalfQuad(19, 18);
  encoder.setCount(105 * ENCODER_RESOLUTION);
}
 
void loop(void) {

  // measure temp
  uint8_t i;
  float average;
 
  // take N samples in a row, with a slight delay
  for (i=0; i< NUMSAMPLES; i++) {
   samples[i] = analogRead(THERMISTORPIN);
  }
  // average all the samples out
  average = 0;
  for (i=0; i< NUMSAMPLES; i++) {
     average += samples[i];
  }
  average /= NUMSAMPLES;
  // convert the value to resistance
  average = 4095 / average - 1;
  average = SERIESRESISTOR / average;
  // calculate actual temp
  float steinhart;
  steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;           // convert absolute temp to C
  steinhart += CALIBRATION;
  // do the update
  setpoint = ((float)encoder.getCount())/ENCODER_RESOLUTION;
  input = steinhart;
  myPID.Compute();
  if (millis() % 20 == 0){
    updateDisplay();
  }
  if (millis() - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
//  Serial.println("setpoint*(WindowSize/100)="+String(setpoint*(WindowSize/100))+"millis-windowstarttime="+String(millis() - windowStartTime));
  if (millis() - windowStartTime < output * (WindowSize/100)) digitalWrite(SSR_PIN, HIGH);
  else digitalWrite(SSR_PIN, LOW);
}
