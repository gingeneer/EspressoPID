#include "EEPROM.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Encoder.h>
#include <PID_v1.h>
#include <SPI.h>
#include <Wire.h>

// which analog pin to connect
#define THERMISTORPIN 4
#define SSR_PIN 2
#define ENCODER_BUTTON_PIN 5
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
#define PID_ENCODER_RESOLUTION 100.0

#define EEPROM_SIZE 16
int samples[NUMSAMPLES];

float calibration = -2.3;

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);

ESP32Encoder encoder;

//Define Variables we'll be connecting to
// output = active time of SSR / 10ms
// 1 -> 10ms = 1 zero crossing
double setpoint, input, output;
//Specify the links and initial tuning parameters
double Kp = 2, Ki = 5, Kd = 1;
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
// how long is one period of SSR control in ms
int WindowSize = 500;
unsigned long windowStartTime;

volatile uint8_t state;
uint8_t previousState = 0;
unsigned long previousPress;
unsigned long pressTime = 0;
int loopRuntime = 0;
volatile bool longPress = false;

uint8_t upper(float input)
{
    uint32_t temp = (uint32_t)setpoint;
    return (byte)(temp >> 8);
}

void saveValues()
{
    //save values to eeprom, value * 100, then first lower byte, then upper
    // value * 1000 for pid values and calibration
    EEPROM.write(0, (byte)(setpoint * 100));
    EEPROM.write(1, upper(setpoint * 100));

    EEPROM.write(2, (byte)(Kp * 1000));
    EEPROM.write(3, upper(Kp * 1000));

    EEPROM.write(4, (byte)(Ki * 1000));
    EEPROM.write(5, upper(Ki * 1000));

    EEPROM.write(6, (byte)(Kd * 1000));
    EEPROM.write(7, upper(Kd * 1000));

    EEPROM.write(8, (byte)(calibration * 1000));
    EEPROM.write(9, upper(calibration * 1000));
    EEPROM.commit();
}

float loadValues(uint8_t addr)
{
    float value = EEPROM.read(addr);
    value += (uint32_t)(EEPROM.read(addr + 1) << 8);
    return value;
}

// update the display output
void updateDisplay()
{
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(input);
    display.print(F(" C"));
    display.setTextSize(1);
    display.print((int)output);
    display.println("%\n");
    display.print(F("t:"));
    if (state == 0)
    {
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    }
    display.print(setpoint);
    display.setTextColor(SSD1306_WHITE);
    display.print(F("  p:"));
    if (state == 1)
    {
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    }
    display.println(Kp);
    display.setTextColor(SSD1306_WHITE);
    display.print(F("i:"));
    if (state == 2)
    {
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    }
    display.print(Ki);
    display.setTextColor(SSD1306_WHITE);
    display.print(F("  d:"));
    if (state == 3)
    {
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    }
    display.println(Kd);
    display.setTextColor(SSD1306_WHITE);
    display.print(F("cal:"));
    if (state == 4)
    {
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    }
    display.println(calibration);
    display.display();
}

void IRAM_ATTR encoderPress()
{

    if (digitalRead(ENCODER_BUTTON_PIN) == LOW)
    {
        if (millis() - previousPress > 100)
        {
            state = (state + 1) % 5;
            pressTime = millis();
        }
        previousPress = millis();
    }
    if (digitalRead(ENCODER_BUTTON_PIN) == HIGH)
    {
        if (millis() - pressTime > 3000){
            // long press
            longPress = true;
        }
    }
}

void setup(void)
{

    Serial.begin(115200);
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    pinMode(SSR_PIN, OUTPUT);

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    {
        Serial.println(F("SSD1306 allocation failed"));
        // also probably set output low
        digitalWrite(SSR_PIN, LOW);
        for (;;)
            ; // Don't proceed, loop forever
    }

    display.display();
    delay(500);
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE); // Draw white text
    // initialize values from EEPROM
    if (!EEPROM.begin(EEPROM_SIZE))
    {
        display.println("faled to initialise EEPROM");
        delay(2000);
    }
    setpoint = loadValues(0) / 100;
    Kp = loadValues(2) / 1000;
    Ki = loadValues(4) / 1000;
    Kd = loadValues(6) / 1000;
    calibration = loadValues(8) / 1000;

    // set initial PID values
    input = 25.0;
    myPID.SetOutputLimits(0, 100);
    myPID.SetMode(AUTOMATIC);
    analogSetAttenuation(ADC_11db);

    // encoder stuff
    // Enable the weak pull up resistors
    ESP32Encoder::useInternalWeakPullResistors = UP;
    // Attache pins for use as encoder pins
    encoder.attachHalfQuad(19, 18);
    encoder.setCount(105 * ENCODER_RESOLUTION);
    pinMode(ENCODER_BUTTON_PIN, INPUT_PULLUP);
    attachInterrupt(ENCODER_BUTTON_PIN, encoderPress, CHANGE);
}

void loop(void)
{
    unsigned long startTime = millis();
    if (millis() % 20 == 0)
    {

        // measure temp
        uint8_t i;
        float average;

        // take N samples in a row, with a slight delay
        for (i = 0; i < NUMSAMPLES; i++)
        {
            samples[i] = analogRead(THERMISTORPIN);
        }
        // average all the samples out
        average = 0;
        for (i = 0; i < NUMSAMPLES; i++)
        {
            average += samples[i];
        }
        average /= NUMSAMPLES;
        // convert the value to resistance
        average = 4095 / average - 1;
        average = SERIESRESISTOR / average;
        // calculate actual temp
        float steinhart;
        steinhart = average / THERMISTORNOMINAL;          // (R/Ro)
        steinhart = log(steinhart);                       // ln(R/Ro)
        steinhart /= BCOEFFICIENT;                        // 1/B * ln(R/Ro)
        steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
        steinhart = 1.0 / steinhart;                      // Invert
        steinhart -= 273.15;                              // convert absolute temp to C
        steinhart += calibration;

        // do the update for the FSM
        switch (state)
        {
        case 0:
            // roatary encoder controlls the setpoint
            if (previousState != 0)
            {
                encoder.setCount(((int)setpoint * ENCODER_RESOLUTION));
            }
            setpoint = ((float)encoder.getCount()) / ENCODER_RESOLUTION;
            break;
        case 1:
            if (previousState != 1)
            {
                encoder.setCount(((int)Kp * PID_ENCODER_RESOLUTION));
            }
            if (Kp != (((float)encoder.getCount()) / PID_ENCODER_RESOLUTION))
            {
                Kp = ((float)encoder.getCount()) / PID_ENCODER_RESOLUTION;
                myPID.SetTunings(Kp, Ki, Kd);
            }
            break;
        case 2:
            if (previousState != 2)
            {
                encoder.setCount(((int)Ki * PID_ENCODER_RESOLUTION));
            }
            if (Ki != (((float)encoder.getCount()) / PID_ENCODER_RESOLUTION))
            {
                Ki = ((float)encoder.getCount()) / PID_ENCODER_RESOLUTION;
                myPID.SetTunings(Kp, Ki, Kd);
            }

            break;
        case 3:
            if (previousState != 3)
            {
                encoder.setCount(((int)Kd * PID_ENCODER_RESOLUTION));
            }
            if (Kd != (((float)encoder.getCount()) / PID_ENCODER_RESOLUTION))
            {
                Kd = ((float)encoder.getCount()) / PID_ENCODER_RESOLUTION;
                myPID.SetTunings(Kp, Ki, Kd);
            }
            break;
        case 4:
            if (previousState != 4)
            {
                encoder.setCount(((int)calibration * 20.0));
            }
            calibration = ((float)encoder.getCount()) / 20.0;
            break;
        }
        previousState = state;
        input = steinhart;
        myPID.Compute();
        updateDisplay();
        loopRuntime = startTime - millis();
    }
    unsigned long curTime = millis();

    // output logic
    if (curTime - windowStartTime > WindowSize)
    {
        windowStartTime += WindowSize;
        Serial.println(String(input) + ", " + String(output));
        //      Serial.println(loopRuntime);
    }
    if (curTime - windowStartTime <= output * (WindowSize / 100))
        digitalWrite(SSR_PIN, HIGH);
    else
        digitalWrite(SSR_PIN, LOW);
    
    // check for longpress and save to EEPROM if true
    if (longPress)
    {
        saveValues();
        display.writeFillRect(0, 0, 128, 64, SSD1306_WHITE);
        display.display();
    }
}