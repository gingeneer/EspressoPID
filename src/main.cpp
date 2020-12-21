#include "EEPROM.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Bounce2.h>
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

#define EEPROM_SIZE 1000
int samples[NUMSAMPLES];

uint8_t temperature[64];
uint8_t outputArray[64];
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);

ESP32Encoder encoder;

//Define Variables we'll be connecting to
// output = active time of SSR / 10ms
// 1 -> 10ms = 1 zero crossing
double setpoint = 105, input, output;
float calibration = -2.3;
//Specify the links and initial tuning parameters
double Kp = 2, Ki = 5, Kd = 1;
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
// how long is one period of SSR control in ms
int WindowSize = 500;
unsigned long windowStartTime;

uint8_t menu = 0;
uint8_t state;
uint8_t previousState = 0;
uint8_t arrayIndex = 0;

bool longPress = false;

Bounce b = Bounce();

void saveValues()
{
    //save values to eeprom, value * 100, then first lower byte, then upper
    // value * 1000 for pid values and calibration
    int addr = 0;
    EEPROM.writeFloat(addr, setpoint);
    addr += sizeof(float);
    EEPROM.writeFloat(addr, Kp);
    addr += sizeof(float);
    EEPROM.writeFloat(addr, Ki);
    addr += sizeof(float);
    EEPROM.writeFloat(addr, Kd);
    addr += sizeof(float);
    EEPROM.writeFloat(addr, calibration);
    EEPROM.commit();
}

// update the display output
void updateDisplay()
{
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextColor(SSD1306_WHITE);
    display.printf("%3d%% %5.1f", (int)output, setpoint);
    display.setTextSize(2);
    display.setCursor(0, 9);
    display.print(input, 1);
    display.setTextSize(1);
    display.print(F("o"));
    if (menu == 1)
    {
        display.setCursor(0, 32);
        if (state == 0)
        {
            display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        }
        display.print(F("t:  "));
        display.setTextColor(SSD1306_WHITE);
        display.println(setpoint, 1);
        if (state == 1)
        {
            display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        }
        display.print(F("p:  "));
        display.setTextColor(SSD1306_WHITE);
        display.println(Kp);
        if (state == 2)
        {
            display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        }
        display.print(F("i:  "));
        display.setTextColor(SSD1306_WHITE);
        display.println(Ki);
        if (state == 3)
        {
            display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        }
        display.print(F("d:  "));
        display.setTextColor(SSD1306_WHITE);
        display.println(Kd);
        if (state == 4)
        {
            display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        }
        display.print(F("cal:"));
        display.setTextColor(SSD1306_WHITE);
        display.println(calibration);
    }
    else if (menu == 0)
    {
        int index = arrayIndex;
        for (int i = 0; i < 64; i++)
        {
            uint8_t temp = temperature[index];
            // if (temp != 0)
            // {
                uint8_t dutyCycle = outputArray[index];
                display.drawLine(i, 127, i, 127 - temp, SSD1306_WHITE);
                if (dutyCycle <= temp)
                {
                    display.drawPixel(i, 127 - outputArray[index], SSD1306_BLACK);
                }
                else
                {
                    display.drawPixel(i, 127 - outputArray[index], SSD1306_WHITE);
                }
                index = (index + 1) % 64;
            // }
        }
    }

    display.display();
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
        delay(1000);
        ESP.restart();
    }

    display.display();
    delay(500);
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setRotation(3);
    // initialize values from EEPROM
    if (!EEPROM.begin(EEPROM_SIZE))
    {
        display.println("faled to initialise EEPROM");
        delay(2000);
    }
    else
    {
        // load the values from EEPROM
        for (int i = 0; i < 5; i += sizeof(float))
        {
            printf("EEPROM %d: %f\n", i, EEPROM.readFloat(i));
        }
        int addr = 0;
        setpoint = EEPROM.readFloat(addr);
        addr += sizeof(float);
        Kp = EEPROM.readFloat(addr);
        addr += sizeof(float);
        Ki = EEPROM.readFloat(addr);
        addr += sizeof(float);
        Kd = EEPROM.readFloat(addr);
        addr += sizeof(float);
        calibration = EEPROM.readFloat(addr);
    }
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
    encoder.setCount(setpoint * ENCODER_RESOLUTION);
    b.attach(ENCODER_BUTTON_PIN, INPUT_PULLUP);
    b.interval(25);
}

void loop(void)
{
    unsigned long looptime = millis();
    b.update();
    bool changed = false;
    if (b.changed())
    {
        if (b.read() == 0)
        {
            state = (state + 1) % 5;
            longPress = true;
            changed = true;
        }
    }
    if (looptime % 100 == 0 || changed)
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
        if (menu == 0)
        {
            if (previousState != 0)
            {
                encoder.setCount(setpoint * ENCODER_RESOLUTION);
                previousState = 0;
            }
            setpoint = ((float)encoder.getCount()) / ENCODER_RESOLUTION;
        }
        if (menu == 1)
        {
            // do the update for the FSM
            switch (state)
            {
            case 0:
                // roatary encoder controlls the setpoint
                if (previousState != 0)
                {
                    encoder.setCount(setpoint * ENCODER_RESOLUTION);
                }
                setpoint = ((float)encoder.getCount()) / ENCODER_RESOLUTION;
                break;
            case 1:
                if (previousState != 1)
                {
                    encoder.setCount(Kp * PID_ENCODER_RESOLUTION);
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
                    encoder.setCount(Ki * PID_ENCODER_RESOLUTION);
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
                    encoder.setCount(Kd * PID_ENCODER_RESOLUTION);
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
                    encoder.setCount(calibration * 20.0);
                }
                calibration = ((float)encoder.getCount()) / 20.0;
                break;
            }
            previousState = state;
        }

        input = steinhart;
        myPID.Compute();
        updateDisplay();
    }

    // save to the array for the graphics
    if (looptime % 500 == 0)
    {
        uint8_t temp = (uint8_t)(input - 80);
        if (input - 80 < 0){
            temp = 0;
        }
        temperature[arrayIndex] = (uint8_t)(temp);
        outputArray[arrayIndex] = (uint8_t)(output / 10);
        arrayIndex = (arrayIndex + 1) % 64;
    }

    // output logic
    unsigned long curTime = millis();
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
    if (b.read() == 0 && b.duration() > 3000 && longPress)
    {
        switch (menu)
        {
        case 0:
            state = 0;
            menu = 1;
            break;
        case 1:
            saveValues();
            display.writeFillRect(0, 0, 128, 64, SSD1306_WHITE);
            display.display();
            longPress = false;
            menu = 0;
            state = 0;
            break;
        }
        longPress = false;
    }
}