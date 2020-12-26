#include "EEPROM.h"
#include <Arduino.h>
#include <Bounce2.h>
#include <ESP32Encoder.h>
#include <PID_v1.h>
#include <SPI.h>
#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
#include <Wire.h>

//// these pins are defined in user_setup.h in the tft_espi library
// here just for reference
// #define TFT_MISO 19  // got no use for MISO with my screen
// #define TFT_MOSI 23
// #define TFT_SCLK 18
// #define TFT_CS   15  // Chip select control pin
// #define TFT_DC    2  // Data Command control pin
// #define TFT_RST   4  // Reset pin (could connect to RST pin)

#define THERMISTOR_PIN 34 // VP pin in my case
#define SSR_PIN 13
#define ENCODER_BUTTON_PIN 22
#define ENCODER_CLK 21
#define ENCODER_DT 19
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
#define OFFSET_ENCODER_RESOLUTION 2.0
#define PID_ENCODER_RESOLUTION 100.0

#define EEPROM_SIZE 1000
int samples[NUMSAMPLES];

uint16_t tempArray[TFT_WIDTH];
uint8_t outputArray[TFT_WIDTH];

ESP32Encoder encoder;

TFT_eSPI tft = TFT_eSPI(); // Invoke library, pins defined in User_Setup.h

//Define Variables we'll be connecting to PID
double setpoint = 105, input, output;
float calibration = -2.3;
//Specify the links and initial tuning parameters
double Kp = 2, Ki = 5, Kd = 1;
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
// how long is one period of SSR control in ms
int windowSize = 500;
unsigned long windowStartTime;
unsigned long shotTimerStart;
unsigned long shotTimerStop;
bool timerRunning;

uint8_t menu = 0;
uint8_t state;
uint8_t previousState = 0;
uint8_t previousMenu = 0;
uint8_t arrayIndex = 0;
uint8_t selection = 0;
bool longPressPossible = false;

Bounce bounce = Bounce();

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
    long curTime = millis();
    tft.setCursor(0, 0, 1);
    // Set the font colour to be white with a black background, set text size multiplier to 1
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(1);
    int uptime_mins = curTime / (1000 * 60);
    int uptime_secs = (curTime / (1000)) % 60;
    tft.printf("%3d:%02d", uptime_mins, uptime_secs);
    tft.setTextDatum(TR_DATUM); // top right
    tft.drawFloat(setpoint, 1, TFT_WIDTH, 0);
    tft.setTextDatum(TC_DATUM); // top center
    char buf[5];
    snprintf(buf, 5, "%d%%", (int)output);
    tft.drawString(buf, TFT_WIDTH / 2, 0);
    tft.drawFastHLine(0, tft.fontHeight(1) + 1, TFT_WIDTH, TFT_ORANGE);

    switch (menu)
    {
    case 0:
        // cant do that...
        // tft.fillRect(0, 12, TFT_WIDTH, TFT_HEIGHT, TFT_BLACK);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setTextDatum(TL_DATUM);
        tft.setTextFont(2);
        uint8_t len = tft.drawFloat(input, 1, 0, 12);
        tft.setTextFont(1);
        tft.drawString("o", len, 12);
        tft.setTextDatum(TR_DATUM);
        char buf[7];
        switch (state)
        {
        case 0:
            strncpy(buf, "  0.0s", 7);
            tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
            break;
        case 1:
            snprintf(buf, 7, "%.1fs", float(curTime - shotTimerStart) / 1000);
            tft.setTextColor(TFT_WHITE, TFT_BLACK);
            break;
        case 2:
            snprintf(buf, 7, "%.1fs", float(shotTimerStop - shotTimerStart) / 1000);
            tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
            break;
        }
        tft.setTextFont(2);
        tft.drawString(buf, TFT_WIDTH, 12);
        // graph part
        const int min = 70;
        const int max = 160;
        // const int height = 90;
        tft.setTextDatum(BL_DATUM);
        tft.setTextFont(1);
        tft.setTextSize(1);
        tft.setTextColor(TFT_DARKGREY);
        tft.drawNumber(min, 0, TFT_HEIGHT);
        tft.setTextDatum(CL_DATUM);
        len = tft.drawNumber(100, 0, TFT_HEIGHT - 100 + min);
        tft.drawFastHLine(len, TFT_HEIGHT - 100 + min, TFT_WIDTH, TFT_DARKGREY);
        len = tft.drawNumber(max, 0, TFT_HEIGHT - max + min);
        tft.drawFastHLine(len, TFT_HEIGHT - max + min, TFT_WIDTH, TFT_DARKGREY);
        // this is too slow
        // int index = arrayIndex;
        // uint8_t previousDuty = outputArray[0];
        // uint8_t previousTemp = tempArray[0];
        // for (int i = 0; i < TFT_WIDTH; i++)
        // {
        //     uint8_t temp = TFT_HEIGHT - ((tempArray[index] / 100) - 70);
        //     uint8_t dutyCycle = TFT_HEIGHT - outputArray[index];
        //     tft.drawFastVLine(i, previousTemp, temp, TFT_RED);
        //     tft.drawFastVLine(i, previousDuty, dutyCycle, TFT_GREEN);
        //     index = (index + 1) % TFT_WIDTH;
        //     previousDuty = dutyCycle;
        //     previousTemp = temp;
        // }
    }
}

void setup(void)
{
    Serial.begin(115200);
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    pinMode(SSR_PIN, OUTPUT);
    // initialize the TFT
    tft.init();
    tft.setRotation(2);
    tft.fillScreen(TFT_BLACK);
    // We can now plot text on screen using the "print" class
    // tft.println("Hello World!");
    // initialize values from EEPROM
    if (!EEPROM.begin(EEPROM_SIZE))
    {
        tft.println("faled to initialise EEPROM");
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
    encoder.attachHalfQuad(ENCODER_CLK, ENCODER_DT);
    encoder.setCount(setpoint * OFFSET_ENCODER_RESOLUTION);
    bounce.attach(ENCODER_BUTTON_PIN, INPUT_PULLUP);
    bounce.interval(25);
}

void loop(void)
{
    unsigned long looptime = millis();
    bounce.update();
    bool selectPressed = false;
    // bool backPressed = false;
    bool longSelect = false;
    bool update = false;
    if (bounce.changed())
    {
        if (bounce.read() == 0)
        {
            update = true;
            longPressPossible = true;
            selectPressed = true;
        }
    }
    // check for longpress
    if (bounce.read() == 0 && bounce.duration() > 3000 && longPressPossible)
    {
        longSelect = true;
        longPressPossible = false;
        update = true;
    }
    // FSM update
    if (looptime % 20 == 0 || update)
    {
        switch (menu)
        {
        case 0:
            setpoint = ((float)encoder.getCount()) / OFFSET_ENCODER_RESOLUTION;
            switch (state)
            {
            case 0:
                // if pressed, start timer and switch to running state
                if (selectPressed)
                {
                    shotTimerStart = millis();
                    state = 1;
                }
                break;
            case 1:
                // timer running, stop if pressed
                if (selectPressed)
                {
                    shotTimerStop = millis();
                    state = 2;
                }
                break;
            case 2:
                if (selectPressed)
                {
                    state = 0;
                }
                break;
            }
            if (longSelect)
            {
                menu = 1;
                state = 0;
                encoder.setCount(setpoint * OFFSET_ENCODER_RESOLUTION);
            }
            break;
        case 1:
            // screen to config the values
            switch (state)
            {
            case 0:
                // roatary encoder controlls the setpoint
                if (previousState != 0 || previousMenu != menu)
                {
                    encoder.setCount(setpoint * OFFSET_ENCODER_RESOLUTION);
                }
                setpoint = ((float)encoder.getCount()) / OFFSET_ENCODER_RESOLUTION;
                if (selectPressed)
                {
                    state = 1;
                    encoder.setCount(Kp * PID_ENCODER_RESOLUTION);
                }
                break;
            case 1:
                if (Kp != (((float)encoder.getCount()) / PID_ENCODER_RESOLUTION))
                {
                    Kp = ((float)encoder.getCount()) / PID_ENCODER_RESOLUTION;
                    myPID.SetTunings(Kp, Ki, Kd);
                }
                if (selectPressed)
                {
                    state = 2;
                    encoder.setCount(Ki * PID_ENCODER_RESOLUTION);
                }
                break;
            case 2:
                if (Ki != (((float)encoder.getCount()) / PID_ENCODER_RESOLUTION))
                {
                    Ki = ((float)encoder.getCount()) / PID_ENCODER_RESOLUTION;
                    myPID.SetTunings(Kp, Ki, Kd);
                }
                if (selectPressed)
                {
                    state = 3;
                    encoder.setCount(Kd * PID_ENCODER_RESOLUTION);
                }
                break;
            case 3:
                if (Kd != (((float)encoder.getCount()) / PID_ENCODER_RESOLUTION))
                {
                    Kd = ((float)encoder.getCount()) / PID_ENCODER_RESOLUTION;
                    myPID.SetTunings(Kp, Ki, Kd);
                }
                if (selectPressed)
                {
                    state = 4;
                    encoder.setCount(calibration * 20.0);
                }
                break;
            case 4:
                calibration = ((float)encoder.getCount()) / 20.0;
                if (selectPressed)
                {
                    encoder.setCount(setpoint * OFFSET_ENCODER_RESOLUTION);
                    state = 0;
                }
                break;
            }
            if (longSelect)
            {
                // saveValues();
                menu = 0;
                state = 0;
                encoder.setCount(setpoint * OFFSET_ENCODER_RESOLUTION);
            }
            break;
        }
    }
    if (looptime % 50 == 0)
    {
        // measure temp
        uint8_t i;
        float average;

        // take N samples in a row, with a slight delay
        for (i = 0; i < NUMSAMPLES; i++)
        {
            samples[i] = analogRead(THERMISTOR_PIN);
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
        input = steinhart;
        myPID.Compute();
        updateDisplay();
    }
    // save to the array for the graphics
    if (looptime % 500 == 0)
    {
        uint8_t temp = (uint8_t)(input - 80);
        if (input - 80 < 0)
        {
            temp = 0;
        }
        tempArray[arrayIndex] = (uint16_t)(temp * 100);
        outputArray[arrayIndex] = (uint8_t)(output);
        arrayIndex = (arrayIndex + 1) % TFT_WIDTH;
        Serial.println(String(input) + ", " + String(output));
    }

    // output logic for the SSR
    unsigned long curTime = millis();
    if (curTime - windowStartTime > windowSize)
    {
        windowStartTime += windowSize;
    }
    if (curTime - windowStartTime <= output * (windowSize / 100))
        digitalWrite(SSR_PIN, HIGH);
    else
        digitalWrite(SSR_PIN, LOW);
}
