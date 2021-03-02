#include "EEPROM.h"
#include <Adafruit_MAX31865.h> // PT100 temp sensor
#include <Arduino.h>
#include <Bounce2.h>
#include <ESP32Encoder.h>
#include <PID_AutoTune_v0.h>
#include <PID_v1.h>
// #include <SPI.h>
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

#define SSR_PIN 13
#define ENCODER_BUTTON_PIN 32
#define BACK_BUTTON_PIN 33
#define ENCODER_CLK 36 //VP
#define ENCODER_DT 39 //VN
// PT100 stuff:
// Use software SPI: CS, DI, DO, CLK
// Adafruit_MAX31865 pt100 = Adafruit_MAX31865(14, 27, 26, 25);
// use hardware SPI, just pass in the CS pin
Adafruit_MAX31865 pt100 = Adafruit_MAX31865(14);
// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF 430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL 100.0

// how many speps per degree
#define SETPOINT_ENCODER_RESOLUTION 2.0
#define PID_ENCODER_RESOLUTION 10.0

#define EEPROM_SIZE 1000

int previousTempY = 0;
int previousTemp2Y = 0;
int previousDutyY = 40;

ESP32Encoder encoder;

const int displayInterval = 50, graphInterval = 500, pidInterval = 100, FSMInterval = 20;
unsigned long previousDisplayTime = 0, previousGraphTime = 0, previousPidTime = 0, previousFSMTime = 0;

TFT_eSPI tft = TFT_eSPI(); // Invoke library, pins defined in User_Setup.h
double thermistor, thermocouple;
//Define Variables we'll be connecting to PID
double setpoint = 105, input, output;
double calibration = -2.3;
double overshoot = 100.0;
bool overshootMode = false;
bool startup = true;
uint8_t errorState = 0;
//offset for display (what boiler temp equals which water temp)
double temp_offset = 0;
double thermocouple_offset;
//Specify the links and initial tuning parameters
double Kp = 7.4, Ki = 3.7, Kd = 52.4;
// aggressive tunings for PID outside of overshoot range
double aKp = 100, aKi = 0, aKd = 0;
// 0: thermistor
// 1: thermocouple
// 2: avg(thermistor, thermocouple)
uint8_t pidSource = 0;
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// autotune variables:
double aTuneStep = 50, aTuneNoise = 0.2, aTuneStartValue = 50;
unsigned int aTuneLookBack = 30;
boolean tuning = false;
// y tho?
byte ATuneModeRemember = 2;

PID_ATune aTune(&input, &output);
// how long is one period of SSR control in ms
int windowSize = 500;
unsigned long windowStartTime;
unsigned long shotTimerStart;
unsigned long shotTimerStop;
bool initGraph = false;

enum SetupStates
{
    SETPOINT,
    P,
    I,
    D,
    CAL,
    TM_OFFSET,
    TC_OFFSET,
    OVERSHOOT,
    PID_MODE,
    QUIT
};
enum MainScreenStates
{
    RESET,
    START,
    STOP
};
enum MenuState
{
    MAIN,
    SETUP
};

enum MenuState menu;
enum SetupStates setupState;
enum MainScreenStates mainScreenState;

// uint8_t menu = 0;
// uint8_t state;
uint8_t previousState = 0;
uint8_t previousMenu = 0;
uint8_t graphX = 0;
uint8_t selection = 0;
bool longPressPossible = false;
bool longBackPossible = false;

Bounce encoderBounce = Bounce();
Bounce backButtonBounce = Bounce();

void saveValues()
{
    //save values to eeprom, value * 100, then first lower byte, then upper
    // value * 1000 for pid values and calibration
    int addr = 0;
    EEPROM.writeDouble(addr, setpoint);
    addr += sizeof(double);
    EEPROM.writeDouble(addr, Kp);
    addr += sizeof(double);
    EEPROM.writeDouble(addr, Ki);
    addr += sizeof(double);
    EEPROM.writeDouble(addr, Kd);
    addr += sizeof(double);
    EEPROM.writeDouble(addr, calibration);
    addr += sizeof(double);
    EEPROM.writeDouble(addr, temp_offset);
    addr += sizeof(double);
    EEPROM.writeDouble(addr, thermocouple_offset);
    addr += sizeof(double);
    EEPROM.writeDouble(addr, overshoot);
    addr += sizeof(double);
    EEPROM.writeShort(addr, pidSource);
    EEPROM.commit();
}

void updateGraph(double temp1, double temp2, float dutycycle, bool initialize)
{
    if (menu != MAIN)
        return;
    // min/max temp
    const int min = 70;
    const int max = 120;
    // height of graphing area
    const int height = 240;
    // how many pixel per temp
    float scale = float(height) / (float(max) - min);
    int nextIndex = (graphX + 1) % TFT_WIDTH;
    int tempY = TFT_HEIGHT - int(((temp1 - min) * scale + 0.5));
    int temp2Y = TFT_HEIGHT - int(((temp2 - min) * scale + 0.5));
    // int dutyCycleY = TFT_HEIGHT - int(dutycycle * ((float(max) / 100.0f) + 0.5);
    int dutyCycleY = TFT_HEIGHT - int(dutycycle * ((float(80 - min) * scale) / 100.0f) + 0.5);
    if (dutyCycleY == TFT_HEIGHT)
        dutyCycleY = TFT_HEIGHT - 1;
    // draw the graph from back to front
    // legend
    uint8_t len = tft.textWidth("100", 1);
    tft.drawFastVLine(graphX, TFT_HEIGHT - height - 4, height + 4, TFT_BLACK);
    // tft.drawPixel(graphX, TFT_HEIGHT - int((70 - min) * scale + 0.5), TFT_DARKGREY);
    tft.drawPixel(graphX, TFT_HEIGHT - int((80 - min) * scale + 0.5), TFT_DARKGREY);
    tft.drawPixel(graphX, TFT_HEIGHT - int((90 - min) * scale + 0.5), TFT_DARKGREY);
    tft.drawPixel(graphX, TFT_HEIGHT - int((110 - min) * scale + 0.5), TFT_DARKGREY);
    // tft.drawPixel(graphX, TFT_HEIGHT - int((120 - min) * scale + 0.5), TFT_DARKGREY);
    // tft.drawPixel(graphX, TFT_HEIGHT - int((130 - min) * scale + 0.5), TFT_DARKGREY);
    // tft.drawPixel(graphX, TFT_HEIGHT - int((140 - min) * scale + 0.5), TFT_DARKGREY);
    // tft.drawPixel(graphX, TFT_HEIGHT - int((150 - min) * scale + 0.5), TFT_DARKGREY);
    if (initialize)
    {
        // tft.drawFastHLine(0, TFT_HEIGHT - int((70 - min) * scale + 0.5), TFT_WIDTH, TFT_DARKGREY);
        tft.drawFastHLine(0, TFT_HEIGHT - int((80 - min) * scale + 0.5), TFT_WIDTH, TFT_DARKGREY);
        tft.drawFastHLine(0, TFT_HEIGHT - int((90 - min) * scale + 0.5), TFT_WIDTH, TFT_DARKGREY);
        tft.drawFastHLine(len, TFT_HEIGHT - int((100 - min) * scale + 0.5), TFT_WIDTH, TFT_LIGHTGREY);
        tft.drawFastHLine(0, TFT_HEIGHT - int((110 - min) * scale + 0.5), TFT_WIDTH, TFT_DARKGREY);
        // tft.drawFastHLine(0, TFT_HEIGHT - int((120 - min) * scale + 0.5), TFT_WIDTH, TFT_DARKGREY);
        // tft.drawFastHLine(0, TFT_HEIGHT - int((130 - min) * scale + 0.5), TFT_WIDTH, TFT_DARKGREY);
        // tft.drawFastHLine(0, TFT_HEIGHT - int((140 - min) * scale + 0.5), TFT_WIDTH, TFT_DARKGREY);
        // tft.drawFastHLine(0, TFT_HEIGHT - int((150 - min) * scale + 0.5), TFT_WIDTH, TFT_DARKGREY);
        tft.drawFastHLine(len, TFT_HEIGHT - int((max - min) * scale + 0.5), TFT_WIDTH, TFT_DARKGREY);
    }
    if (graphX <= len || initialize)
    {
        tft.setTextDatum(BL_DATUM);
        tft.setTextFont(1);
        tft.setTextSize(1);
        tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
        tft.drawNumber(min, 0, TFT_HEIGHT);
        tft.setTextDatum(CL_DATUM);
        tft.drawNumber(100, 0, TFT_HEIGHT - int((100.0 - min) * scale + 0.5));
        //tft.drawFastHLine(len, TFT_HEIGHT - int((100.0 - min) * scale + 0.5), TFT_WIDTH, TFT_DARKGREY);
        tft.drawNumber(max, 0, TFT_HEIGHT - int(float(max - min) * scale + 0.5));
        //tft.drawFastHLine(len, TFT_HEIGHT - int(float(max - min) * scale + 0.5), TFT_WIDTH, TFT_DARKGREY);
        // actual graph
    }
    else
    {
        tft.drawPixel(graphX, TFT_HEIGHT - int((100 - min) * scale + 0.5), TFT_LIGHTGREY);
        tft.drawPixel(graphX, TFT_HEIGHT - int((max - min) * scale + 0.5), TFT_DARKGREY);
    }
    if (graphX <= 1)
    {
        previousTempY = tempY;
        previousTemp2Y = temp2Y;
        previousDutyY = dutyCycleY;
    }
    if (previousTempY < TFT_HEIGHT - height)
    {
        previousTempY = TFT_HEIGHT - height;
    }
    // following code is for dutycycle line, like temp
    // if (dutyCycleY > previousDutyY)
    // {
    //     tft.drawFastVLine(graphX, previousDutyY, dutyCycleY - previousDutyY + 1, TFT_GREEN);
    // }
    // else if (dutyCycleY < previousDutyY)
    // {
    //     tft.drawFastVLine(graphX, dutyCycleY, previousDutyY - dutyCycleY + 1, TFT_GREEN);
    // }
    // else
    // {
    //     tft.drawPixel(graphX, dutyCycleY, TFT_GREEN);
    // }
    // draw like a bar at bottom
    tft.drawFastVLine(graphX, dutyCycleY, TFT_HEIGHT - dutyCycleY, TFT_GREEN);
    if (temp1 >= min && temp1 <= max)
    {
        if (tempY > previousTempY)
        {
            tft.drawFastVLine(graphX, previousTempY, tempY - previousTempY + 1, TFT_RED);
        }
        else if (tempY < previousTempY)
        {
            tft.drawFastVLine(graphX, tempY, previousTempY - tempY + 1, TFT_RED);
        }
        else
        {
            tft.drawPixel(graphX, tempY, TFT_RED);
        }
    }
    if (temp2 >= min && temp2 <= max)
    {
        if (temp2Y > previousTemp2Y)
        {
            tft.drawFastVLine(graphX, previousTemp2Y, temp2Y - previousTemp2Y + 1, TFT_PINK);
        }
        else if (temp2Y < previousTemp2Y)
        {
            tft.drawFastVLine(graphX, temp2Y, previousTemp2Y - temp2Y + 1, TFT_PINK);
        }
        else
        {
            tft.drawPixel(graphX, temp2Y, TFT_PINK);
        }
    }
    tft.drawFastVLine(nextIndex, TFT_HEIGHT - height - 4, height + 4, TFT_WHITE);
    if (nextIndex < TFT_WIDTH - 1)
    {
        tft.drawFastVLine(nextIndex + 1, TFT_HEIGHT - height - 4, height + 4, TFT_WHITE);
    }

    previousDutyY = dutyCycleY;
    previousTempY = tempY;
    previousTemp2Y = temp2Y;
}

// update the display output
void updateDisplay()
{
    long curTime = millis();
    tft.setCursor(0, 0, 1);
    // Set the font colour to be white with a black background, set text size multiplier to 1
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(1);
    tft.setTextFont(2);
    int uptime_mins = curTime / (1000 * 60);
    int uptime_secs = (curTime / (1000)) % 60;
    tft.printf("%3d:%02d", uptime_mins, uptime_secs);
    tft.setTextDatum(TR_DATUM); // top right]
    char buf[12];
    snprintf(buf, 6, "%5.1f", setpoint);
    tft.drawString(buf, TFT_WIDTH, 0);
    tft.setTextDatum(TC_DATUM); // top center
    if (overshootMode)
    {
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    }
    if (tuning)
    {
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
    }
    if (startup)
    {
        tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    }
    snprintf(buf, 5, "%3d%%", (int)output);
    tft.drawString(buf, TFT_WIDTH / 2, 0);
    tft.drawFastHLine(0, tft.fontHeight(2) + 1, TFT_WIDTH, TFT_ORANGE);

    uint8_t curHeight = tft.fontHeight(2) + 5;
    // degrees thermistor
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextDatum(TL_DATUM);
    tft.setTextFont(4);
    // tft.fillRect(0, curHeight, TFT_WIDTH/2, tft.fontHeight(4), TFT_BLACK);
    snprintf(buf, 11, "%6.1f`C    ", input);
    uint8_t len = tft.drawString(buf, 0, curHeight);
    if (errorState)
    {
        tft.setTextFont(2);
        tft.setTextColor(TFT_RED, TFT_BLACK);
        len += tft.drawString("ERROR: ", len, curHeight);
        tft.drawNumber(errorState, len, curHeight);
    }

    switch (menu)
    {
    case MAIN:
        // cant do that...
        // tft.fillRect(0, 12, TFT_WIDTH, TFT_HEIGHT, TFT_BLACK);
        tft.setTextDatum(TR_DATUM);
        char buf[7];
        switch (mainScreenState)
        {
        case RESET:
            strncpy(buf, "  0.0s", 7);
            tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
            break;
        case START:
            snprintf(buf, 7, "%.1fs", float(curTime - shotTimerStart) / 1000);
            tft.setTextColor(TFT_WHITE, TFT_BLACK);
            break;
        case STOP:
            snprintf(buf, 7, "%.1fs", float(shotTimerStop - shotTimerStart) / 1000);
            tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
            break;
        }
        tft.setTextFont(4);
        tft.drawString(buf, TFT_WIDTH, curHeight);
        break;
    case SETUP:
        // show all the values, the selection is controlled by moveselecion in the FSM
        int curLine = 60;
        int lineIncr = 10;
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setTextFont(1);
        tft.setCursor(10, curLine);
        tft.printf("setpoint: %7.2f", setpoint);
        curLine += lineIncr;
        tft.setCursor(10, curLine);
        tft.printf("P: %6.3f", Kp);
        curLine += lineIncr;
        tft.setCursor(10, curLine);
        tft.printf("I: %6.3f", Ki);
        curLine += lineIncr;
        tft.setCursor(10, curLine);
        tft.printf("D: %6.3f", Kd);
        curLine += lineIncr;
        tft.setCursor(10, curLine);
        tft.printf("Therm. cal:%7.2f", calibration);
        curLine += lineIncr;
        tft.setCursor(10, curLine);
        tft.printf("T offset:%4.1f", temp_offset);
        curLine += lineIncr;
        tft.setCursor(10, curLine);
        tft.printf("TC cal:%7.2f", thermocouple_offset);
        curLine += lineIncr;
        tft.setCursor(10, curLine);
        tft.printf("overshoot:%5.2f", overshoot);
        curLine += lineIncr;
        tft.setCursor(10, curLine);
        tft.printf("pid mode: %d", pidSource);
        curLine += lineIncr;

        // uint8_t fontheight = tft.fontHeight(1);
        if (setupState == QUIT)
        {
            tft.setCursor(10, curLine);
            tft.setTextColor(TFT_ORANGE, TFT_BLACK);
            tft.print("save to EEPROM?");
            curLine += lineIncr;
            tft.setCursor(10, curLine);
            if ((encoder.getCount() & 0b1) == 0)
                tft.setTextColor(TFT_BLACK, TFT_GREEN);
            else
                tft.setTextColor(TFT_WHITE, TFT_BLACK);
            tft.print("Yes");
            if ((encoder.getCount() & 0b1) == 1)
                tft.setTextColor(TFT_BLACK, TFT_RED);
            else
                tft.setTextColor(TFT_WHITE, TFT_BLACK);
            tft.setCursor(60, curLine);
            tft.print("No");
        }
        break;
    }
}

void moveSelection(uint8_t from, uint8_t to, bool visible)
{
    // 0-5: x: 2, y: 42+x*10, cursor 6x6 rect
    tft.fillRect(2, 60 + from * 10, 7, 7, TFT_BLACK);
    if (visible)
        tft.fillRect(2, 60 + to * 10, 7, 7, TFT_RED);
}

void AutoTuneHelper(boolean start)
{
    if (start)
        ATuneModeRemember = myPID.GetMode();
    else
        myPID.SetMode(ATuneModeRemember);
}
void changeAutoTune()
{
    if (!tuning)
    {
        //Set the output to the desired starting frequency.
        output = aTuneStartValue;
        aTune.SetNoiseBand(aTuneNoise);
        aTune.SetOutputStep(aTuneStep);
        aTune.SetLookbackSec((int)aTuneLookBack);
        AutoTuneHelper(true);
        tuning = true;
    }
    else
    { //cancel autotune
        aTune.Cancel();
        tuning = false;
        AutoTuneHelper(false);
    }
}

double getTemp_PT100()
{
    double temp = pt100.temperature(RNOMINAL, RREF) + calibration;
    // Serial.println(temp);
    uint8_t fault = pt100.readFault();
    if (fault)
    {
        errorState = fault;
        Serial.print("Fault 0x");
        Serial.println(fault, HEX);
        if (fault & MAX31865_FAULT_HIGHTHRESH)
        {
            Serial.println("RTD High Threshold");
        }
        if (fault & MAX31865_FAULT_LOWTHRESH)
        {
            Serial.println("RTD Low Threshold");
        }
        if (fault & MAX31865_FAULT_REFINLOW)
        {
            Serial.println("REFIN- > 0.85 x Bias");
        }
        if (fault & MAX31865_FAULT_REFINHIGH)
        {
            Serial.println("REFIN- < 0.85 x Bias - FORCE- open");
        }
        if (fault & MAX31865_FAULT_RTDINLOW)
        {
            Serial.println("RTDIN- < 0.85 x Bias - FORCE- open");
        }
        if (fault & MAX31865_FAULT_OVUV)
        {
            Serial.println("Under/Over voltage");
        }
        pt100.clearFault();
    }
    return temp;
}

void setup(void)
{
    Serial.begin(115200);
    pinMode(SSR_PIN, OUTPUT);
    // initialize the TFT
    pt100.begin(MAX31865_3WIRE);
    pt100.enable50Hz(true);
    tft.init();
    tft.setRotation(0);
    tft.fillScreen(TFT_BLACK);
    encoderBounce.attach(ENCODER_BUTTON_PIN, INPUT_PULLUP);
    encoderBounce.interval(25);
    backButtonBounce.attach(BACK_BUTTON_PIN, INPUT_PULLUP);
    backButtonBounce.interval(25);
    delay(100);
    // initialize values from EEPROM
    backButtonBounce.update();
    if (!EEPROM.begin(EEPROM_SIZE))
    {
        tft.println("faled to initialise EEPROM");
        delay(2000);
    }
    // hold back button on startup to skip load from EEPROM
    else if (backButtonBounce.read() == HIGH)
    {
        // load the values from EEPROM
        int addr = 0;
        setpoint = EEPROM.readDouble(addr);
        addr += sizeof(double);
        Kp = EEPROM.readDouble(addr);
        addr += sizeof(double);
        Ki = EEPROM.readDouble(addr);
        addr += sizeof(double);
        Kd = EEPROM.readDouble(addr);
        addr += sizeof(double);
        calibration = EEPROM.readDouble(addr);
        addr += sizeof(double);
        temp_offset = EEPROM.readDouble(addr);
        addr += sizeof(double);
        thermocouple_offset = EEPROM.readDouble(addr);
        addr += sizeof(double);
        overshoot = EEPROM.readDouble(addr);
        addr += sizeof(double);
        pidSource = EEPROM.readShort(addr);
    }
    // set initial PID values
    input = 25.0;
    myPID.SetOutputLimits(0, 100);
    myPID.SetMode(AUTOMATIC);
    myPID.SetSampleTime(pidInterval);
    myPID.SetTunings(Kp, Ki, Kd);
    if (tuning)
    {
        tuning = false;
        changeAutoTune();
        tuning = true;
    }
    // set control type to PID
    aTune.SetControlType(1);

    // encoder stuff
    // Enable the weak pull up resistors
    ESP32Encoder::useInternalWeakPullResistors = UP;
    // Attache pins for use as encoder pins
    encoder.attachHalfQuad(ENCODER_DT, ENCODER_CLK);
    encoder.setCount(setpoint * SETPOINT_ENCODER_RESOLUTION);
    initGraph = true;
}

void FSM()
{
    encoderBounce.update();
    backButtonBounce.update();
    bool selectPressed = false;
    bool backPressed = false;
    bool longSelect = false;
    bool update = false;
    bool longBack = false;
    if (encoderBounce.changed())
    {
        if (encoderBounce.read() == 0)
        {
            update = true;
            longPressPossible = true;
            selectPressed = true;
        }
    }
    if (backButtonBounce.changed())
    {
        if (backButtonBounce.read() == 0)
        {
            update = true;
            backPressed = true;
            longBackPossible = true;
        }
    }
    // check for longpress
    if (encoderBounce.read() == 0 && encoderBounce.duration() > 2000 && longPressPossible)
    {
        longSelect = true;
        longPressPossible = false;
        update = true;
    }
    if (backButtonBounce.read() == 0 && backButtonBounce.duration() > 2000 && longBackPossible)
    {
        longBack = true;
        longBackPossible = false;
        update = true;
    }

    // FSM update
    unsigned long curTime = millis();
    if (update || curTime - previousFSMTime >= FSMInterval)
    {
        previousFSMTime = curTime;
        switch (menu)
        {
        case MAIN:
            setpoint = ((double)encoder.getCount()) / SETPOINT_ENCODER_RESOLUTION;
            switch (mainScreenState)
            {
            case RESET:
                // if pressed, start timer and switch to running state
                if (selectPressed)
                {
                    shotTimerStart = millis();
                    mainScreenState = START;
                    tft.fillScreen(TFT_BLACK);
                    initGraph = true;
                    graphX = 0;
                }
                break;
            case START:
                // timer running, stop if pressed
                if (selectPressed)
                {
                    shotTimerStop = millis();
                    mainScreenState = STOP;
                }
                break;
            case STOP:
                if (selectPressed)
                {
                    mainScreenState = RESET;
                }
                break;
            }
            if (longSelect)
            {
                tft.fillScreen(TFT_BLACK);
                moveSelection(0, 0, true);
                menu = SETUP;
                setupState = SETPOINT;
                encoder.setCount(setpoint * SETPOINT_ENCODER_RESOLUTION);
            }
            if (longBack)
            {
                changeAutoTune();
            }
            break;
        case SETUP:
            // screen to config the values
            switch (setupState)
            {
            case SETPOINT:
                // roatary encoder controlls the setpoint
                setpoint = ((double)encoder.getCount()) / SETPOINT_ENCODER_RESOLUTION;
                if (selectPressed)
                {
                    setupState = P;
                    encoder.setCount(Kp * PID_ENCODER_RESOLUTION);
                    moveSelection(0, 1, true);
                }
                if (backPressed)
                {
                    setupState = QUIT;
                    encoder.setCount(0);
                    moveSelection(0, 0, false);
                }
                break;
            case P:
                if (Kp != (((double)encoder.getCount()) / PID_ENCODER_RESOLUTION))
                {
                    Kp = ((double)encoder.getCount()) / PID_ENCODER_RESOLUTION;
                    myPID.SetTunings(Kp, Ki, Kd);
                }
                if (selectPressed)
                {
                    setupState = I;
                    encoder.setCount(Ki * PID_ENCODER_RESOLUTION);
                    moveSelection(1, 2, true);
                }
                if (backPressed)
                {
                    setupState = QUIT;
                    encoder.setCount(0);
                    moveSelection(0, 0, false);
                }
                break;
            case I:
                if (Ki != (((double)encoder.getCount()) / PID_ENCODER_RESOLUTION))
                {
                    Ki = ((double)encoder.getCount()) / PID_ENCODER_RESOLUTION;
                    myPID.SetTunings(Kp, Ki, Kd);
                }
                if (selectPressed)
                {
                    setupState = D;
                    encoder.setCount(Kd * PID_ENCODER_RESOLUTION);
                    moveSelection(2, 3, true);
                }
                if (backPressed)
                {
                    setupState = QUIT;
                    encoder.setCount(0);
                    moveSelection(0, 0, false);
                }
                break;
            case D:
                if (Kd != (((double)encoder.getCount()) / PID_ENCODER_RESOLUTION))
                {
                    Kd = ((double)encoder.getCount()) / PID_ENCODER_RESOLUTION;
                    myPID.SetTunings(Kp, Ki, Kd);
                }
                if (selectPressed)
                {
                    setupState = CAL;
                    encoder.setCount(calibration * 20.0);
                    moveSelection(3, 4, true);
                }
                if (backPressed)
                {
                    setupState = QUIT;
                    encoder.setCount(0);
                    moveSelection(0, 0, false);
                }
                break;
            case CAL:
                calibration = ((double)encoder.getCount()) / 20.0;
                if (selectPressed)
                {
                    encoder.setCount(temp_offset * 10.0);
                    setupState = TM_OFFSET;
                    moveSelection(4, 5, true);
                }
                if (backPressed)
                {
                    setupState = QUIT;
                    encoder.setCount(0);
                    moveSelection(0, 0, false);
                }
                break;
            case TM_OFFSET:
                temp_offset = ((double)encoder.getCount()) / 10.0;
                if (selectPressed)
                {
                    encoder.setCount(thermocouple_offset * 10.0);
                    setupState = TC_OFFSET;
                    moveSelection(5, 6, true);
                }
                if (backPressed)
                {
                    setupState = QUIT;
                    encoder.setCount(0);
                    moveSelection(0, 0, false);
                }
                break;
            case TC_OFFSET:
                thermocouple_offset = ((double)encoder.getCount()) / 10.0;
                if (selectPressed)
                {
                    encoder.setCount(overshoot * 2.0);
                    setupState = OVERSHOOT;
                    moveSelection(6, 7, true);
                }
                if (backPressed)
                {
                    setupState = QUIT;
                    encoder.setCount(0);
                    moveSelection(0, 0, false);
                }
                break;
            case OVERSHOOT:
                overshoot = double(encoder.getCount()) / 2.0;
                if (selectPressed)
                {
                    encoder.setCount(pidSource);
                    setupState = PID_MODE;
                    moveSelection(7, 8, true);
                }
                if (backPressed)
                {
                    setupState = QUIT;
                    encoder.setCount(0);
                    moveSelection(0, 0, false);
                }
                break;
            case PID_MODE:
                pidSource = abs(encoder.getCount() % 3);
                if (selectPressed)
                {
                    encoder.setCount(setpoint * SETPOINT_ENCODER_RESOLUTION);
                    setupState = SETPOINT;
                    moveSelection(8, 0, true);
                }
                if (backPressed)
                {
                    setupState = QUIT;
                    encoder.setCount(0);
                    moveSelection(0, 0, false);
                }
                break;
            case QUIT:
                if (selectPressed)
                {
                    if ((encoder.getCount() & 0b1) == 0)
                    {
                        saveValues();
                    }
                    menu = MAIN;
                    mainScreenState = RESET;
                    encoder.setCount(setpoint * SETPOINT_ENCODER_RESOLUTION);
                    tft.fillScreen(TFT_BLACK);
                    initGraph = true;
                    graphX = 0;
                }
                if (backPressed)
                {
                    encoder.setCount(setpoint * SETPOINT_ENCODER_RESOLUTION);
                    setupState = SETPOINT;
                    tft.fillScreen(TFT_BLACK);
                    moveSelection(0, 0, true);
                }
                break;
            }
            break;
        }
    }
}

void loop(void)
{
    FSM();
    unsigned long curTime = millis();
    if (curTime - previousDisplayTime >= displayInterval)
    {
        previousDisplayTime = curTime;
        updateDisplay();
    }
    if (curTime - previousPidTime >= pidInterval)
    {
        previousPidTime = curTime;

        thermistor = getTemp_PT100();
        input = thermistor;
        input += temp_offset;
        if ((input <= 1 || input > 180) && !errorState)
        {
            errorState = 69;
        }else if ((input > 1 && input < 180) && errorState == 69)
        {
            errorState = 0;
        }
        
        if (!tuning)
        {
            // use more aggressive tunings when far away from setpoint
            if (!overshootMode && abs(input - setpoint) >= overshoot)
            {
                overshootMode = true;
                myPID.SetTunings(aKp, aKi, aKd);
            }
            else if (overshootMode && abs(input - setpoint) < overshoot)
            {
                overshootMode = false;
                myPID.SetTunings(Kp, Ki, Kd);
            }
            myPID.Compute();
        }
    }
    curTime = millis();
    if (curTime - previousGraphTime >= graphInterval)
    {
        previousGraphTime = curTime;
        graphX = (graphX + 1) % TFT_WIDTH;
        // Serial.println(String(input) + ", " + String(output));
        updateGraph(input, thermocouple, output, initGraph);
        initGraph = false;
    }
    if (tuning)
    {
        byte val = (aTune.Runtime());
        if (val != 0)
        {
            tuning = false;
        }
        if (!tuning)
        { //we're done, set the tuning parameters
            Kp = aTune.GetKp();
            Ki = aTune.GetKi();
            Kd = aTune.GetKd();
            myPID.SetTunings(Kp, Ki, Kd);
            AutoTuneHelper(false);
        }
    }

    // at startup, overheat in order to heat the whole thing up faster
    if (startup && input < 120.0)
    {
        output = 100;
    }
    else if (startup && input >= 120.0)
    {
        startup = false;
    }
    if (errorState)
    {
        output = 0;
    }
    // output logic for the SSR
    curTime = millis();
    if (curTime - windowStartTime > windowSize)
    {
        windowStartTime += windowSize;
    }
    if (curTime - windowStartTime <= output * (windowSize / 100))
        digitalWrite(SSR_PIN, HIGH);
    else
        digitalWrite(SSR_PIN, LOW);
}
