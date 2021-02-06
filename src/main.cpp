#include "EEPROM.h"
#include <Adafruit_ADS1015.h>
#include <Arduino.h>
#include <Bounce2.h>
#include <ESP32Encoder.h>
#include <PID_AutoTune_v0.h>
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

#define THERMISTOR_PIN 34
#define THERMOCOUPLE_PIN 35
#define SSR_PIN 13
#define ENCODER_BUTTON_PIN 32
#define BACK_BUTTON_PIN 14
#define ENCODER_CLK 33
#define ENCODER_DT 25
// resistance at 25 degrees C
#define THERMISTORNOMINAL 100000
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25

// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
// #define SERIESRESISTOR 99000
// 10k better for this temp range
#define SERIESRESISTOR 9850
// how many speps per degree
#define SETPOINT_ENCODER_RESOLUTION 2.0
#define PID_ENCODER_RESOLUTION 100.0

#define EEPROM_SIZE 1000

// uint16_t tempArray[TFT_WIDTH];
// uint8_t outputArray[TFT_WIDTH];
uint8_t previousTempY = 0;
uint8_t previousTemp2Y = 0;
uint8_t previousDutyY = 40;

ESP32Encoder encoder;

// how many samples to take and average, more takes longer
// but is more 'smooth' (NUMSAMPLE*ADCSampleInterval < pidInterval)
#define NUMSAMPLES 5
const int displayInterval = 50, graphInterval = 500, pidInterval = 100, ADCSampleInterval = 5;
unsigned long previousDisplayTime = 0, previousGraphTime = 0, previousPidTime = 0, previousADCSampleTime = 0, startADCSampleTime = 0;
uint8_t currentADCSample = 0;
uint32_t ThermistorSamples, ThermocoupleSamples;

TFT_eSPI tft = TFT_eSPI(); // Invoke library, pins defined in User_Setup.h
float thermistor, thermocouple;
//Define Variables we'll be connecting to PID
double setpoint = 105, input, output;
double calibration = -2.3;
double overshoot = 2.0;
bool overshootMode = false;
//offset for display (what boiler temp equals which water temp)
double temp_offset = 0;
double thermocouple_offset;
//Specify the links and initial tuning parameters
double Kp = 2, Ki = 5, Kd = 1;
// aggressive tunings for PID outside of overshoot range
double aKp = 100, aKi = 0, aKd = 0;
// 0: thermistor
// 1: thermocouple
// 2: avg(thermistor, thermocouple)
uint8_t pidSource = 0;
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// autotune variables:
double aTuneStep = 50, aTuneNoise = 0.2, aTuneStartValue = 50;
unsigned int aTuneLookBack = 10;
boolean tuning = false;
// y tho?
byte ATuneModeRemember = 2;

PID_ATune aTune(&input, &output);
// how long is one period of SSR control in ms
int windowSize = 500;
unsigned long windowStartTime;
unsigned long shotTimerStart;
unsigned long shotTimerStop;
bool timerRunning;
bool initGraph = false;

// external ADC, since the one on esp32 is shite
Adafruit_ADS1015 ads(0x48);

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
    const uint8_t min = 80;
    const uint8_t max = 120;
    // height of graphing area
    const uint8_t height = 120;
    // how many pixel per temp
    float scale = float(height) / (float(max) - min);
    uint8_t nextIndex = (graphX + 1) % TFT_WIDTH;
    uint8_t tempY = TFT_HEIGHT - uint8_t(((temp1 - min) * scale + 0.5));
    uint8_t temp2Y = TFT_HEIGHT - uint8_t(((temp2 - min) * scale + 0.5));
    uint8_t dutyCycleY = TFT_HEIGHT - uint8_t(dutycycle * (float(height) / 100.0f) + 0.5);
    if (dutyCycleY == TFT_HEIGHT)
        dutyCycleY = TFT_HEIGHT - 1;
    // draw the graph from back to front
    // legend
    uint8_t len = tft.textWidth("100", 1);
    tft.drawFastVLine(graphX, TFT_HEIGHT - height - 4, height + 4, TFT_BLACK);
    tft.drawPixel(graphX, TFT_HEIGHT - int((90 - min) * scale + 0.5), TFT_DARKGREY);
    tft.drawPixel(graphX, TFT_HEIGHT - int((110 - min) * scale + 0.5), TFT_DARKGREY);
    if (initialize)
    {
        tft.drawFastHLine(0, TFT_HEIGHT - int((90 - min) * scale + 0.5), TFT_WIDTH, TFT_DARKGREY);
        tft.drawFastHLine(len, TFT_HEIGHT - int((100 - min) * scale + 0.5), TFT_WIDTH, TFT_DARKGREY);
        tft.drawFastHLine(0, TFT_HEIGHT - int((110 - min) * scale + 0.5), TFT_WIDTH, TFT_DARKGREY);
        tft.drawFastHLine(len, TFT_HEIGHT - int((max - min) * scale + 0.5), TFT_WIDTH, TFT_DARKGREY);
    }
    if (graphX <= len || initialize)
    {
        tft.setTextDatum(BL_DATUM);
        tft.setTextFont(1);
        tft.setTextSize(1);
        tft.setTextColor(TFT_DARKGREY);
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
        tft.drawPixel(graphX, TFT_HEIGHT - int((100 - min) * scale + 0.5), TFT_DARKGREY);
        tft.drawPixel(graphX, TFT_HEIGHT - int((max - min) * scale + 0.5), TFT_DARKGREY);
    }

    if (dutyCycleY > previousDutyY)
    {
        tft.drawFastVLine(graphX, previousDutyY, dutyCycleY - previousDutyY + 1, TFT_GREEN);
    }
    else if (dutyCycleY < previousDutyY)
    {
        tft.drawFastVLine(graphX, dutyCycleY, previousDutyY - dutyCycleY + 1, TFT_GREEN);
    }
    else
    {
        tft.drawPixel(graphX, dutyCycleY, TFT_GREEN);
    }
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
    int uptime_mins = curTime / (1000 * 60);
    int uptime_secs = (curTime / (1000)) % 60;
    tft.printf("%3d:%02d", uptime_mins, uptime_secs);
    tft.setTextDatum(TR_DATUM); // top right]
    char buf[6];
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
    snprintf(buf, 5, "%3d%%", (int)output);
    tft.drawString(buf, TFT_WIDTH / 2, 0);
    tft.drawFastHLine(0, tft.fontHeight(1) + 1, TFT_WIDTH, TFT_ORANGE);

    // degrees thermistor
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextDatum(TL_DATUM);
    tft.setTextFont(2);
    snprintf(buf, 6, "%5.1f", thermistor);
    uint8_t len = tft.drawString(buf, 0, 12);
    // uint8_t len = tft.drawFloat(thermistor, 1, 0, 12);
    tft.setTextFont(1);
    len += tft.drawString("o", len, 12);
    // degrees from thermocouple
    len += 4;
    snprintf(buf, 6, "%5.1f", thermocouple);
    // len += tft.drawFloat(thermocouple, 1, len, 12);
    len += tft.drawString(buf, len, 19);
    len += tft.drawString("c", len, 19);

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
        tft.setTextFont(2);
        tft.drawString(buf, TFT_WIDTH, 12);
        break;
    case SETUP:
        // show all the values, the selection is controlled by moveselecion in the FSM
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setTextFont(1);
        tft.setCursor(10, 40);
        tft.printf("setpoint: %7.2f", setpoint);
        tft.setCursor(10, 50);
        tft.printf("P: %6.3f", Kp);
        tft.setCursor(10, 60);
        tft.printf("I: %6.3f", Ki);
        tft.setCursor(10, 70);
        tft.printf("D: %6.3f", Kd);
        tft.setCursor(10, 80);
        tft.printf("Therm. cal:%7.2f", calibration);
        tft.setCursor(10, 90);
        tft.printf("T offset:%4.1f", temp_offset);
        tft.setCursor(10, 100);
        tft.printf("TC cal:%7.2f", thermocouple_offset);
        tft.setCursor(10, 110);
        tft.printf("overshoot:%5.2f", overshoot);
        tft.setCursor(10, 120);
        tft.printf("pid mode: %d", pidSource);

        // uint8_t fontheight = tft.fontHeight(1);
        if (setupState == QUIT)
        {
            tft.setCursor(10, 140);
            tft.setTextColor(TFT_ORANGE, TFT_BLACK);
            tft.print("save to EEPROM?");

            tft.setCursor(10, 150);
            if ((encoder.getCount() & 0b1) == 0)
                tft.setTextColor(TFT_BLACK, TFT_GREEN);
            else
                tft.setTextColor(TFT_WHITE, TFT_BLACK);
            tft.print("Yes");
            if ((encoder.getCount() & 0b1) == 1)
                tft.setTextColor(TFT_BLACK, TFT_RED);
            else
                tft.setTextColor(TFT_WHITE, TFT_BLACK);
            tft.setCursor(60, 150);
            tft.print("No");
        }
        break;
    }
}

void moveSelection(uint8_t from, uint8_t to, bool visible)
{
    // 0-5: x: 2, y: 42+x*10, cursor 6x6 rect
    tft.fillRect(2, 40 + from * 10, 7, 7, TFT_BLACK);
    if (visible)
        tft.fillRect(2, 40 + to * 10, 7, 7, TFT_RED);
}

double round(double x, int precision)
{
    long factor = pow(10, precision);
    long temp = long(x) * factor;
    return float(temp) / factor;
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

void setup(void)
{
    Serial.begin(115200);
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    pinMode(SSR_PIN, OUTPUT);
    // initialize the TFT
    tft.init();
    tft.setRotation(2);
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
    analogSetAttenuation(ADC_11db);

    if (tuning)
    {
        tuning = false;
        changeAutoTune();
        tuning = true;
    }

    // encoder stuff
    // Enable the weak pull up resistors
    ESP32Encoder::useInternalWeakPullResistors = UP;
    // Attache pins for use as encoder pins
    encoder.attachHalfQuad(ENCODER_DT, ENCODER_CLK);
    encoder.setCount(setpoint * SETPOINT_ENCODER_RESOLUTION);
    ads.setGain(GAIN_ONE);
    ads.begin();
    initGraph = true;
}

void loop(void)
{
    unsigned long looptime = millis();
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
    if (looptime % 20 == 0 || update)
    {
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
                    encoder.setCount(overshoot * 10.0);
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
                overshoot = double(encoder.getCount()) / 10.0;
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
        if (longBack)
        {
            changeAutoTune();
        }
    }
    unsigned long curTime = millis();
    if (curTime - previousDisplayTime >= displayInterval)
    {
        previousDisplayTime = curTime;
        updateDisplay();
    }
    // curTime = millis();
    // gather adc samples before computing the PID vals
    // if (curTime >= startADCSampleTime && curTime - previousADCSampleTime >= ADCSampleInterval)
    // {
    //     previousADCSampleTime = curTime;
    //     uint16_t temp = analogRead(THERMISTOR_PIN);
    //     ThermistorSamples += temp;
    //     Serial.print(temp);
    //     temp = analogRead(THERMOCOUPLE_PIN);
    //     ThermocoupleSamples += temp;
    //     Serial.print(", ");
    //     Serial.println(temp);
    //     currentADCSample++;
    // }
    curTime = millis();
    if (curTime - previousPidTime >= pidInterval)
    {
        previousPidTime = curTime;
        Serial.printf("therm: %d, TC: %d, samples: %d\n", ThermistorSamples, ThermocoupleSamples, currentADCSample);
        startADCSampleTime = (curTime + pidInterval) - NUMSAMPLES * ADCSampleInterval;
        // compute the temps from adc samples
        // float steinhart = float(ThermistorSamples) / float(currentADCSample);
        double steinhart = double(ads.readADC_SingleEnded(0));
        steinhart = (((3.3 / 4.096) * 2047.0) / steinhart) - 1;
        steinhart = SERIESRESISTOR / steinhart;
        steinhart = steinhart / THERMISTORNOMINAL;        // (R/Ro)
        steinhart = log(steinhart);                       // ln(R/Ro)
        steinhart /= BCOEFFICIENT;                        // 1/B * ln(R/Ro)
        steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
        steinhart = 1.0 / steinhart;                      // Invert
        steinhart -= 273.15;                              // convert absolute temp to C
        steinhart += calibration;
        thermistor = steinhart;

        // float thermocouple_voltage = ((float(ThermocoupleSamples) / currentADCSample) * 3.3) / (4095);
        float thermocouple_voltage = float(ads.readADC_SingleEnded(2) * 4.096 / 2047);
        thermocouple = ((thermocouple_voltage - 1.25) / 0.005) + thermocouple_offset;
        switch (pidSource)
        {
        case 0:
            input = thermistor;
            break;
        case 1:
            input = thermocouple;
            break;
        case 2:
            input = (thermistor + thermocouple) / 2;
        }
        input += temp_offset;
        // use more aggressive tunings when far away from setpoint
        // if (!overshootMode && abs(input - setpoint) >= overshoot)
        // {
        //     overshootMode = true;
        //     myPID.SetTunings(aKp, aKi, aKd);
        // }
        // else if (overshootMode && abs(input - setpoint) < overshoot)
        // {
        //     overshootMode = false;
        //     myPID.SetTunings(Kp, Ki, Kd);
        // }

        // myPID.Compute();
        ThermistorSamples = 0;
        ThermocoupleSamples = 0;
        currentADCSample = 0;
    }
    curTime = millis();
    if (curTime - previousGraphTime >= graphInterval)
    {
        previousGraphTime = curTime;
        graphX = (graphX + 1) % TFT_WIDTH;
        // Serial.println(String(input) + ", " + String(output));
        updateGraph(thermistor, thermocouple, output, initGraph);
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
    else
        myPID.Compute();
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
