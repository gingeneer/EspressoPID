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
#define BACK_BUTTON_PIN 14
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
// #define SERIESRESISTOR 99000
// 10k better for this temp range
#define SERIESRESISTOR 9850
// how many speps per degree
#define SETPOINT_ENCODER_RESOLUTION 2.0
#define PID_ENCODER_RESOLUTION 100.0

#define EEPROM_SIZE 1000
int samples[NUMSAMPLES];

// uint16_t tempArray[TFT_WIDTH];
// uint8_t outputArray[TFT_WIDTH];
uint8_t previousTempY = 0;
uint8_t previousDutyY = 40;

ESP32Encoder encoder;

TFT_eSPI tft = TFT_eSPI(); // Invoke library, pins defined in User_Setup.h

//Define Variables we'll be connecting to PID
double setpoint = 105, input, output;
double calibration = -2.3;
//offset for display (what boiler temp equals which water temp)
double temp_offset = 0;
//Specify the links and initial tuning parameters
double Kp = 2, Ki = 5, Kd = 1;
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
// how long is one period of SSR control in ms
int windowSize = 250;
unsigned long windowStartTime;
unsigned long shotTimerStart;
unsigned long shotTimerStop;
bool timerRunning;
bool initGraph = false;

uint8_t menu = 0;
uint8_t state;
uint8_t previousState = 0;
uint8_t previousMenu = 0;
uint8_t graphX = 0;
uint8_t selection = 0;
bool longPressPossible = false;

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
    EEPROM.commit();
}

void updateGraph(double temp, float dutycycle, bool initialize)
{
    if (menu != 0)
        return;
    // min/max temp
    const uint8_t min = 80;
    const uint8_t max = 120;
    // height of graphing area
    const uint8_t height = 120;
    // how many pixel per temp
    float scale = float(height) / (float(max) - min);
    uint8_t nextIndex = (graphX + 1) % TFT_WIDTH;
    uint8_t tempY = TFT_HEIGHT - uint8_t(((temp - min) * scale + 0.5));
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
    if (temp >= min && temp <= max)
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
    tft.drawFastVLine(nextIndex, TFT_HEIGHT - height - 4, height + 4, TFT_WHITE);
    previousDutyY = dutyCycleY;
    previousTempY = tempY;
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
    snprintf(buf, 5, "%3d%%", (int)output);
    tft.drawString(buf, TFT_WIDTH / 2, 0);
    tft.drawFastHLine(0, tft.fontHeight(1) + 1, TFT_WIDTH, TFT_ORANGE);

    // degrees
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextDatum(TL_DATUM);
    tft.setTextFont(2);
    uint8_t len = tft.drawFloat(input, 1, 0, 12);
    tft.setTextFont(1);
    tft.drawString("o", len, 12);

    switch (menu)
    {
    case 0:
        // cant do that...
        // tft.fillRect(0, 12, TFT_WIDTH, TFT_HEIGHT, TFT_BLACK);
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
        break;
    case 1:
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
        // uint8_t fontheight = tft.fontHeight(1);
        if (state == 6)
        {
           
            tft.setCursor(10, 110);
            tft.setTextColor(TFT_ORANGE, TFT_BLACK);
            tft.print("save to EEPROM?");
            
            tft.setCursor(10, 120);
            if ((encoder.getCount() & 0b1) == 0)
                tft.setTextColor(TFT_BLACK, TFT_GREEN);
            else
                tft.setTextColor(TFT_WHITE, TFT_BLACK);
            tft.print("Yes");
            if ((encoder.getCount() & 0b1) == 1)
                tft.setTextColor(TFT_BLACK, TFT_RED);
            else
                tft.setTextColor(TFT_WHITE, TFT_BLACK);
            tft.setCursor(60, 120);
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
    encoder.attachHalfQuad(ENCODER_DT, ENCODER_CLK);
    encoder.setCount(setpoint * SETPOINT_ENCODER_RESOLUTION);
   
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
            update=true;
            backPressed = true;
        }
    }
    // check for longpress
    if (encoderBounce.read() == 0 && encoderBounce.duration() > 3000 && longPressPossible)
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
            setpoint = ((double)encoder.getCount()) / SETPOINT_ENCODER_RESOLUTION;
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
                tft.fillScreen(TFT_BLACK);
                moveSelection(0, 0, true);
                menu = 1;
                state = 0;
                encoder.setCount(setpoint * SETPOINT_ENCODER_RESOLUTION);
            }
            break;
        case 1:
            // screen to config the values
            switch (state)
            {
            case 0:
                // roatary encoder controlls the setpoint
                // if (previousState != 0 || previousMenu != menu)
                // {
                //     encoder.setCount(setpoint * SETPOINT_ENCODER_RESOLUTION);
                // }
                setpoint = ((double)encoder.getCount()) / SETPOINT_ENCODER_RESOLUTION;
                if (selectPressed)
                {
                    state = 1;
                    encoder.setCount(Kp * PID_ENCODER_RESOLUTION);
                    moveSelection(0, 1, true);

                }
                if (backPressed)
                {
                    state = 6;
                    encoder.setCount(0);
                    moveSelection(0, 0, false);
                }
                break;
            case 1:
                if (Kp != (((double)encoder.getCount()) / PID_ENCODER_RESOLUTION))
                {
                    Kp = ((double)encoder.getCount()) / PID_ENCODER_RESOLUTION;
                    myPID.SetTunings(Kp, Ki, Kd);
                }
                if (selectPressed)
                {
                    state = 2;
                    encoder.setCount(Ki * PID_ENCODER_RESOLUTION);
                    moveSelection(1, 2, true);
                }
                if (backPressed)
                {
                    state = 6;
                    encoder.setCount(0);
                    moveSelection(0, 0, false);
                }
                break;
            case 2:
                if (Ki != (((double)encoder.getCount()) / PID_ENCODER_RESOLUTION))
                {
                    Ki = ((double)encoder.getCount()) / PID_ENCODER_RESOLUTION;
                    myPID.SetTunings(Kp, Ki, Kd);
                }
                if (selectPressed)
                {
                    state = 3;
                    encoder.setCount(Kd * PID_ENCODER_RESOLUTION);
                    moveSelection(2, 3, true);
                }
                if (backPressed)
                {
                    state = 6;
                    encoder.setCount(0);
                    moveSelection(0, 0, false);
                }
                break;
            case 3:
                if (Kd != (((double)encoder.getCount()) / PID_ENCODER_RESOLUTION))
                {
                    Kd = ((double)encoder.getCount()) / PID_ENCODER_RESOLUTION;
                    myPID.SetTunings(Kp, Ki, Kd);
                }
                if (selectPressed)
                {
                    state = 4;
                    encoder.setCount(calibration * 20.0);
                    moveSelection(3, 4, true);
                }
                if (backPressed)
                {
                    state = 6;
                    encoder.setCount(0);
                    moveSelection(0, 0, false);
                }
                break;
            case 4:
                calibration = ((double)encoder.getCount()) / 20.0;
                if (selectPressed)
                {
                    encoder.setCount(temp_offset * 10.0);
                    state = 5;
                    moveSelection(4, 5, true);
                }
                if (backPressed)
                {
                    state = 6;
                    encoder.setCount(0);
                    moveSelection(0, 0, false);
                }
                break;
            case 5:
                temp_offset = ((double)encoder.getCount()) / 10.0;
                if (selectPressed)
                {
                    encoder.setCount(setpoint * SETPOINT_ENCODER_RESOLUTION);
                    state = 0;
                    moveSelection(5, 0, true);
                }
                if (backPressed)
                {
                    state = 6;
                    encoder.setCount(0);
                    moveSelection(0, 0, false);
                }
                break;
            case 6:
                if (selectPressed)
                {
                    if ((encoder.getCount() & 0b1) == 0)
                    {
                        saveValues();
                    }
                    menu = 0;
                    state = 0;
                    encoder.setCount(setpoint * SETPOINT_ENCODER_RESOLUTION);
                    tft.fillScreen(TFT_BLACK);
                    initGraph = true;
                    graphX = 0;
                }
                if (backPressed)
                {
                    encoder.setCount(setpoint * SETPOINT_ENCODER_RESOLUTION);
                    state = 0;
                    tft.fillScreen(TFT_BLACK);
                    moveSelection(0, 0, true);
                }
                break;
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
        input = steinhart + temp_offset;
        myPID.Compute();
        updateDisplay();
    }
    // save to the array for the graphics
    if (looptime % 500 == 0)
    {
        graphX = (graphX + 1) % TFT_WIDTH;
        Serial.println(String(input) + ", " + String(output));
        updateGraph(input, output, initGraph);
        initGraph = false;
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
