//
//  main.c
//  cxg-60ewt
//
//  Created by Leonid Mesentsev on 26/11/2019.
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy
//  of this software and associated documentation files (the "Software"), to deal
//  in the Software without restriction, including without limitation the rights
//  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//  copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//  THE SOFTWARE.
//

#include <stm8s.h>
#include <stm8s_pins.h>
#include <main.h>
#include <delay.h>
#include <pwm.h>
#include <s7c.h>
#include <adc.h>
#include <eeprom.h>
#include <clock.h>
#include <menu.h>
#include <buttons.h>

#ifndef F_CPU
#warning "F_CPU not defined, using 16MHz by default"
#define F_CPU 16000000UL
#endif

enum WorkingModes
{
    NORMAL_MODE,
    FORCED_MODE,
    SLEEP_MODE,
    DEEPSLEEP_MODE,
    CALIBRATION_MODE1,
    CALIBRATION_MODE2
};

#define MIN_HEAT 70           // minimum heating temperature
#define MAX_HEAT 450          // maximum heating temperature
#define CALIBRATION_TEMP1 100 // calibration temperature 1
#define CALIBRATION_TEMP2 350 // calibration temperature 2

#define PWM_POWER_OFF 100   // inverted, off heating, 0%
#define PWM_MAX_110_220V 55 // inverted, for heater 110 volt max 45% (220 volt in power outlet)
#define PWM_MAX_220_220V 15 // inverted, for heater 220 volt max 85% (220 volt in power outlet)
#define PWM_MAX_110_110V 15 // inverted, for heater 110 volt max 85% (110 volt in power outlet)
#define PWM_MAX_220_110V 0  // inverted, for heater 220 volt max 100% (110 volt in power outlet)

#define SLEEP_TEMP 100  // sleep temperature, SL1
#define REAL_SECOND 320 // in mainLoop(), 1 second = (localCnt == REAL_SECOND)
#define EEPROM_SAVE_TIMEOUT 2000
#define HEATPOINT_DISPLAY_DELAY 2500

uint32_t _haveToSaveData = 0;
static uint32_t _sleepTimer = 0;
static uint32_t _heatPointDisplayTime = 0;
static uint8_t _currentState = NORMAL_MODE;
int16_t _calibrationADC1;
int16_t _calibrationADC2;
uint8_t error = 0;
int16_t _coefA;
int16_t _coefB;
uint32_t _nowTime;

struct EEPROM_DATA _eepromData;
struct Button _btnPlus = {PB7, 0, 0, 0, 0, 0};
struct Button _btnMinus = {PB6, 0, 0, 0, 0, 0};

void deepSleep();
uint8_t checkSleep(uint32_t nowTime);
void checkHeatPointValidity();
void calcCoef();

void setup()
{
    // Configure the clock for maximum speed on the 16MHz HSI oscillator
    // At startup the clock output is divided by 8
    CLK_CKDIVR = 0x0;
    disable_interrupts();
    TIM4_init();
    enable_interrupts();

    // Configure mercury sensor and button pins
    pinMode(PB5, INPUT);
    pinMode(PB7, INPUT);
    pinMode(PB7, INPUT);

    // Configure 7-segments display
    S7C_init();

    // Configure PWM
    pinMode(PD4, OUTPUT);
    PWM_init(PWM_CH1);
    PWM_duty(PWM_CH1, PWM_POWER_OFF);           // set heater OFF

    _sleepTimer = currentMillis();
    _heatPointDisplayTime = _sleepTimer + HEATPOINT_DISPLAY_DELAY;
    
    // EEPROM
    eeprom_read(EEPROM_START_ADDR, &_eepromData, sizeof(_eepromData));
    // First launch, eeprom empty OR -button pressed when power the device
    if (_eepromData.heatPoint == 0 || ((getPin(PB6) == LOW && getPin(PB7) > 0)))
    {
        while (getPin(PB6) == LOW);             //pending -button release
        if (_eepromData.calibrationADC1 == 0)   //only first launch if eeprom empty
        {
            _eepromData.calibrationADC1 = 64;   // ADC value at CALIBRATION_TEMP1 //my = 64
            _eepromData.calibrationADC2 = 114;  // ADC value at CALIBRATION_TEMP2 //my = 114
        }
        _eepromData.heatPoint = 90;             // default temperature
        _eepromData.enableSound = 1;            // default sound enable
        _eepromData.calibrationValue = 0;       // default calibration constant
        _eepromData.sleepTimeout = 3;           // 3 min, heatPoint 100 C
        _eepromData.deepSleepTimeout = 10;      // 10 min, heatPoint 0 C
        _eepromData.forceModeIncrement = 40;    // 40 degrees
        _eepromData.heaterVoltage = 110;        // voltage heater, 110 or 220 volt
        eeprom_write(EEPROM_START_ADDR, &_eepromData, sizeof(_eepromData));            
    }

    beepAlarm();

    // Press +button when power the device will enter to Setup Menu
    if (getPin(PB7) == LOW && getPin(PB6) > 0)
    {
        while (getPin(PB7) == LOW); //pending +button release
        setup_menu(); 
    }
    
    // The calculation of the coefficients of the equation of the line
    calcCoef();

    // Enter in calibration mode (hold both buttons and turn on the device)
    if (getPin(PB6) == LOW && getPin(PB7) == LOW)
    {
        while (getPin(PB6) == LOW || getPin(PB7) == LOW);       //pending +-button release
        _currentState = CALIBRATION_MODE1;
        _eepromData.heatPoint = CALIBRATION_TEMP1;
        eeprom_write(EEPROM_START_ADDR, &_eepromData, sizeof(_eepromData));
    }
}

void mainLoop()
{
    static uint16_t localCnt = 0;    
    static uint16_t adcValStart = 0; // for error = 4
           
    uint8_t displaySymbol = 0;
    _nowTime = currentMillis();

    // Input power sensor
    static uint16_t oldADCUI = 220;
    uint16_t adcUIn = ADC_read(ADC1_CSR_CH1);
    adcUIn = ((oldADCUI * 7) + adcUIn) >> 3;                    // noise filter
    oldADCUI = adcUIn; // value < 200 for 110 volt in power outlet, value >= 200 for 220 volt in power outlet
    
    // Temperature sensor
    static uint16_t oldADCVal = 50;
    uint16_t adcVal = ADC_read(ADC1_CSR_CH0);
    adcVal = ((oldADCVal * 7) + adcVal) >> 3;                   // noise filter
    oldADCVal = adcVal;

    // Degrees value
    int16_t currentDegrees = (_coefA * adcVal) + _coefB;          //equations of a line, calculation of degrees from ADC
    currentDegrees += _eepromData.calibrationValue;             //correction factor from menu "CAL"
    currentDegrees = (currentDegrees < 0) ? 0 : currentDegrees; //temperature cannot be negative
    
    // ER1: short on sensor
    // ER2: sensor is broken
    // ER3: exceeded maximum temperature
    // ER4: heating element is broken    
    error = (adcVal < 10) ? 1 : (adcVal > 1000) ? 2 : (currentDegrees > (MAX_HEAT + 30)) ? 3 : 0;
    static int8_t flagErr4 = 0;         //for debug flagErr4 = 1
    adcValStart = (localCnt == (REAL_SECOND / 3)) ? adcVal : adcValStart; //wait 333 ms, initial temperature
    if (localCnt == (REAL_SECOND * 4) && !flagErr4 && adcValStart < _calibrationADC1 - 10) //wait 4 sec, once
    {
        flagErr4 = adcVal - adcValStart;
        error = (flagErr4 < 4) ? 4 : 0; //whether the soldering iron is 20 degrees above the initial temperature
        flagErr4 = 1;
    }
    if (error > 0)                                              // if error in device
    {
        PWM_duty(PWM_CH1, PWM_POWER_OFF);                       // switch OFF the heater
        beep();
        localCnt = 0;
        while(1)
        {
            uint8_t flashing = ((localCnt / 300) % 2) ? 1 : 0;  // flashing ER*
            if (!flashing)
            {
                S7C_setChars("ER");
                S7C_setDigit(2, error);
            }
            else
            {
                S7C_blank();
            }
            S7C_refreshDisplay(localCnt++);
            delay_ms(1);

        }
    }

    // Check for sleep
    static uint8_t oldSleepState = 0;
    uint8_t sleepState = checkSleep(_nowTime);
    if (sleepState != oldSleepState)
    {
        beepAlarm();
        _currentState = sleepState;
        oldSleepState = sleepState;
    }

    // Check for buttons
    static uint8_t oldAction = 0;
    uint16_t oldHeatPoint = _eepromData.heatPoint;
    uint8_t action = checkButton(&_btnPlus, &_eepromData.heatPoint, 5, _nowTime) +  // PLUS button
                     checkButton(&_btnMinus, &_eepromData.heatPoint, -5, _nowTime); // MINUS button
    if (action)
    {
        // when any buttons were pressed we will display target temperature
        _heatPointDisplayTime = _nowTime + HEATPOINT_DISPLAY_DELAY;
        if (action != oldAction && action > 1) // two buttons were pressed
        {
            if (_currentState == CALIBRATION_MODE2)
            {
                while (getPin(PB6) == LOW || getPin(PB7) == LOW); //pending +-button release
                _eepromData.calibrationADC2 = adcVal;                
                _eepromData.heatPoint = 90;
                _currentState = FORCED_MODE;
                //eeprom_write(EEPROM_START_ADDR, &_eepromData, sizeof(_eepromData));
                _haveToSaveData = _nowTime;
            }
            if (_currentState == CALIBRATION_MODE1)
            {
                while (getPin(PB6) == LOW || getPin(PB7) == LOW); //pending +-button release
                _eepromData.calibrationADC1 = adcVal;                
                _eepromData.heatPoint = CALIBRATION_TEMP2;
                _currentState = CALIBRATION_MODE2;
                //eeprom_write(EEPROM_START_ADDR, &_eepromData, sizeof(_eepromData));
                _haveToSaveData = _nowTime;
            }
            beepAlarm();
            if (_currentState == CALIBRATION_MODE1 || _currentState == CALIBRATION_MODE2) //calculation of new coefficients when changing calibration values
            {
                // The calculation of the coefficients of the equation of the line
                calcCoef();
                //eeprom_read(EEPROM_START_ADDR, &_eepromData, sizeof(_eepromData));
                //_calibrationADC1 = _eepromData.calibrationADC1;
                //_calibrationADC2 = _eepromData.calibrationADC2;
                //_coefA = (CALIBRATION_TEMP1 - CALIBRATION_TEMP2) / (_calibrationADC1 - _calibrationADC2);
                //_coefB = CALIBRATION_TEMP2 - (_coefA * _calibrationADC2);
            }
            
            if (_currentState != CALIBRATION_MODE1 && _currentState != CALIBRATION_MODE2) //skip forced mode in calibration mode
            {
                _currentState = (_currentState == FORCED_MODE) ? NORMAL_MODE : FORCED_MODE;
            }
        }
        if (oldHeatPoint != _eepromData.heatPoint)
        {
            checkHeatPointValidity();
            _haveToSaveData = _nowTime;
        }
    }
    oldAction = action;

    // Set target temperature
    int16_t targetHeatPoint = 0;
    switch (_currentState)
    {
    case SLEEP_MODE:
        targetHeatPoint = SLEEP_TEMP;
        break;
    case DEEPSLEEP_MODE:
        targetHeatPoint = 0;
        break;
    case FORCED_MODE:
        targetHeatPoint = _eepromData.heatPoint + _eepromData.forceModeIncrement;
        targetHeatPoint = targetHeatPoint > MAX_HEAT ? MAX_HEAT : targetHeatPoint;
        break;
    //case NORMAL_MODE:
    default:
        targetHeatPoint = _eepromData.heatPoint;
    }
    
    // Setup heater
    int16_t pwmValMax = (adcUIn >= 200 && _eepromData.heaterVoltage == 110) ? PWM_MAX_110_220V : (adcUIn >= 200 && _eepromData.heaterVoltage == 220) ? PWM_MAX_220_220V : 
                (adcUIn < 200 && _eepromData.heaterVoltage == 110) ? PWM_MAX_110_110V : (adcUIn < 200 && _eepromData.heaterVoltage == 220) ? PWM_MAX_220_110V :
                PWM_MAX_110_220V;
    // 50 degrees before the heatPoint we start to slow down the heater
    // before that we keep the heater at xx%
    // if the diff is negative, we'll stop the heater
    int16_t diff = targetHeatPoint - currentDegrees;
    int16_t pwmVal = (diff < 0) ? PWM_POWER_OFF : (diff > 50) ? pwmValMax : 75 - diff; 
    pwmVal = (pwmVal < pwmValMax) ? pwmValMax : pwmVal;

    // Soft start heating
    int16_t pwmValSoftHeating = 100 - pwmVal;
    static uint8_t flagSoftHeating = 0;
    if (!flagSoftHeating)
    {
        pwmVal = (localCnt < REAL_SECOND) ? (pwmVal + ((pwmValSoftHeating / 2) + 15)) : 
                        ((localCnt >= REAL_SECOND) && (localCnt < (REAL_SECOND * 2))) ? (pwmVal + (pwmValSoftHeating / 3)) : 
                        ((localCnt >= (REAL_SECOND * 2)) && (localCnt < (REAL_SECOND * 3))) ? (pwmVal + (pwmValSoftHeating / 4)) : 85;
        flagSoftHeating = (localCnt == (REAL_SECOND * 3)) ? 1 : 0;
    }

    // Heat
    PWM_duty(PWM_CH1, pwmVal);
       
    /*// Realtime test for debug
    static uint16_t tick = 0;
    static uint8_t second = 0;
    tick++;
    if (tick == 320)
    {
        tick = 0;
        second++;
    }*/
   
    // Setup display value
    // We will show the current heatPoint
    //   * if any button is pressed
    //   * till _heatPointDisplayTime timeout is reached
    //   * when the current temperature is in range ±10 degrees
    uint16_t displayVal = currentDegrees;
    uint8_t tempInRange = (displayVal >= targetHeatPoint - 15) && (displayVal <= targetHeatPoint + 15) && (_currentState < CALIBRATION_MODE1);
    if (_nowTime < _heatPointDisplayTime || tempInRange)
    {
        displayVal = targetHeatPoint;
        displaySymbol |= SYM_TEMP;
    }

    // Setup status symbol, flashing using local counter overflow
    displaySymbol |= (_currentState >= SLEEP_MODE) && ((localCnt / REAL_SECOND) % 2) ? SYM_MOON : 0;    // 1Hz flashing moon if sleep
    displaySymbol |= pwmVal < 100 && ((localCnt / (REAL_SECOND / 10)) % 2) ? SYM_SUN : 0;               // 10Hz flashing sun if heating
    displaySymbol |= (_currentState == FORCED_MODE) ? SYM_FARS : 0;                                     // F in forced mode
    displaySymbol |= (_currentState >= CALIBRATION_MODE1) ? 107 : 0;                                    // All symbol light in calibration mode

    displaySymbol |= SYM_CELS;
    S7C_setDigit(0, displayVal / 100);
    S7C_setDigit(1, (displayVal / 10) % 10);
    S7C_setDigit(2, displayVal % 10);
    S7C_setSymbol(3, displaySymbol);
    //if (_currentState != DEEPSLEEP_MODE)
    //{
    //    
    //}
    //else
    //{
    //    // Set blank display
    //    S7C_setSymbol(0, 0);
    //    S7C_setSymbol(1, 0);
    //    S7C_setSymbol(2, 0);
    //}
    checkPendingDataSave(_nowTime);
    S7C_refreshDisplay(_nowTime);
    localCnt++;
    delay_ms(1);
}

uint8_t checkSleep(uint32_t nowTime)
{
    static uint8_t oldSensorState = 0;
    uint8_t sensorState = getPin(PB5);
    if (sensorState != oldSensorState)
    {
        _sleepTimer = nowTime;
        oldSensorState = sensorState;
        return NORMAL_MODE;
    }
    else if ((nowTime - _sleepTimer) > _eepromData.deepSleepTimeout * 60000)
    {
        deepSleep();
        return DEEPSLEEP_MODE;
    }
    else if ((nowTime - _sleepTimer) > _eepromData.sleepTimeout * 60000)
    {
        return SLEEP_MODE;
    }
    return NORMAL_MODE;
}

void checkHeatPointValidity()
{
    if (_eepromData.heatPoint > MAX_HEAT)
        _eepromData.heatPoint = MAX_HEAT;
    if (_eepromData.heatPoint < MIN_HEAT)
        _eepromData.heatPoint = MIN_HEAT;
}

void checkPendingDataSave(uint32_t nowTime)
{
    if (_haveToSaveData && (nowTime - _haveToSaveData) > EEPROM_SAVE_TIMEOUT)
    {
        S7C_setSymbol(3, SYM_SAVE);
        eeprom_write(EEPROM_START_ADDR, &_eepromData, sizeof(_eepromData));
        _haveToSaveData = 0;
    }
}

void deepSleep()
{
    static uint16_t localCnt = 0;
    PWM_duty(PWM_CH1, 100); // set heater OFF
    // Set blank display
    S7C_blank();
    while (1)
    {
        uint8_t displaySymbol = ((localCnt / 500) % 2) ? SYM_MOON : 0; // 1Hz flashing moon
        S7C_setSymbol(3, displaySymbol);
        S7C_refreshDisplay(localCnt++);
        delay_ms(1);
    }
}

void calcCoef()     // The calculation of the coefficients of the equation of the line
{
    _calibrationADC1 = _eepromData.calibrationADC1;
    _calibrationADC2 = _eepromData.calibrationADC2;
    _coefA = (CALIBRATION_TEMP1 - CALIBRATION_TEMP2) / (_calibrationADC1 - _calibrationADC2);
    _coefB = CALIBRATION_TEMP2 - (_coefA * _calibrationADC2);
}

void main()
{
    setup();
    while (1)
    {
        mainLoop();
    }
}
