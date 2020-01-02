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
    DEEPSLEEP_MODE
};

#define MIN_HEAT 50
#define MAX_HEAT 450
#define MAX_ADC_RT 130
#define MIN_ADC_RT 35

#define PWM_POWER_OFF 100
#define PWM_POWER_ON 50
#define SLEEP_TEMP 100
#define EEPROM_SAVE_TIMEOUT 2000
#define HEATPOINT_DISPLAY_DELAY 2000

uint32_t _haveToSaveData = 0;
static uint32_t _sleepTimer = 0;
static uint32_t _heatPointDisplayTime = 0;
static uint8_t _currentState = NORMAL_MODE;

struct EEPROM_DATA _eepromData;
struct Button _btnPlus = {PB7, 0, 0, 0, 0, 0};
struct Button _btnMinus = {PB6, 0, 0, 0, 0, 0};

void deepSleep();
uint8_t checkSleep(uint32_t nowTime);
void checkHeatPointValidity();

void setup()
{
    // Configure the clock for maximum speed on the 16MHz HSI oscillator
    // At startup the clock output is divided by 8
    CLK_CKDIVR = 0x0;
    disable_interrupts();
    TIM4_init();
    enable_interrupts();

    // Configure 7-segments display
    S7C_init();

    // Configure PWM
    pinMode(PD4, OUTPUT);
    PWM_init(PWM_CH1);
    PWM_duty(PWM_CH1, PWM_POWER_OFF); // set heater OFF

    _sleepTimer = currentMillis();
    _heatPointDisplayTime = _sleepTimer + HEATPOINT_DISPLAY_DELAY;
    
    // EEPROM
    eeprom_read(EEPROM_START_ADDR, &_eepromData, sizeof(_eepromData));
    // First launch, eeprom empty OR -button pressed when power the device
    if (_eepromData.heatPoint == 0 || getPin(PB6) == LOW)
    {
        _eepromData.heatPoint = 270;
        _eepromData.enableSound = 1;
        _eepromData.calibrationValue = 0;
        _eepromData.sleepTimeout = 3;       // 3 min, heatPoint 100C
        _eepromData.deepSleepTimeout = 10;  // 10 min, heatPoint 0
        _eepromData.forceModeIncrement = 0; // 0 degrees
        eeprom_write(EEPROM_START_ADDR, &_eepromData, sizeof(_eepromData));            
    }

    beepAlarm();
    // Press +button when power the device will enter to Setup Menu
    if (getPin(PB7) == LOW)
    {
        setup_menu();        
    }
    
    // Now we can switch ON the heater at 50% (value PWM_POWER_ON on define)
    PWM_duty(PWM_CH1, PWM_POWER_ON);
}

void mainLoop()
{
    static uint16_t localCnt = 0;    
    static uint16_t adcValStart = 0;
           
    uint8_t displaySymbol = 0;
    uint32_t nowTime = currentMillis();

    // Input power sensor
    static uint16_t oldADCUI = 0;
    uint16_t adcUIn = ADC_read(ADC1_CSR_CH1);
    adcUIn = ((oldADCUI * 7) + adcUIn) >> 3; // noise filter
    oldADCUI = adcUIn;

    // Temperature sensor
    static uint16_t oldADCVal = 0;
    uint16_t adcVal = ADC_read(ADC1_CSR_CH0);
    adcVal = ((oldADCVal * 7) + adcVal) >> 3; // noise filter
    oldADCVal = adcVal;

    // Degrees value
    adcVal = (adcVal < MIN_ADC_RT) ? MIN_ADC_RT : adcVal;
    int16_t currentDegrees = (MAX_HEAT - MIN_HEAT) * (adcVal - MIN_ADC_RT) / (MAX_ADC_RT - MIN_ADC_RT);
    currentDegrees += _eepromData.calibrationValue;    
    currentDegrees = (currentDegrees < 0) ? 0 : currentDegrees;
    
    // ER1: short on sensor
    // ER2: sensor is broken
    // ER4: heating element is broken    
    uint8_t error = (adcVal < 10) ? 1 : (adcVal > 1000) ? 2 : 0;    
    adcValStart = (localCnt == 333) ? adcVal : adcValStart; //wait 300 ms, initial temperature
    static int8_t flagError4 = 0;
    if (localCnt == 3333 && !flagError4) //wait 3 sec, once
    {
        flagError4 = adcVal - adcValStart;
        error = (flagError4 < 4) ? 4 : 0; //whether the soldering iron is 20 degrees above the initial temperature
        flagError4 = 1;
    }
    if (error)
    {
        PWM_duty(PWM_CH1, PWM_POWER_OFF); // switch OFF the heater
        S7C_setChars("ER");
        S7C_setDigit(2, error);
        S7C_refreshDisplay(nowTime);
        beep();
        return;
    }

    // Check for sleep
    static uint8_t oldSleepState = 0;
    uint8_t sleepState = checkSleep(nowTime);
    if (sleepState != oldSleepState)
    {
        beepAlarm();
        _currentState = sleepState;
        oldSleepState = sleepState;
    }

    // Check for buttons
    static uint8_t oldAction = 0;
    uint16_t oldHeatPoint = _eepromData.heatPoint;
    uint8_t action = checkButton(&_btnPlus, &_eepromData.heatPoint, 1, nowTime) +  // ADD button
                     checkButton(&_btnMinus, &_eepromData.heatPoint, -1, nowTime); // MINUS button
    if (action)
    {
        // when any buttons were pressed we will display target temperature
        _heatPointDisplayTime = nowTime + HEATPOINT_DISPLAY_DELAY;
        if (action != oldAction && action > 1) // two butons were pressed
        {
            beepAlarm();            
            _currentState = (_currentState == FORCED_MODE) ? NORMAL_MODE : FORCED_MODE;
        }
        if (oldHeatPoint != _eepromData.heatPoint)
        {
            checkHeatPointValidity();
            _haveToSaveData = nowTime;
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
    case NORMAL_MODE:
    default:
        targetHeatPoint = _eepromData.heatPoint;
    }

    // Setup heater
    // 50 degrees before the heatPoint we start to slow down the heater
    // before that we keep the heater at 50%
    // if the diff is negative, we'll stop the heater
    int16_t diff = targetHeatPoint - currentDegrees;
    int16_t pwmVal = (diff < 0) ? PWM_POWER_OFF : (diff > 50) ? PWM_POWER_ON : 90 - diff;
    PWM_duty(PWM_CH1, pwmVal);

    // Setup display value
    // We will show the current heatPoint
    //   * if any button is pressed
    //   * till _heatPointDisplayTime timeout is reached
    //   * when the current temperature is in range ±10 degrees
    uint16_t displayVal = currentDegrees;
    uint8_t tempInRange = (displayVal >= targetHeatPoint - 10) && (displayVal <= targetHeatPoint + 10);
    if (nowTime < _heatPointDisplayTime || tempInRange)
    {
        displayVal = targetHeatPoint;
        displaySymbol |= SYM_TEMP;
    }

    // Setup status symbol, flashing using local counter overflow
    displaySymbol |= (_currentState >= SLEEP_MODE) && ((localCnt / 500) % 2) ? SYM_MOON : 0; // 1Hz flashing moon
    displaySymbol |= pwmVal < 100 && ((localCnt / 50) % 2) ? SYM_SUN : 0;                    // 10Hz flashing heater
    displaySymbol |= (_currentState == FORCED_MODE) ? SYM_FARS : 0;                          // F in forced mode

    if (_currentState != DEEPSLEEP_MODE)
    {
        displaySymbol |= SYM_CELS;
        S7C_setDigit(0, displayVal / 100);
        S7C_setDigit(1, (displayVal / 10) % 10);
        S7C_setDigit(2, displayVal % 10);
    }
    else
    {
        // Set blank display
        S7C_setSymbol(0, 0);
        S7C_setSymbol(1, 0);
        S7C_setSymbol(2, 0);
    }
    S7C_setSymbol(3, displaySymbol);

    checkPendingDataSave(nowTime);
    S7C_refreshDisplay(nowTime);
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
    S7C_setSymbol(0, 0);
    S7C_setSymbol(1, 0);
    S7C_setSymbol(2, 0);
    while (1)
    {
        uint8_t displaySymbol = ((localCnt / 500) % 2) ? SYM_MOON : 0; // 1Hz flashing moon
        S7C_setSymbol(3, displaySymbol);
        S7C_refreshDisplay(localCnt++);
        delay_ms(1);
    }
}

void main()
{
    setup();
    while (1)
    {
        mainLoop();
    }
}
