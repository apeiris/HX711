/*
   -------------------------------------------------------------------------------------
   HX711_ADC
   Arduino library for HX711 24-Bit Analog-to-Digital Converter for Weight Scales
   Olav Kallhovd sept2017
   -------------------------------------------------------------------------------------
*/
/*
   Settling time (number of samples) and data filtering can be adjusted in the config.h file
   For calibration and storing the calibration value in eeprom, see example file "Calibration.ino"

   The update() function checks for new data and starts the next conversion. In order to acheive maximum effective
   sample rate, update() should be called at least as often as the HX711 sample rate; >10Hz@10SPS, >80Hz@80SPS.

   This example shows how call the update() function from an ISR with interrupt on the dout pin.
   Try this if you experince longer settling time due to time consuming code in the loop(),
   i.e. if you are refreshing an graphical LCD, etc.
   The pin used for dout must be external interrupt capable.
*/

#include <HX711_ADC.h>
#if defined(ESP8266) || defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif
#include "esp32-hal-timer.h"
// declarations for timer
volatile int interruptCounter;
int totalInterruptCounter;

const int HX711_dout = 18; // mcu > HX711 dout pin, must be external interrupt capable!
const int HX711_sck = 19;  // mcu > HX711 sck pin

// HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);

const int calVal_eepromAdress = 0;
unsigned long t = 0;
volatile boolean newDataReady;
bool signled = 0;
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
 
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  signled = 1;
  interruptCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
 
}

// interrupt routine:
void dataReadyISR()
{
    if (LoadCell.update())
    {
        newDataReady = 1;
    }
}

void setup()
{
    Serial.begin(115200);
    delay(10);
    Serial.println();
    Serial.println("Interrupt.cpp is starting...");

     timer = timerBegin(0, 80, true);
     timerAttachInterrupt(timer, &onTimer, true);
     timerAlarmWrite(timer, 100000, true);
      timerAlarmEnable(timer);

    float calibrationValue;     // calibration value
  //  calibrationValue = -900.57; // uncomment this if you want to set this value in the sketch
 //   calibrationValue= -891.90;
  calibrationValue = -857.72;// 1kg cell @5v
#if defined(ESP8266) || defined(ESP32)
    // EEPROM.begin(512); // uncomment this if you use ESP8266 and want to fetch the value from eeprom
#endif
    // EEPROM.get(calVal_eepromAdress, calibrationValue); // uncomment this if you want to fetch the value from eeprom

    LoadCell.begin();
    // LoadCell.setReverseOutput();
    unsigned long stabilizingtime = 7000; // tare preciscion can be improved by adding a few seconds of stabilizing time
    boolean _tare = true;                 // set this to false if you don't want tare to be performed in the next step
    LoadCell.start(stabilizingtime, _tare);
    if (LoadCell.getTareTimeoutFlag())
    {
        printf("Timeout, check MCU>HX711 wiring and pin designations\n");
        while (1)
            ;
    }
    else
    {
        LoadCell.setCalFactor(calibrationValue); // set calibration value (float)
        printf("Startup is complete, sample size = %i\n", SAMPLES);
    }

    attachInterrupt(digitalPinToInterrupt(HX711_dout), dataReadyISR, FALLING);
}

float weigh()
{
    const int serialPrintInterval = 0; // increase value to slow down serial print activity
    signled=false;
    timerAlarmDisable(timer);
    timerAlarmWrite(timer,700000,true);
    while (!signled) 
    do {}
    while(!newDataReady);
        float i = LoadCell.getData();
        newDataReady = 0;
        return i;
    

}
#define noop
void loop()
{

    if (Serial.available() > 0)
    {
        char inByte = Serial.read();
        if (inByte == 'w')
        {
            timerAlarmDisable(timer);
            newDataReady = 0;
            signled=false;
          //  timerAlarmWrite(timer, 7000, true);
         
            do {
             printf("*");
            }
            while  (!signled);
            printf("\n\nMeasuring \n");
          



           

            float i = weigh();
            printf("Loadcell Value=%.1lf time %lu tare complete=%i\n", i, millis() - t,1);
            t = millis();
        }
        // receive command from serial terminal, send 't' to initiate tare operation:
        else if (inByte == 't')
        {
            t=millis();
            printf("Tare begun, ");
            LoadCell.tareNoDelay();
            do
            {
             } while (!LoadCell.getTareStatus());
            
            printf("completed ,elapsed ms= %i\n",millis()-t);
        }
    }
}