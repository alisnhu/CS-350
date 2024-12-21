/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>


/* Driver Header files */
#include <ti/drivers/GPIO.h>

/* Driver configuration */
#include "ti_drivers_config.h"

//timer
#include <ti/drivers/Timer.h>
//i2c
#include <ti/drivers/I2C.h>
//UART
#include <ti/drivers/UART2.h>
size_t bytesWritten;
#define DISPLAY(x) UART2_write(uart, output, x,&bytesWritten);


// UART global variables
char output[64];
int bytesToSend;
UART2_Handle uart;

void initUART(void)
{
    UART2_Params uartParams;

    // Configure the driver
    UART2_Params_init(&uartParams);
    uartParams.baudRate = 115200;
    // Open the driver
    uart = UART2_open(CONFIG_UART2_0, &uartParams);
    if (uart == NULL)
    {
        /* UART2_open() failed */
        while (1) {}
    }
}

Timer_Handle timer0;
volatile unsigned char TimerFlag = 0;
void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    TimerFlag = 1;
}
void initTimer(void)
{
    Timer_Params params;

    // Init the driver
    Timer_init();

    // Configure the driver
    Timer_Params_init(&params);
    params.period = 100000;                         // Set period to 1/10th of 1 second.
    params.periodUnits = Timer_PERIOD_US;           // Period specified in micro seconds
    params.timerMode = Timer_CONTINUOUS_CALLBACK;   // Timer runs continuously.
    params.timerCallback = timerCallback;           // Calls timerCallback method for timer callback.

    // Open the driver
    timer0 = Timer_open(CONFIG_TIMER_0, &params);
    if (timer0 == NULL)
    {
        /* Failed to initialized timer */
        while (1) {}
    }
    if (Timer_start(timer0) == Timer_STATUS_ERROR)
    {
        /* Failed to start timer */
        while (1) {}
    }
}

// I2C Global Variables
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
} sensors[3] = {
    { 0x48, 0x0000, "11X" },
    { 0x49, 0x0000, "116" },
    { 0x41, 0x0001, "006" }
};
uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;
I2C_Handle i2c;


// Initialize I2C
void initI2C(void)
{
    int8_t i, found;
    I2C_Params i2cParams;

    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "));

    // Init the driver
    I2C_init();

    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL)
    {
        DISPLAY(snprintf(output, 64, "Failed\n\r"));
        while (1);
    }

    DISPLAY(snprintf(output, 32, "Passed\n\r"));

    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses.

    /* Common I2C transaction setup */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;

    found =false;
    for (i=0; i<3; ++i)
    {
         i2cTransaction.targetAddress = sensors[i].address;
         txBuffer[0] = sensors[i].resultReg;

         DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id));
         if (I2C_transfer(i2c, &i2cTransaction))
         {
             DISPLAY(snprintf(output, 64, "Found\n\r"));
             found = true;
             break;
         }
         DISPLAY(snprintf(output, 64, "No\n\r"));
    }

    if(found)
    {
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.targetAddress));
    }
    else
    {
        DISPLAY(snprintf(output, 64, "Temperature sensor not found, contact professor\n\r"));
    }
}
int16_t readTemp(void)
{
    int16_t temperature = 0;
    i2cTransaction.readCount = 2;
    if (I2C_transfer(i2c, &i2cTransaction))
    {
        /*
        * Extract degrees C from the received data;
        * see TMP sensor datasheet
        */
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]); temperature *= 0.0078125;
        /*
        * If the MSB is set '1', then we have a 2's complement * negative value which needs to be sign extended
        */
        if (rxBuffer[0] & 0x80)
        {
            temperature |= 0xF000;
        }
    }
    else
    {
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r",i2cTransaction.status));
        DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"));
    }
    return temperature;
}









/*
++++++++++++++++++++++Defined by me Student++++++++++++++++++++++++++++
*/

//nessesary varaibles
unsigned long second=0;
int16_t myTemperature = 0;
int16_t myTarget = 20;

//define state

enum HEATER_STATES {HEATER_ON,HEATER_OFF,HEATER_INITIAL};
enum BUTTONS_STATES {ICREASE,DECREASE,BUTTONS_INITIAL};
enum BUTTON_STATUS_STATES {PRESSED,RELEASED};
enum TEMPRETURE_STATES{READ,NO_READ,TEMP_INITIAL};

enum HEATER_STATES HEATER_STATE = HEATER_INITIAL;
enum BUTTONS_STATES BUTTONS_STATE = BUTTONS_INITIAL;
enum BUTTON_STATUS_STATES BUTTON_STATUS_STATE = RELEASED;
enum TEMPRETURE_STATES TEMPRETURE_STATE = TEMP_INITIAL;



typedef struct heater
{
    int period ;
    int collapse;
    void (*callback)(void);

} heaterStruct;

typedef struct button
{
    int period ;
    int collapse ;
    void (*callback)(void);
} buttonStruct;


typedef struct temprature
{
    int period ;
    int collapse ;
    void (*callback)(void);
} tempratureStruct;

void buttonCallback()
{
    if(BUTTON_STATUS_STATE==PRESSED)
    {

       if(BUTTONS_STATE==ICREASE)
       {
          myTarget++;
          BUTTON_STATUS_STATE=RELEASED;
          BUTTONS_STATE = BUTTONS_INITIAL;
       }
       else if (BUTTONS_STATE==DECREASE)
       {

            myTarget--;
            BUTTON_STATUS_STATE=RELEASED;
            BUTTONS_STATE = BUTTONS_INITIAL;
       }

    }
}
void tempratureCallback()
{
    //first time use setting up
    if(TEMPRETURE_STATE == TEMP_INITIAL)
    {
        TEMPRETURE_STATE = READ;
    }

    if(TEMPRETURE_STATE ==READ)
    {
         myTemperature = readTemp();    // Sets the current ambient temperature.
         TEMPRETURE_STATE = NO_READ;
    }

}

void heaterCallback()
{

    if(myTarget>myTemperature)
    {
        if(HEATER_STATE != HEATER_ON)
        {
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            HEATER_STATE = HEATER_ON;
        }
    }
    else
    {
        if(HEATER_STATE == HEATER_ON)
        {
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            HEATER_STATE = HEATER_OFF;
        }
    }
    short thisState = (HEATER_STATE==HEATER_ON) ? 1 : 0;
    DISPLAY(snprintf(output, 64,"<%02d,%02d,%d,%04d>\n\r",
                           myTemperature,
                           myTarget,
                           thisState,
                           second));
    second++;
}

heaterStruct myHeater = {
    .period = 10,
    .collapse = 1,
    .callback = &heaterCallback
};

buttonStruct myButton = {
    .period = 2,
    .collapse = 1,
    .callback = &buttonCallback
};

heaterStruct myTemprature = {
    .period = 5,
    .collapse = 1,
    .callback = &tempratureCallback
};




/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{

    //to avid multiple pressed we always control button pressed before and process in 200 ms
    if(BUTTON_STATUS_STATE==RELEASED)
    {
        BUTTON_STATUS_STATE=PRESSED;
        BUTTONS_STATE = DECREASE;
    }

}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    //to avid multiple pressed we always control button pressed before and process in 200 ms
    if(BUTTON_STATUS_STATE==RELEASED)
    {
        BUTTON_STATUS_STATE=PRESSED;
        BUTTONS_STATE = ICREASE;
    }

}

/*
 *  ======== mainThread ========
 */


void initGPIO(void)
{

    /* Call driver init functions */
    GPIO_init();
    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    //GPIO_setConfig(CONFIG_GPIO_LED_1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW); // bu kisim hatayi duzeltmek icin kaldirildi
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1)
    {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }
}




void *mainThread(void *arg0)
{

    initGPIO();
    initUART();
    initI2C();
    initTimer();
    int timer=0;
    int mytick =1;
    while(1)
    {

        //temprature process

        //DISPLAY(snprintf(output, 64, "temprature collapse %02d \n\r",myTemprature.collapse));
        if(myTemprature.period<=myTemprature.collapse)
        {
            //DISPLAY(snprintf(output, 64, "temprature tck  %02d \n\r",mytick));
            myTemprature.callback();
            myTemprature.collapse=1;
        }
        else
        {
            myTemprature.collapse+=1;
        }




        //heater process

        //DISPLAY(snprintf(output, 64, "heater collapse %02d \n\r",myHeater.collapse));
        if(myHeater.period<myHeater.collapse)
        {
            //DISPLAY(snprintf(output, 64, "heater tick %02d \n\r",mytick));
            myHeater.callback();
            myHeater.collapse=1;
        }
        else
        {
            myHeater.collapse+=1;
        }



        //button process
        //DISPLAY(snprintf(output, 64, "button collapse %02d \n\r",myButton.collapse));
        if(myButton.period<=myButton.collapse)
        {
            //DISPLAY(snprintf(output, 64, "button tick %02d \n\r",mytick));
            myButton.callback();
            myButton.collapse=1;
        }
        else
        {
            myButton.collapse+=1;
        }
        mytick++;


        while(!TimerFlag){}
        TimerFlag=0;
        timer++;
    }


    return (NULL);
}
