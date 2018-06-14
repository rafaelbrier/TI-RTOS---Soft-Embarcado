/*
 *  ======== main.c ========
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
// #include <ti/drivers/I2C.h>
// #include <ti/drivers/SDSPI.h>
// #include <ti/drivers/SPI.h>
// #include <ti/drivers/UART.h>
// #include <ti/drivers/Watchdog.h>
// #include <ti/drivers/WiFi.h>

/* Board Header file */
#include "Board.h"

#define TASKSTACKSIZE   512

Task_Struct taskButtonStruct;
Task_Struct taskCardStruct;
Task_Struct taskBluetoothStruct;
Task_Struct taskGetResponseStruct;
Task_Struct taskValvulaStruct;

Char taskStack[TASKSTACKSIZE];
//Char taskCard[TASKSTACKSIZE];
//Char taskBluetooth[TASKSTACKSIZE];
//Char taskGetResponse[TASKSTACKSIZE];
//Char taskValvula[TASKSTACKSIZE];

/*
 *  ======== heartBeatFxn ========
 *  Toggle the Board_LED0. The Task_sleep is determined by arg0 which
 *  is configured for the heartBeat Task instance.
 */

uint32_t i, ii;
Void taskButtonFunc(UArg arg0, UArg arg1)
{
    while (1) {
    for(i=0; i<=100; i++){
    for(ii=0; ii<=100000; ii++){
    }
    GPIO_toggle(Board_LED0);
}
    }}

Void taskCardFunc(UArg arg0, UArg arg1)
{
    while (1) {
        Task_sleep(1000);
         GPIO_toggle(Board_LED1);

    }
}

Void taskBluetoothFunc(UArg arg0, UArg arg1)
{
    while (1) {

    }
}

Void taskGetResponseFunc(UArg arg0, UArg arg1)
{
    while (1) {

    }
}

Void taskValvulaFunc(UArg arg0, UArg arg1)
{
    while (1) {

    }
}

/*
 *  ======== main ========
 */
int main(void)
{
    Task_Params taskParams;

    /* Call board init functions */
    Board_initGeneral();
    Board_initGPIO();
    // Board_initI2C();
    // Board_initSDSPI();
    // Board_initSPI();
    // Board_initUART();
    // Board_initUSB(Board_USBDEVICE);
    // Board_initWatchdog();
    // Board_initWiFi();

    /* Construct heartBeat Task  thread */
    Task_Params_init(&taskParams);
    taskParams.arg0 = 1000;
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &taskStack;

    Task_construct(&taskButtonStruct, (Task_FuncPtr)taskButtonFunc, &taskParams, NULL);
    Task_construct(&taskCardStruct, (Task_FuncPtr)taskCardFunc, &taskParams, NULL);
    Task_construct(&taskBluetoothStruct, (Task_FuncPtr)taskBluetoothFunc, &taskParams, NULL);
    Task_construct(&taskGetResponseStruct, (Task_FuncPtr)taskGetResponseFunc, &taskParams, NULL);
    Task_construct(&taskValvulaStruct, (Task_FuncPtr)taskValvulaFunc, &taskParams, NULL);


    /* Turn on user LED */
   // GPIO_write(Board_LED0, Board_LED_ON);

    System_printf("Starting the example\nSystem provider is set to SysMin. "
                  "Halt the target to view any SysMin contents in ROV.\n");
    /* SysMin will only print to the console when you call flush or exit */
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
