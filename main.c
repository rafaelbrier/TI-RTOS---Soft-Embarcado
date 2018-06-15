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

/*Libs*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "tm4c123gh6pm.h"

#define redLED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define blueLED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define greenLED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))


#define TASKSTACKSIZE   512

void initUart1();
void initLeds();
void writeStringToUart1(char* str);
int uartEchoReceivedString();

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


int index;
char rxChar[10];

Void taskButtonFunc(UArg arg0, UArg arg1)
{
    while (1) {
        if(uartEchoReceivedString()){
            System_printf("String Recebida (TRUE)");
            System_flush();
        }
    }
}

Void taskCardFunc(UArg arg0, UArg arg1)
{
    while (1) {
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
    /* Call board init functions *///Board_initGeneral();// Board_initGPIO(); // Board_initI2C();
    // Board_initSDSPI();// Board_initSPI();// Board_initUART();// Board_initUSB(Board_USBDEVICE);
    // Board_initWatchdog();// Board_initWiFi();
    initLeds();
    initUart1();

    /* Construct heartBeat Task  thread */
    Task_Params taskParams;

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


void initUart1(){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1); //habilita periférico UART0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); //habilita periférico GPIOA

    GPIOPinConfigure(GPIO_PB0_U1RX); //Pin PA0 definido como RECEIVER
    GPIOPinConfigure(GPIO_PB1_U1TX); //Pin PA1 definido como TRANSMITER
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 9600,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

}


void writeStringToUart1(char* str)   //write a string to Uart1
{
    int i;
    for (i = 0; i < strlen(str); i++) {
        UARTCharPut(UART1_BASE, str[i]);
    }
}

void initLeds(){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
}

int uartEchoReceivedString(){
    while(UARTCharsAvail(UART1_BASE)) //loop while there are chars
    {
        rxChar[index] = UART1_DR_R;
        if(rxChar[index-1]==13)
        {
            rxChar[index-1]='\0';
            if(strcmp(rxChar,"red")==0)
                redLED^=1;
            if(strcmp(rxChar,"blue")==0)
                blueLED^=1;
            if(strcmp(rxChar,"green")==0)
                greenLED^=1;

            index=0;
            writeStringToUart1(rxChar);
            return 1;
        }
        else {
            index++;
        }
    }
    return 0;
}

