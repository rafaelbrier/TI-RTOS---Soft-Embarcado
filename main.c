/*
 * RFID---------------------------
 * SDA / CS / FSS ------------ PB5
 * SCK  / CLK     ------------ PB4
 * MOSI / TX      ------------ PB7
 * MISO /  RX     ------------ PB6
 * RST            ------------ PF0 (PUSH 2)
 * --------------------------------
 *
 * BlueTooth-----------------------
 * RX ----------------------- PB1
 * TX ----------------------- PB0
 * ---------------------------------
 *
 * LCD-----------------------------
 *  * Pin Connections:
 * PC4 [pin 1]  -> RS
 * PC5 [pin 0]  -> EN
 * PD0 [pin 23] -> D4
 * PD1 [pin 24] -> D5
 * PD2 [pin 25] -> D6
 * PD3 [pin 26] -> D7
 *---------------------------------
 *
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Header files */
// #include <ti/drivers/GPIO.h>
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
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "tm4c123gh6pm.h"

/*Libs User*/
#include "lib/display.h"
#include "lib/Mfrc522.h"

#define redLED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define blueLED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define greenLED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))

#define TASKSTACKSIZE   512

void initUart1();
void initLeds();
void writeStringToUart1(char* str);
int uartEchoReceivedString();
void initButton1();

Task_Struct taskButtonStruct;
Task_Struct taskCardStruct;
Task_Struct taskBluetoothStruct;
Task_Struct taskGetResponseStruct;
Task_Struct taskValvulaStruct;

Char taskStack[TASKSTACKSIZE];

int index;
char rxChar[10];
char strrTeste[10];
uint32_t i, ii;

Void taskButtonFunc(UArg arg0, UArg arg1)
{
    while (1) {
        if(GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4)==0x00){
            System_printf("Botao 1 apertado . \n");
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
        if(uartEchoReceivedString()){
                  System_printf("String Recebida (TRUE). \n");
                  System_flush();
              }
              /* for test *///////////////
              for(i=0; i<=100; i++){
                 for(ii=0; ii<=15000000; ii++){
                 }
                 sprintf(strrTeste, "teste: %d", i); // puts string into buffer
                 writeStringToUart1(strrTeste);
             }
             /////////////////////////////
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
    initButton1();
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

    System_printf("Programa Iniciado.");
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}

//==================================================================================================


void initUart1(){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1); //habilita periférico UART1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); //habilita periférico GPIOB

    GPIOPinConfigure(GPIO_PB0_U1RX); //Pin PB0 definido como RECEIVER
    GPIOPinConfigure(GPIO_PB1_U1TX); //Pin PB1 definido como TRANSMITER
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 9600,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    System_printf("UART1 (BlueTooth) Iniciado.");
    System_flush();
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
    System_printf("Leds iniciados.");
    System_flush();
}

void initButton1(){
       //Iniciar Botao 1 apenas--------------------------------------------------------------
       //Configuração dos botões. Desbloqueio e Estado Pull-Up
        HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
        HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
        HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
        //Pin4 do PORTF são os botões, aqui os define como INPUT
        GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_DIR_MODE_IN);
        //Define o pino 4 PORTF como PULL-UP
        GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
        //----------------------------------------------------------------------------------
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
