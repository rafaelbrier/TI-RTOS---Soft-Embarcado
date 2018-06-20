/*
 * Buttons (3x)--------------------
 * PE1
 * PE2
 * PE3
 * --------------------------------
 *
 * BUZZER ------------------------
 * PE4 *
 * -------------------------------
 *
 * RELÉ--------------------------
 * PE5, PA6, PA7
 * colocar os 3 pra selecionar com Jumper
 * (segurança)
 * -------------------------------
 *
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
#include <xdc/cfg/global.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/hal/Timer.h>

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
#include "driverlib/ssi.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "tm4c123gh6pm.h"

/*Libs User*/
#include "lib/display.h"
#include "lib/Mfrc522.h"

//#define redLED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
//#define blueLED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
//#define greenLED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define redLED   0x00000002
#define blueLED  0x00000004
#define greenLED 0x00000008

#define TASKSTACKSIZE   512
#define CARD_LENGTH 5

int chipSelectPin = 0x20;  //PB5
int NRSTPD = 0x01; //PF0

int index;
int countOverFlow;
char rxChar[10];
char strrTeste[10];
uint32_t i, ii;
uint8_t Version;
uint8_t buttonPressed;
uint8_t AntennaGain;
uint8_t status;
uint32_t readTeste;
unsigned char str[MAX_LEN];
unsigned char cardID[CARD_LENGTH];

//Library modified to work with CCS
#ifdef __cplusplus
Mfrc522 Mfrc522(chipSelectPin, NRSTPD);
#endif

//Functions
void initUart1();
void initLeds();
void writeStringToUart1(char* str);
int uartEchoReceivedString();
void initButton1();
void dumpHex(unsigned char* buffer, int len);
void InitSSI();

//Tasks----------------------------------------------------------------------------------

extern "C" {
Void taskButtonFunc(UArg arg0, UArg arg1)
{ //  Semaphore_post(semphButton);
    System_printf("Task Button em primeira execução. \n");
    while (1) {
        if(GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4)==0x00){
            System_printf("Botao 1 apertado . \n");
            buttonPressed = 1;
            Semaphore_pend(semphButton, BIOS_WAIT_FOREVER); //Aguarda o Semaforo Botao
        }
    }
}

Void taskCardFunc(UArg arg0, UArg arg1)
{
    System_printf("Task Card em primeira execução. \n");
    countOverFlow=0;
    Clock_start(timeOutClock);

    while(1){
        if(countOverFlow==0) {
            status = Mfrc522.Request(PICC_REQIDL, str);
            if(status == MI_OK){
                System_printf("Cartao Detectado! \n"); //Card Detected
                GPIOPinWrite(GPIO_PORTF_BASE, blueLED, blueLED);

                status = Mfrc522.Anticoll(str);
                memcpy(cardID, str, CARD_LENGTH);

                if(status == MI_OK){
                    System_printf("ID: ");
                    dumpHex((unsigned char*)cardID, CARD_LENGTH);
                    GPIOPinWrite(GPIO_PORTF_BASE, blueLED, 0);
                    Semaphore_pend(semphCard, BIOS_WAIT_FOREVER); //Aguarda o Semaforo Botao
                } else {
                    System_printf("Não foi possível ler o cartão. Favor segurar mais tempo. \n");
                    GPIOPinWrite(GPIO_PORTF_BASE, blueLED, 0);
                }
            }
        } else if (countOverFlow==1) {
            /*Se o timer estourar libera o semáforo da primeira Task. Aguarda o cartão por 10 seg apenas.*/
            Clock_stop(timeOutClock);
            System_printf("Tempo expirado. \n");
            Semaphore_post(semphButton);
            /*Para voltar execução*/
            countOverFlow=0;
            Clock_start(timeOutClock);
        }
    }
}


Void taskBluetoothFunc(UArg arg0, UArg arg1)
{
    System_printf("Task Bluetooth em primeira execução. \n");
    while (1) {
        if(uartEchoReceivedString()){
            System_printf("String Recebida (TRUE). \n");
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
    System_printf("Task GetResponse em primeira execução. \n");
    while (1) {
        if(uartEchoReceivedString()){
            System_printf("String Recebida (TRUE). \n");
        }
    }
}

Void taskValvulaFunc(UArg arg0, UArg arg1)
{
    System_printf("Task Valvula em primeira execução. \n");
    while (1) {

    }
}
Void timeOutClockFunc(UArg arg0){
    //Clock_stop(timeOutClock);
    countOverFlow=1;
}
}//Fim Extern C

/*
 *  ======== main ========
 */
int main(void)
{
    initLeds();
    initButton1();
    initUart1();
    InitSSI();

    GPIOPinWrite(GPIO_PORTB_BASE, chipSelectPin, 0);
    GPIOPinWrite(GPIO_PORTF_BASE, NRSTPD, NRSTPD);

    Mfrc522.Init();

    Version = Mfrc522.ReadReg(VersionReg);
    AntennaGain = Mfrc522.ReadReg(PICC_REQIDL) & (0x07<<4);

    System_printf("Version: '0x%x' \n", Version);
    System_printf("Antenna Gain: '0x%x' \n\n", AntennaGain);

    System_printf("Programa Iniciado.\n\n");

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

    System_printf("UART1 (BlueTooth) Iniciado.\n");
}

void InitSSI(){
    uint32_t junkAuxVar;

    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); //SDA
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); //reset

    GPIOPinConfigure(GPIO_PB4_SSI2CLK);
    //GPIOPinConfigure(GPIO_PB5_SSI2FSS);
    GPIOPinConfigure(GPIO_PB6_SSI2RX);
    GPIOPinConfigure(GPIO_PB7_SSI2TX);

    GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_6 |
                   GPIO_PIN_7);

    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5); //chipSelectPin
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0); //NRSTPD

    //
    SSIConfigSetExpClk(SSI2_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 4000000, 8);
    //
    // Enable the SSI0 module.
    //
    SSIEnable(SSI2_BASE);

    while(SSIDataGetNonBlocking(SSI2_BASE, &junkAuxVar)){}

    System_printf("SSI Enabled! SPI Mode!  \nData: 8bits.\n\n");
    System_flush();
}

void initLeds(){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    System_printf("Leds iniciados.\n");
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

void writeStringToUart1(char* str)   //write a string to Uart1
{
    int i;
    for (i = 0; i < strlen(str); i++) {
        UARTCharPut(UART1_BASE, str[i]);
    }
}

void dumpHex(unsigned char* buffer, int len){
    int i;

    System_printf(" ");
    for(i=0; i < len; i++) {
        System_printf("%x-", buffer[i]);
    }
    System_printf("\n");
}

int uartEchoReceivedString(){
    while(UARTCharsAvail(UART1_BASE)) //loop while there are chars
    {
        rxChar[index] = UART1_DR_R;
        if(rxChar[index-1]==13)
        {
            rxChar[index-1]='\0';
            //            if(strcmp(rxChar,"red")==0)
            //                redLED^=1;
            //            if(strcmp(rxChar,"blue")==0)
            //                blueLED^=1;
            //            if(strcmp(rxChar,"green")==0)
            //                greenLED^=1;

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
