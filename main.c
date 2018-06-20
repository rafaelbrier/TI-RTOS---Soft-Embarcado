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

#define redLED   0x00000002
#define blueLED  0x00000004
#define greenLED 0x00000008

#define CARD_WAIT_SEC 10
#define SERVER_WAIT_SEC 15

#define TASKSTACKSIZE   512
#define CARD_LENGTH 5

int chipSelectPin = 0x20;  //PB5
int NRSTPD = 0x01; //PF0

bool aButtonIsPressed;
int secondCount;
bool isUserFound;

int index;
char rxChar[10];
char strrTeste[10];
uint32_t i, ii;
uint8_t Version;
uint8_t buttonPressed;
uint8_t AntennaGain;
uint8_t status;
uint32_t readTeste;
unsigned char str[MAX_LEN];
char cardID[CARD_LENGTH];

//Library modified to work with CCS
#ifdef __cplusplus
Mfrc522 Mfrc522(chipSelectPin, NRSTPD);
#endif

//Functions
void initUart1();
void initLeds();
void writeIDToUart1(char* str);
int uartEchoReceivedString();
void initButton1();
void dumpHex(unsigned char* buffer, int len);
void InitSSI();

//Tasks----------------------------------------------------------------------------------

extern "C" {
Void taskButtonFunc(UArg arg0, UArg arg1)
{
    // System_printf("Task Button em primeira execução. \n");
    aButtonIsPressed=false;

    while (1) {
        if(GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4)==0x00){
            buttonPressed = 1;
            aButtonIsPressed=true;
        } else {
            GPIOPinWrite(GPIO_PORTF_BASE, greenLED, greenLED);
        }

        if(aButtonIsPressed){
            System_printf("Botao %d apertado. Aguardando cartão... \n\n", buttonPressed);
            GPIOPinWrite(GPIO_PORTF_BASE, greenLED, 0);
            Semaphore_pend(semphButton, BIOS_WAIT_FOREVER); //Aguarda o Semaforo Botao
            aButtonIsPressed=false;
        }
    }
}

Void taskCardFunc(UArg arg0, UArg arg1)
{
    //System_printf("Task Card em primeira execução. \n\n");
    secondCount=0;
    Clock_start(oneSecondCount);

    while(1){
        if(secondCount < CARD_WAIT_SEC) {
            status = Mfrc522.Request(PICC_REQIDL, str);
            if(status == MI_OK){
                System_printf("Cartao Detectado! "); //Card Detected
                GPIOPinWrite(GPIO_PORTF_BASE, blueLED, blueLED);

                status = Mfrc522.Anticoll(str);
                memcpy(cardID, str, CARD_LENGTH);

                if(status == MI_OK){
                    System_printf("ID: ");
                    dumpHex((unsigned char*)cardID, CARD_LENGTH);
                    GPIOPinWrite(GPIO_PORTF_BASE, blueLED, 0);
                    Semaphore_pend(semphCard, BIOS_WAIT_FOREVER); //Aguarda o Semaforo Card
                    Semaphore_post(semphButton);
                    /*Para voltar execução*/
                    secondCount=0;
                    Clock_start(oneSecondCount);
                } else {
                    System_printf("Não foi possível ler o cartão. Favor segurar mais tempo. \n\n");
                    GPIOPinWrite(GPIO_PORTF_BASE, blueLED, 0);
                    Clock_start(oneSecondCount);
                }
            }
        } else if (secondCount >= CARD_WAIT_SEC) {
            /*Se o timer estourar libera o semáforo da primeira Task. Aguarda o cartão por 10 seg apenas.*/
            Clock_stop(oneSecondCount);
            System_printf("Tempo expirado! Nenhum cartão foi lido! \n\n");
            Semaphore_post(semphButton);
            /*Para voltar execução*/
            secondCount=0;
            Clock_start(oneSecondCount);
        }
    }
}


Void taskBluetoothFunc(UArg arg0, UArg arg1)
{
    //System_printf(">>>Task Bluetooth em primeira execução. \n\n");
    while (1) {
        writeIDToUart1(cardID);
        System_printf("Aguardando resposta do servidor! \n\n");
        Semaphore_pend(semphBluetooth, BIOS_WAIT_FOREVER);
        Semaphore_post(semphCard);
    }
}

Void taskGetResponseFunc(UArg arg0, UArg arg1)
{
    //System_printf(">>>Task GetResponse em primeira execução. \n\n");
    secondCount=0;
    Clock_start(oneSecondCount);

    while(1){
        if(secondCount < SERVER_WAIT_SEC) {
            if(uartEchoReceivedString()){
                Semaphore_pend(semphGetResponse, BIOS_WAIT_FOREVER);
                Semaphore_post(semphBluetooth);
                /*Para voltar execução*/
                secondCount=0;
                Clock_start(oneSecondCount);
            }
        } else if (secondCount >= SERVER_WAIT_SEC) {
            /*Se o timer estourar libera o semáforo da primeira Task. Aguarda o cartão por 10 seg apenas.*/
            Clock_stop(oneSecondCount);
            System_printf("Tempo expirado! Nenhuma resposta recebida do servidor! \n\n");
            Semaphore_post(semphBluetooth);
            /*Para voltar execução*/
            secondCount=0;
            Clock_start(oneSecondCount);
        }
    }
}

Void taskValvulaFunc(UArg arg0, UArg arg1)
{
    //System_printf(">>>Task Valvula em primeira execução. \n\n");

    //Clock_start(timeOutClock);
    while (1) {

    }
}

Void oneSecondCountFunc(UArg arg0){
    secondCount++;
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

void writeIDToUart1(char* str)   //write a string to Uart1
{
    int i;
    char hexToChar[2];

    UARTCharPut(UART1_BASE, buttonPressed + '0');
    UARTCharPut(UART1_BASE, ',');
    UARTCharPut(UART1_BASE, ' ');

    for (i = 0; i < strlen(str); i++) {
        sprintf(hexToChar,"%x",str[i]);
        UARTCharPut(UART1_BASE, hexToChar[0]);
        UARTCharPut(UART1_BASE, hexToChar[1]);
    }
}

void dumpHex(unsigned char* buffer, int len){
    int i;

    System_printf(" ");
    for(i=0; i < len; i++) {
        System_printf("%x-", buffer[i]);
    }
    System_printf("\n\n");
}

int uartEchoReceivedString(){
    isUserFound = false;
    while(UARTCharsAvail(UART1_BASE)) //loop while there are chars
    {
        rxChar[index] = UART1_DR_R;
        if(rxChar[index-1]==13)
        {
            rxChar[index-1]='\0';
            index=0;
            if(strcmp(rxChar,"Null")==0){
                System_printf("Nenhum usuário encontrado cadastrado no cartão!\n", rxChar);
                isUserFound = false;
            } else {
                System_printf("Usuário encontrado! Nome: %s\n", rxChar);
                isUserFound = true;
            }
            return 1;
        }
        else {
            index++;
        }
    }
    return 0;
}
