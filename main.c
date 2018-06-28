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
 *BOTAO DE LEITURA ----------- 9
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
#define RXCHAR_SIZE 100

#define TASKSTACKSIZE   512
#define CARD_LENGTH 5

int chipSelectPin = 0x20;  //PB5
int NRSTPD = 0x01; //PF0

String drinkStr;
bool aButtonIsPressed;
int secondCount;
bool isUserFound;
bool cardRegistering;
int drinkTimeToWait_SEC;

int index;
char rxChar[RXCHAR_SIZE];
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
void initBuzzRele();
void writeIDToUart1(char* str);
int uartEchoReceivedString();
void initButton1();
void dumpHex(unsigned char* buffer, int len);
void InitSSI();
void writeToUart1(char* str);

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
                GPIOPinWrite(GPIO_PORTF_BASE, blueLED, blueLED);//LigaBlueLed
                GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0);//LigaBuzzer

                status = Mfrc522.Anticoll(str);
                memcpy(cardID, str, CARD_LENGTH);

                if(status == MI_OK){
                    System_printf("ID: ");
                    dumpHex((unsigned char*)cardID, CARD_LENGTH);
                    SysCtlDelay(1000000);
                    GPIOPinWrite(GPIO_PORTF_BASE, blueLED, 0);//DesligaBlueLed
                    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4);//DesligaBuzzer
                    Semaphore_pend(semphCard, BIOS_WAIT_FOREVER); //Aguarda o Semaforo Card
                    Semaphore_post(semphButton);
                    /*Para voltar execução*/
                    secondCount=0;
                    Clock_start(oneSecondCount);
                } else {
                    System_printf("Não foi possível ler o cartão. Favor segurar mais tempo. \n\n");
                    SysCtlDelay(1000000);
                    GPIOPinWrite(GPIO_PORTF_BASE, blueLED, 0);//DesligaBlueLed
                    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4);//DesligaBuzzer
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
        if(buttonPressed == 9){
            writeIDToUart1(cardID);
            System_printf("ID enviado, retornando! \n\n");
            Semaphore_post(semphCard);
        } else {
            writeIDToUart1(cardID);
            System_printf("Aguardando resposta do servidor! \n\n");
            Semaphore_pend(semphBluetooth, BIOS_WAIT_FOREVER);
            Semaphore_post(semphCard);
        }

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
    secondCount=0;
    Clock_start(oneSecondCount);

    while (1) {
        if(isUserFound) {
            GPIOPinWrite(GPIO_PORTF_BASE, redLED, redLED);

            switch(buttonPressed){
            case 1 : //300mL
                drinkTimeToWait_SEC = 6;
                drinkStr = "Gavioli 300mL";
                break;
            case 2 : //500mL
                drinkTimeToWait_SEC = 10;
                drinkStr = "Gavioli 500mL";
                break;
            case 3 : //700mL
                drinkTimeToWait_SEC = 14;
                drinkStr = "Gavioli 700mL";
                break;
            default :
                drinkTimeToWait_SEC = 0; //Error, nunca deve entrar aqui.
            }

            System_printf("Servindo: %s !! \n\n", drinkStr);

            while(secondCount < drinkTimeToWait_SEC) {
                GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5);//Liga Relé
            }
            GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0); //Desliga Relé

            System_printf("Bebida servida com sucesso. \n\n");
            writeToUart1("Ok"); //Confirmação de Bebida servida com sucesso

            Clock_stop(oneSecondCount);
            GPIOPinWrite(GPIO_PORTF_BASE, redLED, 0);
            //GPIOPinWrite(GPIO_PORTF_BASE, redLED, redLED); Desligar pino bebida
            Semaphore_post(semphGetResponse);
            secondCount=0;
            Clock_start(oneSecondCount);
        } else {
            Clock_stop(oneSecondCount);
            Semaphore_post(semphGetResponse);
            secondCount=0;
        }
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
    initBuzzRele();
    initUart1();
    InitSSI();

    GPIOPinWrite(GPIO_PORTB_BASE, chipSelectPin, 0);
    GPIOPinWrite(GPIO_PORTF_BASE, NRSTPD, NRSTPD);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4); //Buzzer ativa em Nivel Baixo
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0); //Relé Off

    Mfrc522.Init();

    Version = Mfrc522.ReadReg(VersionReg);
    AntennaGain = Mfrc522.ReadReg(PICC_REQIDL) & (0x07<<4);

    System_printf("Version: '0x%x' \n", Version);
    System_printf("Antenna Gain: '0x%x' \n\n", AntennaGain);

    System_printf("Programa Iniciado.\n\n");

    isUserFound = false;
    cardRegistering = false;

    /* Start BIOS */
    BIOS_start();


    return (0);
}

//==================================================================================================
void initBuzzRele(){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);

}

void initUart1(){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1); //habilita periférico UART1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); //habilita periférico GPIOB

    GPIOPinConfigure(GPIO_PB0_U1RX); //Pin PB0 definido como RECEIVER
    GPIOPinConfigure(GPIO_PB1_U1TX); //Pin PB1 definido como TRANSMITER
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 9600,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    while(UARTCharsAvail(UART1_BASE)){
        UARTCharGet(UART1_BASE);
    } // clear buffer

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

    System_printf("SSI Enabled! SPI Mode!  \nData: 8bits.\n");
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

    UARTCharPut(UART1_BASE, buttonPressed +'0');
    UARTCharPut(UART1_BASE, ',');
    UARTCharPut(UART1_BASE, ' ');

    for (i = 0; i < strlen(str); i++) {
        sprintf(hexToChar,"%x",str[i]);
        UARTCharPut(UART1_BASE, hexToChar[0]);
        UARTCharPut(UART1_BASE, hexToChar[1]);
    }
    UARTCharPut(UART1_BASE, '\n');
}

void writeToUart1(char* str){
    int i;
    for (i = 0; i < strlen(str); i++) {
        UARTCharPut(UART1_BASE, str[i]);
    }
    UARTCharPut(UART1_BASE, '\n');
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
        rxChar[index] = UARTCharGet(UART1_BASE);//UART1_DR_R;
        if(rxChar[index]==13)
        {
            rxChar[index]='\0';
            index=0;
            if(strcmp(rxChar,"Null")==0){
                System_printf("Nenhum usuário encontrado cadastrado no cartão!\n\n");
                isUserFound = false;

            } else if (strcmp(rxChar,"Error")==0) {
                System_printf("Saldo insufuciente!\n\n");
                isUserFound = false;

            }  else {
                System_printf("Usuário encontrado!\nNome: %s\n\n", rxChar);
                isUserFound = true;

            }
            memset(rxChar, 0, RXCHAR_SIZE);
            return 1;

        } else {
            index++;
        }
    }
    return 0;
}
