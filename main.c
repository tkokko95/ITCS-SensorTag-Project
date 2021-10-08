/* Tehnyt: Taavi Kokko

Virtanappi on (yllättäen) virtanappi, toinen painike vaihtaa oletusnäytön ja viestinäytön välillä.

Lähettää tervehdyksen lähistöllä oleville laitteille 50 askeleen välein. Tai ainakin sinne päin,
patterin loppuessa en ehtinyt testaamaan sopivaa kynnysarvoa askelmittarille kovin tarkasti.

Ledin pitäisi syttyä kun uusi viesti saapuu ja sammua kun viestinäyttö avataan, tätä en ehtinyt testaamaan
koska testiserveri oli jo viety pois.
*/

/*
 *  ======== main.c ========
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>

/* TI-RTOS Header files */
#include <ti/drivers/I2C.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/i2c/I2CCC26XX.h>
#include <ti/mw/display/Display.h>
#include <ti/mw/display/DisplayExt.h>

/* Board Header files */
#include "Board.h"
#include "wireless/comm_lib.h"

//Omat funktiot:
#include "myfuncs.h"

/* Task Stacks */
#define STACKSIZE 2048
Char i2cTaskStack[STACKSIZE];
Char commTaskStack[STACKSIZE];

Display_Handle hDisplay;

// Virtapainike
static PIN_Handle hPowerButton;
static PIN_State sPowerButton;
PIN_Config cPowerButton[] = {
    Board_BUTTON1 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE};
PIN_Config cPowerWake[] = {
    Board_BUTTON1 | PIN_INPUT_EN | PIN_PULLUP | PINCC26XX_WAKEUP_NEGEDGE,
    PIN_TERMINATE};

//Button0 inputtina
static PIN_Handle hButton0;
static PIN_State sButton0;

PIN_Config cButton0[] = {
    Board_BUTTON0 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE};

static PIN_Handle hLed;
static PIN_State sLed;
PIN_Config cLed[] = {
    Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE};

//MPU
static PIN_Handle hMpuPin;
static PIN_State MpuPinState;
static PIN_Config MpuPinConfig[] = {
    Board_MPU_POWER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE};

static const I2CCC26XX_I2CPinCfg i2cMPUCfg = {
    .pinSDA = Board_I2C0_SDA1,
    .pinSCL = Board_I2C0_SCL1};

//Tämä määrää näyttömoodin, oletuksena näytetään askeleet ja liikkuminen
bool disp_mode_is_msgs = false;

/*Viestien tulostus olisi pitänyt kai olisi ollut parempi toteuttaa kommunikaatiotaskissa
  mutta en saanut sitä toimimaan järkevästi sitten millään joten mennään purkkaratkaisulla*/
char msg_buffer1[16], msg_buffer2[16], msg_buffer3[16], msg_buffer4[16], msg_buffer5[16],
    msg_buffer6[16], msg_buffer7[16], msg_buffer8[16];

//Virtapainikkeen kahva
Void powerButtonFxn(PIN_Handle handle, PIN_Id pinId)
{

    Display_clear(hDisplay);
    Display_close(hDisplay);
    Task_sleep(100000 / Clock_tickPeriod);

    PIN_close(hPowerButton);

    PINCC26XX_setWakeup(cPowerWake);
    Power_shutdown(NULL, 0);
}

//Napin kahva
Void buttonFxn(PIN_Handle handle, PIN_Id pinId)
{

    if (disp_mode_is_msgs)
    {
        disp_mode_is_msgs = false;
    }
    else
    {
        disp_mode_is_msgs = true;
    }
}

//Kommunikaatiotaski
Void commTask(UArg arg0, UArg arg1)
{
    uint16_t sender_addr;

    int32_t result = StartReceive6LoWPAN();
    if (result != true)
    {
        System_abort("Wireless receive mode failed");
    }

    while (1)
    {

        if (GetRXFlag())
        {
            //Ledi päälle
            PIN_setOutputValue(hLed, Board_LED0, PIN_GPIO_HIGH);
            //Uusi viesti ensimmäiseen puskuriin, edelliset antavat tilaa.
            memset(msg_buffer8, 0, 16);
            memcpy(msg_buffer8, msg_buffer7, 16);
            memset(msg_buffer7, 0, 16);
            memcpy(msg_buffer7, msg_buffer6, 16);
            memset(msg_buffer6, 0, 16);
            memcpy(msg_buffer6, msg_buffer5, 16);
            memset(msg_buffer5, 0, 16);
            memcpy(msg_buffer5, msg_buffer4, 16);
            memset(msg_buffer4, 0, 16);
            memcpy(msg_buffer4, msg_buffer3, 16);
            memset(msg_buffer3, 0, 16);
            memcpy(msg_buffer3, msg_buffer2, 16);
            memset(msg_buffer2, 0, 16);
            memcpy(msg_buffer2, msg_buffer1, 16);

            memset(msg_buffer1, 0, 16);
            Receive6LoWPAN(&sender_addr, msg_buffer1, 16);
        }
    }
}

//Sensoritaski
Void i2cTask(UArg arg0, UArg arg1)
{

    I2C_Handle i2c;
    I2C_Params i2cParams;

    I2C_Handle i2cMPU;
    I2C_Params i2cMPUParams;

    //Paineen ja lämpötilan muuttujat ja puskurit niiden kirjoittamiseksi näytölle
    double temp, pres;
    char temp_print_buffer[10];
    char pres_print_buffer[14];

    //Liikesensoriin liittyvät muuttujat
    float mpu_buffer[80][3], acc_mean_x = 0.00, acc_mean_y = 0.00, acc_mean_z = -1.00;
    char peak_count_x, peak_count_y, peak_count_z, max_peaks, i;
    uint16_t steps = 0;

    char steps_print_buffer[20], msg_send_buffer[16];

    uint16_t msg_treshold = 50;

    //Alustetaan MPU
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    I2C_Params_init(&i2cMPUParams);
    i2cMPUParams.bitRate = I2C_400kHz;
    i2cMPUParams.custom = (uintptr_t)&i2cMPUCfg;

    i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
    if (i2cMPU == NULL)
    {
        System_abort("Error Initializing I2CMPU\n");
    }

    PIN_setOutputValue(hMpuPin, Board_MPU_POWER, Board_MPU_POWER_ON);

    Task_sleep(100000 / Clock_tickPeriod);
    System_printf("MPU9250: Power ON\n");
    System_flush();

    System_printf("MPU9250: Setup and calibration...\n");
    System_flush();

    mpu9250_setup(&i2cMPU);

    System_printf("MPU9250: Setup and calibration OK\n");
    System_flush();

    I2C_close(i2cMPU);

    //Ja sama BMP:lle, onneksi vähemmällä koodilla
    i2c = I2C_open(Board_I2C0, &i2cParams);
    if (i2c == NULL)
    {
        System_abort("Error Initializing I2C\n");
    }

    bmp280_setup(&i2c);
    I2C_close(i2c);

    //Alustetaan näyttö
    Display_Params displayParams;
    displayParams.lineClearMode = DISPLAY_CLEAR_BOTH;
    Display_Params_init(&displayParams);

    hDisplay = Display_open(Display_Type_LCD, &displayParams);
    if (hDisplay == NULL)
    {
        System_abort("Error initializing Display\n");
    }

    //Päälooppi
    while (1)
    {

        //Avataan i2c-yhteys ja kysellään dataa BMP:ltä
        i2c = I2C_open(Board_I2C, &i2cParams);
        if (i2c == NULL)
        {
            System_abort("Error Initializing I2C\n");
        }
        bmp280_get_data(&i2c, &pres, &temp);

        //I2C suljetaan, tulokset näytölle
        I2C_close(i2c);
        sprintf(temp_print_buffer, "%.2f C", temp);
        sprintf(pres_print_buffer, "%.2f hPa", pres);

        Display_print0(hDisplay, 0, 0, temp_print_buffer);
        Display_print0(hDisplay, 1, 0, pres_print_buffer);

        //MPU:n vuoro:
        i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
        if (i2cMPU == NULL)
        {
            System_abort("Error Initializing I2CMPU\n");
        }

        //Kysellään MPU:lta dataa 80 kertaa (reilut kaksi sekuntia) ja täytetään sillä puskuri
        for (i = 0; i < 80; i++)
        {
            mpu9250_get_data(&i2cMPU, &mpu_buffer[i][0]);
            Task_sleep(30000 / Clock_tickPeriod);
        }

        I2C_close(i2cMPU);

        //Lasketaan piikit, lopullinen tulos määräytyy suurimman piikkimäärän mukaan
        peak_count_x = find_peaks(mpu_buffer, 0, acc_mean_x);
        peak_count_y = find_peaks(mpu_buffer, 1, acc_mean_y);
        peak_count_z = find_peaks(mpu_buffer, 2, acc_mean_z);

        //Tällä pyritään ohittamaan ilmiselvästi virheelliset mittaukset
        if (peak_count_x <= 6 && peak_count_y <= 6 && peak_count_z <= 6)
        {

            max_peaks = peak_count_x;

            if (peak_count_y > max_peaks)
            {
                max_peaks = peak_count_y;
            }

            if (peak_count_z > max_peaks)
            {
                max_peaks = peak_count_z;
            }
            steps += max_peaks;
        }

        if (!disp_mode_is_msgs)
        {
            Display_clearLines(hDisplay, 2, 3);
            Display_clearLines(hDisplay, 7, 10);
            if (max_peaks > 0)
            {
                Display_print0(hDisplay, 4, 0, "Liikut!");
            }

            else
            {
                Display_print0(hDisplay, 4, 0, "Et liiku.");
            }

            if (peak_count_x > 6 || peak_count_y > 6 || peak_count_z > 6)
            {
                Display_print0(hDisplay, 6, 0, "(Ei mitata)");
            }

            else
            {
                Display_print0(hDisplay, 6, 0, "");
            }

            sprintf(steps_print_buffer, "Askeleet: %d", steps);
            Display_print0(hDisplay, 5, 0, steps_print_buffer);
        }

        else
        {
            PIN_setOutputValue(hLed, Board_LED0, PIN_GPIO_LOW);
            Display_print0(hDisplay, 3, 0, msg_buffer1);
            Display_print0(hDisplay, 4, 0, msg_buffer2);
            Display_print0(hDisplay, 5, 0, msg_buffer3);
            Display_print0(hDisplay, 6, 0, msg_buffer4);
            Display_print0(hDisplay, 7, 0, msg_buffer5);
            Display_print0(hDisplay, 8, 0, msg_buffer6);
            Display_print0(hDisplay, 9, 0, msg_buffer7);
            Display_print0(hDisplay, 10, 0, msg_buffer8);
        }

        if (steps >= msg_treshold)
        {
            sprintf(msg_send_buffer, "%d askelta!", msg_treshold);
            Send6LoWPAN(0xFFFF, msg_send_buffer, strlen(msg_send_buffer));
            StartReceive6LoWPAN();
            msg_treshold += 50;
        }

        Task_sleep(1000000 / Clock_tickPeriod);
    }
}

Int main(void)
{

    Task_Handle hi2cTask;
    Task_Params i2cTaskParams;
    Task_Handle hCommTask;
    Task_Params commTaskParams;

    Board_initGeneral();
    Board_initI2C();

    hPowerButton = PIN_open(&sPowerButton, cPowerButton);
    if (!hPowerButton)
    {
        System_abort("Error initializing power button shut pins\n");
    }
    if (PIN_registerIntCb(hPowerButton, &powerButtonFxn) != 0)
    {
        System_abort("Error registering power button callback function");
    }

    hButton0 = PIN_open(&sButton0, cButton0);
    if (!hButton0)
    {
        System_abort("Error initializing button pins\n");
    }

    if (PIN_registerIntCb(hButton0, &buttonFxn) != 0)
    {
        System_abort("Error registering button callback function");
    }

    hLed = PIN_open(&sLed, cLed);
    if (!hLed)
    {
        System_abort("Error initializing LED pin\n");
    }

    //MPU:n pinnien avaus
    hMpuPin = PIN_open(&MpuPinState, MpuPinConfig);
    if (hMpuPin == NULL)
    {
        System_abort("Pin open failed!");
    }

    Task_Params_init(&i2cTaskParams);
    i2cTaskParams.stackSize = STACKSIZE;
    i2cTaskParams.stack = &i2cTaskStack;
    i2cTaskParams.priority = 2;

    hi2cTask = Task_create(i2cTask, &i2cTaskParams, NULL);
    if (hi2cTask == NULL)
    {
        System_abort("Task create failed!");
    }

    Init6LoWPAN();

    Task_Params_init(&commTaskParams);
    commTaskParams.stackSize = STACKSIZE;
    commTaskParams.stack = &commTaskStack;
    commTaskParams.priority = 1;

    hCommTask = Task_create(commTask, &commTaskParams, NULL);
    if (hCommTask == NULL)
    {
        System_abort("Task create failed!");
    }

    System_printf("Hello world!\n");
    System_flush();

    BIOS_start();

    return (0);
}
