//*****************************************************************************
//
// Codigo de partida comunicacion TIVA-QT (Marzo2020)
// Autores: Eva Gonzalez, Ignacio Herrero, Jose Manuel Cano
//
//  Estructura de aplicacion basica para el desarrollo de aplicaciones genericas
//  basada en la TIVA, en las que existe un intercambio de mensajes con un interfaz
//  gráfico (GUI) Qt.
//  La aplicacion se basa en un intercambio de mensajes con ordenes e informacion, a traves  de la
//  configuracion de un perfil CDC de USB (emulacion de puerto serie) y un protocolo
//  de comunicacion con el PC que permite recibir ciertas ordenes y enviar determinados datos en respuesta.
//   En el ejemplo basico de partida se implementara la recepcion de un mensaje
//  generico que permite el apagado y encendido de los LEDs de la placa; asi como un segundo
//  mensaje enviado desde la placa al GUI, para mostrar el estado de los botones.
//
//*****************************************************************************
#include<stdbool.h>
#include<stdint.h>
#include <stdlib.h>              // rand()
#include "inc/hw_memmap.h"       // TIVA: Definiciones del mapa de memoria
#include "inc/hw_types.h"        // TIVA: Definiciones API
#include "inc/hw_ints.h"         // TIVA: Definiciones para configuracion de interrupciones
#include "driverlib/gpio.h"      // TIVA: Funciones API de GPIO
#include "driverlib/debug.h"     //Temperatura
#include "driverlib/pin_map.h"   // TIVA: Mapa de pines del chip
#include "driverlib/rom.h"       // TIVA: Funciones API incluidas en ROM de micro (MAP_)
#include "driverlib/rom_map.h"   // TIVA: Mapeo automatico de funciones API incluidas en ROM de micro (MAP_)
#include "driverlib/sysctl.h"    // TIVA: Funciones API control del sistema
#include "driverlib/uart.h"      // TIVA: Funciones API manejo UART
#include "driverlib/interrupt.h" // TIVA: Funciones API manejo de interrupciones
#include "driverlib/timer.h"     // TIVA: Funciones API manejo de timers
#include <utils/uartstdioMod.h>     // TIVA: Funciones API UARTSTDIO (printf)
#include "drivers/buttons.h"     // TIVA: Funciones API manejo de botones
#include "drivers/rgb.h"         // TIVA: Funciones API manejo de leds con PWM
#include "FreeRTOS.h"            // FreeRTOS: definiciones generales
#include "task.h"                // FreeRTOS: definiciones relacionadas con tareas
#include "semphr.h"              // FreeRTOS: definiciones relacionadas con semaforos
#include "queue.h"               // FreeRTOS: definiciones relacionadas con colas de mensajes
#include "timers.h"
#include "commands.h"
#include "utils/cpu_usage.h"
#include <serial2USBprotocol.h>
#include <usb_dev_serial.h>
#include "usb_messages_table.h"

//I2C
#include "sensorlib/i2cm_drv.h"
//sensor temperatura
#include "sensorlib/hw_tmp006.h"
#include "sensorlib/tmp006.h"

//sensor humedad
#include "sensorlib/hw_sht21.h"
#include "sensorlib/sht21.h"

//sensor presion
#include <math.h>
#include "sensorlib/hw_bmp180.h"
#include "sensorlib/bmp180.h"
//sensor luz
#include "sensorlib/hw_isl29023.h"
#include "sensorlib/isl29023.h"

//sensor movimiento
#include "sensorlib/hw_mpu9150.h"
#include "sensorlib/hw_ak8975.h"
#include "sensorlib/ak8975.h"
#include "sensorlib/mpu9150.h"
#include "sensorlib/comp_dcm.h"


//temperatura
#define TEMPTASKPRIO 1
#define TEMPTASKSTACKSIZE 128

//humedad
#define HUMTASKPRIO 1
#define HUMTASKSTACKSIZE 128

//presion
#define PRETASKPRIO 1
#define PRETASKSTACKSIZE 128

//luz
#define LUZTASKPRIO 1
#define LUZTASKSTACKSIZE 128

//movimiento
#define MOVTASKPRIO 1
#define MOVTASKSTACKSIZE 128



TimerHandle_t xTimer;
SemaphoreHandle_t semaforoTemperatura;
SemaphoreHandle_t semaforoHumedad;
SemaphoreHandle_t semaforoPresion;
SemaphoreHandle_t semaforoLuz;
SemaphoreHandle_t semaforoMovimiento;
QueueHandle_t colaPresionMar;
QueueHandle_t colaPresionMuestreo;
QueueHandle_t colaTempADC;
QueueHandle_t colaHumMR;
QueueHandle_t colaDeclinacion;
QueueHandle_t colaRangoLuz;
QueueHandle_t colaResolLuz;
QueueHandle_t colaMovimientoDLPF;
QueueHandle_t colaMovimientoGIRO;

// Variables globales "main"
uint32_t g_ui32CPUUsage;
uint32_t g_ui32SystemClock;
QueueSetHandle_t grupo_colasPresion;
QueueSetHandle_t grupo_colasTemperatura;
QueueSetHandle_t grupo_colasHumedad;
QueueSetHandle_t grupo_colasMovimiento;
QueueSetHandle_t grupo_colasLuz;
float pfAccelX,pfAccelY,pfAccelZ;
float pfGyroX,pfGyroY,pfGyroZ;
float pfMagX,pfMagY,pfMagZ;
float pfRoll,pfPitch,pfYaw;

//modo automatico
bool automatico=true;
//*****************************************************************************
//
// Define TMP006 I2C Address and SHT21 I2C Address.
//
//*****************************************************************************
#define TMP006_I2C_ADDRESS      0x41
#define SHT21_I2C_ADDRESS  0x40
#define BMP180_I2C_ADDRESS      0x77
#define ISL29023_I2C_ADDRESS    0x44
#define MPU9150_I2C_ADDRESS     0x68


//*****************************************************************************
//
// Global counter to control and slow down the rate of data to the terminal.
//
//*****************************************************************************
#define PRINT_SKIP_COUNT        4
#define POSICIONES_COLA 5

uint32_t g_ui32PrintSkipCounter;
//*****************************************************************************
//
// Global instance structure for the I2C master driver.
//
//*****************************************************************************
tI2CMInstance g_sI2CInst;

//*****************************************************************************
//
// Global instance structure for the TMP006 sensor driver.
//
//*****************************************************************************
tTMP006 g_sTMP006Inst;
tSHT21 g_sSHT21Inst;
tBMP180 g_sBMP180Inst;
tISL29023 g_sISL29023Inst;
tMPU9150 g_sMPU9150Inst;
tCompDCM g_sCompDCMInst;



//Variables para el sensor del movimiento
float radian=57.2958;

//*****************************************************************************
//
// Global new data flag to alert main that TMP006 data is ready.
//
//*****************************************************************************
volatile uint_fast8_t g_vui8DataFlag_TMP006;
volatile uint_fast8_t g_vui8DataFlag_SHT21;
volatile uint_fast8_t g_vui8DataFlag_BMP180;
volatile unsigned long g_vui8DataFlag_ISL29023;
volatile uint_fast8_t g_vui8I2CDataFlag_MPU9150;
//*****************************************************************************
//
// Global new error flag to store the error condition if encountered.
//
//*****************************************************************************
volatile uint_fast8_t g_vui8ErrorFlag_TMP006;
volatile uint_fast8_t g_vui8ErrorFlag_SHT21;
volatile uint_fast8_t g_vui8ErrorFlag_BMP180;
volatile unsigned long g_vui8ErrorFlag_ISL29023;
volatile uint_fast8_t g_vui8ErrorFlag_MPU9150;


//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}

#endif

//*****************************************************************************
//
// Aqui incluimos los "ganchos" a los diferentes eventos del FreeRTOS
//
//*****************************************************************************

//Esto es lo que se ejecuta cuando el sistema detecta un desbordamiento de pila
//
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
	//
	// This function can not return, so loop forever.  Interrupts are disabled
	// on entry to this function, so no processor interrupts will interrupt
	// this loop.
	//
	while(1)
	{
	}
}

//Esto se ejecuta cada Tick del sistema. LLeva la estadistica de uso de la CPU (tiempo que la CPU ha estado funcionando)
void vApplicationTickHook( void )
{
	static uint8_t ui8Count = 0;

	if (++ui8Count == 10)
	{
		g_ui32CPUUsage = CPUUsageTick();
		ui8Count = 0;
	}
	//return;
}

//Esto se ejecuta cada vez que entra a funcionar la tarea Idle
void vApplicationIdleHook (void)
{
	SysCtlSleep();
}


//Esto se ejecuta cada vez que se produce un fallo de asignacio de heap
void vApplicationMallocFailedHook (void)
{
	while(1);
}

///////////////////////////TEMPERATURA///////////////////////////////


//****************************************************************************
//Sensor callback function.  Called at the end of  sensor driver
// transactions. This is called from I2C interrupt context. Therefore, we just
// set a flag and let main do the bulk of the computations and display.
//*****************************************************************************
void
TMP006AppCallback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    //
    // If the transaction succeeded set the data flag to indicate to
    // application that this transaction is complete and data may be ready.
    //
    if(ui8Status == I2CM_STATUS_SUCCESS)
    {
        g_vui8DataFlag_TMP006 = 1;
    }

    //
    // Store the most recent status in case it was an error condition
    //
    g_vui8ErrorFlag_TMP006 = ui8Status;
}

//**********************************************************
//Application error handler.
//**********************************************************
void
TMP006AppErrorHandler(char *pcFilename, uint_fast32_t ui32Line)
{
    //
    // Set terminal color to red and print error status and locations
    //
    UARTprintf("\033[31;1m");
    UARTprintf("Error: %d, File: %s, Line: %d\n"
               "See I2C status definitions in utils\\i2cm_drv.h\n",
               g_vui8ErrorFlag_TMP006, pcFilename, ui32Line);



    //
    // Go to sleep wait for interventions.  A more robust application could
    // attempt corrective actions here.
    //
    while(1)
    {
        MAP_SysCtlSleep();
    }
}
//*************************************************************
// Function to wait for the TMP006 transactions to complete.
//*************************************************************
void
TMP006AppI2CWait(char *pcFilename, uint_fast32_t ui32Line)
{
    //
    // Put the processor to sleep while we wait for the I2C driver to
    // indicate that the transaction is complete.
    //
    while((g_vui8DataFlag_TMP006 == 0) && (g_vui8ErrorFlag_TMP006 == 0))
    {
        MAP_SysCtlSleep();
    }

    //
    // If an error occurred call the error handler immediately.
    //
    if(g_vui8ErrorFlag_TMP006)
    {
        TMP006AppErrorHandler(pcFilename, ui32Line);
    }

    //
    // clear the data flag for next use.
    //
    g_vui8DataFlag_TMP006= 0;
}




///////////////////////////HUMEDAD///////////////////////////////

//****************************************************************************
//Sensor callback function.  Called at the end of  sensor driver
// transactions. This is called from I2C interrupt context. Therefore, we just
// set a flag and let main do the bulk of the computations and display.
//*****************************************************************************
void
SHT21AppCallback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    //
    // If the transaction succeeded set the data flag to indicate to
    // application that this transaction is complete and data may be ready.
    //
    if(ui8Status == I2CM_STATUS_SUCCESS)
    {
        g_vui8DataFlag_SHT21 = 1;
    }

    //
    // Store the most recent status in case it was an error condition
    //
    g_vui8ErrorFlag_SHT21 = ui8Status;
}
//**********************************************************
//Application error handler.
//**********************************************************
void
SHT21AppErrorHandler(char *pcFilename, uint_fast32_t ui32Line)
{
    //
    // Set terminal color to red and print error status and locations
    //
    UARTprintf("\033[31;1m");
    UARTprintf("Error: %d, File: %s, Line: %d\n"
               "See I2C status definitions in utils\\i2cm_drv.h\n",
               g_vui8ErrorFlag_SHT21, pcFilename, ui32Line);



    //
    // Go to sleep wait for interventions.  A more robust application could
    // attempt corrective actions here.
    //
    while(1)
    {
        MAP_SysCtlSleep();
    }
}

//*************************************************************
// Function to wait for the sHT21 transactions to complete.
//*************************************************************
void
SHT21AppI2CWait(char *pcFilename, uint_fast32_t ui32Line)
{
    //
    // Put the processor to sleep while we wait for the I2C driver to
    // indicate that the transaction is complete.
    //
    while((g_vui8DataFlag_SHT21 == 0) && (g_vui8ErrorFlag_SHT21 == 0))
    {
        MAP_SysCtlSleep();
    }

    //
    // If an error occurred call the error handler immediately.
    //
    if(g_vui8ErrorFlag_SHT21)
    {
        SHT21AppErrorHandler(pcFilename, ui32Line);
    }

    //
    // clear the data flag for next use.
    //
    g_vui8DataFlag_SHT21 = 0;
}


///////////////////////////PRESION///////////////////////////////

//*****************************************************************************
//
//Sensor callback function.  Called at the end of  sensor driver
// transactions. This is called from I2C interrupt context. Therefore, we just
// set a flag and let main do the bulk of the computations and display.
//
//*****************************************************************************


void
BMP180AppCallback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    //
    // If the transaction succeeded set the data flag to indicate to
    // application that this transaction is complete and data may be ready.
    //
    if(ui8Status == I2CM_STATUS_SUCCESS)
    {
        g_vui8DataFlag_BMP180 = 1;
    }

    //
    // Store the most recent status in case it was an error condition
    //
    g_vui8ErrorFlag_BMP180 = ui8Status;
}

//************************************************************
//Application error handler.
//************************************************************
void
BMP180AppErrorHandler(char *pcFilename, uint_fast32_t ui32Line)
{
    //
    // Set terminal color to red and print error status and locations
    //
    UARTprintf("\033[31;1m");
    UARTprintf("Error: %d, File: %s, Line: %d\n"
               "See I2C status definitions in utils\\i2cm_drv.h\n",
               g_vui8ErrorFlag_BMP180, pcFilename, ui32Line);



    //
    // Go to sleep wait for interventions.  A more robust application could
    // attempt corrective actions here.
    //
    while(1)
    {
        MAP_SysCtlSleep();
    }
}

//*************************************************************
// Function to wait for the BMP180 transactions to complete.
//*************************************************************
void
BMP180AppI2CWait(char *pcFilename, uint_fast32_t ui32Line)
{
    //
    // Put the processor to sleep while we wait for the I2C driver to
    // indicate that the transaction is complete.
    //
    while((g_vui8DataFlag_BMP180 == 0) && (g_vui8ErrorFlag_BMP180 == 0))
    {
        MAP_SysCtlSleep();
    }

    //
    // If an error occurred call the error handler immediately.
    //
    if(g_vui8ErrorFlag_BMP180)
    {
        BMP180AppErrorHandler(pcFilename, ui32Line);
    }

    //
    // clear the data flag for next use.
    //
    g_vui8DataFlag_BMP180 = 0;
}


///////////////////////////LUZ///////////////////////////////

void
ISL29023AppCallback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    //
    // If the transaction succeeded set the data flag to indicate to
    // application that this transaction is complete and data may be ready.
    //
    if(ui8Status == I2CM_STATUS_SUCCESS)
    {
        g_vui8DataFlag_ISL29023 = 1;
    }

    //
    // Store the most recent status in case it was an error condition
    //
    g_vui8ErrorFlag_ISL29023 = ui8Status;
}

void
ISL29023AppErrorHandler(char *pcFilename, uint_fast32_t ui32Line)
{
    //
    // Set terminal color to red and print error status and locations
    //
    UARTprintf("\033[31;1m");
    UARTprintf("Error: %d, File: %s, Line: %d\n"
               "See I2C status definitions in utils\\i2cm_drv.h\n",
               g_vui8ErrorFlag_ISL29023, pcFilename, ui32Line);

    //
    // Go to sleep wait for interventions.  A more robust application could
    // attempt corrective actions here.
    //
    while(1)
    {
        //
        // Do Nothing
        //
    }
}
void
ISL29023AppI2CWait(char *pcFilename, uint_fast32_t ui32Line)
{
    //
    // Put the processor to sleep while we wait for the I2C driver to
    // indicate that the transaction is complete.
    //
    while((g_vui8DataFlag_ISL29023 == 0) && (g_vui8ErrorFlag_ISL29023 == 0))
    {
        //
        // Do Nothing
        //
    }

    //
    // If an error occurred call the error handler immediately.
    //
    if(g_vui8ErrorFlag_ISL29023)
    {
        ISL29023AppErrorHandler(pcFilename, ui32Line);
    }

    //
    // clear the data flag for next use.
    //
    g_vui8DataFlag_ISL29023 = 0;
}



//*****************************************************************************
//
// Configure the UART and its pins. This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}


//Funcion para el timer SW
void vTimerCallback( TimerHandle_t pxTimer )
{
    //abro el semaforo cada segundo
    xSemaphoreGive(semaforoHumedad);//Abre el semaforo FreeRTOS
    xSemaphoreGive(semaforoPresion);//Abre el semaforo FreeRTOS
    xSemaphoreGive(semaforoLuz);//Abre el semaforo FreeRTOS

}

///////////////////////////MOVIMIENTO///////////////////////////////
//*****************************************************************************
//
// MPU9150 Sensor callback function.  Called at the end of MPU9150 sensor
// driver transactions. This is called from I2C interrupt context. Therefore,
// we just set a flag and let main do the bulk of the computations and display.
//
//*****************************************************************************

void
MPU9150AppCallback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    //
    // If the transaction succeeded set the data flag to indicate to
    // application that this transaction is complete and data may be ready.
    //
    if(ui8Status == I2CM_STATUS_SUCCESS)
    {
        g_vui8I2CDataFlag_MPU9150 = 1;
    }

    //
    // Store the most recent status in case it was an error condition
    //
    g_vui8ErrorFlag_MPU9150 = ui8Status;
}

//*****************************************************************************
//
// MPU9150 Application error handler. Show the user if we have encountered an
// I2C error.
//
//*****************************************************************************
void
MPU9150AppErrorHandler(char *pcFilename, uint_fast32_t ui32Line)
{
    //
    // Set terminal color to red and print error status and locations
    //
    UARTprintf("\033[31;1m");
    UARTprintf("Error: %d, File: %s, Line: %d\n"
               "See I2C status definitions in sensorlib\\i2cm_drv.h\n",
               g_vui8ErrorFlag_MPU9150, pcFilename, ui32Line);



    //
    // Go to sleep wait for interventions.  A more robust application could
    // attempt corrective actions here.
    //
    while(1)
    {
        //
        // Do Nothing
        //
    }
}

//*****************************************************************************
//
// Function to wait for the MPU9150 transactions to complete. Use this to spin
// wait on the I2C bus.
//
//*****************************************************************************
void
MPU9150AppI2CWait(char *pcFilename, uint_fast32_t ui32Line)
{
    //
    // Put the processor to sleep while we wait for the I2C driver to
    // indicate that the transaction is complete.
    //
    while((g_vui8I2CDataFlag_MPU9150 == 0) && (g_vui8ErrorFlag_MPU9150 == 0))
    {
        //
        // Do Nothing
        //
    }

    //
    // If an error occurred call the error handler immediately.
    //
    if(g_vui8ErrorFlag_MPU9150)
    {
        MPU9150AppErrorHandler(pcFilename, ui32Line);
    }

    //
    // clear the data flag for next use.
    //
    g_vui8I2CDataFlag_MPU9150 = 0;
}
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////TAREAS////////////////////////////////////////////////////
//// Codigo para procesar los mensajes recibidos a traves del canal USB

static portTASK_FUNCTION( USBMessageProcessingTask, pvParameters ){

    uint8_t pui8Frame[MAX_FRAME_SIZE];  //Ojo, esto hace que esta tarea necesite bastante pila
    int32_t i32Numdatos;
    uint8_t ui8Message;
    void *ptrtoreceivedparam; // <---?
    uint32_t ui32Errors=0;

    /* The parameters are not used. */
    ( void ) pvParameters;

    //
    // Mensaje de bienvenida inicial.
    //
    UARTprintf("\n\nBienvenido a la aplicacion simulación de vuelo!\n");
    UARTprintf("\nAutora: María Poyatos Peinado ");

    for(;;)
    {
        //Espera hasta que se reciba una trama con datos serializados por el interfaz USB
        i32Numdatos=receive_frame(pui8Frame,MAX_FRAME_SIZE); //Esta funcion es bloqueante
        if (i32Numdatos>0)
        {   //Si no hay error, proceso la trama que ha llegado.
            i32Numdatos=destuff_and_check_checksum(pui8Frame,i32Numdatos); // Primero, "destuffing" y comprobaci�n checksum
            if (i32Numdatos<0)
            {
                //Error de checksum (PROT_ERROR_BAD_CHECKSUM), ignorar el paquete
                ui32Errors++;
                // Procesamiento del error (TODO, POR HACER!!)
            }
            else
            {
                //El paquete esta bien, luego procedo a tratarlo.
                //Obtiene el valor del campo mensaje
                ui8Message=decode_message_type(pui8Frame);
                //Obtiene un puntero al campo de parametros y su tamanio.
                i32Numdatos=get_message_param_pointer(pui8Frame,i32Numdatos,&ptrtoreceivedparam);
                switch(ui8Message)
                {
                case MENSAJE_PING :
                    //A un mensaje de ping se responde con el propio mensaje
                    i32Numdatos=create_frame(pui8Frame,ui8Message,0,0,MAX_FRAME_SIZE);
                    if (i32Numdatos>=0)
                    {
                        send_frame(pui8Frame,i32Numdatos);
                    }else{
                        //Error de creacion de trama: determinar el error y abortar operacion
                        ui32Errors++;
                        // Procesamiento del error (TODO)
//                      // Esto de aqui abajo podria ir en una funcion "createFrameError(numdatos)  para evitar
//                      // tener que copiar y pegar todo en cada operacion de creacion de paquete
                        switch(i32Numdatos){
                        case PROT_ERROR_NOMEM:
                            // Procesamiento del error NO MEMORY (TODO)
                            break;
                        case PROT_ERROR_STUFFED_FRAME_TOO_LONG:
//                          // Procesamiento del error STUFFED_FRAME_TOO_LONG (TODO)
                            break;
                        case PROT_ERROR_MESSAGE_TOO_LONG:
//                          // Procesamiento del error MESSAGE TOO LONG (TODO)
                            break;
                        }
                        case PROT_ERROR_INCORRECT_PARAM_SIZE:
                        {
                            // Procesamiento del error INCORRECT PARAM SIZE (TODO)
                        }
                        break;
                    }
                    break;
                case MENSAJE_AUTO:{
                    PARAM_MENSAJE_AUTO parametro;

                    if (check_and_extract_message_param(ptrtoreceivedparam, i32Numdatos, sizeof(parametro),&parametro)>0){
                        automatico= parametro.automatico;
                    }else{//Error de tama�o de parametro
                        ui32Errors++; // Tratamiento del error
                    }

                }
                 break;
                case MENSAJE_PRESIONMAR:{
                    PARAM_MENSAJE_PRESIONMAR parametro;

                    if (check_and_extract_message_param(ptrtoreceivedparam, i32Numdatos, sizeof(parametro),&parametro)>0){
                        xQueueSend (colaPresionMar,&parametro.presion,0);
                    }else{//Error de tama�o de parametro
                        ui32Errors++; // Tratamiento del error
                    }

                }
                 break;

                case MENSAJE_MUESTREOPRESION:{
                    PARAM_MENSAJE_MUESTREOPRESION parametro;

                    if (check_and_extract_message_param(ptrtoreceivedparam, i32Numdatos, sizeof(parametro),&parametro)>0){
                        xQueueSend (colaPresionMuestreo,&parametro.IndexMuestreo,0);
                    }else{//Error de tama�o de parametro
                        ui32Errors++; // Tratamiento del error
                    }

                }

                 break;
                case MENSAJE_ADCTEMP:{
                    PARAM_MENSAJE_ADCTEMP parametro;

                    if (check_and_extract_message_param(ptrtoreceivedparam, i32Numdatos, sizeof(parametro),&parametro)>0){
                        xQueueSend (colaTempADC,&parametro.IndexADC,0);
                    }else{//Error de tama�o de parametro
                        ui32Errors++; // Tratamiento del error
                    }

                }
                 break;
                case MENSAJE_HUMRESOLUCION:{
                    PARAM_MENSAJE_HUMRESOLUCION parametro;

                    if (check_and_extract_message_param(ptrtoreceivedparam, i32Numdatos, sizeof(parametro),&parametro)>0){
                        xQueueSend (colaHumMR,&parametro.IndexMR,0);
                    }else{//Error de tama�o de parametro
                        ui32Errors++; // Tratamiento del error
                    }

                }
                 break;

                case MENSAJE_DECLINACIONBRUJULA:{
                    PARAM_MENSAJE_DECLINACIONBRUJULA parametro;

                    if (check_and_extract_message_param(ptrtoreceivedparam, i32Numdatos, sizeof(parametro),&parametro)>0){
                        xQueueSend (colaDeclinacion,&parametro.Declinacion,0);
                    }else{//Error de tama�o de parametro
                        ui32Errors++; // Tratamiento del error
                    }

                }
                 break;

                case MENSAJE_DLPFMOV:{
                    PARAM_MENSAJE_DLPFMOV parametro;

                    if (check_and_extract_message_param(ptrtoreceivedparam, i32Numdatos, sizeof(parametro),&parametro)>0){
                        xQueueSend (colaMovimientoDLPF,&parametro.IndexDLPF,0);
                    }else{//Error de tama�o de parametro
                        ui32Errors++; // Tratamiento del error
                    }

                }
                 break;
                case MENSAJE_GIROMOV:{
                    PARAM_MENSAJE_GIROMOV parametro;

                    if (check_and_extract_message_param(ptrtoreceivedparam, i32Numdatos, sizeof(parametro),&parametro)>0){
                        xQueueSend (colaMovimientoGIRO,&parametro.IndexGIRO,0);
                    }else{//Error de tama�o de parametro
                        ui32Errors++; // Tratamiento del error
                    }

                }
                 break;
                default:
                 {
                    PARAM_MENSAJE_NO_IMPLEMENTADO parametro;
                    parametro.message=ui8Message;
                    //El mensaje esta bien pero no esta implementado
                    i32Numdatos=create_frame(pui8Frame,MENSAJE_NO_IMPLEMENTADO,&parametro,sizeof(parametro),MAX_FRAME_SIZE);
                    if (i32Numdatos>=0)
                    {
                        send_frame(pui8Frame,i32Numdatos);
                    }
                    break;
                 }
                }// switch
            }
        }else{ // if (ui32Numdatos >0)
            //Error de recepcion de trama(PROT_ERROR_RX_FRAME_TOO_LONG), ignorar el paquete
            ui32Errors++;
            // Procesamiento del error (TODO)
        }
    }
}

///////////////////////////////////////////TAREA DE LA TEMPERATURA/////////////////////////////////////////////////
static portTASK_FUNCTION(TEMPTask,pvParameters)
                        {

    int Modo, ADC;
    float fAmbient, fObject;
    QueueSetMemberHandle_t Activado;
    // Delay for 10 milliseconds for reset to complete.
     // Not explicitly required. Datasheet does not say how long a reset takes.
     //
     MAP_SysCtlDelay(MAP_SysCtlClockGet() / (100));

     TMP006ReadModifyWrite(&g_sTMP006Inst, TMP006_O_CONFIG,
                              ~TMP006_CONFIG_EN_DRDY_PIN_M,
                              TMP006_CONFIG_EN_DRDY_PIN, TMP006AppCallback,
                              &g_sTMP006Inst);
     TMP006AppI2CWait(__FILE__, __LINE__);
    while(1)
        {
            Activado = xQueueSelectFromSet( grupo_colasTemperatura, portMAX_DELAY);
            if (Activado==semaforoTemperatura){
                xSemaphoreTake(semaforoTemperatura,0);

                TMP006AppI2CWait(__FILE__, __LINE__);

                // Get a local copy of the latest data in float format.
                //
                TMP006DataTemperatureGetFloat(&g_sTMP006Inst, &fAmbient, &fObject);



                //QT
                unsigned char frame[MAX_FRAME_SIZE];
                int num_datos;
                PARAM_MENSAJE_TEMP temp;
                temp.temperatura= fObject;
                num_datos=create_frame(frame, MENSAJE_TEMP, &temp, sizeof(temp), MAX_FRAME_SIZE);
                if (num_datos>=0){
                    send_frame(frame, num_datos);
            }
                }else if(Activado==colaTempADC){
                    xQueueReceive(colaTempADC,&ADC,0);
                    switch(Modo){
                        case 0:
                            TMP006ReadModifyWrite(&g_sTMP006Inst, TMP006_O_CONFIG,
                                                      ~TMP006_CONFIG_CR_M ,
                                                      TMP006_CONFIG_CR_4 , TMP006AppCallback,
                                                      &g_sTMP006Inst);
                            TMP006AppI2CWait(__FILE__, __LINE__);
                        break;
                        case 1:
                            TMP006ReadModifyWrite(&g_sTMP006Inst, TMP006_O_CONFIG,
                                                      ~TMP006_CONFIG_CR_M  ,
                                                      TMP006_CONFIG_CR_2 , TMP006AppCallback,
                                                      &g_sTMP006Inst);
                            TMP006AppI2CWait(__FILE__, __LINE__);
                        break;
                        case 2:
                            TMP006ReadModifyWrite(&g_sTMP006Inst, TMP006_O_CONFIG,
                                                      ~TMP006_CONFIG_CR_M ,
                                                      TMP006_CONFIG_CR_1 , TMP006AppCallback,
                                                      &g_sTMP006Inst);
                            TMP006AppI2CWait(__FILE__, __LINE__);
                        break;
                        case 3:
                            TMP006ReadModifyWrite(&g_sTMP006Inst, TMP006_O_CONFIG,
                                                      ~TMP006_CONFIG_CR_M  ,
                                                      TMP006_CONFIG_CR_0_5 , TMP006AppCallback,
                                                      &g_sTMP006Inst);
                            TMP006AppI2CWait(__FILE__, __LINE__);
                        break;
                        case 4:
                            TMP006ReadModifyWrite(&g_sTMP006Inst, TMP006_O_CONFIG,
                                                      ~TMP006_CONFIG_CR_M  ,
                                                      TMP006_CONFIG_CR_0_25 , TMP006AppCallback,
                                                      &g_sTMP006Inst);
                            TMP006AppI2CWait(__FILE__, __LINE__);
                        break;
                }
            }
        }

  }



///////////////////////////////////////////TAREA DE LA HUMEDAD/////////////////////////////////////////////////
static portTASK_FUNCTION(HUMTask,pvParameters)
                        {
    //nos aseguramos de que el semaforo esta cerrado
    QueueSetMemberHandle_t Activado;
    // xSemaphoreTake(semaforoHumedad,0);
    float fTemperature, fHumidity;
    int MR;
    while(1)
       {
        Activado = xQueueSelectFromSet( grupo_colasHumedad, portMAX_DELAY);

        if (Activado==semaforoHumedad){

               //espera infinita hasta que el semaforo se abra
               xSemaphoreTake(semaforoHumedad,portMAX_DELAY);
               //
               // Write the command to start a humidity measurement
               //
               SHT21Write(&g_sSHT21Inst, SHT21_CMD_MEAS_RH, g_sSHT21Inst.pui8Data, 0,
                          SHT21AppCallback, &g_sSHT21Inst);

               //
               // Wait for the I2C transactions to complete before moving forward
               //
               SHT21AppI2CWait(__FILE__, __LINE__);

               //
               // Wait 33 milliseconds before attempting to get the result. Datasheet
               // claims this can take as long as 29 milliseconds
               //
               MAP_SysCtlDelay(MAP_SysCtlClockGet() / (30 * 3));

               //
               // Get the raw data from the sensor over the I2C bus
               //
               SHT21DataRead(&g_sSHT21Inst, SHT21AppCallback, &g_sSHT21Inst);

               //
               // Wait for the I2C transactions to complete before moving forward
               //
               SHT21AppI2CWait(__FILE__, __LINE__);

               //
               // Get a copy of the most recent raw data in floating point format.
               //
               SHT21DataHumidityGetFloat(&g_sSHT21Inst, &fHumidity);

               //
               // Write the command to start a temperature measurement
               //
               SHT21Write(&g_sSHT21Inst, SHT21_CMD_MEAS_T, g_sSHT21Inst.pui8Data, 0,
                          SHT21AppCallback, &g_sSHT21Inst);

               //
               // Wait for the I2C transactions to complete before moving forward
               //
               SHT21AppI2CWait(__FILE__, __LINE__);

               //
               // Wait 100 milliseconds before attempting to get the result. Datasheet
               // claims this can take as long as 85 milliseconds
               //
               MAP_SysCtlDelay(MAP_SysCtlClockGet() / (10 * 3));

               //
               // Read the conversion data from the sensor over I2C.
               //
               SHT21DataRead(&g_sSHT21Inst, SHT21AppCallback, &g_sSHT21Inst);

               //
               // Wait for the I2C transactions to complete before moving forward
               //
               SHT21AppI2CWait(__FILE__, __LINE__);


               //
               // Get the most recent temperature result as a float in celcius.
               //
               SHT21DataTemperatureGetFloat(&g_sSHT21Inst, &fTemperature);

               //
               // Convert the floats to an integer part and fraction part for easy
               // print. Humidity is returned as 0.0 to 1.0 so multiply by 100 to get
               // percent humidity.
               //
               fHumidity *= 100.0f;

               unsigned char frame[MAX_FRAME_SIZE];
               int num_datos;
               PARAM_MENSAJE_HUM humedad;
               humedad.humedad= fHumidity;
               num_datos=create_frame(frame, MENSAJE_HUM, &humedad, sizeof(humedad), MAX_FRAME_SIZE);
               if (num_datos>=0){
                   send_frame(frame, num_datos);
               }
        }else if(Activado==colaHumMR){
            xQueueReceive(colaHumMR,&MR,0);
            switch(MR){
                case 0:
                    SHT21ReadModifyWrite(&g_sSHT21Inst, SHT21_CMD_WRITE_CONFIG ,  ~SHT21_CONFIG_RES_M ,
                                         SHT21_CONFIG_RES_12, SHT21AppCallback,
                                         &g_sSHT21Inst);
                    SHT21AppI2CWait(__FILE__, __LINE__);
                break;
                case 1:
                    SHT21ReadModifyWrite(&g_sSHT21Inst, SHT21_CMD_WRITE_CONFIG ,  ~SHT21_CONFIG_RES_M ,
                                         SHT21_CONFIG_RES_8, SHT21AppCallback,
                                         &g_sSHT21Inst);
                    SHT21AppI2CWait(__FILE__, __LINE__);
                break;
                case 2:
                    SHT21ReadModifyWrite(&g_sSHT21Inst, SHT21_CMD_WRITE_CONFIG ,  ~SHT21_CONFIG_RES_M ,
                                         SHT21_CONFIG_RES_10, SHT21AppCallback,
                                         &g_sSHT21Inst);
                    SHT21AppI2CWait(__FILE__, __LINE__);
               break;
                case 3:
                    SHT21ReadModifyWrite(&g_sSHT21Inst, SHT21_CMD_WRITE_CONFIG ,  ~SHT21_CONFIG_RES_M ,
                                         SHT21_CONFIG_RES_11, SHT21AppCallback,
                                         &g_sSHT21Inst);
                    SHT21AppI2CWait(__FILE__, __LINE__);
                break;
            }
        }
       }
  }

///////////////////////////////////////////TAREA DE LA PRESION/////////////////////////////////////////////////
static portTASK_FUNCTION(PRETask,pvParameters)
                        {
    //nos aseguramos de que el semaforo esta cerrado
    QueueSetMemberHandle_t Activado;
    // xSemaphoreTake(semaforoPresion,0);
    float fPressure, fAltitude;
    float PresionMar=101300.0f;
    int Muestreo;


    while(1)
       {
        //espera a que se desbloquee uno de los IPC
        Activado = xQueueSelectFromSet( grupo_colasPresion, portMAX_DELAY);

        if (Activado==semaforoPresion)
                {
                xSemaphoreTake(semaforoPresion,0);
               //
               // Read the data from the BMP180 over I2C.  This command starts a
               // temperature measurement.  Then polls until temperature is ready.
               // Then automatically starts a pressure measurement and polls for that
               // to complete. When both measurement are complete and in the local
               // buffer then the application callback is called from the I2C
               // interrupt context.  Polling is done on I2C interrupts allowing
               // processor to continue doing other tasks as needed.
               //
               BMP180DataRead(&g_sBMP180Inst, BMP180AppCallback, &g_sBMP180Inst);
               BMP180AppI2CWait(__FILE__, __LINE__);


               //
               // Get a local copy of the latest air pressure data in float format.
               //
               BMP180DataPressureGetFloat(&g_sBMP180Inst, &fPressure);

               //
               // Calculate the altitude.
               //
               fAltitude = 44330.0f * (1.0f - powf(fPressure / PresionMar,
                                                   1.0f / 5.255f));


               unsigned char frame[MAX_FRAME_SIZE];
               int num_datos;
               PARAM_MENSAJE_ALT altitud;
               altitud.altitud= fAltitude;
               altitud.presion= fPressure;

               num_datos=create_frame(frame, MENSAJE_ALT, &altitud, sizeof(altitud), MAX_FRAME_SIZE);
               if (num_datos>=0){
                   send_frame(frame, num_datos);
               }
         }else if(Activado== colaPresionMar){
             xQueueReceive(colaPresionMar,&PresionMar,0);

         }else if(Activado== colaPresionMuestreo){
             xQueueReceive(colaPresionMuestreo,&Muestreo,0);

             switch(Muestreo){
                 case 0:
                     BMP180ReadModifyWrite(&g_sBMP180Inst, BMP180_O_CTRL_MEAS ,  ~BMP180_CTRL_MEAS_OSS_M ,
                                          BMP180_CTRL_MEAS_OSS_1, BMP180AppCallback,
                                          &g_sBMP180Inst);
                     BMP180AppI2CWait(__FILE__, __LINE__);

                 break;
                 case 1:
                     BMP180ReadModifyWrite(&g_sBMP180Inst, BMP180_O_CTRL_MEAS ,  ~BMP180_CTRL_MEAS_OSS_M ,
                                          BMP180_CTRL_MEAS_OSS_2, BMP180AppCallback,
                                          &g_sBMP180Inst);
                     BMP180AppI2CWait(__FILE__, __LINE__);

                 break;
                 case 2:
                     BMP180ReadModifyWrite(&g_sBMP180Inst, BMP180_O_CTRL_MEAS ,  ~BMP180_CTRL_MEAS_OSS_M ,
                                          BMP180_CTRL_MEAS_OSS_4, BMP180AppCallback,
                                          &g_sBMP180Inst);
                     BMP180AppI2CWait(__FILE__, __LINE__);

                 break;
                 case 3:
                     BMP180ReadModifyWrite(&g_sBMP180Inst, BMP180_O_CTRL_MEAS ,  ~BMP180_CTRL_MEAS_OSS_M ,
                                          BMP180_CTRL_MEAS_OSS_8, BMP180AppCallback,
                                          &g_sBMP180Inst);
                     BMP180AppI2CWait(__FILE__, __LINE__);

                 break;
             }

         }
       }//while end
  }

///////////////////////////////////////////TAREA DE LA LUZ/////////////////////////////////////////////////
static portTASK_FUNCTION(LUZTask,pvParameters)
                        {
     QueueSetMemberHandle_t Activado;
     int Rango, Resolucion;
     float fAmbient;
     uint8_t ui8Mask;

     //
     // Configure the ISL29023 to measure ambient light continuously. Set a 8
     // sample persistence before the INT pin is asserted. Clears the INT flag.
     // Persistence setting of 8 is sufficient to ignore camera flashes.
     //
     ui8Mask = (ISL29023_CMD_I_OP_MODE_M | ISL29023_CMD_I_INT_PERSIST_M |
                ISL29023_CMD_I_INT_FLAG_M);
     ISL29023ReadModifyWrite(&g_sISL29023Inst, ISL29023_O_CMD_I, ~ui8Mask,
                             (ISL29023_CMD_I_OP_MODE_ALS_CONT |
                              ISL29023_CMD_I_INT_PERSIST_8),
                             ISL29023AppCallback, &g_sISL29023Inst);

     //
     // Wait for transaction to complete
     //
     ISL29023AppI2CWait(__FILE__, __LINE__);

     automatico=true; //automatico por defecto

     while(1)
     {
       Activado = xQueueSelectFromSet( grupo_colasLuz, portMAX_DELAY);

       if (Activado==semaforoLuz)
           {
            //espera infinita hasta que el semáforo se abra
            xSemaphoreTake(semaforoLuz,portMAX_DELAY);


            if(automatico==true){

                ISL29023DataRead(&g_sISL29023Inst, ISL29023AppCallback, &g_sISL29023Inst);

                //
                // Get a local floating point copy of the latest light data
                //
                ISL29023DataLightVisibleGetFloat(&g_sISL29023Inst, &fAmbient);

                //envio a qt
                unsigned char frame[MAX_FRAME_SIZE];
                int num_datos;
                PARAM_MENSAJE_NOCHE fondo;
                //40 lux es el valor en un dia nublado
               if (fAmbient<40){
                    fondo.modo=1;
                num_datos=create_frame(frame, MENSAJE_NOCHE, &fondo, sizeof(fondo), MAX_FRAME_SIZE);
                    if (num_datos>=0){
                        send_frame(frame, num_datos);
                    }
               }else {
                    fondo.modo=0;
                    num_datos=create_frame(frame, MENSAJE_NOCHE, &fondo, sizeof(fondo), MAX_FRAME_SIZE);
                     if (num_datos>=0){
                         send_frame(frame, num_datos);
                       }
                }
           }
         }else if(Activado==colaRangoLuz){
             xQueueReceive(colaRangoLuz,&Rango,0);

             switch(Rango){
             case 0:
                 ISL29023ReadModifyWrite(&g_sISL29023Inst, ISL29023_O_CMD_II, ~ISL29023_CMD_II_RANGE_M,
                                         ISL29023_CMD_II_RANGE_1K,
                                         ISL29023AppCallback, &g_sISL29023Inst);
                 ISL29023AppI2CWait(__FILE__, __LINE__);
             break;
             case 1:
                 ISL29023ReadModifyWrite(&g_sISL29023Inst, ISL29023_O_CMD_II, ~ui8Mask,
                                         ISL29023_CMD_II_RANGE_4K,
                                         ISL29023AppCallback, &g_sISL29023Inst);
                 ISL29023AppI2CWait(__FILE__, __LINE__);
             break;
             case 2:
                 ISL29023ReadModifyWrite(&g_sISL29023Inst, ISL29023_O_CMD_II, ~ui8Mask,
                                         ISL29023_CMD_II_RANGE_16K,
                                         ISL29023AppCallback, &g_sISL29023Inst);
                 ISL29023AppI2CWait(__FILE__, __LINE__);
             break;
             case 3:
                 ISL29023ReadModifyWrite(&g_sISL29023Inst, ISL29023_O_CMD_II, ~ui8Mask,
                                         ISL29023_CMD_II_RANGE_64K,
                                         ISL29023AppCallback, &g_sISL29023Inst);
                 ISL29023AppI2CWait(__FILE__, __LINE__);
             break;
         }
         }else if(Activado==colaResolLuz){
             xQueueReceive(colaResolLuz,&Resolucion,0);
             switch(Resolucion){
                 case 0:
                     ISL29023ReadModifyWrite(&g_sISL29023Inst, ISL29023_O_CMD_II, ~ISL29023_CMD_II_ADC_RES_M,
                                             ISL29023_CMD_II_ADC_RES_16 ,
                                             ISL29023AppCallback, &g_sISL29023Inst);
                     ISL29023AppI2CWait(__FILE__, __LINE__);
                 break;
                 case 1:
                     ISL29023ReadModifyWrite(&g_sISL29023Inst, ISL29023_O_CMD_II, ~ISL29023_CMD_II_ADC_RES_M,
                                             ISL29023_CMD_II_ADC_RES_12 ,
                                             ISL29023AppCallback, &g_sISL29023Inst);
                     ISL29023AppI2CWait(__FILE__, __LINE__);
                 break;
                 case 2:
                     ISL29023ReadModifyWrite(&g_sISL29023Inst, ISL29023_O_CMD_II, ~ISL29023_CMD_II_ADC_RES_M,
                                             ISL29023_CMD_II_ADC_RES_8,
                                             ISL29023AppCallback, &g_sISL29023Inst);
                     ISL29023AppI2CWait(__FILE__, __LINE__);
                 break;
                 case 3:
                     ISL29023ReadModifyWrite(&g_sISL29023Inst, ISL29023_O_CMD_II, ~ISL29023_CMD_II_ADC_RES_M,
                                             ISL29023_CMD_II_ADC_RES_4 ,
                                             ISL29023AppCallback, &g_sISL29023Inst);
                     ISL29023AppI2CWait(__FILE__, __LINE__);
                 break;
             }
         }

     }
}


///////////////////////////////////////////TAREA DEL MOVIMIENTO/////////////////////////////////////////////////
static portTASK_FUNCTION(MOVTask,pvParameters)
                        {
    uint_fast32_t ui32CompDCMStarted;

     // Registro de configuracion: filtro paso bajo
     MPU9150ReadModifyWrite(&g_sMPU9150Inst, MPU9150_O_CONFIG, ~MPU9150_CONFIG_DLPF_CFG_M,
     MPU9150_CONFIG_DLPF_CFG_10_10, MPU9150AppCallback, &g_sMPU9150Inst);
     MPU9150AppI2CWait(__FILE__, __LINE__);

     // Registro de configuracion del giroscopo: rango
     MPU9150ReadModifyWrite(&g_sMPU9150Inst, MPU9150_O_GYRO_CONFIG, ~MPU9150_GYRO_CONFIG_FS_SEL_M,
     MPU9150_GYRO_CONFIG_FS_SEL_500, MPU9150AppCallback, &g_sMPU9150Inst);
     MPU9150AppI2CWait(__FILE__, __LINE__);

     // Registro de configuracion del acelerometro: rango
     MPU9150ReadModifyWrite(&g_sMPU9150Inst, MPU9150_O_ACCEL_CONFIG, ~MPU9150_ACCEL_CONFIG_AFS_SEL_M,
     MPU9150_ACCEL_CONFIG_AFS_SEL_8G, MPU9150AppCallback, &g_sMPU9150Inst);
     MPU9150AppI2CWait(__FILE__, __LINE__);

     // Registro de configuracion de la frecuencia de muestreo
     MPU9150ReadModifyWrite(&g_sMPU9150Inst, MPU9150_O_SMPLRT_DIV, ~MPU9150_SMPLRT_DIV_M,
     199, MPU9150AppCallback, &g_sMPU9150Inst);
     MPU9150AppI2CWait(__FILE__, __LINE__);


     // Configure the data ready interrupt pin output of the MPU9150
     MPU9150ReadModifyWrite(&g_sMPU9150Inst, MPU9150_O_INT_PIN_CFG,
     ~(MPU9150_INT_PIN_CFG_INT_LEVEL | MPU9150_INT_PIN_CFG_INT_RD_CLEAR | MPU9150_INT_PIN_CFG_LATCH_INT_EN),
     MPU9150_INT_PIN_CFG_INT_LEVEL | MPU9150_INT_PIN_CFG_INT_RD_CLEAR | MPU9150_INT_PIN_CFG_LATCH_INT_EN, MPU9150AppCallback, &g_sMPU9150Inst);
     MPU9150AppI2CWait(__FILE__, __LINE__);

     MPU9150ReadModifyWrite(&g_sMPU9150Inst, MPU9150_O_INT_ENABLE,
     ~(MPU9150_INT_ENABLE_DATA_RDY_EN), MPU9150_INT_ENABLE_DATA_RDY_EN, MPU9150AppCallback, &g_sMPU9150Inst);
     MPU9150AppI2CWait(__FILE__, __LINE__);


     // Initialize the DCM system. 50 hz sample rate.
     // accel weight = .2, gyro weight = .8, mag weight = .2
     //
     CompDCMInit(&g_sCompDCMInst, 1.0f / 50.0f, 0.2f, 0.6f, 0.2f);

     ui32CompDCMStarted =0;

     pfRoll =0.0;
     pfPitch=0.0;
     pfYaw=0.0;


     QueueSetMemberHandle_t Activado;
     float declinacion=1.26;
     float angulo;
     float pi=3.141592;
     int valDLPF;
     int ConfGiro;

     while(1)
     {
         Activado = xQueueSelectFromSet( grupo_colasMovimiento, portMAX_DELAY);

         if (Activado==semaforoMovimiento)
                 {
             xSemaphoreTake(semaforoMovimiento,portMAX_DELAY);
             //espera infinita hasta que el semaforo se abra
             //xSemaphoreTake(semaforoMovimiento,portMAX_DELAY);
             MPU9150AppI2CWait(__FILE__, __LINE__);


             //
             // Get floating point version of angular velocities in rad/sec
             //
             MPU9150DataGyroGetFloat(&g_sMPU9150Inst,&pfGyroX, &pfGyroY,&pfGyroZ);
             // Get floating point version of the Accel Data in m/s^2.
             //
             MPU9150DataAccelGetFloat(&g_sMPU9150Inst, &pfAccelX, &pfAccelY,
                                      &pfAccelZ);
             //
             // Get floating point version of magnetic fields strength in tesla
             //
             MPU9150DataMagnetoGetFloat(&g_sMPU9150Inst, &pfMagX,&pfMagY,&pfMagZ);

             //
                     // Check if this is our first data ever.
                     //
                     if(ui32CompDCMStarted == 0)
                    {
                          //
                          // Set flag indicating that DCM is started.
                         // Perform the seeding of the DCM with the first data set.
                         //
                         ui32CompDCMStarted = 1;
                         CompDCMMagnetoUpdate(&g_sCompDCMInst, pfMagX, pfMagY,
                                              pfMagZ);
                         CompDCMAccelUpdate(&g_sCompDCMInst, pfAccelX, pfAccelY,
                                           pfAccelZ);
                         CompDCMGyroUpdate(&g_sCompDCMInst, pfGyroX, pfGyroY,
                                           pfGyroZ);
                         CompDCMStart(&g_sCompDCMInst);
                     }
                     else
                     {
                         //
                         // DCM Is already started.  Perform the incremental update.
                         //
                         CompDCMMagnetoUpdate(&g_sCompDCMInst, pfMagX, pfMagY,
                                              pfMagZ);
                        CompDCMAccelUpdate(&g_sCompDCMInst, pfAccelX, pfAccelY,
                                           pfAccelZ);
                         CompDCMGyroUpdate(&g_sCompDCMInst, -pfGyroX, -pfGyroY,
                                           -pfGyroZ);
                         CompDCMUpdate(&g_sCompDCMInst);
                     }

            // Increment the skip counter.  Skip counter is used so we do not
            // overflow the UART with data.
            g_ui32PrintSkipCounter++;

            if(g_ui32PrintSkipCounter >= PRINT_SKIP_COUNT)
              {


                 g_ui32PrintSkipCounter = 0;
                 //
                 // Get Euler data. (Roll Pitch Yaw)
                 //
                 CompDCMComputeEulers(&g_sCompDCMInst, &pfRoll, &pfPitch,
                                      &pfYaw);
                 //
                 // convert mag data to micro-tesla for better human interpretation.
                 //
                 pfMagX *= 1e6;
                 pfMagY *= 1e6;
                 pfMagZ *= 1e6;

                 unsigned char frame[MAX_FRAME_SIZE];
                 int num_datos;
                 PARAM_MENSAJE_BRUJULA bruj;

                 angulo = atan2(pfMagY, pfMagX);
                 angulo=angulo*(180/pi);//convertimos de Radianes a grados
                 angulo=angulo+declinacion; //corregimos la declinación magnética

                 if(angulo<0) angulo=angulo+360;

                 bruj.Brujula=angulo;
                 num_datos=create_frame(frame, MENSAJE_BRUJULA, &bruj, sizeof(bruj), MAX_FRAME_SIZE);
                 if (num_datos>=0){
                     send_frame(frame, num_datos);
                 }


                 //
                 // Convert Eulers to degrees. 180/PI = 57.29...
                 // Convert Yaw to 0 to 360 to approximate compass headings.
                 //
                 pfRoll *= radian;
                 pfPitch *= radian;
                 pfYaw *= radian;
                 if(pfYaw < 0)
                 {
                     pfYaw += 360.0f;
                 }


                 //envio a qt


                 PARAM_MENSAJE_EJES ejes;

                 ejes.fPitch= pfPitch;
                 ejes.fRoll= pfRoll;
                 ejes.fYaw= pfYaw;

                 num_datos=create_frame(frame, MENSAJE_EJES, &ejes, sizeof(ejes), MAX_FRAME_SIZE);
                 if (num_datos>=0){
                         send_frame(frame, num_datos);
                 }
              }
         }else if(Activado== colaDeclinacion){
             xQueueReceive(colaDeclinacion,&declinacion,0);
         }else if(Activado== colaMovimientoDLPF){
             xQueueReceive(colaMovimientoDLPF,&valDLPF,0);
             switch(valDLPF){
                 case 0:
                     MPU9150ReadModifyWrite(&g_sMPU9150Inst, MPU9150_O_CONFIG, ~MPU9150_CONFIG_DLPF_CFG_M,
                     MPU9150_CONFIG_DLPF_CFG_260_256, MPU9150AppCallback, &g_sMPU9150Inst);
                     MPU9150AppI2CWait(__FILE__, __LINE__);
                 break;
                 case 1:
                     MPU9150ReadModifyWrite(&g_sMPU9150Inst, MPU9150_O_CONFIG, ~MPU9150_CONFIG_DLPF_CFG_M,
                     MPU9150_CONFIG_DLPF_CFG_184_188, MPU9150AppCallback, &g_sMPU9150Inst);
                     MPU9150AppI2CWait(__FILE__, __LINE__);
                 break;
                 case 2:
                     MPU9150ReadModifyWrite(&g_sMPU9150Inst, MPU9150_O_CONFIG, ~MPU9150_CONFIG_DLPF_CFG_M,
                     MPU9150_CONFIG_DLPF_CFG_94_98, MPU9150AppCallback, &g_sMPU9150Inst);
                     MPU9150AppI2CWait(__FILE__, __LINE__);
                 break;
                 case 3:
                     MPU9150ReadModifyWrite(&g_sMPU9150Inst, MPU9150_O_CONFIG, ~MPU9150_CONFIG_DLPF_CFG_M,
                     MPU9150_CONFIG_DLPF_CFG_44_42, MPU9150AppCallback, &g_sMPU9150Inst);
                     MPU9150AppI2CWait(__FILE__, __LINE__);
                 break;
                 case 4:
                     MPU9150ReadModifyWrite(&g_sMPU9150Inst, MPU9150_O_CONFIG, ~MPU9150_CONFIG_DLPF_CFG_M,
                     MPU9150_CONFIG_DLPF_CFG_21_20, MPU9150AppCallback, &g_sMPU9150Inst);
                     MPU9150AppI2CWait(__FILE__, __LINE__);
                 break;
                 case 5:
                     MPU9150ReadModifyWrite(&g_sMPU9150Inst, MPU9150_O_CONFIG, ~MPU9150_CONFIG_DLPF_CFG_M,
                     MPU9150_CONFIG_DLPF_CFG_10_10, MPU9150AppCallback, &g_sMPU9150Inst);
                     MPU9150AppI2CWait(__FILE__, __LINE__);
                 break;
                 case 6:
                     MPU9150ReadModifyWrite(&g_sMPU9150Inst, MPU9150_O_CONFIG, ~MPU9150_CONFIG_DLPF_CFG_M,
                     MPU9150_CONFIG_DLPF_CFG_5_5, MPU9150AppCallback, &g_sMPU9150Inst);
                     MPU9150AppI2CWait(__FILE__, __LINE__);
                 break;
                 case 7:
                     MPU9150ReadModifyWrite(&g_sMPU9150Inst, MPU9150_O_CONFIG, ~MPU9150_CONFIG_DLPF_CFG_M,
                     MPU9150_CONFIG_EXT_SYNC_SET_S, MPU9150AppCallback, &g_sMPU9150Inst);
                     MPU9150AppI2CWait(__FILE__, __LINE__);
                 break;
             }
         }else if(Activado== colaMovimientoGIRO){
             xQueueReceive(colaMovimientoGIRO,&ConfGiro,0);
             switch(ConfGiro){
                 case 0:
                     MPU9150ReadModifyWrite(&g_sMPU9150Inst, MPU9150_O_GYRO_CONFIG, ~MPU9150_GYRO_CONFIG_FS_SEL_M,
                     MPU9150_GYRO_CONFIG_FS_SEL_250, MPU9150AppCallback, &g_sMPU9150Inst);
                     MPU9150AppI2CWait(__FILE__, __LINE__);
                 break;
                 case 1:
                     MPU9150ReadModifyWrite(&g_sMPU9150Inst, MPU9150_O_GYRO_CONFIG, ~MPU9150_GYRO_CONFIG_FS_SEL_M,
                     MPU9150_GYRO_CONFIG_FS_SEL_500, MPU9150AppCallback, &g_sMPU9150Inst);
                     MPU9150AppI2CWait(__FILE__, __LINE__);
                 break;
                 case 2:
                     MPU9150ReadModifyWrite(&g_sMPU9150Inst, MPU9150_O_GYRO_CONFIG, ~MPU9150_GYRO_CONFIG_FS_SEL_M,
                     MPU9150_GYRO_CONFIG_FS_SEL_1000, MPU9150AppCallback, &g_sMPU9150Inst);
                     MPU9150AppI2CWait(__FILE__, __LINE__);
                 break;
                 case 3:
                     MPU9150ReadModifyWrite(&g_sMPU9150Inst, MPU9150_O_GYRO_CONFIG, ~MPU9150_GYRO_CONFIG_FS_SEL_M,
                     MPU9150_GYRO_CONFIG_FS_SEL_2000, MPU9150AppCallback, &g_sMPU9150Inst);
                     MPU9150AppI2CWait(__FILE__, __LINE__);
                 break;
                     }
                 }
             }
         }

//*****************************************************************************
//
// Funcion main(), Inicializa los perifericos, crea las tareas, etc... y arranca el bucle del sistema
//
//*****************************************************************************
int main(void)
{

	//
	// Set the clocking to run at 40 MHz from the PLL.
	//
	MAP_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
			SYSCTL_OSC_MAIN);	//Ponermos el reloj principal a 50 MHz (200 Mhz del Pll dividido por 4)


    // Get the system clock speed.
    g_ui32SystemClock = SysCtlClockGet();
    //Habilita el clock gating de los perifericos durante el bajo consumo --> perifericos que se desee activos en modo Sleep
    //                                                                        deben habilitarse con SysCtlPeripheralSleepEnable
    MAP_SysCtlPeripheralClockGating(true);
     // Inicializa el subsistema de medida del uso de CPU (mide el tiempo que la CPU no esta dormida)
     // Para eso utiliza un timer, que aqui hemos puesto que sea el TIMER3 (ultimo parametro que se pasa a la funcion)
     // (y por tanto este no se deberia utilizar para otra cosa).
     CPUUsageInit(g_ui32SystemClock, configTICK_RATE_HZ/10, 3);

    //
    // Enable the peripherals used by this example.
    //
     MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
     MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    //
    // Initialize the UART.
    //
    ConfigureUART();


    //
    // The I2C3 peripheral must be enabled before use.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    //
    // Configure the pin muxing for I2C3 functions on port D0 and D1.
    // This step is not necessary if your part does not support pin muxing.
    //
    MAP_GPIOPinConfigure(GPIO_PD0_I2C3SCL);
    MAP_GPIOPinConfigure(GPIO_PD1_I2C3SDA);

    //
    // Select the I2C function for these pins.  This function will also
    // configure the GPIO pins pins for I2C operation, setting them to
    // open-drain operation with weak pull-ups.  Consult the data sheet
    // to see which functions are allocated per pin.
    //
    GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
    MAP_GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);

    //
    // Configure and Enable the GPIO interrupt. Used for DRDY from the TMP006
    //
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0);
    GPIOIntEnable(GPIO_PORTE_BASE, GPIO_PIN_0);
    MAP_GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_FALLING_EDGE);

    MAP_GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_5);
    GPIOIntEnable(GPIO_PORTE_BASE, GPIO_PIN_5);
    MAP_GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_FALLING_EDGE);
    MAP_IntEnable(INT_GPIOE);

    MAP_GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_2);
    MAP_GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_FALLING_EDGE);
    MAP_IntEnable(INT_GPIOB);
    //
    // Keep only some parts of the systems running while in sleep mode.
    // UART0 is the virtual serial port
    // I2C3 is the I2C interface to the TMP006 and MPU9150
    //

    //Habilita el clock gating de los perifericos durante el bajo consumo --> perifericos que se desee activos en modo Sleep
      //                                                                        deben habilitarse con
    MAP_SysCtlPeripheralClockGating(true);
    MAP_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOB);//Esto es necesario para que el GPIOB siga funcionando en bajo consumo
    MAP_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOE);//Esto es necesario para que el GPIOE siga funcionando en bajo consumo
    MAP_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART0);//Esto es necesario para que el UART0 siga funcionando en bajo consumo
    MAP_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_I2C3);//Esto es necesario para que el I2C siga funcionando en bajo consumo

    // Initialización I2C3 peripheral.
    //
    I2CMInit(&g_sI2CInst, I2C3_BASE, INT_I2C3, 0xff, 0xff,
             SysCtlClockGet());

    //
    // Initialización the BMP180
    BMP180Init(&g_sBMP180Inst, &g_sI2CInst, BMP180_I2C_ADDRESS,
               BMP180AppCallback, &g_sBMP180Inst);

   BMP180AppI2CWait(__FILE__, __LINE__);


   // Initialización the  SHT21
    SHT21Init(&g_sSHT21Inst, &g_sI2CInst, SHT21_I2C_ADDRESS,
              SHT21AppCallback, &g_sSHT21Inst);

    SHT21AppI2CWait(__FILE__, __LINE__);

    MAP_SysCtlDelay(MAP_SysCtlClockGet() / (50 * 3));

    // Initialización the TMP006
    TMP006Init(&g_sTMP006Inst, &g_sI2CInst, TMP006_I2C_ADDRESS,
               TMP006AppCallback, &g_sTMP006Inst);
    TMP006AppI2CWait (__FILE__, __LINE__);
    MAP_SysCtlDelay(MAP_SysCtlClockGet() / (100 * 3));

    // Initialización the ISL29023

    ISL29023Init(&g_sISL29023Inst, &g_sI2CInst, ISL29023_I2C_ADDRESS,
                     ISL29023AppCallback, &g_sISL29023Inst);

     // Wait for transaction to complete
     ISL29023AppI2CWait(__FILE__, __LINE__);
     MAP_SysCtlDelay(MAP_SysCtlClockGet() / (100 * 3));

     // Initialización the MPU9150 Driver.
     MPU9150Init(&g_sMPU9150Inst, &g_sI2CInst, MPU9150_I2C_ADDRESS,
                     MPU9150AppCallback, &g_sMPU9150Inst);

     //
     // Wait for transaction to complete
     //
     MPU9150AppI2CWait(__FILE__, __LINE__);

    //si ponemos pdTrue se recarga el timer automaticamente
          xTimer = xTimerCreate("TimerSW",configTICK_RATE_HZ*0.3, pdTRUE,NULL,vTimerCallback);
              if( xTimer == NULL )
                       {
                           /* The timer was not created. */
                  while(1);
                       }
              else{
                 /* Start the timer.  No block time is specified, and even if one was
                  it would be ignored because the RTOS scheduler has not yet been
                  started. */
                 if( xTimerStart( xTimer, 0 ) != pdPASS )
                 {
                     /* The timer could not be set into the Active state. */
                      while(1);
                 }
              }


	/**                                              Creacion de tareas 									**/

     USBSerialInit(32,32);   //Inicializo el  sistema USB


	//
	// Crea la tarea que gestiona los mensajes USB (definidos en USBMessageProcessingTask)
	//
	if(xTaskCreate(USBMessageProcessingTask, (portCHAR *)"usbser",512, NULL, tskIDLE_PRIORITY + 2, NULL) != pdTRUE)
	{
		while(1);
	}
    //Inicializacion de tareas
    if((xTaskCreate(TEMPTask, "Temp", TEMPTASKSTACKSIZE,NULL,tskIDLE_PRIORITY + TEMPTASKPRIO, NULL) != pdTRUE))
    {
        while(1);
       }
    if((xTaskCreate(HUMTask, "Humedad", HUMTASKSTACKSIZE,NULL,tskIDLE_PRIORITY + HUMTASKPRIO, NULL) != pdTRUE))
    {
        while(1);
              }
    if((xTaskCreate(PRETask, "Pre", PRETASKSTACKSIZE,NULL,tskIDLE_PRIORITY + PRETASKPRIO, NULL) != pdTRUE))
    {
        while(1);
    }
    if((xTaskCreate(LUZTask, "Luz", LUZTASKSTACKSIZE,NULL,tskIDLE_PRIORITY + LUZTASKPRIO, NULL) != pdTRUE))
    {
        while(1);
    }

   if((xTaskCreate(MOVTask, "Mov", MOVTASKSTACKSIZE,NULL,tskIDLE_PRIORITY + MOVTASKPRIO, NULL) != pdTRUE))
   {
       while(1);
    }
////////////////////////////////////////////SEMAFOROS/////////////////////////////////////////////////////////
    //semafora para el timer SW
   semaforoTemperatura=xSemaphoreCreateBinary();
       if ((semaforoTemperatura==NULL))
           {
               while (1);  //No hay memoria para los semaforo
           }
   semaforoHumedad=xSemaphoreCreateBinary();
       if ((semaforoHumedad==NULL))
          {
               while (1);  //No hay memoria para los semaforo
          }

   semaforoPresion=xSemaphoreCreateBinary();
       if ((semaforoPresion==NULL))
          {
               while (1);  //No hay memoria para los semaforo
          }
   semaforoLuz=xSemaphoreCreateBinary();
       if ((semaforoLuz==NULL))
          {
               while (1);  //No hay memoria para los semaforo
          }
   semaforoMovimiento=xSemaphoreCreateBinary();
       if ((semaforoMovimiento==NULL))
          {
               while (1);  //No hay memoria para los semaforo
          }

////////////////////////////////////////////COLAS/////////////////////////////////////////////////////////
   colaPresionMar = xQueueCreate(POSICIONES_COLA,sizeof(float));
       if( colaPresionMar == NULL )
          {
              while(1);
          }

   colaPresionMuestreo= xQueueCreate(POSICIONES_COLA,sizeof(int));
       if( colaPresionMuestreo == NULL )
          {
             while(1);
          }

   colaTempADC= xQueueCreate(POSICIONES_COLA,sizeof(int));
       if( colaTempADC == NULL )
          {
            while(1);
          }

   colaHumMR= xQueueCreate(POSICIONES_COLA,sizeof(int));
       if( colaHumMR== NULL )
          {
            while(1);
          }
   colaDeclinacion= xQueueCreate(POSICIONES_COLA,sizeof(int));
       if( colaDeclinacion== NULL )
          {
            while(1);
          }
   colaRangoLuz= xQueueCreate(POSICIONES_COLA,sizeof(int));
       if( colaHumMR== NULL )
          {
            while(1);
          }
   colaResolLuz= xQueueCreate(POSICIONES_COLA,sizeof(int));
       if( colaDeclinacion== NULL )
          {
            while(1);
          }
    colaMovimientoDLPF= xQueueCreate(POSICIONES_COLA,sizeof(int));
        if( colaMovimientoDLPF== NULL )
           {
             while(1);
           }
    colaMovimientoGIRO= xQueueCreate(POSICIONES_COLA,sizeof(int));
        if( colaMovimientoGIRO== NULL )
           {
             while(1);
           }

////////////////////////////////////////////GRUPO DE COLAS/////////////////////////////////////////////////////////
    grupo_colasPresion = xQueueCreateSet( POSICIONES_COLA + POSICIONES_COLA + 1);    // El de la cola, mas uno por cada semaforo binario
       if (grupo_colasPresion==NULL){
            while(1);
         }
    //colaPresionMar se utilizará para obtener la presion a nivel del mar
    if (xQueueAddToSet(colaPresionMar, grupo_colasPresion)!=pdPASS)
         {
             while(1);
         }
    //colaPresionMuestreo se utilizará para obtener el muestreo que se quiere
    if (xQueueAddToSet(colaPresionMuestreo, grupo_colasPresion)!=pdPASS)
         {
             while(1);
         }
    //semaforoPresion controla el tiempo que tiene que leerse el sensor otra vez
    if (xQueueAddToSet(  semaforoPresion, grupo_colasPresion)!=pdPASS)
         {
             while(1);
         }

    grupo_colasTemperatura = xQueueCreateSet( POSICIONES_COLA +POSICIONES_COLA + 1);    // El de la cola, mas uno por cada semaforo binario
    if (grupo_colasTemperatura==NULL){
             while(1);
         }
    //semaforoTemperatura
    if (xQueueAddToSet(semaforoTemperatura, grupo_colasTemperatura)!=pdPASS)
         {
             while(1);
         }

    //colaTempADC se utilizará para obtener el muestreo que se quiere
    if (xQueueAddToSet(colaTempADC, grupo_colasTemperatura)!=pdPASS)
         {
             while(1);
         }

     grupo_colasHumedad = xQueueCreateSet( POSICIONES_COLA + 1);    // El de la cola, mas uno por cada semaforo binario
     if (grupo_colasHumedad==NULL){
               while(1);
           }
     //semaforoHumedad
     if (xQueueAddToSet(semaforoHumedad, grupo_colasHumedad)!=pdPASS)
           {
              while(1);
           }
     //colaHumMR se utilizará para obtener
     if (xQueueAddToSet(colaHumMR, grupo_colasHumedad)!=pdPASS)
           {
             while(1);
           }


     grupo_colasLuz = xQueueCreateSet( POSICIONES_COLA +POSICIONES_COLA + 1);    // El de la cola, mas uno por cada semaforo binario
     if (grupo_colasLuz==NULL){
              while(1);
          }
     //semaforoLuz
     if (xQueueAddToSet(semaforoLuz, grupo_colasLuz)!=pdPASS)
          {
              while(1);
          }
     //colaRangoLuz se utilizará para obtener el muestreo que se quiere
     if (xQueueAddToSet(colaRangoLuz, grupo_colasLuz)!=pdPASS)
          {
              while(1);
          }
     //colaResolLuz se utilizará para obtener el muestreo que se quiere
     if (xQueueAddToSet(colaResolLuz, grupo_colasLuz)!=pdPASS)
          {
              while(1);
          }


      grupo_colasMovimiento = xQueueCreateSet( POSICIONES_COLA +POSICIONES_COLA +POSICIONES_COLA +1);    // El de la cola, mas uno por cada semaforo binario
      if (grupo_colasMovimiento==NULL){
             while(1);
           }
     //semaforoHumedad
     if (xQueueAddToSet(semaforoMovimiento, grupo_colasMovimiento)!=pdPASS)
           {
             while(1);
           }

     if (xQueueAddToSet(colaDeclinacion, grupo_colasMovimiento)!=pdPASS)
           {
             while(1);
           }
     if (xQueueAddToSet(colaMovimientoDLPF, grupo_colasMovimiento)!=pdPASS)
           {
             while(1);
           }
     if (xQueueAddToSet(colaMovimientoGIRO, grupo_colasMovimiento)!=pdPASS)
           {
             while(1);
           }

	//
	// Arranca el  scheduler.  Pasamos a ejecutar las tareas que se hayan activado.
	//
	vTaskStartScheduler();	//el RTOS habilita las interrupciones al entrar aqui, asi que no hace falta habilitarlas

	//De la funcion vTaskStartScheduler no se sale nunca... a partir de aqui pasan a ejecutarse las tareas.
	while(1)
	{
		//Si llego aqui es que algo raro ha pasado
	}
}
////////////////////////////////Rutinas de interrupcion//////////////////////////////////////////////////////////7

void
IntGPIOb(void)
{
    unsigned long ulStatus;
    signed portBASE_TYPE higherPriorityTaskWoken=pdFALSE;   //Hay que inicializarlo a False!!

    ulStatus = GPIOIntStatus(GPIO_PORTB_BASE, true);

    //
    // Clear all the pin interrupts that are set
    //
    GPIOIntClear(GPIO_PORTB_BASE, ulStatus);

    if(ulStatus & GPIO_PIN_2)
    {

        xSemaphoreGiveFromISR(semaforoMovimiento,&higherPriorityTaskWoken);
       //
        // MPU9150 Data is ready for retrieval and processing.
        //
        MPU9150DataRead(&g_sMPU9150Inst, MPU9150AppCallback, &g_sMPU9150Inst);
    }
}

//*****************************************************************************
//
// Called by the NVIC as a result of I2C3 Interrupt. I2C3 is the I2C connection
// to the sensor.
//
//*****************************************************************************
void
I2CIntHandler(void)
{
    //
    // Pass through to the I2CM interrupt handler provided by sensor library.
    // This is required to be at application level so that I2CMIntHandler can
    // receive the instance structure pointer as an argument.
    //
    I2CMIntHandler(&g_sI2CInst);
}

//*****************************************************************************
//
// Called by the NVIC as a result of GPIO port E interrupt event. For this
// application GPIO port E pin 0 is the interrupt line for the TMP006
//
//*****************************************************************************
void
IntGPIOe(void)
{
    uint32_t ui32Status;

    ui32Status = GPIOIntStatus(GPIO_PORTE_BASE, true);
    signed portBASE_TYPE higherPriorityTaskWoken=pdFALSE;   //Hay que inicializarlo a False!!

    //
    // Clear all the pin interrupts that are set
    //
    GPIOIntClear(GPIO_PORTE_BASE, ui32Status);

    if(ui32Status & GPIO_PIN_0)
    {
        xSemaphoreGiveFromISR(semaforoTemperatura,&higherPriorityTaskWoken);
        //
        // This interrupt indicates a conversion is complete and ready to be
        // fetched.  So we start the process of getting the data.
        //
        TMP006DataRead(&g_sTMP006Inst, TMP006AppCallback, &g_sTMP006Inst);
    }

}

