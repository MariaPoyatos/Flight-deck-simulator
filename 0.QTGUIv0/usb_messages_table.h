/*
 * Listado de los tipos de mensajes empleados en la aplicación, así como definiciones de sus parámetros.
*/
#ifndef __USB_MESSAGES_TABLE_H
#define __USB_MESSAGES_TABLE_H

#include<stdint.h>

//Codigos de los mensajes. El estudiante deberá definir los códigos para los mensajes que vaya
// a crear y usar. Estos deberan ser compatibles con los usados en la parte Qt

typedef enum {
    MENSAJE_NO_IMPLEMENTADO,
    MENSAJE_PING,
    MENSAJE_EJES,
    MENSAJE_TEMP,
    MENSAJE_HUM,
    MENSAJE_ALT,
    MENSAJE_NOCHE,
    MENSAJE_AUTO,
    MENSAJE_PRESIONMAR,
    MENSAJE_MUESTREOPRESION,
    MENSAJE_ADCTEMP,
    MENSAJE_HUMRESOLUCION,
    MENSAJE_BRUJULA,
    MENSAJE_DECLINACIONBRUJULA,
    MENSAJE_RANGOLUZ,
    MENSAJE_RESOLUCIONLUZ,
    MENSAJE_FRECMUESTMOV,
    MENSAJE_GIROMOV,
    //etc, etc...
} messageTypes;

//Estructuras relacionadas con los parametros de los mensajes. El estuadiante debera crear las
// estructuras adecuadas a los mensajes usados, y asegurarse de su compatibilidad con el extremo Qt

#pragma pack(1)   //Con esto consigo que el alineamiento de las estructuras en memoria del PC (32 bits) no tenga relleno.
//Con lo de abajo consigo que el alineamiento de las estructuras en memoria del microcontrolador no tenga relleno
#define PACKED //__attribute__ ((packed))

typedef struct {
    uint8_t message;
}PACKED PARAM_MENSAJE_NO_IMPLEMENTADO;

#pragma pack()    //...Pero solo para los mensajes que voy a intercambiar, no para el resto

typedef struct {//parametros de las direcciones
        float fPitch;
        float fRoll;
        float fYaw;

} PACKED PARAM_MENSAJE_EJES;

typedef struct {
    float temperatura;
} PACKED  PARAM_MENSAJE_TEMP; //defino el parametro para temperatura


typedef struct {
    float humedad;
} PACKED  PARAM_MENSAJE_HUM; //defino el parametro para humedad

typedef struct {
    float altitud;
    float presion;
} PACKED  PARAM_MENSAJE_ALT; //defino el parametro para altitud


typedef struct {
    bool modo;
} PACKED  PARAM_MENSAJE_NOCHE; //defino el parametro para cambiar el fondo

typedef struct {
    bool automatico;
} PACKED  PARAM_MENSAJE_AUTO; //defino el parametro para cambiar el fondo
typedef struct {
    float presion;
} PACKED  PARAM_MENSAJE_PRESIONMAR; //defino el parametro para cambiar el valor de la presion a nivel del mar

typedef struct {
    int IndexMuestreo;
} PACKED  PARAM_MENSAJE_MUESTREOPRESION; //defino el parametro para cambiar el valor del muestreo de la presion

typedef struct {
    int IndexADC;
} PACKED  PARAM_MENSAJE_ADCTEMP; //defino el parametro para cambiar el valor del ADC de la temperatura
typedef struct {
    int IndexMR;
} PACKED  PARAM_MENSAJE_HUMRESOLUCION; //defino el parametro para cambiar el valor d

typedef struct {
    float Brujula;
} PACKED  PARAM_MENSAJE_BRUJULA; //defino el parametro para cambiar el valor de la brujula

typedef struct {
    float Declinacion;
} PACKED  PARAM_MENSAJE_DECLINACIONBRUJULA; //defino el parametro para cambiar el valor de declinacion magnetica

typedef struct {
    int IndexRango;
} PACKED  PARAM_MENSAJE_RANGOLUZ; //defino el parametro para cambiar el valor del rango de la luz

typedef struct {
    int IndexResolucion;
} PACKED  PARAM_MENSAJE_RESOLUCIONLUZ; //defino el parametro para cambiar el valor de la resolucion de la luz
typedef struct {
    int IndexFrec;
} PACKED  PARAM_MENSAJE_FRECMUESTMOV; //defino el parametro para cambiar el valor del Digital Low Pass Filter del movimiento

typedef struct {
    int IndexGIRO;
} PACKED  PARAM_MENSAJE_GIROMOV; //defino el parametro para cambiar el valor del Rango de escala completa del movimiento
#endif
