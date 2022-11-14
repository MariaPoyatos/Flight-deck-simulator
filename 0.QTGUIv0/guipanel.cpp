#include "guipanel.h"
#include "ui_guipanel.h"
#include <QSerialPort>      // Comunicacion por el puerto serie
#include <QSerialPortInfo>  // Comunicacion por el puerto serie

#include<stdint.h>      // Cabecera para usar tipos de enteros con tamaño
#include<stdbool.h>     // Cabecera para usar booleanos

extern "C" {
#include "serial2USBprotocol.h"    // Cabecera de funciones de gestión de tramas; se indica que está en C, ya que QTs
// se integra en C++, y el C puede dar problemas si no se indica.
}

#include "usb_messages_table.h"

GUIPanel::GUIPanel(QWidget *parent) :  // Constructor de la clase
    QWidget(parent),
    ui(new Ui::GUIPanel)               // Indica que guipanel.ui es el interfaz grafico de la clase
  , transactionCount(0)
{
    ui->setupUi(this);                // Conecta la clase con su interfaz gráfico.
    setWindowTitle(tr("Interfaz de Control")); // Título de la ventana

    // Conexion por el puerto serie-USB
    fConnected=false;                 // Todavía no hemos establecido la conexión USB
    ui->serialPortComboBox->clear(); // Vacía de componentes la comboBox
    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
        // La identificación nos permite que SOLO aparezcan los interfaces tipo USB serial de Texas Instrument
        if ((info.vendorIdentifier()==0x1CBE) && (info.productIdentifier()==0x0002))
        {
            ui->serialPortComboBox->addItem(info.portName());
        }
    ui->serialPortComboBox->setFocus();   // Componente del GUI seleccionado de inicio
    // Las funciones CONNECT son la base del funcionamiento de QT; conectan dos componentes
    // o elementos del sistema; uno que GENERA UNA SEÑAL; y otro que EJECUTA UNA FUNCION (SLOT) al recibir dicha señal.
    // En el ejemplo se conecta la señal readyRead(), que envía el componente que controla el puerto USB serie (serial),
    // con la propia clase PanelGUI, para que ejecute su funcion readRequest() en respuesta.
    // De esa forma, en cuanto el puerto serie esté preparado para leer, se lanza una petición de datos por el
    // puerto serie.El envío de readyRead por parte de "serial" es automatico, sin necesidad de instrucciones
    // del programador
    connect(&serial, SIGNAL(readyRead()), this, SLOT(readRequest()));

    ui->pingButton->setEnabled(false);    // Se deshabilita el botón de ping del interfaz gráfico, hasta que
    // se haya establecido conexión

    //Inicializa la ventana pop-up PING
    ventanaPopUp.setIcon(QMessageBox::Information);
    ventanaPopUp.setText(tr("Status: RESPUESTA A PING RECIBIDA"));
    ventanaPopUp.setStandardButtons(QMessageBox::Ok);
    ventanaPopUp.setWindowTitle(tr("Evento"));
    ventanaPopUp.setParent(this,Qt::Popup);
}

GUIPanel::~GUIPanel() // Destructor de la clase
{
    delete ui;   // Borra el interfaz gráfico asociado a la clase
}

void GUIPanel::readRequest()
{
    int StopCharPosition,StartCharPosition,tam;   // Solo uso notacin hungara en los elementos que se van a
    // intercambiar con el micro - para control de tamaño -
    uint8_t *pui8Frame; // Puntero a zona de memoria donde reside la trama recibida
    void *ptrtoparam;
    uint8_t ui8Message; // Para almacenar el mensaje de la trama entrante


    incommingDataBuffer.append(serial.readAll()); // Añade el contenido del puerto serie USB al array de bytes 'incommingDataBuffer'
    // así vamos acumulando en el array la información que va llegando

    // Busca la posición del primer byte de fin de trama (0xFD) en el array. Si no estuviera presente,
    // salimos de la funcion, en caso contrario, es que ha llegado al menos una trama.
    // Hay que tener en cuenta que pueden haber llegado varios paquetes juntos.
    StopCharPosition=incommingDataBuffer.indexOf((char)STOP_FRAME_CHAR,0);
    while (StopCharPosition>=0)
    {
        //Ahora buscamos el caracter de inicio correspondiente.
        StartCharPosition=incommingDataBuffer.lastIndexOf((char)START_FRAME_CHAR,0); //Este seria el primer caracter de inicio que va delante...

        if (StartCharPosition<0)
        {
            //En caso de que no lo encuentre, no debo de hacer nada, pero debo vaciar las primeras posiciones hasta STOP_FRAME_CHAR (inclusive)
            incommingDataBuffer.remove(0,StopCharPosition+1);
            LastError=QString("Status:Fallo trozo paquete recibido");
        } else
        {
            incommingDataBuffer.remove(0,StartCharPosition); //Si hay datos anteriores al caracter de inicio, son un trozo de trama incompleto. Los tiro.
            tam=StopCharPosition-StartCharPosition+1;//El tamanio de la trama es el numero de bytes desde inicio hasta fin, ambos inclusive.
            if (tam>=MINIMUM_FRAME_SIZE)
            {
                pui8Frame=(uint8_t*)incommingDataBuffer.data(); // Puntero de trama al inicio del array de bytes
                pui8Frame++; //Nos saltamos el caracter de inicio.
                tam-=2; //Descontamos los bytes de inicio y fin del tamanio del paquete

                // Paso 1: Destuffing y cálculo del CRC. Si todo va bien, obtengo la trama
                // con valores actualizados y sin bytes de CRC.
                tam=destuff_and_check_checksum((unsigned char *)pui8Frame,tam);
                if (tam>=0)
                {
                    //El paquete está bien, luego procedo a tratarlo.
                    ui8Message=decode_message_type(pui8Frame); // Obtencion del byte de Mensaje
                    tam=get_message_param_pointer(pui8Frame,tam,&ptrtoparam);
                    switch(ui8Message) // Segun el mensaje tengo que hacer cosas distintas
                    {
                    /** A PARTIR AQUI ES DONDE SE DEBEN AÑADIR NUEVAS RESPUESTAS ANTE LOS MENSAJES QUE SE ENVIEN DESDE LA TIVA **/
                    case MENSAJE_PING:  // Algunos mensajes no tiene parametros
                        // Crea una ventana popup con el texto indicado
                        pingResponseReceived();
                        break;

                    case MENSAJE_NO_IMPLEMENTADO:
                    {
                        // En otros mensajes hay que extraer los parametros de la trama y copiarlos
                        // a una estructura para poder procesar su informacion
                        PARAM_MENSAJE_NO_IMPLEMENTADO parametro;
                        if (check_and_extract_message_param(ptrtoparam, tam, sizeof(parametro),&parametro)>0)
                        {
                            // Muestra en una etiqueta (statuslabel) del GUI el mensaje
                            ui->statusLabel->setText(tr("  Mensaje rechazado,"));
                        }
                        else
                        {
                            // TRATAMIENTO DE ERRORES
                        }
                    }
                        break;
                    case MENSAJE_EJES:
                    {
                        PARAM_MENSAJE_EJES ejes;
                        if (check_and_extract_message_param(ptrtoparam, tam, sizeof(ejes),&ejes)>0)
                        {

                            ui->Pitch->setPitch(ejes.fPitch);
                            ui->Pitch->update();
                            ui->Roll->setTurnRate(ejes.fRoll);
                            ui->Roll->update();
                            ui->Yaw->setHeading(ejes.fYaw);
                            ui->Yaw->update();
                        }
                    }
                        break;

                    case MENSAJE_ALT :{
                        PARAM_MENSAJE_ALT altitud;
                            float ConversionMetroPies=3.28084;
                            float altitudPies;
                        if (check_and_extract_message_param(ptrtoparam, tam, sizeof(altitud),&altitud)>0)
                         {
                            //actualizamos la altitud

                            altitudPies= altitud.altitud*ConversionMetroPies;

                            ui->Altimetro->setAltitude(altitudPies);
                            ui->Altimetro->update();
                            ui->PresNumero->setValue(altitud.presion);

                        }
                   }
                    break;

                    case MENSAJE_TEMP :{
                        PARAM_MENSAJE_TEMP temp;
                        float tempf;
                        if (check_and_extract_message_param(ptrtoparam, tam, sizeof(temp),&temp)>0)
                         {

                            if(ui->Fahrenheit->isVisible()){
                                tempf= (temp.temperatura*9/5)+32;
                                ui->Temperatura->setValue(tempf);
                                ui->TempNumero->setValue(tempf);
                            }else if(ui->Celsius->isVisible()){
                                ui->Temperatura->setValue(temp.temperatura);
                                ui->TempNumero->setValue(temp.temperatura);
                            }
                        }

                    }
                        break;

                    case MENSAJE_HUM :{
                        PARAM_MENSAJE_HUM hum;
                        if (check_and_extract_message_param(ptrtoparam, tam, sizeof(hum),&hum)>0)
                         {
                             ui->Humedad->setValue(hum.humedad);
                             ui->HumNumero->setValue(hum.humedad);
                        }

                    }
                        break;
                    case MENSAJE_BRUJULA :{
                        PARAM_MENSAJE_BRUJULA brujula;
                        if (check_and_extract_message_param(ptrtoparam, tam, sizeof(brujula),&brujula)>0)
                         {
                            ui->brujula->setHeading(brujula.Brujula);
                            ui->brujula->update();

                        }

                    }
                        break;
                    case MENSAJE_NOCHE :{
                        PARAM_MENSAJE_NOCHE fondo;

                        if (check_and_extract_message_param(ptrtoparam, tam, sizeof(fondo),&fondo)>0)
                         {
                            QPixmap noche,dia;
                            QPixmap ajudia, ajunoche;

                            ajunoche.load(":/images/fondo negro.png");
                            ajudia.load(":/images/fondo blanco.png");
                            noche.load(":/images/modo noche1.png");
                            dia.load(":/images/modo dia1.png");
                            if(fondo.modo==1){//Modo noche
                                  ui->Fondo->setPixmap(noche);
                                  ui->ajufondo->setPixmap(ajunoche);
                                  ui->ajufondo_2->setPixmap(ajunoche);
                                  //cambiar color letras

                                  QPalette Escala(palette());
                                  Escala.setColor(QPalette::Text,Qt::white);
                                  Escala.setColor(QPalette::WindowText,Qt::white);

                                   ui->MODONOCHE->setStyleSheet("QLabel {color : white; }");
                                   ui->textoPresion->setStyleSheet("QLabel {color : white; }");
                                   ui->textoUnidades->setStyleSheet("QLabel {color : white; }");
                                   ui->textoUnidades_2->setStyleSheet("QLabel {color : white; }");
                                   ui->textoPascales_3->setStyleSheet("QLabel {color : white; }");
                                   ui->textoPascales_2->setStyleSheet("QLabel {color : white; }");
                                   ui->Sobremuestreo->setStyleSheet("QLabel {color : white; }");
                                   ui->TempADC->setStyleSheet("QLabel {color : white; }");
                                   ui->HumResol->setStyleSheet("QLabel {color : white; }");
                                   ui->ResolucionLuz->setStyleSheet("QLabel {color : white; }");
                                   ui->RangoLuz->setStyleSheet("QLabel {color : white; }");
                                   ui->FrecMov->setStyleSheet("QLabel {color : white; }");
                                   ui->FSYNC->setStyleSheet("QLabel {color : white; }");
                                   ui->serialPortLabel->setStyleSheet("QLabel {color : white; }");
                                   ui->textoPascales->setStyleSheet("QLabel {color : white; }");
                                   ui->Celsius->setStyleSheet("QLabel {color : white; }");
                                   ui->Celsius_2->setStyleSheet("QLabel {color : white; }");
                                   ui->Fahrenheit->setStyleSheet("QLabel {color : white; }");
                                   ui->Fahrenheit_2->setStyleSheet("QLabel {color : white; }");
                                   ui->Hume->setStyleSheet("QLabel {color : white; }");
                                   ui->Hume_2->setStyleSheet("QLabel {color : white; }");
                                   ui->Presion->setStyleSheet("QLabel {color : white; }");
                                   ui->groupBox->setStyleSheet("QGroupBox  {color: white;}");
                                   ui->AjusteVisual->setStyleSheet("QGroupBox  {color: white;}");
                                   ui->AjusteAltura->setStyleSheet("QGroupBox  {color: white;}");
                                   ui->AjusteTemperatura->setStyleSheet("QGroupBox  {color: white;}");
                                   ui->AjusteBrujula->setStyleSheet("QGroupBox  {color: white;}");
                                   ui->AjustePresion->setStyleSheet("QGroupBox  {color: white;}");
                                   ui->AjustesTemp->setStyleSheet("QGroupBox  {color: white;}");
                                   ui->AjusteHum->setStyleSheet("QGroupBox  {color: white;}");
                                   ui->AjusteLuz->setStyleSheet("QGroupBox  {color: white;}");
                                   ui->AjusteMov->setStyleSheet("QGroupBox  {color: white;}");
                                   ui->Temperatura->setPalette(Escala);

                                   ui->Humedad->setPalette(Escala);


                            }else{
                                  ui->Fondo->setPixmap(dia);
                                  ui->ajufondo->setPixmap(ajudia);
                                  ui->ajufondo_2->setPixmap(ajudia);
                                  //cambiar color letras

                                  QPalette Escala(palette());

                                  Escala.setColor(QPalette::Text,Qt::black);
                                  Escala.setColor(QPalette::WindowText,Qt::black);

                                  ui->MODONOCHE->setStyleSheet("QLabel {color : black; }");
                                  ui->textoPresion->setStyleSheet("QLabel {color : black; }");
                                  ui->textoUnidades->setStyleSheet("QLabel {color : black; }");
                                  ui->textoUnidades_2->setStyleSheet("QLabel {color : black; }");
                                  ui->textoPascales_3->setStyleSheet("QLabel {color : black; }");
                                  ui->textoPascales_2->setStyleSheet("QLabel {color : black; }");
                                  ui->Sobremuestreo->setStyleSheet("QLabel {color : black; }");
                                  ui->TempADC->setStyleSheet("QLabel {color : black; }");
                                  ui->HumResol->setStyleSheet("QLabel {color : black; }");
                                  ui->ResolucionLuz->setStyleSheet("QLabel {color : black; }");
                                  ui->RangoLuz->setStyleSheet("QLabel {color : black; }");
                                  ui->FrecMov->setStyleSheet("QLabel {color : black; }");
                                  ui->FSYNC->setStyleSheet("QLabel {color : black; }");
                                  ui->serialPortLabel->setStyleSheet("QLabel {color : black; }");
                                  ui->textoPascales->setStyleSheet("QLabel {color : black; }");
                                  ui->Celsius->setStyleSheet("QLabel {color : black; }");
                                  ui->Celsius_2->setStyleSheet("QLabel {color : black; }");
                                  ui->Fahrenheit->setStyleSheet("QLabel {color : black; }");
                                  ui->Fahrenheit_2->setStyleSheet("QLabel {color : black; }");
                                  ui->Hume->setStyleSheet("QLabel {color : black; }");
                                  ui->Hume_2->setStyleSheet("QLabel {color : black; }");
                                  ui->Presion->setStyleSheet("QLabel {color : black; }");
                                  ui->groupBox->setStyleSheet("QGroupBox  {color: black;}");
                                  ui->AjusteVisual->setStyleSheet("QGroupBox  {color: black;}");
                                  ui->AjusteAltura->setStyleSheet("QGroupBox  {color: black;}");
                                  ui->AjusteTemperatura->setStyleSheet("QGroupBox  {color: black;}");
                                  ui->AjusteBrujula->setStyleSheet("QGroupBox  {color: black;}");
                                  ui->AjustePresion->setStyleSheet("QGroupBox  {color: black;}");
                                  ui->AjustesTemp->setStyleSheet("QGroupBox  {color: black;}");
                                  ui->AjusteHum->setStyleSheet("QGroupBox  {color: black;}");
                                  ui->AjusteLuz->setStyleSheet("QGroupBox  {color: black;}");
                                  ui->AjusteMov->setStyleSheet("QGroupBox  {color: black;}");
                                  ui->Temperatura->setPalette(Escala);
                                  ui->Humedad->setPalette(Escala);

                            }
                        }

                    }
                        break;

                    default:
                        //Este error lo notifico mediante la señal statusChanged
                        LastError=QString("Status: Recibido paquete inesperado");
                        ui->statusLabel->setText(tr("  Recibido paquete inesperado,"));
                        break;
                    }
                }
                else
                {
                    //LastError=QString("Status: Error de stuffing o CRC");
                   // ui->statusLabel->setText(tr(" Error de stuffing o CRC"));
                 }
            }
            else
            {

                // B. La trama no está completa o no tiene el tamano adecuado... no lo procesa
                //Este error lo notifico mediante la señal statusChanged
                LastError=QString("Status: Error trozo paquete recibido");
                ui->statusLabel->setText(tr(" Fallo trozo paquete recibido"));
            }
            incommingDataBuffer.remove(0,StopCharPosition-StartCharPosition+1); //Elimino el trozo que ya he procesado
        }

        StopCharPosition=incommingDataBuffer.indexOf((char)STOP_FRAME_CHAR,0); //Compruebo si el se ha recibido alguna trama completa mas. (Para ver si tengo que salir del bucle o no
    } //Fin del while....
}

// Funciones auxiliares a la gestión comunicación USB

// Establecimiento de la comunicación USB serie a través del interfaz seleccionado en la comboBox, tras pulsar el
// botón RUN del interfaz gráfico. Se establece una comunicacion a 9600bps 8N1 y sin control de flujo en el objeto
// 'serial' que es el que gestiona la comunicación USB serie en el interfaz QT
void GUIPanel::startSlave()
{
    if (serial.portName() != ui->serialPortComboBox->currentText()) {
        serial.close();
        serial.setPortName(ui->serialPortComboBox->currentText());

        if (!serial.open(QIODevice::ReadWrite)) {
            processError(tr("No puedo abrir el puerto %1, error code %2")
                         .arg(serial.portName()).arg(serial.error()));
            return;
        }

        if (!serial.setBaudRate(9600)) {
            processError(tr("No puedo establecer tasa de 9600bps en el puerto %1, error code %2")
                         .arg(serial.portName()).arg(serial.error()));
            return;
        }

        if (!serial.setDataBits(QSerialPort::Data8)) {
            processError(tr("No puedo establecer 8bits de datos en el puerto %1, error code %2")
                         .arg(serial.portName()).arg(serial.error()));
            return;
        }

        if (!serial.setParity(QSerialPort::NoParity)) {
            processError(tr("NO puedo establecer parida en el puerto %1, error code %2")
                         .arg(serial.portName()).arg(serial.error()));
            return;
        }

        if (!serial.setStopBits(QSerialPort::OneStop)) {
            processError(tr("No puedo establecer 1bitStop en el puerto %1, error code %2")
                         .arg(serial.portName()).arg(serial.error()));
            return;
        }

        if (!serial.setFlowControl(QSerialPort::NoFlowControl)) {
            processError(tr("No puedo establecer el control de flujo en el puerto %1, error code %2")
                         .arg(serial.portName()).arg(serial.error()));
            return;
        }
    }

    ui->runButton->setEnabled(false);

    // Se indica que se ha realizado la conexión en la etiqueta 'statusLabel'
    ui->statusLabel->setText(tr("Estado: Ejecucion, conectado al puerto %1.")
                             .arg(ui->serialPortComboBox->currentText()));

    // Y se habilitan los controles
    ui->pingButton->setEnabled(true);

    // Variable indicadora de conexión a TRUE, para que se permita enviar mensajes en respuesta
    // a eventos del interfaz gráfico
    fConnected=true;


    // Configuración inicial del indicadores varios

    //Valores Inciales

    ui->Altimetro->setPressure(0);
    ui->Altimetro->update();
    ui->Altimetro->setAltitude(0);
    ui->Altimetro->update();

    ui->Humedad->setValue(0.0);

    ui->Roll->setTurnRate(0);
    ui->Roll->update();
    ui->Pitch->setPitch(0);
    ui->Pitch->update();
    ui->Yaw->setHeading(0);
    ui->Yaw->update();


    ui->P0->setValue(1013);
    ui->P0->update();

    ui->Fahrenheit->setVisible(false);
    ui->Celsius->setVisible(true);

    ui->Fahrenheit_2->setVisible(false);
    ui->Celsius_2->setVisible(true);
    //cambiar el nombre a las paginas
    ui->tabWidget->setTabText(0, "APP");
    ui->tabWidget->setTabText(1, "Ajustes");
    ui->tabWidget->setTabText(2, "Sensores");

    ui->brujula->setHeading(0);
    ui->brujula->update();
    ui->OpcionesFrec->setCurrentIndex(5);
    ui->OpcionesFrec->update();


}

// Funcion auxiliar de procesamiento de errores de comunicación (usada por startSlave)
void GUIPanel::processError(const QString &s)
{
    activateRunButton(); // Activa el botón RUN
    // Muestra en la etiqueta de estado la razón del error (notese la forma de pasar argumentos a la cadena de texto)
    ui->statusLabel->setText(tr("Status: Not running, %1.").arg(s));
}

// Funcion de habilitacion del boton de inicio/conexion
void GUIPanel::activateRunButton()
{
    ui->runButton->setEnabled(true);
}

// Funciones SLOT que se crean automaticamente desde QTDesigner al activar una señal de un Widget del interfaz gráfico
// Se suelen asociar a funciones auxiliares, en muchos caso, por comodidad.

// SLOT asociada a pulsación del botón RUN
void GUIPanel::on_runButton_clicked()
{
    startSlave();
}

// SLOT asociada a pulsación del botón PING
void GUIPanel::on_pingButton_clicked()
{
    pingDevice();
}

// SLOT asociada al borrado del mensaje de estado al pulsar el boton
void GUIPanel::on_statusButton_clicked()
{
    ui->statusLabel->setText(tr(""));
}

// Funciones de usuario asociadas a la respuesta a mensajes. La estructura va a ser muy parecida en casi todos los
// casos. Se va a crear una trama de un tamaño maximo (100), y se le van a introducir los elementos de
// num_secuencia, mensaje, y parametros.

// Envío de un mensaje PING

void GUIPanel::pingDevice()
{
    uint8_t paquete[MAX_FRAME_SIZE];
    int size;

    if (fConnected) // Para que no se intenten enviar datos si la conexion USB no esta activa
    {
        // El mensaje PING no necesita parametros; de ahí el NULL, y el 0 final.
        // No vamos a usar el mecanismo de numeracion de tramas; pasamos un 0 como n de trama
        size=create_frame(paquete, MENSAJE_PING, nullptr, 0, MAX_FRAME_SIZE);
        // Si la trama se creó correctamente, se escribe el paquete por el puerto serie USB
        if (size>0) serial.write((const char*)paquete,size);
    }
}

void GUIPanel::pingResponseReceived()

{
    // Ventana popUP para el caso de mensaje PING; no te deja definirla en un "caso"
    ventanaPopUp.setStyleSheet("background-color: lightgrey");
    ventanaPopUp.setModal(true);
    ventanaPopUp.show();
}

void GUIPanel::on_botonON_clicked()
{
    QPixmap noche;
    noche.load(":/images/modo noche1.png");
    ui->Fondo->setPixmap(noche);

    QPixmap ajunoche;
    ajunoche.load(":/images/fondo negro.png");
    ui->ajufondo->setPixmap(ajunoche);
    ui->ajufondo_2->setPixmap(ajunoche);


    QPalette Escala(palette());
    Escala.setColor(QPalette::Text,Qt::white);
    Escala.setColor(QPalette::WindowText,Qt::white);
    //cambiar color letras
    ui->MODONOCHE->setStyleSheet("QLabel {color : white; }");
    ui->textoPresion->setStyleSheet("QLabel {color : white; }");
    ui->textoUnidades->setStyleSheet("QLabel {color : white; }");
    ui->textoUnidades_2->setStyleSheet("QLabel {color : white; }");
    ui->textoPascales_3->setStyleSheet("QLabel {color : white; }");
    ui->textoPascales_2->setStyleSheet("QLabel {color : white; }");
    ui->Sobremuestreo->setStyleSheet("QLabel {color : white; }");
    ui->TempADC->setStyleSheet("QLabel {color : white; }");
    ui->HumResol->setStyleSheet("QLabel {color : white; }");
    ui->ResolucionLuz->setStyleSheet("QLabel {color : white; }");
    ui->RangoLuz->setStyleSheet("QLabel {color : white; }");
    ui->FrecMov->setStyleSheet("QLabel {color : white; }");
    ui->FSYNC->setStyleSheet("QLabel {color : white; }");
    ui->serialPortLabel->setStyleSheet("QLabel {color : white; }");
    ui->textoPascales->setStyleSheet("QLabel {color : white; }");
    ui->Celsius->setStyleSheet("QLabel {color : white; }");
    ui->Celsius_2->setStyleSheet("QLabel {color : white; }");
    ui->Fahrenheit->setStyleSheet("QLabel {color : white; }");
    ui->Fahrenheit_2->setStyleSheet("QLabel {color : white; }");
    ui->Hume->setStyleSheet("QLabel {color : white; }");
    ui->Hume_2->setStyleSheet("QLabel {color : white; }");
    ui->Presion->setStyleSheet("QLabel {color : white; }");
    ui->groupBox->setStyleSheet("QGroupBox  {color: white;}");
    ui->AjusteVisual->setStyleSheet("QGroupBox  {color: white;}");
    ui->AjusteAltura->setStyleSheet("QGroupBox  {color: white;}");
    ui->AjusteTemperatura->setStyleSheet("QGroupBox  {color: white;}");
    ui->AjusteBrujula->setStyleSheet("QGroupBox  {color: white;}");
    ui->AjustePresion->setStyleSheet("QGroupBox  {color: white;}");
    ui->AjustesTemp->setStyleSheet("QGroupBox  {color: white;}");
    ui->AjusteHum->setStyleSheet("QGroupBox  {color: white;}");
    ui->AjusteLuz->setStyleSheet("QGroupBox  {color: white;}");
    ui->AjusteMov->setStyleSheet("QGroupBox  {color: white;}");
    ui->Temperatura->setPalette(Escala);

    ui->Humedad->setPalette(Escala);

    PARAM_MENSAJE_AUTO parametro;
    uint8_t paquete[MAX_FRAME_SIZE];
    int size;
    parametro.automatico=false;
    size=create_frame(paquete, MENSAJE_AUTO, &parametro, sizeof(parametro), MAX_FRAME_SIZE);
    // Si la trama se creó correctamente, se escribe el paquete por el puerto serie USB
    if (size>0) serial.write((const char*)paquete,size);
}

void GUIPanel::on_botonOFF_clicked()
{
    QPixmap dia;
    dia.load(":/images/modo dia1.png");
    ui->Fondo->setPixmap(dia);

    QPixmap ajudia;
    ajudia.load(":/images/fondo blanco.png");
    ui->ajufondo->setPixmap(ajudia);
    ui->ajufondo_2->setPixmap(ajudia);
    //cambiar color letras
    QPalette Escala(palette());

    Escala.setColor(QPalette::Text,Qt::black);
    Escala.setColor(QPalette::WindowText,Qt::black);

    ui->MODONOCHE->setStyleSheet("QLabel {color : black; }");
    ui->textoPresion->setStyleSheet("QLabel {color : black; }");
    ui->textoUnidades->setStyleSheet("QLabel {color : black; }");
    ui->textoUnidades_2->setStyleSheet("QLabel {color : black; }");
    ui->textoPascales_3->setStyleSheet("QLabel {color : black; }");
    ui->textoPascales_2->setStyleSheet("QLabel {color : black; }");
    ui->Sobremuestreo->setStyleSheet("QLabel {color : black; }");
    ui->TempADC->setStyleSheet("QLabel {color : black; }");
    ui->HumResol->setStyleSheet("QLabel {color : black; }");
    ui->ResolucionLuz->setStyleSheet("QLabel {color : black; }");
    ui->RangoLuz->setStyleSheet("QLabel {color : black; }");
    ui->FrecMov->setStyleSheet("QLabel {color : black; }");
    ui->FSYNC->setStyleSheet("QLabel {color : black; }");
    ui->serialPortLabel->setStyleSheet("QLabel {color : black; }");
    ui->textoPascales->setStyleSheet("QLabel {color : black; }");
    ui->Celsius->setStyleSheet("QLabel {color : black; }");
    ui->Celsius_2->setStyleSheet("QLabel {color : black; }");
    ui->Fahrenheit->setStyleSheet("QLabel {color : black; }");
    ui->Fahrenheit_2->setStyleSheet("QLabel {color : black; }");
    ui->Hume->setStyleSheet("QLabel {color : black; }");
    ui->Hume_2->setStyleSheet("QLabel {color : black; }");
    ui->Presion->setStyleSheet("QLabel {color : black; }");
    ui->groupBox->setStyleSheet("QGroupBox  {color: black;}");
    ui->AjusteVisual->setStyleSheet("QGroupBox  {color: black;}");
    ui->AjusteAltura->setStyleSheet("QGroupBox  {color: black;}");
    ui->AjusteTemperatura->setStyleSheet("QGroupBox  {color: black;}");
    ui->AjusteBrujula->setStyleSheet("QGroupBox  {color: black;}");
    ui->AjustePresion->setStyleSheet("QGroupBox  {color: black;}");
    ui->AjustesTemp->setStyleSheet("QGroupBox  {color: black;}");
    ui->AjusteHum->setStyleSheet("QGroupBox  {color: black;}");
    ui->AjusteLuz->setStyleSheet("QGroupBox  {color: black;}");
    ui->AjusteMov->setStyleSheet("QGroupBox  {color: black;}");
    ui->Temperatura->setPalette(Escala);
    ui->Humedad->setPalette(Escala);

    PARAM_MENSAJE_AUTO parametro;
    uint8_t paquete[MAX_FRAME_SIZE];
    int size;
    parametro.automatico=false;
    size=create_frame(paquete, MENSAJE_AUTO, &parametro, sizeof(parametro), MAX_FRAME_SIZE);
    // Si la trama se creó correctamente, se escribe el paquete por el puerto serie USB
    if (size>0) serial.write((const char*)paquete,size);

}

void GUIPanel::on_botonAuto_clicked()
{
    PARAM_MENSAJE_AUTO parametro;
    uint8_t paquete[MAX_FRAME_SIZE];
    int size;

    parametro.automatico=true;
    size=create_frame(paquete, MENSAJE_AUTO, &parametro, sizeof(parametro), MAX_FRAME_SIZE);
    // Si la trama se creó correctamente, se escribe el paquete por el puerto serie USB
    if (size>0) serial.write((const char*)paquete,size);
}

void GUIPanel::on_P0_valueChanged(double arg1)
{
    PARAM_MENSAJE_PRESIONMAR parametro;
    uint8_t paquete[MAX_FRAME_SIZE];
    int size;

    parametro.presion=arg1*100;
    size=create_frame(paquete, MENSAJE_PRESIONMAR, &parametro, sizeof(parametro), MAX_FRAME_SIZE);
    // Si la trama se creó correctamente, se escribe el paquete por el puerto serie USB
    if (size>0) serial.write((const char*)paquete,size);
}

void GUIPanel::on_botonF_clicked()
{
    if((ui->Fahrenheit->isVisible()==false) and(ui->Fahrenheit_2->isVisible()==false)){
        ui->Fahrenheit->setVisible(true);
        ui->Celsius->setVisible(false);

        ui->Fahrenheit_2->setVisible(true);
        ui->Celsius_2->setVisible(false);
        //cambiar limites
        ui->Temperatura->setLowerBound(-5);
        ui->Temperatura->setUpperBound(115);
        ui->Temperatura->setScaleStepSize(20);

        ui->TempNumero->setMinimum(-5);
        ui->TempNumero->setMaximum(115);


        //pasar de celsius a fahrenheit
        ui->Temperatura->setValue((ui->Temperatura->value()*9/5)+32);
        ui->TempNumero->setValue((ui->TempNumero->value()*9/5)+32);
    }
}

void GUIPanel::on_botonC_clicked()
{
    if((ui->Celsius->isVisible()==false) and(ui->Celsius_2->isVisible()==false)){
        ui->Fahrenheit->setVisible(false);
        ui->Celsius->setVisible(true);

        ui->Fahrenheit_2->setVisible(false);
        ui->Celsius_2->setVisible(true);
        //cambiar limites
        ui->Temperatura->setLowerBound(-20);
        ui->Temperatura->setUpperBound(45);
        ui->Temperatura->setScaleStepSize(10);
        ui->TempNumero->setMinimum(-20);
        ui->TempNumero->setMaximum(45);
        //pasar de fahrenheit a celsius

        ui->TempNumero->setValue((ui->TempNumero->value()-32)*5/9);
        ui->Temperatura->setValue((ui->TempNumero->value()-32)*5/9);

   }
}

void GUIPanel::on_OpcionesMuestreo_currentIndexChanged(int index)
{
    PARAM_MENSAJE_MUESTREOPRESION parametro;
    uint8_t paquete[MAX_FRAME_SIZE];
    int size;

    parametro.IndexMuestreo=index;
    size=create_frame(paquete, MENSAJE_MUESTREOPRESION, &parametro, sizeof(parametro), MAX_FRAME_SIZE);
    // Si la trama se creó correctamente, se escribe el paquete por el puerto serie USB
    if (size>0) serial.write((const char*)paquete,size);
}


void GUIPanel::on_OpcionesTempADC_currentIndexChanged(int index)
{
    PARAM_MENSAJE_ADCTEMP parametro;
    uint8_t paquete[MAX_FRAME_SIZE];
    int size;

    parametro.IndexADC=index;
    size=create_frame(paquete, MENSAJE_ADCTEMP, &parametro, sizeof(parametro), MAX_FRAME_SIZE);
    // Si la trama se creó correctamente, se escribe el paquete por el puerto serie USB
    if (size>0) serial.write((const char*)paquete,size);
}

void GUIPanel::on_OpcionesHumedad_currentIndexChanged(int index)
{
    PARAM_MENSAJE_HUMRESOLUCION parametro;
    uint8_t paquete[MAX_FRAME_SIZE];
    int size;

    parametro.IndexMR=index;
    size=create_frame(paquete, MENSAJE_HUMRESOLUCION, &parametro, sizeof(parametro), MAX_FRAME_SIZE);
    // Si la trama se creó correctamente, se escribe el paquete por el puerto serie USB
    if (size>0) serial.write((const char*)paquete,size);
}

void GUIPanel::on_CambiarDeclinacionMag_clicked()
{
    PARAM_MENSAJE_DECLINACIONBRUJULA parametro;
    uint8_t paquete[MAX_FRAME_SIZE];
    int size;
    if(ui->OpcionesEOBruj->currentIndex()==0){//este
            parametro.Declinacion=-(ui->gradoBruj->value()+0.1*ui->MinBruj->value());
    }else{//oeste
            parametro.Declinacion=ui->gradoBruj->value()+0.1*ui->MinBruj->value();
    }
    size=create_frame(paquete, MENSAJE_DECLINACIONBRUJULA, &parametro, sizeof(parametro), MAX_FRAME_SIZE);
    // Si la trama se creó correctamente, se escribe el paquete por el puerto serie USB
    if (size>0) serial.write((const char*)paquete,size);
}

void GUIPanel::on_OpcionesRangoLuz_currentIndexChanged(int index)
{
    PARAM_MENSAJE_RANGOLUZ parametro;
    uint8_t paquete[MAX_FRAME_SIZE];
    int size;

    parametro.IndexRango=index;
    size=create_frame(paquete, MENSAJE_RANGOLUZ, &parametro, sizeof(parametro), MAX_FRAME_SIZE);
    // Si la trama se creó correctamente, se escribe el paquete por el puerto serie USB
    if (size>0) serial.write((const char*)paquete,size);
}

void GUIPanel::on_OpcionesResolLuz_currentIndexChanged(int index)
{
    PARAM_MENSAJE_RESOLUCIONLUZ parametro;
    uint8_t paquete[MAX_FRAME_SIZE];
    int size;

    parametro.IndexResolucion=index;
    size=create_frame(paquete, MENSAJE_RESOLUCIONLUZ, &parametro, sizeof(parametro), MAX_FRAME_SIZE);
    // Si la trama se creó correctamente, se escribe el paquete por el puerto serie USB
    if (size>0) serial.write((const char*)paquete,size);
}


void GUIPanel::on_OpcionesFrec_currentIndexChanged(int index)
{
    PARAM_MENSAJE_FRECMUESTMOV parametro;
    uint8_t paquete[MAX_FRAME_SIZE];
    int size;

    parametro.IndexFrec=index;
    size=create_frame(paquete, MENSAJE_FRECMUESTMOV, &parametro, sizeof(parametro), MAX_FRAME_SIZE);
    // Si la trama se creó correctamente, se escribe el paquete por el puerto serie USB
    if (size>0) serial.write((const char*)paquete,size);
}

void GUIPanel::on_OpcionesGiroscopio_currentIndexChanged(int index)
{
    PARAM_MENSAJE_GIROMOV parametro;
    uint8_t paquete[MAX_FRAME_SIZE];
    int size;

    parametro.IndexGIRO=index;
    size=create_frame(paquete, MENSAJE_GIROMOV, &parametro, sizeof(parametro), MAX_FRAME_SIZE);
    // Si la trama se creó correctamente, se escribe el paquete por el puerto serie USB
    if (size>0) serial.write((const char*)paquete,size);
}

