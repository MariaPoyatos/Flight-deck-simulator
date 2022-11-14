#ifndef GUIPANEL_H
#define GUIPANEL_H

#include <QWidget>
#include <QSerialPort>
#include <QMessageBox>

namespace Ui {
class GUIPanel;
}


class GUIPanel : public QWidget
{
    Q_OBJECT
    
public:
    //GUIPanel(QWidget *parent = 0);
    explicit GUIPanel(QWidget *parent = 0);
    ~GUIPanel(); // Da problemas
    
private slots:
    void readRequest();
    void on_pingButton_clicked();
    void on_runButton_clicked();
    void on_statusButton_clicked();

    void on_botonON_clicked();

    void on_botonOFF_clicked();

    void on_botonAuto_clicked();

    void on_P0_valueChanged(double arg1);

    void on_botonF_clicked();

    void on_botonC_clicked();

    void on_OpcionesMuestreo_currentIndexChanged(int index);

    void on_OpcionesTempADC_currentIndexChanged(int index);

    void on_OpcionesHumedad_currentIndexChanged(int index);

    void on_CambiarDeclinacionMag_clicked();

    void on_OpcionesRangoLuz_currentIndexChanged(int index);

    void on_OpcionesResolLuz_currentIndexChanged(int index);

    void on_OpcionesFrec_currentIndexChanged(int index);

    void on_OpcionesGiroscopio_currentIndexChanged(int index);

private: // funciones privadas
    void pingDevice();
    void startSlave();
    void processError(const QString &s);
    void activateRunButton();
    void pingResponseReceived();

private:
    Ui::GUIPanel *ui;
    int transactionCount;
    bool fConnected;
    QSerialPort serial;
    QByteArray incommingDataBuffer;
    QString LastError;
    QMessageBox ventanaPopUp;
};

#endif // GUIPANEL_H
