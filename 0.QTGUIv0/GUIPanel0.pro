#-------------------------------------------------
#
# Project created by QtCreator 2013-04-05T19:35:15
#
# Compatible desde Qt5.3 en adelante
#-------------------------------------------------

QT       += core gui serialport widgets svg
CONFIG   += qwt analogwidgets qmqtt ColorWidgets embeddeduma

TARGET = GUIPanel
TEMPLATE = app


SOURCES += main.cpp\
        guipanel.cpp \
    crc.c \
    serial2USBprotocol.c

HEADERS  += guipanel.h \
    crc.h \
    serial2USBprotocol.h \
    usb_messages_table.h

FORMS    += guipanel.ui

RESOURCES += \
    images.qrc
