TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    map_util.cpp

CONFIG += link_pkgconfig
PKGCONFIG += opencv

HEADERS += \
    map_util.h
