TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    emulated_lidar_scanner.cpp

CONFIG += link_pkgconfig
PKGCONFIG += opencv
