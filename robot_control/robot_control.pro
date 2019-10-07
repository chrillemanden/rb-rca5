TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

INCLUDEPATH +=$$PWD/../../../fuzzylite/fuzzylite
LIBS += -L$$OUT_PWD/../../../fuzzylite/fuzzylite/release/bin -lfuzzylite-static

SOURCES += main.cpp \
    lidar.cpp \
    camera.cpp \
    robot.cpp \
    gazebo.cpp \
    fuzzy_controller.cpp

CONFIG += link_pkgconfig
PKGCONFIG += gazebo
PKGCONFIG += opencv

DISTFILES += \
    ObstacleAvoidance.fll

HEADERS += \
    camera.h \
    lidar.h \
    robot.h \
    gazebo.h \
    fuzzy_controller.h


