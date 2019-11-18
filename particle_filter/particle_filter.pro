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
    fuzzy_controller.cpp \
    ../localization/localize.cpp \
    ../map_util/map_util.cpp \
    ../emulated_lidar_scanner/emulated_lidar_scanner.cpp

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
    fuzzy_controller.h \
    ../localization/localize.h \
    ../map_util/map_util.h \
    emulated_lidar_scanner.h


