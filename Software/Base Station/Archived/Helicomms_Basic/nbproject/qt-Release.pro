# This file is generated automatically. Do not edit.
# Use project properties -> Build -> Qt -> Expert -> Custom Definitions.
TEMPLATE = app
DESTDIR = dist/Release/GNU-Linux-x86
TARGET = Helicomms_Basic
VERSION = 1.0.0
CONFIG -= debug_and_release app_bundle lib_bundle
CONFIG += link_pkgconfig release 
PKGCONFIG += cvblob opencv
QT = core gui widgets network opengl printsupport
SOURCES += BasicWindow.cpp Helicopter.cpp Multicast.cpp OpencvCamera.cpp PoseEstimation.cpp TCP_client.cpp UDP_socket.cpp ekf_coder/Reb.cpp ekf_coder/correctStateAndCov.cpp ekf_coder/ekf_coder_emxutil.cpp ekf_coder/ekf_coder_initialize.cpp ekf_coder/ekf_coder_terminate.cpp ekf_coder/eulerAnglesFromQuaternion.cpp ekf_coder/eye.cpp ekf_coder/inv.cpp ekf_coder/mpower.cpp ekf_coder/projectStateAndCov.cpp ekf_coder/quaternionRotation.cpp ekf_coder/rtGetInf.cpp ekf_coder/rtGetNaN.cpp ekf_coder/rt_nonfinite.cpp ekf_coder/solvePoseEst.cpp main.cpp qcustomplot.cpp
HEADERS += BasicWindow.h Helicopter.h Multicast.h OpencvCamera.h PoseEstimation.h TCP_client.h UDP_socket.h ekf_coder/Reb.h ekf_coder/correctStateAndCov.h ekf_coder/ekf_coder_emxutil.h ekf_coder/ekf_coder_initialize.h ekf_coder/ekf_coder_terminate.h ekf_coder/ekf_coder_types.h ekf_coder/eulerAnglesFromQuaternion.h ekf_coder/eye.h ekf_coder/inv.h ekf_coder/mpower.h ekf_coder/projectStateAndCov.h ekf_coder/quaternionRotation.h ekf_coder/rtGetInf.h ekf_coder/rtGetNaN.h ekf_coder/rt_defines.h ekf_coder/rt_nonfinite.h ekf_coder/rtwtypes.h ekf_coder/solvePoseEst.h qcustomplot.h
FORMS += BasicWindow.ui
RESOURCES +=
TRANSLATIONS +=
OBJECTS_DIR = build/Release/GNU-Linux-x86
MOC_DIR = 
RCC_DIR = 
UI_DIR = 
QMAKE_CC = gcc
QMAKE_CXX = g++
DEFINES += 
INCLUDEPATH += ./ekf_coder /usr/include /usr/include/x86_64-linux-gnu/qt5 
LIBS += -lrt -lboost_system -larmadillo -lboost_signals-mt -lpthread   -Wl,-rpath,/opt/intel/composer_xe_2013.1.117/compiler/lib/ia32 
