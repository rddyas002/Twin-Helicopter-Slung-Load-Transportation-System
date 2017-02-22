# This file is generated automatically. Do not edit.
# Use project properties -> Build -> Qt -> Expert -> Custom Definitions.
TEMPLATE = app
DESTDIR = dist/Debug/GNU-Linux-x86
TARGET = Helicomms
VERSION = 1.0.0
CONFIG -= debug_and_release app_bundle lib_bundle
CONFIG += link_pkgconfig debug 
PKGCONFIG += opencv cvblob
QT = core gui
SOURCES += /home/yashren/Documents/Netbeans_Master/Helicomms/ekf_coder/Reb.cpp /home/yashren/Documents/Netbeans_Master/Helicomms/ekf_coder/correctStateAndCov.cpp /home/yashren/Documents/Netbeans_Master/Helicomms/ekf_coder/ekf_coder_emxutil.cpp /home/yashren/Documents/Netbeans_Master/Helicomms/ekf_coder/ekf_coder_initialize.cpp /home/yashren/Documents/Netbeans_Master/Helicomms/ekf_coder/ekf_coder_rtwutil.cpp /home/yashren/Documents/Netbeans_Master/Helicomms/ekf_coder/ekf_coder_terminate.cpp /home/yashren/Documents/Netbeans_Master/Helicomms/ekf_coder/eulerAnglesFromQuaternion.cpp /home/yashren/Documents/Netbeans_Master/Helicomms/ekf_coder/eye.cpp /home/yashren/Documents/Netbeans_Master/Helicomms/ekf_coder/inv.cpp /home/yashren/Documents/Netbeans_Master/Helicomms/ekf_coder/mpower.cpp /home/yashren/Documents/Netbeans_Master/Helicomms/ekf_coder/projectStateAndCov.cpp /home/yashren/Documents/Netbeans_Master/Helicomms/ekf_coder/quaternionRotation.cpp /home/yashren/Documents/Netbeans_Master/Helicomms/ekf_coder/rtGetInf.cpp /home/yashren/Documents/Netbeans_Master/Helicomms/ekf_coder/rtGetNaN.cpp /home/yashren/Documents/Netbeans_Master/Helicomms/ekf_coder/rt_nonfinite.cpp Window.cpp main.cpp src/Blob.cpp src/Computer_channel.cpp src/OpencvCamera.cpp src/PoseEstimation.cpp src/TCP_client.cpp src/TCP_server.cpp src/UDP_socket.cpp src/getIP.cpp src/qcustomplot.cpp
HEADERS += /home/yashren/Documents/Netbeans_Master/Helicomms/ekf_coder/Reb.h /home/yashren/Documents/Netbeans_Master/Helicomms/ekf_coder/correctStateAndCov.h /home/yashren/Documents/Netbeans_Master/Helicomms/ekf_coder/ekf_coder_emxutil.h /home/yashren/Documents/Netbeans_Master/Helicomms/ekf_coder/ekf_coder_initialize.h /home/yashren/Documents/Netbeans_Master/Helicomms/ekf_coder/ekf_coder_rtwutil.h /home/yashren/Documents/Netbeans_Master/Helicomms/ekf_coder/ekf_coder_terminate.h /home/yashren/Documents/Netbeans_Master/Helicomms/ekf_coder/ekf_coder_types.h /home/yashren/Documents/Netbeans_Master/Helicomms/ekf_coder/eulerAnglesFromQuaternion.h /home/yashren/Documents/Netbeans_Master/Helicomms/ekf_coder/eye.h /home/yashren/Documents/Netbeans_Master/Helicomms/ekf_coder/inv.h /home/yashren/Documents/Netbeans_Master/Helicomms/ekf_coder/mpower.h /home/yashren/Documents/Netbeans_Master/Helicomms/ekf_coder/projectStateAndCov.h /home/yashren/Documents/Netbeans_Master/Helicomms/ekf_coder/quaternionRotation.h /home/yashren/Documents/Netbeans_Master/Helicomms/ekf_coder/rtGetInf.h /home/yashren/Documents/Netbeans_Master/Helicomms/ekf_coder/rtGetNaN.h /home/yashren/Documents/Netbeans_Master/Helicomms/ekf_coder/rt_defines.h /home/yashren/Documents/Netbeans_Master/Helicomms/ekf_coder/rt_nonfinite.h /home/yashren/Documents/Netbeans_Master/Helicomms/ekf_coder/rtwtypes.h Window.h include/Blob.h include/Computer_channel.h include/OpencvCamera.h include/PoseEstimation.h include/TCP_client.h include/TCP_server.h include/UDP_socket.h include/getIP.h include/qcustomplot.h
FORMS += Window.ui
RESOURCES +=
TRANSLATIONS +=
OBJECTS_DIR = build/Debug/GNU-Linux-x86
MOC_DIR = 
RCC_DIR = 
UI_DIR = 
QMAKE_CC = gcc
QMAKE_CXX = g++
DEFINES += 
INCLUDEPATH += "-I/usr/local/include/opencv -I/usr/local/include" /home/yashren/Documents/Netbeans_Master/Helicomms/include /home/yashren/Documents/Netbeans_Master/Helicomms/ekf_coder 
LIBS += -lpthread  -larmadillo -lrt -lboost_system -lboost_signals-mt  
