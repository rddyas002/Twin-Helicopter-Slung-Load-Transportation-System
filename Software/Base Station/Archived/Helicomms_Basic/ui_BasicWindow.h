/********************************************************************************
** Form generated from reading UI file 'BasicWindow.ui'
**
** Created by: Qt User Interface Compiler version 5.4.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_BASICWINDOW_H
#define UI_BASICWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>
#include "qcustomplot.h"

QT_BEGIN_NAMESPACE

class Ui_BasicWindow
{
public:
    QWidget *centralwidget;
    QFrame *frame;
    QLCDNumber *lcdNumber;
    QPushButton *btnConnectHelicopter1;
    QPushButton *btnConnectHelicopter2;
    QFrame *frame_2;
    QGroupBox *groupBox_2;
    QLineEdit *editIP1;
    QLineEdit *editIP2;
    QLineEdit *editIP3;
    QLineEdit *editIP4;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *label_4;
    QLabel *lblEKFRate;
    QLabel *lblCorrectionRate;
    QLineEdit *editHeli1;
    QLabel *label_11;
    QLabel *label_12;
    QLabel *label_13;
    QLabel *label_14;
    QLabel *lblTimeSinceLastCorrection;
    QLineEdit *editHeli2;
    QLabel *label_15;
    QFrame *frame_3;
    QGroupBox *groupBox_3;
    QCustomPlot *qplotGraph1;
    QCustomPlot *qplotGraph2;
    QCustomPlot *qplotGraph3;
    QCustomPlot *qplotGraph4;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *BasicWindow)
    {
        if (BasicWindow->objectName().isEmpty())
            BasicWindow->setObjectName(QStringLiteral("BasicWindow"));
        BasicWindow->resize(1280, 1024);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(BasicWindow->sizePolicy().hasHeightForWidth());
        BasicWindow->setSizePolicy(sizePolicy);
        centralwidget = new QWidget(BasicWindow);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        frame = new QFrame(centralwidget);
        frame->setObjectName(QStringLiteral("frame"));
        frame->setGeometry(QRect(10, 10, 291, 151));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        lcdNumber = new QLCDNumber(frame);
        lcdNumber->setObjectName(QStringLiteral("lcdNumber"));
        lcdNumber->setGeometry(QRect(10, 80, 271, 61));
        btnConnectHelicopter1 = new QPushButton(frame);
        btnConnectHelicopter1->setObjectName(QStringLiteral("btnConnectHelicopter1"));
        btnConnectHelicopter1->setGeometry(QRect(10, 10, 131, 61));
        btnConnectHelicopter1->setDefault(false);
        btnConnectHelicopter1->setFlat(false);
        btnConnectHelicopter2 = new QPushButton(frame);
        btnConnectHelicopter2->setObjectName(QStringLiteral("btnConnectHelicopter2"));
        btnConnectHelicopter2->setGeometry(QRect(150, 10, 131, 61));
        frame_2 = new QFrame(centralwidget);
        frame_2->setObjectName(QStringLiteral("frame_2"));
        frame_2->setGeometry(QRect(10, 170, 291, 321));
        frame_2->setFrameShape(QFrame::StyledPanel);
        frame_2->setFrameShadow(QFrame::Raised);
        groupBox_2 = new QGroupBox(frame_2);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        groupBox_2->setGeometry(QRect(10, 10, 271, 311));
        groupBox_2->setAutoFillBackground(false);
        groupBox_2->setStyleSheet(QStringLiteral(""));
        groupBox_2->setFlat(false);
        groupBox_2->setCheckable(false);
        editIP1 = new QLineEdit(groupBox_2);
        editIP1->setObjectName(QStringLiteral("editIP1"));
        editIP1->setGeometry(QRect(80, 30, 113, 27));
        editIP2 = new QLineEdit(groupBox_2);
        editIP2->setObjectName(QStringLiteral("editIP2"));
        editIP2->setGeometry(QRect(80, 60, 113, 27));
        editIP3 = new QLineEdit(groupBox_2);
        editIP3->setObjectName(QStringLiteral("editIP3"));
        editIP3->setGeometry(QRect(80, 90, 113, 27));
        editIP4 = new QLineEdit(groupBox_2);
        editIP4->setObjectName(QStringLiteral("editIP4"));
        editIP4->setGeometry(QRect(80, 120, 113, 27));
        label = new QLabel(groupBox_2);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(10, 35, 66, 17));
        label_2 = new QLabel(groupBox_2);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(10, 65, 66, 17));
        label_3 = new QLabel(groupBox_2);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(10, 95, 66, 17));
        label_4 = new QLabel(groupBox_2);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setGeometry(QRect(10, 125, 66, 17));
        lblEKFRate = new QLabel(groupBox_2);
        lblEKFRate->setObjectName(QStringLiteral("lblEKFRate"));
        lblEKFRate->setGeometry(QRect(200, 255, 66, 17));
        lblEKFRate->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        lblCorrectionRate = new QLabel(groupBox_2);
        lblCorrectionRate->setObjectName(QStringLiteral("lblCorrectionRate"));
        lblCorrectionRate->setGeometry(QRect(200, 230, 66, 17));
        lblCorrectionRate->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        editHeli1 = new QLineEdit(groupBox_2);
        editHeli1->setObjectName(QStringLiteral("editHeli1"));
        editHeli1->setGeometry(QRect(80, 150, 113, 27));
        label_11 = new QLabel(groupBox_2);
        label_11->setObjectName(QStringLiteral("label_11"));
        label_11->setGeometry(QRect(10, 155, 66, 17));
        label_12 = new QLabel(groupBox_2);
        label_12->setObjectName(QStringLiteral("label_12"));
        label_12->setGeometry(QRect(10, 230, 151, 17));
        label_13 = new QLabel(groupBox_2);
        label_13->setObjectName(QStringLiteral("label_13"));
        label_13->setGeometry(QRect(10, 255, 151, 17));
        label_14 = new QLabel(groupBox_2);
        label_14->setObjectName(QStringLiteral("label_14"));
        label_14->setGeometry(QRect(10, 280, 211, 17));
        lblTimeSinceLastCorrection = new QLabel(groupBox_2);
        lblTimeSinceLastCorrection->setObjectName(QStringLiteral("lblTimeSinceLastCorrection"));
        lblTimeSinceLastCorrection->setGeometry(QRect(200, 280, 66, 17));
        lblTimeSinceLastCorrection->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        editHeli2 = new QLineEdit(groupBox_2);
        editHeli2->setObjectName(QStringLiteral("editHeli2"));
        editHeli2->setGeometry(QRect(80, 180, 113, 27));
        label_15 = new QLabel(groupBox_2);
        label_15->setObjectName(QStringLiteral("label_15"));
        label_15->setGeometry(QRect(10, 185, 66, 17));
        frame_3 = new QFrame(centralwidget);
        frame_3->setObjectName(QStringLiteral("frame_3"));
        frame_3->setGeometry(QRect(310, 10, 961, 961));
        frame_3->setFrameShape(QFrame::StyledPanel);
        frame_3->setFrameShadow(QFrame::Raised);
        groupBox_3 = new QGroupBox(frame_3);
        groupBox_3->setObjectName(QStringLiteral("groupBox_3"));
        groupBox_3->setGeometry(QRect(10, 10, 941, 941));
        groupBox_3->setAutoFillBackground(false);
        groupBox_3->setFlat(false);
        qplotGraph1 = new QCustomPlot(groupBox_3);
        qplotGraph1->setObjectName(QStringLiteral("qplotGraph1"));
        qplotGraph1->setGeometry(QRect(20, 30, 455, 345));
        qplotGraph2 = new QCustomPlot(groupBox_3);
        qplotGraph2->setObjectName(QStringLiteral("qplotGraph2"));
        qplotGraph2->setGeometry(QRect(470, 30, 455, 345));
        qplotGraph3 = new QCustomPlot(groupBox_3);
        qplotGraph3->setObjectName(QStringLiteral("qplotGraph3"));
        qplotGraph3->setGeometry(QRect(20, 375, 455, 345));
        qplotGraph4 = new QCustomPlot(groupBox_3);
        qplotGraph4->setObjectName(QStringLiteral("qplotGraph4"));
        qplotGraph4->setGeometry(QRect(470, 375, 455, 345));
        BasicWindow->setCentralWidget(centralwidget);
        frame_3->raise();
        frame->raise();
        frame_2->raise();
        menubar = new QMenuBar(BasicWindow);
        menubar->setObjectName(QStringLiteral("menubar"));
        menubar->setGeometry(QRect(0, 0, 1280, 25));
        BasicWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(BasicWindow);
        statusbar->setObjectName(QStringLiteral("statusbar"));
        BasicWindow->setStatusBar(statusbar);

        retranslateUi(BasicWindow);

        QMetaObject::connectSlotsByName(BasicWindow);
    } // setupUi

    void retranslateUi(QMainWindow *BasicWindow)
    {
        BasicWindow->setWindowTitle(QApplication::translate("BasicWindow", "BasicWindow", 0));
        btnConnectHelicopter1->setText(QApplication::translate("BasicWindow", "Helicopter 1 \n"
"Connect", 0));
        btnConnectHelicopter2->setText(QApplication::translate("BasicWindow", "Helicopter 2\n"
"Connect", 0));
        groupBox_2->setTitle(QApplication::translate("BasicWindow", "Image Processing", 0));
        editIP1->setText(QApplication::translate("BasicWindow", "192.168.1.100", 0));
        editIP2->setText(QApplication::translate("BasicWindow", "192.168.1.101", 0));
        editIP3->setText(QApplication::translate("BasicWindow", "192.168.1.102", 0));
        editIP4->setText(QApplication::translate("BasicWindow", "127.0.0.1", 0));
        label->setText(QApplication::translate("BasicWindow", "PC1 IP:", 0));
        label_2->setText(QApplication::translate("BasicWindow", "PC2 IP:", 0));
        label_3->setText(QApplication::translate("BasicWindow", "PC3 IP:", 0));
        label_4->setText(QApplication::translate("BasicWindow", "PC4 IP:", 0));
        lblEKFRate->setText(QApplication::translate("BasicWindow", "---", 0));
        lblCorrectionRate->setText(QApplication::translate("BasicWindow", "---", 0));
        editHeli1->setText(QApplication::translate("BasicWindow", "192.168.1.104", 0));
        label_11->setText(QApplication::translate("BasicWindow", "HELI1 IP:", 0));
        label_12->setText(QApplication::translate("BasicWindow", "Measurement update:", 0));
        label_13->setText(QApplication::translate("BasicWindow", "State update:", 0));
        label_14->setText(QApplication::translate("BasicWindow", "Time since last correction (ms):", 0));
        lblTimeSinceLastCorrection->setText(QApplication::translate("BasicWindow", "---", 0));
        editHeli2->setText(QApplication::translate("BasicWindow", "192.168.1.110", 0));
        label_15->setText(QApplication::translate("BasicWindow", "HELI2 IP:", 0));
        groupBox_3->setTitle(QApplication::translate("BasicWindow", "Data Plot", 0));
    } // retranslateUi

};

namespace Ui {
    class BasicWindow: public Ui_BasicWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_BASICWINDOW_H
