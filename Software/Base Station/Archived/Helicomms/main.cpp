/*
 * File:   main.cpp
 * Author: yashren
 *
 * Created on 11 March 2013, 9:36 AM
 */

#include <QtGui/QApplication>
#include <qt4/QtGui/qwidget.h>
#include "Window.h"

int main(int argc, char *argv[]) {            
    QApplication app(argc, argv);

    // create and show your widgets here
    Window window;
    window.setWindowTitle("Helicomms Master");
    window.show();
        
    return app.exec();
}
