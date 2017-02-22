#include <QApplication>
#include <stdlib.h>
#include <stdio.h>
#include "Helicopter.h"
#include "BasicWindow.h"

int main(int argc, char *argv[]) {
    // initialize resources, if needed
    // Q_INIT_RESOURCE(resfile);

    QApplication app(argc, argv);
    
    BasicWindow window;
    window.setWindowTitle("Helicomms");
    window.show();    
    
    return app.exec();
}
