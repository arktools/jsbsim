#include <QApplication>
#include <QSplashScreen>
#include "MainWindow.hpp"
#include <X11/Xlib.h>

int main (int argc, char * argv[])
{
	XInitThreads();
    QApplication app(argc,argv);
	app.processEvents();
    MainWindow w;
	w.show();
    return app.exec();
}
