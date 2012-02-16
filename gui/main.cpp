#include <QApplication>
#include <QSplashScreen>
#include "MainWindow.hpp"

#ifdef USE_X11
#include <X11/Xlib.h>
#endif

int main (int argc, char * argv[])
{
    #ifdef USE_X11
	XInitThreads();
    #endif
    QApplication app(argc,argv);
	app.processEvents();
    MainWindow w;
	w.show();
    return app.exec();
}
