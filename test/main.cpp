#include <QApplication>
#include <QSplashScreen>
#include "MainWindow.hpp"

int main (int argc, char * argv[])
{
    QApplication app(argc,argv);
	app.processEvents();
    MainWindow w;
	w.show();
    return app.exec();
}
