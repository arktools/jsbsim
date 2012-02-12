/*
 * MainWindow.hpp
 * Copyright (C) James Goppert 2010 <james.goppert@gmail.com>
 *
 * MainWindow.hpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * MainWindow.hpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef MainWindow_HPP
#define MainWindow_HPP

#include <QWidget>
#include "ui_MainWindow.h"
#include "arkosg/osgUtils.hpp"
#include <iomanip>
#include <QSettings>
//#include "config.h"
#include <stdexcept>
#include <QThread>
#include <QTime>

// include the plugins we need
#ifdef USE_FREETYPE
        USE_OSGPLUGIN(freetype)
#endif

#ifdef OSG_LIBRARY_STATIC
USE_DOTOSGWRAPPER_LIBRARY(osg)
USE_DOTOSGWRAPPER_LIBRARY(osgUtil)
USE_DOTOSGWRAPPER_LIBRARY(osgDB)
USE_DOTOSGWRAPPER_LIBRARY(osgFX)
USE_DOTOSGWRAPPER_LIBRARY(osgParticle)
USE_DOTOSGWRAPPER_LIBRARY(osgShadow)
USE_DOTOSGWRAPPER_LIBRARY(osgSim)
USE_DOTOSGWRAPPER_LIBRARY(osgTerrain)
USE_DOTOSGWRAPPER_LIBRARY(osgText)
USE_DOTOSGWRAPPER_LIBRARY(osgViewer)
USE_DOTOSGWRAPPER_LIBRARY(osgVolume)
USE_DOTOSGWRAPPER_LIBRARY(osgWidget)

USE_SERIALIZER_WRAPPER_LIBRARY(osg)
USE_SERIALIZER_WRAPPER_LIBRARY(osgUtil)
USE_SERIALIZER_WRAPPER_LIBRARY(osgDB)
USE_SERIALIZER_WRAPPER_LIBRARY(osgAnimation)
USE_SERIALIZER_WRAPPER_LIBRARY(osgFX)
USE_SERIALIZER_WRAPPER_LIBRARY(osgManipulator)
USE_SERIALIZER_WRAPPER_LIBRARY(osgParticle)
USE_SERIALIZER_WRAPPER_LIBRARY(osgShadow)
USE_SERIALIZER_WRAPPER_LIBRARY(osgSim)
USE_SERIALIZER_WRAPPER_LIBRARY(osgTerrain)
USE_SERIALIZER_WRAPPER_LIBRARY(osgText)
USE_SERIALIZER_WRAPPER_LIBRARY(osgVolume)
#endif

// include the platform specific GraphicsWindow implementation.
//USE_GRAPHICSWINDOW()



class MainWindow;

class SimulateThread : public QThread
{
	Q_OBJECT
public:
	SimulateThread(MainWindow * window);
	void run();
	MainWindow * window;
	QTimer timer;
	void quit()
	{
		timer.stop();
		QThread::quit();
	}
};

class MainWindow : public QMainWindow, private Ui::MainWindow
{
    Q_OBJECT
	friend class SimulateThread;
public:
    MainWindow();
    virtual ~MainWindow();

private slots:
	void showMsg(const QString & str);
	void simulate();	

private:
	QTime clock;
	SimulateThread simThread;
    void loadModel(const std::string & name);
    template <class varType>
    void prompt(const std::string & str, varType & var)
    {
        std::cout << str + " [" << std::setw(10) << var << "]\t: ";
        if (std::cin.peek() != '\n')
        {
            std::cin >> var;
            std::cin.ignore(1000, '\n');
        }
        else std::cin.get();
    }
    arkosg::Plane * plane;
	osg::Group * sceneRoot;
};

#endif

// vim:ts=4:sw=4
