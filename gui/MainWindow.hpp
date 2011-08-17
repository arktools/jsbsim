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
#include "MainWindow.ui.h"
#include "osgUtils.hpp"
#include "math/FGNelderMead.h"
#include <iomanip>
#include <QThread>
#include <QSettings>
//#include "config.h"
#include <stdexcept>
#include <math/FGStateSpace.h>
#include <initialization/FGTrimmer.h>
#include <QMutex>

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

class TrimThread : public QThread
{
	Q_OBJECT
public:
	TrimThread(MainWindow * window);
	void run();
	MainWindow * window;
};

class MainWindow : public QMainWindow, private Ui::MainWindow
{
    Q_OBJECT
	friend class SimulateThread;
	friend class TrimThread;
public:
    MainWindow();
    virtual ~MainWindow();
	QMutex fdmMutex;

signals:
	void showMsgBuffered(const QString & str);

private slots:
    void on_toolButton_enginePath_pressed();
    void on_toolButton_systemsPath_pressed();
    void on_toolButton_aircraftPath_pressed();
    void on_toolButton_aircraft_pressed();
    void on_toolButton_initScript_pressed();
    void on_pushButton_trim_pressed();
    void on_pushButton_stop_pressed();
    void on_pushButton_linearize_pressed();
    void on_pushButton_simulate_pressed();
	void showMsg(const QString & str);
	void simulate();
	void trim();

private:
	class SolverCallback : public JSBSim::FGNelderMead::Callback
	{
	public:
		SolverCallback(MainWindow * window) : window(window)
		{
		}
		void eval(const std::vector<double> & v)
		{
			double maxDeflection = 20.0*3.14/180.0; // TODO: this is rough
				// should depend on aircraft, but currently no access
			std::vector<double> data = window->trimmer->constrain(v);
			window->viewer->mutex.lock();
			window->plane->setEuler(data[0],data[1],v[5]);
				// phi, theta, beta to show orient, and side slip
			window->plane->setU(v[0],v[3]*maxDeflection,
					v[1]*maxDeflection,v[4]*maxDeflection);
			window->viewer->mutex.unlock();
		}
		MainWindow * window;
	};
	SimulateThread simThread;
	TrimThread trimThread;
	SolverCallback * callback;
	    osg::ref_ptr<osg::Group> sceneRoot;
    void loadModel(const std::string & name);

	void stopSolver();
	volatile bool stopRequested;
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
	QSettings * settings;
	void writeSettings();
	void readSettings();
    mavsim::visualization::Plane * plane;
	JSBSim::FGStateSpace * ss;
	JSBSim::FGTrimmer * trimmer;
	JSBSim::FGFDMExec * fdm;
};

#endif

// vim:ts=4:sw=4
