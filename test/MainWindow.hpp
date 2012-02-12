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
