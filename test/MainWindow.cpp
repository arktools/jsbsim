/*
 * MainWindow.cpp
 * Copyright (C) James Goppert 2010 <james.goppert@gmail.com>
 *
 * MainWindow.cpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * MainWindow.cpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "MainWindow.hpp"
#include <QFileDialog>
#include <QMessageBox>
#include <osg/Vec3d>

#include <cstdlib>
#include <fstream>
#include "config.h"

MainWindow::MainWindow() : sceneRoot(new osg::Group), simThread(this)
{
    setupUi(this);
    viewer->setSceneData(sceneRoot);
    viewer->setCameraManipulator(new osgGA::TrackballManipulator);
    viewer->getCameraManipulator()->setHomePosition(osg::Vec3d(50,50,-50),osg::Vec3d(0,0,0),osg::Vec3d(0,0,-1),false);
    viewer->getCameraManipulator()->home(0);
    sceneRoot->addChild(new arkosg::Frame(20,"N","E","D"));
    sceneRoot->addChild(new arkosg::Terrain(std::string(DATADIR)+"/images/lz.rgb",osg::Vec3d(100,100,100)));

	// read initial settings
	QCoreApplication::setOrganizationName("arkTools");
    QCoreApplication::setOrganizationDomain("github.com/arkTools");
    QCoreApplication::setApplicationName("qtTest");

	// load plane
    try
    {
        plane = new arkosg::Plane(DATADIR+std::string("/models/plane.ac"));
		plane->setPosition(osg::Vec3(0,0,-10));
        plane->addChild(new arkosg::Frame(15,"X","Y","Z"));
        sceneRoot->addChild(plane);
    }
    catch(const std::exception & e)
    {
		showMsg(e.what());		
        return;
    }
	clock.start();
	simThread.start();
}

MainWindow::~MainWindow()
{
	simThread.quit();
    delete viewer;
}

void MainWindow::showMsg(const QString & str)
{
	QMessageBox msgBox;
	msgBox.setText(str);
	msgBox.exec();
};

void MainWindow::simulate() {
	viewer->mutex.lock();
	float t= clock.elapsed()/1000.0;
	float period = 10; // seconds
	float phi = 0.5*sin(2*M_PI/period*t);
	float theta = 0.5*sin(2*M_PI/period*t);
	float psi = 0.5*sin(2*M_PI/period*t);
	float throttle = 0.5*sin(2*M_PI/period*t);	
	float aileron = 0.5*sin(2*M_PI/period*t);
   	float elevator = 0.5*sin(2*M_PI/period*t);
   	float rudder = 0.5*sin(2*M_PI/period*t);
	float pN = 10*sin(2*M_PI/period*t);
	float pE = 10*cos(2*M_PI/period*t);
	float pD = -(10+10*sin(2*M_PI/period*t));
	plane->setPosition(osg::Vec3(pN,pE,pD));
	plane->setEuler(phi,theta,psi);
	plane->setU(throttle,aileron,elevator,rudder);
	viewer->mutex.unlock();
}

SimulateThread::SimulateThread(MainWindow * window) : window(window), timer(this)
{
	connect(&timer, SIGNAL(timeout()), window, SLOT(simulate()));
}

void SimulateThread::run()
{
	std::cout << "simulation started" << std::endl;
	timer.start(1000/60);
	exec();
}

// vim:ts=4:sw=4
