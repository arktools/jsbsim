/*
 * Viewer.cpp
 * Copyright (C) James Goppert 2010 <jgoppert@users.sourceforge.net>
 *
 * Viewer.cpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Viewer.cpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Viewer.hpp"
#include <boost/bind.hpp>
#include <osgGA/TrackballManipulator>

namespace mavsim
{

namespace visualization
{

Viewer::Viewer(int fps) :
    myThread(), myMutex(), myFps(fps)
{
    using namespace osgViewer;
    setThreadSafeReferenceCounting(true);
    setThreadSafeRefUnref(true);
    setCameraManipulator(new osgGA::TrackballManipulator);
}

Viewer::~Viewer()
{
    if (myThread) myThread->join();
}

int Viewer::run()
{
    realize();
    myThread.reset(new boost::thread(boost::bind(&Viewer::loop,this)));
    return 0;
}

void Viewer::loop()
{
    while (!done())
    {
        lock();
        frame();
        unlock();
        usleep(1e6/myFps);
    }
}

void Viewer::lock()
{
    myMutex.lock();
}

void Viewer::unlock()
{
    myMutex.unlock();
}

} // visualization

} // mavsim


// vim:ts=4:sw=4
