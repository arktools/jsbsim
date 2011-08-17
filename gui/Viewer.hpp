/*
 * Viewer.hpp
 * Copyright (C) James Goppert 2010 <jgoppert@users.sourceforge.net>
 *
 * Viewer.hpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Viewer.hpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef mavsim_Viewer_HPP
#define mavsim_Viewer_HPP

#include <osgViewer/Viewer>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/scoped_ptr.hpp>

namespace mavsim
{

namespace visualization
{

/**
 * The purpose of this class is to create a viewer for simulation purposes.
 */
class Viewer : public osgViewer::Viewer
{
public:
    /**
     * Constructor
     */
    Viewer(int fps = 20);
    virtual ~Viewer();
    /**
     * Override default run to be mutex lockable
     */
    int run();
    /**
     * Function to frame viewer
     */
    void loop();
    /**
     * Unlock the mutex and allow the viewer to frame
     */
    void unlock();
    /**
     * Lock the mutex and prevent viewer from framing
     */
    void lock();
private:
    boost::scoped_ptr<boost::thread> myThread; // viewer thread
    boost::mutex myMutex; // mutex lock to protect data for multi-threading
    int myFps; // Approxmiate frames per second, neglects framing time
};

} // visualization

} // mavsim

#endif

// vim:ts=4:sw=4
