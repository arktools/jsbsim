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

#ifdef WITH_ARKOSG
#include <osg/Vec3d>
#endif

#include <cstdlib>
#include <fstream>
#include <models/FGAircraft.h>
#include <models/FGMassBalance.h>
#include <models/propulsion/FGTank.h>
#include <models/propulsion/FGEngine.h>
#include <models/propulsion/FGTurbine.h>
#include <models/propulsion/FGTurboProp.h>
#include <FGJSBBase.h>
#include "input_output/FGPropertyManager.h"
#include <stdexcept>

MainWindow::MainWindow() : 
#ifdef WITH_ARKOSG
    sceneRoot(new osg::Group),
#endif
	callback(new SolverCallback(this)), trimThread(this), simThread(this),
#ifdef WITH_ARKOSG
    plane(NULL),
#endif
    solver(NULL),
	fdm(NULL),
    ss(NULL),
    constraints(new JSBSim::FGTrimmer::Constraints),
	trimmer(NULL),
    socket(NULL),
    mutex(QMutex::NonRecursive)
{
    setupUi(this);
#ifdef WITH_ARKOSG
    viewer->setSceneData(sceneRoot);
    viewer->setCameraManipulator(new osgGA::TrackballManipulator);
    viewer->getCameraManipulator()->setHomePosition(osg::Vec3d(30,30,-30),osg::Vec3d(0,0,0),osg::Vec3d(0,0,-1),false);
    viewer->getCameraManipulator()->home(0);
    sceneRoot->addChild(new arkosg::Frame(20,"N","E","D"));
#endif

	// read initial settings
	QCoreApplication::setOrganizationName("jsbsim");
    QCoreApplication::setOrganizationDomain("jsbsim.sf.net");
    QCoreApplication::setApplicationName("trim");

	settings = new QSettings;
	readSettings();
	writeSettings();

#ifdef WITH_ARKOSG
	// load plane model
    try
    {
        plane = new arkosg::Plane(ARKOSG_DATA_DIR+std::string("/arkosg/models/plane.ac"));
        plane->addChild(new arkosg::Frame(15,"X","Y","Z"));
        sceneRoot->addChild(plane);
    }
    catch(const std::exception & e)
    {
		showMsg(e.what());		
    }
#endif
}

MainWindow::~MainWindow()
{
	writeSettings();
#ifdef WITH_ARKOSG
    delete viewer;
#endif
    flightGearDisconnect();
}

void MainWindow::writeSettings()
{
	settings->beginGroup("aircraft");
	settings->setValue("modelSimRate",lineEdit_modelSimRate->text());
	settings->setValue("enginePath",lineEdit_enginePath->text());
	settings->setValue("systemsPath",lineEdit_systemsPath->text());
	settings->setValue("aircraftPath",lineEdit_aircraftPath->text());
	settings->setValue("aircraft",lineEdit_aircraft->text());
	settings->setValue("initScript",lineEdit_initScript->text());
	settings->endGroup();

	settings->beginGroup("trim");
	settings->setValue("velocity",lineEdit_velocity->text());
	settings->setValue("rollRate",lineEdit_rollRate->text());
	settings->setValue("pitchRate",lineEdit_pitchRate->text());
	settings->setValue("yawRate",lineEdit_yawRate->text());
	settings->setValue("altitude",lineEdit_altitude->text());
	settings->setValue("gamma",lineEdit_gamma->text());
	settings->setValue("payload",lineEdit_payload->text());
	settings->setValue("fuel",lineEdit_fuel->text());
	settings->setValue("flapPos",lineEdit_flapPos->text());
	settings->setValue("variablePropPitch",checkBox_variablePropPitch->checkState());
	settings->setValue("stabAxisRoll",checkBox_stabAxisRoll->checkState());
	settings->endGroup();

    settings->beginGroup("input");
    settings->setValue("joystick",lineEdit_joystick->text());
    settings->setValue("joystickEnabled",checkBox_joystickEnabled->checkState());
	settings->endGroup();

    settings->beginGroup("solver");
    settings->setValue("rtol",lineEdit_rtol->text());
    settings->setValue("abstol",lineEdit_abstol->text());
    settings->setValue("speed",lineEdit_speed->text());
    settings->setValue("iterMax",lineEdit_iterMax->text());
    settings->setValue("debugLevel",comboBox_debugLevel->currentIndex());
    settings->setValue("random",lineEdit_random->text());
    settings->setValue("pause",checkBox_pause->checkState());
    settings->setValue("showSimplex",checkBox_showSimplex->checkState());
    settings->setValue("showConvergence",checkBox_showConvergence->checkState());
    settings->endGroup();

    settings->beginGroup("guess");

    settings->setValue("throttleGuess",lineEdit_throttleGuess->text());
    settings->setValue("throttleMin",lineEdit_throttleMin->text());
    settings->setValue("throttleMax",lineEdit_throttleMax->text());
    settings->setValue("throttleInitialStepSize",lineEdit_throttleInitialStepSize->text());

    settings->setValue("aileronGuess",lineEdit_aileronGuess->text());
    settings->setValue("aileronMin",lineEdit_aileronMin->text());
    settings->setValue("aileronMax",lineEdit_aileronMax->text());
    settings->setValue("aileronInitialStepSize",lineEdit_aileronInitialStepSize->text());

    settings->setValue("rudderGuess",lineEdit_rudderGuess->text());
    settings->setValue("rudderMin",lineEdit_rudderMin->text());
    settings->setValue("rudderMax",lineEdit_rudderMax->text());
    settings->setValue("rudderInitialStepSize",lineEdit_rudderInitialStepSize->text());

    settings->setValue("elevatorGuess",lineEdit_elevatorGuess->text());
    settings->setValue("elevatorMin",lineEdit_elevatorMin->text());
    settings->setValue("elevatorMax",lineEdit_elevatorMax->text());
    settings->setValue("elevatorInitialStepSize",lineEdit_elevatorInitialStepSize->text());

    settings->setValue("alphaGuess",lineEdit_alphaGuess->text());
    settings->setValue("alphaMin",lineEdit_alphaMin->text());
    settings->setValue("alphaMax",lineEdit_alphaMax->text());
    settings->setValue("alphaInitialStepSize",lineEdit_alphaInitialStepSize->text());

    settings->setValue("betaGuess",lineEdit_betaGuess->text());
    settings->setValue("betaMin",lineEdit_betaMin->text());
    settings->setValue("betaMax",lineEdit_betaMax->text());
    settings->setValue("betaInitialStepSize",lineEdit_betaInitialStepSize->text());
	settings->endGroup();

    settings->beginGroup("output");
    settings->setValue("outputPath",lineEdit_outputPath->text());
    settings->setValue("caseName",lineEdit_caseName->text());
    settings->setValue("flightGearPort",lineEdit_flightGearPort->text());
    settings->setValue("flightGearHost",lineEdit_flightGearHost->text());
    settings->setValue("flightGearRate",lineEdit_flightGearRate->text());
	settings->endGroup();
}

void MainWindow::readSettings()
{
	QString root(INSTALL_DATA_DIR);

	settings->beginGroup("aircraft");
	lineEdit_modelSimRate->setText(settings->value("modelSimRate",120).toString());
	lineEdit_enginePath->setText(settings->value("enginePath",root+"/engine").toString());
	lineEdit_systemsPath->setText(settings->value("systemsPath",root+"/aircraft/f16/Systems").toString());
	lineEdit_aircraftPath->setText(settings->value("aircraftPath",root+"/aircraft/f16").toString());
	lineEdit_aircraft->setText(settings->value("aircraft","f16").toString());
	lineEdit_initScript->setText(settings->value("initScript","/aircraft/f16/reset00.xml").toString());
	settings->endGroup();

	settings->beginGroup("trim");
    lineEdit_velocity->setText(settings->value("velocity",600).toString());
    lineEdit_rollRate->setText(settings->value("rollRate",0).toString());
    lineEdit_pitchRate->setText(settings->value("pitchRate",0).toString());
    lineEdit_yawRate->setText(settings->value("yawRate",0).toString());
    lineEdit_altitude->setText(settings->value("altitude",100).toString());
    lineEdit_gamma->setText(settings->value("gamma",0).toString());
    lineEdit_payload->setText(settings->value("payload",0).toString());
    lineEdit_fuel->setText(settings->value("fuel",100).toString());
    lineEdit_flapPos->setText(settings->value("flapPos",0).toString());
    checkBox_variablePropPitch->setCheckState((Qt::CheckState)settings->value("variablePropPitch",Qt::Unchecked).toInt());
    checkBox_stabAxisRoll->setCheckState((Qt::CheckState)settings->value("stabAxisRoll",Qt::Checked).toInt());
    settings->endGroup();

    settings->beginGroup("solver");
    lineEdit_rtol->setText(settings->value("rtol",1e-4).toString());
    lineEdit_abstol->setText(settings->value("abstol",1e-2).toString());
    lineEdit_speed->setText(settings->value("speed",2).toString());
    lineEdit_iterMax->setText(settings->value("iterMax",2000).toString());
    comboBox_debugLevel->setCurrentIndex(settings->value("debugLevel",0).toInt());
    lineEdit_random->setText(settings->value("random",0).toString());
    checkBox_pause->setCheckState((Qt::CheckState)settings->value("pause",Qt::Unchecked).toInt());
    checkBox_showSimplex->setCheckState((Qt::CheckState)settings->value("showSimplex",Qt::Unchecked).toInt());
    checkBox_showConvergence->setCheckState((Qt::CheckState)settings->value("showConvergence",Qt::Checked).toInt());
    settings->endGroup();

    settings->beginGroup("input");
    lineEdit_joystick->setText(settings->value("joystick","/dev/input/js0").toString());
    checkBox_joystickEnabled->setCheckState((Qt::CheckState)settings->value("joystickEnabled",Qt::Unchecked).toInt());
	settings->endGroup();

    settings->beginGroup("guess");

    lineEdit_throttleGuess->setText(settings->value("throttleGuess",50).toString());
    lineEdit_throttleMin->setText(settings->value("throttleMin",0).toString());
    lineEdit_throttleMax->setText(settings->value("throttleMax",100).toString());
    lineEdit_throttleInitialStepSize->setText(settings->value("throttleInitialStepSize",5).toString());

    lineEdit_aileronGuess->setText(settings->value("aileronGuess",0).toString());
    lineEdit_aileronMin->setText(settings->value("aileronMin",-100).toString());
    lineEdit_aileronMax->setText(settings->value("aileronMax",100).toString());
    lineEdit_aileronInitialStepSize->setText(settings->value("aileronInitialStepSize",5).toString());

    lineEdit_rudderGuess->setText(settings->value("rudderGuess",0).toString());
    lineEdit_rudderMin->setText(settings->value("rudderMin",-100).toString());
    lineEdit_rudderMax->setText(settings->value("rudderMax",100).toString());
    lineEdit_rudderInitialStepSize->setText(settings->value("rudderInitialStepSize",5).toString());

    lineEdit_elevatorGuess->setText(settings->value("elevatorGuess",0).toString());
    lineEdit_elevatorMin->setText(settings->value("elevatorMin",-100).toString());
    lineEdit_elevatorMax->setText(settings->value("elevatorMax",100).toString());
    lineEdit_elevatorInitialStepSize->setText(settings->value("elevatorInitialStepSize",5).toString());

    lineEdit_alphaGuess->setText(settings->value("alphaGuess",0).toString());
    lineEdit_alphaMin->setText(settings->value("alphaMin",-10).toString());
    lineEdit_alphaMax->setText(settings->value("alphaMax",20).toString());
    lineEdit_alphaInitialStepSize->setText(settings->value("alphaInitialStepSize",5).toString());

    lineEdit_betaGuess->setText(settings->value("betaGuess",0).toString());
    lineEdit_betaMin->setText(settings->value("betaMin",-10).toString());
    lineEdit_betaMax->setText(settings->value("betaMax",10).toString());
    lineEdit_betaInitialStepSize->setText(settings->value("betaInitialStepSize",5).toString());

    settings->endGroup();

    settings->beginGroup("output");
    lineEdit_outputPath->setText(settings->value("outputPath",".").toString());
    lineEdit_caseName->setText(settings->value("caseName","1").toString());
    lineEdit_flightGearPort->setText(settings->value("flightGearPort","6001").toString());
    lineEdit_flightGearHost->setText(settings->value("flightGearHost","localhost").toString());
    lineEdit_flightGearRate->setText(settings->value("flightGearRate","120").toString());
	settings->endGroup();
}

void MainWindow::on_toolButton_enginePath_pressed()
{
    lineEdit_enginePath->setText(QFileDialog::getExistingDirectory(
                                     this, tr("Select Engine Path"),
                                     lineEdit_enginePath->text(),
                                     QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks));
}

void MainWindow::on_toolButton_systemsPath_pressed()
{
    lineEdit_systemsPath->setText(QFileDialog::getExistingDirectory(
                                      this, tr("Select Systems Path"),
                                      lineEdit_systemsPath->text(),
                                      QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks));
}

void MainWindow::on_toolButton_aircraftPath_pressed()
{
    lineEdit_aircraftPath->setText(QFileDialog::getExistingDirectory(
                                       this, tr("Select Aircraft Path"),
                                       lineEdit_aircraftPath->text(),
                                       QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks));
}

void MainWindow::on_toolButton_aircraft_pressed()
{
    QString path(QFileDialog::getExistingDirectory(
                     this, tr("Select Aircraft Directory"),
                     lineEdit_aircraftPath->text(),
                     QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks));
    if (!path.isNull())
    {
        QFileInfo pathInfo(path);
        lineEdit_aircraft->setText(pathInfo.fileName());
    }
    
}

void MainWindow::on_toolButton_initScript_pressed()
{
    lineEdit_initScript->setText(QFileDialog::getOpenFileName(this,
                                 tr("Select Initialization Script"),lineEdit_initScript->text(),
                                 tr("JSBSim Scripts (*.xml)")));
}

void MainWindow::on_toolButton_outputPath_pressed()
{
    lineEdit_outputPath->setText(QFileDialog::getExistingDirectory(
                                       this, tr("Select Output Path"),
                                       lineEdit_outputPath->text(),
                                       QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks));
}

void MainWindow::on_pushButton_trim_pressed()
{
    label_status->setText("trimming");
    trimThread.start();
}

void MainWindow::on_pushButton_linearize_pressed()
{
    label_status->setText("linearizing");
    linearize();
    label_status->setText("linearized");
}

void MainWindow::on_pushButton_save_pressed()
{
    save();
}

void MainWindow::on_pushButton_setGuess_pressed()
{
    if (isLocked(mutex)) {
        std::cout << "please press stop first" << std::endl;
        stop();
        return;
    }
    QMutexLocker locker(&mutex);
    //std::cout << "linearize locked mutex" << std::endl;

	using namespace JSBSim;

    if (!fdm) {
        stop();
        return;
    }
    lineEdit_throttleGuess->setText(QString::number(fdm->GetFCS()->GetThrottleCmd()[0]*100,'g',6));
    lineEdit_aileronGuess->setText(QString::number(fdm->GetFCS()->GetDaCmd()*100,'g',6));
    lineEdit_rudderGuess->setText(QString::number(fdm->GetFCS()->GetDrCmd()*100,'g',6));
    lineEdit_elevatorGuess->setText(QString::number(fdm->GetFCS()->GetDeCmd()*100,'g',6));
    lineEdit_alphaGuess->setText(QString::number(fdm->GetAuxiliary()->Getalpha(FGJSBBase::inDegrees),'g',6));
    lineEdit_betaGuess->setText(QString::number(fdm->GetAuxiliary()->Getbeta(FGJSBBase::inDegrees),'g',6));
    writeSettings();
}

void MainWindow::on_pushButton_flightGearConnect_pressed() {
    flightGearConnect();
}

void MainWindow::on_pushButton_flightGearDisconnect_pressed() {
    flightGearDisconnect();
}

void MainWindow::flightGearConnect() {
    if (!socket) {
        label_status->setText("please press simulate or trim first");
        return;
    }
    int subSystems = 1;
    std::vector<JSBSim::FGPropertyManager *> outputProperties;
    socket->Disable();
    if (!socket->Load(subSystems,"UDP","FLIGHTGEAR",
                lineEdit_flightGearPort->text().toStdString(),
                lineEdit_flightGearHost->text().toStdString(),
                lineEdit_flightGearRate->text().toUInt(),
                outputProperties)) {
        label_status->setText("unable to open FlightGear socket");
    }
    socket->Enable();
}

void MainWindow::flightGearDisconnect() {
    if (fdm) delete fdm;
    if (socket) delete socket;
    label_status->setText("closed socket and terminated simulation");
}

void MainWindow::save()
{
    if (isLocked(mutex)) {
        std::cout << "please press stop first" << std::endl;
        stop();
        return;
    }
    QMutexLocker locker(&mutex);
    //std::cout << "save locked mutex" << std::endl;

    if (!(trimmer && solver && solver->status() == 0)) {
        std::cout << "trim first" << std::endl;
        stop();
        return;
    }

	using namespace JSBSim;

    std::vector< std::vector<double> > A,B,C,D;
    std::vector<double> x0 = ss->x.get(), u0 = ss->u.get();
    std::vector<double> y0 = x0; // state feedback
    ss->linearize(x0,u0,y0,A,B,C,D);

    // strings
    std::string aircraft = lineEdit_aircraft->text().toStdString();
    std::string outputPath = lineEdit_outputPath->text().toStdString();
    std::string caseName = lineEdit_caseName->text().toStdString();

	// write scicoslab file
    std::string linearizationFileName = outputPath+"/"+aircraft+"_"+caseName+"_lin.sce";
    std::ofstream scicos(linearizationFileName.c_str());
    scicos.precision(10);
    int width=20;
    scicos
        << std::scientific
        << "x0=..\n" << std::setw(width) << x0 << ";\n"
        << "u0=..\n" << std::setw(width) << u0 << ";\n"
        << "sys = syslin('c',..\n"
        << std::setw(width) << A << ",..\n"
        << std::setw(width) << B << ",..\n"
        << std::setw(width) << C << ",..\n"
        << std::setw(width) << D << ");\n"
        << "tfm = ss2tf(sys);\n"
        << std::endl;

	// write trim file
    std::string trimFileName = outputPath+"/"+aircraft+"_"+caseName+"_trim.txt";
    std::ofstream trimFile(trimFileName.c_str());
    trimmer->printSolution(trimFile,solver->getSolution()); // this also loads the solution into the fdm
    label_status->setText(std::string("case:  " + caseName + " saved").c_str());

    //std::cout << "save unlocking mutex" << std::endl;
}

void MainWindow::on_pushButton_generateScript_pressed()
{
    if (isLocked(mutex)) {
        std::cout << "please press stop first" << std::endl;
        stop();
        return;
    }
    QMutexLocker locker(&mutex);
    //std::cout << "generate script locked mutex" << std::endl;
    //std::cout << "generate script unlocking mutex" << std::endl;
}

void MainWindow::on_pushButton_stop_pressed()
{
    stop();
}

void MainWindow::on_pushButton_simulate_pressed()
{
    if (!fdm) {
        stop();
        return;
    }
	writeSettings();
    label_status->setText("simulating");
	simThread.start();
}

void MainWindow::linearize()
{
    if (isLocked(mutex)) {
        std::cout << "please press stop first" << std::endl;
        stop();
        return;
    }
    QMutexLocker locker(&mutex);
    //std::cout << "linearize locked mutex" << std::endl;

	using namespace JSBSim;

    if (!fdm) {
        stop();
        return;
    }
	writeSettings();

	std::cout << "\nlinearization: " << std::endl;
	std::vector< std::vector<double> > A,B,C,D;
	std::vector<double> x0 = ss->x.get(), u0 = ss->u.get();
	std::vector<double> y0 = x0; // state feedback

	ss->linearize(x0,u0,y0,A,B,C,D);
	int width=10;
	std::cout.precision(3);
	std::cout
	<< std::fixed
	<< std::right
    << "x0=..\n" << std::setw(width) << x0 << ";\n"
    << "u0=..\n" << std::setw(width) << u0 << ";\n"
	<< "\nA=\n" << std::setw(width) << A
	<< "\nB=\n" << std::setw(width) << B
	<< "\nC=\n" << std::setw(width) << C
	<< "\nD=\n" << std::setw(width) << D
	<< std::endl;

    //std::cout << "linearize unlocking mutex" << std::endl;
}

void MainWindow::stop()
{
	simThread.quit();
	trimThread.quit();
    label_status->setText("stopped");
}

bool MainWindow::isLocked(QMutex & mutex) {
    if (!mutex.tryLock()) {
        return true;
    } else {
        mutex.unlock();
        return false;
    }
}

void MainWindow::stopSolver()
{
	stopRequested = true;
}

void MainWindow::showMsg(const QString & str)
{
	QMessageBox msgBox;
	msgBox.setText(str);
	msgBox.exec();
};

bool MainWindow::setupFdm() {
    using namespace JSBSim;

    if (fdm) delete fdm;
    fdm = new FGFDMExec;
    if (socket) delete socket;
    socket = new JSBSim::FGOutput(fdm);
    socket->Disable();

    if (ss) delete ss;
    ss = new FGStateSpace(JSBSim::FGStateSpace(fdm));

    if (trimmer) delete trimmer;
    trimmer = new FGTrimmer(fdm,constraints);

	double dt = 1./atof(lineEdit_modelSimRate->text().toAscii());
	int debugLevel = atoi(comboBox_debugLevel->currentText().toAscii());
	fdm->Setdt(dt);
    fdm->SetDebugLevel(debugLevel);

	// paths
	std::string aircraft=lineEdit_aircraft->text().toStdString();
	std::string aircraftPath=lineEdit_aircraftPath->text().toStdString();
	std::string enginePath=lineEdit_enginePath->text().toStdString();
	std::string systemsPath=lineEdit_systemsPath->text().toStdString();
	std::string initScript=lineEdit_initScript->text().toStdString();

	// flight conditions
	bool stabAxisRoll = checkBox_stabAxisRoll->isChecked();
	bool variablePropPitch = checkBox_variablePropPitch->isChecked();

	if (!fdm->LoadModel(aircraftPath,enginePath,systemsPath,aircraft,false))
	{
        label_status->setText("model paths incorrect");
		return false;
	}
	std::string aircraftName = fdm->GetAircraft()->GetAircraftName();
	std::cout << "\tsuccessfully loaded: " <<  aircraftName << std::endl;

    // Set fuel level
    for (int i=0;i<fdm->GetPropulsion()->GetNumTanks();i++) {
        fdm->GetPropulsion()->GetTank(i)->SetContents(
            atof(lineEdit_fuel->text().toAscii())/100.0*
            fdm->GetPropulsion()->GetTank(i)->GetCapacity());
    }

	// Turn on propulsion system
	fdm->GetPropulsion()->InitRunning(-1);
    fdm->GetFCS()->SetDfCmd(atof(lineEdit_flapPos->text().toAscii())/100.0);

    // Set payload, assuming payload is a point mass at c.g. so can just 
    // increase the empty weight of the aircraft
    fdm->GetMassBalance()->SetEmptyWeight(fdm->GetMassBalance()->GetEmptyWeight() + 
        atof(lineEdit_payload->text().toAscii()));

 	// get propulsion pointer to determine type/ etc.
	FGEngine * engine0 = fdm->GetPropulsion()->GetEngine(0);
	FGThruster * thruster0 = engine0->GetThruster();

    ss->clear();

	// longitudinal states
	ss->x.add(new FGStateSpace::Vt); 	// 0 
	ss->x.add(new FGStateSpace::Alpha); // 1
	ss->x.add(new FGStateSpace::Theta); // 2
	ss->x.add(new FGStateSpace::Q); 	// 3
	ss->x.add(new FGStateSpace::Alt); 	// 4

	// lateral states
	ss->x.add(new FGStateSpace::Beta);  // 5
	ss->x.add(new FGStateSpace::Phi); 	// 6
	ss->x.add(new FGStateSpace::P); 	// 7
	ss->x.add(new FGStateSpace::R); 	// 8
	ss->x.add(new FGStateSpace::Psi); 	// 9

	// nav states
	ss->x.add(new FGStateSpace::Longitude); // 10
	ss->x.add(new FGStateSpace::Latitude); // 11

	// propulsion states
	if (thruster0->GetType()==FGThruster::ttPropeller)
    {
        ss->x.add(new FGStateSpace::Rpm0);
        if (variablePropPitch) ss->x.add(new FGStateSpace::PropPitch);
		int numEngines = fdm->GetPropulsion()->GetNumEngines();
		if (numEngines>1) ss->x.add(new FGStateSpace::Rpm1);
		if (numEngines>2) ss->x.add(new FGStateSpace::Rpm2);
		if (numEngines>3) ss->x.add(new FGStateSpace::Rpm3);
    }

	// input
	ss->u.add(new FGStateSpace::ThrottleCmd); 	// 0
	ss->u.add(new FGStateSpace::DaCmd); 		// 1
	ss->u.add(new FGStateSpace::DeCmd); 		// 2
	ss->u.add(new FGStateSpace::DrCmd); 		// 3

	// state feedback
	ss->y = ss->x;

    // if there is a trim solution, load it
    if (trimmer && solver && solver->status() == 0) {
        trimmer->printSolution(std::cout,solver->getSolution()); // this also loads the solution into the fdm
    }

    // connect to socket
    // TODO do this based on switch
    flightGearConnect();
    return true;
}

void MainWindow::simulate()
{
    if (isLocked(mutex)) {
        std::cout << "please press stop first" << std::endl;
        stop();
        return;
    }
    QMutexLocker locker(&mutex);
    //std::cout << "sim thread locked mutex" << std::endl;

    if (!fdm) {
        stop();
        return; 
    }

	fdm->Run();

#ifdef WITH_ARKOSG
    if (plane) {
        double maxDeflection = 20.0*3.14/180.0; // TODO: this is rough
        viewer->mutex.lock();
        plane->setEuler(ss->x.get(6),ss->x.get(2),ss->x.get(9));
        plane->setU(ss->u.get(0),ss->u.get(1)*maxDeflection,
                ss->u.get(2)*maxDeflection,ss->u.get(3)*maxDeflection);
        viewer->mutex.unlock();
    }
#endif
    if (socket) socket->FlightGearSocketOutput();

    //std::cout << "sim thread unlocked mutex" << std::endl;
}

void MainWindow::trim()
{
	using namespace JSBSim;

    if (isLocked(mutex)) {
        std::cout << "please press stop first" << std::endl;
        stop();
        return;
    }
    QMutexLocker locker(&mutex);

    if (!setupFdm()) {
        stop();
        return;
    }
	writeSettings();

    // constraints
	constraints->velocity = atof(lineEdit_velocity->text().toAscii());
	constraints->altitude= atof(lineEdit_altitude->text().toAscii());
	constraints->gamma= atof(lineEdit_gamma->text().toAscii())*M_PI/180.0;
	constraints->rollRate= atof(lineEdit_rollRate->text().toAscii());
	constraints->pitchRate= atof(lineEdit_pitchRate->text().toAscii());
	constraints->yawRate= atof(lineEdit_yawRate->text().toAscii());

   
	// solver properties 
	bool showConvergeStatus = checkBox_showConvergence->isChecked();
	bool showSimplex = checkBox_showSimplex->isChecked();
	bool pause = checkBox_pause->isChecked();
	double rtol = atof(lineEdit_rtol->text().toAscii());
	double abstol = atof(lineEdit_abstol->text().toAscii());
	double speed = atof(lineEdit_speed->text().toAscii());
	double random = atof(lineEdit_random->text().toAscii());
	int iterMax = atof(lineEdit_iterMax->text().toAscii());

	// initial solver state
	int n = 6;
	std::vector<double> initialGuess(n), lowerBound(n), upperBound(n), initialStepSize(n);

	lowerBound[0] = atof(lineEdit_throttleMin->text().toAscii())/100.0;
	lowerBound[1] = atof(lineEdit_elevatorMin->text().toAscii())/100.0;
	lowerBound[2] = atof(lineEdit_alphaMin->text().toAscii())*M_PI/180.0;
	lowerBound[3] = atof(lineEdit_aileronMin->text().toAscii())/100.0;
	lowerBound[4] = atof(lineEdit_rudderMin->text().toAscii())/100.0;
	lowerBound[5] = atof(lineEdit_betaMin->text().toAscii())*M_PI/180.0;

	upperBound[0] = atof(lineEdit_throttleMax->text().toAscii())/100.0; 
	upperBound[1] = atof(lineEdit_elevatorMax->text().toAscii())/100.0; 
	upperBound[2] = atof(lineEdit_alphaMax->text().toAscii())*M_PI/180.0; 
	upperBound[3] = atof(lineEdit_aileronMax->text().toAscii())/100.0; 
	upperBound[4] = atof(lineEdit_rudderMax->text().toAscii())/100.0; 
	upperBound[5] = atof(lineEdit_betaMax->text().toAscii())*M_PI/180.0; 

	initialGuess[0] = atof(lineEdit_throttleGuess->text().toAscii())/100.0; 
	initialGuess[1] = atof(lineEdit_elevatorGuess->text().toAscii())/100.0; 
	initialGuess[2] = atof(lineEdit_alphaGuess->text().toAscii())*M_PI/180.0; 
	initialGuess[3] = atof(lineEdit_aileronGuess->text().toAscii())/100.0; 
	initialGuess[4] = atof(lineEdit_rudderGuess->text().toAscii())/100.0; 
	initialGuess[5] = atof(lineEdit_betaGuess->text().toAscii())*M_PI/180.0; 

	initialStepSize[0] = atof(lineEdit_throttleInitialStepSize->text().toAscii())/100.0; 
	initialStepSize[1] = atof(lineEdit_elevatorInitialStepSize->text().toAscii())/100.0; 
	initialStepSize[2] = atof(lineEdit_alphaInitialStepSize->text().toAscii())*M_PI/180.0; 
	initialStepSize[3] = atof(lineEdit_aileronInitialStepSize->text().toAscii())/100.0; 
	initialStepSize[4] = atof(lineEdit_rudderInitialStepSize->text().toAscii())/100.0; 
	initialStepSize[5] = atof(lineEdit_betaInitialStepSize->text().toAscii())*M_PI/180.0; 

	// solve
    if (solver) delete solver;
	solver = new FGNelderMead(trimmer,initialGuess, lowerBound, upperBound, initialStepSize,
						iterMax,rtol,abstol,speed, random, showConvergeStatus,showSimplex,pause,callback);
	stopRequested = false;
	while(1) {
        if (stopRequested || (solver->status() != 1) ) {
            break;
        } else {
            solver->update();
        }
    }

	// output
    if (trimmer && solver && solver->status() == 0) {
        trimmer->printSolution(std::cout,solver->getSolution()); // this also loads the solution into the fdm
    }
    if (stopRequested) {
        label_status->setText("stopped");
    } else if (solver->status()==0) {
        label_status->setText("trim converged");
    } else if (solver->status()==-1) {
        label_status->setText("trim failed to converge");
    } else {
        label_status->setText("unknown trim status");
    }

    //std::cout << "trim thread unlocking mutex" << std::endl;
}

SimulateThread::SimulateThread(MainWindow * window) : window(window), timer(this)
{
    moveToThread(this);
	connect(&timer, SIGNAL(timeout()), window, SLOT(simulate()),Qt::QueuedConnection);
}

void SimulateThread::run()
{
	timer.start(1000/120);
	exec();
}

void SimulateThread::quit()
{
    timer.stop();
    QThread::quit();
}

TrimThread::TrimThread(MainWindow * window) : window(window)
{
    moveToThread(this);
}

void TrimThread::trim() {
    try {
        window->trim();
    }
    catch(const std::exception & e)
    {
        std::cout << e.what() << std::endl;
        window->label_status->setText("trim failed");
    }
   quit();
}

void TrimThread::run()
{
    QTimer::singleShot(0,this,SLOT(trim()));
    exec();
}

void TrimThread::quit()
{
    window->stopSolver(); 
    QThread::quit();
}

// vim:ts=4:sw=4
