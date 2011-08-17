/********************************************************************************
** Form generated from reading UI file 'MainWindow.ui'
**
** Created: Tue Aug 16 22:39:02 2011
**      by: Qt User Interface Compiler version 4.7.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QDockWidget>
#include <QtGui/QGridLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QMainWindow>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QTabWidget>
#include <QtGui/QToolButton>
#include <QtGui/QWidget>
#include "QOSGAdapterWidget.hpp"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionQuit;
    QAction *actionLoad_Map;
    QWidget *centralwidget;
    QGridLayout *gridLayout;
    ViewerQT *viewer;
    QPushButton *pushButton_trim;
    QPushButton *pushButton_simulate;
    QPushButton *pushButton_linearize;
    QPushButton *pushButton_stop;
    QStatusBar *statusbar;
    QDockWidget *guidanceDock;
    QWidget *dockWidgetContents_6;
    QGridLayout *gridLayout_4;
    QTabWidget *tabWidget;
    QWidget *tab_3;
    QGridLayout *gridLayout_5;
    QLabel *label_2;
    QLineEdit *lineEdit_enginePath;
    QLabel *label_4;
    QLabel *label_3;
    QLineEdit *lineEdit_systemsPath;
    QToolButton *toolButton_enginePath;
    QToolButton *toolButton_systemsPath;
    QToolButton *toolButton_aircraftPath;
    QLabel *label_9;
    QLineEdit *lineEdit_aircraft;
    QToolButton *toolButton_aircraft;
    QLabel *label_5;
    QLineEdit *lineEdit_initScript;
    QToolButton *toolButton_initScript;
    QLineEdit *lineEdit_aircraftPath;
    QLabel *label_38;
    QLineEdit *lineEdit_modelSimRate;
    QLabel *label_39;
    QWidget *tab;
    QGridLayout *gridLayout_2;
    QLabel *label_10;
    QComboBox *comboBox_mode;
    QLabel *label;
    QLineEdit *lineEdit_velocity;
    QLabel *label_11;
    QLabel *label_6;
    QLineEdit *lineEdit_rollRate;
    QLabel *label_12;
    QLabel *label_7;
    QLineEdit *lineEdit_pitchRate;
    QLabel *label_13;
    QLabel *label_8;
    QLabel *label_14;
    QLineEdit *lineEdit_yawRate;
    QCheckBox *checkBox_stabAxisRoll;
    QLabel *label_40;
    QLineEdit *lineEdit_altitude;
    QLabel *label_41;
    QLabel *label_42;
    QLineEdit *lineEdit_gamma;
    QCheckBox *checkBox_variablePropPitch;
    QLabel *label_43;
    QWidget *tab_2;
    QGridLayout *gridLayout_6;
    QLabel *label_20;
    QLabel *label_21;
    QLabel *label_22;
    QLabel *label_23;
    QLineEdit *lineEdit_abstol;
    QLineEdit *lineEdit_speed;
    QLineEdit *lineEdit_iterMax;
    QComboBox *comboBox_debugLevel;
    QLabel *label_34;
    QCheckBox *checkBox_pause;
    QLineEdit *lineEdit_rtol;
    QCheckBox *checkBox_showSimplex;
    QCheckBox *checkBox_showConvergence;
    QLabel *label_44;
    QLineEdit *lineEdit_random;
    QWidget *tab_6;
    QGridLayout *gridLayout_8;
    QLabel *label_36;
    QLineEdit *lineEdit_joystick;
    QToolButton *toolButton;
    QCheckBox *checkBox_joystickEnabled;
    QWidget *tab_4;
    QGridLayout *gridLayout_7;
    QLabel *label_15;
    QLabel *label_17;
    QLabel *label_19;
    QLineEdit *lineEdit_throttleGuess;
    QLineEdit *lineEdit_aileronGuess;
    QLineEdit *lineEdit_elevatorGuess;
    QLineEdit *lineEdit_alphaGuess;
    QLineEdit *lineEdit_betaGuess;
    QLabel *label_24;
    QLabel *label_25;
    QLabel *label_26;
    QLabel *label_27;
    QLabel *label_28;
    QLineEdit *lineEdit_throttleMin;
    QLineEdit *lineEdit_aileronMin;
    QLineEdit *lineEdit_elevatorMin;
    QLineEdit *lineEdit_alphaMin;
    QLineEdit *lineEdit_betaMin;
    QLabel *label_29;
    QLabel *label_30;
    QLineEdit *lineEdit_throttleMax;
    QLineEdit *lineEdit_aileronMax;
    QLineEdit *lineEdit_elevatorMax;
    QLineEdit *lineEdit_alphaMax;
    QLineEdit *lineEdit_betaMax;
    QLabel *label_31;
    QLabel *label_16;
    QLabel *label_18;
    QLabel *label_32;
    QLineEdit *lineEdit_rudderGuess;
    QLineEdit *lineEdit_rudderMin;
    QLineEdit *lineEdit_rudderMax;
    QLabel *label_33;
    QLineEdit *lineEdit_throttleInitialStepSize;
    QLineEdit *lineEdit_aileronInitialStepSize;
    QLineEdit *lineEdit_rudderInitialStepSize;
    QLineEdit *lineEdit_elevatorInitialStepSize;
    QLineEdit *lineEdit_alphaInitialStepSize;
    QLineEdit *lineEdit_betaInitialStepSize;
    QLabel *label_37;
    QWidget *tab_5;
    QGridLayout *gridLayout_3;
    QLabel *label_35;
    QLineEdit *lineEdit_linearizationFile;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(825, 862);
        QSizePolicy sizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(MainWindow->sizePolicy().hasHeightForWidth());
        MainWindow->setSizePolicy(sizePolicy);
        actionQuit = new QAction(MainWindow);
        actionQuit->setObjectName(QString::fromUtf8("actionQuit"));
        actionLoad_Map = new QAction(MainWindow);
        actionLoad_Map->setObjectName(QString::fromUtf8("actionLoad_Map"));
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        gridLayout = new QGridLayout(centralwidget);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        viewer = new ViewerQT(centralwidget);
        viewer->setObjectName(QString::fromUtf8("viewer"));
        sizePolicy.setHeightForWidth(viewer->sizePolicy().hasHeightForWidth());
        viewer->setSizePolicy(sizePolicy);

        gridLayout->addWidget(viewer, 0, 0, 1, 4);

        pushButton_trim = new QPushButton(centralwidget);
        pushButton_trim->setObjectName(QString::fromUtf8("pushButton_trim"));

        gridLayout->addWidget(pushButton_trim, 1, 0, 1, 1);

        pushButton_simulate = new QPushButton(centralwidget);
        pushButton_simulate->setObjectName(QString::fromUtf8("pushButton_simulate"));

        gridLayout->addWidget(pushButton_simulate, 1, 3, 1, 1);

        pushButton_linearize = new QPushButton(centralwidget);
        pushButton_linearize->setObjectName(QString::fromUtf8("pushButton_linearize"));

        gridLayout->addWidget(pushButton_linearize, 1, 2, 1, 1);

        pushButton_stop = new QPushButton(centralwidget);
        pushButton_stop->setObjectName(QString::fromUtf8("pushButton_stop"));

        gridLayout->addWidget(pushButton_stop, 1, 1, 1, 1);

        MainWindow->setCentralWidget(centralwidget);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);
        guidanceDock = new QDockWidget(MainWindow);
        guidanceDock->setObjectName(QString::fromUtf8("guidanceDock"));
        sizePolicy.setHeightForWidth(guidanceDock->sizePolicy().hasHeightForWidth());
        guidanceDock->setSizePolicy(sizePolicy);
        guidanceDock->setMinimumSize(QSize(685, 446));
        dockWidgetContents_6 = new QWidget();
        dockWidgetContents_6->setObjectName(QString::fromUtf8("dockWidgetContents_6"));
        gridLayout_4 = new QGridLayout(dockWidgetContents_6);
        gridLayout_4->setObjectName(QString::fromUtf8("gridLayout_4"));
        tabWidget = new QTabWidget(dockWidgetContents_6);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tab_3 = new QWidget();
        tab_3->setObjectName(QString::fromUtf8("tab_3"));
        gridLayout_5 = new QGridLayout(tab_3);
        gridLayout_5->setObjectName(QString::fromUtf8("gridLayout_5"));
        label_2 = new QLabel(tab_3);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout_5->addWidget(label_2, 1, 1, 1, 1);

        lineEdit_enginePath = new QLineEdit(tab_3);
        lineEdit_enginePath->setObjectName(QString::fromUtf8("lineEdit_enginePath"));

        gridLayout_5->addWidget(lineEdit_enginePath, 1, 2, 1, 1);

        label_4 = new QLabel(tab_3);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        gridLayout_5->addWidget(label_4, 2, 1, 1, 1);

        label_3 = new QLabel(tab_3);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        gridLayout_5->addWidget(label_3, 3, 1, 1, 1);

        lineEdit_systemsPath = new QLineEdit(tab_3);
        lineEdit_systemsPath->setObjectName(QString::fromUtf8("lineEdit_systemsPath"));

        gridLayout_5->addWidget(lineEdit_systemsPath, 2, 2, 1, 1);

        toolButton_enginePath = new QToolButton(tab_3);
        toolButton_enginePath->setObjectName(QString::fromUtf8("toolButton_enginePath"));

        gridLayout_5->addWidget(toolButton_enginePath, 1, 3, 1, 1);

        toolButton_systemsPath = new QToolButton(tab_3);
        toolButton_systemsPath->setObjectName(QString::fromUtf8("toolButton_systemsPath"));

        gridLayout_5->addWidget(toolButton_systemsPath, 2, 3, 1, 1);

        toolButton_aircraftPath = new QToolButton(tab_3);
        toolButton_aircraftPath->setObjectName(QString::fromUtf8("toolButton_aircraftPath"));

        gridLayout_5->addWidget(toolButton_aircraftPath, 3, 3, 1, 1);

        label_9 = new QLabel(tab_3);
        label_9->setObjectName(QString::fromUtf8("label_9"));

        gridLayout_5->addWidget(label_9, 4, 1, 1, 1);

        lineEdit_aircraft = new QLineEdit(tab_3);
        lineEdit_aircraft->setObjectName(QString::fromUtf8("lineEdit_aircraft"));

        gridLayout_5->addWidget(lineEdit_aircraft, 4, 2, 1, 1);

        toolButton_aircraft = new QToolButton(tab_3);
        toolButton_aircraft->setObjectName(QString::fromUtf8("toolButton_aircraft"));

        gridLayout_5->addWidget(toolButton_aircraft, 4, 3, 1, 1);

        label_5 = new QLabel(tab_3);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        gridLayout_5->addWidget(label_5, 5, 1, 1, 1);

        lineEdit_initScript = new QLineEdit(tab_3);
        lineEdit_initScript->setObjectName(QString::fromUtf8("lineEdit_initScript"));

        gridLayout_5->addWidget(lineEdit_initScript, 5, 2, 1, 1);

        toolButton_initScript = new QToolButton(tab_3);
        toolButton_initScript->setObjectName(QString::fromUtf8("toolButton_initScript"));

        gridLayout_5->addWidget(toolButton_initScript, 5, 3, 1, 1);

        lineEdit_aircraftPath = new QLineEdit(tab_3);
        lineEdit_aircraftPath->setObjectName(QString::fromUtf8("lineEdit_aircraftPath"));

        gridLayout_5->addWidget(lineEdit_aircraftPath, 3, 2, 1, 1);

        label_38 = new QLabel(tab_3);
        label_38->setObjectName(QString::fromUtf8("label_38"));

        gridLayout_5->addWidget(label_38, 0, 1, 1, 1);

        lineEdit_modelSimRate = new QLineEdit(tab_3);
        lineEdit_modelSimRate->setObjectName(QString::fromUtf8("lineEdit_modelSimRate"));

        gridLayout_5->addWidget(lineEdit_modelSimRate, 0, 2, 1, 1);

        label_39 = new QLabel(tab_3);
        label_39->setObjectName(QString::fromUtf8("label_39"));

        gridLayout_5->addWidget(label_39, 0, 3, 1, 1);

        tabWidget->addTab(tab_3, QString());
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        gridLayout_2 = new QGridLayout(tab);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        label_10 = new QLabel(tab);
        label_10->setObjectName(QString::fromUtf8("label_10"));

        gridLayout_2->addWidget(label_10, 0, 0, 1, 1);

        comboBox_mode = new QComboBox(tab);
        comboBox_mode->setObjectName(QString::fromUtf8("comboBox_mode"));

        gridLayout_2->addWidget(comboBox_mode, 0, 1, 1, 6);

        label = new QLabel(tab);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout_2->addWidget(label, 1, 0, 1, 1);

        lineEdit_velocity = new QLineEdit(tab);
        lineEdit_velocity->setObjectName(QString::fromUtf8("lineEdit_velocity"));

        gridLayout_2->addWidget(lineEdit_velocity, 1, 1, 1, 5);

        label_11 = new QLabel(tab);
        label_11->setObjectName(QString::fromUtf8("label_11"));

        gridLayout_2->addWidget(label_11, 1, 6, 1, 1);

        label_6 = new QLabel(tab);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        gridLayout_2->addWidget(label_6, 2, 0, 1, 1);

        lineEdit_rollRate = new QLineEdit(tab);
        lineEdit_rollRate->setObjectName(QString::fromUtf8("lineEdit_rollRate"));

        gridLayout_2->addWidget(lineEdit_rollRate, 2, 1, 1, 5);

        label_12 = new QLabel(tab);
        label_12->setObjectName(QString::fromUtf8("label_12"));

        gridLayout_2->addWidget(label_12, 2, 6, 1, 1);

        label_7 = new QLabel(tab);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        gridLayout_2->addWidget(label_7, 3, 0, 1, 1);

        lineEdit_pitchRate = new QLineEdit(tab);
        lineEdit_pitchRate->setObjectName(QString::fromUtf8("lineEdit_pitchRate"));

        gridLayout_2->addWidget(lineEdit_pitchRate, 3, 1, 1, 5);

        label_13 = new QLabel(tab);
        label_13->setObjectName(QString::fromUtf8("label_13"));

        gridLayout_2->addWidget(label_13, 3, 6, 1, 1);

        label_8 = new QLabel(tab);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        gridLayout_2->addWidget(label_8, 4, 0, 1, 1);

        label_14 = new QLabel(tab);
        label_14->setObjectName(QString::fromUtf8("label_14"));

        gridLayout_2->addWidget(label_14, 4, 6, 1, 1);

        lineEdit_yawRate = new QLineEdit(tab);
        lineEdit_yawRate->setObjectName(QString::fromUtf8("lineEdit_yawRate"));

        gridLayout_2->addWidget(lineEdit_yawRate, 4, 1, 1, 5);

        checkBox_stabAxisRoll = new QCheckBox(tab);
        checkBox_stabAxisRoll->setObjectName(QString::fromUtf8("checkBox_stabAxisRoll"));
        checkBox_stabAxisRoll->setCheckable(true);
        checkBox_stabAxisRoll->setChecked(true);

        gridLayout_2->addWidget(checkBox_stabAxisRoll, 8, 0, 1, 1);

        label_40 = new QLabel(tab);
        label_40->setObjectName(QString::fromUtf8("label_40"));

        gridLayout_2->addWidget(label_40, 5, 0, 1, 1);

        lineEdit_altitude = new QLineEdit(tab);
        lineEdit_altitude->setObjectName(QString::fromUtf8("lineEdit_altitude"));

        gridLayout_2->addWidget(lineEdit_altitude, 5, 1, 1, 5);

        label_41 = new QLabel(tab);
        label_41->setObjectName(QString::fromUtf8("label_41"));

        gridLayout_2->addWidget(label_41, 5, 6, 1, 1);

        label_42 = new QLabel(tab);
        label_42->setObjectName(QString::fromUtf8("label_42"));

        gridLayout_2->addWidget(label_42, 6, 0, 1, 1);

        lineEdit_gamma = new QLineEdit(tab);
        lineEdit_gamma->setObjectName(QString::fromUtf8("lineEdit_gamma"));

        gridLayout_2->addWidget(lineEdit_gamma, 6, 1, 1, 5);

        checkBox_variablePropPitch = new QCheckBox(tab);
        checkBox_variablePropPitch->setObjectName(QString::fromUtf8("checkBox_variablePropPitch"));

        gridLayout_2->addWidget(checkBox_variablePropPitch, 7, 0, 1, 1);

        label_43 = new QLabel(tab);
        label_43->setObjectName(QString::fromUtf8("label_43"));

        gridLayout_2->addWidget(label_43, 6, 6, 1, 1);

        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        gridLayout_6 = new QGridLayout(tab_2);
        gridLayout_6->setObjectName(QString::fromUtf8("gridLayout_6"));
        label_20 = new QLabel(tab_2);
        label_20->setObjectName(QString::fromUtf8("label_20"));

        gridLayout_6->addWidget(label_20, 0, 0, 1, 1);

        label_21 = new QLabel(tab_2);
        label_21->setObjectName(QString::fromUtf8("label_21"));

        gridLayout_6->addWidget(label_21, 1, 0, 1, 1);

        label_22 = new QLabel(tab_2);
        label_22->setObjectName(QString::fromUtf8("label_22"));

        gridLayout_6->addWidget(label_22, 2, 0, 1, 1);

        label_23 = new QLabel(tab_2);
        label_23->setObjectName(QString::fromUtf8("label_23"));

        gridLayout_6->addWidget(label_23, 3, 0, 1, 1);

        lineEdit_abstol = new QLineEdit(tab_2);
        lineEdit_abstol->setObjectName(QString::fromUtf8("lineEdit_abstol"));

        gridLayout_6->addWidget(lineEdit_abstol, 1, 1, 1, 2);

        lineEdit_speed = new QLineEdit(tab_2);
        lineEdit_speed->setObjectName(QString::fromUtf8("lineEdit_speed"));

        gridLayout_6->addWidget(lineEdit_speed, 2, 1, 1, 2);

        lineEdit_iterMax = new QLineEdit(tab_2);
        lineEdit_iterMax->setObjectName(QString::fromUtf8("lineEdit_iterMax"));

        gridLayout_6->addWidget(lineEdit_iterMax, 3, 1, 1, 2);

        comboBox_debugLevel = new QComboBox(tab_2);
        comboBox_debugLevel->setObjectName(QString::fromUtf8("comboBox_debugLevel"));

        gridLayout_6->addWidget(comboBox_debugLevel, 4, 1, 1, 2);

        label_34 = new QLabel(tab_2);
        label_34->setObjectName(QString::fromUtf8("label_34"));

        gridLayout_6->addWidget(label_34, 4, 0, 1, 1);

        checkBox_pause = new QCheckBox(tab_2);
        checkBox_pause->setObjectName(QString::fromUtf8("checkBox_pause"));

        gridLayout_6->addWidget(checkBox_pause, 7, 0, 1, 1);

        lineEdit_rtol = new QLineEdit(tab_2);
        lineEdit_rtol->setObjectName(QString::fromUtf8("lineEdit_rtol"));

        gridLayout_6->addWidget(lineEdit_rtol, 0, 1, 1, 2);

        checkBox_showSimplex = new QCheckBox(tab_2);
        checkBox_showSimplex->setObjectName(QString::fromUtf8("checkBox_showSimplex"));

        gridLayout_6->addWidget(checkBox_showSimplex, 7, 1, 1, 1);

        checkBox_showConvergence = new QCheckBox(tab_2);
        checkBox_showConvergence->setObjectName(QString::fromUtf8("checkBox_showConvergence"));
        checkBox_showConvergence->setChecked(true);

        gridLayout_6->addWidget(checkBox_showConvergence, 7, 2, 1, 1);

        label_44 = new QLabel(tab_2);
        label_44->setObjectName(QString::fromUtf8("label_44"));

        gridLayout_6->addWidget(label_44, 6, 0, 1, 1);

        lineEdit_random = new QLineEdit(tab_2);
        lineEdit_random->setObjectName(QString::fromUtf8("lineEdit_random"));

        gridLayout_6->addWidget(lineEdit_random, 6, 1, 1, 1);

        tabWidget->addTab(tab_2, QString());
        tab_6 = new QWidget();
        tab_6->setObjectName(QString::fromUtf8("tab_6"));
        gridLayout_8 = new QGridLayout(tab_6);
        gridLayout_8->setObjectName(QString::fromUtf8("gridLayout_8"));
        label_36 = new QLabel(tab_6);
        label_36->setObjectName(QString::fromUtf8("label_36"));

        gridLayout_8->addWidget(label_36, 0, 0, 1, 1);

        lineEdit_joystick = new QLineEdit(tab_6);
        lineEdit_joystick->setObjectName(QString::fromUtf8("lineEdit_joystick"));

        gridLayout_8->addWidget(lineEdit_joystick, 0, 1, 1, 1);

        toolButton = new QToolButton(tab_6);
        toolButton->setObjectName(QString::fromUtf8("toolButton"));

        gridLayout_8->addWidget(toolButton, 0, 2, 1, 1);

        checkBox_joystickEnabled = new QCheckBox(tab_6);
        checkBox_joystickEnabled->setObjectName(QString::fromUtf8("checkBox_joystickEnabled"));

        gridLayout_8->addWidget(checkBox_joystickEnabled, 1, 0, 1, 2);

        tabWidget->addTab(tab_6, QString());
        tab_4 = new QWidget();
        tab_4->setObjectName(QString::fromUtf8("tab_4"));
        gridLayout_7 = new QGridLayout(tab_4);
        gridLayout_7->setObjectName(QString::fromUtf8("gridLayout_7"));
        label_15 = new QLabel(tab_4);
        label_15->setObjectName(QString::fromUtf8("label_15"));

        gridLayout_7->addWidget(label_15, 1, 0, 1, 1);

        label_17 = new QLabel(tab_4);
        label_17->setObjectName(QString::fromUtf8("label_17"));

        gridLayout_7->addWidget(label_17, 6, 0, 1, 1);

        label_19 = new QLabel(tab_4);
        label_19->setObjectName(QString::fromUtf8("label_19"));

        gridLayout_7->addWidget(label_19, 7, 0, 1, 1);

        lineEdit_throttleGuess = new QLineEdit(tab_4);
        lineEdit_throttleGuess->setObjectName(QString::fromUtf8("lineEdit_throttleGuess"));

        gridLayout_7->addWidget(lineEdit_throttleGuess, 1, 1, 1, 1);

        lineEdit_aileronGuess = new QLineEdit(tab_4);
        lineEdit_aileronGuess->setObjectName(QString::fromUtf8("lineEdit_aileronGuess"));

        gridLayout_7->addWidget(lineEdit_aileronGuess, 3, 1, 1, 1);

        lineEdit_elevatorGuess = new QLineEdit(tab_4);
        lineEdit_elevatorGuess->setObjectName(QString::fromUtf8("lineEdit_elevatorGuess"));

        gridLayout_7->addWidget(lineEdit_elevatorGuess, 5, 1, 1, 1);

        lineEdit_alphaGuess = new QLineEdit(tab_4);
        lineEdit_alphaGuess->setObjectName(QString::fromUtf8("lineEdit_alphaGuess"));

        gridLayout_7->addWidget(lineEdit_alphaGuess, 6, 1, 1, 1);

        lineEdit_betaGuess = new QLineEdit(tab_4);
        lineEdit_betaGuess->setObjectName(QString::fromUtf8("lineEdit_betaGuess"));

        gridLayout_7->addWidget(lineEdit_betaGuess, 7, 1, 1, 1);

        label_24 = new QLabel(tab_4);
        label_24->setObjectName(QString::fromUtf8("label_24"));

        gridLayout_7->addWidget(label_24, 1, 5, 1, 1);

        label_25 = new QLabel(tab_4);
        label_25->setObjectName(QString::fromUtf8("label_25"));

        gridLayout_7->addWidget(label_25, 3, 5, 1, 1);

        label_26 = new QLabel(tab_4);
        label_26->setObjectName(QString::fromUtf8("label_26"));

        gridLayout_7->addWidget(label_26, 5, 5, 1, 1);

        label_27 = new QLabel(tab_4);
        label_27->setObjectName(QString::fromUtf8("label_27"));

        gridLayout_7->addWidget(label_27, 6, 5, 1, 1);

        label_28 = new QLabel(tab_4);
        label_28->setObjectName(QString::fromUtf8("label_28"));

        gridLayout_7->addWidget(label_28, 7, 5, 1, 1);

        lineEdit_throttleMin = new QLineEdit(tab_4);
        lineEdit_throttleMin->setObjectName(QString::fromUtf8("lineEdit_throttleMin"));

        gridLayout_7->addWidget(lineEdit_throttleMin, 1, 2, 1, 1);

        lineEdit_aileronMin = new QLineEdit(tab_4);
        lineEdit_aileronMin->setObjectName(QString::fromUtf8("lineEdit_aileronMin"));

        gridLayout_7->addWidget(lineEdit_aileronMin, 3, 2, 1, 1);

        lineEdit_elevatorMin = new QLineEdit(tab_4);
        lineEdit_elevatorMin->setObjectName(QString::fromUtf8("lineEdit_elevatorMin"));

        gridLayout_7->addWidget(lineEdit_elevatorMin, 5, 2, 1, 1);

        lineEdit_alphaMin = new QLineEdit(tab_4);
        lineEdit_alphaMin->setObjectName(QString::fromUtf8("lineEdit_alphaMin"));

        gridLayout_7->addWidget(lineEdit_alphaMin, 6, 2, 1, 1);

        lineEdit_betaMin = new QLineEdit(tab_4);
        lineEdit_betaMin->setObjectName(QString::fromUtf8("lineEdit_betaMin"));

        gridLayout_7->addWidget(lineEdit_betaMin, 7, 2, 1, 1);

        label_29 = new QLabel(tab_4);
        label_29->setObjectName(QString::fromUtf8("label_29"));

        gridLayout_7->addWidget(label_29, 0, 1, 1, 1);

        label_30 = new QLabel(tab_4);
        label_30->setObjectName(QString::fromUtf8("label_30"));

        gridLayout_7->addWidget(label_30, 0, 2, 1, 1);

        lineEdit_throttleMax = new QLineEdit(tab_4);
        lineEdit_throttleMax->setObjectName(QString::fromUtf8("lineEdit_throttleMax"));

        gridLayout_7->addWidget(lineEdit_throttleMax, 1, 3, 1, 1);

        lineEdit_aileronMax = new QLineEdit(tab_4);
        lineEdit_aileronMax->setObjectName(QString::fromUtf8("lineEdit_aileronMax"));

        gridLayout_7->addWidget(lineEdit_aileronMax, 3, 3, 1, 1);

        lineEdit_elevatorMax = new QLineEdit(tab_4);
        lineEdit_elevatorMax->setObjectName(QString::fromUtf8("lineEdit_elevatorMax"));

        gridLayout_7->addWidget(lineEdit_elevatorMax, 5, 3, 1, 1);

        lineEdit_alphaMax = new QLineEdit(tab_4);
        lineEdit_alphaMax->setObjectName(QString::fromUtf8("lineEdit_alphaMax"));

        gridLayout_7->addWidget(lineEdit_alphaMax, 6, 3, 1, 1);

        lineEdit_betaMax = new QLineEdit(tab_4);
        lineEdit_betaMax->setObjectName(QString::fromUtf8("lineEdit_betaMax"));

        gridLayout_7->addWidget(lineEdit_betaMax, 7, 3, 1, 1);

        label_31 = new QLabel(tab_4);
        label_31->setObjectName(QString::fromUtf8("label_31"));

        gridLayout_7->addWidget(label_31, 0, 3, 1, 1);

        label_16 = new QLabel(tab_4);
        label_16->setObjectName(QString::fromUtf8("label_16"));

        gridLayout_7->addWidget(label_16, 5, 0, 1, 1);

        label_18 = new QLabel(tab_4);
        label_18->setObjectName(QString::fromUtf8("label_18"));

        gridLayout_7->addWidget(label_18, 3, 0, 1, 1);

        label_32 = new QLabel(tab_4);
        label_32->setObjectName(QString::fromUtf8("label_32"));

        gridLayout_7->addWidget(label_32, 4, 0, 1, 1);

        lineEdit_rudderGuess = new QLineEdit(tab_4);
        lineEdit_rudderGuess->setObjectName(QString::fromUtf8("lineEdit_rudderGuess"));

        gridLayout_7->addWidget(lineEdit_rudderGuess, 4, 1, 1, 1);

        lineEdit_rudderMin = new QLineEdit(tab_4);
        lineEdit_rudderMin->setObjectName(QString::fromUtf8("lineEdit_rudderMin"));

        gridLayout_7->addWidget(lineEdit_rudderMin, 4, 2, 1, 1);

        lineEdit_rudderMax = new QLineEdit(tab_4);
        lineEdit_rudderMax->setObjectName(QString::fromUtf8("lineEdit_rudderMax"));

        gridLayout_7->addWidget(lineEdit_rudderMax, 4, 3, 1, 1);

        label_33 = new QLabel(tab_4);
        label_33->setObjectName(QString::fromUtf8("label_33"));

        gridLayout_7->addWidget(label_33, 4, 5, 1, 1);

        lineEdit_throttleInitialStepSize = new QLineEdit(tab_4);
        lineEdit_throttleInitialStepSize->setObjectName(QString::fromUtf8("lineEdit_throttleInitialStepSize"));

        gridLayout_7->addWidget(lineEdit_throttleInitialStepSize, 1, 4, 1, 1);

        lineEdit_aileronInitialStepSize = new QLineEdit(tab_4);
        lineEdit_aileronInitialStepSize->setObjectName(QString::fromUtf8("lineEdit_aileronInitialStepSize"));

        gridLayout_7->addWidget(lineEdit_aileronInitialStepSize, 3, 4, 1, 1);

        lineEdit_rudderInitialStepSize = new QLineEdit(tab_4);
        lineEdit_rudderInitialStepSize->setObjectName(QString::fromUtf8("lineEdit_rudderInitialStepSize"));

        gridLayout_7->addWidget(lineEdit_rudderInitialStepSize, 4, 4, 1, 1);

        lineEdit_elevatorInitialStepSize = new QLineEdit(tab_4);
        lineEdit_elevatorInitialStepSize->setObjectName(QString::fromUtf8("lineEdit_elevatorInitialStepSize"));

        gridLayout_7->addWidget(lineEdit_elevatorInitialStepSize, 5, 4, 1, 1);

        lineEdit_alphaInitialStepSize = new QLineEdit(tab_4);
        lineEdit_alphaInitialStepSize->setObjectName(QString::fromUtf8("lineEdit_alphaInitialStepSize"));

        gridLayout_7->addWidget(lineEdit_alphaInitialStepSize, 6, 4, 1, 1);

        lineEdit_betaInitialStepSize = new QLineEdit(tab_4);
        lineEdit_betaInitialStepSize->setObjectName(QString::fromUtf8("lineEdit_betaInitialStepSize"));

        gridLayout_7->addWidget(lineEdit_betaInitialStepSize, 7, 4, 1, 1);

        label_37 = new QLabel(tab_4);
        label_37->setObjectName(QString::fromUtf8("label_37"));

        gridLayout_7->addWidget(label_37, 0, 4, 1, 1);

        tabWidget->addTab(tab_4, QString());
        tab_5 = new QWidget();
        tab_5->setObjectName(QString::fromUtf8("tab_5"));
        gridLayout_3 = new QGridLayout(tab_5);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        label_35 = new QLabel(tab_5);
        label_35->setObjectName(QString::fromUtf8("label_35"));

        gridLayout_3->addWidget(label_35, 0, 0, 1, 1);

        lineEdit_linearizationFile = new QLineEdit(tab_5);
        lineEdit_linearizationFile->setObjectName(QString::fromUtf8("lineEdit_linearizationFile"));

        gridLayout_3->addWidget(lineEdit_linearizationFile, 0, 1, 1, 1);

        tabWidget->addTab(tab_5, QString());

        gridLayout_4->addWidget(tabWidget, 0, 0, 1, 1);

        guidanceDock->setWidget(dockWidgetContents_6);
        MainWindow->addDockWidget(static_cast<Qt::DockWidgetArea>(8), guidanceDock);

        retranslateUi(MainWindow);
        QObject::connect(actionQuit, SIGNAL(triggered()), MainWindow, SLOT(close()));

        tabWidget->setCurrentIndex(5);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "mavsim Trim Program (for JSBSim models)", 0, QApplication::UnicodeUTF8));
        actionQuit->setText(QApplication::translate("MainWindow", "Quit", 0, QApplication::UnicodeUTF8));
        actionLoad_Map->setText(QApplication::translate("MainWindow", "Load Map", 0, QApplication::UnicodeUTF8));
        pushButton_trim->setText(QApplication::translate("MainWindow", "Trim", 0, QApplication::UnicodeUTF8));
        pushButton_simulate->setText(QApplication::translate("MainWindow", "Simulate", 0, QApplication::UnicodeUTF8));
        pushButton_linearize->setText(QApplication::translate("MainWindow", "Linearize", 0, QApplication::UnicodeUTF8));
        pushButton_stop->setText(QApplication::translate("MainWindow", "Stop", 0, QApplication::UnicodeUTF8));
        guidanceDock->setWindowTitle(QApplication::translate("MainWindow", "Trim Algorithm", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MainWindow", "Engine Path", 0, QApplication::UnicodeUTF8));
        lineEdit_enginePath->setText(QString());
        label_4->setText(QApplication::translate("MainWindow", "Systems Path", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("MainWindow", "Aircraft Path", 0, QApplication::UnicodeUTF8));
        lineEdit_systemsPath->setText(QString());
        toolButton_enginePath->setText(QApplication::translate("MainWindow", "...", 0, QApplication::UnicodeUTF8));
        toolButton_systemsPath->setText(QApplication::translate("MainWindow", "...", 0, QApplication::UnicodeUTF8));
        toolButton_aircraftPath->setText(QApplication::translate("MainWindow", "...", 0, QApplication::UnicodeUTF8));
        label_9->setText(QApplication::translate("MainWindow", "Aircraft", 0, QApplication::UnicodeUTF8));
        lineEdit_aircraft->setText(QString());
        toolButton_aircraft->setText(QApplication::translate("MainWindow", "...", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("MainWindow", "Initialization Script", 0, QApplication::UnicodeUTF8));
        lineEdit_initScript->setText(QString());
        toolButton_initScript->setText(QApplication::translate("MainWindow", "...", 0, QApplication::UnicodeUTF8));
        lineEdit_aircraftPath->setText(QString());
        label_38->setText(QApplication::translate("MainWindow", "model sim rate", 0, QApplication::UnicodeUTF8));
        lineEdit_modelSimRate->setText(QApplication::translate("MainWindow", "120", 0, QApplication::UnicodeUTF8));
        label_39->setText(QApplication::translate("MainWindow", "Hz", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QApplication::translate("MainWindow", "Aircraft", 0, QApplication::UnicodeUTF8));
        label_10->setText(QApplication::translate("MainWindow", "mode", 0, QApplication::UnicodeUTF8));
        comboBox_mode->clear();
        comboBox_mode->insertItems(0, QStringList()
         << QApplication::translate("MainWindow", "Steady Level Flight", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "Coordinated Turn", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "Push Over", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "Pull Back", 0, QApplication::UnicodeUTF8)
        );
        label->setText(QApplication::translate("MainWindow", "velocity", 0, QApplication::UnicodeUTF8));
        lineEdit_velocity->setText(QApplication::translate("MainWindow", "40", 0, QApplication::UnicodeUTF8));
        label_11->setText(QApplication::translate("MainWindow", "ft/s", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("MainWindow", "roll rate", 0, QApplication::UnicodeUTF8));
        lineEdit_rollRate->setText(QApplication::translate("MainWindow", "0", 0, QApplication::UnicodeUTF8));
        label_12->setText(QApplication::translate("MainWindow", "rad/s", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("MainWindow", "pitch rate", 0, QApplication::UnicodeUTF8));
        lineEdit_pitchRate->setText(QApplication::translate("MainWindow", "0", 0, QApplication::UnicodeUTF8));
        label_13->setText(QApplication::translate("MainWindow", "rad/s", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("MainWindow", "yaw rate", 0, QApplication::UnicodeUTF8));
        label_14->setText(QApplication::translate("MainWindow", "rad/s", 0, QApplication::UnicodeUTF8));
        lineEdit_yawRate->setText(QApplication::translate("MainWindow", "0", 0, QApplication::UnicodeUTF8));
        checkBox_stabAxisRoll->setText(QApplication::translate("MainWindow", "stability axis roll", 0, QApplication::UnicodeUTF8));
        label_40->setText(QApplication::translate("MainWindow", "altitude", 0, QApplication::UnicodeUTF8));
        lineEdit_altitude->setText(QApplication::translate("MainWindow", "1000", 0, QApplication::UnicodeUTF8));
        label_41->setText(QApplication::translate("MainWindow", "ft", 0, QApplication::UnicodeUTF8));
        label_42->setText(QApplication::translate("MainWindow", "flight path angle", 0, QApplication::UnicodeUTF8));
        lineEdit_gamma->setText(QApplication::translate("MainWindow", "0", 0, QApplication::UnicodeUTF8));
        checkBox_variablePropPitch->setText(QApplication::translate("MainWindow", "variable prop ptich", 0, QApplication::UnicodeUTF8));
        label_43->setText(QApplication::translate("MainWindow", "deg", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("MainWindow", "Trim Conditions", 0, QApplication::UnicodeUTF8));
        label_20->setText(QApplication::translate("MainWindow", "relative tolerance", 0, QApplication::UnicodeUTF8));
        label_21->setText(QApplication::translate("MainWindow", "absolute tolerance", 0, QApplication::UnicodeUTF8));
        label_22->setText(QApplication::translate("MainWindow", "convergence relative step size", 0, QApplication::UnicodeUTF8));
        label_23->setText(QApplication::translate("MainWindow", "max iterations", 0, QApplication::UnicodeUTF8));
        lineEdit_abstol->setText(QApplication::translate("MainWindow", "1e-2", 0, QApplication::UnicodeUTF8));
        lineEdit_speed->setText(QApplication::translate("MainWindow", "1.1", 0, QApplication::UnicodeUTF8));
        lineEdit_iterMax->setText(QApplication::translate("MainWindow", "2000", 0, QApplication::UnicodeUTF8));
        comboBox_debugLevel->clear();
        comboBox_debugLevel->insertItems(0, QStringList()
         << QApplication::translate("MainWindow", "0", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "1", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "2", 0, QApplication::UnicodeUTF8)
        );
        label_34->setText(QApplication::translate("MainWindow", "debug level", 0, QApplication::UnicodeUTF8));
        checkBox_pause->setText(QApplication::translate("MainWindow", "pause ", 0, QApplication::UnicodeUTF8));
        lineEdit_rtol->setText(QApplication::translate("MainWindow", "1e-4", 0, QApplication::UnicodeUTF8));
        checkBox_showSimplex->setText(QApplication::translate("MainWindow", "show simplex", 0, QApplication::UnicodeUTF8));
        checkBox_showConvergence->setText(QApplication::translate("MainWindow", "show convergence", 0, QApplication::UnicodeUTF8));
        label_44->setText(QApplication::translate("MainWindow", "randomization factor", 0, QApplication::UnicodeUTF8));
        lineEdit_random->setText(QApplication::translate("MainWindow", "0", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("MainWindow", "Solver", 0, QApplication::UnicodeUTF8));
        label_36->setText(QApplication::translate("MainWindow", "joystick device", 0, QApplication::UnicodeUTF8));
        lineEdit_joystick->setText(QApplication::translate("MainWindow", "/dev/input/js0", 0, QApplication::UnicodeUTF8));
        toolButton->setText(QApplication::translate("MainWindow", "...", 0, QApplication::UnicodeUTF8));
        checkBox_joystickEnabled->setText(QApplication::translate("MainWindow", "joystick enabled", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_6), QApplication::translate("MainWindow", "Input", 0, QApplication::UnicodeUTF8));
        label_15->setText(QApplication::translate("MainWindow", "Throttle", 0, QApplication::UnicodeUTF8));
        label_17->setText(QApplication::translate("MainWindow", "Alpha", 0, QApplication::UnicodeUTF8));
        label_19->setText(QApplication::translate("MainWindow", "Beta", 0, QApplication::UnicodeUTF8));
        lineEdit_throttleGuess->setText(QApplication::translate("MainWindow", "50", 0, QApplication::UnicodeUTF8));
        lineEdit_aileronGuess->setText(QApplication::translate("MainWindow", "0", 0, QApplication::UnicodeUTF8));
        lineEdit_elevatorGuess->setText(QApplication::translate("MainWindow", "0", 0, QApplication::UnicodeUTF8));
        lineEdit_alphaGuess->setText(QApplication::translate("MainWindow", "0", 0, QApplication::UnicodeUTF8));
        lineEdit_betaGuess->setText(QApplication::translate("MainWindow", "0", 0, QApplication::UnicodeUTF8));
        label_24->setText(QApplication::translate("MainWindow", "%", 0, QApplication::UnicodeUTF8));
        label_25->setText(QApplication::translate("MainWindow", "%", 0, QApplication::UnicodeUTF8));
        label_26->setText(QApplication::translate("MainWindow", "%", 0, QApplication::UnicodeUTF8));
        label_27->setText(QApplication::translate("MainWindow", "deg", 0, QApplication::UnicodeUTF8));
        label_28->setText(QApplication::translate("MainWindow", "deg", 0, QApplication::UnicodeUTF8));
        lineEdit_throttleMin->setText(QApplication::translate("MainWindow", "0", 0, QApplication::UnicodeUTF8));
        lineEdit_aileronMin->setText(QApplication::translate("MainWindow", "0", 0, QApplication::UnicodeUTF8));
        lineEdit_elevatorMin->setText(QApplication::translate("MainWindow", "-100", 0, QApplication::UnicodeUTF8));
        lineEdit_alphaMin->setText(QApplication::translate("MainWindow", "-10", 0, QApplication::UnicodeUTF8));
        lineEdit_betaMin->setText(QApplication::translate("MainWindow", "-10", 0, QApplication::UnicodeUTF8));
        label_29->setText(QApplication::translate("MainWindow", "Guess", 0, QApplication::UnicodeUTF8));
        label_30->setText(QApplication::translate("MainWindow", "Lower Bound", 0, QApplication::UnicodeUTF8));
        lineEdit_throttleMax->setText(QApplication::translate("MainWindow", "100", 0, QApplication::UnicodeUTF8));
        lineEdit_aileronMax->setText(QApplication::translate("MainWindow", "0", 0, QApplication::UnicodeUTF8));
        lineEdit_elevatorMax->setText(QApplication::translate("MainWindow", "100", 0, QApplication::UnicodeUTF8));
        lineEdit_alphaMax->setText(QApplication::translate("MainWindow", "20", 0, QApplication::UnicodeUTF8));
        lineEdit_betaMax->setText(QApplication::translate("MainWindow", "5", 0, QApplication::UnicodeUTF8));
        label_31->setText(QApplication::translate("MainWindow", "Upper Bound", 0, QApplication::UnicodeUTF8));
        label_16->setText(QApplication::translate("MainWindow", "Elevator", 0, QApplication::UnicodeUTF8));
        label_18->setText(QApplication::translate("MainWindow", "Aileron", 0, QApplication::UnicodeUTF8));
        label_32->setText(QApplication::translate("MainWindow", "Rudder", 0, QApplication::UnicodeUTF8));
        lineEdit_rudderGuess->setText(QApplication::translate("MainWindow", "0", 0, QApplication::UnicodeUTF8));
        lineEdit_rudderMin->setText(QApplication::translate("MainWindow", "-100", 0, QApplication::UnicodeUTF8));
        lineEdit_rudderMax->setText(QApplication::translate("MainWindow", "100", 0, QApplication::UnicodeUTF8));
        label_33->setText(QApplication::translate("MainWindow", "%", 0, QApplication::UnicodeUTF8));
        lineEdit_throttleInitialStepSize->setText(QApplication::translate("MainWindow", "20", 0, QApplication::UnicodeUTF8));
        lineEdit_aileronInitialStepSize->setText(QApplication::translate("MainWindow", "10", 0, QApplication::UnicodeUTF8));
        lineEdit_rudderInitialStepSize->setText(QApplication::translate("MainWindow", "10", 0, QApplication::UnicodeUTF8));
        lineEdit_elevatorInitialStepSize->setText(QApplication::translate("MainWindow", "10", 0, QApplication::UnicodeUTF8));
        lineEdit_alphaInitialStepSize->setText(QApplication::translate("MainWindow", "10", 0, QApplication::UnicodeUTF8));
        lineEdit_betaInitialStepSize->setText(QApplication::translate("MainWindow", "5", 0, QApplication::UnicodeUTF8));
        label_37->setText(QApplication::translate("MainWindow", "Initial Step Size", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_4), QApplication::translate("MainWindow", "Initial Guess", 0, QApplication::UnicodeUTF8));
        label_35->setText(QApplication::translate("MainWindow", "Linearization File", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_5), QApplication::translate("MainWindow", "Output", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // MAINWINDOW_H
