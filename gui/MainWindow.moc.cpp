/****************************************************************************
** Meta object code from reading C++ file 'MainWindow.hpp'
**
** Created: Tue Aug 16 21:04:18 2011
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "MainWindow.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'MainWindow.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_SimulateThread[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

       0        // eod
};

static const char qt_meta_stringdata_SimulateThread[] = {
    "SimulateThread\0"
};

const QMetaObject SimulateThread::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_SimulateThread,
      qt_meta_data_SimulateThread, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &SimulateThread::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *SimulateThread::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *SimulateThread::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_SimulateThread))
        return static_cast<void*>(const_cast< SimulateThread*>(this));
    return QThread::qt_metacast(_clname);
}

int SimulateThread::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    return _id;
}
static const uint qt_meta_data_TrimThread[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

       0        // eod
};

static const char qt_meta_stringdata_TrimThread[] = {
    "TrimThread\0"
};

const QMetaObject TrimThread::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_TrimThread,
      qt_meta_data_TrimThread, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &TrimThread::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *TrimThread::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *TrimThread::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_TrimThread))
        return static_cast<void*>(const_cast< TrimThread*>(this));
    return QThread::qt_metacast(_clname);
}

int TrimThread::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    return _id;
}
static const uint qt_meta_data_MainWindow[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
      13,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      16,   12,   11,   11, 0x05,

 // slots: signature, parameters, type, tag, flags
      41,   11,   11,   11, 0x08,
      76,   11,   11,   11, 0x08,
     112,   11,   11,   11, 0x08,
     149,   11,   11,   11, 0x08,
     182,   11,   11,   11, 0x08,
     217,   11,   11,   11, 0x08,
     246,   11,   11,   11, 0x08,
     275,   11,   11,   11, 0x08,
     309,   11,   11,   11, 0x08,
     342,   12,   11,   11, 0x08,
     359,   11,   11,   11, 0x08,
     370,   11,   11,   11, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_MainWindow[] = {
    "MainWindow\0\0str\0showMsgBuffered(QString)\0"
    "on_toolButton_enginePath_pressed()\0"
    "on_toolButton_systemsPath_pressed()\0"
    "on_toolButton_aircraftPath_pressed()\0"
    "on_toolButton_aircraft_pressed()\0"
    "on_toolButton_initScript_pressed()\0"
    "on_pushButton_trim_pressed()\0"
    "on_pushButton_stop_pressed()\0"
    "on_pushButton_linearize_pressed()\0"
    "on_pushButton_simulate_pressed()\0"
    "showMsg(QString)\0simulate()\0trim()\0"
};

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow,
      qt_meta_data_MainWindow, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &MainWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: showMsgBuffered((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 1: on_toolButton_enginePath_pressed(); break;
        case 2: on_toolButton_systemsPath_pressed(); break;
        case 3: on_toolButton_aircraftPath_pressed(); break;
        case 4: on_toolButton_aircraft_pressed(); break;
        case 5: on_toolButton_initScript_pressed(); break;
        case 6: on_pushButton_trim_pressed(); break;
        case 7: on_pushButton_stop_pressed(); break;
        case 8: on_pushButton_linearize_pressed(); break;
        case 9: on_pushButton_simulate_pressed(); break;
        case 10: showMsg((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 11: simulate(); break;
        case 12: trim(); break;
        default: ;
        }
        _id -= 13;
    }
    return _id;
}

// SIGNAL 0
void MainWindow::showMsgBuffered(const QString & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
