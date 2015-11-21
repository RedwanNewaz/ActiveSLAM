/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.4.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../mainwindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.4.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_MainWindow_t {
    QByteArrayData data[18];
    char stringdata[343];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainWindow_t, stringdata) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainWindow_t qt_meta_stringdata_MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 10), // "MainWindow"
QT_MOC_LITERAL(1, 11, 23), // "on_buttonMoveit_clicked"
QT_MOC_LITERAL(2, 35, 0), // ""
QT_MOC_LITERAL(3, 36, 22), // "on_buttonStart_clicked"
QT_MOC_LITERAL(4, 59, 21), // "on_buttonStop_clicked"
QT_MOC_LITERAL(5, 81, 24), // "on_buttonTakeOff_clicked"
QT_MOC_LITERAL(6, 106, 21), // "on_buttonLand_clicked"
QT_MOC_LITERAL(7, 128, 21), // "on_pushButton_clicked"
QT_MOC_LITERAL(8, 150, 21), // "on_rollKp_sliderMoved"
QT_MOC_LITERAL(9, 172, 6), // "action"
QT_MOC_LITERAL(10, 179, 21), // "on_rollKd_sliderMoved"
QT_MOC_LITERAL(11, 201, 22), // "on_pitchKp_sliderMoved"
QT_MOC_LITERAL(12, 224, 22), // "on_pitchKd_sliderMoved"
QT_MOC_LITERAL(13, 247, 20), // "on_altKp_sliderMoved"
QT_MOC_LITERAL(14, 268, 20), // "on_altKd_sliderMoved"
QT_MOC_LITERAL(15, 289, 20), // "on_YawKp_sliderMoved"
QT_MOC_LITERAL(16, 310, 20), // "on_YawKd_sliderMoved"
QT_MOC_LITERAL(17, 331, 11) // "timeChanged"

    },
    "MainWindow\0on_buttonMoveit_clicked\0\0"
    "on_buttonStart_clicked\0on_buttonStop_clicked\0"
    "on_buttonTakeOff_clicked\0on_buttonLand_clicked\0"
    "on_pushButton_clicked\0on_rollKp_sliderMoved\0"
    "action\0on_rollKd_sliderMoved\0"
    "on_pitchKp_sliderMoved\0on_pitchKd_sliderMoved\0"
    "on_altKp_sliderMoved\0on_altKd_sliderMoved\0"
    "on_YawKp_sliderMoved\0on_YawKd_sliderMoved\0"
    "timeChanged"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      15,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   89,    2, 0x08 /* Private */,
       3,    0,   90,    2, 0x08 /* Private */,
       4,    0,   91,    2, 0x08 /* Private */,
       5,    0,   92,    2, 0x08 /* Private */,
       6,    0,   93,    2, 0x08 /* Private */,
       7,    0,   94,    2, 0x08 /* Private */,
       8,    1,   95,    2, 0x08 /* Private */,
      10,    1,   98,    2, 0x08 /* Private */,
      11,    1,  101,    2, 0x08 /* Private */,
      12,    1,  104,    2, 0x08 /* Private */,
      13,    1,  107,    2, 0x08 /* Private */,
      14,    1,  110,    2, 0x08 /* Private */,
      15,    1,  113,    2, 0x08 /* Private */,
      16,    1,  116,    2, 0x08 /* Private */,
      17,    0,  119,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    9,
    QMetaType::Void, QMetaType::Int,    9,
    QMetaType::Void, QMetaType::Int,    9,
    QMetaType::Void, QMetaType::Int,    9,
    QMetaType::Void, QMetaType::Int,    9,
    QMetaType::Void, QMetaType::Int,    9,
    QMetaType::Void, QMetaType::Int,    9,
    QMetaType::Void, QMetaType::Int,    9,
    QMetaType::Void,

       0        // eod
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MainWindow *_t = static_cast<MainWindow *>(_o);
        switch (_id) {
        case 0: _t->on_buttonMoveit_clicked(); break;
        case 1: _t->on_buttonStart_clicked(); break;
        case 2: _t->on_buttonStop_clicked(); break;
        case 3: _t->on_buttonTakeOff_clicked(); break;
        case 4: _t->on_buttonLand_clicked(); break;
        case 5: _t->on_pushButton_clicked(); break;
        case 6: _t->on_rollKp_sliderMoved((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 7: _t->on_rollKd_sliderMoved((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 8: _t->on_pitchKp_sliderMoved((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 9: _t->on_pitchKd_sliderMoved((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 10: _t->on_altKp_sliderMoved((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 11: _t->on_altKd_sliderMoved((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 12: _t->on_YawKp_sliderMoved((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 13: _t->on_YawKd_sliderMoved((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 14: _t->timeChanged(); break;
        default: ;
        }
    }
}

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow.data,
      qt_meta_data_MainWindow,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow.stringdata))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 15)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 15;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 15)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 15;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
