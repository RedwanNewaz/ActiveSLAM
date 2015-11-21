/****************************************************************************
** Meta object code from reading C++ file 'ros_launch.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.4.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../ros_launch.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'ros_launch.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.4.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_ros_launch_t {
    QByteArrayData data[25];
    char stringdata[337];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ros_launch_t, stringdata) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ros_launch_t qt_meta_stringdata_ros_launch = {
    {
QT_MOC_LITERAL(0, 0, 10), // "ros_launch"
QT_MOC_LITERAL(1, 11, 6), // "ImageQ"
QT_MOC_LITERAL(2, 18, 0), // ""
QT_MOC_LITERAL(3, 19, 17), // "singal_sensor_sub"
QT_MOC_LITERAL(4, 37, 15), // "ardrone_battery"
QT_MOC_LITERAL(5, 53, 11), // "nav_battery"
QT_MOC_LITERAL(6, 65, 8), // "posRobot"
QT_MOC_LITERAL(7, 74, 8), // "position"
QT_MOC_LITERAL(8, 83, 7), // "posRviz"
QT_MOC_LITERAL(9, 91, 4), // "Rviz"
QT_MOC_LITERAL(10, 96, 14), // "sub_pointcloud"
QT_MOC_LITERAL(11, 111, 8), // "localmap"
QT_MOC_LITERAL(12, 120, 15), // "main_pointcloud"
QT_MOC_LITERAL(13, 136, 11), // "sig_tracker"
QT_MOC_LITERAL(14, 148, 16), // "main_sig_tracker"
QT_MOC_LITERAL(15, 165, 21), // "main_Occupancy_signal"
QT_MOC_LITERAL(16, 187, 26), // "nav_msgs::OccupancyGridPtr"
QT_MOC_LITERAL(17, 214, 13), // "sub_occupancy"
QT_MOC_LITERAL(18, 228, 15), // "slot_ros_launch"
QT_MOC_LITERAL(19, 244, 16), // "slot_nav_battery"
QT_MOC_LITERAL(20, 261, 13), // "slot_posRobot"
QT_MOC_LITERAL(21, 275, 12), // "slot_posRviz"
QT_MOC_LITERAL(22, 288, 12), // "slot_tracker"
QT_MOC_LITERAL(23, 301, 15), // "slot_pointcloud"
QT_MOC_LITERAL(24, 317, 19) // "slot_occupancy_grid"

    },
    "ros_launch\0ImageQ\0\0singal_sensor_sub\0"
    "ardrone_battery\0nav_battery\0posRobot\0"
    "position\0posRviz\0Rviz\0sub_pointcloud\0"
    "localmap\0main_pointcloud\0sig_tracker\0"
    "main_sig_tracker\0main_Occupancy_signal\0"
    "nav_msgs::OccupancyGridPtr\0sub_occupancy\0"
    "slot_ros_launch\0slot_nav_battery\0"
    "slot_posRobot\0slot_posRviz\0slot_tracker\0"
    "slot_pointcloud\0slot_occupancy_grid"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ros_launch[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      21,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
      14,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,  119,    2, 0x06 /* Public */,
       3,    1,  122,    2, 0x06 /* Public */,
       4,    1,  125,    2, 0x06 /* Public */,
       5,    1,  128,    2, 0x06 /* Public */,
       6,    1,  131,    2, 0x06 /* Public */,
       7,    1,  134,    2, 0x06 /* Public */,
       8,    1,  137,    2, 0x06 /* Public */,
       9,    1,  140,    2, 0x06 /* Public */,
      10,    1,  143,    2, 0x06 /* Public */,
      12,    1,  146,    2, 0x06 /* Public */,
      13,    1,  149,    2, 0x06 /* Public */,
      14,    1,  152,    2, 0x06 /* Public */,
      15,    1,  155,    2, 0x06 /* Public */,
      17,    1,  158,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      18,    1,  161,    2, 0x0a /* Public */,
      19,    1,  164,    2, 0x0a /* Public */,
      20,    1,  167,    2, 0x0a /* Public */,
      21,    1,  170,    2, 0x0a /* Public */,
      22,    1,  173,    2, 0x0a /* Public */,
      23,    1,  176,    2, 0x0a /* Public */,
      24,    1,  179,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::QImage,    2,
    QMetaType::Void, QMetaType::QImage,    2,
    QMetaType::Void, QMetaType::Double,    2,
    QMetaType::Void, QMetaType::Double,    2,
    QMetaType::Void, QMetaType::QPointF,    2,
    QMetaType::Void, QMetaType::QPointF,    2,
    QMetaType::Void, QMetaType::QPointF,    2,
    QMetaType::Void, QMetaType::QPointF,    2,
    QMetaType::Void, 0x80000000 | 11,    2,
    QMetaType::Void, 0x80000000 | 11,    2,
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void, 0x80000000 | 16,    2,
    QMetaType::Void, 0x80000000 | 16,    2,

 // slots: parameters
    QMetaType::Void, QMetaType::QImage,    2,
    QMetaType::Void, QMetaType::Double,    2,
    QMetaType::Void, QMetaType::QPointF,    2,
    QMetaType::Void, QMetaType::QPointF,    2,
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void, 0x80000000 | 11,    2,
    QMetaType::Void, 0x80000000 | 16,    2,

       0        // eod
};

void ros_launch::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        ros_launch *_t = static_cast<ros_launch *>(_o);
        switch (_id) {
        case 0: _t->ImageQ((*reinterpret_cast< const QImage(*)>(_a[1]))); break;
        case 1: _t->singal_sensor_sub((*reinterpret_cast< const QImage(*)>(_a[1]))); break;
        case 2: _t->ardrone_battery((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 3: _t->nav_battery((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 4: _t->posRobot((*reinterpret_cast< QPointF(*)>(_a[1]))); break;
        case 5: _t->position((*reinterpret_cast< QPointF(*)>(_a[1]))); break;
        case 6: _t->posRviz((*reinterpret_cast< QPointF(*)>(_a[1]))); break;
        case 7: _t->Rviz((*reinterpret_cast< QPointF(*)>(_a[1]))); break;
        case 8: _t->sub_pointcloud((*reinterpret_cast< localmap(*)>(_a[1]))); break;
        case 9: _t->main_pointcloud((*reinterpret_cast< localmap(*)>(_a[1]))); break;
        case 10: _t->sig_tracker((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 11: _t->main_sig_tracker((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 12: _t->main_Occupancy_signal((*reinterpret_cast< const nav_msgs::OccupancyGridPtr(*)>(_a[1]))); break;
        case 13: _t->sub_occupancy((*reinterpret_cast< const nav_msgs::OccupancyGridPtr(*)>(_a[1]))); break;
        case 14: _t->slot_ros_launch((*reinterpret_cast< const QImage(*)>(_a[1]))); break;
        case 15: _t->slot_nav_battery((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 16: _t->slot_posRobot((*reinterpret_cast< QPointF(*)>(_a[1]))); break;
        case 17: _t->slot_posRviz((*reinterpret_cast< QPointF(*)>(_a[1]))); break;
        case 18: _t->slot_tracker((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 19: _t->slot_pointcloud((*reinterpret_cast< localmap(*)>(_a[1]))); break;
        case 20: _t->slot_occupancy_grid((*reinterpret_cast< const nav_msgs::OccupancyGridPtr(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<int*>(_a[0]) = -1; break;
        case 8:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< localmap >(); break;
            }
            break;
        case 9:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< localmap >(); break;
            }
            break;
        case 12:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< nav_msgs::OccupancyGridPtr >(); break;
            }
            break;
        case 13:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< nav_msgs::OccupancyGridPtr >(); break;
            }
            break;
        case 19:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< localmap >(); break;
            }
            break;
        case 20:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< nav_msgs::OccupancyGridPtr >(); break;
            }
            break;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (ros_launch::*_t)(const QImage & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ros_launch::ImageQ)) {
                *result = 0;
            }
        }
        {
            typedef void (ros_launch::*_t)(const QImage & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ros_launch::singal_sensor_sub)) {
                *result = 1;
            }
        }
        {
            typedef void (ros_launch::*_t)(double );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ros_launch::ardrone_battery)) {
                *result = 2;
            }
        }
        {
            typedef void (ros_launch::*_t)(double );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ros_launch::nav_battery)) {
                *result = 3;
            }
        }
        {
            typedef void (ros_launch::*_t)(QPointF );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ros_launch::posRobot)) {
                *result = 4;
            }
        }
        {
            typedef void (ros_launch::*_t)(QPointF );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ros_launch::position)) {
                *result = 5;
            }
        }
        {
            typedef void (ros_launch::*_t)(QPointF );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ros_launch::posRviz)) {
                *result = 6;
            }
        }
        {
            typedef void (ros_launch::*_t)(QPointF );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ros_launch::Rviz)) {
                *result = 7;
            }
        }
        {
            typedef void (ros_launch::*_t)(localmap );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ros_launch::sub_pointcloud)) {
                *result = 8;
            }
        }
        {
            typedef void (ros_launch::*_t)(localmap );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ros_launch::main_pointcloud)) {
                *result = 9;
            }
        }
        {
            typedef void (ros_launch::*_t)(QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ros_launch::sig_tracker)) {
                *result = 10;
            }
        }
        {
            typedef void (ros_launch::*_t)(QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ros_launch::main_sig_tracker)) {
                *result = 11;
            }
        }
        {
            typedef void (ros_launch::*_t)(const nav_msgs::OccupancyGridPtr );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ros_launch::main_Occupancy_signal)) {
                *result = 12;
            }
        }
        {
            typedef void (ros_launch::*_t)(const nav_msgs::OccupancyGridPtr );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ros_launch::sub_occupancy)) {
                *result = 13;
            }
        }
    }
}

const QMetaObject ros_launch::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_ros_launch.data,
      qt_meta_data_ros_launch,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *ros_launch::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ros_launch::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_ros_launch.stringdata))
        return static_cast<void*>(const_cast< ros_launch*>(this));
    return QThread::qt_metacast(_clname);
}

int ros_launch::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 21)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 21;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 21)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 21;
    }
    return _id;
}

// SIGNAL 0
void ros_launch::ImageQ(const QImage & _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void ros_launch::singal_sensor_sub(const QImage & _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void ros_launch::ardrone_battery(double _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void ros_launch::nav_battery(double _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void ros_launch::posRobot(QPointF _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void ros_launch::position(QPointF _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
void ros_launch::posRviz(QPointF _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}

// SIGNAL 7
void ros_launch::Rviz(QPointF _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 7, _a);
}

// SIGNAL 8
void ros_launch::sub_pointcloud(localmap _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 8, _a);
}

// SIGNAL 9
void ros_launch::main_pointcloud(localmap _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 9, _a);
}

// SIGNAL 10
void ros_launch::sig_tracker(QString _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 10, _a);
}

// SIGNAL 11
void ros_launch::main_sig_tracker(QString _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 11, _a);
}

// SIGNAL 12
void ros_launch::main_Occupancy_signal(const nav_msgs::OccupancyGridPtr _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 12, _a);
}

// SIGNAL 13
void ros_launch::sub_occupancy(const nav_msgs::OccupancyGridPtr _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 13, _a);
}
QT_END_MOC_NAMESPACE
