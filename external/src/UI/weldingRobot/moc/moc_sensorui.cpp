/****************************************************************************
** Meta object code from reading C++ file 'sensorui.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../base/sensorui.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'sensorui.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_MyData_t {
    QByteArrayData data[1];
    char stringdata0[7];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MyData_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MyData_t qt_meta_stringdata_MyData = {
    {
QT_MOC_LITERAL(0, 0, 6) // "MyData"

    },
    "MyData"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MyData[] = {

 // content:
       7,       // revision
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

void MyData::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

const QMetaObject MyData::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_MyData.data,
      qt_meta_data_MyData,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *MyData::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MyData::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_MyData.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int MyData::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    return _id;
}
struct qt_meta_stringdata_SensorUI_t {
    QByteArrayData data[29];
    char stringdata0[455];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_SensorUI_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_SensorUI_t qt_meta_stringdata_SensorUI = {
    {
QT_MOC_LITERAL(0, 0, 8), // "SensorUI"
QT_MOC_LITERAL(1, 9, 9), // "getRecord"
QT_MOC_LITERAL(2, 19, 0), // ""
QT_MOC_LITERAL(3, 20, 1), // "r"
QT_MOC_LITERAL(4, 22, 5), // "motor"
QT_MOC_LITERAL(5, 28, 21), // "on_openCamera_clicked"
QT_MOC_LITERAL(6, 50, 22), // "on_takePicture_clicked"
QT_MOC_LITERAL(7, 73, 11), // "showPicture"
QT_MOC_LITERAL(8, 85, 7), // "showPic"
QT_MOC_LITERAL(9, 93, 27), // "on_autoSaveImg_stateChanged"
QT_MOC_LITERAL(10, 121, 4), // "arg1"
QT_MOC_LITERAL(11, 126, 25), // "on_reconnectLaser_clicked"
QT_MOC_LITERAL(12, 152, 22), // "on_alwaysLight_clicked"
QT_MOC_LITERAL(13, 175, 21), // "on_laserClose_clicked"
QT_MOC_LITERAL(14, 197, 15), // "mousePressEvent"
QT_MOC_LITERAL(15, 213, 12), // "QMouseEvent*"
QT_MOC_LITERAL(16, 226, 5), // "event"
QT_MOC_LITERAL(17, 232, 14), // "mouseMoveEvent"
QT_MOC_LITERAL(18, 247, 17), // "mouseReleaseEvent"
QT_MOC_LITERAL(19, 265, 13), // "keyPressEvent"
QT_MOC_LITERAL(20, 279, 10), // "QKeyEvent*"
QT_MOC_LITERAL(21, 290, 10), // "paintEvent"
QT_MOC_LITERAL(22, 301, 12), // "QPaintEvent*"
QT_MOC_LITERAL(23, 314, 20), // "on_btnMoveTo_clicked"
QT_MOC_LITERAL(24, 335, 18), // "on_btnSave_clicked"
QT_MOC_LITERAL(25, 354, 18), // "on_btnHome_clicked"
QT_MOC_LITERAL(26, 373, 30), // "on_seamList_currentTextChanged"
QT_MOC_LITERAL(27, 404, 31), // "on_btnObject_currentTextChanged"
QT_MOC_LITERAL(28, 436, 18) // "on_openPic_clicked"

    },
    "SensorUI\0getRecord\0\0r\0motor\0"
    "on_openCamera_clicked\0on_takePicture_clicked\0"
    "showPicture\0showPic\0on_autoSaveImg_stateChanged\0"
    "arg1\0on_reconnectLaser_clicked\0"
    "on_alwaysLight_clicked\0on_laserClose_clicked\0"
    "mousePressEvent\0QMouseEvent*\0event\0"
    "mouseMoveEvent\0mouseReleaseEvent\0"
    "keyPressEvent\0QKeyEvent*\0paintEvent\0"
    "QPaintEvent*\0on_btnMoveTo_clicked\0"
    "on_btnSave_clicked\0on_btnHome_clicked\0"
    "on_seamList_currentTextChanged\0"
    "on_btnObject_currentTextChanged\0"
    "on_openPic_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SensorUI[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      21,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,  119,    2, 0x06 /* Public */,
       4,    0,  122,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       5,    0,  123,    2, 0x08 /* Private */,
       6,    0,  124,    2, 0x08 /* Private */,
       7,    0,  125,    2, 0x08 /* Private */,
       8,    0,  126,    2, 0x08 /* Private */,
       9,    1,  127,    2, 0x08 /* Private */,
      11,    0,  130,    2, 0x08 /* Private */,
      12,    0,  131,    2, 0x08 /* Private */,
      13,    0,  132,    2, 0x08 /* Private */,
      14,    1,  133,    2, 0x08 /* Private */,
      17,    1,  136,    2, 0x08 /* Private */,
      18,    1,  139,    2, 0x08 /* Private */,
      19,    1,  142,    2, 0x08 /* Private */,
      21,    1,  145,    2, 0x08 /* Private */,
      23,    0,  148,    2, 0x08 /* Private */,
      24,    0,  149,    2, 0x08 /* Private */,
      25,    0,  150,    2, 0x08 /* Private */,
      26,    1,  151,    2, 0x08 /* Private */,
      27,    1,  154,    2, 0x08 /* Private */,
      28,    0,  157,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   10,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 15,   16,
    QMetaType::Void, 0x80000000 | 15,   16,
    QMetaType::Void, 0x80000000 | 15,   16,
    QMetaType::Void, 0x80000000 | 20,   16,
    QMetaType::Void, 0x80000000 | 22,   16,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   10,
    QMetaType::Void, QMetaType::QString,   10,
    QMetaType::Void,

       0        // eod
};

void SensorUI::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        SensorUI *_t = static_cast<SensorUI *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->getRecord((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 1: _t->motor(); break;
        case 2: _t->on_openCamera_clicked(); break;
        case 3: _t->on_takePicture_clicked(); break;
        case 4: _t->showPicture(); break;
        case 5: _t->showPic(); break;
        case 6: _t->on_autoSaveImg_stateChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 7: _t->on_reconnectLaser_clicked(); break;
        case 8: _t->on_alwaysLight_clicked(); break;
        case 9: _t->on_laserClose_clicked(); break;
        case 10: _t->mousePressEvent((*reinterpret_cast< QMouseEvent*(*)>(_a[1]))); break;
        case 11: _t->mouseMoveEvent((*reinterpret_cast< QMouseEvent*(*)>(_a[1]))); break;
        case 12: _t->mouseReleaseEvent((*reinterpret_cast< QMouseEvent*(*)>(_a[1]))); break;
        case 13: _t->keyPressEvent((*reinterpret_cast< QKeyEvent*(*)>(_a[1]))); break;
        case 14: _t->paintEvent((*reinterpret_cast< QPaintEvent*(*)>(_a[1]))); break;
        case 15: _t->on_btnMoveTo_clicked(); break;
        case 16: _t->on_btnSave_clicked(); break;
        case 17: _t->on_btnHome_clicked(); break;
        case 18: _t->on_seamList_currentTextChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 19: _t->on_btnObject_currentTextChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 20: _t->on_openPic_clicked(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            typedef void (SensorUI::*_t)(QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SensorUI::getRecord)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (SensorUI::*_t)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SensorUI::motor)) {
                *result = 1;
                return;
            }
        }
    }
}

const QMetaObject SensorUI::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_SensorUI.data,
      qt_meta_data_SensorUI,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *SensorUI::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SensorUI::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_SensorUI.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int SensorUI::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 21)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 21;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 21)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 21;
    }
    return _id;
}

// SIGNAL 0
void SensorUI::getRecord(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void SensorUI::motor()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
