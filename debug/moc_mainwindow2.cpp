/****************************************************************************
** Meta object code from reading C++ file 'mainwindow2.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.6.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../mainwindow2.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow2.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.6.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_MainWindow2_t {
    QByteArrayData data[15];
    char stringdata0[326];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainWindow2_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainWindow2_t qt_meta_stringdata_MainWindow2 = {
    {
QT_MOC_LITERAL(0, 0, 11), // "MainWindow2"
QT_MOC_LITERAL(1, 12, 32), // "on_pushButton_GetWorkIfo_clicked"
QT_MOC_LITERAL(2, 45, 0), // ""
QT_MOC_LITERAL(3, 46, 20), // "on_cbxMode_activated"
QT_MOC_LITERAL(4, 67, 4), // "arg1"
QT_MOC_LITERAL(5, 72, 21), // "on_cbxState_activated"
QT_MOC_LITERAL(6, 94, 22), // "on_cbxRelate_activated"
QT_MOC_LITERAL(7, 117, 29), // "on_pushButton_goState_clicked"
QT_MOC_LITERAL(8, 147, 32), // "on_pushButton_goRelation_clicked"
QT_MOC_LITERAL(9, 180, 25), // "on_listView_doubleClicked"
QT_MOC_LITERAL(10, 206, 5), // "index"
QT_MOC_LITERAL(11, 212, 26), // "on_pushButton_goto_clicked"
QT_MOC_LITERAL(12, 239, 28), // "on_pushButton_goMode_clicked"
QT_MOC_LITERAL(13, 268, 25), // "on_pushButton_fit_clicked"
QT_MOC_LITERAL(14, 294, 31) // "on_pushButton_circleFit_clicked"

    },
    "MainWindow2\0on_pushButton_GetWorkIfo_clicked\0"
    "\0on_cbxMode_activated\0arg1\0"
    "on_cbxState_activated\0on_cbxRelate_activated\0"
    "on_pushButton_goState_clicked\0"
    "on_pushButton_goRelation_clicked\0"
    "on_listView_doubleClicked\0index\0"
    "on_pushButton_goto_clicked\0"
    "on_pushButton_goMode_clicked\0"
    "on_pushButton_fit_clicked\0"
    "on_pushButton_circleFit_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow2[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      11,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   69,    2, 0x08 /* Private */,
       3,    1,   70,    2, 0x08 /* Private */,
       5,    1,   73,    2, 0x08 /* Private */,
       6,    1,   76,    2, 0x08 /* Private */,
       7,    0,   79,    2, 0x08 /* Private */,
       8,    0,   80,    2, 0x08 /* Private */,
       9,    1,   81,    2, 0x08 /* Private */,
      11,    0,   84,    2, 0x08 /* Private */,
      12,    0,   85,    2, 0x08 /* Private */,
      13,    0,   86,    2, 0x08 /* Private */,
      14,    0,   87,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    4,
    QMetaType::Void, QMetaType::QString,    4,
    QMetaType::Void, QMetaType::QString,    4,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QModelIndex,   10,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void MainWindow2::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MainWindow2 *_t = static_cast<MainWindow2 *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->on_pushButton_GetWorkIfo_clicked(); break;
        case 1: _t->on_cbxMode_activated((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 2: _t->on_cbxState_activated((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 3: _t->on_cbxRelate_activated((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 4: _t->on_pushButton_goState_clicked(); break;
        case 5: _t->on_pushButton_goRelation_clicked(); break;
        case 6: _t->on_listView_doubleClicked((*reinterpret_cast< const QModelIndex(*)>(_a[1]))); break;
        case 7: _t->on_pushButton_goto_clicked(); break;
        case 8: _t->on_pushButton_goMode_clicked(); break;
        case 9: _t->on_pushButton_fit_clicked(); break;
        case 10: _t->on_pushButton_circleFit_clicked(); break;
        default: ;
        }
    }
}

const QMetaObject MainWindow2::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow2.data,
      qt_meta_data_MainWindow2,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *MainWindow2::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow2::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow2.stringdata0))
        return static_cast<void*>(const_cast< MainWindow2*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow2::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 11)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 11;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 11)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 11;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
