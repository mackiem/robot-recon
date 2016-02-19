/****************************************************************************
** Meta object code from reading C++ file 'filteredstructlight.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.3.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../filteredstructlight.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'filteredstructlight.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.3.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_FilteredStructLight_t {
    QByteArrayData data[8];
    char stringdata[133];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_FilteredStructLight_t, stringdata) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_FilteredStructLight_t qt_meta_stringdata_FilteredStructLight = {
    {
QT_MOC_LITERAL(0, 0, 19),
QT_MOC_LITERAL(1, 20, 13),
QT_MOC_LITERAL(2, 34, 0),
QT_MOC_LITERAL(3, 35, 8),
QT_MOC_LITERAL(4, 44, 29),
QT_MOC_LITERAL(5, 74, 22),
QT_MOC_LITERAL(6, 97, 24),
QT_MOC_LITERAL(7, 122, 10)
    },
    "FilteredStructLight\0update_images\0\0"
    "frame_no\0start_reconstruction_sequence\0"
    "handle_frame_filenames\0std::vector<std::string>\0"
    "image_list"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_FilteredStructLight[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   29,    2, 0x08 /* Private */,
       4,    0,   32,    2, 0x0a /* Public */,
       5,    1,   33,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 6,    7,

       0        // eod
};

void FilteredStructLight::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        FilteredStructLight *_t = static_cast<FilteredStructLight *>(_o);
        switch (_id) {
        case 0: _t->update_images((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->start_reconstruction_sequence(); break;
        case 2: _t->handle_frame_filenames((*reinterpret_cast< std::vector<std::string>(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject FilteredStructLight::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_FilteredStructLight.data,
      qt_meta_data_FilteredStructLight,  qt_static_metacall, 0, 0}
};


const QMetaObject *FilteredStructLight::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *FilteredStructLight::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_FilteredStructLight.stringdata))
        return static_cast<void*>(const_cast< FilteredStructLight*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int FilteredStructLight::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 3)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 3;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
