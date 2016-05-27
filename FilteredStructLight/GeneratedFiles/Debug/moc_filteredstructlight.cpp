/****************************************************************************
** Meta object code from reading C++ file 'filteredstructlight.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../filteredstructlight.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'filteredstructlight.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_QArrayPushButton_t {
    QByteArrayData data[5];
    char stringdata0[55];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_QArrayPushButton_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_QArrayPushButton_t qt_meta_stringdata_QArrayPushButton = {
    {
QT_MOC_LITERAL(0, 0, 16), // "QArrayPushButton"
QT_MOC_LITERAL(1, 17, 15), // "clicked_with_id"
QT_MOC_LITERAL(2, 33, 0), // ""
QT_MOC_LITERAL(3, 34, 2), // "id"
QT_MOC_LITERAL(4, 37, 17) // "intercept_clicked"

    },
    "QArrayPushButton\0clicked_with_id\0\0id\0"
    "intercept_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_QArrayPushButton[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   24,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       4,    0,   27,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::Int,    3,

 // slots: parameters
    QMetaType::Void,

       0        // eod
};

void QArrayPushButton::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        QArrayPushButton *_t = static_cast<QArrayPushButton *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->clicked_with_id((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->intercept_clicked(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (QArrayPushButton::*_t)(int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&QArrayPushButton::clicked_with_id)) {
                *result = 0;
            }
        }
    }
}

const QMetaObject QArrayPushButton::staticMetaObject = {
    { &QPushButton::staticMetaObject, qt_meta_stringdata_QArrayPushButton.data,
      qt_meta_data_QArrayPushButton,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *QArrayPushButton::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *QArrayPushButton::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_QArrayPushButton.stringdata0))
        return static_cast<void*>(const_cast< QArrayPushButton*>(this));
    return QPushButton::qt_metacast(_clname);
}

int QArrayPushButton::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QPushButton::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 2)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 2;
    }
    return _id;
}

// SIGNAL 0
void QArrayPushButton::clicked_with_id(int _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
struct qt_meta_stringdata_QArrayRadioButton_t {
    QByteArrayData data[5];
    char stringdata0[56];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_QArrayRadioButton_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_QArrayRadioButton_t qt_meta_stringdata_QArrayRadioButton = {
    {
QT_MOC_LITERAL(0, 0, 17), // "QArrayRadioButton"
QT_MOC_LITERAL(1, 18, 15), // "clicked_with_id"
QT_MOC_LITERAL(2, 34, 0), // ""
QT_MOC_LITERAL(3, 35, 2), // "id"
QT_MOC_LITERAL(4, 38, 17) // "intercept_clicked"

    },
    "QArrayRadioButton\0clicked_with_id\0\0"
    "id\0intercept_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_QArrayRadioButton[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   24,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       4,    0,   27,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::Int,    3,

 // slots: parameters
    QMetaType::Void,

       0        // eod
};

void QArrayRadioButton::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        QArrayRadioButton *_t = static_cast<QArrayRadioButton *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->clicked_with_id((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->intercept_clicked(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (QArrayRadioButton::*_t)(int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&QArrayRadioButton::clicked_with_id)) {
                *result = 0;
            }
        }
    }
}

const QMetaObject QArrayRadioButton::staticMetaObject = {
    { &QRadioButton::staticMetaObject, qt_meta_stringdata_QArrayRadioButton.data,
      qt_meta_data_QArrayRadioButton,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *QArrayRadioButton::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *QArrayRadioButton::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_QArrayRadioButton.stringdata0))
        return static_cast<void*>(const_cast< QArrayRadioButton*>(this));
    return QRadioButton::qt_metacast(_clname);
}

int QArrayRadioButton::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QRadioButton::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 2)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 2;
    }
    return _id;
}

// SIGNAL 0
void QArrayRadioButton::clicked_with_id(int _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
struct qt_meta_stringdata_FilteredStructLight_t {
    QByteArrayData data[16];
    char stringdata0[296];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_FilteredStructLight_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_FilteredStructLight_t qt_meta_stringdata_FilteredStructLight = {
    {
QT_MOC_LITERAL(0, 0, 19), // "FilteredStructLight"
QT_MOC_LITERAL(1, 20, 13), // "update_images"
QT_MOC_LITERAL(2, 34, 0), // ""
QT_MOC_LITERAL(3, 35, 8), // "frame_no"
QT_MOC_LITERAL(4, 44, 19), // "save_recon_settings"
QT_MOC_LITERAL(5, 64, 19), // "save_swarm_settings"
QT_MOC_LITERAL(6, 84, 19), // "swarm_conf_filepath"
QT_MOC_LITERAL(7, 104, 19), // "load_swarm_settings"
QT_MOC_LITERAL(8, 124, 26), // "load_swarm_config_settings"
QT_MOC_LITERAL(9, 151, 26), // "save_swarm_config_settings"
QT_MOC_LITERAL(10, 178, 29), // "start_reconstruction_sequence"
QT_MOC_LITERAL(11, 208, 22), // "handle_frame_filenames"
QT_MOC_LITERAL(12, 231, 24), // "std::vector<std::string>"
QT_MOC_LITERAL(13, 256, 10), // "image_list"
QT_MOC_LITERAL(14, 267, 22), // "update_time_step_count"
QT_MOC_LITERAL(15, 290, 5) // "count"

    },
    "FilteredStructLight\0update_images\0\0"
    "frame_no\0save_recon_settings\0"
    "save_swarm_settings\0swarm_conf_filepath\0"
    "load_swarm_settings\0load_swarm_config_settings\0"
    "save_swarm_config_settings\0"
    "start_reconstruction_sequence\0"
    "handle_frame_filenames\0std::vector<std::string>\0"
    "image_list\0update_time_step_count\0"
    "count"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_FilteredStructLight[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       9,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   59,    2, 0x08 /* Private */,
       4,    0,   62,    2, 0x08 /* Private */,
       5,    1,   63,    2, 0x08 /* Private */,
       7,    1,   66,    2, 0x08 /* Private */,
       8,    0,   69,    2, 0x08 /* Private */,
       9,    0,   70,    2, 0x08 /* Private */,
      10,    0,   71,    2, 0x0a /* Public */,
      11,    1,   72,    2, 0x0a /* Public */,
      14,    1,   75,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    6,
    QMetaType::Void, QMetaType::QString,    6,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 12,   13,
    QMetaType::Void, QMetaType::Int,   15,

       0        // eod
};

void FilteredStructLight::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        FilteredStructLight *_t = static_cast<FilteredStructLight *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->update_images((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->save_recon_settings(); break;
        case 2: _t->save_swarm_settings((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 3: _t->load_swarm_settings((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 4: _t->load_swarm_config_settings(); break;
        case 5: _t->save_swarm_config_settings(); break;
        case 6: _t->start_reconstruction_sequence(); break;
        case 7: _t->handle_frame_filenames((*reinterpret_cast< std::vector<std::string>(*)>(_a[1]))); break;
        case 8: _t->update_time_step_count((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject FilteredStructLight::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_FilteredStructLight.data,
      qt_meta_data_FilteredStructLight,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *FilteredStructLight::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *FilteredStructLight::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_FilteredStructLight.stringdata0))
        return static_cast<void*>(const_cast< FilteredStructLight*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int FilteredStructLight::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 9)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 9;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 9)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 9;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
