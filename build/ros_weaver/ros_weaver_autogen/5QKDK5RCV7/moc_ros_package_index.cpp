/****************************************************************************
** Meta object code from reading C++ file 'ros_package_index.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.13)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../src/ros_weaver/include/ros_weaver/core/ros_package_index.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#include <QtCore/QList>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'ros_package_index.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.13. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_ros_weaver__RosPackageIndex_t {
    QByteArrayData data[16];
    char stringdata0[253];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ros_weaver__RosPackageIndex_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ros_weaver__RosPackageIndex_t qt_meta_stringdata_ros_weaver__RosPackageIndex = {
    {
QT_MOC_LITERAL(0, 0, 27), // "ros_weaver::RosPackageIndex"
QT_MOC_LITERAL(1, 28, 18), // "searchResultsReady"
QT_MOC_LITERAL(2, 47, 0), // ""
QT_MOC_LITERAL(3, 48, 21), // "QList<RosPackageInfo>"
QT_MOC_LITERAL(4, 70, 8), // "packages"
QT_MOC_LITERAL(5, 79, 19), // "packageDetailsReady"
QT_MOC_LITERAL(6, 99, 14), // "RosPackageInfo"
QT_MOC_LITERAL(7, 114, 7), // "package"
QT_MOC_LITERAL(8, 122, 17), // "messageTypesReady"
QT_MOC_LITERAL(9, 140, 21), // "QList<RosMessageInfo>"
QT_MOC_LITERAL(10, 162, 8), // "messages"
QT_MOC_LITERAL(11, 171, 18), // "localPackagesReady"
QT_MOC_LITERAL(12, 190, 11), // "searchError"
QT_MOC_LITERAL(13, 202, 5), // "error"
QT_MOC_LITERAL(14, 208, 21), // "onSearchReplyFinished"
QT_MOC_LITERAL(15, 230, 22) // "onDetailsReplyFinished"

    },
    "ros_weaver::RosPackageIndex\0"
    "searchResultsReady\0\0QList<RosPackageInfo>\0"
    "packages\0packageDetailsReady\0"
    "RosPackageInfo\0package\0messageTypesReady\0"
    "QList<RosMessageInfo>\0messages\0"
    "localPackagesReady\0searchError\0error\0"
    "onSearchReplyFinished\0onDetailsReplyFinished"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ros_weaver__RosPackageIndex[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       5,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   49,    2, 0x06 /* Public */,
       5,    1,   52,    2, 0x06 /* Public */,
       8,    1,   55,    2, 0x06 /* Public */,
      11,    1,   58,    2, 0x06 /* Public */,
      12,    1,   61,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      14,    0,   64,    2, 0x08 /* Private */,
      15,    0,   65,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 6,    7,
    QMetaType::Void, 0x80000000 | 9,   10,
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, QMetaType::QString,   13,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void ros_weaver::RosPackageIndex::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<RosPackageIndex *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->searchResultsReady((*reinterpret_cast< const QList<RosPackageInfo>(*)>(_a[1]))); break;
        case 1: _t->packageDetailsReady((*reinterpret_cast< const RosPackageInfo(*)>(_a[1]))); break;
        case 2: _t->messageTypesReady((*reinterpret_cast< const QList<RosMessageInfo>(*)>(_a[1]))); break;
        case 3: _t->localPackagesReady((*reinterpret_cast< const QList<RosPackageInfo>(*)>(_a[1]))); break;
        case 4: _t->searchError((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 5: _t->onSearchReplyFinished(); break;
        case 6: _t->onDetailsReplyFinished(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (RosPackageIndex::*)(const QList<RosPackageInfo> & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&RosPackageIndex::searchResultsReady)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (RosPackageIndex::*)(const RosPackageInfo & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&RosPackageIndex::packageDetailsReady)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (RosPackageIndex::*)(const QList<RosMessageInfo> & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&RosPackageIndex::messageTypesReady)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (RosPackageIndex::*)(const QList<RosPackageInfo> & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&RosPackageIndex::localPackagesReady)) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (RosPackageIndex::*)(const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&RosPackageIndex::searchError)) {
                *result = 4;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject ros_weaver::RosPackageIndex::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_ros_weaver__RosPackageIndex.data,
    qt_meta_data_ros_weaver__RosPackageIndex,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *ros_weaver::RosPackageIndex::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ros_weaver::RosPackageIndex::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_ros_weaver__RosPackageIndex.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int ros_weaver::RosPackageIndex::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 7;
    }
    return _id;
}

// SIGNAL 0
void ros_weaver::RosPackageIndex::searchResultsReady(const QList<RosPackageInfo> & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void ros_weaver::RosPackageIndex::packageDetailsReady(const RosPackageInfo & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void ros_weaver::RosPackageIndex::messageTypesReady(const QList<RosMessageInfo> & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void ros_weaver::RosPackageIndex::localPackagesReady(const QList<RosPackageInfo> & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void ros_weaver::RosPackageIndex::searchError(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
