/****************************************************************************
** Meta object code from reading C++ file 'weaver_canvas.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.13)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../src/ros_weaver/include/ros_weaver/canvas/weaver_canvas.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'weaver_canvas.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.13. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_ros_weaver__WeaverCanvas_t {
    QByteArrayData data[18];
    char stringdata0[234];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ros_weaver__WeaverCanvas_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ros_weaver__WeaverCanvas_t qt_meta_stringdata_ros_weaver__WeaverCanvas = {
    {
QT_MOC_LITERAL(0, 0, 24), // "ros_weaver::WeaverCanvas"
QT_MOC_LITERAL(1, 25, 13), // "blockSelected"
QT_MOC_LITERAL(2, 39, 0), // ""
QT_MOC_LITERAL(3, 40, 13), // "PackageBlock*"
QT_MOC_LITERAL(4, 54, 5), // "block"
QT_MOC_LITERAL(5, 60, 18), // "blockDoubleClicked"
QT_MOC_LITERAL(6, 79, 17), // "connectionCreated"
QT_MOC_LITERAL(7, 97, 15), // "ConnectionLine*"
QT_MOC_LITERAL(8, 113, 10), // "connection"
QT_MOC_LITERAL(9, 124, 12), // "groupCreated"
QT_MOC_LITERAL(10, 137, 10), // "NodeGroup*"
QT_MOC_LITERAL(11, 148, 5), // "group"
QT_MOC_LITERAL(12, 154, 13), // "canvasCleared"
QT_MOC_LITERAL(13, 168, 12), // "onPinHovered"
QT_MOC_LITERAL(14, 181, 8), // "pinIndex"
QT_MOC_LITERAL(15, 190, 8), // "isOutput"
QT_MOC_LITERAL(16, 199, 14), // "onPinUnhovered"
QT_MOC_LITERAL(17, 214, 19) // "onBlockMoveFinished"

    },
    "ros_weaver::WeaverCanvas\0blockSelected\0"
    "\0PackageBlock*\0block\0blockDoubleClicked\0"
    "connectionCreated\0ConnectionLine*\0"
    "connection\0groupCreated\0NodeGroup*\0"
    "group\0canvasCleared\0onPinHovered\0"
    "pinIndex\0isOutput\0onPinUnhovered\0"
    "onBlockMoveFinished"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ros_weaver__WeaverCanvas[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       5,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   54,    2, 0x06 /* Public */,
       5,    1,   57,    2, 0x06 /* Public */,
       6,    1,   60,    2, 0x06 /* Public */,
       9,    1,   63,    2, 0x06 /* Public */,
      12,    0,   66,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      13,    3,   67,    2, 0x08 /* Private */,
      16,    1,   74,    2, 0x08 /* Private */,
      17,    1,   77,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 7,    8,
    QMetaType::Void, 0x80000000 | 10,   11,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3, QMetaType::Int, QMetaType::Bool,    4,   14,   15,
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 3,    4,

       0        // eod
};

void ros_weaver::WeaverCanvas::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<WeaverCanvas *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->blockSelected((*reinterpret_cast< PackageBlock*(*)>(_a[1]))); break;
        case 1: _t->blockDoubleClicked((*reinterpret_cast< PackageBlock*(*)>(_a[1]))); break;
        case 2: _t->connectionCreated((*reinterpret_cast< ConnectionLine*(*)>(_a[1]))); break;
        case 3: _t->groupCreated((*reinterpret_cast< NodeGroup*(*)>(_a[1]))); break;
        case 4: _t->canvasCleared(); break;
        case 5: _t->onPinHovered((*reinterpret_cast< PackageBlock*(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< bool(*)>(_a[3]))); break;
        case 6: _t->onPinUnhovered((*reinterpret_cast< PackageBlock*(*)>(_a[1]))); break;
        case 7: _t->onBlockMoveFinished((*reinterpret_cast< PackageBlock*(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (WeaverCanvas::*)(PackageBlock * );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&WeaverCanvas::blockSelected)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (WeaverCanvas::*)(PackageBlock * );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&WeaverCanvas::blockDoubleClicked)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (WeaverCanvas::*)(ConnectionLine * );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&WeaverCanvas::connectionCreated)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (WeaverCanvas::*)(NodeGroup * );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&WeaverCanvas::groupCreated)) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (WeaverCanvas::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&WeaverCanvas::canvasCleared)) {
                *result = 4;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject ros_weaver::WeaverCanvas::staticMetaObject = { {
    QMetaObject::SuperData::link<QGraphicsView::staticMetaObject>(),
    qt_meta_stringdata_ros_weaver__WeaverCanvas.data,
    qt_meta_data_ros_weaver__WeaverCanvas,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *ros_weaver::WeaverCanvas::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ros_weaver::WeaverCanvas::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_ros_weaver__WeaverCanvas.stringdata0))
        return static_cast<void*>(this);
    return QGraphicsView::qt_metacast(_clname);
}

int ros_weaver::WeaverCanvas::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QGraphicsView::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 8)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 8;
    }
    return _id;
}

// SIGNAL 0
void ros_weaver::WeaverCanvas::blockSelected(PackageBlock * _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void ros_weaver::WeaverCanvas::blockDoubleClicked(PackageBlock * _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void ros_weaver::WeaverCanvas::connectionCreated(ConnectionLine * _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void ros_weaver::WeaverCanvas::groupCreated(NodeGroup * _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void ros_weaver::WeaverCanvas::canvasCleared()
{
    QMetaObject::activate(this, &staticMetaObject, 4, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
