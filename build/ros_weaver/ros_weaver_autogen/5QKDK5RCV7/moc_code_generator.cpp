/****************************************************************************
** Meta object code from reading C++ file 'code_generator.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.13)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../src/ros_weaver/include/ros_weaver/core/code_generator.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'code_generator.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.13. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_ros_weaver__CodeGenerator_t {
    QByteArrayData data[10];
    char stringdata0[130];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ros_weaver__CodeGenerator_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ros_weaver__CodeGenerator_t qt_meta_stringdata_ros_weaver__CodeGenerator = {
    {
QT_MOC_LITERAL(0, 0, 25), // "ros_weaver::CodeGenerator"
QT_MOC_LITERAL(1, 26, 17), // "generationStarted"
QT_MOC_LITERAL(2, 44, 0), // ""
QT_MOC_LITERAL(3, 45, 18), // "generationProgress"
QT_MOC_LITERAL(4, 64, 7), // "percent"
QT_MOC_LITERAL(5, 72, 7), // "message"
QT_MOC_LITERAL(6, 80, 18), // "generationFinished"
QT_MOC_LITERAL(7, 99, 7), // "success"
QT_MOC_LITERAL(8, 107, 13), // "fileGenerated"
QT_MOC_LITERAL(9, 121, 8) // "filePath"

    },
    "ros_weaver::CodeGenerator\0generationStarted\0"
    "\0generationProgress\0percent\0message\0"
    "generationFinished\0success\0fileGenerated\0"
    "filePath"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ros_weaver__CodeGenerator[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   34,    2, 0x06 /* Public */,
       3,    2,   35,    2, 0x06 /* Public */,
       6,    1,   40,    2, 0x06 /* Public */,
       8,    1,   43,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int, QMetaType::QString,    4,    5,
    QMetaType::Void, QMetaType::Bool,    7,
    QMetaType::Void, QMetaType::QString,    9,

       0        // eod
};

void ros_weaver::CodeGenerator::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<CodeGenerator *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->generationStarted(); break;
        case 1: _t->generationProgress((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< const QString(*)>(_a[2]))); break;
        case 2: _t->generationFinished((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->fileGenerated((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (CodeGenerator::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&CodeGenerator::generationStarted)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (CodeGenerator::*)(int , const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&CodeGenerator::generationProgress)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (CodeGenerator::*)(bool );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&CodeGenerator::generationFinished)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (CodeGenerator::*)(const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&CodeGenerator::fileGenerated)) {
                *result = 3;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject ros_weaver::CodeGenerator::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_ros_weaver__CodeGenerator.data,
    qt_meta_data_ros_weaver__CodeGenerator,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *ros_weaver::CodeGenerator::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ros_weaver::CodeGenerator::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_ros_weaver__CodeGenerator.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int ros_weaver::CodeGenerator::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void ros_weaver::CodeGenerator::generationStarted()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void ros_weaver::CodeGenerator::generationProgress(int _t1, const QString & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void ros_weaver::CodeGenerator::generationFinished(bool _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void ros_weaver::CodeGenerator::fileGenerated(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
