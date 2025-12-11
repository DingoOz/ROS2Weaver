/****************************************************************************
** Meta object code from reading C++ file 'param_dashboard.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.13)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../src/ros_weaver/include/ros_weaver/widgets/param_dashboard.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'param_dashboard.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.13. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_ros_weaver__ParamDashboard_t {
    QByteArrayData data[13];
    char stringdata0[169];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ros_weaver__ParamDashboard_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ros_weaver__ParamDashboard_t qt_meta_stringdata_ros_weaver__ParamDashboard = {
    {
QT_MOC_LITERAL(0, 0, 26), // "ros_weaver::ParamDashboard"
QT_MOC_LITERAL(1, 27, 16), // "parameterChanged"
QT_MOC_LITERAL(2, 44, 0), // ""
QT_MOC_LITERAL(3, 45, 4), // "name"
QT_MOC_LITERAL(4, 50, 5), // "value"
QT_MOC_LITERAL(5, 56, 18), // "parametersModified"
QT_MOC_LITERAL(6, 75, 13), // "onItemChanged"
QT_MOC_LITERAL(7, 89, 16), // "QTreeWidgetItem*"
QT_MOC_LITERAL(8, 106, 4), // "item"
QT_MOC_LITERAL(9, 111, 6), // "column"
QT_MOC_LITERAL(10, 118, 14), // "onAddParameter"
QT_MOC_LITERAL(11, 133, 17), // "onRemoveParameter"
QT_MOC_LITERAL(12, 151, 17) // "onResetToDefaults"

    },
    "ros_weaver::ParamDashboard\0parameterChanged\0"
    "\0name\0value\0parametersModified\0"
    "onItemChanged\0QTreeWidgetItem*\0item\0"
    "column\0onAddParameter\0onRemoveParameter\0"
    "onResetToDefaults"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ros_weaver__ParamDashboard[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    2,   44,    2, 0x06 /* Public */,
       5,    0,   49,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       6,    2,   50,    2, 0x08 /* Private */,
      10,    0,   55,    2, 0x08 /* Private */,
      11,    0,   56,    2, 0x08 /* Private */,
      12,    0,   57,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString, QMetaType::QVariant,    3,    4,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 7, QMetaType::Int,    8,    9,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void ros_weaver::ParamDashboard::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<ParamDashboard *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->parameterChanged((*reinterpret_cast< const QString(*)>(_a[1])),(*reinterpret_cast< const QVariant(*)>(_a[2]))); break;
        case 1: _t->parametersModified(); break;
        case 2: _t->onItemChanged((*reinterpret_cast< QTreeWidgetItem*(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 3: _t->onAddParameter(); break;
        case 4: _t->onRemoveParameter(); break;
        case 5: _t->onResetToDefaults(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (ParamDashboard::*)(const QString & , const QVariant & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ParamDashboard::parameterChanged)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (ParamDashboard::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ParamDashboard::parametersModified)) {
                *result = 1;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject ros_weaver::ParamDashboard::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_ros_weaver__ParamDashboard.data,
    qt_meta_data_ros_weaver__ParamDashboard,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *ros_weaver::ParamDashboard::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ros_weaver::ParamDashboard::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_ros_weaver__ParamDashboard.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int ros_weaver::ParamDashboard::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 6)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 6;
    }
    return _id;
}

// SIGNAL 0
void ros_weaver::ParamDashboard::parameterChanged(const QString & _t1, const QVariant & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void ros_weaver::ParamDashboard::parametersModified()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
