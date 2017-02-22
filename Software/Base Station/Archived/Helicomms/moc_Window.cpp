/****************************************************************************
** Meta object code from reading C++ file 'Window.h'
**
** Created: Mon Jul 7 08:14:59 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "Window.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'Window.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_Window[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      19,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
       8,    7,    7,    7, 0x0a,
      22,    7,    7,    7, 0x0a,
      35,    7,    7,    7, 0x0a,
      51,    7,    7,    7, 0x0a,
      73,    7,    7,    7, 0x0a,
      92,   86,    7,    7, 0x0a,
     117,  112,    7,    7, 0x0a,
     143,  141,    7,    7, 0x0a,
     170,  141,    7,    7, 0x0a,
     188,  141,    7,    7, 0x0a,
     204,  141,    7,    7, 0x0a,
     220,  141,    7,    7, 0x0a,
     244,  237,    7,    7, 0x0a,
     267,    7,    7,    7, 0x0a,
     286,    7,    7,    7, 0x0a,
     304,    7,    7,    7, 0x0a,
     317,  141,    7,    7, 0x0a,
     336,  141,    7,    7, 0x0a,
     355,  141,    7,    7, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_Window[] = {
    "Window\0\0toggleComms()\0toggleSend()\0"
    "toggleReceive()\0toggleMotionCapture()\0"
    "updateData()\0index\0onRxRateChange(int)\0"
    "text\0onTxRateChange(QString)\0i\0"
    "onImageProcessChanged(int)\0onAutoChange(int)\0"
    "autoSample(int)\0syncSample(int)\0"
    "onGPSChange(int)\0image1\0processingDone(QImage)\0"
    "dataRequestTimer()\0startProcessing()\0"
    "killServer()\0onRLEDChanged(int)\0"
    "onGLEDChanged(int)\0onBLEDChanged(int)\0"
};

void Window::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        Window *_t = static_cast<Window *>(_o);
        switch (_id) {
        case 0: _t->toggleComms(); break;
        case 1: _t->toggleSend(); break;
        case 2: _t->toggleReceive(); break;
        case 3: _t->toggleMotionCapture(); break;
        case 4: _t->updateData(); break;
        case 5: _t->onRxRateChange((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: _t->onTxRateChange((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 7: _t->onImageProcessChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 8: _t->onAutoChange((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 9: _t->autoSample((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 10: _t->syncSample((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 11: _t->onGPSChange((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 12: _t->processingDone((*reinterpret_cast< QImage(*)>(_a[1]))); break;
        case 13: _t->dataRequestTimer(); break;
        case 14: _t->startProcessing(); break;
        case 15: _t->killServer(); break;
        case 16: _t->onRLEDChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 17: _t->onGLEDChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 18: _t->onBLEDChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData Window::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject Window::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_Window,
      qt_meta_data_Window, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &Window::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *Window::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *Window::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_Window))
        return static_cast<void*>(const_cast< Window*>(this));
    return QDialog::qt_metacast(_clname);
}

int Window::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 19)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 19;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
