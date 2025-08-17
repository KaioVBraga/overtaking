#include "py_wrapper.h"

#include <Python.h>

#include <iostream>

// --- Global Python Objects ---
PyObject *pModule, *pFunc_drive;

// --- Bot Functions ---

// Called once at the start of the race
void PyBot::init(int *index) {
  // Initialize the Python Interpreter
  Py_Initialize();

  // Add the current directory to Python's path to find our script
  PyObject *sys = PyImport_ImportModule("sys");
  PyObject *path = PyObject_GetAttrString(sys, "path");
  PyList_Append(path, PyUnicode_FromString("."));

  // Load our Python script as a module
  pModule = PyImport_ImportModule("my_bot");
  if (!pModule) {
    PyErr_Print();
    std::cerr << "Error: Could not import python module 'my_bot.py'.\n";
    return;
  }

  // Get a reference to the drive function
  pFunc_drive = PyObject_GetAttrString(pModule, "drive");
  if (!pFunc_drive || !PyCallable_Check(pFunc_drive)) {
    PyErr_Print();
    std::cerr << "Error: Could not find function 'drive' in 'my_bot.py'.\n";
    return;
  }
}

// Called for every simulation step
CarControl PyBot::drive(CarState &cs) {
  CarControl cc;  // The command to return

  // 1. Create a Python dictionary from the CarState object
  PyObject *pStateDict = PyDict_New();
  PyDict_SetItemString(pStateDict, "speed_x", PyFloat_FromDouble(cs.getSpeedX()));
  PyDict_SetItemString(pStateDict, "angle", PyFloat_FromDouble(cs.getAngle()));
  PyDict_SetItemString(pStateDict, "track_pos", PyFloat_FromDouble(cs.getTrackPos()));
  // Add track edge sensors
  PyObject *pTrackSensors = PyList_New(19);
  for (int i = 0; i < 19; ++i) {
    PyList_SetItem(pTrackSensors, i, PyFloat_FromDouble(cs.getTrack(i)));
  }
  PyDict_SetItemString(pStateDict, "track_sensors", pTrackSensors);

  // 2. Call the Python 'drive' function with the state dictionary
  PyObject *pArgs = PyTuple_Pack(1, pStateDict);
  PyObject *pResult = PyObject_CallObject(pFunc_drive, pArgs);

  Py_DECREF(pStateDict);
  Py_DECREF(pArgs);

  // 3. Parse the returned dictionary into our CarControl object
  if (pResult != NULL) {
    cc.setAccel(PyFloat_AsDouble(PyDict_GetItemString(pResult, "accel")));
    cc.setBrake(PyFloat_AsDouble(PyDict_GetItemString(pResult, "brake")));
    cc.setSteer(PyFloat_AsDouble(PyDict_GetItemString(pResult, "steer")));
    Py_DECREF(pResult);
  } else {
    PyErr_Print();
    std::cerr << "Error: Python 'drive' function call failed.\n";
  }

  return cc;
}

// Called when the race ends
void PyBot::onShutdown() {
  // Clean up Python objects
  Py_XDECREF(pFunc_drive);
  Py_XDECREF(pModule);

  // Shutdown the Python Interpreter
  Py_Finalize();
}