#include <Python.h>
#include <robot.h>

#include <iostream>

// --- Global Python Objects ---
static PyObject *pModule, *pFunc_drive;

// --- C-style functions that will be our bot's callbacks ---

// This is the main driving function called by TORCS every simulation step
static void pyDrive(int index, tCarElt *car, tSituation *s) {
  // 1. Create a Python dictionary from the car's state (tCarElt)
  PyObject *pStateDict = PyDict_New();
  //   PyDict_SetItemString(pStateDict, "speed_x", PyFloat_FromDouble(car->priv.speed.x));
  PyDict_SetItemString(pStateDict, "speed_x", PyFloat_FromDouble(car->pub.speed));
  PyDict_SetItemString(pStateDict, "angle", PyFloat_FromDouble(car->pub.angle));
  PyDict_SetItemString(pStateDict, "track_pos", PyFloat_FromDouble(car->pub.trkPos.toMiddle));

  // Add track edge sensors
  PyObject *pTrackSensors = PyList_New(19);
  for (int i = 0; i < 19; ++i) {
    PyList_SetItem(pTrackSensors, i, PyFloat_FromDouble(car->pub.sensors.track[i]));
  }
  PyDict_SetItemString(pStateDict, "track_sensors", pTrackSensors);

  // 2. Call the Python 'drive' function
  PyObject *pArgs = PyTuple_Pack(1, pStateDict);
  PyObject *pResult = PyObject_CallObject(pFunc_drive, pArgs);
  Py_DECREF(pStateDict);
  Py_DECREF(pArgs);

  // 3. Parse the returned dictionary and apply commands to the car
  if (pResult != NULL) {
    car->ctrl.accelCmd = PyFloat_AsDouble(PyDict_GetItemString(pResult, "accel"));
    car->ctrl.brakeCmd = PyFloat_AsDouble(PyDict_GetItemString(pResult, "brake"));
    car->ctrl.steer = PyFloat_AsDouble(PyDict_GetItemString(pResult, "steer"));
    Py_DECREF(pResult);
  } else {
    PyErr_Print();
    std::cerr << "Error: Python 'drive' function call failed." << std::endl;
  }

  // Set other commands for simplicity
  car->ctrl.gear = 1;
  car->ctrl.clutchCmd = 0;
}

// Called once for each new race
static void pyNewRace(int index, tCarElt *car, tSituation *s) {
  Py_Initialize();
  PyObject *sys = PyImport_ImportModule("sys");
  PyObject *path = PyObject_GetAttrString(sys, "path");
  PyList_Append(path, PyUnicode_FromString("."));

  pModule = PyImport_ImportModule("my_bot");
  if (!pModule) {
    PyErr_Print();
    std::cerr << "Error: Could not import python module 'my_bot.py'." << std::endl;
    return;
  }

  pFunc_drive = PyObject_GetAttrString(pModule, "drive");
  if (!pFunc_drive || !PyCallable_Check(pFunc_drive)) {
    PyErr_Print();
    std::cerr << "Error: Could not find function 'drive' in 'my_bot.py'." << std::endl;
    return;
  }
}

// Called once when the bot is unloaded
static void pyShutdown(int index) {
  Py_XDECREF(pFunc_drive);
  Py_XDECREF(pModule);
  Py_Finalize();
}

// --- TORCS Interface Setup ---

// This struct holds the function pointers that TORCS will call
static tRobotItf robotItf =
    {
        (tfRbNewTrack)NULL,
        (tfRbNewRace)pyNewRace,
        (tfRbEndRace)NULL,
        (tfRbDrive)pyDrive,
        (tfRbPitCmd)NULL,
        (tfRbShutdown)pyShutdown,
        0  // index
};

// This is the function that TORCS looks for in the .so file
extern "C" tRobotItf *robotGetItf(int index) {
  robotItf.index = index;
  return &robotItf;
}
