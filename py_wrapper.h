#ifndef PY_WRAPPER_H
#define PY_WRAPPER_H

#include <robot.h>

class PyBot : public Driver {
 public:
  void init(int *index);
  CarControl drive(CarState &cs);
  void onShutdown();
};

#endif  // PY_WRAPPER_H