

#ifndef _DATA_STRUCTURE_H_
#define _DATA_STRUCTURE_H_

// #include "fixed_point.h"
#include "helper_3dmath.h"

typedef struct {
  // long long isn't recommended. it seems that the number is doing something strange 
  unsigned int sample;

  VectorFloat accel,
              gyro,
              mag;

  // from FOV of world
  Quaternion q;

} States;


#endif // _DATA_STRUCTURE_H_