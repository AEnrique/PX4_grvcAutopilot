#ifndef STRUCT_VALUES_H
#define STRUCT_VALUES_H

typedef struct{
  float roll_s;
  float pitch_s;
  float yaw_s;
  float quat[4];
  float att_control[3];
  uint64_t timestamp;
} attitudeValues;

typedef struct{
  float lat;
  float lon;
  float alt;
  float vn;
  float ve;
  float vd;
  float pressure_alt;
  uint64_t timestamp;
} global_pos_values;

typedef struct{
  float x;
  float y;
  float z;
  float vx;
  float vy;
  float vz;
  uint64_t timestamp;
} local_pos_values;
#endif
