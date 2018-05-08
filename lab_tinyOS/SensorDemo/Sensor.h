#ifndef SENSOR_H
#define SENSOR_H

#define TEMPORARY 0
#define HUMIDITY 1
#define PHOTOVOLTAIC 2

typedef nx_struct SensorMsg{
  nx_uint16_t nodeid;
  nx_uint16_t kind;
  nx_uint16_t data;
} SensorMsg;

enum{
  AM_SENSORMSG = 6,
};

#endif
