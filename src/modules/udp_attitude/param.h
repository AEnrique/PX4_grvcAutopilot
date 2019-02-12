#ifndef ATTITUDE_H
#define ATTITUDE_H

#define BUFLEN 512  //Max length of buffer
#define PORT 20000   //The port on which to send data
//#define SERVER "192.168.1.30"
//#define SERVER "192.168.43.129"
#define SERVER "10.0.0.201"
//#define SERVER "10.0.0.31"


// typedef struct{
// float roll_s;
// float pitch_s;
// float yaw_s;
// float quat[4];
// float att_control[3];
// }attitudeValues;


typedef struct{
float roll_s;
float pitch_s;
float yaw_s;
float quat[4];
float accel_x;
float accel_y;
float accel_z;
float x;
float y;
float z;
int timestamp;
}attitudeValues;



#endif
