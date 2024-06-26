#include "lwip/sockets.h"

typedef struct
{
	  int sockfd;
	  struct sockaddr_in servaddr;
	  struct sockaddr_in cliaddr;
} ParamsUDP;
typedef struct
{
	  int16_t GyroX;
	  int16_t GyroY;
	  int16_t GyroZ;
	  int16_t AccelX;
	  int16_t AccelY;
	  int16_t AccelZ;
} AnglesMPU;
typedef struct
{
	float error_prev;
	float integral;
	float kp;
	float ki;
	float kd;
	float control_signal;
} PID_struct;
typedef struct
{
	float X;
	float Y;
	float Z;
	float P;
	float I;
	float D;
} AnglesDegree;
