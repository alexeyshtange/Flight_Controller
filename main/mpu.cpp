#include "mpu.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
typedef struct
{
	  int struct_member;
} my_struct;
extern my_struct My_struct;
my_class My_class;
extern "C"  int return_class() {
return My_class.output();
}


