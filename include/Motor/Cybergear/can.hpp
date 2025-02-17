#ifndef __CAN__
#define __CAN__

#include "controlcan.h"
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <ctime>
#include <cstdlib>
#include "unistd.h"
#include "Motor/Cybergear/cybergear_base.hpp"
#include "m3508_base.hpp"

class CAN
{
public:
    motor_drive *cybergear[8];
    motor_3508 *m3508[2];
    int m3508_num = 0;
    int cybergear_num = 0;
    CAN(); // 初始化
    void CAN_open();

private:
    VCI_INIT_CONFIG config;
};

#endif