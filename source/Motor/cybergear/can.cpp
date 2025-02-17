#include "Motor/Cybergear/can.hpp"
// #include "leg.hpp"
#include <stdio.h>
#include <iostream>

using namespace std;
static int can_open_flag = 0;
motor_3508 m3508_1(0x201, VCI_USBCAN_ALYSTII, 0);
motor_3508 m3508_5(0x205, VCI_USBCAN_ALYSTII, 0);
motor_drive cybergear_1(can_master_id, motor_1_id, VCI_USBCAN_ALYSTII, 0);
motor_drive cybergear_2(can_master_id, motor_2_id, VCI_USBCAN_ALYSTII, 0);
motor_drive cybergear_3(can_master_id, motor_3_id, VCI_USBCAN_ALYSTII, 0);
motor_drive cybergear_4(can_master_id, motor_4_id, VCI_USBCAN_ALYSTII, 0);
motor_drive cybergear_5(can_master_id, motor_5_id, VCI_USBCAN_ALYSTII, 0);
motor_drive cybergear_6(can_master_id, motor_6_id, VCI_USBCAN_ALYSTII, 0);
motor_drive cybergear_7(can_master_id, motor_7_id, VCI_USBCAN_ALYSTII, 0);
motor_drive cybergear_8(can_master_id, motor_8_id, VCI_USBCAN_ALYSTII, 0);

CAN::CAN()
{
    cybergear[0] = &cybergear_1;
    cybergear[1] = &cybergear_2;
    cybergear[2] = &cybergear_3;
    cybergear[3] = &cybergear_4;
    cybergear[4] = &cybergear_5;
    cybergear[5] = &cybergear_6;
    cybergear[6] = &cybergear_7;
    cybergear[7] = &cybergear_8;
    m3508[0] = &m3508_1;
    m3508[1] = &m3508_5;
    m3508_num = sizeof(m3508) / sizeof(m3508[0]);
    cybergear_num = sizeof(cybergear) / sizeof(cybergear[0]);
    printf(">>motor init success\n");
}

void CAN::CAN_open()
{
    if (can_open_flag == 0)
    {
        std::cout << can_open_flag << std::endl;

        can_open_flag = 1;
        if (VCI_OpenDevice(VCI_USBCAN_ALYSTII, 0, 0) == 1) // 打开设备
        {
            printf(">>open device success!\n"); // 打开设备成功
        }
        else
        {
            printf(">>open device error!\n");
            exit(1);
        }
        std::cout << can_open_flag << std::endl;
    }

    config.AccCode = 0;
    config.AccMask = 0xFFFFFFFF;
    config.Filter = 1;
    config.Timing0 = 0x00;
    config.Timing1 = 0x14;
    config.Mode = 0; // 正常模式

    while (VCI_InitCAN(VCI_USBCAN_ALYSTII, 0, 0, &config) != 1)
    {
        cout << "Init CAN1 error!" << endl;
        sleep(1);
    }

    if (VCI_StartCAN(VCI_USBCAN_ALYSTII, 0, 0) != 1)
    {
        printf(">>Start CAN1 error\n");
        VCI_CloseDevice(VCI_USBCAN_ALYSTII, 0);
    }

}