#ifndef __BLUETOOTH_H_
#define __BLUETOOTH_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "../../Core/Inc/stm32f4xx_it.h"
#include "../../Core/Inc/main.h"
#include "motor.h"

extern uint8_t Fore, Back, Left, Right;
extern uint8_t Maze_Enable;
extern volatile uint8_t Calib_Start_Request;
extern volatile uint8_t Calib_Stop_Request;
extern volatile uint8_t Map_Dump_Request;
extern volatile uint8_t Map_Clear_Request;
extern volatile uint8_t Lidar_Scan_Dump_Request;
extern volatile uint8_t Lidar_Reinit_Request;
extern volatile uint8_t Topo_Dump_Request;
extern volatile uint8_t Topo_Return_Request;
extern volatile uint8_t Topo_Return_Clear_Request;
extern volatile uint8_t Manual_Turn_Left_Request;
extern volatile uint8_t Manual_Turn_Right_Request;
extern volatile uint8_t Manual_Turn_Cancel_Request;
extern uint8_t rx_buf[2], Bluetooth_data;
extern int connect;

void JudgeConnect(void);
void BlueToothControl(uint8_t Bluetooth_data);

#endif
