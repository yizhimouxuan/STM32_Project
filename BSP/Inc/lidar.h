#ifndef __LIDAR_H__
#define __LIDAR_H__

#include "stm32f4xx_hal.h"

#define LIDAR_SCAN_BIN_COUNT 72U

/* 简单的帧处理入口，后续可在此解析 RPLIDAR C1 的 UART 数据帧 */
void Lidar_ProcessFrame(uint8_t *data, uint16_t len);
void Lidar_StartScan(void);
void Lidar_Stop(void);
void Lidar_SetMotorPwm(uint16_t pwm);
void Lidar_ForwardTask(void);
void Lidar_RequestInfo(void);
uint16_t Lidar_CopyLastFrameHex(char *out, uint16_t maxlen);
uint16_t Lidar_GetLastFrameLen(void);
int      Lidar_HasNewFrame(void);
uint32_t Lidar_GetTxCount(void);
uint32_t Lidar_GetTxAttemptCount(void);
uint32_t Lidar_GetTxErrorCount(void);
uint32_t Lidar_GetCurrentBaud(void);
int      Lidar_IsBaudLocked(void);
uint32_t Lidar_GetMotorMode(void);
float    Lidar_GetFrontDistanceM(void);
float    Lidar_GetRightDistanceM(void);
float    Lidar_GetRightFrontDistanceM(void);
float    Lidar_GetRightRearDistanceM(void);
float    Lidar_GetRearDistanceM(void);
float    Lidar_GetLeftRearDistanceM(void);
float    Lidar_GetLeftDistanceM(void);
float    Lidar_GetLeftFrontDistanceM(void);
uint16_t Lidar_GetScanBinCount(void);
uint32_t Lidar_GetScanTick(void);
uint16_t Lidar_CopyScanBinsMm(uint16_t *out_bins_mm, uint16_t max_bins);
uint16_t Lidar_GetScanMaskedCount(void);
void Lidar_InitSequence(void);

#endif /* __LIDAR_H__ */
