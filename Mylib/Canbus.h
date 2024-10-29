#ifndef CANBUS_H_
#define CANBUS_H_

#include "main.h"

#define CAN_START_ENCODER			0xFF
#define CAN_RESET_ENCODER			0xFE
#define CAN_ENCODER_TXDELAY		0xFD
#define CAN_ENCODER_POSITION	0xFC
#define CAN_ENCODER_FEEDBACK	0xFB
#define CAN_IMU_FEEDBACK			0x05
#define CAN_IMU_TXDELAY				0x04
#define CAN_IMU_ANGLE					0x03
#define CAN_START_IMU					0x02
#define CAN_RESET_IMU					0x01

typedef struct
{
	float x;
	float y;
	float z;
}Angle_ReadTypeDef;

typedef union
{
	float f;
	uint8_t byte[4];
}Float_to_Uint8_t; //Chuy�n float sang uint8_t

//D�ng khi MCU c� 1 CAN connection v� chi d�ng filteridhigh
//Khai b�o filter
void CAN_Single_FilterMaskIDHigh_Config(CAN_HandleTypeDef *hcan, CAN_FilterTypeDef *sFilterConfig, uint8_t filter_bank, 
															uint8_t fifo, uint16_t filteridhigh, uint16_t filtermaskidhigh);
// Khoi tao ID cho TxHeader
void CAN_TxHeader_Init(CAN_TxHeaderTypeDef *TxHeader, uint32_t DLC, uint32_t StdId);
//Bat d�u thi�t bi
void CAN_Start(CAN_HandleTypeDef *hcan, uint8_t device ,CAN_TxHeaderTypeDef *TxHeader, uint16_t Tx_Delay);
//Reser thiet bi
void CAN_Reset(CAN_HandleTypeDef *hcan, uint8_t device, CAN_TxHeaderTypeDef *TxHeader);
//Y�u cau slave gui data theo chu k� 
void CAN_RequestData(CAN_HandleTypeDef *hcan,CAN_TxHeaderTypeDef *TxHeader, uint8_t device ,uint16_t tx_delay);
//Hoat d�ng nhu flag bat d�u
uint8_t CAN_Encoder_TxStart_Init(CAN_HandleTypeDef *hcan, uint32_t RxFifo, CAN_RxHeaderTypeDef *pHeader, uint8_t aData[]);
uint8_t CAN_IMU_TxStart_Init(CAN_HandleTypeDef *hcan, uint32_t RxFifo, CAN_RxHeaderTypeDef *pHeader, uint8_t aData[]);
//Nh�n chu k� gui data
void CAN_Encoder_TxDelay(uint8_t aData[]);
void CAN_IMU_TxDelay(uint8_t aData[]);
//Nh�n data tu Encoder/IMU
void CAN_Encoder_Recieve(uint8_t aData[], Float_to_Uint8_t *x_position, Float_to_Uint8_t *y_position);
void CAN_IMU_Recieve(uint8_t aData[], Angle_ReadTypeDef *angle);
//Gui data cua Encoder/IMU
void CAN_Encoder_Transmit(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *TxHeader, Float_to_Uint8_t x_position, Float_to_Uint8_t y_position);
void CAN_IMU_Transmit(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *TxHeader, uint8_t aData[]);
//Feedback chu k� truy�n data cho master
void CAN_Encoder_Feedback(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *TxHeader, uint8_t RxData[]);
void CAN_IMU_Feedback(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *TxHeader, uint8_t RxData[]);
//Nh�n th�ng tin feedback
uint16_t CAN_Encoder_RxFeedback(uint8_t aData[]);
uint16_t CAN_IMU_RxFeedback(uint8_t aData[]);


#endif



