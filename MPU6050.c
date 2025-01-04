/*
 * MPU6050.c
 *
 *  Created on: Dec 31, 2024
 *      Author: Alejandro
 */

// Libraries
#include "MPU6050.h"

I2C_HandleTypeDef _i2cMPU;

struct MPU6050_Handler _MPU_Values = {};

uint8_t buffer[14];

int16_t offsets[6];

uint32_t fifoTimeout = MPU6050_FIFO_DEFAULT_TIMEOUT;

void MPU6050_Init(I2C_HandleTypeDef *i2cMPU){
	_i2cMPU = *i2cMPU;
	setClockSource(MPU6050_CLOCK_PLL_XGYRO);
	setFullScaleGyroRange(MPU6050_GYRO_FS_250);
	setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
	setSleepEnabled(false);

}

uint32_t micros(){
	return 0;
}


// ================================ Wrapper Functions ====================================

// Write Bit MPU6050

bool MPU6050_WriteBit(uint8_t reg, uint8_t bitNum, uint8_t value){
	return writeBit(&_i2cMPU, MPU6050_DEFAULT_ADDRESS, reg, bitNum, value);
}

// Write Bits MPU6050

bool MPU6050_WriteBits(uint8_t reg, uint8_t BitStart, uint8_t length, uint8_t value){
	return writeBits(&_i2cMPU, MPU6050_DEFAULT_ADDRESS, reg, BitStart, length, value);
}


// Write Byte MPU6050

bool MPU6050_WriteByte(uint8_t reg, uint8_t value){
    return writeByte(&_i2cMPU, MPU6050_DEFAULT_ADDRESS, reg, value);
}

// Write Bytes MPU6050

bool MPU6050_WriteBytes(uint8_t reg, uint8_t length, uint8_t *data){
	return writeBytes(&_i2cMPU, MPU6050_DEFAULT_ADDRESS, reg, length, data);
}

// Write Word MPU6050

bool MPU6050_WriteWord(uint8_t reg, uint16_t value){
	return writeWord(&_i2cMPU, MPU6050_DEFAULT_ADDRESS, reg, value);
}

// Write Words MPU6050
bool MPU6050_WriteWords(uint8_t reg, uint8_t length, uint16_t *pdata){
	return writeWords(&_i2cMPU, MPU6050_DEFAULT_ADDRESS, reg, length, pdata);
}


// Read Bit MPU6050

void MPU6050_ReadBit(uint8_t reg, uint8_t BitNum, uint8_t *pdata){
	readBit(&_i2cMPU, MPU6050_DEFAULT_ADDRESS, reg, BitNum, pdata);
}

// Read Bits MPU6050

void MPU6050_ReadBits(uint8_t reg, uint8_t BitStart, uint8_t length, uint8_t *pdata){
	readBits(&_i2cMPU, MPU6050_DEFAULT_ADDRESS, reg, BitStart, length, pdata);
}

// Read Byte MPU6050

HAL_StatusTypeDef MPU6050_ReadByte(uint8_t reg, uint8_t *pdata){
	return readByte(&_i2cMPU, MPU6050_DEFAULT_ADDRESS, reg, pdata);
}

// Read Bytes MPU6050

HAL_StatusTypeDef MPU6050_ReadBytes(uint8_t reg, uint8_t num, uint8_t *pdata){
	return readBytes(&_i2cMPU, MPU6050_DEFAULT_ADDRESS, reg, num, pdata);
}

// Read Words MPU6050
HAL_StatusTypeDef MPU6050_ReadWords(uint8_t reg, uint8_t length, uint16_t *pdata){
	return readWords(&_i2cMPU, MPU6050_DEFAULT_ADDRESS, reg, length, pdata);
}


// ====================================== MPU6050 Functions ===========================================

// SMPLRT_DIV register

uint8_t getRate(){
	MPU6050_ReadByte(MPU6050_RA_SMPLRT_DIV, buffer);
	return buffer[0];
}

void setRate(uint8_t rate){
	MPU6050_WriteByte(MPU6050_RA_SMPLRT_DIV, rate);
}

// CONFIG register

uint8_t getExternalFrameSync(){
	MPU6050_ReadBits(MPU6050_RA_CONFIG, MPU6050_CFG_EXT_SYNC_SET_BIT, MPU6050_CFG_EXT_SYNC_SET_LENGTH, buffer);
	return buffer[0];
}

void setExternalFrameSync(uint8_t sync){
	MPU6050_WriteBits(MPU6050_RA_CONFIG, MPU6050_CFG_EXT_SYNC_SET_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, sync);
}

uint8_t getDLPFMode(){
	MPU6050_ReadBits(MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, buffer);
	return buffer[0];
}

void setDLPFMode(uint8_t mode){
	MPU6050_WriteBits(MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}

// GYRO_CONFIG register

uint8_t getFullScaleGyroRange(){
	MPU6050_ReadBits(MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, buffer);
	return buffer[0];
}

void setFullScaleGyroRange(uint8_t range){
	MPU6050_WriteBits(MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

// ACCEL_CONFIG register
uint8_t getFullScaleAccelRange(){
	MPU6050_ReadBits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, buffer);
	return buffer[0];
}

void setFullScaleAccelRange(uint8_t range){
	MPU6050_WriteBits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

void setDHPFMode(uint8_t mode){
	MPU6050_WriteBits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_ACCEL_HPF_BIT, MPU6050_ACONFIG_ACCEL_HPF_LENGTH, mode);
}


// FF_THR register
uint8_t getFreefallDetectionThreshold(){
	MPU6050_ReadByte(MPU6050_RA_FF_THR, buffer);
	return buffer[0];
}
void setFreefallDetectionThreshold(uint8_t threshold){
	MPU6050_WriteByte(MPU6050_RA_FF_THR, threshold);
}

// FF_DUR register
uint8_t getFreefallDetectionDuration(){
	MPU6050_ReadByte(MPU6050_RA_FF_DUR, buffer);
	return buffer[0];
}
void setFreefallDetectionDuration(uint8_t duration){
	MPU6050_WriteByte(MPU6050_RA_FF_DUR, duration);
}

// MOT_THR register
uint8_t getMotionDetectionThreshold(){
	MPU6050_ReadByte(MPU6050_RA_MOT_THR, buffer);
	return buffer[0];
}
void setMotionDetectionThreshold(uint8_t threshold){
	MPU6050_WriteByte(MPU6050_RA_MOT_THR, threshold);
}

// MOT_DUR register
uint8_t getMotionDetectionDuration(){
	MPU6050_ReadByte(MPU6050_RA_MOT_DUR, buffer);
	return buffer[0];
}
void setMotionDetectionDuration(uint8_t duration){
	MPU6050_WriteByte(MPU6050_RA_MOT_DUR, duration);
}

// ZRMOT_THR register
uint8_t getZeroMotionDetectionThreshold(){
	MPU6050_ReadByte(MPU6050_RA_ZRMOT_THR, buffer);
	return buffer[0];
}
void setZeroMotionDetectionThreshold(uint8_t threshold){
	MPU6050_WriteByte(MPU6050_RA_ZRMOT_THR, threshold);
}

// ZRMOT_DUR register
uint8_t getZeroMotionDetectionDuration(){
	MPU6050_ReadByte(MPU6050_RA_ZRMOT_DUR, buffer);
	return buffer[0];
}
void setZeroMotionDetectionDuration(uint8_t duration){
	MPU6050_WriteByte(MPU6050_RA_ZRMOT_DUR, duration);
}


// FIFO_EN register
bool getTempFIFOEnabled(){
	MPU6050_ReadBit(MPU6050_RA_FIFO_EN, MPU6050_TEMP_FIFO_EN_BIT, buffer);
	return buffer[0];
}
void setTempFIFOEnabled(bool enabled){
	MPU6050_WriteBit(MPU6050_RA_FIFO_EN, MPU6050_TEMP_FIFO_EN_BIT, enabled);
}
bool getXGyroFIFOEnabled(){
	MPU6050_ReadBit(MPU6050_RA_FIFO_EN, MPU6050_XG_FIFO_EN_BIT, buffer);
	return buffer[0];
}
void setXGyroFIFOEnabled(bool enabled){
	MPU6050_WriteBit(MPU6050_RA_FIFO_EN, MPU6050_XG_FIFO_EN_BIT, enabled);
}
bool getYGyroFIFOEnabled(){
	MPU6050_ReadBit(MPU6050_RA_FIFO_EN, MPU6050_YG_FIFO_EN_BIT, buffer);
	return buffer[0];
}
void setYGyroFIFOEnabled(bool enabled){
	MPU6050_WriteBit(MPU6050_RA_FIFO_EN, MPU6050_YG_FIFO_EN_BIT, enabled);
}
bool getZGyroFIFOEnabled(){
	MPU6050_ReadBit(MPU6050_RA_FIFO_EN, MPU6050_ZG_FIFO_EN_BIT, buffer);
	return buffer[0];
}
void setZGyroFIFOEnabled(bool enabled){
	MPU6050_WriteBit(MPU6050_RA_FIFO_EN, MPU6050_ZG_FIFO_EN_BIT, enabled);
}
bool getAccelFIFOEnabled(){
	MPU6050_ReadBit(MPU6050_RA_FIFO_EN, MPU6050_ACCEL_FIFO_EN_BIT, buffer);
	return buffer[0];
}
void setAccelFIFOEnabled(bool enabled){
	MPU6050_WriteBit(MPU6050_RA_FIFO_EN, MPU6050_ACCEL_FIFO_EN_BIT, enabled);
}

bool getSlave2FIFOEnabled(){
	MPU6050_ReadBit(MPU6050_RA_FIFO_EN, MPU6050_SLV2_FIFO_EN_BIT, buffer);
	return buffer[0];
}
void setSlave2FIFOEnabled(bool enabled){
	MPU6050_WriteBit(MPU6050_RA_FIFO_EN, MPU6050_SLV2_FIFO_EN_BIT, enabled);
}
bool getSlave1FIFOEnabled(){
	MPU6050_ReadBit(MPU6050_RA_FIFO_EN, MPU6050_SLV1_FIFO_EN_BIT, buffer);
	return buffer[0];
}
void setSlave1FIFOEnabled(bool enabled){
	MPU6050_WriteBit(MPU6050_RA_FIFO_EN, MPU6050_SLV1_FIFO_EN_BIT, enabled);
}
bool getSlave0FIFOEnabled(){
	MPU6050_ReadBit(MPU6050_RA_FIFO_EN, MPU6050_SLV0_FIFO_EN_BIT, buffer);
	return buffer[0];
}
void setSlave0FIFOEnabled(bool enabled){
	MPU6050_WriteBit(MPU6050_RA_FIFO_EN, MPU6050_SLV0_FIFO_EN_BIT, enabled);
}




// INT_PIN_CFG register
bool getInterruptMode(){
	MPU6050_ReadBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_LEVEL_BIT, buffer);
	return buffer[0];
}
void setInterruptMode(bool mode){
	MPU6050_WriteBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_LEVEL_BIT, mode);
}
bool getInterruptDrive(){
	MPU6050_ReadBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_OPEN_BIT, buffer);
	return buffer[0];
}
void setInterruptDrive(bool drive){
	MPU6050_WriteBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_OPEN_BIT, drive);
}
bool getInterruptLatch(){
	MPU6050_ReadBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_LATCH_INT_EN_BIT, buffer);
	return buffer[0];
}
void setInterruptLatch(bool latch){
	MPU6050_WriteBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_LATCH_INT_EN_BIT, latch);
}
bool getInterruptLatchClear(){
	MPU6050_ReadBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_RD_CLEAR_BIT, buffer);
	return buffer[0];
}
void setInterruptLatchClear(bool clear){
	MPU6050_WriteBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_RD_CLEAR_BIT, clear);
}
bool getFSyncInterruptLevel(){
	MPU6050_ReadBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT, buffer);
	return buffer[0];
}
void setFSyncInterruptLevel(bool level){
	MPU6050_WriteBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT, level);
}
bool getFSyncInterruptEnabled(){
	MPU6050_ReadBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_FSYNC_INT_EN_BIT, buffer);
	return buffer[0];
}
void setFSyncInterruptEnabled(bool enabled){
	MPU6050_WriteBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_FSYNC_INT_EN_BIT, enabled);
}
bool getI2CBypassEnabled(){
	MPU6050_ReadBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, buffer);
	return buffer[0];
}
void setI2CBypassEnabled(bool enabled){
	MPU6050_WriteBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}
bool getClockOutputEnabled(){
	MPU6050_ReadBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_CLKOUT_EN_BIT, buffer);
	return buffer[0];
}
void setClockOutputEnabled(bool enabled){
	MPU6050_WriteBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_CLKOUT_EN_BIT, enabled);
}

// INT_ENABLE register
uint8_t getIntEnabled(){
	MPU6050_ReadByte(MPU6050_RA_INT_ENABLE, buffer);
	return buffer[0];
}
void setIntEnabled(uint8_t enabled){
	MPU6050_WriteByte(MPU6050_RA_INT_ENABLE, enabled);
}
bool getIntFreefallEnabled(){
	MPU6050_ReadBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_FF_BIT, buffer);
	return buffer[0];
}
void setIntFreefallEnabled(bool enabled){
	MPU6050_WriteBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_FF_BIT, enabled);
}
bool getIntMotionEnabled(){
	MPU6050_ReadBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_MOT_BIT, buffer);
	return buffer[0];
}
void setIntMotionEnabled(bool enabled){
	MPU6050_WriteBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_MOT_BIT, enabled);
}
bool getIntZeroMotionEnabled(){
	MPU6050_ReadBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_ZMOT_BIT, buffer);
	return buffer[0];
}
void setIntZeroMotionEnabled(bool enabled){
	MPU6050_WriteBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_ZMOT_BIT, enabled);
}
bool getIntFIFOBufferOverflowEnabled(){
	MPU6050_ReadBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_FIFO_OFLOW_BIT, buffer);
	return buffer[0];
}
void setIntFIFOBufferOverflowEnabled(bool enabled){
	MPU6050_WriteBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_FIFO_OFLOW_BIT, enabled);
}
bool getIntI2CMasterEnabled(){
	MPU6050_ReadBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_I2C_MST_INT_BIT, buffer);
	return buffer[0];
}
void setIntI2CMasterEnabled(bool enabled){
	MPU6050_WriteBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_I2C_MST_INT_BIT, enabled);
}
bool getIntDataReadyEnabled(){
	MPU6050_ReadBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, buffer);
	return buffer[0];
}
void setIntDataReadyEnabled(bool enabled){
	MPU6050_WriteBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, enabled);
}


// INT_STATUS register
uint8_t getIntStatus(){
	MPU6050_ReadByte(MPU6050_RA_INT_STATUS, buffer);
	return buffer[0];
}
bool getIntFreefallStatus(){
	MPU6050_ReadBit(MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_FF_BIT, buffer);
	return buffer[0];
}
bool getIntMotionStatus(){
	MPU6050_ReadBit(MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_MOT_BIT, buffer);
	return buffer[0];
}
bool getIntZeroMotionStatus(){
	MPU6050_ReadBit(MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_ZMOT_BIT, buffer);
	return buffer[0];
}
bool getIntFIFOBufferOverflowStatus(){
	MPU6050_ReadBit(MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_FIFO_OFLOW_BIT, buffer);
	return buffer[0];
}
bool getIntI2CMasterStatus(){
	MPU6050_ReadBit(MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_I2C_MST_INT_BIT, buffer);
	return buffer[0];
}
bool getIntDataReadyStatus(){
	MPU6050_ReadBit(MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_DATA_RDY_BIT, buffer);
	return buffer[0];
}


// ACCEL_*OUT_* registers
void getMotion9(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* mx, int16_t* my, int16_t* mz){
	(void)mx; // unused parameter
	(void)my; // unused parameter
	(void)mz; // unused parameter

	getMotion6(ax, ay, az, gx, gy, gz);
	// TODO: magnetometer integration
}
void getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz){
	MPU6050_ReadBytes(MPU6050_RA_ACCEL_XOUT_H, 14, buffer);
	*ax = (((int16_t)buffer[0]) << 8) | buffer[1];
	*ay = (((int16_t)buffer[2]) << 8) | buffer[3];
	*az = (((int16_t)buffer[4]) << 8) | buffer[5];
	*gx = (((int16_t)buffer[8]) << 8) | buffer[9];
	*gy = (((int16_t)buffer[10]) << 8) | buffer[11];
	*gz = (((int16_t)buffer[12]) << 8) | buffer[13];
}
void getAcceleration(int16_t* x, int16_t* y, int16_t* z){
	MPU6050_ReadBytes(MPU6050_RA_ACCEL_XOUT_H, 6, buffer);
	*x = (((int16_t)buffer[0]) << 8) | buffer[1];
	*y = (((int16_t)buffer[2]) << 8) | buffer[3];
	*z = (((int16_t)buffer[4]) << 8) | buffer[5];
}
int16_t getAccelerationX(){
	MPU6050_ReadBytes(MPU6050_RA_ACCEL_XOUT_H, 2, buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}
int16_t getAccelerationY(){
	MPU6050_ReadBytes(MPU6050_RA_ACCEL_YOUT_H, 2, buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}
int16_t getAccelerationZ(){
	MPU6050_ReadBytes(MPU6050_RA_ACCEL_ZOUT_H, 2, buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}

// TEMP_OUT_* registers
int16_t getTemperature(){
	MPU6050_ReadBytes(MPU6050_RA_TEMP_OUT_H, 2, buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}

// GYRO_*OUT_* registers
void getRotation(int16_t* x, int16_t* y, int16_t* z){
	MPU6050_ReadBytes(MPU6050_RA_GYRO_XOUT_H, 6, buffer);
	*x = (((int16_t)buffer[0]) << 8) | buffer[1];
	*y = (((int16_t)buffer[2]) << 8) | buffer[3];
	*z = (((int16_t)buffer[4]) << 8) | buffer[5];
}
int16_t getRotationX(){
	MPU6050_ReadBytes(MPU6050_RA_GYRO_XOUT_H, 2, buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}
int16_t getRotationY(){
	MPU6050_ReadBytes(MPU6050_RA_GYRO_YOUT_H, 2, buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}
int16_t getRotationZ(){
	MPU6050_ReadBytes(MPU6050_RA_GYRO_ZOUT_H, 2, buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}


// USER_CTRL register
bool getFIFOEnabled(){
	MPU6050_ReadBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT, buffer);
	return buffer[0];
}
void setFIFOEnabled(bool enabled){
	MPU6050_WriteBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT, enabled);
}
bool getI2CMasterModeEnabled(){
	MPU6050_ReadBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, buffer);
	return buffer[0];
}
void setI2CMasterModeEnabled(bool enabled){
	MPU6050_WriteBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}
void resetFIFO(){
	MPU6050_WriteBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, true);
}
void resetI2CMaster(){
	MPU6050_WriteBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_RESET_BIT, true);
}
void resetSensors(){
	MPU6050_WriteBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_SIG_COND_RESET_BIT, true);
}


// PWR_MGMT_1 register
void reset(){
	MPU6050_WriteBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, true);
}
bool getSleepEnabled(){
	MPU6050_ReadBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, buffer);
	return buffer[0];
}
void setSleepEnabled(bool enabled){
	MPU6050_WriteBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

bool getWakeCycleEnabled(){
	MPU6050_ReadBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CYCLE_BIT, buffer);
	return buffer[0];
}
void setWakeCycleEnabled(bool enabled){
	MPU6050_WriteBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CYCLE_BIT, enabled);
}
bool getTempSensorEnabled(){
	MPU6050_ReadBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_TEMP_DIS_BIT, buffer);
	return buffer[0] == 0;// 1 is actually disabled here
}
void setTempSensorEnabled(bool enabled){
	// 1 is actually disabled here
	MPU6050_WriteBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_TEMP_DIS_BIT, !enabled);
}
uint8_t getClockSource(){
	MPU6050_ReadBits(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, buffer);
	return buffer[0];
}
void setClockSource(uint8_t source){
	MPU6050_WriteBits(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}


// PWR_MGMT_2 register
uint8_t getWakeFrequency(){
	MPU6050_ReadBits(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_LP_WAKE_CTRL_BIT, MPU6050_PWR2_LP_WAKE_CTRL_LENGTH, buffer);
	return buffer[0];
}
void setWakeFrequency(uint8_t frequency){
	MPU6050_WriteBits(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_LP_WAKE_CTRL_BIT, MPU6050_PWR2_LP_WAKE_CTRL_LENGTH, frequency);
}
bool getStandbyXAccelEnabled(){
	MPU6050_ReadBit(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_XA_BIT, buffer);
	return buffer[0];
}
void setStandbyXAccelEnabled(bool enabled){
	MPU6050_WriteBit(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_XA_BIT, enabled);
}
bool getStandbyYAccelEnabled(){
	MPU6050_ReadBit(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_YA_BIT, buffer);
		return buffer[0];
}
void setStandbyYAccelEnabled(bool enabled){
	MPU6050_WriteBit(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_YA_BIT, enabled);
}
bool getStandbyZAccelEnabled(){
	MPU6050_ReadBit(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_ZA_BIT, buffer);
	return buffer[0];
}
void setStandbyZAccelEnabled(bool enabled){
	MPU6050_WriteBit(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_ZA_BIT, enabled);
}
bool getStandbyXGyroEnabled(){
	MPU6050_ReadBit(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_XG_BIT, buffer);
	return buffer[0];
}
void setStandbyXGyroEnabled(bool enabled){
	MPU6050_WriteBit(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_XG_BIT, enabled);
}
bool getStandbyYGyroEnabled(){
	MPU6050_ReadBit(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_YG_BIT, buffer);
	return buffer[0];
}
void setStandbyYGyroEnabled(bool enabled){
	MPU6050_WriteBit(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_YG_BIT, enabled);
}
bool getStandbyZGyroEnabled(){
	MPU6050_ReadBit(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_ZG_BIT, buffer);
	return buffer[0];
}
void setStandbyZGyroEnabled(bool enabled){
	MPU6050_WriteBit(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_ZG_BIT, enabled);
}


// FIFO_COUNT_* registers
uint16_t getFIFOCount(){
	MPU6050_ReadBytes(MPU6050_RA_FIFO_COUNTH, 2, buffer);
	return (((uint16_t)buffer[0]) << 8) | buffer[1];
}

// FIFO_R_W register
uint8_t getFIFOByte(){
	MPU6050_ReadByte(MPU6050_RA_FIFO_R_W, buffer);
	return buffer[0];
}

int8_t GetCurrentFIFOPacket(uint8_t *data, uint8_t length){
	int16_t fifoC;
	uint32_t BreakTimer = micros();
	bool packetReceived = false;
	do {
		if ((fifoC = getFIFOCount())  > length) {

			if (fifoC > 200) { // if you waited to get the FIFO buffer to > 200 bytes it will take longer to get the last packet in the FIFO Buffer than it will take to  reset the buffer and wait for the next to arrive
				resetFIFO(); // Fixes any overflow corruption
				fifoC = 0;
				while (!(fifoC = getFIFOCount()) && ((micros() - BreakTimer) <= (getFIFOTimeout()))); // Get Next New Packet
			} else { //We have more than 1 packet but less than 200 bytes of data in the FIFO Buffer
				uint8_t Trash[32];
				while ((fifoC = getFIFOCount()) > length) {  // Test each time just in case the MPU is writing to the FIFO Buffer
					fifoC = fifoC - length; // Save the last packet
					uint16_t  RemoveBytes;
					while (fifoC) { // fifo count will reach zero so this is safe
						RemoveBytes = (fifoC < 32) ? fifoC : 32; // Buffer Length is different than the packet length this will efficiently clear the buffer
						getFIFOBytes(Trash, (uint8_t)RemoveBytes);
						fifoC -= RemoveBytes;
					}
				}
			}
		}
		if (!fifoC) return 0; // Called too early no data or we timed out after FIFO Reset
		// We have 1 packet
		packetReceived = fifoC == length;
		if (!packetReceived && (micros() - BreakTimer) > (getFIFOTimeout())) return 0;
	} while (!packetReceived);
	getFIFOBytes(data, length); //Get 1 packet
	return 1;
}

void setFIFOByte(uint8_t data){
	MPU6050_WriteByte(MPU6050_RA_FIFO_R_W, data);
}
void getFIFOBytes(uint8_t *data, uint8_t length){
	if(length > 0){
		MPU6050_ReadBytes(MPU6050_RA_FIFO_R_W, length, data);
	} else {
		*data = 0;
	}
}
void setFIFOTimeout(uint32_t FIFOTimeout){
	fifoTimeout = FIFOTimeout;
}
uint32_t getFIFOTimeout(){
	return fifoTimeout;
}


// WHO_AM_I register
uint8_t getDeviceID(){
	MPU6050_ReadBits(MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, buffer);
	return buffer[0];
}
void setDeviceID(uint8_t id){
	MPU6050_WriteBits(MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, id);
}


bool testConnection(){
	return getDeviceID() == 0x34;
}

// ======== UNDOCUMENTED/DMP REGISTERS/METHODS ========


// XG_OFFS_TC register
uint8_t getOTPBankValid(){
	MPU6050_ReadBit(MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT, buffer);
	return buffer[0];
}
void setOTPBankValid(bool enabled){
	MPU6050_WriteBit(MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT, enabled);
}
int8_t getXGyroOffsetTC(){
	MPU6050_ReadBits(MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, buffer);
	return buffer[0];
}
void setXGyroOffsetTC(int8_t offset){
	MPU6050_WriteBits(MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

// YG_OFFS_TC register
int8_t getYGyroOffsetTC(){
	MPU6050_ReadBits(MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, buffer);
	return buffer[0];
}
void setYGyroOffsetTC(int8_t offset){
	MPU6050_WriteBits(MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

// ZG_OFFS_TC register
int8_t getZGyroOffsetTC(){
	MPU6050_ReadBits(MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, buffer);
	return buffer[0];
}
void setZGyroOffsetTC(int8_t offset){
	MPU6050_WriteBits(MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

// X_FINE_GAIN register
int8_t getXFineGain(){
	MPU6050_ReadByte(MPU6050_RA_X_FINE_GAIN, buffer);
	return buffer[0];
}
void setXFineGain(int8_t gain){
	MPU6050_WriteByte(MPU6050_RA_X_FINE_GAIN, gain);
}

// Y_FINE_GAIN register
int8_t getYFineGain(){
	MPU6050_ReadByte(MPU6050_RA_Y_FINE_GAIN, buffer);
	return buffer[0];
}
void setYFineGain(int8_t gain){
	MPU6050_WriteByte(MPU6050_RA_Y_FINE_GAIN, gain);
}

// Z_FINE_GAIN register
int8_t getZFineGain(){
	MPU6050_ReadByte(MPU6050_RA_Z_FINE_GAIN, buffer);
	return buffer[0];
}
void setZFineGain(int8_t gain){
	MPU6050_WriteByte(MPU6050_RA_Z_FINE_GAIN, gain);
}

// XA_OFFS_* registers
int16_t getXAccelOffset(){
	uint8_t SaveAddress = ((getDeviceID() < 0x38 )? MPU6050_RA_XA_OFFS_H:0x77); // MPU6050,MPU9150 Vs MPU6500,MPU9250
	MPU6050_ReadBytes(SaveAddress, 2, buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void setXAccelOffset(int16_t offset){
	uint8_t SaveAddress = ((getDeviceID() < 0x38 )? MPU6050_RA_XA_OFFS_H:0x77); // MPU6050,MPU9150 Vs MPU6500,MPU9250
	MPU6050_WriteWord(SaveAddress, offset);
}

// YA_OFFS_* register
int16_t getYAccelOffset(){
	uint8_t SaveAddress = ((getDeviceID() < 0x38 )? MPU6050_RA_YA_OFFS_H:0x7A); // MPU6050,MPU9150 Vs MPU6500,MPU9250
	MPU6050_ReadBytes(SaveAddress, 2, buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void setYAccelOffset(int16_t offset){
	uint8_t SaveAddress = ((getDeviceID() < 0x38 )? MPU6050_RA_YA_OFFS_H:0x77); // MPU6050,MPU9150 Vs MPU6500,MPU9250
	MPU6050_WriteWord(SaveAddress, offset);
}

// ZA_OFFS_* register
int16_t getZAccelOffset(){
	uint8_t SaveAddress = ((getDeviceID() < 0x38 )? MPU6050_RA_ZA_OFFS_H:0x7A); // MPU6050,MPU9150 Vs MPU6500,MPU9250
		MPU6050_ReadBytes(SaveAddress, 2, buffer);
		return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void setZAccelOffset(int16_t offset){
	uint8_t SaveAddress = ((getDeviceID() < 0x38 )? MPU6050_RA_ZA_OFFS_H:0x77); // MPU6050,MPU9150 Vs MPU6500,MPU9250
		MPU6050_WriteWord(SaveAddress, offset);
}

// XG_OFFS_USR* registers
int16_t getXGyroOffset(){
	MPU6050_ReadBytes(MPU6050_RA_XG_OFFS_USRH, 2, buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void setXGyroOffset(int16_t offset){
	MPU6050_WriteWord(MPU6050_RA_XG_OFFS_USRH, offset);
}

// YG_OFFS_USR* register
int16_t getYGyroOffset(){
	MPU6050_ReadBytes(MPU6050_RA_YG_OFFS_USRH, 2, buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void setYGyroOffset(int16_t offset){
	MPU6050_WriteWord(MPU6050_RA_YG_OFFS_USRH, offset);
}

// ZG_OFFS_USR* register
int16_t getZGyroOffset(){
	MPU6050_ReadBytes(MPU6050_RA_ZG_OFFS_USRH, 2, buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void setZGyroOffset(int16_t offset){
	MPU6050_WriteWord(MPU6050_RA_ZG_OFFS_USRH, offset);
}

// INT_ENABLE register (DMP functions)
bool getIntPLLReadyEnabled(){
	MPU6050_ReadBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_PLL_RDY_INT_BIT, buffer);
	return buffer[0];
}
void setIntPLLReadyEnabled(bool enabled){
	MPU6050_WriteBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_PLL_RDY_INT_BIT, enabled);
}
bool getIntDMPEnabled(){
	MPU6050_ReadBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DMP_INT_BIT, buffer);
	return buffer[0];
}
void setIntDMPEnabled(bool enabled){
	MPU6050_WriteBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DMP_INT_BIT, enabled);
}

// DMP_INT_STATUS
bool getDMPInt5Status(){
	MPU6050_ReadBit(MPU6050_RA_DMP_INT_STATUS, MPU6050_DMPINT_5_BIT, buffer);
	return buffer[0];
}
bool getDMPInt4Status(){
	MPU6050_ReadBit(MPU6050_RA_DMP_INT_STATUS, MPU6050_DMPINT_4_BIT, buffer);
	return buffer[0];
}
bool getDMPInt3Status(){
	MPU6050_ReadBit(MPU6050_RA_DMP_INT_STATUS, MPU6050_DMPINT_3_BIT, buffer);
	return buffer[0];
}
bool getDMPInt2Status(){
	MPU6050_ReadBit(MPU6050_RA_DMP_INT_STATUS, MPU6050_DMPINT_2_BIT, buffer);
	return buffer[0];
}
bool getDMPInt1Status(){
	MPU6050_ReadBit(MPU6050_RA_DMP_INT_STATUS, MPU6050_DMPINT_1_BIT, buffer);
	return buffer[0];
}
bool getDMPInt0Status(){
	MPU6050_ReadBit(MPU6050_RA_DMP_INT_STATUS, MPU6050_DMPINT_0_BIT, buffer);
	return buffer[0];
}

// INT_STATUS register (DMP functions)
bool getIntPLLReadyStatus(){
	MPU6050_ReadBit(MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_PLL_RDY_INT_BIT, buffer);
	return buffer[0];
}
bool getIntDMPStatus(){
	MPU6050_ReadBit(MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_DMP_INT_BIT, buffer);
	return buffer[0];
}

// USER_CTRL register (DMP functions)
bool getDMPEnabled(){
	MPU6050_ReadBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT, buffer);
	return buffer[0];
}
void setDMPEnabled(bool enabled){
	MPU6050_WriteBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT, enabled);
}
void resetDMP(){
	MPU6050_WriteBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_RESET_BIT, true);
}

// BANK_SEL register
void setMemoryBank_Default(uint8_t bank){
	setMemoryBank(bank, false, false);
}

void setMemoryBank(uint8_t bank, bool prefetchEnabled, bool userBank){
	// Default values
	// prefetchEnabled=false, userBank=false
	bank &= 0x1F;
	if (userBank) bank |= 0x20;
	if (prefetchEnabled) bank |= 0x40;
	MPU6050_WriteByte(MPU6050_RA_BANK_SEL, bank);
}

// MEM_START_ADDR register
void setMemoryStartAddress(uint8_t address){
	MPU6050_WriteByte(MPU6050_RA_MEM_START_ADDR, address);
}

// MEM_R_W register
uint8_t readMemoryByte(){
	MPU6050_ReadByte(MPU6050_RA_MEM_R_W, buffer);
	return buffer[0];
}
void writeMemoryByte(uint8_t data){
	MPU6050_WriteByte(MPU6050_RA_MEM_R_W, data);
}

void readMemoryBlock_Default(uint8_t *data, uint16_t dataSize){
	readMemoryBlock(data, dataSize, 0, 0);
}

void readMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address){
	// Default values
	// bank=0, address=0
	setMemoryBank_Default(bank);
	setMemoryStartAddress(address);
	uint8_t chunkSize;
	for (uint16_t i = 0; i < dataSize;) {
		// determine correct chunk size according to bank position and data size
		chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

		// make sure we don't go past the data size
		if (i + chunkSize > dataSize) chunkSize = dataSize - i;

		// make sure this chunk doesn't go past the bank boundary (256 bytes)
		if (chunkSize > 256 - address) chunkSize = 256 - address;

		// read the chunk of data as specified
		MPU6050_ReadBytes(MPU6050_RA_MEM_R_W, chunkSize, data + i);

		// increase byte index by [chunkSize]
		i += chunkSize;

		// uint8_t automatically wraps to 0 at 256
		address += chunkSize;

		// if we aren't done, update bank (if necessary) and address
		if (i < dataSize) {
			if (address == 0) bank++;
			setMemoryBank_Default(bank);
			setMemoryStartAddress(address);
		}
	}
}

bool writeMemoryBlock_Default(const uint8_t *data, uint16_t dataSize){
	return writeMemoryBlock(data, dataSize, 0, 0, true);
}
bool writeMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify){
	// Default values
	// bank=0, address=0, verify=true
	setMemoryBank_Default(bank);
	setMemoryStartAddress(address);
	uint8_t chunkSize;
	uint8_t *verifyBuffer=0;
	uint8_t *progBuffer=0;
	uint16_t i;
	if (verify) verifyBuffer = (uint8_t *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
	for (i = 0; i < dataSize;) {
		// determine correct chunk size according to bank position and data size
		chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

		// make sure we don't go past the data size
		if (i + chunkSize > dataSize) chunkSize = dataSize - i;

		// make sure this chunk doesn't go past the bank boundary (256 bytes)
		if (chunkSize > 256 - address) chunkSize = 256 - address;

		// write the chunk of data as specified
		progBuffer = (uint8_t *)data + i;


		MPU6050_WriteBytes(MPU6050_RA_MEM_R_W, chunkSize, progBuffer);

		// verify data if needed
		if (verify && verifyBuffer) {
			setMemoryBank_Default(bank);
			setMemoryStartAddress(address);
			MPU6050_ReadBytes(MPU6050_RA_MEM_R_W, chunkSize, verifyBuffer);
			if (memcmp(progBuffer, verifyBuffer, chunkSize) != 0) {
				/*Serial.print("Block write verification error, bank ");
	                Serial.print(bank, DEC);
	                Serial.print(", address ");
	                Serial.print(address, DEC);
	                Serial.print("!\nExpected:");
	                for (j = 0; j < chunkSize; j++) {
	                    Serial.print(" 0x");
	                    if (progBuffer[j] < 16) Serial.print("0");
	                    Serial.print(progBuffer[j], HEX);
	                }
	                Serial.print("\nReceived:");
	                for (uint8_t j = 0; j < chunkSize; j++) {
	                    Serial.print(" 0x");
	                    if (verifyBuffer[i + j] < 16) Serial.print("0");
	                    Serial.print(verifyBuffer[i + j], HEX);
	                }
	                Serial.print("\n");*/
				free(verifyBuffer);
				return false; // uh oh.
			}
		}

		// increase byte index by [chunkSize]
		i += chunkSize;

		// uint8_t automatically wraps to 0 at 256
		address += chunkSize;

		// if we aren't done, update bank (if necessary) and address
		if (i < dataSize) {
			if (address == 0) bank++;
			setMemoryBank_Default(bank);
			setMemoryStartAddress(address);
		}
	}
	if (verify) free(verifyBuffer);
	return true;
}


bool writeDMPConfigurationSet(const uint8_t *data, uint16_t dataSize){

	uint8_t *progBuffer = 0;
	uint8_t success, special;
	uint16_t i;

	// config set data is a long string of blocks with the following structure:
	// [bank] [offset] [length] [byte[0], byte[1], ..., byte[length]]
	uint8_t bank, offset, length;
	for (i = 0; i < dataSize;) {
		bank = data[i++];
		offset = data[i++];
		length = data[i++];

		// write data or perform special action
		if (length > 0) {
			// regular block of data to write
			/*Serial.print("Writing config block to bank ");
	            Serial.print(bank);
	            Serial.print(", offset ");
	            Serial.print(offset);
	            Serial.print(", length=");
	            Serial.println(length);*/
			progBuffer = (uint8_t *)data + i;
			success = writeMemoryBlock(progBuffer, length, bank, offset, true);
			i += length;
		} else {
			// special instruction
			// NOTE: this kind of behavior (what and when to do certain things)
			// is totally undocumented. This code is in here based on observed
			// behavior only, and exactly why (or even whether) it has to be here
			// is anybody's guess for now.
			special = data[i++];
			/*Serial.print("Special command code ");
	            Serial.print(special, HEX);
	            Serial.println(" found...");*/
			if (special == 0x01) {
				// enable DMP-related interrupts

				//setIntZeroMotionEnabled(true);
				//setIntFIFOBufferOverflowEnabled(true);
				//setIntDMPEnabled(true);
				MPU6050_WriteByte(MPU6050_RA_INT_ENABLE, 0x32); // single operation

				success = true;
			} else {
				// unknown special command
				success = false;
			}
		}

		if (!success) {
			return false; // uh oh
		}
	}
	return true;
}


// DMP_CFG_1 register
uint8_t getDMPConfig1(){
	MPU6050_ReadByte(MPU6050_RA_DMP_CFG_1, buffer);
	return buffer[0];
}
void setDMPConfig1(uint8_t config){
	MPU6050_WriteByte(MPU6050_RA_DMP_CFG_1, config);
}

// DMP_CFG_2 register
uint8_t getDMPConfig2(){
	MPU6050_ReadByte(MPU6050_RA_DMP_CFG_2, buffer);
	return buffer[0];
}
void setDMPConfig2(uint8_t config){
	MPU6050_WriteByte(MPU6050_RA_DMP_CFG_2, config);
}

//***************************************************************************************
//**********************           Calibration Routines            **********************
//***************************************************************************************
/**
  @brief      Fully calibrate Gyro from ZERO in about 6-7 Loops 600-700 readings
*/

void CalibrateGyro_Default(){
	CalibrateGyro(15);
}
void CalibrateGyro(uint8_t Loops){
	double kP = 0.3;
	double kI = 90;
	float x;
	x = (100 - map(Loops, 1, 5, 20, 0)) * .01;
	kP *= x;
	kI *= x;

	PID( 0x43,  kP, kI,  Loops);
}
void CalibrateAccel_Default(){
	CalibrateAccel(15);
}
void CalibrateAccel(uint8_t Loops){

	float kP = 0.3;
	float kI = 20;
	float x;
	x = (100 - map(Loops, 1, 5, 20, 0)) * .01;
	kP *= x;
	kI *= x;
	PID( 0x3B, kP, kI,  Loops);
}
void PID(uint8_t ReadAddress, float kP,float kI, uint8_t Loops){
	uint8_t SaveAddress = (ReadAddress == 0x3B)?((getDeviceID() < 0x38 )? 0x06:0x77):0x13;

	int16_t  Data;
	float Reading;
	int16_t BitZero[3];
	uint8_t shift =(SaveAddress == 0x77)?3:2;
	float Error, PTerm, ITerm[3];
	int16_t eSample;
	uint32_t eSum;
	uint16_t gravity = 8192; // prevent uninitialized compiler warning
	if (ReadAddress == 0x3B) gravity = 16384 >> getFullScaleAccelRange();
	printf(">");
	for (int i = 0; i < 3; i++) {
		MPU6050_ReadWords(SaveAddress + (i * shift), 1, (uint16_t *)&Data); // reads 1 or more 16 bit integers (Word)
		Reading = Data;
		if(SaveAddress != 0x13){
			BitZero[i] = Data & 1;										 // Capture Bit Zero to properly handle Accelerometer calibration
			ITerm[i] = ((float)Reading) * 8;
		} else {
			ITerm[i] = Reading * 4;
		}
	}
	for (int L = 0; L < Loops; L++) {
		eSample = 0;
		for (int c = 0; c < 100; c++) {// 100 PI Calculations
			eSum = 0;
			for (int i = 0; i < 3; i++) {
				MPU6050_ReadWords(ReadAddress + (i * 2), 1, (uint16_t *)&Data); // reads 1 or more 16 bit integers (Word)
				Reading = Data;
				if ((ReadAddress == 0x3B)&&(i == 2)) Reading -= gravity;	//remove Gravity
				Error = -Reading;
				eSum += abs(Reading);
				PTerm = kP * Error;
				ITerm[i] += (Error * 0.001) * kI;				// Integral term 1000 Calculations a second = 0.001
				if(SaveAddress != 0x13){
					Data = round((PTerm + ITerm[i] ) / 8);		//Compute PID Output
					Data = ((Data)&0xFFFE) |BitZero[i];			// Insert Bit0 Saved at beginning
				} else Data = round((PTerm + ITerm[i] ) / 4);	//Compute PID Output
				MPU6050_WriteWords(SaveAddress + (i * shift), 1, (uint16_t *)&Data);
			}
			if((c == 99) && eSum > 1000){						// Error is still to great to continue
				c = 0;
				printf("*");
			}
			if((eSum * ((ReadAddress == 0x3B)?.05: 1)) < 5) eSample++;	// Successfully found offsets prepare to  advance
			if((eSum < 100) && (c > 10) && (eSample >= 10)) break;		// Advance to next Loop
			HAL_Delay(1);
		}
		printf(".");
		kP *= .75;
		kI *= .75;
		for (int i = 0; i < 3; i++){
			if(SaveAddress != 0x13) {
				Data = round((ITerm[i] ) / 8);		//Compute PID Output
				Data = ((Data)&0xFFFE) |BitZero[i];	// Insert Bit0 Saved at beginning
			} else Data = round((ITerm[i]) / 4);
			MPU6050_WriteWords(SaveAddress + (i * shift), 1, (uint16_t *)&Data);
		}
	}
	resetFIFO();
	resetDMP();
}
void PrintActiveOffsets(){
	GetActiveOffsets();
	//	A_OFFSET_H_READ_A_OFFS(Data);
	printf("%.5f,\t",(float)offsets[0]);
	printf("%.5f,\t",(float)offsets[1]);
	printf("%.5f,\t",(float)offsets[2]);

	//	XG_OFFSET_H_READ_OFFS_USR(Data);
	printf("%.5f,\t",(float)offsets[3]);
	printf("%.5f,\t",(float)offsets[4]);
	printf("%.5f\n\n",(float)offsets[5]);

	fflush(stdout);
}
int16_t * GetActiveOffsets(){
	uint8_t AOffsetRegister = (getDeviceID() < 0x38 )? MPU6050_RA_XA_OFFS_H:0x77;
	if(AOffsetRegister == 0x06)	MPU6050_ReadWords(AOffsetRegister, 3, (uint16_t *)offsets);
	else {
		MPU6050_ReadWords(AOffsetRegister, 1, (uint16_t *)offsets);
		MPU6050_ReadWords(AOffsetRegister+3, 1, (uint16_t *)(offsets+1));
		MPU6050_ReadWords(AOffsetRegister+6, 1, (uint16_t *)(offsets+2));
	}
	MPU6050_ReadWords(0x13, 3, (uint16_t *)(offsets+3));
	return offsets;
}

long map(long x, long in_min, long in_max, long out_min, long out_max){
	long output =(x- in_min)*(out_max - out_min)/(in_max - in_min) + out_min;
	return output;
}


