/*
 * IMU.hpp
 *
 *  Created on: Sep 26, 2022
 *      Author: efmgh
 */

#ifndef INC_IMU_HPP_
#define INC_IMU_HPP_

#define STORE_NUM 5
#define R_IMU 0.01 //0.03 Lowpath filter constant. The smaller it is, the more effective/

class IMU{
private:
	//int16_t xa_store_[STORE_NUM], ya_store_[STORE_NUM], za_store_[STORE_NUM];
	//int16_t xg_store_[STORE_NUM], yg_store_[STORE_NUM], zg_store_[STORE_NUM];
	int16_t xa_, ya_, za_, xg_, yg_, zg_;
	//uint16_t array_idx;
	float offset_;

public:
	IMU();
	void init();
	//void storeValues();
	void updateValues();
	float getOmega();
	void calibration();
	float getOffsetVal();

};

#endif /* INC_IMU_HPP_ */
