/*****************************************************************
 * ina219_cal.h
 *
 *  Created on: Nov 24, 2010
 *      Author: hcl
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef INA219_CAL_H_
#define INA219_CAL_H_
/*****************************************************************
 *
 * Calibration data
 * @shunt_mohm 		: Shunt resistor in milliohms
 * @max_current_mamp: Max rail current in milliamps
 * @vbus_max_mvolt	: Max rail voltage in millivolts
 */
struct ina219_cal {
	u32 shunt_mohm;
	u32 max_current_mamp;
	u32 vbus_max_mvolt;
};
#endif /* INA219_CAL_H_ */
