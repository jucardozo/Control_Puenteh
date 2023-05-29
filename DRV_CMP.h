/*
 * DRV_CMP.h
 *
 *  Created on: 27 may. 2023
 *      Author: Salta
 */

#ifndef DRV_CMP_H_
#define DRV_CMP_H_
#include "PORT.h"
#include <stdbool.h>


typedef enum{
	CMP_IN0=0,
	CMP_IN1,
	CMP_IN2,
	CMP_IN3,
	CMP_IN4,
	CMP_IN5,
	CMP_IN6,
	CMP_IN7
}CMP_MUX_t;


typedef CMP_Type *CMP_t;
void Init_CMP(CMP_t cmp); //unica funcionalidad => Continous Mode


#endif /* DRV_CMP_H_ */
