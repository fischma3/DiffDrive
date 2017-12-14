/** ***************************************************************************
 * @file
 * @brief Motor Control Functions for Drive
 *
 *
 * @note
 *
 * Device:
 *
 * <b>Changelog</b>
 * Date       | Author  | Changes
 * :--------: | :-----: | :----------------------------------------------------------:
 * 13.12.2017 | ficm    | File created
 *
 * @author ficm
 *****************************************************************************/

#ifndef MOTORS_MOTOR_CTRL_H_
#define MOTORS_MOTOR_CTRL_H_

#include "IQmathLib.h"

/******************************************************************************
 * Defines
 *****************************************************************************/
#define SPEED_CTRL_C0			1
#define SPEED_CTRL_C1			1

#define CURRENT_CTRL_C0			1
#define CURRENT_CTRL_C1			1

///Struct for a PI Controller
struct piCtrl_t {
	float e;				///< Regelfehler
	float e0;				///< Regelfehler z^-1
	float u;				///< Steuergr�sse
	float u0;				///< Steuergr�sse z^-1
	float uMin;				///< Minimale Steuergr�sse
	float uMax;				///< Maximale Steuergr�sse
	float c0;				///<
	float c1;				///<
};

#define PI_CTRL_DEFAULTS {0, 0, 0, 0, 0, 0, 0, 0}
#define SPEED_CTRL_DEFAULTS {0, 0, 0, 0, -0.95, 0.95, SPEED_CTRL_C0, SPEED_CTRL_C1}
#define CURRENT_CTRL_ID_DEFAULTS {0, 0, 0, 0, -0.43, 0.43, CURRENT_CTRL_C0, CURRENT_CTRL_C1}
#define CURRENT_CTRL_IQ_DEFAULTS {0, 0, 0, 0, -0.9, 0.9, CURRENT_CTRL_C0, CURRENT_CTRL_C1}


///Struct for a Rate Limiter
struct rateLim_t {
	_iq15 in;				///< Input for the rate Limiter
	_iq15 out;				///< Output of the rate Limiter
	_iq15 slope;			///< Slope per call of the rate Limiter
};

#define RATE_LIM_DEFAULTS {0, 0, 0}

/******************************************************************************
 * Variables
 *****************************************************************************/
extern struct piCtrl_t speedCtrl_Motor1;
extern struct piCtrl_t speedCtrl_Motor2;
extern struct piCtrl_t currentCtrl_Id_Motor1;
extern struct piCtrl_t currentCtrl_Iq_Motor1;
extern struct piCtrl_t currentCtrl_Id_Motor2;
extern struct piCtrl_t currentCtrl_Iq_Motor2;
/******************************************************************************
 * Functions
 *****************************************************************************/
void piController(struct piCtrl_t* piCtrl);
void rateLimiter(struct rateLim_t* rateLim);

#endif /* MOTORS_MOTOR_CTRL_H_ */
