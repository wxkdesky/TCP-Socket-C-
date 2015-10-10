// clkAndsync.cpp : 定义 DLL 应用程序的导出函数。
//

#include "stdafx.h"
#include "clkAndsync.h"
#include <stdio.h>
#define log_writef printf


#define chiSquareMax 150.0

static const double CLOCK_PERIOD_SEC = (double)(0x10000000000LL) / (64.0*998.4e6);
static const double ONE_OVER_ROOT_TWO = 0.70710678118654752440084436210485;

//CS_Kalman Constructor - initialize the kalman filter variables
CS_Kalman::CS_Kalman(void)
{
	measNoiseVar = 3e-20;
	processNoiseVar = 5e-20;
	prevTxTime = 0.0;
	init = false;
	counter = 0;
	outlier_counter = 0;

	x0 = 0.0;
	x1 = 1.0;
	p0 = p1 = p2 = p3 = 0.0;
}

//CS_Kalman "process" - run Kalman filter
//parameters: timeRxs - slave anchor time in seconds of CCP frame RX
//            timeTxs - time in senconds that is sum of master anchor CCP frame TX timestamp
//                      and the constant TOF (fixed offset) between master and anchor

void CS_Kalman::process(double timeRxs, double timeTxs)
{
	if (counter < 50)
		counter++;
	if (init)// if initlization has been done
	{
		double k0, k1;
		double Y, outlierMetric = 0;
		double p_tmp;
		double _x0, _p0, _p1, _p2, _p3;

		dt = fmod(timeTxs, CLOCK_PERIOD_SEC) - prevTxTime;
		outlier = 0;

		if (dt<0)
			dt += CLOCK_PERIOD_SEC;

		// Save x0 and p to restore in case of outlier
		_x0 = x0;
		_p0 = p0;
		_p1 = p1;
		_p2 = p2;
		_p3 = p3;

		//Prediction for state vector and covariance
		//x = A*x;
		//[x0;x1] = [1,dt;0,1][x0;x1]
		x0 = dt*x1 + x0;
		x0 = fmod(x0, CLOCK_PERIOD_SEC);

		//P = A*P*trans(A)+Q;
		p0 = (dt*dt*processNoiseVar)*0.5 + dt*(dt*p3 + p2) + dt*p1 + p0;
		p_tmp = ONE_OVER_ROOT_TWO*dt*processNoiseVar + dt*p3;
		p1 = p_tmp + p1;
		p2 = p_tmp + p2;
		p3 = processNoiseVar + p3;

		measuredError = fmod((timeRxs - x0 + CLOCK_PERIOD_SEC*0.5), CLOCK_PERIOD_SEC) - CLOCK_PERIOD_SEC*0.5;

		//y = 1/(H*P*trans(H)+R)
		Y = 1.0 / (measNoiseVar + p0);
		outlier = measuredError*Y*measuredError;

		if ((outlierMetric>chiSquareMax) && (counter>30))
		{
			outlier = 1;
			measuredError = 0.0;
			outlier_counter++;
			if (outlier_counter>9)
			{
				outlier_counter = 0;
				counter = 0;
			}
			x0 = _x0;
			p0 = _p0;
			p1 = _p1;
			p2 = _p2;
			p3 = _p3;
		}
		else
		{
			// Compute Kalman gain factor
			// K = P*trans(H)*inv(H*P*trans(H)+R)
			k0 = p0 / (measNoiseVar + p0);
			k1 = p2 / (measNoiseVar + p0);

			// Correction based on observation
			// x = x+K*measuredError
			x0 = k0*measuredError + x0;
			x1 = k1*measuredError + x1;

			if (x0<0.0)
				x0 += CLOCK_PERIOD_SEC;
			else
				x0 = fmod(x0, CLOCK_PERIOD_SEC);

			if (outlier_counter>0)
				outlier_counter--;

			// Covariance Update
			// P = P-K*H*P;

			p3 = p3 - k1*p1;
			p2 = p2 - k1*p0;
			p1 = p1 - k0*p1;
			p0 = p0 - k0*p0;

			prevTxTime = fmod(timeTxs, CLOCK_PERIOD_SEC);
		}
	}
	else
	{
		x0 = timeRxs;
		x1 = 1.0;
		init = false;
		prevTxTime = fmod(timeTxs, CLOCK_PERIOD_SEC);
	}
}//end process


//CS_Kalman syncTOA function - convert slave blink TOA to anchor time base
//parameters: blinkTime - slave anchor time of receiving a blink frame
//            returns = blink time converted to master anchor timebase

double CS_Kalman::syncTOA(double blinkTime)
{
	double timeSinceLast = blinkTime - x0;
	if (timeSinceLast<0)
		timeSinceLast += CLOCK_PERIOD_SEC;
	return fmod(prevTxTime + timeSinceLast / x1, CLOCK_PERIOD_SEC);
}

// CS_Kalman log function - log data for performance checking
// called immediately after call to 'process' function if we are logging 
// data to check that wireless sync is operating correctly
// parameters: timeRxs = as used in the call to the 'process' function
//             timeTxs = as used in the call to the 'process' function
//             seqnum = frame sequence number (so can see any missing CCP frames)
void CS_Kalman::log(double timeRxs, double timeTxs, int seqnum)
{
	if (outlier)
		log_writef("CS_Kalman rejected:"
		"%d %3d %+15.15e %+15.15e %+15.15e %+15.15e %+15.15e %+15.15e\n",
		id, seqnum, timeTxs, timeRxs, dt, measuredError, x0, x1);
	else
		log_writef("CS_Kalman rejected:"
		"%d %3d %+15.15e %+15.15e %+15.15e %+15.15e %+15.15e %+15.15e\n",
		id, seqnum, timeTxs, timeRxs, dt, measuredError, x0, x1);
}// end log
void CS_Kalman::test(int a)
{
	cout << a;
}