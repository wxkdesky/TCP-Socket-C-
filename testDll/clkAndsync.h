#include <math.h>
#include <iostream>
using namespace std;
#ifndef _DECA_CSKALMAN_H
#define _DECA_CSKALMAN_H

extern "C"{
	class  _declspec(dllexport) CS_Kalman
	{
	public:
		//Constructor
		CS_Kalman(void);

		void process(double timeRxs, double timeTxs);
		double syncTOA(double blinkTime);

		void log(double timeRxs, double timeTxs, int seqnum);

		void test(int a);

		double measNoiseVar;
		double processNoiseVar;
		double prevTxTime;
		int counter;
		int outlier_counter;

		bool init;
		int id;

		//x = state vector estimate
		//p = covariance of the state vector estimate
		double x0, x1;
		double p0, p1, p2, p3;

		//logging
		double dt, measuredError;
		int outlier;
	};
}
#endif
