#ifndef VALVECONTROLLER_H
#define VALVECONTROLLER_H

#include <NIDAQmx.h>

#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else

class ValveController {
private:
	TaskHandle valve[8];
	uInt8 state_on = 1;
	uInt8 state_off = 0;
	int32 written;
	void create_valve(TaskHandle taskHandle[]);
public:
	ValveController();
	void set_state(int valve_id, int state);
};

#endif