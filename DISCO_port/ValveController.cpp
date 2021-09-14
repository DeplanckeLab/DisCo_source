// Valve controller class to wrap the NIDAQmx class.

#include "ValveController.h"

ValveController::ValveController() {
	create_valve(valve);
}

void ValveController::set_state(int valve_id, int state) {
	if (state == 1) {
		DAQmxWriteDigitalLines(this->valve[valve_id], 1, 1, 10.0, DAQmx_Val_GroupByChannel, &this->state_on, &this->written, NULL);
	}
	if (state == 0) {
		DAQmxWriteDigitalLines(this->valve[valve_id], 1, 1, 10.0, DAQmx_Val_GroupByChannel, &this->state_off, &this->written, NULL);
	}
}

void ValveController::create_valve(TaskHandle taskHandle[]) {
	DAQmxCreateTask("", &taskHandle[0]);
	DAQmxCreateTask("", &taskHandle[1]);
	DAQmxCreateTask("", &taskHandle[2]);
	DAQmxCreateTask("", &taskHandle[3]);
	DAQmxCreateTask("", &taskHandle[4]);
	DAQmxCreateTask("", &taskHandle[5]);
	DAQmxCreateTask("", &taskHandle[6]);
	DAQmxCreateTask("", &taskHandle[7]);

	const char* output_pin1 = "Dev1/port0/line7";
	const char* output_pin2 = "Dev1/port0/line6";
	const char* output_pin3 = "Dev1/port0/line5";
	const char* output_pin4 = "Dev1/port0/line4";
	const char* output_pin5 = "Dev1/port0/line3";
	const char* output_pin6 = "Dev1/port0/line2";
	const char* output_pin7 = "Dev1/port0/line1";
	const char* output_pin8 = "Dev1/port0/line0";

	DAQmxCreateDOChan(taskHandle[0], output_pin1, "", DAQmx_Val_ChanForAllLines);
	DAQmxCreateDOChan(taskHandle[1], output_pin2, "", DAQmx_Val_ChanForAllLines);
	DAQmxCreateDOChan(taskHandle[2], output_pin3, "", DAQmx_Val_ChanForAllLines);
	DAQmxCreateDOChan(taskHandle[3], output_pin4, "", DAQmx_Val_ChanForAllLines);
	DAQmxCreateDOChan(taskHandle[4], output_pin5, "", DAQmx_Val_ChanForAllLines);
	DAQmxCreateDOChan(taskHandle[5], output_pin6, "", DAQmx_Val_ChanForAllLines);
	DAQmxCreateDOChan(taskHandle[6], output_pin7, "", DAQmx_Val_ChanForAllLines);
	DAQmxCreateDOChan(taskHandle[7], output_pin8, "", DAQmx_Val_ChanForAllLines);

	DAQmxStartTask(taskHandle[0]);
	DAQmxStartTask(taskHandle[1]);
	DAQmxStartTask(taskHandle[2]);
	DAQmxStartTask(taskHandle[3]);
	DAQmxStartTask(taskHandle[4]);
	DAQmxStartTask(taskHandle[5]);
	DAQmxStartTask(taskHandle[6]);
	DAQmxStartTask(taskHandle[7]);
}