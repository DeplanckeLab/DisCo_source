// DisCo headerfile. Contains includes, default variable definitions of commonly changed variables, function prototypes, and defines global variables.

#ifndef DISCO_H
#define DISCO_H

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <stddef.h>
#include <windows.h>
#include "xiApi.h" 
#include <ctime>
#include <math.h>
#include <string>
#include <thread>
#include <memory.h>
#include <vector>
#include <algorithm>
#include "ValveController.h"
#include <Elveflow64.h> 
#include <fstream>
#include <map>

// Compiler instruction to include calibration menus
// Set to false to exclude from compile

#define _WARMUP_MENU true
#define _OSCILLATION_DETERM true
#define _DROPLET_DETERM true

// Path to the OB1 calibration file

#define OBCALIBFILE "D:\\CodeDev\\DISCO_V3.2\\calib"

// Define camera exposure. Adjust if necessary

#define CAM_EXPOSURE 300

// Dropleting default settings

#define CLEAN_PRESSURE 460
#define DROPLET_PRESSURE 650

// Default oscillation values

#define OSCI_TIME 10
#define OSCI_DELTA_TIME 0

// Define optical properties of particles

#define CELLSIZE 80
#define CELLCIRCULARITY 17
#define CELLCONTRAST 35

#define BEADSIZE 500
#define BEADCIRCULARITY 16
#define BEADCONTRAST 40

// Define ROIs for cell/bead stopping


// Function prototypes

void pressure_ramp(int pres, int time, int channel, int &MyOB1_ID);
void valve_oscilate(int *oscilate, int &modbus_con, time_t &recov_time, int &osc_const, int &modbus_conoc);
void monitor_area(std::vector<XI_IMG> *image_array, std::vector<cv::Mat> *proc_array, int *state, std::mutex &mut_img_loaded, int *ROI_id, int *old_ROI_id, int *bufcoord, std::vector<cv::Rect> ROI_array, std::mutex &state_write, std::mutex &buffer_fill, int thread_id, std::condition_variable &state_change, int *part_size, int *contrast, int &circ, int &channel_id, bool *monitor_on);
void get_images(std::vector<XI_IMG> *image_array);
int particledetect(int minsize, cv::Mat last_frame, cv::Mat current_frame, int sens, int circ, int &channel_id);
cv::Mat processimage(cv::Rect detectROI, std::vector<XI_IMG> *image_array, int time);
void control_channel(int *state, int *stateoc, int &MyOB1_ID, std::mutex &mut, std::condition_variable &cond_var, int &modbus_con, std::condition_variable &cond_var_sleep, std::mutex &sleepmut, int &Pressure, int &channel, int &other_channel, int minsize, int sens, int &modbus_conoc, std::vector<XI_IMG> *image_array, std::vector<cv::Rect> ROI_array, int &leadch, int &circ);
void eject(int &MyOB1_ID, int &state_c1, int &state_c2, bool *drop_mode_analyze);
void determ_osci(int &MyOB1_ID, std::mutex &mut, std::condition_variable &cond_var, int &modbus_con, int &Pressure, int &channel, int minsize, int sens, int &modbus_conoc, std::vector<XI_IMG> *image_array, std::vector<cv::Rect> ROI_array, int &circ, bool *finished);
void determ_drop(int &MyOB1_ID, std::mutex &mut, std::condition_variable &cond_var, int &modbus_con, int &Pressure, int &channel, int minsize, int sens, int &modbus_conoc, std::vector<XI_IMG> *image_array, std::vector<cv::Rect> ROI_array, int &circ, bool *finished, int *start, int *drop_test);

// global variables

extern double *Calibration; 

//extern int speed_celljump;
extern int *cel_jump;

//extern int speed_beadjump;
extern int *bead_jump;

extern int pressio[];
extern int press_fast[];

extern int osc_var;
extern int delta_celljump;

extern ValveController valves;

extern int PressureC1;
extern int PressureC2;

extern int current_time;
extern int last_time;

extern int process_pause;
extern int fps;

extern std::mutex m1;
extern std::mutex m2;
extern std::mutex m3;

extern std::mutex c1_sleep;
extern std::mutex c2_sleep;
extern std::mutex mut_eject;

extern std::condition_variable cond_var_c1_sleep;
extern std::condition_variable cond_var_c2_sleep;

extern std::condition_variable cond_var_c1;
extern std::condition_variable cond_var_c2;

extern std::condition_variable eject_cond_var;

extern std::condition_variable cond_var_img_loaded;

extern bool imgloadready[];

extern bool eject_start;

extern int pres_clean_nozzle;
extern int pres_make_drop;

extern bool leadchecked;

extern std::map<int, int> state_map;

#endif