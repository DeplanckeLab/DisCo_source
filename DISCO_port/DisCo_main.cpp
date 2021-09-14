// Main of the DisCo code. Sets up submenus, sets up channel detection threads, and globally coordinates both channels (e.g. ejection)

#include"DisCo.h"

double *Calibration = new double[1000];

// Pressures for oscillation particle placement
int speed_celljump = 40;
int *cel_jump = &speed_celljump;
int speed_beadjump = 100;
int *bead_jump = &speed_beadjump;

int pressio[] = { 0,*bead_jump,*cel_jump }; //Particle placement pressures
int press_fast[] = {0, 800, 300};			//Particle detection pressures

// Oscillation times
int osc_var = OSCI_TIME;
int delta_celljump = OSCI_DELTA_TIME;

ValveController valves;

int PressureC1 = pressio[1];
int PressureC2 = pressio[2];

// Variables for dropleting
int pres_clean_nozzle =	CLEAN_PRESSURE; // inital pressure to clean encapsulation area
int pres_make_drop = DROPLET_PRESSURE; // droplet generation pressure

int fps = 0;

int current_time = 0;
int last_time = 0;
int process_pause = 0;

// Condition variables and mutex locks for multithreading.

std::mutex m1;
std::mutex m2;
std::mutex m3;

std::mutex c1_sleep;
std::mutex c2_sleep;
std::mutex mut_eject;

std::condition_variable cond_var_c1_sleep;
std::condition_variable cond_var_c2_sleep;

std::condition_variable cond_var_c1;
std::condition_variable cond_var_c2;

std::condition_variable eject_cond_var;

std::condition_variable cond_var_img_loaded;

bool imgloadready[] = {false, false, false};

bool eject_start(false);

bool leadchecked = false;

//Analytical variables

std::map<int, int> state_map;

int main() {

	// setup OB1 controller

	int MyOB1_ID;
	int error = 0;
	MyOB1_ID = -1;
	error = OB1_Initialization("01CB2A13", Z_regulator_type__0_2000_mbar, Z_regulator_type__0_2000_mbar, Z_regulator_type__0_2000_mbar, Z_regulator_type_none, &MyOB1_ID);
	std::cout << "OB1_ID: " << error << std::endl;

	// load the file contain in the path, if no path or non valid path, prompt ask the user to chose the file
	Elveflow_Calibration_Load(OBCALIBFILE, Calibration, 1000);

	HWND console_window = GetConsoleWindow();
	SetWindowPos(console_window, 0, 1120, 500, 800, 500, SWP_NOZORDER);

	//initializing stable valve state
	valves.set_state(0, 1);
	valves.set_state(1, 1);
	valves.set_state(2, 0);
	valves.set_state(3, 1);
	valves.set_state(4, 0);
	valves.set_state(5, 1);
	valves.set_state(6, 0);
	valves.set_state(7, 0);

	// Define detection regions
	std::vector<cv::Rect> ROI_c1;
	std::vector<cv::Rect> ROI_c2;

	cv::Rect myROI1_c1(300, 75, 300, 80); 
	cv::Rect myROI2_c1(800, 20, 200, 170); 

	ROI_c1.push_back(myROI1_c1);
	ROI_c1.push_back(myROI2_c1);

	cv::Rect myROI1_c2(0, 620, 900, 80); 
	cv::Rect myROI2_c2(800, 575, 200, 170); 

	ROI_c2.push_back(myROI1_c2);
	ROI_c2.push_back(myROI2_c2);

	cv::Rect ROI_stop_c1(240, 220, 100, 140); 									 
	cv::Rect ROI_stop_c2(125, 430, 100, 130);

	ROI_c1.push_back(ROI_stop_c1);
	ROI_c2.push_back(ROI_stop_c2);

	cv::Rect ROI_check_c1(200, 220, 140, 140); 
	cv::Rect ROI_check_c2(0, 430, 350, 130); 

	ROI_c1.push_back(ROI_check_c1);
	ROI_c2.push_back(ROI_check_c2);

	cv::Rect recordROI(0, 220, 1000, 350);

	// Define state variables for both channels
	int state_c1 = 0; // 0 = running, 1 = stop
	int *state_c1_ref = &state_c1;
	int state_c2 = 0; // 0 = running, 1 = stop
	int *state_c2_ref = &state_c2;

	int valveid_c1 = 0;
	int valveid_c2 = 1;

	int pressurechannel1 = 1;
	int pressurechannel2 = 2;

	// Setup camera and image buffer

	std::vector<XI_IMG> *image_array = new std::vector<XI_IMG>(10);
	
	memset(&image_array->at(0), 0, sizeof(image_array[0]));
	image_array->at(0).size = sizeof(XI_IMG);
	memset(&image_array->at(1), 0, sizeof(image_array[1]));
	image_array->at(1).size = sizeof(XI_IMG);
	memset(&image_array->at(2), 0, sizeof(image_array[2]));
	image_array->at(2).size = sizeof(XI_IMG);
	memset(&image_array->at(3), 0, sizeof(image_array[3]));
	image_array->at(3).size = sizeof(XI_IMG);
	memset(&image_array->at(4), 0, sizeof(image_array[4]));
	image_array->at(4).size = sizeof(XI_IMG);
	memset(&image_array->at(5), 0, sizeof(image_array[5]));
	image_array->at(5).size = sizeof(XI_IMG);
	memset(&image_array->at(6), 0, sizeof(image_array[6]));
	image_array->at(6).size = sizeof(XI_IMG);
	memset(&image_array->at(7), 0, sizeof(image_array[7]));
	image_array->at(7).size = sizeof(XI_IMG);
	memset(&image_array->at(8), 0, sizeof(image_array[8]));
	image_array->at(8).size = sizeof(XI_IMG);
	memset(&image_array->at(9), 0, sizeof(image_array[9]));
	image_array->at(9).size = sizeof(XI_IMG);

	std::thread camera(get_images, image_array);

	Sleep(1000);

	// Defining which channel stops a particle first, i.e. is lead channel
	int leadch1 = 1;
	int leadch2 = 0;

	// Loading optical properties
	int part_size_ch1 = BEADSIZE; 
	int part_size_ch2 = CELLSIZE; 

	int contrast_ch1 = BEADCONTRAST;
	int contrast_ch2 = CELLCONTRAST;

	int circ_ch1 = BEADCIRCULARITY;
	int circ_ch2 = CELLCIRCULARITY;

	int pause = 0;

	cv::Mat temp_frame_dis;

	std::mutex view_img;

	bool oscidet_finished = FALSE;
	bool *finished_pnt = &oscidet_finished;

	bool drop_mode_analyze = TRUE;
	bool* drop_mode_analyze_ref = &drop_mode_analyze;
	std::thread eject_t(eject,
		std::ref(MyOB1_ID),
		std::ref(state_c1),
		std::ref(state_c2),
		std::ref(drop_mode_analyze_ref));

	#if _WARMUP_MENU
	// This section generates the valve warm up routine menu.
	{
		cv::namedWindow("Live feed", cv::WINDOW_AUTOSIZE);
		cv::moveWindow("Live feed", 20, 20);

		int valve_warm = 0;
		int state_valve_warm = 0;
		cv::namedWindow("Start", cv::WINDOW_AUTOSIZE);
		cv::moveWindow("Start", 1120, 20);

		cv::createTrackbar("Start:", "Start", &pause, 1);
		cv::createTrackbar("Warm valve:", "Start", &valve_warm, 1);

		std::vector<int> channels = { 1,2,3,4 };

		while (pause == 0) {
			std::unique_lock<std::mutex> lk(view_img);
			cond_var_img_loaded.wait(lk);

			temp_frame_dis = cv::Mat(image_array->at(current_time).height, image_array->at(current_time).width, CV_8UC1, image_array->at(current_time).bp);

			cv::rectangle(temp_frame_dis, myROI1_c1, cv::Scalar(255, 255, 255), 0.8, 8, 0);
			cv::rectangle(temp_frame_dis, myROI2_c1, cv::Scalar(255, 255, 255), 0.8, 8, 0);

			cv::rectangle(temp_frame_dis, ROI_stop_c1, cv::Scalar(255, 255, 255), 0.8, 8, 0);
			cv::rectangle(temp_frame_dis, ROI_stop_c2, cv::Scalar(255, 255, 255), 0.8, 8, 0);

			putText(temp_frame_dis, cv::format("frame=%6d", fps), cv::Point(300, 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255));
			arrowedLine(temp_frame_dis, cv::Point(390, 392), cv::Point(440, 392), cv::Scalar(0, 0, 255), 3, 8, 0, 0.3);


			if (valve_warm == 1) {
				if (state_valve_warm == 0) {
					valves.set_state(0, 1);
					valves.set_state(1, 1);
					valves.set_state(2, 0);
					state_valve_warm = 1;
					Sleep(100);
				}
				if (state_valve_warm == 1) {
					valves.set_state(0, 0);
					valves.set_state(1, 0);
					valves.set_state(2, 1);
					state_valve_warm = 0;
				}
			}

			imshow("Live feed", temp_frame_dis);
			cv::waitKey(30);
		}


		cv::destroyWindow("Start");

		pause = 0;
	}
	#endif

	#if _OSCILLATION_DETERM
	// This section generates the oscillation value adjustment menu.
	{
	cv::namedWindow("Osci Determ", cv::WINDOW_AUTOSIZE);
	cv::moveWindow("Osci Determ", 1120, 20);

	int SetOsc = 0;
	int AutomDetem = 0;

	cv::createTrackbar("Start:", "Osci Determ", &pause, 1);
	cv::createTrackbar("Manual Osc:", "Osci Determ", &SetOsc, 25);
	cv::createTrackbar("Autom Determ:", "Osci Determ", &AutomDetem, 1);

	valves.set_state(5, 1);
	valves.set_state(4, 0);

	while (pause == 0) {
		std::unique_lock<std::mutex> lk(view_img);
		cond_var_img_loaded.wait(lk);

		temp_frame_dis = cv::Mat(image_array->at(current_time).height, image_array->at(current_time).width, CV_8UC1, image_array->at(current_time).bp);

		cv::rectangle(temp_frame_dis, myROI1_c1, cv::Scalar(255, 255, 255), 0.8, 8, 0);
		cv::rectangle(temp_frame_dis, myROI2_c1, cv::Scalar(255, 255, 255), 0.8, 8, 0);

		cv::rectangle(temp_frame_dis, ROI_stop_c1, cv::Scalar(255, 255, 255), 0.8, 8, 0);
		cv::rectangle(temp_frame_dis, ROI_stop_c2, cv::Scalar(255, 255, 255), 0.8, 8, 0);

		putText(temp_frame_dis, cv::format("frame=%6d", fps), cv::Point(300, 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255));
		arrowedLine(temp_frame_dis, cv::Point(390, 392), cv::Point(440, 392), cv::Scalar(0, 0, 255), 3, 8, 0, 0.3);

		imshow("Live feed", temp_frame_dis);
		cv::waitKey(30);

	}

	if (AutomDetem == 0 && SetOsc != 0) {
		osc_var = SetOsc;
	}

	if (AutomDetem == 1) {
		std::thread oscivar_det(determ_osci,
			std::ref(MyOB1_ID),
			std::ref(m1),
			std::ref(cond_var_c1),
			std::ref(valveid_c1),
			std::ref(PressureC1),
			std::ref(pressurechannel1),
			part_size_ch1,
			contrast_ch1,
			std::ref(valveid_c2),
			image_array,
			ref(ROI_c1),
			std::ref(circ_ch1),
			std::ref(finished_pnt)
		);

		while (*finished_pnt != TRUE) {
			std::unique_lock<std::mutex> lk(view_img);
			cond_var_img_loaded.wait(lk);

			temp_frame_dis = cv::Mat(image_array->at(current_time).height, image_array->at(current_time).width, CV_8UC1, image_array->at(current_time).bp);

			cv::rectangle(temp_frame_dis, myROI1_c1, cv::Scalar(255, 255, 255), 0.8, 8, 0);
			cv::rectangle(temp_frame_dis, myROI2_c1, cv::Scalar(255, 255, 255), 0.8, 8, 0);

			cv::rectangle(temp_frame_dis, ROI_stop_c1, cv::Scalar(255, 255, 255), 0.8, 8, 0);
			cv::rectangle(temp_frame_dis, ROI_stop_c2, cv::Scalar(255, 255, 255), 0.8, 8, 0);

			imshow("Live feed", temp_frame_dis);
			cv::waitKey(30);
		}

		oscivar_det.join();
	}

	cv::destroyWindow("Osci Determ");

	pause = 0;
	}

	#endif

	#if _DROPLET_DETERM
	// This section generates the droplet  value adjustment menu.
	{
		int drop_mode_stop = 0;
		int *drop_mode_stop_pnt = &drop_mode_stop;
		int drop_mode_start = 0;
		int *drop_mode_start_pnt = &drop_mode_start;

		std::thread eject_t(eject,
			std::ref(MyOB1_ID),
			std::ref(state_c1),
			std::ref(state_c2),
			std::ref(drop_mode_analyze_ref));

		cv::namedWindow("Droplet test", cv::WINDOW_AUTOSIZE);
		cv::moveWindow("Droplet test", 1120, 20);

		cv::createTrackbar("StopDrop:", "Droplet test", &drop_mode_stop, 1);
		cv::createTrackbar("Nozzle Clean Pres:", "Droplet test", &pres_clean_nozzle, 600);
		cv::createTrackbar("Drop form pressyre:", "Droplet test", &pres_make_drop, 1000);
		cv::createTrackbar("StartDrop:", "Droplet test", &drop_mode_start, 1);


		Sleep(500);

		std::thread determ_drop_thread(determ_drop,
			std::ref(MyOB1_ID),
			std::ref(m1),
			std::ref(cond_var_c1),
			std::ref(valveid_c1),
			std::ref(PressureC1),
			std::ref(pressurechannel1),
			part_size_ch1,
			contrast_ch1,
			std::ref(valveid_c2),
			image_array,
			ref(ROI_c1),
			std::ref(circ_ch1),
			std::ref(finished_pnt),
			std::ref(drop_mode_start_pnt),
			std::ref(drop_mode_stop_pnt)
			);

		Sleep(100);

		while (drop_mode_stop == 0) {
			std::unique_lock<std::mutex> lk(view_img);
			cond_var_img_loaded.wait(lk);

			temp_frame_dis = cv::Mat(image_array->at(current_time).height, image_array->at(current_time).width, CV_8UC1, image_array->at(current_time).bp);

			cv::rectangle(temp_frame_dis, myROI1_c1, cv::Scalar(255, 255, 255), 0.8, 8, 0);
			cv::rectangle(temp_frame_dis, myROI2_c1, cv::Scalar(255, 255, 255), 0.8, 8, 0);

			cv::rectangle(temp_frame_dis, ROI_stop_c1, cv::Scalar(255, 255, 255), 0.8, 8, 0);
			cv::rectangle(temp_frame_dis, ROI_stop_c2, cv::Scalar(255, 255, 255), 0.8, 8, 0);

			imshow("Live feed", temp_frame_dis);
			cv::waitKey(30);
		}

		drop_mode_start = 1;

		drop_mode_analyze = FALSE;
		determ_drop_thread.join();
		cv::destroyWindow("Droplet test");

		cv::namedWindow("Start", cv::WINDOW_AUTOSIZE);

		cv::createTrackbar("Start:", "Start", &pause, 1);

		while (pause == 0) {
			std::unique_lock<std::mutex> lk(view_img);
			cond_var_img_loaded.wait(lk);

			temp_frame_dis = cv::Mat(image_array->at(current_time).height, image_array->at(current_time).width, CV_8UC1, image_array->at(current_time).bp);

			cv::rectangle(temp_frame_dis, myROI1_c1, cv::Scalar(255, 255, 255), 0.8, 8, 0);
			cv::rectangle(temp_frame_dis, myROI2_c1, cv::Scalar(255, 255, 255), 0.8, 8, 0);

			cv::rectangle(temp_frame_dis, ROI_stop_c1, cv::Scalar(255, 255, 255), 0.8, 8, 0);
			cv::rectangle(temp_frame_dis, ROI_stop_c2, cv::Scalar(255, 255, 255), 0.8, 8, 0);

			imshow("Live feed", temp_frame_dis);
			cv::waitKey(30);
		}

		cv::destroyWindow("Start");

	}
	#endif
	
	drop_mode_analyze = FALSE;

	// DisCo run is initialized here

	cv::VideoWriter output_video;
	output_video.open("test.avi", cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 25.0, recordROI.size());

	if (!output_video.isOpened())
	{
		std::cout << "!!! Output video could not be opened" << std::endl;
	}

	// Starting control threads for each channel

	std::thread channel1(control_channel,
		std::ref(state_c1_ref),
		std::ref(state_c2_ref),
		std::ref(MyOB1_ID),
		std::ref(m1),
		std::ref(cond_var_c1),
		std::ref(valveid_c1),
		std::ref(cond_var_c1_sleep),
		std::ref(c1_sleep),
		std::ref(PressureC1),
		std::ref(pressurechannel1),
		std::ref(pressurechannel2), // other channel
		part_size_ch1,
		contrast_ch1,
		std::ref(valveid_c2),
		image_array,
		ref(ROI_c1),
		std::ref(leadch1),
		std::ref(circ_ch1)
	);

	std::thread channel2(control_channel,
		std::ref(state_c2_ref),
		std::ref(state_c1_ref),
		std::ref(MyOB1_ID),
		std::ref(m2),
		std::ref(cond_var_c2),
		std::ref(valveid_c2),
		std::ref(cond_var_c2_sleep),
		std::ref(c2_sleep),
		std::ref(PressureC2),
		std::ref(pressurechannel2),
		std::ref(pressurechannel1), //other channel
		part_size_ch2,
		contrast_ch2, 
		std::ref(valveid_c1),
		image_array,
		ref(ROI_c2),
		std::ref(leadch2),
		std::ref(circ_ch2)
	);

	// Generate window for dynamic run control.

	cv::namedWindow("Settings", cv::WINDOW_AUTOSIZE);
	cv::moveWindow("Settings", 1100, 20);

	time_t inittime = time(0);

	int flush = 0;
	int pause_init = 0;
	int stoprec = 0;

	cv::createTrackbar("Flush:", "Settings", &flush, 1);
	cv::createTrackbar("Stop recording", "Settings", &stoprec, 1);
	cv::createTrackbar("Jump beads [ms]:", "Settings", &osc_var, 20);
	cv::createTrackbar("Delta jump [ms]", "Settings", &delta_celljump, 15);
	cv::createTrackbar("Cell jump pressure", "Settings", &speed_celljump, 300);
	cv::createTrackbar("Bead jump pressure", "Settings", &speed_beadjump, 300);
	cv::createTrackbar("Pause:", "Settings", &process_pause, 1);

	cv::Mat RecCrop;

	std::unique_lock<std::mutex> lock(m3);

	// State variables and timers for global process control

	int timewaitplace = 100;
	bool lockstate(false);
	time_t last_flush = clock();
	bool flushstate = false;
	time_t flushtime = clock();
	int framevid = 0;

	int encaps_counter = 0;

	int clean_flush = 0;

	// Loop to display live images and global process control (e.g. oil flush and encapsulation)

	while (1) {
		if (flush == 1) {
			valves.set_state(3, 0);
			flushstate = true;
		}
		if (flush == 0 && flushstate == true) {
			valves.set_state(3, 1);
			flushstate = false;
		}

		if (10000 < clock() - flushtime) {
			timewaitplace = 0;
		}

		// get, annotate, and display image
		std::unique_lock<std::mutex> lk(view_img);
		cond_var_img_loaded.wait(lk);

		temp_frame_dis = cv::Mat(image_array->at(current_time).height, image_array->at(current_time).width, CV_8UC1, image_array->at(current_time).bp);

		cv::rectangle(temp_frame_dis, myROI1_c1, cv::Scalar(255, 255, 255), 0.8, 8, 0);
		cv::rectangle(temp_frame_dis, myROI2_c1, cv::Scalar(255, 255, 255), 0.8, 8, 0);
		cv::rectangle(temp_frame_dis, myROI1_c2, cv::Scalar(255, 255, 255), 0.8, 8, 0);
		cv::rectangle(temp_frame_dis, myROI2_c2, cv::Scalar(255, 255, 255), 0.8, 8, 0);
		cv::rectangle(temp_frame_dis, ROI_stop_c1, cv::Scalar(255, 255, 255), 0.8, 8, 0);
		cv::rectangle(temp_frame_dis, ROI_stop_c2, cv::Scalar(255, 255, 255), 0.8, 8, 0);
		cv::rectangle(temp_frame_dis, recordROI, cv::Scalar(255, 255, 255), 0.8, 8, 0);

		putText(temp_frame_dis, cv::format("frame=%3d", fps), cv::Point(0, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255));
		putText(temp_frame_dis, cv::format("# Encaps=%4d", encaps_counter), cv::Point(175, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255));
		arrowedLine(temp_frame_dis, cv::Point(390, 392), cv::Point(440, 392), cv::Scalar(0, 0, 255), 3, 8, 0, 0.3);

		imshow("Live feed", temp_frame_dis);

		// if the system has been running without an encapsulation for above 60s, flush oil

		if (clock() - last_flush > 60000 && clean_flush == 0 && state_c1 != 3 && state_c2 != 3) {
			flush = 1;
			clean_flush = 1;
			flushtime = clock();
			std::cout << "Oil Flush" << std::endl;
		}
		if (clean_flush == 1 && clock() - flushtime > 100) {
			flush = 0;
			last_flush = clock();
			clean_flush = 0;
		}

		// if both channels have captured a cell, start encapsulation
		if (state_c1 == 3 && state_c2 == 3 && !eject_start) {
			std::cout << "ej start" << std::endl;
			std::unique_lock<std::mutex> lock_eject(mut_eject);
			eject_start = true;
			eject_cond_var.notify_one();
			flushtime = clock();
			timewaitplace = 200;
			encaps_counter++;
			last_flush = clock();
		}

		// pause process button
		if (process_pause == 1 && pause_init == 0 && state_c2 == 0) {
			pause_init = 1;
			cond_var_c1.notify_one();
			cond_var_c2.notify_one();
		}

		if (process_pause == 0 && pause_init == 1) {
			pause_init = 0;

			cond_var_c1.notify_one();
			cond_var_c2.notify_one();
		}

		RecCrop = temp_frame_dis(recordROI);

		putText(RecCrop, cv::format("time=%6d", time(0) - inittime), cv::Point(150, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255));

		// write frame in video
		output_video << RecCrop;
		framevid = framevid + 1;

		cv::waitKey(30);

		if (stoprec == 1) {
			output_video.release();
		}
	}

	camera.join();
	channel1.join();
	channel2.join();
	eject_t.join();
}