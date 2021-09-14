// Contains function definitions for the DisCo main code.

#include"DisCo.h"

void pressure_ramp(int pres, int time, int channel, int &MyOB1_ID) {
	// Function to ramp pressures to avoid spikes.

	double *pres_meas = new double;
	OB1_Get_Press(MyOB1_ID, 3, 1, Calibration, pres_meas, 1000);

	std::cout << "read_val: " << *pres_meas << " RampVal: " << std::endl;

	int set_val = *pres_meas;

	int pres_dif = pres - set_val;

	float ramp_val = float(pres_dif) / float(time);
	time_t time_start;
	time_t time_now;

	std::cout << "target val: " << pres << " RampVal: " << ramp_val << std::endl;

	time_start = clock();

	while (set_val < pres) {
		OB1_Set_Press(MyOB1_ID, 3, set_val, Calibration, 1000);
		time_now = clock() - time_start;
		set_val = set_val + (time_now * ramp_val);
		Sleep(1);
	}

	OB1_Set_Press(MyOB1_ID, 3, pres, Calibration, 1000);
}

void valve_oscilate(int *oscilate, int &modbus_con, time_t &recov_time, int &osc_const, int &modbus_conoc) {
	time_t reltimer = clock();
	int openstate = 0;

	valves.set_state(modbus_conoc, 1);

	Sleep(500);

	while (*oscilate == 1) {
		if (100 < clock() - reltimer && openstate == 0) { //100
			valves.set_state(modbus_con, 0);
			openstate = 1;
			reltimer = clock();
		}

		if (osc_const < clock() - reltimer && openstate == 1) { //25
			valves.set_state(modbus_con, 1);
			openstate = 0;
			Sleep(1);
			reltimer = clock();
		}
	}
	valves.set_state(modbus_con, 1);
}

void monitor_area(std::vector<XI_IMG> *image_array, std::vector<cv::Mat> *proc_array, int *state, std::mutex &mut_img_loaded, int *ROI_id, int *old_ROI_id, int *bufcoord, std::vector<cv::Rect> ROI_array, std::mutex &state_write, std::mutex &buffer_fill, int thread_id, std::condition_variable &state_change, int *part_size, int *contrast, int &circ, int &channel_id, bool *monitor_on) {
	// Detection function that is run in a separate thread. 

	int thread_time = 0;
	int thread_lasttime = 0;

	int first = 0;
	int bufaccess = 0;
	int buffsize = 7;

	cv::Mat ref_img;
	cv::Mat filler;

	int temp_state = 0;

	Sleep(100);

	while (*monitor_on == TRUE) {
		// Acquire lock. This makes sure only one thread in the pool analyzes an incoming image.
		std::unique_lock<std::mutex> lk(mut_img_loaded);

		// Wait for new image
		while (imgloadready[channel_id] == false) {
			cond_var_img_loaded.wait(lk);
		}
		imgloadready[channel_id] = false;
		// Release lock to allow for the next thread to queue
		lk.unlock();

		thread_lasttime = thread_time;
		thread_time = current_time;

		// For a new ROI the buffer is filled with one image.
		// Shared buffer is protected by lock.
		// As only one image available, skip detection by continue.

		std::unique_lock<std::mutex> buffer_lock(buffer_fill);

		if (*ROI_id != *old_ROI_id) {
			Sleep(1);
			filler = processimage(ROI_array.at(*ROI_id), image_array, thread_time);
			proc_array->at(0) = filler;
			proc_array->at(1) = filler;
			proc_array->at(2) = filler;
			proc_array->at(3) = filler;
			proc_array->at(4) = filler;
			proc_array->at(5) = filler;
			proc_array->at(6) = filler;
			proc_array->at(7) = filler;
			proc_array->at(8) = filler;
			proc_array->at(9) = filler;

			*old_ROI_id = *ROI_id;
			buffer_lock.unlock();
			continue;
		}

		// If same ROI, process and detect particle

		buffer_lock.unlock();

		// Calculate buffer id for image to compare to. Place processed image at current buffer coordinate.

		bufaccess = *bufcoord - buffsize;

		if (bufaccess < 0) {
			bufaccess = bufaccess + 10;
		}

		proc_array->at(*bufcoord) = processimage(ROI_array.at(*ROI_id), image_array, thread_time);

		// Detect particle

		temp_state = particledetect(*part_size, proc_array->at(bufaccess), proc_array->at(*bufcoord), *contrast, circ, channel_id); // was 350,30

		// Increment buffer coordinate.

		if (*bufcoord < 9) {
			*bufcoord = *bufcoord + 1;
		}
		else {
			*bufcoord = 0;
		}

		// If particle was detected, notify all.

		if (temp_state == 1) {
			state_change.notify_all();
		}

		std::unique_lock<std::mutex> state_mut(state_write);
		*state = temp_state;
		state_mut.unlock();

	}
	Sleep(100);
}

void get_images(std::vector<XI_IMG> *image_array) {
	// Function to grab images from camera. 
	// This function runs in thread and works continously.

	XI_RETURN stat = XI_OK;
	HANDLE xiH = NULL;
	xiOpenDevice(0, &xiH);

	int new_time = 0;
 
	std::cout << "Set timing mode: " << stat << std::endl;

	// Set Camera modes.
	stat = xiSetParamInt(xiH, XI_PRM_DECIMATION_VERTICAL, 2);
	stat = xiSetParamInt(xiH, XI_PRM_DECIMATION_HORIZONTAL, 2);
	stat = xiSetParamInt(xiH, XI_PRM_SENSOR_DATA_BIT_DEPTH, XI_BPP_8);

	std::cout << "Set decimation mode: " << stat << std::endl;

	stat = xiSetParamInt(xiH, XI_PRM_EXPOSURE, CAM_EXPOSURE);

	std::cout << "Set exposure mode: " << stat << std::endl;

	// Initiate Camera connection

	// Build initiation routine 
	stat = xiStartAcquisition(xiH);

	Sleep(100);

	time_t frame_time = 0;
	time_t frame_time_dif = 0;

	time_t start = clock();

	// Loop to get image, and place it in buffer.

	while (1) {
		if (current_time == 9) {
			new_time = 0;
			frame_time_dif = clock() - frame_time;
			frame_time = clock();
			fps = 10000 / frame_time_dif; //1000ms/timefor10frames * 10
		}
		else {
			new_time++;
		}
		start = clock();
		xiGetImage(xiH, 10, &image_array->at(new_time));

		current_time = new_time;

		imgloadready[0] = true;
		imgloadready[1] = true;
		imgloadready[2] = true;

		cond_var_img_loaded.notify_all();	
	}
	xiStopAcquisition(xiH);
	xiCloseDevice(xiH);
}

int particledetect(int minsize, cv::Mat last_frame, cv::Mat current_frame, int sens, int circ, int &channel_id) {
	// Particle detection function.

	// Subtract images
	cv::Mat im_dif;
	absdiff(last_frame, current_frame, im_dif);

	// Threshold resulting image
	cv::Mat im_dif_thresh;
	threshold(im_dif, im_dif_thresh, sens, 255, cv::THRESH_BINARY);

	// Erode image
	cv::Mat imerode;
	dilate(im_dif_thresh, imerode, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);

	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	// Find contours and analyze each contour by circularity and Area.

	findContours(imerode, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

	std::vector<cv::Moments> mu(contours.size());
	for (int i = 0; i < contours.size(); i++)
	{
		mu[i] = moments(contours[i], false);
	}

	std::vector<cv::Point2f> mc(contours.size());
	for (int i = 0; i < contours.size(); i++)
	{
		mc[i] = cv::Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
	}

	for (int i = 0; i < contours.size(); i++) {
		if (contourArea(contours[i]) > minsize) {
			int area = contourArea(contours[i]);
			int peri = arcLength(contours[i], TRUE);
			if (circ > pow(peri, 2) / area) {
				return 1;
			}
		}
	}

	im_dif.release();
	im_dif_thresh.release();
	imerode.release();

	return 0;
}

cv::Mat processimage(cv::Rect detectROI, std::vector<XI_IMG> *image_array, int time) {
	// Image preprocessing function

	// Get most recent image.
	cv::Mat image_cv;
	image_cv = cv::Mat(image_array->at(time).height, image_array->at(time).width, CV_8UC1, image_array->at(time).bp);
	cv::Mat croppedImage;

	// Crop image.
	croppedImage = image_cv(detectROI);

	// Blur image
	cv::Mat im_gauss;
	GaussianBlur(croppedImage, im_gauss, cv::Size(3, 3), 0, 0);

	image_cv.release();
	croppedImage.release();

	return im_gauss;
}

void control_channel(int *state, int *stateoc, int &MyOB1_ID, std::mutex &mut, std::condition_variable &cond_var, int &modbus_con, std::condition_variable &cond_var_sleep, std::mutex &sleepmut, int &Pressure, int &channel, int &other_channel, int minsize, int sens, int &modbus_conoc, std::vector<XI_IMG> *image_array, std::vector<cv::Rect> ROI_array, int &leadch, int &circ) {
	
	// Function that controls particle placement for one channel.
	std::mutex state_write;
	std::mutex mut_img_analyzed;
	std::mutex state_change_mut;
 
	std::vector<cv::Mat> *proc_array = new std::vector<cv::Mat>(10);
	std::condition_variable state_change;
	std::mutex mut_img_loaded;
	std::mutex buffer_fill;

	int *part_size = &minsize;
	int *contrast = &sens;

	Sleep(200);

	// Set up variable shared between different detection/monitor threads
	// ROI id: Current ROI
	// ROI old: used before

	int roi_id = 0;
	int *roi_ref = &roi_id;

	int roiold_id = 10;
	int *roiold_ref = &roiold_id;

	int buffcoord_var = 0;
	int *buffcoord_ref = &buffcoord_var;

	bool monitor_on = TRUE;
	bool *monitor_on_pnt = &monitor_on;

	// Generate a thread pool of 8 threads to detect particles. 

	std::thread monitor1(monitor_area, std::ref(image_array), std::ref(proc_array), std::ref(state), ref(mut_img_loaded), std::ref(roi_ref), std::ref(roiold_ref), std::ref(buffcoord_ref), ref(ROI_array), ref(state_write), ref(buffer_fill), std::ref(channel), std::ref(state_change), std::ref(part_size), std::ref(contrast), std::ref(circ), std::ref(channel), std::ref(monitor_on_pnt));
	std::thread monitor2(monitor_area, std::ref(image_array), std::ref(proc_array), std::ref(state), ref(mut_img_loaded), std::ref(roi_ref), std::ref(roiold_ref), std::ref(buffcoord_ref), ref(ROI_array), ref(state_write), ref(buffer_fill), std::ref(channel), std::ref(state_change), std::ref(part_size), std::ref(contrast), std::ref(circ), std::ref(channel), std::ref(monitor_on_pnt));
	std::thread monitor3(monitor_area, std::ref(image_array), std::ref(proc_array), std::ref(state), ref(mut_img_loaded), std::ref(roi_ref), std::ref(roiold_ref), std::ref(buffcoord_ref), ref(ROI_array), ref(state_write), ref(buffer_fill), std::ref(channel), std::ref(state_change), std::ref(part_size), std::ref(contrast), std::ref(circ), std::ref(channel), std::ref(monitor_on_pnt));
	std::thread monitor4(monitor_area, std::ref(image_array), std::ref(proc_array), std::ref(state), ref(mut_img_loaded), std::ref(roi_ref), std::ref(roiold_ref), std::ref(buffcoord_ref), ref(ROI_array), ref(state_write), ref(buffer_fill), std::ref(channel), std::ref(state_change), std::ref(part_size), std::ref(contrast), std::ref(circ), std::ref(channel), std::ref(monitor_on_pnt));
	std::thread monitor5(monitor_area, std::ref(image_array), std::ref(proc_array), std::ref(state), ref(mut_img_loaded), std::ref(roi_ref), std::ref(roiold_ref), std::ref(buffcoord_ref), ref(ROI_array), ref(state_write), ref(buffer_fill), std::ref(channel), std::ref(state_change), std::ref(part_size), std::ref(contrast), std::ref(circ), std::ref(channel), std::ref(monitor_on_pnt));
	std::thread monitor6(monitor_area, std::ref(image_array), std::ref(proc_array), std::ref(state), ref(mut_img_loaded), std::ref(roi_ref), std::ref(roiold_ref), std::ref(buffcoord_ref), ref(ROI_array), ref(state_write), ref(buffer_fill), std::ref(channel), std::ref(state_change), std::ref(part_size), std::ref(contrast), std::ref(circ), std::ref(channel), std::ref(monitor_on_pnt));
	std::thread monitor7(monitor_area, std::ref(image_array), std::ref(proc_array), std::ref(state), ref(mut_img_loaded), std::ref(roi_ref), std::ref(roiold_ref), std::ref(buffcoord_ref), ref(ROI_array), ref(state_write), ref(buffer_fill), std::ref(channel), std::ref(state_change), std::ref(part_size), std::ref(contrast), std::ref(circ), std::ref(channel), std::ref(monitor_on_pnt));
	std::thread monitor8(monitor_area, std::ref(image_array), std::ref(proc_array), std::ref(state), ref(mut_img_loaded), std::ref(roi_ref), std::ref(roiold_ref), std::ref(buffcoord_ref), ref(ROI_array), ref(state_write), ref(buffer_fill), std::ref(channel), std::ref(state_change), std::ref(part_size), std::ref(contrast), std::ref(circ), std::ref(channel), std::ref(monitor_on_pnt));

	// Initialize time variables

	time_t recovtime = std::clock();

	std::unique_lock<std::mutex> lock(mut);
	std::unique_lock<std::mutex> lock_sleep(sleepmut);
	std::unique_lock<std::mutex> wait_changelk(state_change_mut);

	std::cout << "starting thread" << std::endl;

	OB1_Set_Press(MyOB1_ID, channel, Pressure, Calibration, 1000);

	cv::Mat stop_reg_check;
	cv::Mat ej_check_mat;

	int time_wait = 4;

	std::chrono::seconds waitTime(time_wait);

	int oscilate_var = 1;
	int *oscilate_ref = &oscilate_var;
	time_t time_dif_activation;

	int osc_def;
	int time_wait_init = 1;
	std::chrono::seconds waitTime_init(time_wait_init);

	time_t lead_start = clock();

	// Infinte loop to control channel. This loop is a common loop for both bead and cell channel.
	// As the cell channel is stopping a cell first (for low cell concentrations times vary strongly), the lead channel is waiting.
	// After a cell is stopped beads are getting stopped.

	while (1) {
		// Start detection in ROI 1. Both (lead/nonlead) Do this in parallel.
		roi_id = 0;
		state_map[channel] = 0;
		Sleep(100);
		valves.set_state(modbus_con, 0);
		Sleep(5);

		// If something was detected and this channel is not lead (cells).
		// Set pressure to slow to capture the particle
		if (*state == 1 && leadch == 0) {
			OB1_Set_Press(MyOB1_ID, channel, pressio[channel], Calibration, 1000);
			state_change.wait_for(wait_changelk, waitTime_init);
		}

		// If channel is empty. set speed to fast

		if (*state == 0 && leadch == 0) {
			OB1_Set_Press(MyOB1_ID, channel, press_fast[channel], Calibration, 1000);
		}

		std::cout << "wait for call normal" << std::endl;

		// If this channel is lead channel (beads) and other channel has not finished placement.
		// Wait for the non lead channel to stop a particle (cell).

		if (leadch == 1 && *stateoc != 3) {
			valves.set_state(modbus_con, 1);
			OB1_Set_Press(MyOB1_ID, channel, pressio[channel], Calibration, 1000);
			while (*stateoc != 3) {
				Sleep(10);
			}
			Sleep(1000);

			lead_start = clock();

			// Open channel to start detection.
			valves.set_state(modbus_con, 0);
			OB1_Set_Press(MyOB1_ID, channel, press_fast[channel], Calibration, 1000);

		}

		// Wait for something to be detected.

		while (*state == 0) {
			state_map[channel] = 1;
			state_change.wait(wait_changelk);
		}

		// This is the pause function. state == 4 is pause

		if (process_pause == 1) { 
			Sleep(500);
			while (process_pause == 1) {
				state_change.wait(wait_changelk);
			}
			continue;
		}

		// Once here, something was detected in the channel
		// Set state_map to 2 -> stopping
		// Switch to the next region of interest.
		// Set pressure to stopping pressure.

		state_map[channel] = 2;
		roi_id = 2;
		OB1_Set_Press(MyOB1_ID, channel, pressio[*&channel], Calibration, 1000);

		// Reset state variable for next ROI
		// Make sure both valves (cell/bead) to allow for undisturbed placement
		*state = 0;

		OB1_Set_Press(MyOB1_ID, 1, pressio[*&channel], Calibration, 1000);

		valves.set_state(modbus_con, 1);
		valves.set_state(modbus_conoc, 1);

		Sleep(100);

		time_dif_activation = clock() - recovtime;

		std::cout << "Placing particle after ... " << time_dif_activation << std::endl;

		oscilate_var = 1;

		// Define timeout time channel specific.

		if (leadch != 1 && time_dif_activation < 800) {
			time_wait = 8;
		}
		else {
			time_wait = 3;
		}

		std::chrono::seconds waitTime(time_wait);

		// Define oscillation values depending on channel. Delta value is calculated.

		if (leadch == 0) {
			osc_def = osc_var + delta_celljump;// WAS 5
		}
		if (leadch == 1) {
			osc_def = osc_var;
		}

		time_t osci_timer = clock();

		// Start valve oscillation thread.
		std::thread valve_osci(valve_oscilate, std::ref(oscilate_ref), std::ref(modbus_con), std::ref(time_dif_activation), std::ref(osc_def), std::ref(modbus_conoc));
		std::cout << "Osci wait ... " << time_dif_activation << std::endl;


		// While cell not stopped and it is less than 8s wait time
		while (*state != 1 || 8000 < clock() - osci_timer) {
			time_wait = (8000 - (clock() - osci_timer)) / 1000;
			std::chrono::seconds waitTime(time_wait);
			std::cout << "Wait again: " << time_wait << std::endl;
			// Wait for cell detection for.
			state_change.wait_for(wait_changelk, waitTime);
			if (time_wait < 0) {
				break;
			}
		}

		std::cout << "State after wait: " << *state << std::endl;

		// Cell is stopped: state = 3.
		// Stop oscillation
		state_map[channel] = 3;
		oscilate_var = 0;
		valve_osci.join();

		std::cout << "Osci join ... " << time_dif_activation << std::endl;

		// If oscillation took longer than timeout time. Assume fail and start over.
		if (8000 < clock() - osci_timer) {
			valves.set_state(modbus_con, 0);
			OB1_Set_Press(MyOB1_ID, channel, press_fast[channel], Calibration, 1000);
			continue;
		}

		// If below stopping was sucessful. 
		if (8000 > clock() - osci_timer) {
			// Set channel valve closed.
			valves.set_state(modbus_con, 1);
			Sleep(100);
			std::cout << "Finish stop ... " << time_dif_activation << std::endl;

			// If other channel has no particle stopped. Open it for detection.
			// In practice this will start the stopping of the lead channel (beads).
			if (*stateoc == 0) {
				OB1_Set_Press(MyOB1_ID, other_channel, press_fast[other_channel], Calibration, 1000);
				std::cout << "open oc" << std::endl;
				valves.set_state(modbus_conoc, 0);

			}

			std::unique_lock<std::mutex> state_mut(state_write);
			state_map[channel] = 4;
			*state = 3;

			std::cout << "sleep" << std::endl;

			if (leadch == 1 && clock() - lead_start > 5000) {
				Sleep(1000);
			}

			// If stopped wait for being called.
			// This will be called by the ejection thread.

			while (*state == 3) {
				cond_var_sleep.wait(lock_sleep);
			}

			std::cout << "unlocked" << std::endl;
			state_mut.unlock();

			// If unlocked and the channel is lead channel (beads)
			// Wait for 500ms before entering reentering the routine

			if (leadch == 1) {
				Sleep(500);
			}

			recovtime = clock();

		}

	}
	monitor1.join();
	monitor2.join();
	monitor3.join();
	monitor4.join();
	monitor5.join();
	monitor6.join();
	monitor7.join();
	monitor8.join();
}

void eject(int &MyOB1_ID, int &state_c1, int &state_c2, bool *drop_mode_analyze) {

	// Ejection routine. Runs in a separate thread and is getting activated 

	std::unique_lock<std::mutex> lock(mut_eject);

	while (1) {
		// Wait for the ejection to be activated.
		while (!eject_start) {
			eject_cond_var.wait(lock);
			std::cout << "notified" << std::endl;
		}

		// Flushe oil for 500 ms
		// Set pressure in both channels for encapsulation
		valves.set_state(3, 0);
		OB1_Set_Press(MyOB1_ID, 1, 100, Calibration, 1000);
		OB1_Set_Press(MyOB1_ID, 2, 100, Calibration, 1000);	
		Sleep(500); 

		// Ramp pressure to the first set point. This removes waste buffers from the chip.
		// Sleep 500ms for the buffer to exit the chip.

		pressure_ramp(pres_clean_nozzle, 100, 3, MyOB1_ID);
		Sleep(500); 

		// Close oil valve.
		valves.set_state(3, 1);
		Sleep(100);

		// Ramp to produce a droplet in the encapsulation area
		pressure_ramp(pres_make_drop, 50, 3, MyOB1_ID); 
		Sleep(100); 

		// Open sample valve to capture the droplet
		valves.set_state(5, 0);
		Sleep(50);
		valves.set_state(3, 0);


		// Droplet routine with extended wait time to observe droplet during calibration.
		if (*drop_mode_analyze == TRUE) {
			valves.set_state(4, 1);
			Sleep(170);
			valves.set_state(3, 1);
			Sleep(500);
		}

		// As above, with short times. Close waste valve and capture. Turn off oil.
		else {
			Sleep(50);
			valves.set_state(4, 1);
			Sleep(150); 

			valves.set_state(3, 1);
			Sleep(100);
		}

		// Reset chip valves for the next encapsulation.

		OB1_Set_Press(MyOB1_ID, 3, 1000, Calibration, 1000);
		Sleep(20);

		valves.set_state(4, 0);
		Sleep(50);
		valves.set_state(5, 1);

		OB1_Set_Press(MyOB1_ID, 1, press_fast[1], Calibration, 1000);
		OB1_Set_Press(MyOB1_ID, 2, press_fast[2], Calibration, 1000);

		OB1_Set_Press(MyOB1_ID, 3, 1400, Calibration, 1000);

		valves.set_state(0, 0);
		valves.set_state(1, 0);

		Sleep(50);
		OB1_Set_Press(MyOB1_ID, 3, 0, Calibration, 1000);

		int PressureEVrel = 0;

		time_t ejrel_time = clock();

		// Reset variables for the next encapsulation.
		std::unique_lock<std::mutex> lock_sleep_c1(c1_sleep);
		std::unique_lock<std::mutex> lock_sleep_c2(c2_sleep);

		state_c1 = 0;
		state_c2 = 0;

		cond_var_c1_sleep.notify_one();
		cond_var_c2_sleep.notify_one();

		leadchecked = false;
		eject_start = false;
	}
}

void determ_osci(int &MyOB1_ID, std::mutex &mut, std::condition_variable &cond_var, int &modbus_con, int &Pressure, int &channel, int minsize, int sens, int &modbus_conoc, std::vector<XI_IMG> *image_array, std::vector<cv::Rect> ROI_array, int &circ, bool *finished) {

	// This function is run in a thread and is determining the minimum value of the oscillation duration.
	// It starts at 10 ms, tries to stop a bead, and if it fails starts over.

	std::condition_variable state_change;
	std::mutex state_change_mut;
	std::unique_lock<std::mutex> wait_changelk(state_change_mut);

	std::vector<cv::Mat> *proc_array = new std::vector<cv::Mat>(10);

	std::mutex mut_img_loaded;
	std::mutex buffer_fill;

	int *part_size = &minsize;
	int *contrast = &sens;

	Sleep(1000);

	int roi_id = 0;
	int *roi_ref = &roi_id;

	int roiold_id = 10;
	int *roiold_ref = &roiold_id;

	int buffcoord_var = 0;
	int *buffcoord_ref = &buffcoord_var;

	int atmps = 0;
	int success = 0;

	int state_var = 0;
	int *state = &state_var;

	int oscilate_var = 1;
	int *oscilate_ref = &oscilate_var;

	time_t time_dif_activation = 1000;
	std::mutex state_write;

	bool monitor_on = TRUE;
	bool *monitor_on_pnt = &monitor_on;

	std::thread monitor1(monitor_area, std::ref(image_array), std::ref(proc_array), std::ref(state), ref(mut_img_loaded), std::ref(roi_ref), std::ref(roiold_ref), std::ref(buffcoord_ref), ref(ROI_array), ref(state_write), ref(buffer_fill), std::ref(channel), std::ref(state_change), std::ref(part_size), std::ref(contrast), std::ref(circ), std::ref(channel), std::ref(monitor_on_pnt));

	while (1) {
		std::cout << "osci value: " << osc_var << " ... ";
		roi_id = 0;

		valves.set_state(3, 0);
		Sleep(500);
		valves.set_state(3, 1);

		valves.set_state(modbus_con, 0);


		OB1_Set_Press(MyOB1_ID, channel, press_fast[channel], Calibration, 1000);
		OB1_Set_Press(MyOB1_ID, 1, press_fast[1], Calibration, 1000);

		Sleep(500);
		state_change.wait(wait_changelk);
		roi_id = 2;

		OB1_Set_Press(MyOB1_ID, channel, pressio[channel], Calibration, 1000);
		valves.set_state(modbus_con, 1);

		Sleep(100);
		oscilate_var = 1;
		std::chrono::seconds waitTime(3);

		std::thread valve_osci(valve_oscilate, std::ref(oscilate_ref), std::ref(modbus_con), std::ref(time_dif_activation), std::ref(osc_var), std::ref(modbus_conoc));

		state_change.wait_for(wait_changelk, waitTime);

		oscilate_var = 0;

		valve_osci.join();

		atmps = atmps + 1;

		if (*state == 1) {
			success = success + 1;
			std::cout << " success " << std::endl;
		}
		else {
			osc_var = osc_var + 1;
			success = 0;
			atmps = 0;
			std::cout << " failed " << std::endl;
		}
		if (atmps == 3) {
			if (atmps == success) {
				monitor_on = FALSE;
				monitor1.join();
				osc_var = osc_var + 1;
				std::cout << " determined value: " << osc_var << " ms" << std::endl;
				*finished = TRUE;
				return;
			}
			osc_var = osc_var + 1;
			atmps = 0;
		}
	}
}

void determ_drop(int &MyOB1_ID, std::mutex &mut, std::condition_variable &cond_var, int &modbus_con, int &Pressure, int &channel, int minsize, int sens, int &modbus_conoc, std::vector<XI_IMG> *image_array, std::vector<cv::Rect> ROI_array, int &circ, bool *finished, int *start, int *drop_test) {

	// Is run in a separate thread and continously stops beads and produces droplets

	while (*start == 0) {
		Sleep(10);
	}

	std::condition_variable state_change;
	std::mutex state_change_mut;
	std::unique_lock<std::mutex> wait_changelk(state_change_mut);

	std::vector<cv::Mat> *proc_array = new std::vector<cv::Mat>(10);

	std::mutex mut_img_loaded;
	std::mutex buffer_fill;

	int *part_size = &minsize;
	int *contrast = &sens;

	Sleep(1000);

	int roi_id = 0;
	int *roi_ref = &roi_id;

	int roiold_id = 10;
	int *roiold_ref = &roiold_id;

	int buffcoord_var = 0;
	int *buffcoord_ref = &buffcoord_var;

	int state_var = 0;
	int *state = &state_var;

	int oscilate_var = 1;
	int *oscilate_ref = &oscilate_var;

	time_t time_dif_activation = 1000;
	std::mutex state_write;

	bool monitor_on = TRUE;
	bool *monitor_on_pnt = &monitor_on;

	int test_drop_num = 1;

	std::thread monitor1(monitor_area, std::ref(image_array), std::ref(proc_array), std::ref(state), ref(mut_img_loaded), std::ref(roi_ref), std::ref(roiold_ref), std::ref(buffcoord_ref), ref(ROI_array), ref(state_write), ref(buffer_fill), std::ref(channel), std::ref(state_change), std::ref(part_size), std::ref(contrast), std::ref(circ), std::ref(channel), std::ref(monitor_on_pnt));
	//drop_test variable comes from main
	while (*drop_test != 1) {
		std::cout << "osci value: " << osc_var << " ... ";
		roi_id = 0;
		//flush oil for 500msec
		valves.set_state(3, 0);
		Sleep(500);
		valves.set_state(3, 1);

		valves.set_state(modbus_con, 0);
		valves.set_state(1, 0);

		//Set pressure for cell and bead channel

		OB1_Set_Press(MyOB1_ID, channel, press_fast[channel], Calibration, 1000);
		OB1_Set_Press(MyOB1_ID, 2, press_fast[2], Calibration, 1000);

		Sleep(500);

		roi_id = 2;

		OB1_Set_Press(MyOB1_ID, channel, pressio[channel], Calibration, 1000);
		OB1_Set_Press(MyOB1_ID, 2, pressio[2], Calibration, 1000);
		valves.set_state(modbus_con, 1);
		valves.set_state(1, 1);

		Sleep(100);
		
		std::cout << "ej start" << std::endl;
		eject_start = true;

		// Ejection thread starts waiting on another core

		eject_cond_var.notify_all();
		while (eject_start) {
			Sleep(100);
		}

		if (test_drop_num > 20) {
			*drop_test = 1;
		}
		test_drop_num++;

	}
	monitor_on = FALSE;
	monitor1.join();
	valves.set_state(modbus_con, 1);
	valves.set_state(1, 1);
	OB1_Set_Press(MyOB1_ID, channel, pressio[channel], Calibration, 1000);
	OB1_Set_Press(MyOB1_ID, 2, pressio[2], Calibration, 1000);
	return;
}
