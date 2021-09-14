# DisCo_source
This is the repository for the DisCo source code. The repository contains the source code, install instructions for the necessary dependencies, and compilation instructions. 

## General remarks
The code in the presented form was developed specifically for the purpose of deterministically co-encapsulating two particles in droplets on a microfluidic system (as described in *Bues, Biocanin, Pezoldt et al. 2020, BioRxiv* (https://doi.org/10.1101/2020.05.19.103812). The code base was written in C++ (for performance and latency reasons) and is non-object-oriented. 

### Brief technical description
Specifically, it controls the coordination of real-time analysis of images streamed from a camera connected to a microscope, with the actuation of solenoid valves and pressure regulators connected to a microfluidic device. Detection in a defined ROI is achieved by the following processing sequence 1. subsequent image subtraction, 2. thresholding, 3. hole filling, 4. contour detection, and 5. contour thresholding. Each of the two channels is continuously monitored in this manner to detect and place particles. Particles are detected and stopped by the following sequence of events: 1. Detect particles in a first ROI, 2. Initiate a valve placement sequence (valve oscillation), 3. detect the arrival of the particle in the second ROI. Each channel is controlled by an independent thread, which in contains a thread-pool of detection threads. Both channels are coordinating each other, and are stopping particles subsequently. While concurrent particle stopping is possible, we found that it leads to non predictable states. Upon successful particle placement in both channels a globally coordinated routine is ejecting the particle into a droplet, which is finally captured. An important aspect of the code is elaborate timing of events (apparent by the excessive use of sleep statements), only with which robust working of the system is possible. 

### Contents of this repository
This repository contains a Visual Studio solution, which in turn contains the following files:

* **DisCo_main.cpp** - 
  Entry point and global control of the UI elements, and channel control.
* **DisCo_functions.cpp** - 
  A collection of functions for e.g. channel control, particle detection, encapsulation, etc.
* **DisCo.h** - 
  **Most important Default definitions of regularly changed variables, control over compiler instructions for calibration menus.** Furthermore, includes, function prototypes, global variables.
* **ValveController.cpp** - 
  Wrapper class for the NIDAQmx library.
* **ValveController.h** - 
  Corresponding header file for corresponding .cpp file.

## Install and compile instructions
Multiple external libraries are utilized in the DisCo source code. As some of these libraries are proprietary they are not distributed with the DisCo source code and need to be installed by the user. In order to compile the DisCo source code from source the following dependencies are required:
* OpenCV 4 (prebuilt libraries are sufficient)
* Ximea xiAPI
* National Instruments X
* Elveflow SDK
Header files for the libraries are included

### OpenCV 4 prebuilt libraries:
The OpenCV 4 prebuilt libraries can be downloaded from the following website:
https://opencv.org/releases/

### Elveflow SDK
The Elveflow SDK can be downloaded from the following website:
https://www.elveflow.com/microfluidic-products/microfluidics-software/elveflow-software-sdk/

Extract the archive, and extract the zipped SDK contained within.

### National Instruments NIDaqmx
https://www.ni.com/en-us/support/downloads/drivers/download.ni-daqmx.html#409845

### Ximea xiAPI
The installer for the Ximea xiAPI can be downloaded from the following website
https://www.ximea.com/support/wiki/apis/XIMEA_Windows_Software_Package

### Linking libraries
*Note: “…” marks installation specific paths and have to be replaced with the actual installation paths by the user.*
1.	Right click on project select Properties.
3.	In the Properties menu select correct Configuration: *Release* and Platform: *x64*
4.	Select *C/C++ > General*. In the field *Additional Include Directories* select:
    - The OpenCV include folder: “…\build\include”
    - Elveflow SDK include (example path, version specific): “…\SDK_V3_05_03\DLL64\DLL64”
    - Ximea API include: “…\API\xiAPI”
    - National Instruments NIDaqMX: “…\Shared\ExternalCompilerSupport\C\include”
5.	Press approve.
6. Go to *Linker > General*. In field *Additional Library Directories* add:
   - Elveflow SDK library (example path, version specific): …\SDK_V3_05_03\DLL64\DLL64
   - Ximea API library: …\API\64bit
   - The OpenCV library folder: …\build\x64\vc15\lib
   - National Instruments NIDaqMX: …\Shared\ExternalCompilerSupport\C\lib64\msvc
7. Go to *Linker > Input*. In field *Additional Dependencies* add:
   - opencv_world410.lib (Version dependent)
   - xiapi64.lib
   - Elveflow64.lib
   - NIDAQmx.lib

### Compile
In principle the Visual Studio solution should now contain all necessary dependencies and should be able to compile the executable. Make sure to select *Release* and *x64*.

## Code adaptation and reuse
The main intention of this repository is to provide a compilation ready version of the DisCo code to enable other laboratories to replicate our system. As indicated above, the code represents a specialized solution to the application it was written for, and the focus during development was not on reusability. Hence, although parts of the code could be readily incorporated in derivate projects, we recommend to consider refactoring the code before utilizing it as a development basis. 



