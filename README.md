# DisCo_source
This is the repository for the DisCo source code. The repository contains the source code, install instructions for the necessary dependencies, and compilation instructions. 

## General remarks
The code in the presented form was developed specifically for the purpose of deterministically co-encapsulating two particles in droplets on a microfluidic system (as described in *Bues, Biocanin, Pezoldt et al. 2020, BioRxiv* (https://doi.org/10.1101/2020.05.19.103812). Specifically, the coordinates the real-time analysis of images streamed from a camera connected to a microscope with the actuation of solenoid valves and pressure regulators, connected to a microfluidic device. 

It is written as non-object-oriented in C++, relying on multiple external libraries (listed in the install instructions). 

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



