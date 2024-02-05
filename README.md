## MelodyIlluminator

**Introduction**

"Melody Illuminator" is an innovative implementation of an FFT (Fast Fourier Transform)-based LED control system using the MSP432P401R microcontroller. This project dynamically control LEDs according to the frequency characteristics of the input signal. It's an intersection of audio processing and visual feedback.

**Features**

Audio Signal Analysis: Utilizes FFT to analyze the spectrum of an audio signal.
LED control based on frequency analysis
Multiple operational modes with LED indications.
Button-based program control.
LCD screen to read informations

**Hardware Requirements**

MSP432P401R LaunchPad Development Kit.
External LEDs connected to specified GPIO pins.
Educational BoosterPack MKII
Code Composer Studio IDE
ARM CMSIS and TI DriverLib libraries

**Software Dependencies**

TI DriverLib for MSP432P4XX.
ARM CMSIS DSP Library.
CrystalFontz128x128_ST7735 LCD driver (for graphical display).

**Setup and Configuration**

Connect the LEDs to the specified GPIO pins on the MSP432P401R LaunchPad.
Ensure all the required libraries are included in your project.

**Project Layout**

FFT-Based LED Control System Project Layout

├── README.md                 # Project description and usage instructions
├── src                       # Main source code directory
│   ├── main.c                # Main application source code
│   ├── fft.c                 # FFT processing routines
│   └── led_control.c         # LED control algorithms
├── include                   # Header files
│   ├── fft.h                 # Header for FFT processing
│   └── led_control.h         # Header for LED control functions
├── lib                       # External libraries
│   ├── CMSIS                 # ARM CMSIS library
│   └── DriverLib             # TI DriverLib for MSP432P4XX
├── drivers                   # Device specific drivers
│   └── LcdDriver             # LCD driver for Crystalfontz128x128_ST7735
├── tests                     # Test code for the application
│   ├── test_fft.c            # Tests for FFT functionalities
│   └── test_led_control.c    # Tests for LED control logic
├── tools                     # Tools and utilities
│   └── setup                 # Setup scripts and configuration files
├── Dockerfile                # Dockerfile for setting up the development environment
└── docs                      # Documentation
    └── API                   # API documentation and usage examples

## Building the Project

**Open the Project**

Launch CCS and open your project by navigating to File > Open Project from File System.
Select the root directory of your project.

**Configure the Project**

Right-click on the project in the Project Explorer.
Go to Properties.
Ensure that the correct microcontroller (MSP432P401R) is selected and that the paths to the CMSIS and DriverLib are correctly set in the build settings.

**Build the Project**

Click on the Project menu and select Build All.
The IDE compiles the code, and any errors or warnings will be displayed in the console.

**Usage**

Upon powering the MSP432P401R LaunchPad, the system starts in PROGRAM_1 mode and can be switched between different modes using the onboard button. Each mode represents a different LED control algorithm based on the FFT analysis:
PROGRAM_1: Controls LEDs based on the intensity and frequency range of the audio signal.
PROGRAM_2: Indicates the sound’s amplitude through specific LEDs.
PROGRAM_3: LED control based on the dominant frequency (high, medium, low sounds)

**Important Notes**

The system utilizes a DMA controller for efficient data transfer.
Ensure proper scaling of the audio signal to match the ADC input range.
The FFT computation is optimized for a sample length of 512 samples.
The PWM configuration for LED control is adjustable based on desired brightness and response.

