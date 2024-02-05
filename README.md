## MelodyIlluminator

**Introduction**

"Melody Illuminator" is an innovative implementation of an FFT (Fast Fourier Transform)-based LED control system using the MSP432P401R microcontroller. This project dynamically control LEDs according to the frequency characteristics of the input signal. It's an intersection of audio processing and visual feedback.

**Features**

Audio Signal Analysis: Utilizes FFT to analyze the spectrum of an audio signal. <br>
LED control based on frequency analysis <br>
Multiple operational modes with LED indications. <br>
Button-based program control. <br>
LCD screen to read informations

**Hardware Requirements**

MSP432P401R LaunchPad Development Kit. <br>
External LEDs connected to specified GPIO pins. <br>
Educational BoosterPack MKII <br>
Code Composer Studio IDE <br>
ARM CMSIS and TI DriverLib libraries

**Software Dependencies**

TI DriverLib for MSP432P4XX. <br>
ARM CMSIS DSP Library. <br>
CrystalFontz128x128_ST7735 LCD driver (for graphical display).

**Setup and Configuration**

Connect the LEDs to the specified GPIO pins on the MSP432P401R LaunchPad. <br>
Ensure all the required libraries are included in your project.

**Project Layout**

FFT-Based LED Control System Project Layout

├── README.md                 # Project description and usage instructions <br>
├── src                       # Main source code directory <br>
│   ├── main.c                # Main application source code <br>
│   ├── fft.c                 # FFT processing routines <br>
│   └── led_control.c         # LED control algorithms <br>
├── include                   # Header files <br>
│   ├── fft.h                 # Header for FFT processing <br>
│   └── led_control.h         # Header for LED control functions <br>
├── lib                       # External libraries <br>
│   ├── CMSIS                 # ARM CMSIS library <br>
│   └── DriverLib             # TI DriverLib for MSP432P4XX <br>
├── drivers                   # Device specific drivers <br>
│   └── LcdDriver             # LCD driver for Crystalfontz128x128_ST7735 <br>
├── tests                     # Test code for the application <br>
│   ├── test_fft.c            # Tests for FFT functionalities <br>
│   └── test_led_control.c    # Tests for LED control logic <br>
├── tools                     # Tools and utilities <br>
│   └── setup                 # Setup scripts and configuration files <br>
├── Dockerfile                # Dockerfile for setting up the development environment <br>
└── docs                      # Documentation <br>
    └── API                   # API documentation and usage examples <br>

## Building the Project

**Open the Project**

Launch CCS and open your project by navigating to File > Open Project from File System. <br>
Select the root directory of your project.

**Configure the Project**

Right-click on the project in the Project Explorer. <br>
Go to Properties. <br>
Ensure that the correct microcontroller (MSP432P401R) is selected and that the paths to the CMSIS and DriverLib are correctly set in the build settings.

**Build the Project**

Click on the Project menu and select Build All. <br>
The IDE compiles the code, and any errors or warnings will be displayed in the console.

**Usage**

Upon powering the MSP432P401R LaunchPad, the system starts in PROGRAM_1 mode and can be switched between different modes using the onboard button. Each mode represents a different LED control algorithm based on the FFT analysis: <br>
PROGRAM_1: Controls LEDs based on the intensity and frequency range of the audio signal. <br>
PROGRAM_2: Indicates the sound’s amplitude through specific LEDs. <br>
PROGRAM_3: LED control based on the dominant frequency (high, medium, low sounds)

**Important Notes**

The system utilizes a DMA controller for efficient data transfer. <br>
Ensure proper scaling of the audio signal to match the ADC input range. <br>
The FFT computation is optimized for a sample length of 512 samples. <br>
The PWM configuration for LED control is adjustable based on desired brightness and response.

## Links 

**Powerpoint Presentation**

https://docs.google.com/presentation/d/11gHxpk3-geGhoeCDVJ-1NT8zf_D1DbR4idNQLmIGHvU/edit?usp=sharing

**Youtube Video** 

https://youtu.be/ncU5WiL63Xs
