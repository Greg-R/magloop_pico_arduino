/* A "proof of concept" project to replace the STM32F103 "Blue Pill"
   which is used in the "Magnetic Loop Controller" described in the book
   "Microcontroller Projects for Amateur Radio by Jack Purdum, W8TEE, and
   Albert Peter, AC8GY" with the Raspberry Pi Pico.
   Copyright (C) 2022  Gregory Raven

                                                    LICENSE AGREEMENT

  This program source code and its associated hardware design at subject to the GNU General Public License version 2,
                  https://opensource.org/licenses/GPL-2.0
  with the following additional conditions:
    1. Any commercial use of the hardware or software is prohibited without express, written, permission of the authors.
    2. This entire comment, unaltered, must appear at the top of the primary source file. In the Arduino IDE environemnt, this comment must
       appear at the top of the INO file that contains setup() and loop(). In any other environmentm, it must appear in the file containing
       main().
    3. This notice must appear in any derivative work, regardless of language used.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    A copy of the GPL-2.0 license is included in the repository as file LICENSE.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/

#pragma once
#include <stdint.h>
#include <string>
#include <vector>
// #include "Configuration.h"

#define NUMBER_BANDS 3

//  This class is intended to manage various frequency and position related constants and variables.
//  The single object will be referenced by most or maybe all of the other class objects.

class Data
{

public:

const std::string version = "main";
const std::string releaseDate = "8-26-24";

  // Flags used to indicate switch closures.
  bool maxclose;
  bool zeroclose;

  //  These are fundamental size definitions used throughout the project.
  const int PRESETSPERBAND = 6; // Allow this many preset frequencies on each band
  const int MAXBANDS = 5;       // Can only process this many frequency bands
  const int MAXMENUES = 3;
  const int PIXELWIDTH = 320;   // Display pixels width.
  const int PIXELHEIGHT = 240;  // Display pixels height.


uint32_t EIGHTY_METERS = 0;
uint32_t SIXTY_METERS = 1;
uint32_t FORTY_METERS = 2;
uint32_t THIRTY_METERS = 3;
uint32_t TWENTY_METERS = 4;
uint32_t SEVENTEEN_METERS = 5;
uint32_t FIFTEEN_METERS = 6;
uint32_t TWELVE_METERS = 7;
uint32_t TEN_METERS = 8;

// The bands are selected with the following variables.
// The default selections are for 40M, 30M, and 20M.
// Un-comment more lines if NUMBER_BANDS is greater than 3.

uint32_t band0 = FORTY_METERS;
uint32_t band1 = THIRTY_METERS;
uint32_t band2 = TWENTY_METERS;
//uint32_t band3 = TWELVE_METERS;
// const uint32_t band4 =

#if NUMBER_BANDS == 3
std::vector<uint32_t> user_bands = {band0, band1, band2};
#elif NUMBER_BANDS == 4
std::vector<uint32_t> user_bands = {band0, band1, band2, band3};
#elif NUMBER_BANDS == 5
std::vector<uint32_t> user_bands = {band0, band1, band2, band3, band4};
#endif


  // Bands:

  std::vector<std::string> bands = {"80M", "60M", "40M", "30M", "20M", "17M", "12M", "10M"};
  static const uint32_t LOWEND80M  = 3500000;
  static const uint32_t HIGHEND80M = 4000000;

  static const uint32_t LOWEND60M  = 5330000;
  static const uint32_t HIGHEND60M = 5410000; 

  static const uint32_t LOWEND40M  = 7000000;
  static const uint32_t HIGHEND40M = 7300000;

  static const uint32_t LOWEND30M  = 10100000;
  static const uint32_t HIGHEND30M = 10150000;

  static const uint32_t LOWEND20M  = 14000000;
  static const uint32_t HIGHEND20M = 14350000;

  static const uint32_t LOWEND17M  = 18068000;
  static const uint32_t HIGHEND17M = 18168000;

  static const uint32_t LOWEND15M  = 21000000;
  static const uint32_t HIGHEND15M = 21450000;

  static const uint32_t LOWEND12M  = 24890000;
  static const uint32_t HIGHEND12M = 24990000;

  static const uint32_t LOWEND10M  = 28000000;
  static const uint32_t HIGHEND10M = 29700000;


  // Preset frequency constants in the dataStruct are initial defaults; these defaults are saved to the
  // EEPROM initially, but they can be overwritten later if the user desires.  The presets will always
  // be read from the EEPROM.
  struct dataStruct
  {
    uint32_t presetFrequencies[9][6];
/*
     =
        {
            {3503000, 3504000, 3600000, 3615000, 3750000, 3900000},       // 80M
            {5330500, 5346500, 5357000, 5371500, 5403500, 5403500},       // 60M.  This band is channelized.              
            {7030000, 7040000, 7100000, 7150000, 7250000, 7285000},       // 40M
            {10106000, 10116000, 10120000, 10130000, 10140000, 10145000}, // 30M
            {14030000, 14060000, 14100000, 14200000, 14250000, 14285000},  // 20M
            {14030000, 14060000, 14100000, 14200000, 14250000, 14285000},  // 17M
            {14030000, 14060000, 14100000, 14200000, 14250000, 14285000},  // 15M            
            {14030000, 14060000, 14100000, 14200000, 14250000, 14285000},  // 12M
            {14030000, 14060000, 14100000, 14200000, 14250000, 14285000}   // 10M
    };
    */
    uint32_t bandLimitPositionCounts[9][2];
    uint32_t bandEdges[9][2]; // = { // Band edges in Hz
                              //   {LOWEND40M, HIGHEND40M},
                              //   {LOWEND30M, HIGHEND30M},
                              //   {LOWEND20M, HIGHEND20M}};
    uint32_t currentBand;
    uint32_t currentFrequency;
    uint32_t lastFreq[9];  // Used to remember the last autotune frequency.
    uint32_t initialized;
    uint32_t calibrated; //  Please run Initial Calibration! if not set to correct value.
    uint32_t hardware;   //  0x55555555 means hardware is accepted.
  // The following are parameters which must be "tuned" to the particular mechanics in use.
  // Refer to the documentation for setting these values.
  int zero_offset; // 600 for full step.  1000 for half-step.  270 for dummy with 1/16 step.
  int backlash;
  int coarse_sweep;
  int accel;
  int speed;
  bool rotation;
  } workingData;

  //  This should be made variable length arrays.
  float countPerHertz[9];
  float hertzPerStepperUnitVVC[9]; // Voltage Variable Cap

  // GPIO assignments.
  //  Buttons
  int enterButton = 6;
  int autotuneButton = 7;
  int exitButton = 9;
  // Zero and Maximum switches.
  int zeroswitch = 10;
  int maxswitch = 11;
  // Stepper position.  This is here because it is not always convenient to interogate the stepper object.
  int32_t position;
  //  Power controls
  const int STEPPERSLEEPNOT = 9;
  const int OPAMPPOWER = 3;
  const int RFAMPPOWER = 2;
  const int RFRELAYPOWER = 19;
  //  Interface for the DDS object.
  const int DDS_RST = 4;
  const int DDS_DATA = 5;
  const int DDS_FQ_UD = 12;
  const int WLCK = 22;

  Data();

  void computeSlopes();

  void writeDefaultValues();
};