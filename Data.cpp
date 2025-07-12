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

#include "Data.h"

Data::Data()
{
   maxclose = false;
   zeroclose = false;
};

void Data::computeSlopes()
{
   countPerHertz[0] = ((float)workingData.bandLimitPositionCounts[0][1] - (float)workingData.bandLimitPositionCounts[0][0]) / ((float)HIGHEND80M - (float)LOWEND80M);
   countPerHertz[1] = ((float)workingData.bandLimitPositionCounts[1][1] - (float)workingData.bandLimitPositionCounts[1][0]) / ((float)HIGHEND60M - (float)LOWEND60M);
   countPerHertz[2] = ((float)workingData.bandLimitPositionCounts[2][1] - (float)workingData.bandLimitPositionCounts[2][0]) / ((float)HIGHEND40M - (float)LOWEND40M);
   countPerHertz[3] = ((float)workingData.bandLimitPositionCounts[3][1] - (float)workingData.bandLimitPositionCounts[3][0]) / ((float)HIGHEND30M - (float)LOWEND30M);
   countPerHertz[4] = ((float)workingData.bandLimitPositionCounts[4][1] - (float)workingData.bandLimitPositionCounts[4][0]) / ((float)HIGHEND20M - (float)LOWEND20M);
   countPerHertz[5] = ((float)workingData.bandLimitPositionCounts[5][1] - (float)workingData.bandLimitPositionCounts[5][0]) / ((float)HIGHEND17M - (float)LOWEND17M);
   countPerHertz[6] = ((float)workingData.bandLimitPositionCounts[6][1] - (float)workingData.bandLimitPositionCounts[6][0]) / ((float)HIGHEND15M - (float)LOWEND15M);
   countPerHertz[7] = ((float)workingData.bandLimitPositionCounts[7][1] - (float)workingData.bandLimitPositionCounts[7][0]) / ((float)HIGHEND12M - (float)LOWEND12M);
   countPerHertz[8] = ((float)workingData.bandLimitPositionCounts[8][1] - (float)workingData.bandLimitPositionCounts[8][0]) / ((float)HIGHEND10M - (float)LOWEND10M);

   hertzPerStepperUnitVVC[0] = ((float)HIGHEND80M - (float)LOWEND80M) / ((float)workingData.bandLimitPositionCounts[0][1] - (float)workingData.bandLimitPositionCounts[0][0]);
   hertzPerStepperUnitVVC[1] = ((float)HIGHEND60M - (float)LOWEND60M) / ((float)workingData.bandLimitPositionCounts[1][1] - (float)workingData.bandLimitPositionCounts[1][0]);
   hertzPerStepperUnitVVC[2] = ((float)HIGHEND40M - (float)LOWEND40M) / ((float)workingData.bandLimitPositionCounts[2][1] - (float)workingData.bandLimitPositionCounts[2][0]);
   hertzPerStepperUnitVVC[3] = ((float)HIGHEND30M - (float)LOWEND30M) / ((float)workingData.bandLimitPositionCounts[3][1] - (float)workingData.bandLimitPositionCounts[3][0]);
   hertzPerStepperUnitVVC[4] = ((float)HIGHEND20M - (float)LOWEND20M) / ((float)workingData.bandLimitPositionCounts[4][1] - (float)workingData.bandLimitPositionCounts[4][0]);
   hertzPerStepperUnitVVC[5] = ((float)HIGHEND17M - (float)LOWEND17M) / ((float)workingData.bandLimitPositionCounts[5][1] - (float)workingData.bandLimitPositionCounts[5][0]);
   hertzPerStepperUnitVVC[6] = ((float)HIGHEND15M - (float)LOWEND15M) / ((float)workingData.bandLimitPositionCounts[6][1] - (float)workingData.bandLimitPositionCounts[6][0]);
   hertzPerStepperUnitVVC[7] = ((float)HIGHEND12M - (float)LOWEND12M) / ((float)workingData.bandLimitPositionCounts[7][1] - (float)workingData.bandLimitPositionCounts[7][0]);
   hertzPerStepperUnitVVC[8] = ((float)HIGHEND10M - (float)LOWEND10M) / ((float)workingData.bandLimitPositionCounts[8][1] - (float)workingData.bandLimitPositionCounts[8][0]);
}

void Data::writeDefaultValues()
{
   workingData.presetFrequencies[0][0] = 3503000;
   workingData.presetFrequencies[0][1] = 3504000;
   workingData.presetFrequencies[0][2] = 3600000;
   workingData.presetFrequencies[0][3] = 3615000;
   workingData.presetFrequencies[0][4] = 3750000;
   workingData.presetFrequencies[0][5] = 3900000;

   workingData.presetFrequencies[1][0] = 5330500; // 5330500, 5346500, 5357000, 5371500, 5403500, 5403500
   workingData.presetFrequencies[1][1] = 5330500;
   workingData.presetFrequencies[1][2] = 5346500;
   workingData.presetFrequencies[1][3] = 5357000;
   workingData.presetFrequencies[1][4] = 5371500;
   workingData.presetFrequencies[1][5] = 5403500;

   workingData.presetFrequencies[2][0] = 7030000;
   workingData.presetFrequencies[2][1] = 7040000;
   workingData.presetFrequencies[2][2] = 7100000;
   workingData.presetFrequencies[2][3] = 7150000;
   workingData.presetFrequencies[2][4] = 7250000;
   workingData.presetFrequencies[2][5] = 7285000;

   workingData.presetFrequencies[3][0] = 10106000;
   workingData.presetFrequencies[3][1] = 10116000;
   workingData.presetFrequencies[3][2] = 10120000;
   workingData.presetFrequencies[3][3] = 10130000;
   workingData.presetFrequencies[3][4] = 10140000;
   workingData.presetFrequencies[3][5] = 10145000;   

   workingData.presetFrequencies[4][0] = 14030000;
   workingData.presetFrequencies[4][1] = 14060000;
   workingData.presetFrequencies[4][2] = 14100000;
   workingData.presetFrequencies[4][3] = 14200000;
   workingData.presetFrequencies[4][4] = 14250000;
   workingData.presetFrequencies[4][5] = 14285000;

   workingData.presetFrequencies[5][0] = 14030000;
   workingData.presetFrequencies[5][1] = 14060000;
   workingData.presetFrequencies[5][2] = 14100000;
   workingData.presetFrequencies[5][3] = 14200000;
   workingData.presetFrequencies[5][4] = 14250000;
   workingData.presetFrequencies[5][5] = 14285000;   

   workingData.presetFrequencies[6][0] = 14030000;
   workingData.presetFrequencies[6][1] = 14060000;
   workingData.presetFrequencies[6][2] = 14100000;
   workingData.presetFrequencies[6][3] = 14200000;
   workingData.presetFrequencies[6][4] = 14250000;
   workingData.presetFrequencies[6][5] = 14285000;

   workingData.presetFrequencies[7][0] = 14030000;
   workingData.presetFrequencies[7][1] = 14060000;
   workingData.presetFrequencies[7][2] = 14100000;
   workingData.presetFrequencies[7][3] = 14200000;
   workingData.presetFrequencies[7][4] = 14250000;
   workingData.presetFrequencies[7][5] = 14285000;

   workingData.presetFrequencies[8][0] = 14030000;
   workingData.presetFrequencies[8][1] = 14060000;
   workingData.presetFrequencies[8][2] = 14100000;
   workingData.presetFrequencies[8][3] = 14200000;
   workingData.presetFrequencies[8][4] = 14250000;
   workingData.presetFrequencies[8][5] = 14285000;

   workingData.bandLimitPositionCounts[0][0] = 0;
   workingData.bandLimitPositionCounts[0][1] = 0;
   workingData.bandLimitPositionCounts[1][0] = 0;
   workingData.bandLimitPositionCounts[1][1] = 0;
   workingData.bandLimitPositionCounts[2][0] = 0;
   workingData.bandLimitPositionCounts[2][1] = 0;
   workingData.bandLimitPositionCounts[3][0] = 0;
   workingData.bandLimitPositionCounts[3][1] = 0;
   workingData.bandLimitPositionCounts[4][0] = 0;
   workingData.bandLimitPositionCounts[4][1] = 0;
   workingData.bandLimitPositionCounts[5][0] = 0;
   workingData.bandLimitPositionCounts[5][1] = 0;
   workingData.bandLimitPositionCounts[6][0] = 0;
   workingData.bandLimitPositionCounts[6][1] = 0;
   workingData.bandLimitPositionCounts[7][0] = 0;
   workingData.bandLimitPositionCounts[7][1] = 0;
   workingData.bandLimitPositionCounts[8][0] = 0;
   workingData.bandLimitPositionCounts[8][1] = 0;

   workingData.bandEdges[0][0] = LOWEND80M;
   workingData.bandEdges[0][1] = HIGHEND80M;
   workingData.bandEdges[1][0] = LOWEND60M;
   workingData.bandEdges[1][1] = HIGHEND60M;
   workingData.bandEdges[2][0] = LOWEND40M;
   workingData.bandEdges[2][1] = HIGHEND40M;
   workingData.bandEdges[3][0] = LOWEND30M;
   workingData.bandEdges[3][1] = HIGHEND30M;
   workingData.bandEdges[4][0] = LOWEND20M;
   workingData.bandEdges[4][1] = HIGHEND20M;
   workingData.bandEdges[5][0] = LOWEND17M;
   workingData.bandEdges[5][1] = HIGHEND17M;
   workingData.bandEdges[6][0] = LOWEND15M;
   workingData.bandEdges[6][1] = HIGHEND15M;
   workingData.bandEdges[7][0] = LOWEND12M;
   workingData.bandEdges[7][1] = HIGHEND12M;
   workingData.bandEdges[8][0] = LOWEND10M;
   workingData.bandEdges[8][1] = HIGHEND10M;

   workingData.lastFreq[0] =  3550000;
   workingData.lastFreq[1] =  5357000;
   workingData.lastFreq[2] =  7100000;
   workingData.lastFreq[3] = 10125000;
   workingData.lastFreq[4] = 14200000;
   workingData.lastFreq[5] = 18100000;
   workingData.lastFreq[6] = 21200000;
   workingData.lastFreq[7] = 24950000;
   workingData.lastFreq[8] = 28500000;         

   workingData.currentBand = 0;
   workingData.currentFrequency = 0;
   workingData.initialized = 0x55555555;  // 0x55555555 means the workingData struct has been initialized.
   workingData.calibrated = 0x00000000;   // Set to something other than 0 if calibrated.
   workingData.hardware = 0x00000000;     // 0 means hardware not accepted. 0x55555555 is accepted hardware.

   workingData.zero_offset = 480;  // zero offset 1000
   workingData.backlash = 50;       // backlash  50
   workingData.coarse_sweep = 5;   // coarse tune 20
   workingData.accel = 2000;        // acceleration 2000
   workingData.speed = 500;         // speed 500
   workingData.rotation = false;    // stepper motor rotation false
}
