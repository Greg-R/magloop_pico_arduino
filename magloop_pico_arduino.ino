/* A "proof of concept" project to replace the STM32F103 "Blue Pill"
   which is used in the "Magnetic Loop Controller" described in the book
   "Microcontroller Projects for Amateur Radio by Jack Purdum, W8TEE, and
   Albert Peter, AC8GY" with the Raspberry Pi Pico.
   This is the Arduino version.
   Copyright (C) 2024  Gregory Raven

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

// #define PICO_STACK_SIZE _u(0x1000)  // Uncomment if stack gets blown.  This doubles stack size.//
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_ILI9341.h>
#include "DisplayManagement.h"
#include <AccelStepper.h>
#include "StepperManagement.h"
#include "DDS.h"
#include "SWR.h"
#include <EEPROM.h>
#include "Data.h"
#include "Button.h"
#include "TuneInputs.h"
#include "TmcStepper.h"
#include "Hardware.h"

int currentFrequency;
int bypassTest = 5;  // Set to arbitrary value other than 0 or 10.

//  The data object manages constants and variables involved with frequencies, stepper motor positions,
//  and GPIOs.
Data data = Data();

//  Button objects.
Button enterbutton = Button(data.enterButton);
Button autotunebutton = Button(data.autotuneButton);
Button exitbutton = Button(data.exitButton);

//  The display object.  Note that the SPI is handled in the display object.
Adafruit_ILI9341 tft = Adafruit_ILI9341(&SPI1, 16, 13, -1);

//  Next instantiate the DDS.
DDS dds = DDS(data.DDS_RST, data.DDS_DATA, data.DDS_FQ_UD, data.WLCK);

// Instantiate SWR object.  Read bridge offsets later when other circuits are active.
SWR swr = SWR();

//  Instantiate the TMC stepper:
TmcStepper tmcstepper = TmcStepper();

//  Instantiate the Stepper Manager:
StepperManagement stepper = StepperManagement(tft, dds, swr, data, tmcstepper, AccelStepper::MotorInterfaceType::DRIVER, 0, 1);

//  The hardware object has the various test routines.
Hardware testArray = Hardware(tft, dds, swr, enterbutton, autotunebutton, exitbutton, data, stepper, tmcstepper);

// The TuneInputs object.  This object handles user interaction via the buttons and encoders.
TuneInputs tuneInputs = TuneInputs(tft, data, dds, enterbutton, autotunebutton, exitbutton, swr, tmcstepper);

// The DisplayManagement object.  This object has many important methods.  Because it has to touch everything it is large.
DisplayManagement display = DisplayManagement(tft, dds, swr, stepper, tmcstepper, data, enterbutton,
                                              autotunebutton, exitbutton, tuneInputs, testArray);

void setup() {
  // Initialize GPIOs:

  pinMode(0, OUTPUT);  // Stepper Step
  pinMode(1, OUTPUT);  // Stepper Dir
  pinMode(2, OUTPUT);  // RF Amp Power
  pinMode(3, OUTPUT);  // Op Amp Power

  pinMode(10, INPUT_PULLUP);  // Limit switch
  pinMode(11, INPUT_PULLUP);  // Limit switch
  pinMode(19, OUTPUT);        // RF relay

  digitalWrite(2, LOW);  // RF Amp Power off
  digitalWrite(3, LOW);  // Op Amp Power off
  //delay(2000);
  digitalWrite(19, LOW);  // RR relay off

  // Configure SPI1
  SPI1.setCS(13);
  SPI1.setSCK(14);
  SPI1.setTX(15);
  SPI1.setRX(12);
  SPI1.begin(true);

  // Initialize the display.
  tft.initSPI();
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(ILI9341_BLACK);

  // UART setup.  Only the transmit is used for configuring the TMC stepper driver.
  Serial2.setTX(8);
  Serial2.begin(115200);

  // Set the ADC resolution.  The ADC is used for SWR measurement.
  analogReadResolution(12);

  // Start the EEPROM and read the workingData struct into working memory.
  EEPROM.begin(256);

  EEPROM.get(0, data.workingData);  // Read the workingData struct from EEPROM.

  // Slopes can't be computed until the actual values are loaded from EEPROM.
  data.computeSlopes();

  //  Now examine the data in the buffer to see if the EEPROM should be initialized.
  //  There is a specific number written to the EEPROM when it is initialized.
  if (data.workingData.initialized != 0x55555555) {
    data.writeDefaultValues();  //  Writes default values in to the dataStruct in the Data object.
    EEPROM.put(0, data.workingData);
    EEPROM.commit();
    EEPROM.get(0, data.workingData);  // Read the workingData struct from EEPROM.
  } else EEPROM.get(0, data.workingData);

  enterbutton.initialize();
  exitbutton.initialize();
  autotunebutton.initialize();

  tmcstepper.initialize(data.workingData.rotation);  // Initialize TMC stepper using user selection rotation.
  tuneInputs.initialize();
  swr.ReadADCoffsets();  // To initialize; this is repeated later when the circuits are more thermally stable.

  // Initialize the stepper object.
  stepper.initialize();

  //  Initialize and onfigure the display object.
  tft.initSPI();
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(ILI9341_BLACK);

  // Power on all circuits except stepper and relay.  This is done early to allow circuits to stabilize before calibration.
  display.PowerStepDdsCirRelay(false, 7000000, true, false);

  dds.DDSWakeUp();  // This resets the DDS, and it will have no output.

  // Show "Splash" screen for 5 seconds.  This also allows circuits to stabilize.
  display.DrawRaven();
  display.Splash(data.version, data.releaseDate);
  delay(5000);
  tft.fillScreen(ILI9341_BLACK);  // Clear display.

  // Run initial tests if hardware has not been accepted.  Hardware is accepted by using the encoder bypass.
  if (data.workingData.hardware != 0x55555555) {
    display.updateMessageTop("Hardware Tests in Progress");
    //  Bypass tests by using MENU encoder.
    bypassTest = testArray.EncoderBypassTest();
    display.EraseBelowMenu();
    testArray.EraseTitle();
    if (bypassTest == 10) {
      data.workingData.hardware = 0x55555555;
      EEPROM.put(0, data.workingData);
      EEPROM.commit();  // Write to EEPROM.
    }
    if (bypassTest == 0) {
      testArray.InitialTests();  // Run hardware tests.
      display.ErasePage();
      display.updateMessageMiddle("Cycle Power to Restart");
    }
  }

  // Set stepper to zero.  DDS is off, and relay is off.
  // Don't do this after test is bypassed because the motor is not tested yet.
  // Also don't do this until the initial calibration is completed.
  if ((bypassTest != 10) and (data.workingData.calibrated == 1)) {
    display.PowerStepDdsCirRelay(true, 0, true, false);
    stepper.ResetStepperToZero();
  }

  //  Now measure the ADC (SWR bridge) offsets with the DDS inactive.
  //  Note that this should be done as late as possible for circuits to stabilize.
  display.PowerStepDdsCirRelay(true, 0, true, false);
  swr.ReadADCoffsets();
  display.PowerStepDdsCirRelay(false, 0, false, false);  //  Power down all circuits.

  //  Temporary code for examining system.
  // display.PowerStepDdsCirRelay(true, 0, false, false);  //  Stepper on only.
  // uart_write_blocking(uart1, display.tmcstepper.getCommand(display.tmcstepper.powerBrakingConfig), 8);
  // uart_write_blocking(uart1, display.tmcstepper.getCommand(display.tmcstepper.iHoldiRun), 8);
  // stepper.MoveStepperToPosition(4000);  // Preset stepper to some desired frequency.
  // display.manualTune();  //  Manual steps using encoder.
  // display.PowerStepDdsCirRelay(false, 7000000, true, false);

  display.menuIndex = display.TopMenuState::FREQMENU;  // Begin in Frequency menu.
}

void loop() {

  // Main loop state machine:

  //  Refresh display:
  display.ShowMainDisplay(display.menuIndex);  //  This function erases the entire display.
  display.ShowSubmenuData(display.minSWR, data.workingData.currentFrequency);

  display.menuIndex = display.MakeMenuSelection(display.menuIndex);  // Select one of the three top menu choices: Freq, Presets, 1st Cal.

  switch (display.menuIndex) {
    case display.TopMenuState::FREQMENU:  // Manual frequency selection selection and AutoTune.
      display.frequencyMenuOption();
      break;

    case display.TopMenuState::PRESETMENU:  // Preset frequencies by band - set in .ino file, variable: presetFrequencies[0][2];
      display.ProcessPresets();             // Select a preselected frequency.  This should return a frequency???
      break;

    case display.TopMenuState::CALIBRATEMENU:  // Run calibration routines.
      display.CalibrationMachine();
      break;

    default:
      display.menuIndex = display.TopMenuState::FREQMENU;
      break;
  }  // end of switch (menuIndex)

}  // end of loop()
