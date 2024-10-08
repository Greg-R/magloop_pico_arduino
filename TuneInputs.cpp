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

#include "TuneInputs.h"

TuneInputs::TuneInputs(Adafruit_ILI9341 &tft, Data &data, DDS& dds, Button &enterbutton,  
                       Button &autotunebutton, Button &exitbutton, SWR& swr, TmcStepper &tmcstepper)
                     :  DisplayUtility(tft, dds, swr, data, tmcstepper), tft(tft), data(data), // data(data), Does order of initialization make a difference in this case?
                       enterbutton(enterbutton), autotunebutton(autotunebutton), exitbutton(exitbutton), swr(swr), tmcstepper(tmcstepper)
{

}

void TuneInputs::initialize()
{
  parameters[0] = data.workingData.zero_offset;
  parameters[1] = data.workingData.backlash;
  parameters[2] = data.workingData.coarse_sweep;
  parameters[3] = data.workingData.accel;
  parameters[4] = data.workingData.speed;
  parameters[5] = data.workingData.rotation;
  parameterNames = {"Zero Offset", "Backlash", "Coarse Sweep", "Acceleration", "Speed", "Motor Rotation"};
  submenuIndex = 0;
}
// Used in Hardware Settings to select a parameter.
// If the Enter button is pressed, the hardware parameter can be changed and saved.
// Exit saves and moves back up to the Calibrate menu.
void TuneInputs::SelectParameter()
{
//  int frequency;
  bool lastexitbutton = true;
  bool lastenterbutton = true;
  int32_t parameter;
  std::vector<std::string> rotations = {"Forward", "Reverse"};
  
  menuEncoderState = 0;
  state = State::state0; // Enter state0 which does graphics.
  tft.fillScreen(ILI9341_BLACK);
  //  Preset state selection machine
  while (true)
  {
    // Poll 2 buttons:
    enterbutton.buttonPushed();
    exitbutton.buttonPushed();

    switch (state)
    {
    case State::state0: // This state does the graphics.
      updateMessageTop("Menu Encoder to select, Enter to Adjust");
      EraseBelowMenu();
      tft.setTextSize(1); 
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK); 
      // Show current hardware parameters
      for (int i = 0; i < 6; i++)
      {
        tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
        tft.setFont(&FreeSerif12pt7b);
        tft.setCursor(30, 70 + i * 30);
        tft.print(i + 1);
        tft.print(".");
        tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
        tft.setCursor(65, 70 + i * 30);
        tft.print(parameterNames[i].c_str());
        tft.setCursor(225, 70 + i * 30);
        tft.setFont(&FreeMono12pt7b);
        tft.print(parameters[i]);
      }
      tft.setTextColor(ILI9341_MAGENTA, ILI9341_WHITE);
      tft.setCursor(225, 70 + submenuIndex * 30);
      tft.print(parameters[submenuIndex]);  // Highlight selection.
      state = State::state1;
      break;

    case State::state1: // This state reads the encoders and button pushes.
    menuEncoderPoll();
      if (menuEncoderMovement == 1)
      { // Turning clockwise
        RestorePreviousChoice(submenuIndex);
        submenuIndex++;
        if (submenuIndex > 5)
          submenuIndex = 0;
        HighlightNextChoice(submenuIndex);
        menuEncoderMovement = 0;
      }
      if (menuEncoderMovement == -1)
      { // Tuning counter-clockwise
        RestorePreviousChoice(submenuIndex);
        submenuIndex--;
        if (submenuIndex < 0)
          submenuIndex = 5;
        HighlightNextChoice(submenuIndex);
        menuEncoderMovement = 0;
      }
      // Go to the chosen parameter and change it.
      if (enterbutton.pushed & not lastenterbutton)
      {
        // This switch was necessary because it was not convenient to put the parameters in an array.
        // It might be possible to use some form of C++ pointer array instead.
        switch(submenuIndex)
        {
          case 0:
            parameter = data.workingData.zero_offset;
            parameter = ChangeParameter(0, 2000, parameter);
            data.workingData.zero_offset = parameter;
            parameters[0] = parameter;
            break;
          case 1:
            parameter = data.workingData.backlash;
            parameter = ChangeParameter(0, 200, parameter);
            data.workingData.backlash = parameter;
            parameters[1] = parameter;
            break;
          case 2:
            parameter = data.workingData.coarse_sweep;
            parameter = ChangeParameter(0, 50, parameter);
            data.workingData.coarse_sweep = parameter;
            parameters[2] = parameter;
            break;
          case 3:
            parameter = data.workingData.accel;
            parameter = ChangeParameter(100, 2000, parameter);
            data.workingData.accel = parameter;
            parameters[3] = parameter;
            break;
          case 4:
            parameter = data.workingData.speed;
            parameter = ChangeParameter(100, 1000, parameter);
            data.workingData.speed = parameter;
            parameters[4] = parameter;
            break;

            case 5:
            parameter = data.workingData.rotation;
            parameter = ChangeParameter(0, 1, parameter);  // Rotation has only 2 values!
            data.workingData.rotation = parameter;
            parameters[5] = parameter;

            break;

          default:
          break;
        }
        tmcstepper.initialize(data.workingData.rotation);  // Initialize TMC stepper using user selection rotation.
        lastexitbutton = true;  // Prevents exit button from skipping a level.
        EEPROM.put(0, data.workingData);  // Save parameters to EEPROM.
        EEPROM.commit();
        //  Need to refresh graphics, because they were changed by ChangeFrequency!
        state = State::state0; // Refresh the graphics.
      }
      lastenterbutton = enterbutton.pushed;

      break;
    default:
      break;
    } // end switch of state machine
      // Process button pushes after switch statement, but before end of while block.
    if (exitbutton.pushed & not lastexitbutton)
      break; // Exit preset select, return the frequency and proceed to AutoTune.
    lastexitbutton = exitbutton.pushed;

  }   // end while SelectParameter state selection machine
}


/*****
  Purpose: Set new frequency
  This needs to be re-written as a state machine.???
  Argument list:
    int bandIndex    Which of the three bands was selected?
    long frequency   The current frequency.

  Return value:
    The new frequency is returned.

  Dependencies:  DDS, SWR, Adafruit_ILI9341
*****/
int32_t TuneInputs::ChangeParameter(int32_t minValue, int32_t maxValue, int32_t frequency) // Al Mod 9-8-19
{
  int32_t halfScreen, insetMargin;
  insetMargin = 15;
  halfScreen = data.PIXELHEIGHT / 2 - 25;
  digitEncoderMovement = 0;
  menuEncoderMovement = 0;
  updateMessageTop("          Enter New Hardware Parameter");
  tft.drawFastHLine(0, 20, 320, ILI9341_RED);
  //  The following configures the display for parameter selection mode.
  EraseBelowMenu();
  tft.setTextSize(1);
  tft.setFont(&FreeSerif9pt7b);
  tft.setTextColor(ILI9341_WHITE); // Messages
  tft.setCursor(insetMargin, halfScreen + 60);
  tft.print("Increment:");
  tft.setCursor(insetMargin + 90, halfScreen + 60);
  tft.print("Menu Encoder");
  tft.setCursor(insetMargin, halfScreen + 80);
  tft.print("Digit:");
  tft.setCursor(insetMargin + 90, halfScreen + 80);
  tft.print("Frequency Encoder");
  tft.setCursor(insetMargin, halfScreen + 100);
  tft.setCursor(insetMargin + 90, halfScreen + 100);
  tft.setCursor(insetMargin, halfScreen + 120);
  tft.print("Exit:");
  tft.setCursor(insetMargin + 90, halfScreen + 120);
  tft.print("Exit Button");
  // End of custom code for this function.

return ConstrainedNumericInput(exitbutton, exitbutton, minValue, maxValue, frequency);
}

// These functions are slightly different than the ones in the DisplayUtility class.
void TuneInputs::RestorePreviousChoice(int submenuIndex)
{
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK); // restore old background
  tft.setCursor(225, 70 + submenuIndex * 30);
  tft.print(parameters[submenuIndex]);
}

void TuneInputs::HighlightNextChoice(int submenuIndex)
{
  tft.setTextColor(ILI9341_MAGENTA, ILI9341_WHITE); // HIghlight new preset choice
  tft.setCursor(225, 70 + submenuIndex * 30);
  tft.print(parameters[submenuIndex]);
}


void TuneInputs::EncoderTest() {
 // Encoders Test
 int freqEncoderCount = 0;
 int count = 0;
 while(true) {
//  busy_wait_ms(5);
    tft.setCursor(10, 130);
    tft.print("FREQ Encoder");
    freqEncoderPoll();
    if(frequencyEncoderMovement2) {
      freqEncoderCount += frequencyEncoderMovement2;
      frequencyEncoderMovement2 = 0;
      tft.fillRect(192, 113, 40, 20, ILI9341_BLACK); 
    }
    tft.setCursor(200, 130);
    tft.print(freqEncoderCount);

    tft.setCursor(200, 160);
    tft.print(count);
    count = count + 1;
}
}


int32_t TuneInputs::ConstrainedNumericInput(Button buttonAccept, Button buttonReject, int32_t minValue, int32_t maxValue, int32_t number)
{
  int32_t digitSpacing, halfScreen, insetMargin, offset, cursorHome;
  int32_t defaultIncrement = 1;
  int32_t cursorOffset = 0;
  uint32_t numberSize;
  digitSpacing = 28;
  insetMargin = 15;
  defaultIncrement = 1;
  halfScreen = data.PIXELHEIGHT / 2 - 25;
  bool lastAcceptButton = true;
  bool lastRejectButton = true;
  digitEncoderMovement = 0;
  menuEncoderMovement = 0;

  // Determine the size of the number:
  numberSize = std::to_string(number).size();
  offset = 10 - numberSize;

  // First, print the number and the cursor (underscore) to the display.
  tft.setTextSize(1);
  tft.setFont(&FreeMono24pt7b);
  tft.setTextColor(ILI9341_WHITE);
  // Initially place cursor under single digit.
  cursorHome = 10 + digitSpacing * offset + digitSpacing * (numberSize - 1); // The initial placement of the cursor.
  tft.setCursor(cursorHome, halfScreen + 5);                                 // Assume 1KHz increment
  tft.print("_");                                                            // underline selected character position
  tft.setTextColor(ILI9341_GREEN);
  tft.setCursor(10 + digitSpacing * offset, halfScreen);
  tft.setTextSize(1);
  tft.setFont(&FreeMono24pt7b);
  tft.print(number);
  // State Machine for frequency input with encoders.
  while (true)
  { // Update number until user pushes button.
    menuEncoderPoll();
    freqEncoderPoll();
    // Poll accept button.
    buttonAccept.buttonPushed();
    if (buttonAccept.pushed & not lastAcceptButton)
    {
      break; // Break out of the while loop.
    }
    lastAcceptButton = buttonAccept.pushed;

    // Don't look at the reject button if it is the same as the accept.
    if (buttonReject.gpio != buttonAccept.gpio)
    {
      buttonReject.buttonPushed();
      if (buttonReject.pushed & not lastRejectButton)
      {
        return number = 0; // Exit, don't change, returning 0 means exit.
      }
      lastRejectButton = buttonReject.pushed;
    }

    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.setFont(&FreeMono24pt7b);
    // Handle movement of the cursor to the right.
    if (digitEncoderMovement == 1)
    {                                                                            // Change frequency digit increment
      tft.fillRect(0, halfScreen + 6, data.PIXELWIDTH * .91, 20, ILI9341_BLACK); // Erase existing cursor.
      defaultIncrement = defaultIncrement / 10;                                  // Move to right, so divide by 10.
      if (defaultIncrement < 1)
      { // Don't go too far right, reset defaultIncrement.
        defaultIncrement = 1;
      }
      else
        cursorOffset = cursorOffset + digitSpacing; // Move cursor to the right.
      // Don't allow increments > 1000000.
      if (defaultIncrement > 1000000)
      {
        defaultIncrement = 1000000;
      }
      if (cursorOffset > digitSpacing * 6)
      { // Don't overshoot or...
        cursorOffset = cursorOffset - digitSpacing;
      }
      tft.setCursor(cursorHome + cursorOffset, halfScreen + 5); //
      tft.print("_");
      digitEncoderMovement = 0;
    }
    else
    {
      // Handle movement of the cursor to the left.
      if (digitEncoderMovement == -1)
      {
        tft.fillRect(0, halfScreen + 6, data.PIXELWIDTH * .91, 20, ILI9341_BLACK); // Erase existing cursor.
        defaultIncrement = defaultIncrement * 10;
        if (defaultIncrement > 1000000)
        { // Don't go too far left
          defaultIncrement = 1000000;
        }
        else
          cursorOffset = cursorOffset - digitSpacing; // Move cursor to the left.
        if (cursorOffset < -digitSpacing * 6)         // Don't undershoot either
          cursorOffset = cursorOffset + digitSpacing;

        tft.setCursor(cursorHome + cursorOffset, halfScreen + 5);
        tft.print("_");
        digitEncoderMovement = 0;
      }
    }
    tft.setTextColor(ILI9341_GREEN);
    digitEncoderMovement = 0;
    menuEncoderMovement = 0;
    // Change digit value using the Frequency encoder.  This only has to refresh the number.
    if (frequencyEncoderMovement)
    {
      number += (int32_t)(frequencyEncoderMovement * defaultIncrement);

      if(number > maxValue) number = maxValue;
      if(number < minValue) number = minValue;

      tft.fillRect(insetMargin, halfScreen - 35, data.PIXELWIDTH * .91, 40, ILI9341_BLACK); // Erase the old number?

      numberSize = std::to_string(number).size();
      offset = 10 - numberSize;

      tft.setCursor(10 + digitSpacing * offset, halfScreen);
      tft.setTextSize(1);
      tft.setFont(&FreeMono24pt7b);
      tft.print(number);
      frequencyEncoderMovement = 0; // Reset encoder flag
    }
  } // end while loop

  tft.setTextSize(2); // Back to normal
  tft.setTextColor(ILI9341_WHITE);
  digitEncoderMovement = 0;
  menuEncoderMovement = 0;
  return number;
}