// Configuration parameters.  To be adjusted by user.

#pragma once

#include <cstdint>
#include <vector>

#define NUMBER_BANDS 3

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

uint32_t band0 = EIGHTY_METERS;
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