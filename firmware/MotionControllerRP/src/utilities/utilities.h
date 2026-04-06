// --------------------------------------------------------------------------------------
// Project: MicroManipulatorStepper
// License: MIT (see LICENSE file for full description)
//          All text in here must be included in any redistribution.
// Author:  M. S. (diffraction limited)
// --------------------------------------------------------------------------------------
#pragma once
#include <string>
#include <vector>
#include <stdint.h>

//*** FUNCTIONS *************************************************************************

std::vector<std::string> get_file_list(const char* dirname, bool include_dirs);

//*** Classes *************************************************************************
typedef union FourByteArray {
    uint32_t integer;
    uint8_t bytes[sizeof(uint32_t)];
} FourByteArray;

