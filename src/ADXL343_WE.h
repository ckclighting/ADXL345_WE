/******************************************************************************
 *
 * This is a library for the ADXL345 / ADXL343 accelerometer.
 *
 * You'll find several example sketches which should enable you to use the library.
 *
 * You are free to use it, change it or build on it. In case you like it, it would
 * be cool if you give it a star.
 *
 * If you find bugs, please inform me!
 *
 * Written by Wolfgang (Wolle) Ewald
 * https://wolles-elektronikkiste.de/adxl345-teil-1 (German)
 * https://wolles-elektronikkiste.de/en/adxl345-the-universal-accelerometer-part-1 (English)
 *
 *
 ******************************************************************************/

#pragma once

#include "ADXL345_WE.h"
#include "xyzFloat.h"

class ADXL343_WE : public ADXL345_WE
{
  public:
    using ADXL345_WE::ADXL345_WE;
};
