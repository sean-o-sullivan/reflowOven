// bitmap_image.h

#ifndef FLAME_IMAGE_H
#define FLAME_IMAGE_H

#ifdef ESP32
  #include <pgmspace.h>  // pgmspace for ESP32 if needed
#else
  #include <avr/pgmspace.h>
#endif

// Declare the external bitmap array
extern const unsigned short Untitled_design_20[2500];

#endif
