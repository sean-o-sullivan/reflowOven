// bitmap_image.h

#ifndef BITMAP_IMAGE_H
#define BITMAP_IMAGE_H

#ifdef ESP32
  #include <pgmspace.h>  // include pgmspace for ESP32 if needed
#else
  #include <avr/pgmspace.h>
#endif

extern const unsigned short Untitled_design_18[80000];

#endif
