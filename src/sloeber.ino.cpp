#ifdef __IN_ECLIPSE__
//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2022-02-18 22:47:10

#include "Arduino.h"
#define FREQUENCY     160
#include "ESP8266WiFi.h"
extern "C" {
#include "user_interface.h"
}
#include <Wire.h>
#include <eeprom.h>
#include "OneButton.h"
#include "UrsADC.h"
#include <Adafruit_GFX.h>
#include <TFT_eSPI.h>
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"

short get_spoint(short *p, uint8_t rows, uint8_t cols, int8_t x, int8_t y) ;
double eep_read_double(int address) ;
void eep_write_double(int address, double value) ;
double read_batt(void) ;
void toggle_cal_flag(void) ;
void toggle_volt_flag(void) ;
void init_dragging(void) ;
void next_mode(void) ;
void do_maxp(void) ;
void do_maxm(void) ;
void do_minp(void) ;
void do_minm(void) ;
void power_off(void) ;
void show_batt(void) ;
boolean isConnected() ;
void setup() ;
void loop() ;
void drawpixels(short *p, uint8_t rows, uint8_t cols, uint8_t boxWidth, uint8_t boxHeight) ;

#include "thermo-cam_V3.ino"


#endif
