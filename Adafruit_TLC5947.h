/*!
 *  @file Adafruit_TLC5947.h
 *
 * 	Adafruit 24-channel PWM/LED driver
 *
 * 	This is a library for the Adafruit 24-channel PWM/LED driver:
 * 	http://www.adafruit.com/products/1429
 *
 *  These drivers uses SPI to communicate, 3 pins are required to
 *  interface: Data, Clock and Latch. The boards are chainable
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *  Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 *	BSD license (see license.txt)
 */

#ifndef _ADAFRUIT_TLC5947_H
#define _ADAFRUIT_TLC5947_H

#include <Arduino.h>

// modifications for fast digital writes
// taken from here:
// https://masteringarduino.blogspot.com/2013/10/fastest-and-smallest-digitalread-and.html
#define portOfPin(P)\
  (((P)>=0&&(P)<8)?&PORTD:(((P)>7&&(P)<14)?&PORTB:&PORTC))
#define ddrOfPin(P)\
  (((P)>=0&&(P)<8)?&DDRD:(((P)>7&&(P)<14)?&DDRB:&DDRC))
#define pinOfPin(P)\
  (((P)>=0&&(P)<8)?&PIND:(((P)>7&&(P)<14)?&PINB:&PINC))
#define pinIndex(P)((uint8_t)(P>13?P-14:P&7))
#define pinMask(P)((uint8_t)(1<<pinIndex(P)))

#define pinAsInput(P) *(ddrOfPin(P))&=~pinMask(P)
#define pinAsInputPullUp(P) *(ddrOfPin(P))&=~pinMask(P);digitalHigh(P)
#define pinAsOutput(P) *(ddrOfPin(P))|=pinMask(P)
#define digitalLow(P) *(portOfPin(P))&=~pinMask(P)
#define digitalHigh(P) *(portOfPin(P))|=pinMask(P)
#define isHigh(P)((*(pinOfPin(P))& pinMask(P))>0)
#define isLow(P)((*(pinOfPin(P))& pinMask(P))==0)
#define digitalState(P)((uint8_t)isHigh(P))

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            TLC5947 24-channel PWM/LED driver
 */
class Adafruit_TLC5947 {
public:
  Adafruit_TLC5947(uint16_t n, uint8_t c, uint8_t d, uint8_t l);

  boolean begin(void);

  void setPWM(uint16_t chan, uint16_t pwm);
  uint16_t getPWM(uint16_t chan);
  void setLED(uint16_t lednum, uint16_t r, uint16_t g, uint16_t b);
  void write();
  void write_fast();

private:
  uint16_t *pwmbuffer;

  uint16_t numdrivers;
  uint8_t _clk, _dat, _lat;
};

#endif
