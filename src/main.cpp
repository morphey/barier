/**
 * @file main.cpp
 * @brief Этот файл является частью проекта Барьер-1
 * @author Danila E. Evstropov <devstropov@vtsvl.ru>
 * @date 12 июл. 2016 г.
 *
 * @par This file is a part of generator project
 * @par Copyright:
 * Copyright (c) 2016 Danila E. Evstropov <devstropov@vtsvl.ru> \n\n
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version. \n\n
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


#define SERIAL_TX_BUFFER_SIZE 256
#define SERIAL_RX_BUFFER_SIZE 256

#include <Arduino.h>
#include <WString.h>

#include <TimerOne.h>
#include <ThreadController.h>

#include <SoftwareSerial.h>
#include <ESP8266.h>

#include "Battery.h"

#define UDP_PORT	1234

const char* ssid = "VTS";			//!< Ssid сети
const char* password = "76543210";	//!< Пароль сети

//Wi-Fi-manager Object
ESP8266	 *wifi;

//Battery state checker thread
Battery *batCheck;

//Thread Controller
ThreadController *controller;

//System timer value
//extern volatile unsigned long timer0_millis;

//PWM value for 36kHz
const uint8_t pwmval = F_CPU / 2000 / 36;	//!< ШИМ настройка на 36 кГц

int voltage;
double realv;
double gain = 3.3/1023;
double offset = 1.68;


union TimeStamp
{
		uint32_t timestamp_ALL;
		uint8_t timestamp_Array[4];
};

uint8_t result = 0;
TimeStamp tms;
uint32_t time_offset = 0;
uint16_t net_delay = 0;

String currentIP = "192.168.0.72";

unsigned char buffer[256]={0};
unsigned char c;

String str;

SoftwareSerial debug (8,9);

//bool syncTimestamps ();

//Threads controller check
void timerCallback(){
	controller -> run();
};

void pwmStart()
{
	TCCR2A = _BV(WGM20);
	TCCR2B = _BV(WGM22) | _BV(CS20);
	OCR2A = pwmval;
	OCR2B = pwmval / 3;
	TCCR2A |= _BV(COM2B1);
};

void setup ()
{
	//	Setting up thread of battery checker object
	batCheck = new Battery();
	batCheck -> setPin(A7);
	batCheck -> setGain(gain);
	batCheck -> setOffset(offset);
	batCheck -> setInterval(5000);
	//	Setting up threads controller
	controller = new ThreadController();
	controller -> add(batCheck);

	//	Setting up Timer1 interrupt to check threads once in 20ms
	Timer1.initialize(20000);
	Timer1.attachInterrupt(timerCallback);
	Timer1.start();

	//	Setting up WiFi module
	wifi = new ESP8266(Serial,19200);
	String test;
	debug.begin(9600);
	//	Serial.begin(76800);

	pinMode (3, OUTPUT);
	pinMode (4, INPUT);
	pinMode (5, OUTPUT);

	pwmStart();

	debug.print ("setup begin\r\n");
	debug.print ("FW Version: ");
	debug.println (wifi->getVersion().c_str());

	if (wifi->setOprToStation())
	{
		debug.print ("to Station ok\r\n");
	}
	else
	{
		debug.print ("to Station err\r\n");
	}

	if (wifi->joinAP (ssid, password))
	{
		debug.print ("Join AP success\r\n");
		debug.print ("IP: ");
		debug.println (wifi->getLocalIP().c_str());
	}
	else
	{
		debug.print ("Join AP failure\r\n");
	}

	if (wifi->disableMUX())
	{
		debug.print("single ok\r\n");
	}
	else
	{
		debug.print("single err\r\n");
	}

	if (!wifi->registerUDP (currentIP, UDP_PORT))
	{
		debug.println("Cannot create UDP client connection!");
	}
	else
	{
		debug.print("UDP client started!\r\n");
	}

	debug.print("setup end\r\n");
//	syncTimestamps();
}

void loop ()
{

	result = digitalRead(4);

	digitalWrite(5,~result);
	delay (1000);
	tms.timestamp_ALL = millis();

	debug.print("ADC: ");
	debug.println(batCheck->getVoltage());

//	str = String(tms.timestamp_ALL)+"\n";

//	buffer[0] = tms.timestamp_HH;
//	buffer[1] = tms.timestamp_HL;
//	buffer[2] = tms.timestamp_LH;
//	buffer[3] = tms.timestamp_LL;
//	wifi->send(&result,sizeof(uint8_t));
	wifi->send(tms.timestamp_Array,sizeof(uint8_t)*4);
	wifi -> recv()
}

//bool syncTimestamps ()
//{
//
////	uint8_t buffer[128] = {0};
//	uint8_t buffer[16] = {0};
//
//	if (!wifi->registerUDP (currentIP, UDP_PORT))
//	{
//		debug.println("Cannot create UDP client connection!");
//		return false;
//	}
//	sndc_t = millis();
//	buffer[0] = sndc_t & 0xFF;
//	buffer[1] = (sndc_t >> 8) & 0xFF;
//	buffer[2] = (sndc_t >> 16) & 0xFF;
//	buffer[3] = (sndc_t >> 24) & 0xFF;
//
//	wifi->send(buffer, sizeof(buffer));
//	uint32_t len = wifi->recv(buffer, sizeof(buffer), 10000);
//	rcvc_t = millis();
////	uint16_t timestamp_ms = 0, sndc_t = 0, snds_t = 0, rcvs_t = 0, rcvc_t = 0;
////	θ = {[(T2 + dAB) − T1] + [T3 − (T4 + dBA)]} / 2,
////	T1 - sndc_t
////	T2 - rcvs_t
////	T3 - snds_t
////	T4 - rcvc_t
//	rcvs_t = buffer[7];
//	rcvs_t = rcvs_t << 8;
//	rcvs_t |= buffer[6];
//	rcvs_t = rcvs_t << 8;
//	rcvs_t |= buffer[5];
//	rcvs_t = rcvs_t << 8;
//	rcvs_t |= buffer[4];
//
//	snds_t = buffer[11];
//	snds_t = snds_t << 8;
//	snds_t |= buffer[10];
//	snds_t = snds_t << 8;
//	snds_t |= buffer[9];
//	snds_t = snds_t << 8;
//	snds_t |= buffer[8];
//
//	time_offset = ((rcvs_t - sndc_t) + (snds_t - rcvc_t))/2;
//	cli();
//	timer0_millis += time_offset;
//	sei();
//	debug.print("time offset is: ");
//	debug.print(time_offset);
//	debug.print("\r\n");
//
//	if (len > 0)
//	{
//		debug.print("Received:[");
//        for(uint32_t i = 0; i < len; i++)
//        {
//            debug.print((char)buffer[i]);
//        }
//        debug.print("]\r\n");
//	}
//
//
//
//	timestamp_ms = millis () + time_offset;
//	debug.print ("Current time: ");
//	debug.print (timestamp_ms - time_offset);
//	debug.print ("\nCurrent time corrected: ");
//	debug.print (timestamp_ms);
//	debug.print ("\nCurrent offset: ");
//	debug.print (time_offset);
//	return true;
//}

