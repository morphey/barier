/**
 * @file Battery.h
 * @brief Этот файл является реализацией класса Battery.
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

#ifndef BATTERY_H_
#define BATTERY_H_

#include <Thread.h>

/*!
*	\brief Класс для периодического мониторинга состояния батареи
*
*	Периодически мониторит состояние батареи по задаваемым параметрам АЦП.
*	Периодичность задается.
*/

class Battery: public Thread
{

	private:
		double md_value = 0;	//!< Посчитанное напряжение
		int mi_pin = 0;			//!< Номер пина АЦП
		double md_gain = 0;		//!< Коэфициент усиления
		double md_offset = 0;	//!< Смещение

	public:
		/**
		 * \brief Установка коэфициента усиления (поправка)
		 *
		 * Функция устанавливает коэффициент усиления для преобразования
		 * в реальные значения напряжения
		 *
		 * @param gain коэфициент усиления
		 */
		inline void setGain(double gain)
		{
			md_gain = gain;
		}

		/**
		 * \brief Установка коэфициента смещения (поправка)
		 *
		 * Функция устанавливает коэффициент усиления для преобразования
		 * в реальные значения напряжения
		 *
		 * @param offset смещение
		 */
		inline void setOffset(double offset)
		{
			md_offset = offset;
		}

		/**
		* \brief Установка пина АЦП
		*
		* Функция устанавливает пин АЦП на котором производить измерения
		*
		* @param pin номер пина АЦП
		*/
		inline void setPin(uint8_t pin)
		{
			mi_pin = pin;
		}

		/**
		 * \brief Получение реального напряжения с учетом поправок
		 *
		 * Метод возвращает текущее напряжение измеренное АЦП с учетом поправок
		 * @return Текущее напряжение измеренное АЦП
		 */
		inline double getVoltage()
		{
//			return md_value * 0.003225806 + 1.68;
			return md_value * md_gain + md_offset;
		}

		/**
		 * Собственно основная функция треда (непосредственно измерение)
		 */
		void run()
		{
			if (mi_pin != 0)
			{
				// Reads the analog pin, and saves it localy
				md_value = analogRead(mi_pin);
				runned();
			}
		}
};

#endif /* BATTERY_H_ */
