
//====================================================================================================================================================
// Copyright 2022 Lake Orion Robotics FIRST Team 302
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================


#pragma once 
#include <memory>
#include <vector>

#include <frc/AddressableLED.h>


class DragonLeds
{
	public:

		enum LedColor
		{
			SOLID_RED,
			SOLID_GREEN,
			SOLID_BLUE
		};

		static DragonLeds* GetInstance();


		//------------------------------------------------------------------------------
		// Method:		<<constructor>>
		// Description:	Create Servos for use in robot mechanisms
		//------------------------------------------------------------------------------
		void Initialize
		(
			int 						deviceID,			// <I> - PWM ID
			int 						nleds   			// <I> - number of leds
		);
		bool IsInitialized() const;


		void SetColor
		(
			LedColor 	color
		);

		void SetRainbow();

	private:
		DragonLeds();
		virtual ~DragonLeds() = default;


		std::unique_ptr<frc::AddressableLED>        m_leds;
        int                                         m_num;
		std::vector<frc::AddressableLED::LEDData>   m_ledData;
		int											m_firstPixelHue;
		int											m_colorChaseStart;
		LedColor									m_color;

		static DragonLeds*	m_instance;

};
