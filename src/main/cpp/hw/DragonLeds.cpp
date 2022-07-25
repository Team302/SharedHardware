
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

#include <memory>
#include <string>
#include <vector>

#include <hw/DragonLeds.h>
#include <frc/AddressableLED.h>
#include <utils/Logger.h>

using namespace frc;
using namespace std;

DragonLeds* DragonLeds::m_instance = nullptr;
DragonLeds* DragonLeds::GetInstance()
{
	if ( DragonLeds::m_instance == nullptr )
	{
		DragonLeds::m_instance = new DragonLeds();
	}
	return DragonLeds::m_instance;
}

DragonLeds::DragonLeds() :  m_leds(),
                            m_num(0),
                            m_ledData(),
                            m_firstPixelHue(0),
                            m_colorChaseStart(0)
{
}
void DragonLeds::Initialize
(
	int 						deviceID,			// <I> - PWM ID
	int  						numLeds 			// <I> - number of LEDs

) 
{
    if (!IsInitialized())
    {
        m_leds = std::make_unique<frc::AddressableLED>(deviceID);
        m_num = numLeds;
        m_firstPixelHue = 0;
        m_colorChaseStart = 0;
        m_ledData.resize(numLeds);
        for (auto i=0; i<numLeds; ++i)
        {
            m_ledData[i].SetRGB(0, 255, 0);
        }
    }
    else
    {
        Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, string("DragonLeds"), string("Already defined"), string("Only one allowed"));
    }
}

bool DragonLeds::IsInitialized() const
{
    return m_num > 0;
}


void DragonLeds::SetColor
(
    LedColor   color
)
{
    switch (color)
    {
        case SOLID_RED:
            for (auto i=0; i<m_num; ++i)
            {
                m_ledData[i].SetRGB(255, 0, 0);
            }
            break;
    case SOLID_GREEN:
            for (auto i=0; i<m_num; ++i)
            {
                m_ledData[i].SetRGB(0, 255, 0);
            }
        break;
    case SOLID_BLUE:
            for (auto i=0; i<m_num; ++i)
            {
                m_ledData[i].SetRGB(0, 0, 255);
            }
        break;
    
    default:
        break;
    }

    if (color != m_color)
    {
        for (auto i=m_colorChaseStart; i<m_colorChaseStart+3 && i<m_num; ++i)
        {
            m_ledData[i].SetRGB(0, 0, 0);
        }
        m_colorChaseStart += 3;
    }
    else
    {
        m_colorChaseStart =  0;
    }
    m_firstPixelHue = 0;
    m_color = color;
}

void DragonLeds::SetRainbow()
{
    for (auto i=0; i<m_num; ++i) 
    {
        // Calculate the hue - hue is easier for rainbows because the color
        // shape is a circle so only one value needs to precess
        const auto pixelHue = (m_firstPixelHue + (i * 180 / m_num)) % 180;
        // Set the value
        m_ledData[i].SetHSV(pixelHue, 255, 128);
    }
    m_firstPixelHue += 3;  // Increase by to make the rainbow "move"
    m_firstPixelHue %= 180;  // Check bounds
    m_colorChaseStart =  0;
}