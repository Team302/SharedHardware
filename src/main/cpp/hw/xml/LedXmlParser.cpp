
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


// C++ Includes
#include <memory>

// FRC includes

// Team 302 includes
#include <hw/DragonLeds.h>
#include <hw/xml/LedXmlParser.h>
#include <utils/HardwareIDValidation.h>
#include <utils/Logger.h>

// Third Party Includes
#include <pugixml/pugixml.hpp>

using namespace std;

    //-----------------------------------------------------------------------
    // Method:      ParseXML
    // Description: Parse a servo XML element and create a DragonServo from
    //              its definition.
    //
    //
    // Returns:     void        
    //-----------------------------------------------------------------------
    DragonLeds* LedXmlParser::ParseXML
    (
        pugi::xml_node      ledNode
    )
    {
        // initialize attributes to default values
        int pwmID = 0;
        int numLeds = 5460;

        bool hasError = false;

        // parse/validate the xml
        for (pugi::xml_attribute attr = ledNode.first_attribute(); attr && !hasError; attr = attr.next_attribute())
        {
            string attrName (attr.name());
            if (attrName.compare("pwmId" ) == 0)
            {
                pwmID = attr.as_int();
                hasError = HardwareIDValidation::ValidateDIOID( pwmID, string( "ServoXmlParser::ParseXML(PWM ID)" ) );
            }
            else if (attrName.compare("number" ) == 0)
            {
                numLeds = attr.as_int();
            }
            else
            {
                string msg = "unknown attribute ";
                msg += attr.name();
                Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, string("LedXmlParser"), string("ParseXML"), msg );
                hasError = true;
            }
        }

        // create the object
        if ( !hasError )
        {
            auto leds = DragonLeds::GetInstance();
            if (leds != nullptr && !leds->IsInitialized())
            {
                leds->Initialize(pwmID, numLeds);
                return leds;
            }
        }
        return nullptr;
    }
