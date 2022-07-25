
//====================================================================================================================================================
/// Copyright 2022 Lake Orion Robotics FIRST Team 302
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
/// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
/// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
/// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
/// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

//========================================================================================================
/// @class MechansimXmlParser
/// @brief Create a mechaism from an XML definition 
//========================================================================================================

// C++ Includes
#include <memory>
#include <string>
#include <utility>

// FRC includes

// Team 302 includes
#include <hw/interfaces/IDragonMotorController.h>
#include <hw/usages/AnalogInputMap.h>
#include <hw/usages/DigitalInputMap.h>
#include <hw/usages/DragonSolenoidMap.h>
#include <hw/usages/IDragonMotorControllerMap.h>
#include <hw/usages/ServoMap.h>
#include <hw/DragonAnalogInput.h>
#include <basemechanisms/interfaces/IMech.h>
#include <mechanisms/MechanismFactory.h>
#include <mechanisms/MechanismTypes.h>
#include <utils/Logger.h>
#include <hw/xml/AnalogInputXmlParser.h>
#include <hw/xml/CancoderXmlParser.h>
#include <hw/xml/DigitalInputXmlParser.h>
#include <basemechanisms/MechanismXmlParser.h>
#include <hw/xml/MotorXmlParser.h>
#include <hw/xml/ServoXmlParser.h> 
#include <hw/xml/SolenoidXmlParser.h>

// Third Party Includes
#include <pugixml/pugixml.hpp>


using namespace frc;
using namespace pugi;
using namespace std;



/// @brief  Parse a Mechanism XML element and create an IMechanism from its definition.
/// @return IMechanism*   pointer to the mechanism
void MechanismXmlParser::ParseXML
(
    xml_node      mechanismNode
)
{
    // initialize attributes
    MechanismTypes::MECHANISM_TYPE type = MechanismTypes::UNKNOWN_MECHANISM;

    bool hasError       = false;
    string networkTableName;
    string controlFileName;

    // Parse/validate xml
    for (xml_attribute attr = mechanismNode.first_attribute(); attr; attr = attr.next_attribute())
    {
        string attrName (attr.name());
        if ( attrName.compare("type") == 0 )
        {
            string typeStr = attr.as_string();
            for_each( typeStr.begin(), typeStr.end(), [](char & c){c = ::toupper(c);});

            type = MechanismTypes::GetInstance()->GetType(typeStr);
        }
        else if ( attrName.compare("networkTable") == 0 )
        {
            networkTableName = attr.as_string();
        }
        else if ( attrName.compare("controlFile") == 0 )
        {
            controlFileName = attr.as_string();
        }
        else
        {
            string msg = "invalid attribute ";
            msg += attr.name();
            Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, string("MechanismXmlParser"), string("ParseXML"), msg );
            hasError = true;
        }
    }

    // Parse/validate subobject xml
    unique_ptr<MotorXmlParser> motorXML = make_unique<MotorXmlParser>();
    unique_ptr<AnalogInputXmlParser> analogXML = make_unique<AnalogInputXmlParser>();
    unique_ptr<DigitalInputXmlParser> digitalXML = make_unique<DigitalInputXmlParser>();
    unique_ptr<ServoXmlParser> servoXML = make_unique<ServoXmlParser>();
    unique_ptr<SolenoidXmlParser> solenoidXML = make_unique<SolenoidXmlParser>();
    unique_ptr<CancoderXmlParser> cancoderXML = make_unique<CancoderXmlParser>();

    IDragonMotorControllerMap motors;
    ServoMap servos;
    DragonSolenoidMap solenoids;
    AnalogInputMap analogInputs;
    DigitalInputMap digitalInputs;
    shared_ptr<ctre::phoenix::sensors::CANCoder> canCoder = nullptr;

    for (xml_node child = mechanismNode.first_child(); child  && !hasError; child = child.next_sibling())
    {
        if ( strcmp( child.name(), "motor") == 0 )
        {
            auto motor = motorXML.get()->ParseXML(child);
            if ( motor.get() != nullptr )
            {
                motors[ motor.get()->GetType() ] =  motor ;
            }
        }
        else if ( strcmp( child.name(), "analogInput") == 0 )
        {
            auto analogIn = analogXML->ParseXML(child);
            if ( analogIn != nullptr )
            {
                analogInputs[analogIn->GetType()] = analogIn;
            }
        }
        else if ( strcmp( child.name(), "digitalInput") == 0 )
        {
            auto digitalIn = digitalXML->ParseXML(child);
            if ( digitalIn.get() != nullptr )
            {
                digitalInputs[digitalIn.get()->GetType()] = digitalIn;
            }
        }
        else if ( strcmp( child.name(), "servo") == 0 )
        {
            auto servo = servoXML->ParseXML(child);
            if ( servo != nullptr )
            {
                servos[servo->GetUsage()] = servo;
            }
        }
        else if ( strcmp( child.name(), "solenoid" ) == 0 )
        {
            auto sol = solenoidXML->ParseXML(child);
            if ( sol.get() != nullptr )
            {
                solenoids[sol.get()->GetType()] = sol;
            }
        }
        else if ( strcmp( child.name(), "canCoder" ) == 0)
        {
            canCoder = cancoderXML.get()->ParseXML(child);
        }
        else
        {
            string msg = "unknown child ";
            msg += child.name();
            Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, string("MechanismXmlParser"), string("unknown child"), msg );
        }
    }


    // create instance
    if ( !hasError )
    {
        MechanismFactory* factory =  MechanismFactory::GetMechanismFactory();
        factory->CreateIMechanism( type, 
                                   networkTableName,
                                   controlFileName,
                                   motors, 
                                   solenoids, 
                                   servos, 
                                   digitalInputs,
                                   analogInputs, 
                                   canCoder );
    }

}
