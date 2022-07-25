
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

//========================================================================================================
/// RobotXmlParser.cpp
//========================================================================================================
///
/// File Description:
///     Top-level XML parsing file for the robot.  This definition will construct the motor motorControllers,
///     sensors, chassis and mechanisms from an XML file.  This parsing leverages the 3rd party Open Source
///     Pugixml library (https://pugixml.org/).
///
///     This parsing code will call the classes/methods to parse the lower-level objects.  When the parsing
///     has been completed, the robot hardware will be defined.
///
///     The robot definition XML file is:  /home/lvuser/config/robot.xml
///
//========================================================================================================

// C++ Includes
#include <memory>

// FRC includes
#include <frc/Filesystem.h>

// Team 302 includes
#include <hw/DragonPigeon.h>
#include <utils/Logger.h>
#include <hw/xml/CameraXmlParser.h>
#include <chassis/ChassisXmlParser.h>
#include <hw/xml/ledXmlParser.h>
#include <hw/xml/LimelightXmlParser.h>
#include <basemechanisms/MechanismXmlParser.h>
#include <hw/xml/PDPXmlParser.h>
#include <hw/xml/PigeonXmlParser.h>
#include <RobotXmlParser.h>

// Third Party Includes
#include <pugixml/pugixml.hpp>

using namespace frc;
using namespace pugi;
using namespace std;


//-----------------------------------------------------------------------
// Method:      ParseXML
// Description: Parse a robot.xml file
// Returns:     void
//-----------------------------------------------------------------------
void RobotXmlParser::ParseXML()
{
    // set the file to parse
	auto deployDir = frc::filesystem::GetDeployDirectory();
    string filename = deployDir + string("/robot.xml");

    try
    {
       // load the xml file into memory (parse it)
        xml_document doc;
        xml_parse_result result = doc.load_file(filename.c_str());

        // if it is good
        if (result)
        {
            unique_ptr<CameraXmlParser> cameraXML = make_unique<CameraXmlParser>();
            unique_ptr<ChassisXmlParser> chassisXML = make_unique<ChassisXmlParser>();
            unique_ptr<MechanismXmlParser> mechanismXML = make_unique<MechanismXmlParser>();
            unique_ptr<PigeonXmlParser> pigeonXML = make_unique<PigeonXmlParser>();
            unique_ptr<LedXmlParser> ledXML = make_unique<LedXmlParser>();
            unique_ptr<LimelightXmlParser> limelightXML = make_unique<LimelightXmlParser>();
            unique_ptr<PDPXmlParser> pdpXML = make_unique<PDPXmlParser>();

            // get the root node <robot>
            xml_node parent = doc.root();
            for (xml_node node = parent.first_child(); node; node = node.next_sibling())
            {
                // loop through the direct children of <robot> and call the appropriate parser
                for (xml_node child = node.first_child(); child; child = child.next_sibling())
                {
                    if (strcmp(child.name(), "chassis") == 0)
                    {
                        chassisXML.get()->ParseXML(child);
                    }
                    else if (strcmp(child.name(), "mechanism") == 0)
                    {
                        mechanismXML.get()->ParseXML(child);
                    }
                    else if (strcmp(child.name(), "camera") == 0)
                    {
                        cameraXML.get()->ParseXML(child);
                    }
                    else if (strcmp(child.name(), "pdp") == 0)
                    {
                        pdpXML.get()->ParseXML(child);
                    }
                    else if ( strcmp(child.name(), "pigeon") == 0 )
                    {
                        pigeonXML.get()->ParseXML( child);
                    }
                    else if ( strcmp(child.name(), "limelight") == 0 )
                    {
                        limelightXML.get()->ParseXML( child);
                    }
                    else if ( strcmp(child.name(), "led") == 0 )
                    {
                        ledXML.get()->ParseXML(child);
                    }
                    else
                    {
                        string msg = "unknown child ";
                        msg += child.name();
                        Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, string("RobotXmlParser"), string("ParseXML"), msg );
                    }
                }
            }
        }
        else
        {
            string msg = "XML [";
            msg += filename;
            msg += "] parsed with errors, attr value: [";
            msg += doc.child( "prototype" ).attribute( "attr" ).value();
            msg += "]";
            Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, string("RobotXmlParser"), string("ParseXML (1) "), msg );

            msg = "Error description: ";
            msg += result.description();
            Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, string("RobotXmlParser"), string("ParseXML (2) "), msg );

            msg = "Error offset: ";
            msg += result.offset;
            msg += " error at ...";
            msg += filename;
            msg += result.offset;
            Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, string("RobotXmlParser"), string("ParseXML (3) "), msg );
        }
    }
    catch(const std::exception& e)
    {
        Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR_ONCE, string("RobotXmlParser"), string("ParseXML"), string("Error thrown while parsing robot.xml") );
    }
}
