#pragma once

#include <memory>
#include <pugixml/pugixml.hpp>
#include <frc/Encoder.h>

class ThroughBoreEncoderXmlParser
{
    ThroughBoreEncoderXmlParser() = default;

    virtual ~ThroughBoreEncoderXmlParser() = default;
    std::shared_ptr<frc::Encoder> ParseXML
    (
        pugi::xml_node throughBoreEncoderNode
    );

};