
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
#include <map>
#include <memory>
#include <string>

// FRC includes

// Team 302 includes
#include <mechanisms/MechanismTypes.h>
#include <utils/Logger.h>

// Third Party Includes

using namespace std;

MechanismTypes* MechanismTypes::m_instance = nullptr;
MechanismTypes* MechanismTypes::GetInstance()
{
    if ( m_instance == nullptr )
    {
        m_instance = new MechanismTypes();
    }
    return m_instance;
}

MechanismTypes::MechanismTypes()
{
    m_typeMap["LEFT_INTAKE"]    = MECHANISM_TYPE::LEFT_INTAKE;
    m_typeMap["RIGHT_INTAKE"]   = MECHANISM_TYPE::RIGHT_INTAKE;
    m_typeMap["BALL_TRANSFER"]   = MECHANISM_TYPE::RIGHT_INTAKE;
    m_typeMap["INDEXER"]   = MECHANISM_TYPE::RIGHT_INTAKE;
    m_typeMap["LIFT"]   = MECHANISM_TYPE::RIGHT_INTAKE;
    m_typeMap["SHOOTER"]   = MECHANISM_TYPE::RIGHT_INTAKE;
    m_typeMap["CLIMBER"]   = MECHANISM_TYPE::RIGHT_INTAKE;
    m_typeMap["CAMERASERVO"]   = MECHANISM_TYPE::RIGHT_INTAKE;
}

MechanismTypes::~MechanismTypes()
{
    m_typeMap.clear();
}

MechanismTypes::MECHANISM_TYPE MechanismTypes::GetType
(
    string              typeString
)
{
    auto it = m_typeMap.find(typeString);
    if (it != m_typeMap.end())
    {
        return it->second;
    }
    Logger::GetLogger()->LogData(Logger::LOGGER_LEVEL::ERROR, string("MechanismTypes"), string("GetType - unknown mechanism type"), typeString);
    return MechanismTypes::MECHANISM_TYPE::UNKNOWN_MECHANISM;
}
