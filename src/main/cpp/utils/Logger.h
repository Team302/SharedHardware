
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
/// Logger.h
//========================================================================================================
///
/// File Description:
///     This logs error messages, and can also be used to log non-error events.
///
//========================================================================================================

#pragma once

// C++ Includes
#include <string>
#include <set>

// FRC includes
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <frc/SmartDashboard/SendableChooser.h>

// Team 302 includes


// Third Party Includes



class Logger
{
    public:

        /// @enum LOGGER_OPTION
        /// @brief Define where the items being logged should be sent
        enum LOGGER_OPTION
        {
            CONSOLE,        ///< write to the RoboRio Console
            DASHBOARD,      ///< write to the SmartDashboard
            EAT_IT          ///< don't write anything (useful at comps where we want to minimize network traffic)
        };

        /// @enum LOGGER_LEVEL
        /// @brief Define what level the message is as well as this can be used to write only the messages
        /// @brief of a certain level or worse.
        /// @brief The enum is ordered from worse to better and corresponds to the driver's station levels.        
        enum LOGGER_LEVEL
        {
            ERROR_ONCE,     ///< this is catastrophic that we only want to see once
            ERROR,          ///< this is catastrophic
            WARNING_ONCE,   ///< this is a medium level error we only want to see once
            WARNING,        ///< this is a medium level error
            PRINT_ONCE,     ///< this is an information/debug message we only want to see once
            PRINT           ///< this is an information/debug message
        };

        /// @brief Find or create the singleton logger
        /// @returns Logger* pointer to the logger
        static Logger* GetLogger();


        /// @brief log a message
        /// @param [in] LOGGER_LEVEL: message level
        /// @param [in] std::string: network table name or classname to group messages.  If logging option is DASHBOARD, this will be the network table name
        /// @param [in] std::string: message identifier: within a grouping multiple messages may be displayed this is the prefix/look up key
        /// @param [in] std::string: message - text of the message       
        void LogData
        (
            LOGGER_LEVEL            level,
            const std::string&      group,
            const std::string&      identifier,     
            const std::string&      message                 
        );

        /// @brief log a message
        /// @param [in] LOGGER_LEVEL: message level
        /// @param [in] std::string: network table name or classname to group messages.  If logging option is DASHBOARD, this will be the network table name
        /// @param [in] std::string: message identifier: within a grouping multiple messages may be displayed this is the prefix/look up key
        /// @param [in] double: value to display       
        void LogData
        (
            LOGGER_LEVEL            level,
            const std::string&      group,
            const std::string&      identifier,     
            double                  value                 
        );


        /// @brief log a message
        /// @param [in] LOGGER_LEVEL: message level
        /// @param [in] std::string: network table name or classname to group messages.  If logging option is DASHBOARD, this will be the network table name
        /// @param [in] std::string: message identifier: within a grouping multiple messages may be displayed this is the prefix/look up key
        /// @param [in] bool: value to display       
        void LogData
        (
            LOGGER_LEVEL            level,   
            const std::string&      group,
            const std::string&      identifier,     
            bool                    value                 
        );

        /// @brief log a message
        /// @param [in] LOGGER_LEVEL: message level
        /// @param [in] std::string: network table name or classname to group messages.  If logging option is DASHBOARD, this will be the network table name
        /// @param [in] std::string: message identifier: within a grouping multiple messages may be displayed this is the prefix/look up key
        /// @param [in] int: value to display       
        void LogData
        (
            LOGGER_LEVEL            level,   
            const std::string&      group,
            const std::string&      identifier,     
            int                     value                 
        );
        /// @brief Display logging options on dashboard
        void PutLoggingSelectionsOnDashboard();

        /// @brief Read logging option from dashboard, but not every 20ms
        void PeriodicLog();


    protected:


    private:
        /// @brief Determines whether a message should be displayed or not.   For instance if EAT_IT is the logging option, this will return false or if the level is xxx_ONCE, it may return false if the message was already logged.
        /// @param [in] LOGGER_LEVEL: message level
        /// @param [in] std::string: network table name or classname to group messages.  If logging option is DASHBOARD, this will be the network table name
        /// @param [in] std::string: message identifier: within a grouping multiple messages may be displayed this is the prefix/look up key
        /// @param [in] std::string: message/value
        /// @returns bool: true - display the message, false - don't display the message
        bool ShouldDisplayIt
        (
            LOGGER_LEVEL            level,
            const std::string&      group,
            const std::string&      identifier,     
            const std::string&      message  
        );
        
        /// @brief set the option for where the logging messages should be displayed
        /// @param [in] LOGGER_OPTION:  logging option for where to log messages
        void SetLoggingOption
        (
            LOGGER_OPTION option    // <I> - Logging option
        );

        /// @brief set the level for messages that will be displayed
        /// @param [in] LOGGER_LEVEL:  logging level for which messages to display
        void SetLoggingLevel
        (
            LOGGER_LEVEL level    // <I> - Logging level
        );


        Logger();
        ~Logger() = default;

        LOGGER_OPTION                           m_option;               // indicates where the message should go
        LOGGER_LEVEL                            m_level;                // the level at which a message is important enough to send
        std::set<std::string>                   m_alreadyDisplayed;
        static Logger*                          m_instance;
        int                                     m_cyclingCounter;       // count 20ms loops
        frc::SendableChooser<LOGGER_OPTION>     m_optionChooser;
        frc::SendableChooser<LOGGER_LEVEL>      m_levelChooser;
};


