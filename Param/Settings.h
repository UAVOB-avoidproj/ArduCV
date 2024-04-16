//
// Created by Kira on 24-4-16.
//

#ifndef ARDUCV_SETTINGS_H
#define ARDUCV_SETTINGS_H

#include <boost/json.hpp>
#include <fstream>
#include "../Types/Types.h"

namespace ArduCV {

    extern Setting setting;

    class SettingOperator {
    public:
        static bool readSettings(boost::json::object &_setting, const std::string& fileName = "settings.json");

        static bool saveSettings(boost::json::object &_setting, const std::string& fileName = "settings.json");

        static bool initSettings(Setting &_setting, const std::string& fileName);
    };

}


#endif //ARDUCV_SETTINGS_H
