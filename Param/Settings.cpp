//
// Created by Kira on 24-4-16.
//

#include "Settings.h"

namespace ArduCV {
    Setting setting;

    bool SettingOperator::readSettings(Setting &_setting, const std::string& fileName) {
        try {
            std::ifstream file(fileName);
            std::string jsonRaw((std::istreambuf_iterator<char>(file)), (std::istreambuf_iterator<char>()));
            _setting = boost::json::parse(jsonRaw).as_object();
            file.close();
        }
        catch (std::exception &ex) {
            throw ex;
        }
        return true;
    }

    bool SettingOperator::saveSettings(Setting &_setting, const std::string& fileName) {
        try {
            std::ofstream file(fileName, std::ios::out);
            std::string str = boost::json::serialize(_setting);
            file << str;
            file.close();
        }
        catch (std::exception &ex) {
            throw ex;
        }
        return true;
    }

    bool SettingOperator::initSettings(Setting &_setting, const std::string& fileName = "settings.json") {
        _setting["CameraDeviceNumber"] = 0;
        _setting["CameraResolutionWidth"] = 3840;
        _setting["CameraResolutionHeight"] = 1080;
        _setting["FPS"] = 30;
        _setting["WinSizeWidth"] = 640;
        _setting["WinSizeHeight"] = 480;
        _setting["UseBinaryThreshold"] = false;
        _setting["StereoBM_NumDisparities"] = 16;
        _setting["StereoBM_BlockSize"] = 6;
        _setting["StereoBM_UniquenessRatio"] = 3;
        _setting["StereoBM_PreFilterCap"] = 16;
        _setting["Toggle_MSMF"] = true;
        _setting["Toggle_RT_StereoBM"] = true;
        _setting["Toggle_Partitional_Parallel"] = false;
        _setting["Partitional_Parallel_Cols"] = 3;
        _setting["Partitional_Parallel_Rows"] = 3;
        return saveSettings(_setting);
    }
}