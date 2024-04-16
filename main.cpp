#include <iostream>
#include "Reconstruction/Reconstruction.h"
#include "Calibration/Calibration.h"
#include "Types/Types.h"
#include "Tools/Tools.h"
#include "Param/CameraParam.h"
#include "Param/Settings.h"

int main()
{
    std::cout << HIGHLIGHT "Kira OpenCV Stereo Tools\n" CLRST;
    // Load & init settings
    std::string settingFileName = "settings.json";
    std::ifstream check(settingFileName);
    if (check.good())
    {
        check.close();
    }
    else
    {
        check.close();
        ArduCV::SettingOperator::initSettings(ArduCV::setting,settingFileName);
    }
    ArduCV::SettingOperator::readSettings(ArduCV::setting, settingFileName);

    while (true)
    {
        std::cout << HIGHLIGHT R"delemeter(Options:
1. Stereo calibration.
2. StereoBM reconstruction.
3. StereoSGBM reconstruction.
4. Show calibration data.
5. Save and Quit
6. Quit
Enter the number: )delemeter" CLRST;
        int opts;
        std::cin >> opts;
        switch (opts)
        {
            case 1:
            {
                std::string userInput;
                cv::Size boardSize;
                float squareSize;
                std::cin.sync();
                std::getline(std::cin, userInput);
                std::cout << HIGHLIGHT "Enter chessboard corner size, enter the max corner size: \n" CLRST;
                std::cout << HIGHLIGHT "Note, opencv will detect the max size of the board.\n" CLRST;
                reEnter_Calib:
                std::cout << HIGHLIGHT "Height (6): " CLRST;
                std::getline(std::cin, userInput);
                if (userInput != "")
                {
                    try
                    {
                        boardSize.height = std::stoi(userInput);
                    }
                    catch (...)
                    {
                        std::cout << HIGHLIGHT "Invalid Input.\n" CLRST;
                        goto reEnter_Calib;
                    }
                }
                else
                {
                    boardSize.height = 6;
                }
                std::cin.sync();
                std::cout << HIGHLIGHT "Width (9): " CLRST;
                std::getline(std::cin, userInput);
                if (userInput != "")
                {
                    try
                    {
                        boardSize.width = std::stoi(userInput);
                    }
                    catch (...)
                    {
                        std::cout << HIGHLIGHT "Invalid Input.\n" CLRST;
                        goto reEnter_Calib;
                    }
                }
                else
                {
                    boardSize.width = 9;
                }
                std::cout << HIGHLIGHT "Square size (float) (28.5f): " CLRST;
                std::getline(std::cin, userInput);
                if (userInput != "")
                {
                    try
                    {
                        squareSize = std::stof(userInput);
                    }
                    catch (...)
                    {
                        std::cout << HIGHLIGHT "Invalid Input.\n" CLRST;
                        goto reEnter_Calib;
                    }
                }
                else
                {
                    squareSize = 28.5f;
                }
                ArduCV::InteractiveCameraCalibration acc(boardSize.height, boardSize.width, squareSize, static_cast<int>(ArduCV::setting["CameraDeviceNumber"].as_int64()), ArduCV::setting["UseBinaryThreshold"].as_bool());
                ArduCV::CameraParamStereo cameraParamStereo;
                acc.startCalibration(cameraParamStereo);
            }
                break;
            case 2:
            {
                std::cout << "Booting...\n";
                ArduCV::CameraParamStereo param = ArduCV::readCalibrateData("calibration_data.yaml").second;
                ArduCV::Reconstruction srs(param, static_cast<int>(ArduCV::setting["CameraDeviceNumber"].as_int64()));
                srs.InteractiveReconstruction();
            }
                break;
            case 4:
            {
                ArduCV::showCalibrateData(ArduCV::readCalibrateData().second);
            }
                break;
            case 5:
            {
                ArduCV::SettingOperator::saveSettings(ArduCV::setting, settingFileName);
                return 0;
            }
                break;
            case 6:
            {
                std::cout << HIGHLIGHT "Discard changes? (No)" CLRST;
                std::string input;
                std::cin.sync();
                std::getline(std::cin, input);
                std::getline(std::cin, input);
                if (input == "yes" || input == "Yes" || input == "Y" || input == "y")
                {
                    return 0;
                }
            }
                break;
            default:
            {
                std::cout << HIGHLIGHT "Invalid Input\n" CLRST;
            }
                break;
        }
    }
}
