#pragma once

#include <string>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cassert>

namespace image_processing
{
class Config
{
public:
    struct AppSettings
    {
        std::string fillMode = "fast";
        std::string blurType = "bilateral";
        int extrapolate = 1;
        int resize = 0;
        float maxDepth = 10;
        float minDepth = 0.1;
    };

public:
    cv::FileStorage SettingsFile;
    AppSettings app;


public:
    Config(cv::FileStorage &fsSettings):SettingsFile(fsSettings)
    {
        AppSettingsInit();
    };

    Config(){};

    void AppSettingsInit(){
        app.fillMode = static_cast<std::string>(SettingsFile["FillType"]);
        app.blurType = static_cast<std::string>(SettingsFile["BlurType"]);
        app.extrapolate = SettingsFile["Extrapolate"];
        app.resize = SettingsFile["Resize"];
        app.maxDepth = SettingsFile["maxDepth"];
        app.minDepth = SettingsFile["minDepth"];
    };
};

}

