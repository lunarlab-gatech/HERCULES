#include "airsim_settings_parser.h"
#include "sim_mode_from_settings.h"

AirSimSettingsParser::AirSimSettingsParser(const std::string& host_ip, uint16_t host_port)
    : host_ip_(host_ip)
    , host_port_(host_port)
{
    success_ = initializeSettings();
}

bool AirSimSettingsParser::success()
{
    return success_;
}

bool AirSimSettingsParser::getSettingsText(std::string& settings_text) const
{
    msr::airlib::RpcLibClientBase airsim_client(host_ip_, host_port_);
    airsim_client.confirmConnection();

    settings_text = airsim_client.getSettingsString();

    return !settings_text.empty();
}

std::string AirSimSettingsParser::getSimMode()
{
    const auto& settings_json = msr::airlib::Settings::loadJSonString(settings_text_);
    const auto sim_mode = settings_json.getString("SimMode", "");
    if (!sim_mode.empty()) {
        return sim_mode;
    }

    msr::airlib::RpcLibClientBase airsim_client(host_ip_, host_port_, 5.0f);
    const auto detected_mode = sim_mode_from_default_vehicle(airsim_client.listVehicles());
    std::cout << "SimMode omitted from simulator settings; detected " << detected_mode
              << " from the active default vehicle." << std::endl;
    return detected_mode;
}

// mimics void ASimHUD::initializeSettings()
bool AirSimSettingsParser::initializeSettings()
{
    if (getSettingsText(settings_text_)) {
        AirSimSettings::initializeSettings(settings_text_);

        AirSimSettings::singleton().load(std::bind(&AirSimSettingsParser::getSimMode, this));
        std::cout << "SimMode: " << AirSimSettings::singleton().simmode_name << std::endl;

        return true;
    }

    return false;
}
