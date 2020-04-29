#include "driveu_vehicle_data.h"

bool VehicleData::parseImageDict(const Json::Value &image_dict)
{
    m_latitude_ = image_dict["latitude"].asDouble();
    m_longitude_ = image_dict["longitude"].asDouble();
    m_velocity_ = image_dict["velocity"].asDouble();
    m_yaw_rate_ = image_dict["yaw_rate"].asDouble();

    return true;
}

double VehicleData::getVelocity() const
{
    return m_velocity_;
}

double VehicleData::getYawRate() const
{
    return m_yaw_rate_;
}

double VehicleData::getLongitude() const
{
    return m_longitude_;
}
double VehicleData::getLatitude() const
{
    return m_latitude_;
}