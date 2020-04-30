#ifndef DRIVEU_DATASET_DRIVEU_VEHICLE_DATA_H_
#define DRIVEU_DATASET_DRIVEU_VEHICLE_DATA_H_

#include <jsoncpp/json/json.h>

class VehicleData
{
private:
    /// velocity in meters/second
    double m_velocity_;
    /// yaw_rate in rad/s
    double m_yaw_rate_;
    /// GPS longitude
    double m_longitude_;
    /// GPS latitude
    double m_latitude_;

public:
    /**
         * @brief           Constructor
         * @param latitude  GPS latitude
         * @param longitude GPS longitude
         * @param velocity  velocity in meters/second
         * @param yaw_rate  yaw rate in rad/s
         */
    VehicleData() : m_velocity_(0.), m_yaw_rate_(0.), m_longitude_(0.), m_latitude_(0.){};

    /**
         * @brief               Parses image dict from label file
         * @param image_dict    Dictionary of one single frame from label json
         */
    bool parseImageDict(const Json::Value &image_dict);
    /**
         * @brief   return velocity from current frame
         * @return  velocity in meters/second
         */
    double getVelocity() const;
    /**
         * @brief   return yaw rate from current frame
         * @return  yaw rate in rad/s
         */
    double getYawRate() const;
    /**
         * @brief   return longitude from current frame
         * @return  GPS longitude
         */
    double getLongitude() const;
    /**
         * @brief   return latitude from current frame
         * @return GPS latitude
         */
    double getLatitude() const;
};

#endif // DRIVEU_DATASET_DRIVEU_VEHICLE_DATA_H_