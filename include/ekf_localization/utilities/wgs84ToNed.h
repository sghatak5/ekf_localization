#ifndef WGS84TONED_HPP
#define WGS84TONED_HPP

#include <eigen3/Eigen/Dense>

class WGS84toNED{
public:
    WGS84toNED(double refLat, double refLon, double refAlt);
    Eigen::Vector3d convertToNed(double lat, double lon, double alt);
private:
    double refLat_, refLon_, refAlt_;

    double deg2rad(double deg); 

    Eigen::Vector3d WGS84ToEcef(double lat, double lon, double alt); //Compute ECEF Coordinates
};

#endif