#include "ekf_localization/utilities/wgs84ToNed.h"
#include <cmath>
using namespace std;

WGS84toNED::WGS84toNED(double refLat, double refLon, double refAlt)
: refLat_(refLat), refLon_(refLon), refAlt_(refAlt){}

Eigen::Vector3d WGS84toNED::convertToNed(double lat, double lon, double alt){
    //Converting ref and curr coordinate to ECEF
    Eigen::Vector3d currEcef = this->WGS84ToEcef(lat, lon, alt);
    Eigen::Vector3d refEcef = this->WGS84ToEcef(this->refLat_, this->refLon_, this->refAlt_);

    Eigen::Vector3d deltaECEF = currEcef - refEcef;

    double latRefRad = this->deg2rad(this->refLat_);
    double lonRefRad = this->deg2rad(this->refLon_);

    Eigen::Matrix3d rotationMatrix;
    rotationMatrix << -sin(latRefRad) * cos(lonRefRad), -sin(lonRefRad) , cos(latRefRad) * cos(lonRefRad),
                        -sin(latRefRad) * sin(lonRefRad), cos(lonRefRad), cos(latRefRad) * sin(lonRefRad),
                        cos(latRefRad), 0, sin(latRefRad);
    
    Eigen::Vector3d NED = rotationMatrix * deltaECEF;

    return NED;
}

double WGS84toNED::deg2rad(double deg){
    return deg * M_PI / 180;
}

Eigen::Vector3d WGS84toNED::WGS84ToEcef(double lat, double lon, double alt){
    const double a = 6378137.0; //WGS-84 Earth semimajor axis (m)
    const double e = 8.1819190842622e-2; //WGS-84 Earth eccentricity

    double latRad = this->deg2rad(lat);
    double lonRad = this->deg2rad(lon);
    double N = a / sqrt(1 - e * e * sin(latRad) * sin(latRad));

    double x = (N + alt) * cos(latRad) * cos(lonRad);
    double y = (N + alt) * cos(latRad) * sin(lonRad);
    double z = (N * (1 - e * e) + alt) * sin(latRad);

    return Eigen::Vector3d(x, y, z);
}