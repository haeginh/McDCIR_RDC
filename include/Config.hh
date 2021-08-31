#ifndef INCLUDE_OCR_CONFIG_HH_
#define INCLUDE_OCR_CONFIG_HH_

#include <iostream>
#include <string>
#include <eigen3/Eigen/Dense>

class Config {
public:
    Config();
    void saveConfig(std::string fileN);
    bool loadConfig(std::string fileN);

    int getPortNum() const {
        return _port;
    }

    std::string getIpAdress() const {
        return _ipAdress;
    } 

    std::string getPhantomFile() const {
        return _phantom;
    } 

    std::string getPatientFile() const {
        return _patient;
    } 

    std::string getCArmFile() const {
        return _cArm;
    } 

    std::string getGlassFile() const {
        return _glass;
    } 

    Eigen::Vector3d getIsoCenter() const {
        return _isoCenter;
    } 

private:
    int _port;
    std::string _ipAdress, _phantom, _patient, _cArm, _glass;
    Eigen::Vector3d _isoCenter;
};

#endif /* INCLUDE_OCR_CONFIG_HH_ */
