#ifndef INCLUDE_OCR_CONFIG_HH_
#define INCLUDE_OCR_CONFIG_HH_

#include <iostream>
#include <string>

class Config {
public:
    Config();
    void SaveConfig(std::string fileN);
    bool LoadConfig(std::string fileN);
    bool CheckConfig();

    int GetPortNum() const {
        return portNum;
    }

    std::string GetIpAdress() const {
        return ipAddress;
    } 

    std::string GetCArmFile() const {
        return dataDir+cArm;
    } 

    std::string GetCArmDetFile() const {
        return dataDir+cArmDet;
    } 

    std::string GetTableFile() const {
        return dataDir+table;
    } 

    std::string GetGlassFile() const {
        return dataDir+glass;
    } 

    std::string GetCurtainFile() const {
        return dataDir+curtain;
    } 

    std::string GetPatientFile() const {
        return dataDir+patient;
    } 

    std::string GetBeamFile() const {
        return dataDir+beam;
    } 

    std::string GetSphereFile() const {
        return dataDir+sphere;
    } 

    std::string GetProfileFile() const {
        return dataDir+profile;
    } 

    std::string GetPhantomDir() const {
        return phantomDir;
    } 

private:
    bool CheckIP(const std::string& ip_address);
    bool CheckPort(int port);
    int portNum;
    std::string ipAddress, dataDir, phantomDir, cArm, cArmDet, table, glass, 
    curtain, patient, beam, sphere, profile;
};

#endif /* INCLUDE_OCR_CONFIG_HH_ */
