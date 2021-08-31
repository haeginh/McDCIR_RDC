#include "Config.hh"
#include <fstream>

Config::Config() : 
    _port(30303), _ipAdress("localhost"), _phantom("simple2"), _patient("patient3.obj"), _cArm("c-arm.ply"), _glass("glass.ply"),
    _isoCenter(0., 0., 113.5)
{
}

using namespace std;
void Config::saveConfig(std::string fileN)
{
    ofstream fs(fileN);
    fs << "portNumber " << _port << endl;
    fs << "IpAdress " << _ipAdress << endl;
    fs << "phantomFile " << _phantom << endl;
    fs << "patientFile " << _patient << endl;
    fs << "cArmFile " << _cArm << endl;
    fs << "glassFile " << _glass << endl;
    fs << "isoCenter " << _isoCenter.transpose() << endl;
    fs.close();
}

bool Config::loadConfig(std::string fileN)
{
    ifstream fs(fileN);
    if(!fs.is_open()) return false;

    string line;
    while(getline(fs, line))
    {
        if(line.empty()) continue;
        stringstream ss(line);
        string dump;
        ss>>dump;
        if(dump=="portNumber")  ss>>_port;
        else if(dump=="IpAdress") ss>> _ipAdress;
        else if(dump=="phantomFile") ss>>_phantom;
        else if(dump=="patientFile") ss>>_patient;
        else if(dump=="cArmFile") ss>>_cArm;
        else if(dump=="glassFile") ss>>_glass;
        else if(dump=="isoCenter")
        {
            double x, y, z;
            ss>>x>>y>>z;
            _isoCenter = Eigen::Vector3d(x,y,z);
        }
    }
    fs.close();
    return true;
}
