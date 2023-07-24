#include "Config.hh"
#include <fstream>
#include <sstream>
#include <vector>
#include <boost/asio.hpp>
#include <boost/filesystem.hpp>
#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h> 
#include <string.h> 
#include <arpa/inet.h>
#include <igl/readPLY.h>

Config::Config() : 
    portNum(30303), ipAddress("192.168.0.100"), dataDir("/data/VIR_GUI_DATA"), cArm("c-arm.ply"), 
    cArmDet("FPD.ply"), table("table.ply"), glass("leadGlass.ply"), curtain("pbShield.ply"), 
    patient("patient.ply"), beam("beam.ply"), sphere("sphere.ply"), profile("profile.txt")
{}

using namespace std;
void Config::SaveConfig(std::string fileN)
{
    ofstream fs(fileN);
    fs << "portNumber " << portNum << endl;
    fs << "IpAddress " << ipAddress << endl;
    fs << "dataDir " << dataDir << endl;
    fs << "phantomDir " << phantomDir << endl;
    fs << "cArm " << cArm << endl;
    fs << "cArmDet " << cArmDet << endl;
    fs << "table " << table << endl;
    fs << "glass " << glass << endl;
    fs << "curtain " << curtain << endl;
    fs << "patient " << patient << endl;
    fs << "beam " << beam << endl;
    fs << "sphere " << sphere << endl;
    fs << "profile " << profile << endl;
    fs.close();
}

bool Config::LoadConfig(std::string fileN)
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
        if(dump=="portNumber")  ss>>portNum;
        else if(dump=="IpAddress") ss>>ipAddress;
        else if(dump=="dataDir") ss>>dataDir;
        else if(dump=="phantomDir") ss>>phantomDir;
        else if(dump=="cArm") ss>>cArm;
        else if(dump=="cArmDet") ss>>cArmDet;
        else if(dump=="table") ss>>table;
        else if(dump=="glass") ss>>glass;
        else if(dump=="curtain") ss>>curtain;
        else if(dump=="patient") ss>>patient;
        else if(dump=="beam") ss>>beam;
        else if(dump=="sphere") ss>>sphere;
        else if(dump=="profile") ss>>profile;
    }
    fs.close();
    return true;
}

bool Config::CheckConfig()
{
    bool check(true);
    cout<<"----------Network Check----------"<<endl;
    cout<<"check IP ("+ipAddress+")...";
    if(CheckIP(ipAddress)) cout<<"GOOD"<<endl;
    else{
        cout<<"BAD"<<endl;
        check = false;
    }
    cout<<"check PORT ("<<portNum<<")...";
    if(CheckPort(portNum)) cout<<"GOOD (open)"<<endl;
    else{
        cout<<"BAD (closed)"<<endl;
        check = false;
    }
    cout<<"-----------File Check------------"<<endl;
    vector<string> requiredPLY = {cArm, cArmDet, patient, glass};
    vector<string> optionalPLY = {curtain, sphere, beam, table};
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    for(string file:requiredPLY)
    {
        string fileDir = dataDir+"/"+file;
        if(igl::readPLY(fileDir, V, F)) cout<<fileDir<<"...O"<<endl;
        else {cout<<fileDir<<"...X"<<endl; check = false;}
    }
    for(string file:optionalPLY)
    {
        string fileDir = dataDir+"/"+file;
        if(igl::readPLY(fileDir, V, F)) cout<<fileDir<<"...O"<<endl;
        else cout<<fileDir<<"...X"<<endl;
    }
    vector<string> files = {profile};
    for(string file:files)
    {
        string fileDir = dataDir+"/"+file;
        if(boost::filesystem::exists(fileDir)) cout<<fileDir<<"...O"<<endl;
        else cout<<fileDir<<"...X"<<endl;
    }
    cout<<"---------------------------------"<<endl;
    V.resize(0,0); F.resize(0,0);
    return check;
}

bool Config::CheckIP(const std::string& ip_address)
{
    struct ifaddrs * ifAddrStruct=NULL;
    struct ifaddrs * ifa=NULL;
    void * tmpAddrPtr=NULL;
    getifaddrs(&ifAddrStruct);

    for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next) {
        if (!ifa->ifa_addr) {
            continue;
        }
        if (ifa->ifa_addr->sa_family == AF_INET) { // check it is IP4
            // is a valid IP4 Address
            tmpAddrPtr=&((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
            char addressBuffer[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);
            if(string(addressBuffer)==ip_address) return true;
        } 
    }
    if (ifAddrStruct!=NULL) freeifaddrs(ifAddrStruct);
    return false;
}

bool Config::CheckPort(int port)
{
    boost::asio::io_service io_service;
    boost::asio::ip::tcp::acceptor acceptor(io_service);
    boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::tcp::v4(), port);
    boost::system::error_code error;

    acceptor.open(endpoint.protocol());
    acceptor.set_option(boost::asio::ip::tcp::acceptor::reuse_address(true));
    acceptor.bind(endpoint);
    acceptor.listen();

    return !error;
}

