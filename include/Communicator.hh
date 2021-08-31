#ifndef Communicator_HH
#define Communicator_HH

#include "ServerSocket.hh"
#include "Viewer.hh"
#include <map>
using namespace std;

#define BE_ROWS 22

struct DataSet
{
    Affine3d glass_aff;
    RotationList posture;
    bool bodyIn;
    MatrixXd jointC;
};

class Communicator{
public:
    Communicator(string ip, int _port);
    ~Communicator();

    void SetInitPack(RotationList vQ, MatrixXi BE);
    void Initialization();
    void BodyCalibration(int sock_id, string newProfile, PhantomAnimator* phantom);
    void StartMainLoop();

    void CurrentValue(Affine3d &glass_aff, RotationList &posture, MatrixXd &jointC)
    {
        glass_aff = current.glass_aff;
        posture.resize(BE_ROWS);
        for(int i=0;i<BE_ROWS;i++) posture[i] = current.posture[i];
        jointC = current.jointC;
    }
    bool PostureAvailable(){return current.bodyIn;}

    vector<int> GetCamSock(){
        vector<int> sockets;
        for(auto iter:sock_opts) if(iter.second>0) sockets.push_back(iter.first);
        return sockets;
    }

    vector<string> GetLabels(){return labels;}
    vector<float> GetValues(){return values;}
    bool AlreadyStarted(){return mainLoop.joinable();}
    void Record(){record = !record; if(!ofs.is_open())ofs.open("record.txt");}
    bool IsRecording(){return record;}

private:
    string ip;
    int port;

    //socket comm.
    fd_set readfds;
    struct timeval sel_timeout;
    ServerSocket* server;
    map<int, ServerSocket *> client_sockets;
    map<int, int> sock_opts;
    map<int, Affine3d> sock_coord;

    int max_sd;

    array<double, 155> initPack;
    thread mainLoop;
    bool stopSignal;
    DataSet current;

    //ocr
    vector<string> labels;
    vector<float> values;

    //record
    bool record;
    ofstream ofs;

};

#endif