#ifndef Communicator_HH
#define Communicator_HH

#include "ServerSocket.hh"
#include "PhantomAnimator.hh"
#include <map>
#include <thread>
#include <chrono>

#define BE_ROWS 22
#define SERVER_IP "192.168.0.100"
using namespace std;
typedef tuple<string, int, Eigen::Affine3d> WORKER;
struct Body
{
    clock_t time;
    RotationList posture = RotationList(18);
    MatrixXd jointC = MatrixXd::Zero(24, 3);
};

struct DataSet
{
    Affine3d glass_aff = Affine3d::Identity();
    map<int, Body> bodyMap;
    // RotationList posture = RotationList(18);
    // bool bodyIn;
    // MatrixXd jointC = MatrixXd::Zero(24, 3);
};

class Communicator
{
public:
    // singleton
    static Communicator &Instance();
    ~Communicator();
    //////////////////////////////////////////////////////////////////////////////
    // *Server*                                                                 //
    // -StartServer: opens UDP server sockets and start listening loop.         //
    //               while(listening){listen; if(ready to stop)send stop msg;}  //
    // -CloseServer: for "stop listening" sign, send stop msg to workers and    //
    //               closes server socket.                                      //
    //////////////////////////////////////////////////////////////////////////////

    bool StartServer(int port);
    void CloseServer(){
        isListening = false;
        listening.join();
        int code = close(server_fd);
        cout<<"Server: socket closed (code:"<<code<<")"<<endl;
    }

    //////////////////////////////////////////////////////////////////////////////
    // *Worker*                                                                 //
    // -workerData  : contains worker data (workerID: [IP, option, cam T])      //
    // -StartWorker : starts tracker program by SSH                             //
    // -StartWorkers: starts all workers registered in "workerData"             //
    // -GetWorekrNum: gets the number of registered workers                     //
    // -GetAWorker  : gets a worker data                                        //
    // -DeleteWorker: deletes a worker data                                     //
    // -GetDelay    : gets a elapsed time from last response from the worker    // --> unit needs to be checked
    //////////////////////////////////////////////////////////////////////////////
    map<int, WORKER> workerData;
    void StartWorker(string ip, int opt, int port = 22)
    {
        workerData[nextWorkerID] = WORKER(ip, opt, Affine3d::Identity());
        system(("ssh " +ip+" \"2_tracker "+SERVER_IP+" "+to_string(serverPORT)+" " +to_string(nextWorkerID++) + " " + to_string(opt) + "\" &" ).c_str());
    // sock_opts[nextWorkerID++] = opt;
    }
    void StartWorkers()
    {
        for(auto iter:workerData)
        {
            system(("ssh " +get<0>(iter.second)+" \"2_tracker "+SERVER_IP+" "+to_string(serverPORT)+" " +to_string(iter.first)  + " " + to_string(get<1>(iter.second)) + "\" &" ).c_str());
        }
    }
    int GetWorkerNum() { return workerData.size(); }
    WORKER GetAWorker(int i) { return workerData[i]; }
    void DeleteWorker(int i)
    {
        workerData.erase(i);
    }
    float GetDelay(int i)
    {
        if(lastStamp.find(i)==lastStamp.end()) return __DBL_MAX__;
        return float(clock() - lastStamp[i])/CLOCKS_PER_SEC;
    }

    void InitializeDataSet();
    void SetInitPack(RotationList vQ, MatrixXi BE);
    // void Initialization();
    void BodyCalibration(int sock_id, string newProfile, PhantomAnimator *phantom);
    void StartMainLoop();

    // void CurrentValue(Affine3d &glass_aff, RotationList &posture, MatrixXd &jointC)
    // {
    //     glass_aff = current.glass_aff;
    //     posture.resize(BE_ROWS);
    //     for (int i = 0; i < BE_ROWS; i++)
    //         posture[i] = current.posture[i];
    //     jointC = current.jointC;
    // }
    // bool PostureAvailable() { return current.bodyIn; }

    // vector<int> GetCamSock()
    // {
    //     vector<int> sockets;
    //     for (auto iter : sock_opts)
    //         if (iter.second > 0)
    //             sockets.push_back(iter.first);
    //     return sockets;
    // }

    vector<string> GetLabels() { return labels; }
    vector<float> GetValues() { return values; }
    bool AlreadyStarted() { return mainLoop.joinable(); }
    void Record()
    {
        record = !record;
        if (!ofs.is_open())
            ofs.open("record.txt");
    }
    bool IsRecording() { return record; }

    DataSet current;

private:
    Communicator():nextWorkerID(0)
    {}
    
    // udp
    int server_fd;
    int serverPORT;
    int nextWorkerID;
    thread listening;
    bool isListening;
    map<int, clock_t> lastStamp;


    array<double, 155> initPack;
    thread mainLoop;
    bool stopSignal;

    // ocr
    vector<string> labels;
    vector<float> values;

    // record
    bool record;
    ofstream ofs;
};

#endif