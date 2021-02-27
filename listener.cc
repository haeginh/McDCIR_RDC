#include "ServerSocket.h"
#include "SocketException.h"
#include <string>
#include <vector>
#include <array>
#include <utility>
#include "G4Timer.hh"
#include <Eigen/Dense>
#include <thread>
#include <mutex>

using namespace std;

class RemoteCalProcess{
public:
    RemoteCalProcess(ServerSocket* rc_sock, string data)
        :sock(rc_sock), calibChk(false)
    {
        GetInitData(data);
    }
    void GetInitData(string data){
        G4Timer timer; timer.Start();
        string dump; int vNum, tNum, fNum;
        stringstream ss(data); ss>>dump>>vNum>>tNum>>fNum;
        cout<<"RC_proc: Get data for "<<vNum<<" vertices..."<<flush;
        for(int i=0;i<vNum;i++){
            array<double, 3> p;
            array<double, 22> w;
            array<double, 24> wj;
            sock->RecvDoubleBuffer(p.data(),3);
            V.push_back(p);
            sock->RecvDoubleBuffer(w.data(),22);
            W.push_back(w);
            sock->RecvDoubleBuffer(wj.data(),24);
            Wj.push_back(wj);
        } timer.Stop(); double vTime = timer.GetRealElapsed();
        cout<<vTime<<endl; timer.Start();

        cout<<"RC_proc: Get data for "<<tNum<<" tetrahedrons..."<<flush;
        for(int i=0;i<tNum;i++){
            array<int, 4> t;
            sock->RecvIntBuffer(t.data(),4);
            T.push_back(t);
        }timer.Stop(); double tTime = timer.GetRealElapsed();
        cout<<tTime<<endl; timer.Start();

        cout<<"RC_proc: Get data for "<<fNum<<" faces..."<<flush;
        for(int i=0;i<fNum;i++){
            array<int, 3> f;
            sock->RecvIntBuffer(f.data(),3);
            F.push_back(f);
        }timer.Stop(); double fTime = timer.GetRealElapsed();
        cout<<fTime<<endl; timer.Start();
        cout<<"total: "<<vTime+fTime+tTime<<endl;
    }
    void RecvCalibData(){
        sock->RecvDoubleBuffer(calib.data(),72);
        cout<<"RC_proc: received calib data"<<endl;
        calibChk = true;
    }
    bool GetInitData(vector<array<double, 3>>& _V,
                     vector<array<int, 3>>& _F,
                     vector<array<int, 4>>& _T,
                     vector<array<double, 22>>& _W,
                     vector<array<double, 24>>& _Wj){
        if(!V.size()) return false;
        _V = V; _F = F; _T = T; _W = W; _Wj = Wj;
        return true;
    }
    bool GetCalibData(array<double, 72>& _calib){
        if(!calibChk) return false;
        _calib = calib;
        return true;
    }
private:
    ServerSocket* sock;
    vector<array<double, 3>> V;
    vector<array<int, 3>> F;
    vector<array<int, 4>> T;
    vector<array<double, 22>> W;
    vector<array<double, 24>> Wj;
    array<double, 72> calib;
    bool calibChk;
    mutex m;
};

class CalculatorProcess{
public:
    CalculatorProcess(ServerSocket* cal_sock, RemoteCalProcess* _rcProc, int _id)
        :sock(cal_sock), rcProc(_rcProc), id(_id){}
    void SendInitData(){
        vector<array<double, 3>> V;
        vector<array<int, 3>> F;
        vector<array<int, 4>> T;
        vector<array<double, 22>> W;
        vector<array<double, 24>> Wj;
        while(true){
            if(rcProc->GetInitData(V, F, T, W, Wj)) break; usleep(10);
        }

        (*sock) << to_string(id);
        (*sock) << to_string(V.size())+" "+to_string(T.size())+" "+to_string(F.size()) ;
        for(size_t i=0;i<V.size();i++){
            sock->SendDoubleBuffer(V[i].data(),3,1);
            sock->SendDoubleBuffer(W[i].data(),22,1);
            sock->SendDoubleBuffer(Wj[i].data(),24,1);
        }
        for(size_t i=0;i<T.size();i++)
            sock->SendIntBuffer(T[i].data(),4,1);
        for(size_t i=0;i<F.size();i++)
            sock->SendIntBuffer(F[i].data(),3,1);
        cout<<"Cal #"<<id<<": Sent init data"<<endl;
     }
    void SendCalibData(){
        array<double, 72> calib;
        while(true){
            if(rcProc->GetCalibData(calib)) break; usleep(10);
        }
        sock->SendDoubleBuffer(calib.data(),72);
        cout<<"Cal #"<<id<<": Sent calib data"<<endl;
     }
    int GetCalID() {return id;}

private:
    ServerSocket* sock;
    RemoteCalProcess* rcProc;
    int id;
};

void RemoteCalFunc(RemoteCalProcess* rcProc){
    rcProc->RecvCalibData();
    while(1){

    }
}

void CalculatorFunc(CalculatorProcess* calProc){
    calProc->SendInitData();
    calProc->SendCalibData();
}

int main ( int argc, char** argv )
{
    //init data

//    int frameNo(0);
//    vector<double> timeVec;
//    vector<array<double, 88>> quatVec;
//    vector<array<double, 66>> transVec;

    //set the port# from argument
    int port(30303);
    if(argc==2) port = atoi(argv[1]);

    //threads
    thread* remoteCal_th;
    vector<thread*> calculator_th;

    //processes
    RemoteCalProcess* rcProc;
    vector<CalculatorProcess*> calProcs;
    int calID(0);

    try
    {
        // Create the socket
        ServerSocket server( port );
        cout << "Listening....\n";

        //1. Initialization(mesh and weights): process a single connection
        ///need to recieve "init data" from RemoteCal
        ///ignore calculator("init")

        try
        {
            while ( true )
            {
                ServerSocket* new_sock = new ServerSocket;
                server.accept ( *new_sock );
                std::string data("");
                (*new_sock) >> data;
                if(data.substr(0,4)=="init"){
                    rcProc = new RemoteCalProcess(new_sock, data);
                    remoteCal_th = new thread(RemoteCalFunc, rcProc);
                    cout<<"init. data is ready. Listening..."<<endl;
                    break;
                }
                else if(data.substr(0,4)=="Idle"){
                    (*new_sock) << "wait";
                    delete new_sock;
                }
            }

            while ( true )
            {
                ServerSocket* new_sock = new ServerSocket;
                server.accept ( *new_sock );
                std::string data("");
                (*new_sock) >> data;
                if(data.substr(0,4)=="Idle"){
                    (*new_sock)<<to_string(calID);
                    calProcs.push_back(new CalculatorProcess(new_sock, rcProc, calID));
                    calculator_th.push_back(new thread(CalculatorFunc, calProcs.back()));
                    cout<<"Calculator #"<<calID++<<" was activated!"<<endl;
                }
            }
        }
        catch ( SocketException& ) {}

        //
//        while ( true )
//        {
//            ServerSocket new_sock;
//            server.accept ( new_sock );
//            try
//            {
//                std::string data;
//                new_sock >> data;
//                if(data=="quaternion"){
//                    double time;
//                    array<double, 88> quatArr;
//                    array<double, 66> transArr;
//                    new_sock.RecvDoubleBuffer(&time, 1);
//                    new_sock.RecvDoubleBuffer(quatArr.data(), 88);
//                    new_sock.RecvDoubleBuffer(transArr.data(), 66);
////                    cout<<quatArr.at(0)<<" "<<quatArr.at(1)<<" "<<quatArr.at(2)<<" "<<quatArr.at(3)<<endl;
////                    cout<<transArr.at(0)<<" "<<transArr.at(2)<<" "<<transArr.at(3)<<endl;
//                    timeVec.push_back(time);
//                    quatVec.push_back(quatArr);
//                    cout<<"received frame #"<<frameNo++<<" (stacked: "<<timeVec.size()<<")"<<endl;
//                }
//                else if(data=="Idle"){
//                    if(!timeVec.size()) {new_sock.SendIntBuffer(&errCode, 1); continue;}

//                    new_sock.SendDoubleBuffer(&timeVec.back(), 1);
//                    new_sock.SendDoubleBuffer(quatVec.back().data(), 88);
//                    new_sock.SendDoubleBuffer(transVec.back().data(), 66);
//                    cout<<"minus frame!"<<endl;
//                    timeVec.pop_back();
//                    quatVec.pop_back();
//                    transVec.pop_back();
//                }
//                else if(data=="init data"){
//                    string str;
//                    new_sock >> str; cout<<"Get data for "+str+" vertices..."<<flush;
//                    int vNum = atoi(str.c_str());
//                    G4Timer timer; timer.Start();
//                    for(int i=0;i<vNum;i++){
//                        array<double, 3> p;
//                        array<double, 22> w;
//                        array<double, 24> wj;
//                        new_sock.RecvDoubleBuffer(p.data(),3);
//                        new_sock.RecvDoubleBuffer(w.data(),22);
//                        new_sock.RecvDoubleBuffer(wj.data(),24);
//                    }timer.Stop();
//                    cout<<timer.GetRealElapsed()<<endl;
//                }
//                else if(data=="calib info"){
//                    new_sock.RecvDoubleBuffer(calib.data(),72);
//                    calibRecved = true;
//                }
//                else if(data=="init"){
//                    if(!calibRecved) {new_sock<<"not yet!"; continue;}
//                    new_sock<<"prepared";
//                    new_sock.SendDoubleBuffer(calib.data(),72);
//                }
//            }
//            catch ( SocketException& ) {}

//        }
    }
    catch ( SocketException& e )
    {
        std::cout << "Exception was caught:" << e.description() << "\nExiting.\n";
    }

    return 0;
}

