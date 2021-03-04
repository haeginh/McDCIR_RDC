#include "ServerSocket.h"
#include "SocketException.h"
#include <string>
#include <vector>
#include <map>
#include <array>
#include <utility>
#include "G4Timer.hh"
#include <Eigen/Dense>
#include <thread>
#include <mutex>

using namespace std;

int main ( int argc, char** argv )
{
    //set the port# from argument
    int port(30303);
    if(argc==2) port = atoi(argv[1]);

    //threads
    thread* remoteCal_th;
    vector<thread*> calculator_th;
    int calID(0);

    //init data
    vector<array<double, 49>> packets; // V, W, Wj
    vector<array<int, 3>> F;
    vector<array<int, 4>> T;
    bool initChk(false);
    array<double, 72> calib;
    bool calibChk(false);

    //posture data
    vector<array<double, 155>> frameData;
    mutex m;

    bool quitSig(false);

    try
    {
        // Create the socket
        ServerSocket server( port );
        cout << "Listening....\n";

        try
        {
            //1. RC_thread (producer)
            while ( true )
            {
                ServerSocket* _sock = new ServerSocket;
                server.accept ( *_sock );
                std::string _data("");
                (*_sock) >> _data;

                if(_data.substr(0,4)=="init"){
                    remoteCal_th = new thread([&](ServerSocket* sock, string data)
                    {
                            G4Timer timer; timer.Start();
                            string dump; int vNum, tNum, fNum;
                            stringstream ss(data); ss>>dump>>vNum>>tNum>>fNum;
                            dump = "chk";
                            (*sock) << dump;
                            cout<<"RC_proc: Get data for "<<vNum<<" vertices..."<<flush;
                            for(int i=0;i<vNum;i++){
                                  array<double, 49> packet;
                                  sock->RecvDoubleBuffer(packet.data(),49);
                                  packets.push_back(packet);
                                  (*sock) << dump;
                            } timer.Stop(); double vTime = timer.GetRealElapsed();
                            cout<<vTime<<endl; timer.Start();

                            cout<<"RC_proc: Get data for "<<tNum<<" tetrahedrons..."<<flush;
                            for(int i=0;i<tNum;i++){
                                array<int, 4>                        t;
                                sock->RecvIntBuffer(t.data(),4);
                                T.push_back(t);
                                (*sock) << dump;
                            }timer.Stop(); double tTime = timer.GetRealElapsed();
                            cout<<tTime<<endl; timer.Start();

                            cout<<"RC_proc: Get data for "<<fNum<<" faces..."<<flush;
                            for(int i=0;i<fNum;i++){
                                array<int, 3> f;
                                sock->RecvIntBuffer(f.data(),3);
                                F.push_back(f);
                                (*sock) << dump;
                            }timer.Stop(); double fTime = timer.GetRealElapsed();
                            cout<<fTime<<endl; timer.Start();
                            cout<<"total: "<<vTime+fTime+tTime<<endl;
                            initChk =  true;

                            sock->RecvDoubleBuffer(calib.data(),72);
                            cout<<"RC_proc: received calib data"<<endl;
                            calibChk = true;

                            while(true){
                                array<double, 155> pack;
                                sock->RecvDoubleBuffer(pack.data(),155);
                                int frameNo = (int)pack[0];
                                if(frameNo==0) break;

                                m.lock();
                                frameData.push_back(pack);
                                cout<<"RC_proc: received frame #"<<frameNo<<endl;
                                m.unlock();
                            }
                            quitSig = true;
                    },_sock, _data);
                cout<<"rc_thread is ready."<<endl;
                break;
            }
            else if(_data.substr(0,4)=="Idle" || _data.substr(0.3)=="vis"){
                (*_sock) << "wait";
                delete _sock;
            }
        }

        while(true){ //wait for the initialization of the RC_thread
            if(initChk) break; usleep(10);
        }

        //2. CAL_threads (producer)
        //dose data
        map<int, vector<double>> doseData;
        mutex m2;
        thread* remoteCal_vis;
        int numOfPack = floor(F.size()/180)+1;

        while ( true )
        {
            ServerSocket* _sock = new ServerSocket;
            server.accept ( *_sock );
            std::string data("");
            (*_sock) >> data;
            if(data.substr(0,4)=="Idle"){
                (*_sock)<<to_string(calID);
                calculator_th.push_back(
                            new thread([&](ServerSocket* sock, int idx){
                                std::string dump = (to_string(packets.size())+" "+to_string(T.size())+" "+to_string(F.size()));
                                (*sock) << dump ;
                                (*sock) >> dump;
                                for(size_t i=0;i<packets.size();i++){
                                    sock->SendDoubleBuffer(packets[i].data(),49);
                                    (*sock)>>dump;
                                }
                                for(size_t i=0;i<T.size();i++){
                                    sock->SendIntBuffer(T[i].data(),4);
                                    (*sock)>>dump;
                                }
                                for(size_t i=0;i<F.size();i++){
                                    sock->SendIntBuffer(F[i].data(),3);
                                    (*sock)>>dump;
                                }
                                cout<<"Cal #"<<idx<<": Sent init data"<<endl;

                                while(true){
                                    if(calibChk) break; usleep(10);
                                }

                                sock->SendDoubleBuffer(calib.data(),72);
                                cout<<"Cal #"<<idx<<": Sent calib data"<<endl;

                                while((!quitSig) || frameData.size()){
                                    m.lock();
                                    if(!frameData.size()){
                                        m.unlock();
                                        continue;
                                    }

                                    array<double, 155> pack = frameData.back();
                                    frameData.pop_back();
                                    sock->SendDoubleBuffer(pack.data(),155);
                                    cout<<"Cal #"<<idx<<": Sent frame #"<<(int)pack[0]<<" (stacked: "<<frameData.size()<<")"<<endl;
                                    m.unlock();

                                    double buff[180];
                                    vector<double> dose;
                                    for(int i=0, n=0;i<numOfPack;i++){
                                        sock->RecvDoubleBuffer(buff,180);
                                        for(int j=0;j<180 && n<F.size();j++, n++)
                                            dose.push_back(buff[j]);
                                        if(n<F.size())(*sock)<<"chk";
                                    }

                                    m2.lock();
                                    doseData[abs((int)pack[0])] = dose;
                                    m2.unlock();
                                }
                            }, _sock, calID));
                cout<<"Calculator #"<<calID++<<" was activated!"<<endl;
                continue;
            }
            else if(data.substr(0,3)=="vis" && remoteCal_vis==NULL){
                (*_sock)<<"chk";
                remoteCal_vis = new thread([&](ServerSocket* sock){
                        cout<<"VIS_proc was generated"<<endl;
                        string dump;
                        while(true){
                            m2.lock();
                            if(doseData.size()==0){
                                m2.unlock();
                                usleep(10);
                                continue;
                            }
                            cout<<"VIS_proc: send "<<doseData.size()<<" dose data"<<endl;
                            for(auto iter:doseData){
                                sock->SendIntBuffer(&iter.first,1);
                                (*sock)>>dump;
                                double buff[180];
                                for(int i=0, n=0;i<numOfPack;i++){
                                    for(int j=0;j<180 && n<F.size();j++, n++)
                                        buff[j] = iter.second[n];
                                    sock->SendDoubleBuffer(buff,180);
                                    if(n<F.size()) (*sock)>>dump;
                                }
                            }
                            m2.unlock();
                        }
                }, _sock);
                continue;
            }
            delete _sock;
        }
    }
    catch ( SocketException& ) {}
}
catch ( SocketException& e )
{
    std::cout << "Exception was caught:" << e.description() << "\nExiting.\n";
}

return 0;
}

