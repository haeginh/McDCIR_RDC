#include "ServerSocket.h"
#include "SocketException.h"
#include <string>
#include <vector>
#include <array>
#include <utility>
#include <igl/Timer.h>

using namespace std;
int main ( int argc, char** argv )
{
    //init data
    array<double, 72> calib;

    int frameNo(0);
    vector<double> timeVec;
    vector<array<double, 88>> quatVec;
    vector<array<double, 66>> transVec;
    bool calibRecved(false);

    try
    {
        // Create the socket
        ServerSocket server ( 30303 );
        std::cout << "Listening....\n";
        int errCode(0.);

        //1. Initialization(mesh and weights): process a single connection
        ///need to recieve "init data" from RemoteCal
        ///ignore calculator("init")
        vector<array<double, 3>> V;
        vector<array<int, 3>> F;
        vector<array<int, 4>> T;
        vector<array<double, 22>> W;
        vector<array<double, 24>> Wj;

        try
        {
            while ( true )
            {
                ServerSocket new_sock;
                server.accept ( new_sock );
                std::string data;
                new_sock >> data;
                if(data=="init_c"){
                    new_sock.SendIntBuffer(&errCode, 1);
                }
                else if(data=="init"){
                    igl::Timer timer; timer.start();
                    int vNum; new_sock.RecvIntBuffer(&vNum,1);
                    cout<<"Get data for "<<vNum<<" vertices..."<<flush;
                    for(int i=0;i<vNum;i++){
                        array<double, 3> p;
                        array<double, 22> w;
                        array<double, 24> wj;
                        new_sock.RecvDoubleBuffer(p.data(),3);
                        V.push_back(p);
                        new_sock.RecvDoubleBuffer(w.data(),22);
                        W.push_back(w);
                        new_sock.RecvDoubleBuffer(wj.data(),24);
                        Wj.push_back(wj);
                    } timer.stop(); double vTime = timer.getElapsedTimeInSec();
                    cout<<vTime<<endl; timer.start();

                    int tNum; new_sock.RecvIntBuffer(&tNum,1);
                    cout<<"Get data for "<<tNum<<" tetrahedrons..."<<flush;
                    for(int i=0;i<tNum;i++){
                        array<int, 4> t;
                        new_sock.RecvIntBuffer(t.data(),4);
                        T.push_back(t);
                    }timer.stop(); double tTime = timer.getElapsedTimeInSec();
                    cout<<tTime<<endl; timer.start();

                    int fNum; new_sock.RecvIntBuffer(&fNum,1);
                    cout<<"Get data for "<<fNum<<" faces..."<<flush;
                    for(int i=0;i<fNum;i++){
                        array<int, 3> f;
                        new_sock.RecvIntBuffer(f.data(),3);
                        F.push_back(f);
                    }timer.stop();double fTime = timer.getElapsedTimeInSec();
                    cout<<fTime<<endl; timer.start();

                    cout<<"total: "<<vTime+fTime+tTime<<endl;
                    break;
                }
            }
        }
        catch ( SocketException& ) {}
        cout<<"init. data is ready. Listening..."<<endl;

        //
        while ( true )
        {
            ServerSocket new_sock;
            server.accept ( new_sock );
            try
            {
                std::string data;
                new_sock >> data;
                if(data=="quaternion"){
                    double time;
                    array<double, 88> quatArr;
                    array<double, 66> transArr;
                    new_sock.RecvDoubleBuffer(&time, 1);
                    new_sock.RecvDoubleBuffer(quatArr.data(), 88);
                    new_sock.RecvDoubleBuffer(transArr.data(), 66);
//                    cout<<quatArr.at(0)<<" "<<quatArr.at(1)<<" "<<quatArr.at(2)<<" "<<quatArr.at(3)<<endl;
//                    cout<<transArr.at(0)<<" "<<transArr.at(2)<<" "<<transArr.at(3)<<endl;
                    timeVec.push_back(time);
                    quatVec.push_back(quatArr);
                    cout<<"received frame #"<<frameNo++<<" (stacked: "<<timeVec.size()<<")"<<endl;
                }
                else if(data=="Idle"){
                    if(!timeVec.size()) {new_sock.SendIntBuffer(&errCode, 1); continue;}

                    new_sock.SendDoubleBuffer(&timeVec.back(), 1);
                    new_sock.SendDoubleBuffer(quatVec.back().data(), 88);
                    new_sock.SendDoubleBuffer(transVec.back().data(), 66);
                    cout<<"minus frame!"<<endl;
                    timeVec.pop_back();
                    quatVec.pop_back();
                    transVec.pop_back();
                }
                else if(data=="init data"){
                    string str;
                    new_sock >> str; cout<<"Get data for "+str+" vertices..."<<flush;
                    int vNum = atoi(str.c_str());
                    igl::Timer timer; timer.start();
                    for(int i=0;i<vNum;i++){
                        array<double, 3> p;
                        array<double, 22> w;
                        array<double, 24> wj;
                        new_sock.RecvDoubleBuffer(p.data(),3);
                        new_sock.RecvDoubleBuffer(w.data(),22);
                        new_sock.RecvDoubleBuffer(wj.data(),24);
                    }timer.stop();
                    cout<<timer.getElapsedTimeInSec()<<endl;
                }
                else if(data=="calib info"){
                    new_sock.RecvDoubleBuffer(calib.data(),72);
                    calibRecved = true;
                }
                else if(data=="init"){
                    if(!calibRecved) {new_sock<<"not yet!"; continue;}
                    new_sock<<"prepared";
                    new_sock.SendDoubleBuffer(calib.data(),72);
                }
            }
            catch ( SocketException& ) {}

        }
    }
    catch ( SocketException& e )
    {
        std::cout << "Exception was caught:" << e.description() << "\nExiting.\n";
    }

    return 0;
}

