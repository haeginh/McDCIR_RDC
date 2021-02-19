#include "ServerSocket.h"
#include "SocketException.h"
#include <string>
#include <vector>
#include <array>
#include <utility>

using namespace std;
int main ( int argc, char** argv )
{
    int frameNo(0);
    vector<double> timeVec;
    vector<array<double, 88>> quatVec;
    vector<array<double, 66>> transVec;
    array<double, 72> calib;
    bool calibRecved(false);

    try
    {
        // Create the socket
        ServerSocket server ( 30303 );
        std::cout << "listening....\n";
        double errCode(0.);

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
                    transVec.push_back(transArr);
                    cout<<"received frame #"<<frameNo++<<" (stacked: "<<timeVec.size()<<")"<<endl;
                }
                else if(data=="Idle"){
                    if(!timeVec.size()) {new_sock.SendDoubleBuffer(&errCode, 1); continue;}

                    new_sock.SendDoubleBuffer(&timeVec.back(), 1);
                    new_sock.SendDoubleBuffer(quatVec.back().data(), 88);
                    new_sock.SendDoubleBuffer(transVec.back().data(), 66);
                    cout<<"minus frame!"<<endl;
                    timeVec.pop_back();
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

