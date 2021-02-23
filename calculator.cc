#include "ClientSocket.h"
#include "SocketException.h"
#include "ModelImport.h"
#include <iostream>
#include <string>
#include <array>

using namespace std;
int main ( int argc, char** argv)
{
    //read mesh file
    G4String model(argv[1]);
    ModelImport modelImport(model);


    array<double, 72> calib;
    bool calibRecved(false);

    try{
        while(!calibRecved){
            ClientSocket client_socket_init ( "localhost", 30303 );
            client_socket_init<<"init_c";
            int vNum;
            client_socket_init.RecvIntBuffer(&vNum,1);
            if(!vNum)continue;
        }

        while(true){
            ClientSocket client_socket ( "localhost", 30303 );
            client_socket<<"Idle";
            double time;
            array<double, 88> quatArr;
            array<double, 66> transArr;
            client_socket.RecvDoubleBuffer(&time, 1);
            if(time==0) continue;
            client_socket.RecvDoubleBuffer(quatArr.data(), 88);
            client_socket.RecvDoubleBuffer(transArr.data(), 66);
            //              cout<<quatArr.at(0)<<" "<<quatArr.at(1)<<" "<<quatArr.at(2)<<" "<<quatArr.at(3)<<endl;
            //              cout<<transArr.at(0)<<" "<<transArr.at(2)<<" "<<transArr.at(3)<<endl;
        }
    }
    catch ( SocketException& e )
    {
      std::cout << "Exception was caught:" << e.description() << "\n";
    }

  return 0;
}

