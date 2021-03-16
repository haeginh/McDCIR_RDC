#include "ClientSocket.h"
#include "SocketException.h"
#include <string>
#include <sstream>
#include <array>

using namespace std;

int main ( int argc, char** argv )
{
    try{
        string dump;
        while(true){
            array<double, 9> pack = {0,0,0,0,0,0,0,0,0};
            cout<<"pack data: "<<flush;
            dump.clear(); cin>>dump;
            cout<<"stand-by..."<<endl;
            if(!dump.empty()){
                stringstream ss(dump);
                for(int i=0;i<9;i++){
                    double data;
                    ss>>data;
                    pack[i] = data;
                }
            }
            ClientSocket client_socket(string(argv[1]), atoi(argv[2]));
            client_socket<<"tracker";
            client_socket>>dump;
            cout<<"Send data to calculator.."<<endl;
            client_socket.SendDoubleBuffer(pack.data(),9);
        }
    }
    catch ( SocketException& e )
    {
        std::cout << "Exception was caught:" << e.description() << "\nExiting.\n";
    }

    return 0;
}

