#include "ClientSocket.h"
#include "SocketException.h"
#include <iostream>
#include <string>
#include <array>

using namespace std;
int main ( int argc, char** argv)
{
    while(1){
        double sum(0);
        for(int i=0;i<1000000;i++) sum+= i*0.05;
  try
    {

      ClientSocket client_socket ( "localhost", 30303 );

      std::string reply;

      try
	{
          client_socket<<"Test message.";
          double time;
          array<double, 88> quatArr;
          array<double, 66> transArr;
          client_socket.RecvDoubleBuffer(&time, 1);
          if(time>0){
              client_socket.RecvDoubleBuffer(quatArr.data(), 88);
              client_socket.RecvDoubleBuffer(transArr.data(), 66);
//              cout<<quatArr.at(0)<<" "<<quatArr.at(1)<<" "<<quatArr.at(2)<<" "<<quatArr.at(3)<<endl;
//              cout<<transArr.at(0)<<" "<<transArr.at(2)<<" "<<transArr.at(3)<<endl;
          }
    }
      catch ( SocketException& ) {}

    //  std::cout << "We received this response from the server:\n\"" << reply << "\"\n";;

    }
  catch ( SocketException& e )
    {
      std::cout << "Exception was caught:" << e.description() << "\n";
    }
}
  return 0;
}

