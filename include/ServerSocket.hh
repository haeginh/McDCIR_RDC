// Definition of the ServerSocket class

#ifndef ServerSocket_class
#define ServerSocket_class

#include "Socket.hh"


class ServerSocket : private Socket
{
 public:

  ServerSocket ( int port );
  ServerSocket (){}
  virtual ~ServerSocket();

  const ServerSocket& operator << ( const std::string& ) const;
  const ServerSocket& operator >> ( std::string& ) const;

  int RecvDoubleBuffer ( double*, int );
  int SendDoubleBuffer ( const double*, int, int wait=0);
  int RecvIntBuffer ( int*, int );
  int SendIntBuffer ( const int*, int, int wait=0 );

  //Get functions
  sockaddr_in GetAdrrInfo() {return Socket::GetAdrrInfo();}
  int GetSocket() {return Socket::get_socket();}

  void accept ( ServerSocket& );
  void close ( ServerSocket& );

};


#endif

