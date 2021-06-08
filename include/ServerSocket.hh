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

  const ServerSocket& RecvDoubleBuffer ( double*, int ) const;
  const ServerSocket& SendDoubleBuffer ( const double*, int, int wait=0) const;
  const ServerSocket& RecvIntBuffer ( int*, int ) const;
  const ServerSocket& SendIntBuffer ( const int*, int, int wait=0 ) const;

  //Get functions
  sockaddr_in GetAdrrInfo() {return Socket::GetAdrrInfo();}
  int GetSocket() {return Socket::get_socket();}

  void accept ( ServerSocket& );
  void close ( ServerSocket& );

};


#endif

