// Definition of the ServerSocket class

#ifndef ServerSocket_class
#define ServerSocket_class

#include "Socket.h"


class ServerSocket : private Socket
{
 public:

  ServerSocket ( int port );
  ServerSocket (){}
  virtual ~ServerSocket();

  const ServerSocket& operator << ( const std::string& ) const;
  const ServerSocket& operator >> ( std::string& ) const;

  const ServerSocket& RecvDoubleBuffer ( double*, int ) const;
  const ServerSocket& SendDoubleBuffer ( const double*, int ) const;

  void accept ( ServerSocket& );
  void close ( ServerSocket& );

};


#endif

