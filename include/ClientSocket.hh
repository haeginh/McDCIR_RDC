// Definition of the ClientSocket class

#ifndef ClientSocket_class
#define ClientSocket_class

#include "Socket.hh"


class ClientSocket : private Socket
{
 public:

  ClientSocket(){}
  ClientSocket ( std::string host, int port );
  virtual ~ClientSocket(){}

  const ClientSocket& operator << ( const std::string& ) const;
  const ClientSocket& operator >> ( std::string& ) const;

  const ClientSocket& RecvDoubleBuffer ( double*, int ) const;
  const ClientSocket& SendDoubleBuffer ( const double*, int, int wait=0) const;
  const ClientSocket& RecvIntBuffer ( int*, int ) const;
  const ClientSocket& SendIntBuffer ( const int*, int, int wait=0 ) const;

  const bool is_valid() const { return Socket::is_valid(); }
};


#endif

