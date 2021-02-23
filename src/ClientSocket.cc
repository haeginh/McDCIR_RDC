// Implementation of the ClientSocket class

#include "ClientSocket.h"
#include "SocketException.h"


ClientSocket::ClientSocket ( std::string host, int port )
{
  if ( ! Socket::create() )
    {
      throw SocketException ( "Could not create client socket." );
    }

  if ( ! Socket::connect ( host, port ) )
    {
      throw SocketException ( "Could not bind to port." );
    }

}


const ClientSocket& ClientSocket::operator << ( const std::string& s ) const
{
  if ( ! Socket::send ( s ) )
    {
      throw SocketException ( "Could not write to socket." );
    }

  return *this;

}

const ClientSocket& ClientSocket::RecvDoubleBuffer ( double* arr, int num ) const
{
  if ( ! Socket::recv ( arr, num ) )
    {
      throw SocketException ( "Could not read from socket." );
    }

  return *this;
}

const ClientSocket& ClientSocket::SendDoubleBuffer ( const double* buf, int num) const{
    if ( ! Socket::send ( buf, num ) )
      {
        throw SocketException ( "Could not write to socket." );
      }

    return *this;
}

const ClientSocket& ClientSocket::RecvIntBuffer ( int* arr, int num ) const
{
  if ( ! Socket::recv ( arr, num ) )
    {
      throw SocketException ( "Could not read from socket." );
    }

  return *this;
}

const ClientSocket& ClientSocket::SendIntBuffer ( const int* buf, int num) const{
    if ( ! Socket::send ( buf, num ) )
      {
        throw SocketException ( "Could not write to socket." );
      }

    return *this;
}

const ClientSocket& ClientSocket::operator >> ( std::string& s ) const
{
  if ( ! Socket::recv ( s ) )
    {
      throw SocketException ( "Could not read from socket." );
    }

  return *this;
}

