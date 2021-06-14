// Implementation of the ClientSocket class

#include "ClientSocket.hh"
#include "SocketException.hh"


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

  //Socket::set_non_blocking(false);
}


const ClientSocket& ClientSocket::operator << ( const std::string& s ) const
{
  if ( ! Socket::send ( s ) )
    {
      throw SocketException ( "Could not write to socket." );
    }
  else usleep(1);
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

const ClientSocket& ClientSocket::SendDoubleBuffer ( const double* buf, int num, int wait) const{
    if ( ! Socket::send ( buf, num ) )
      {
        throw SocketException ( "Could not write to socket." );
      }
    else usleep(wait);
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

const ClientSocket& ClientSocket::SendIntBuffer ( const int* buf, int num, int wait) const{
    if ( ! Socket::send ( buf, num ) )
      {
        throw SocketException ( "Could not write to socket." );
      }
    else usleep(wait);
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

