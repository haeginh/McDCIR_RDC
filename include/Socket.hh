// Definition of the Socket class

#ifndef Socket_class
#define Socket_class


#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <string>
#include <arpa/inet.h>


const int MAXHOSTNAME = 200;
const int MAXCONNECTIONS = 5;
const int MAXRECV = 500;

class Socket
{
 public:
  Socket();
  virtual ~Socket();

  // Server initialization
  bool create();
  bool bind ( const int port );
  bool listen() const;
  bool accept ( Socket& ) const;
  void close ( Socket& new_socket ) const;

  // Client initialization
  bool connect ( const std::string host, const int port );

  // Data Transimission
  bool send ( const std::string ) const;
  int recv ( std::string& ) const;

  // Buffer Transimission
  bool send ( const double*, int ) const;
  bool send ( const int*, int ) const;
  int recv ( double*, int ) const;
  int recv ( int*, int ) const;


  void set_non_blocking ( const bool );

  bool is_valid() const { return m_sock != -1; }
  int get_socket() const { return m_sock; } 
  //Get functions
  sockaddr_in GetAdrrInfo() {return m_addr;}

 private:

  int m_sock;
  sockaddr_in m_addr;


};


#endif

