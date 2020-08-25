#include "device/socketConnect.h"
#include "base/wzerror.h"
#include "base/data_types.h"

#include <thread>
using namespace std ;

using namespace boost::asio ;
using namespace boost::asio::ip ;

SocketConnect::SocketConnect():
  _bConnected(false),
  _bRun(false)
{
    cout << "无参\n" << flush ;
}

SocketConnect::SocketConnect( SocketConnectType t, const std::string &strIP, int port )
   :_bConnected{false},
    _bRun{false}
{
   reset(t, strIP, port) ;
}

SocketConnect::SocketConnect( SocketConnectType t, const boost::asio::ip::tcp::endpoint &ep)
   :_bConnected{false},
    _bRun{false}
{
   reset( t, ep ) ;
}

SocketConnect::SocketConnect( SocketConnectType t, const boost::asio::ip::udp::endpoint &ep)
   :_bConnected{false},
    _bRun{false}
{
   reset( t, ep ) ;
}

SocketConnect::~SocketConnect()
{
   this->close() ;
}

void SocketConnect::run()
{
   if ( !_bRun ) {
      _threadSocket = std::thread([this]{
            _bRun = true ;
            _service.run();
            _bRun = false ;
            }) ;
   }
}

bool SocketConnect::is_open() const
{
   return _bConnected ;
}

void SocketConnect::close()
{
   if ( _bConnected ) {
       if ( _bRun )
          _service.stop() ;
       if ( _threadSocket.joinable())
            _threadSocket.join() ;
       else {
         while ( _bRun ) this_thread::sleep_for(1_ms) ;
       }
       _bConnected = false ;
      switch ( _connectType ) {
      case SocketConnectType::TCPSERVER:
         _tcpServer_ptr->close() ;
         _tcpServer_ptr.reset() ;
         break ;
      case SocketConnectType::TCPCLIENT:
         _tcpClient_ptr->close() ;
         _tcpClient_ptr.reset() ;
         break ;
      case SocketConnectType::UDPSERVER:
         _udpServer_ptr->close() ;
         _udpServer_ptr.reset() ;
         break ;
      case SocketConnectType::UDPCLIENT:
         _udpClient_ptr->close() ;
         _udpClient_ptr.reset() ;
         break ;
      default:
         BOOST_THROW_EXCEPTION(WZError()<<err_connect("非法连接类型")) ;
      }
   }
}

void SocketConnect::reset( SocketConnectType t, const std::string &strIP, int port )
{
   tcp::endpoint epTCP ;
   udp::endpoint epUDP ;
   if ( "" == strIP ) {
      epTCP = tcp::endpoint(tcp::v4(), port) ;
      epUDP = udp::endpoint(udp::v4(), port) ;
   }
   else {
       epTCP = tcp::endpoint(ip::address_v4::from_string(strIP), port) ;
       epUDP = udp::endpoint(ip::address_v4::from_string(strIP), port) ;
   }
   switch ( t ) {
   case SocketConnectType::TCPSERVER:
      reset(t, epTCP) ;
      break ;
   case SocketConnectType::TCPCLIENT:
      reset(t, epTCP) ;
      break ;
   case SocketConnectType::UDPSERVER:
      reset(t, epUDP) ;
      break ;
   case SocketConnectType::UDPCLIENT:
      _epServer = epUDP ;
      reset(t, epUDP) ;
      break ;
   default:
      BOOST_THROW_EXCEPTION(WZError()<<err_connect("非法连接类型")) ;
   }
}

void SocketConnect::reset( SocketConnectType t, const boost::asio::ip::tcp::endpoint &ep)
{
   this->close() ;
   switch ( t ) {
   case SocketConnectType::TCPSERVER:
      {
         _tcpServer_ptr.reset( new ip::tcp::socket( _service ) ) ;
         _tcpAcceptor_ptr.reset( new ip::tcp::acceptor(_service, ep)) ;
         _tcpClient_ptr.reset( new ip::tcp::socket( _service ) ) ;
      }
      break ;
   case SocketConnectType::TCPCLIENT:
      {
         _tcpClient_ptr.reset( new ip::tcp::socket( _service ) ) ;
         _tcpClient_ptr->connect( ep ) ;
      }
      break ;
   default:
      BOOST_THROW_EXCEPTION(WZError() << err_connect("非TCP服务端网络连接")) ;
   }
   _connectType = t ;
   _bConnected = true ;
}

void SocketConnect::reset( SocketConnectType t, const boost::asio::ip::udp::endpoint &ep)
{
   this->close() ;
   switch ( t ) {
   case SocketConnectType::UDPSERVER:
      _udpServer_ptr.reset( new ip::udp::socket( _service ) ) ;
      _udpServer_ptr->open(ep.protocol()) ;
      _udpServer_ptr->bind(ep) ;
      break ;
   case SocketConnectType::UDPCLIENT:
      _udpClient_ptr.reset( new ip::udp::socket( _service ) ) ;
      _udpClient_ptr->open(ep.protocol()) ;
      break ;
   default:
      BOOST_THROW_EXCEPTION(WZError() << err_connect("非UDP网络连接")) ;
   }
   _connectType = t ;
   _bConnected = true ;
}
