#ifndef _SOCKETCONNECT_H
#define _SOCKETCONNECT_H

#include <boost/asio.hpp>
#include <memory>
#include <iostream>
#include <functional>
#include <thread>

#include "base/wzerror.h"

enum class SocketConnectType{ TCPSERVER, TCPCLIENT, UDPSERVER, UDPCLIENT } ;
const char *const SocketConnectTypeString[] = {"TCPSERVER", "TCPCLIENT", "UDPSERVER", "UDPCLIENT"} ;

class SocketConnect
{
public:
    SocketConnect();

    SocketConnect( SocketConnectType t, const std::string &strIP, int port ) ;

    ~SocketConnect() ;

    bool is_open() const ;

    void close() ;

    void run() ;

    void reset( SocketConnectType t, const std::string &strIP, int port ) ;

    void reset( SocketConnectType t, const boost::asio::ip::tcp::endpoint &ep) ;

    void reset( SocketConnectType t, const boost::asio::ip::udp::endpoint &ep) ;

    template <typename MutableBufferSequence>
    void read( const MutableBufferSequence &buffers )
    {
        switch ( _connectType ) {
        case SocketConnectType::TCPCLIENT:
            //boost::asio::read(*_tcpClient_ptr, buffers) ;
            _tcpClient_ptr->read_some(buffers) ;
            break ;
        case SocketConnectType::UDPSERVER:
            _udpServer_ptr->receive_from(buffers, _epReceive) ;
            break ;
        case SocketConnectType::UDPCLIENT:
            _udpClient_ptr->receive_from(buffers, _epServer) ;
            break ;
        default:
            BOOST_THROW_EXCEPTION(WZError()<<err_connect("非法连接类型")) ;
        }
    }

    template <typename MutableBufferSequence>
    void write( const MutableBufferSequence& buffers )
    {
        switch ( _connectType ) {
        case SocketConnectType::TCPCLIENT:
            _tcpClient_ptr->write_some(buffers) ;
            break ;
        case SocketConnectType::UDPSERVER:
            _udpServer_ptr->send_to(buffers, _epReceive) ;
            break ;
        case SocketConnectType::UDPCLIENT:
            _udpClient_ptr->send_to(buffers, _epServer) ;
            break ;
        default:
            BOOST_THROW_EXCEPTION(WZError()<<err_connect("非法连接类型")) ;
        }
    }

    template <typename MutableBufferSequence, typename Handler>
    void async_read( const MutableBufferSequence& buffers, Handler handle)
    {
        switch ( _connectType ) {
        case SocketConnectType::TCPSERVER:
            readTCP( buffers, handle ) ;
            break ;
        case SocketConnectType::TCPCLIENT:
            readTCP( buffers, handle ) ;
            break ;
        case SocketConnectType::UDPSERVER:
            readUDP(buffers, handle) ;
            break ;
        case SocketConnectType::UDPCLIENT:
            readUDP(buffers, handle) ;
            break ;
        default:
            BOOST_THROW_EXCEPTION(WZError()<<err_connect("非法连接类型")) ;
        }
    }

    template <typename MutableBufferSequence, typename Handler>
    void readTCP( const MutableBufferSequence &buffers, Handler handle)
    {
        switch ( _connectType ) {
        case SocketConnectType::TCPCLIENT:
            _tcpClient_ptr->async_read_some(buffers, [this, buffers, handle]( const boost::system::error_code &ec, size_t s){
                handle(ec, s) ;
                //readTCP( buffers, handle ) ;
            }) ;
            break ;
        case SocketConnectType::TCPSERVER:
            _tcpAcceptor_ptr->async_accept(*_tcpClient_ptr, [this, buffers, handle](const boost::system::error_code &){
                std::cout << "有客戶端連入\n" << std::flush ;
                _connectType = SocketConnectType::TCPCLIENT ;
                _tcpClient_ptr->async_read_some(buffers, [this, buffers, handle]( const boost::system::error_code &ec, size_t s){
                    handle(ec, s) ;
                    //readTCP( buffers, handle ) ;
                }) ;
            }) ;
            break ;
        default:
            BOOST_THROW_EXCEPTION(WZError()<<err_connect("非TCP连接类型")) ;
        }
    }

    template <typename MutableBufferSequence, typename Handler>
    void readUDP( const MutableBufferSequence &buffers,
                  Handler handle
                  )
    {
        using namespace std::placeholders ;
        switch ( _connectType ) {
        case SocketConnectType::UDPSERVER:
        {
            _udpServer_ptr->async_receive_from( buffers, _epReceive, [buffers, this, handle](const boost::system::error_code &ec, size_t s){
                handle( ec, s ) ;
                //readUDP( buffers, handle ) ;
            } ) ;
        }
            break ;
        case SocketConnectType::UDPCLIENT:
        {
            _udpClient_ptr->async_receive_from( buffers, _epServer, [buffers, this, handle](const boost::system::error_code &ec, size_t s){
                handle( ec, s ) ;
                //readUDP( buffers, handle ) ;
            } ) ;
        }
            break ;
        default:
            BOOST_THROW_EXCEPTION(WZError()<<err_connect("非UDP连接类型")) ;
        }
    }

private:
    /// TCP客户端指针
    std::shared_ptr<boost::asio::ip::tcp::socket> _tcpClient_ptr ;
    /// TCP服务端指针
    std::shared_ptr<boost::asio::ip::tcp::socket> _tcpServer_ptr ;
    /// TCP接收器
    std::shared_ptr<boost::asio::ip::tcp::acceptor> _tcpAcceptor_ptr ;
    /// UDP客户端指针
    std::shared_ptr<boost::asio::ip::udp::socket> _udpClient_ptr ;
    /// UDP服务端指针
    std::shared_ptr<boost::asio::ip::udp::socket> _udpServer_ptr ;
    /// asio服务
    std::shared_ptr<boost::asio::io_service> _service_ptr ;
    boost::asio::io_service _service ;
    //boost::asio::io_service::work _work ;
    /// 连接方式
    SocketConnectType _connectType ;
    /// 是否已经连接
    std::thread _threadSocket ;
    bool _bConnected ;
    bool _bRun ;
    boost::asio::ip::udp::endpoint _epReceive ;
    boost::asio::ip::udp::endpoint _epServer ;
private:
    SocketConnect( SocketConnectType t, const boost::asio::ip::tcp::endpoint &ep) ;

    SocketConnect( SocketConnectType t, const boost::asio::ip::udp::endpoint &ep) ;
} ;

#endif
