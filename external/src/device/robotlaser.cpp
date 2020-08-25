#include "device/robotlaser.h"
#include <iostream>
#include <boost/thread.hpp>
#include <boost/algorithm/hex.hpp>
#include "base.h"
#include "base/wzerror.h"
#include "base/config.h"

using namespace boost::asio ;
using namespace std;

RobotLaser::RobotLaser()
    :_laserStatus(CLOSE),
      _bOpenLED{false}
{
    memset( _cmd, 0, sizeof(_cmd) ) ;
    memset( _readBuffer, 0, sizeof(_readBuffer) ) ;
    connectDevice();
    initLight();
}

RobotLaser::~RobotLaser()
{
    disconnectDevice();
}

void RobotLaser::connectDevice()
{
    std::lock_guard<std::recursive_mutex> lck(_mtx) ;
    try {
        if ( !_serialPort_ptr ) {
            Config config("etc/sys.info") ;
            _serialPort_ptr.reset( new  serial_port(_service, config.get<std::string>("laser.serialPort")) );
            _serialPort_ptr->set_option(serial_port::baud_rate(config.get<int>("laser.baudRate"))) ;
            _serialPort_ptr->set_option(serial_port::flow_control(serial_port::flow_control::none)) ;
            _serialPort_ptr->set_option(serial_port::parity(serial_port::parity::none)) ;
            _serialPort_ptr->set_option(serial_port::stop_bits(serial_port::stop_bits::one)) ;
            _serialPort_ptr->set_option(serial_port::character_size(serial_port::character_size(8))) ;
            boost::asio::async_read( *_serialPort_ptr, boost::asio::buffer(_readBuffer, 2),
                                     boost::bind( &RobotLaser::recvReply, this, _1, _2 ) ) ;
            _thService = std::thread([this]() {_service.run();}) ;
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }
    }
    catch ( const boost::system::system_error &e ) {
        std::cout <<"e.what :"<< e.what() << std::endl ;
    }
    catch ( const boost::exception &e ) {
        DUMPERROR(e) ;
    }
}

void RobotLaser::disconnectDevice()
{
    std::lock_guard<std::recursive_mutex> lck(_mtx) ;
    if ( _serialPort_ptr ) {
        if ( _serialPort_ptr->is_open() ) {
            alwaysClose();
            closeLED() ;
            _service.stop() ;
            _serialPort_ptr->close();
            _thService.join() ;
        }
        _serialPort_ptr = nullptr;
    }
}

void RobotLaser::reconnectDevice()
{
    disconnectDevice();
    connectDevice();
}

void RobotLaser::alwaysClose()
{
    std::lock_guard<std::recursive_mutex> lck(_mtx) ;
    if ( _serialPort_ptr && _serialPort_ptr->is_open() ) {
        if ( CLOSE != _laserStatus ) {
            _cmd[0] = 0xA5 ;
            _cmd[1] = 0xF3 ;
            _cmd[2] = 0x00 ;
            _cmd[3] = 0x00 ;
            _cmd[4] = 0x00 ;
            _cmd[5] = 0x00 ;
            _cmd[6] = 0xFF ;
            _cmd[7] = 0xFF ;
            _bReply = false ;
            boost::asio::write(*_serialPort_ptr, buffer(_cmd, 8)) ;
            waitReply() ;
            _laserStatus = CLOSE ;
        }
    }
}

void RobotLaser::alwaysLight()
{
    try
    {
        std::lock_guard<std::recursive_mutex> lck(_mtx) ;
        if ( !_serialPort_ptr || LIGHT == _laserStatus ) return ;
        Config config("etc/sys.info") ;
        _cmd[0] = 0xA5 ;
        _cmd[1] = 0xF2 ;
        _cmd[2] = config.get<int>("laser.brightness");
        std::cout<<"brightness :"<< config.get<int>("laser.brightness")<<std::endl;
        _cmd[3] = 0x00 ;
        _cmd[4] = 0x00 ;
        _cmd[5] = 0x00 ;
        _cmd[6] = 0xFF ;
        _cmd[7] = 0xFF ;
        _bReply = false ;
        boost::asio::write(*_serialPort_ptr, buffer(_cmd, 8)) ;
        waitReply() ;
        _laserStatus = LIGHT ;
    }catch(...)
    {
        cout<<"open light failed"<<endl;
    }
}

void RobotLaser::initLight()
{
    std::lock_guard<std::recursive_mutex> lck(_mtx) ;
    std::cout<<__LINE__<<__FUNCTION__<<std::endl;
    if ( !_serialPort_ptr || LIGHT == _laserStatus ) return ;
    Config config("etc/sys.info") ;
    _cmd[0] = 0xA5 ;
    _cmd[1] = 0xF2 ;
    _cmd[2] = 0x01 ;
    _cmd[3] = 0x00 ;
    _cmd[4] = 0x00 ;
    _cmd[5] = 0x00 ;
    _cmd[6] = 0xFF ;
    _cmd[7] = 0xFF ;
    _bReply = false ;
    boost::asio::write(*_serialPort_ptr, buffer(_cmd, 8)) ;
    waitReply() ;
    _laserStatus = CLOSE ;
}

void RobotLaser::flicker()
{
    std::lock_guard<std::recursive_mutex> lck(_mtx) ;
    if ( !_serialPort_ptr || FLICKER == _laserStatus ) return ;
    Config config("etc/sys.info") ;
    _cmd[0] = 0xA5 ;
    _cmd[1] = 0xF1 ;
    _cmd[2] = config.get<int>("laser.brightness") ;
    _cmd[3] = 0x00 ;
    _cmd[4] = 0x00 ;
    _cmd[5] = 0x00 ;
    _cmd[6] = 0xFF ;
    _cmd[7] = 0xFF ;
    _bReply = false ;
    boost::asio::write(*_serialPort_ptr, buffer(_cmd, 8)) ;
    waitReply() ;
    _laserStatus = FLICKER ;
}

void RobotLaser::openLED()
{
    std::lock_guard<std::recursive_mutex> lck(_mtx) ;
    if ( !_serialPort_ptr || _bOpenLED ) return ;
    _cmd[0] = 0xA5 ;
    _cmd[1] = 0xF4 ;
    _cmd[2] = 0x00 ;
    _cmd[3] = 0x00 ;
    _cmd[4] = 0x00 ; //_config.get<int>("laser.triggerTime") ;
    _cmd[5] = 0x00 ; //_config.get<int>("laser.intervalTime") ;
    _cmd[6] = 0xFF ; //_config.get<int>("laser.brightness") ;
    _cmd[7] = 0xFF ;
    _bReply = false ;
    boost::asio::write(*_serialPort_ptr, buffer(_cmd, 8)) ;
    waitReply() ;
    _bOpenLED = true ;
}

void RobotLaser::closeLED()
{
    std::lock_guard<std::recursive_mutex> lck(_mtx) ;
    if ( !_serialPort_ptr || !_bOpenLED ) return ;
    _cmd[0] = 0xA5 ;
    _cmd[1] = 0xF5 ;
    _cmd[2] = 0x00 ;
    _cmd[3] = 0x00 ;
    _cmd[4] = 0x00 ; //_config.get<int>("laser.triggerTime") ;
    _cmd[5] = 0x00 ; //_config.get<int>("laser.intervalTime") ;
    _cmd[6] = 0xFF ; //_config.get<int>("laser.brightness") ;
    _cmd[7] = 0xFF ;
    _bReply = false ;
    boost::asio::write(*_serialPort_ptr, buffer(_cmd, 8)) ;
    waitReply() ;
    _bOpenLED = false ;
}

void RobotLaser::recvReply( const boost::system::error_code &ec, size_t )
{
    if ( !ec ) {
        std::string out ;
        boost::algorithm::hex(_readBuffer, std::back_inserter(out)) ;
        std::cout <<"out :"<< out << std::endl ;
        boost::asio::async_read( *_serialPort_ptr, boost::asio::buffer(_readBuffer, 2),
                                 boost::bind( &RobotLaser::recvReply, this, _1, _2 ) ) ;
        _bReply = true ;
        _condReply.notify_one() ;
    }
}

void RobotLaser::waitReply()
{
    using namespace std::chrono ;
    Millsecond m1 = steady_clock::now() ;
    std::unique_lock<std::mutex> lck(_mtxReply) ;
    // 考虑到异常情况，设置超时时间
    _condReply.wait_for( lck, 100_ms, [this]{return _bReply ;}) ;
    Millsecond m2 = steady_clock::now() ;
    //std::cout << (m2 - m1).count() << std::endl ;
    std::this_thread::sleep_for( 50_ms ) ;
}
