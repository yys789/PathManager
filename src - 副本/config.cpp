#include "include/config.h"
#include <iostream>
#include <Eigen/Dense>
#include <boost/algorithm/string.hpp>
using namespace std ;

map<string, Config::ConfigInfo> Config::_mapPtree ;
recursive_mutex Config::_mtx ;
string Config::_robotName ;

Config::Config( const decltype(ConfigInfo::pt_ptr->get_child_optional("")) &pt )
    : _bWritten{false}
{
    lock_guard<recursive_mutex> lck(_mtx) ;
    _mapPtree[_fileName].refName = _fileName ;
    _pt_ptr.reset( new boost::property_tree::ptree(*pt) ) ;
    _mtx_ptr = &(_mapPtree[_fileName].mtx) ;
}

Config::Config( const char *pstr, bool bWritten, bool useRobot )
    :Config(string(pstr), bWritten, useRobot)
{
}

Config::Config( const string &fileName, bool bWritten, bool useRobot )
    :_fileName(fileName),
      _bWritten{bWritten}
{
    namespace fs = boost::filesystem ;
    using namespace boost::property_tree ;
    using namespace std ;
    if ( "" == _fileName) {
        _fileName = "etc/sys.info" ;
    }

    lock_guard<recursive_mutex> lck(_mtx) ;

    if ( _mapPtree.end() == _mapPtree.find(_fileName) ) {
        auto &configInfo = _mapPtree[_fileName] ;
        useRobot = false;
        configInfo.useRobot = useRobot;
        configInfo.bDirty = false ;
        //cout << "读入[" << _fileName << "]" << &(configInfo.bDirty) << configInfo.bDirty << endl ;
        configInfo.pt_ptr.reset( new ptree ) ;
        // 读入对应的机器人相关的配置文件， 如果没有则读入逻辑配置文件
        auto &refName = _mapPtree[_fileName].refName ;
        if ( useRobot && "" != _robotName && _fileName != "catch.info") {

            vector<string> str_vec ;
            boost::split(str_vec, _fileName, boost::is_any_of(".")) ;
            if ( str_vec.size() > 1 ) {
                refName = str_vec[0]+ "_" + _robotName + "." + str_vec[1];
            }
        }
        else {
            _mapPtree[_fileName].refName = _fileName ;
        }
        cout << "_mapPtree[_fileName].refName:" << _mapPtree[_fileName].refName << "\n" << flush ;
        try {
            if ( !fs::exists(_mapPtree[_fileName].refName)) {
                if ( !fs::exists(_fileName)) {
                    fs::create_directories(fs::path(_mapPtree[_fileName].refName).remove_filename()) ;
                    ofstream(_mapPtree[_fileName].refName) ;
                }
                else {
                    fs::copy( _fileName, _mapPtree[_fileName].refName) ;
                }
            }
            read_info( _mapPtree[_fileName].refName, *(_mapPtree[_fileName].pt_ptr) ) ;
        }
        catch ( const fs::filesystem_error &e ) {
        }
    }
    _mtx_ptr = &(_mapPtree[_fileName].mtx) ;
    _pt_ptr = _mapPtree[_fileName].pt_ptr ;
    //_mapPtree[_fileName].logicName = _fileName ;
    _connRefChanged = _mapPtree[_fileName].refChanged_sig.connect([this](){
        this->updateRef();}) ;
}

Config::~Config()
{
    lock_guard<recursive_mutex> lck(_mtx) ;
    _connRefChanged.disconnect() ;
    //cout << "析构" << endl ;
    sync() ;
}

/// 更新指向的配置文件[使用时的逻辑名不变，所有逻辑文件指向的配置文件将会发生改变]
/// 如Config("xxx").changeRobot("kuka") 则在所有使用Config("xxx")的地方，实际使用的将是"xxx_kuka"
/// @param [in] robotName 指向新的配置文件
void Config::changeRobot( const string &robotName )
{
    using namespace std ;
    namespace fs = boost::filesystem ;
    lock_guard<recursive_mutex> lck(_mtx) ;
    _robotName = robotName ;
    cout << _robotName << "\n" << flush;
    for ( auto &p : _mapPtree ) {
        const auto &fileName = p.first ;
        const auto &coninfo = p.second;
        if (!coninfo.useRobot)
            continue;

        vector<string> str_vec ;
        boost::split(str_vec, fileName, boost::is_any_of(".")) ;
        if ( str_vec.size() > 1 ) {
            auto refName = str_vec[0]+ "_" + _robotName + "." + str_vec[1];
            if ( !fs::exists(refName) ) {
                fs::copy_file(p.second.refName, refName);
            }
            Config(fileName).changeRef(refName) ;
        }
    }
}

void Config::changeRef( const string &fileName )
{
    using namespace boost::property_tree ;
    namespace fs = boost::filesystem ;
    lock_guard<recursive_mutex> lck(_mtx) ;
    if ( "etc/catch.info" == _fileName ) return ;
    auto &refName = _mapPtree[_fileName].refName ;
    auto &fileInfo = _mapPtree[_fileName] ;
    if ( fileName == refName ) return ;
    sync() ;
    fileInfo.pt_ptr.reset(new ptree) ;
    if ( fs::exists(fileName) ) {
        read_info(fileName, *(fileInfo.pt_ptr)) ;
    }
    refName = fileName ;
    _pt_ptr = fileInfo.pt_ptr ;
    fileInfo.refChanged_sig() ;
    cout << "changeRef:" << refName << endl << flush;
}

void Config::updateRef()
{
    lock_guard<recursive_mutex> lck(_mtx) ;
    sync() ;
    _pt_ptr = _mapPtree[_fileName].pt_ptr ;
    cout << "updateRef:" << _fileName << endl << flush;
}

auto Config::get_child_optional( const boost::property_tree::path &path ) const -> decltype(ConfigInfo::pt_ptr->get_child_optional(path))
{
    return _pt_ptr->get_child_optional(path) ;
}

bool Config::del( const boost::property_tree::path &p )
{
    lock_guard<recursive_mutex> lck(*_mtx_ptr) ;
    auto iter = _pt_ptr->to_iterator(_pt_ptr->find(p.dump())) ;
    if ( _pt_ptr->end() == iter ) return false ;
    _pt_ptr->erase(iter) ;
    _mapPtree[_fileName].bDirty = true ;
    return true ;
}

/// RobotPos
template <>
const RobotPos Config::get<RobotPos>( const boost::property_tree::path &path) const
{
    RobotPos retPos = RobotPos::instance();
    lock_guard<recursive_mutex> lck(_mtx) ;
    retPos.x = _pt_ptr->get<double>(path / "x") ;
    retPos.y = _pt_ptr->get<double>(path / "y") ;
    retPos.z = _pt_ptr->get<double>(path / "z") ;
    retPos.a = _pt_ptr->get<double>(path / "a") ;
    retPos.b = _pt_ptr->get<double>(path / "b") ;
    retPos.c = _pt_ptr->get<double>(path / "c") ;
    retPos.tcp = static_cast<RobotTCP>(_pt_ptr->get<int>(path / "tcp", 0)) ;

    return retPos ;
}

template <>
const RobotPos Config::get<RobotPos>( const boost::property_tree::path &path, RobotPos value) const
{
    lock_guard<recursive_mutex> lck(_mtx) ;
    auto opt = this->get_optional<RobotPos>(path) ;
    if ( opt ) return *opt ;
    return value ;
}

template <>
const boost::optional<RobotPos> Config::get_optional<RobotPos>( const boost::property_tree::path &path ) const
{
    lock_guard<recursive_mutex> lck(_mtx) ;
    if ( _pt_ptr->get_optional<double>(path / "x") ) {
        return boost::optional<RobotPos>(this->get<RobotPos>(path)) ;
    }
    return boost::optional<RobotPos>() ;
}

template <>
void Config::put<RobotPos>( const boost::property_tree::path &p, const RobotPos &pos)
{
    lock_guard<recursive_mutex> lck(_mtx) ;
    _pt_ptr->put(p / "x", pos.x);
    _pt_ptr->put(p / "y", pos.y);
    _pt_ptr->put(p / "z", pos.z);
    _pt_ptr->put(p / "a", pos.a);
    _pt_ptr->put(p / "b", pos.b);
    _pt_ptr->put(p / "c", pos.c);
    _pt_ptr->put(p / "tcp", static_cast<int>(pos.tcp)) ;
    _mapPtree[_fileName].bDirty = true ;
}

/// RobotAxle
template <>
const RobotAxle Config::get<RobotAxle>( const boost::property_tree::path &path) const
{
    RobotAxle retPos ;
    lock_guard<recursive_mutex> lck(_mtx) ;
    retPos.a1 = _pt_ptr->get<double>(path / "a1") ;
    retPos.a2 = _pt_ptr->get<double>(path / "a2") ;
    retPos.a3 = _pt_ptr->get<double>(path / "a3") ;
    retPos.a4 = _pt_ptr->get<double>(path / "a4") ;
    retPos.a5 = _pt_ptr->get<double>(path / "a5") ;
    retPos.a6 = _pt_ptr->get<double>(path / "a6") ;
    return retPos ;
}

template <>
const RobotAxle Config::get<RobotAxle>( const boost::property_tree::path &path, RobotAxle value) const
{
    lock_guard<recursive_mutex> lck(_mtx) ;
    auto opt = this->get_optional<RobotAxle>(path) ;
    if ( opt ) return *opt ;
    return value ;
}

template <>
const boost::optional<RobotAxle> Config::get_optional<RobotAxle>( const boost::property_tree::path &path ) const
{
    lock_guard<recursive_mutex> lck(_mtx) ;
    if ( _pt_ptr->get_optional<double>(path / "a1") ) {
        return boost::optional<RobotAxle>(this->get<RobotAxle>(path)) ;
    }
    return boost::optional<RobotAxle>() ;
}

template <>
void Config::put<RobotAxle>( const boost::property_tree::path &p, const RobotAxle &pos)
{
    lock_guard<recursive_mutex> lck(_mtx) ;
    _pt_ptr->put(p / "a1", pos.a1);
    _pt_ptr->put(p / "a2", pos.a2);
    _pt_ptr->put(p / "a3", pos.a3);
    _pt_ptr->put(p / "a4", pos.a4);
    _pt_ptr->put(p / "a5", pos.a5);
    _pt_ptr->put(p / "a6", pos.a6);
    _mapPtree[_fileName].bDirty = true ;
}


/// point3d
template <>
const point3d<double> Config::get<point3d<double>>( const boost::property_tree::path &path) const
{
    point3d<double> retPos ;
    lock_guard<recursive_mutex> lck(_mtx) ;
    retPos._x = _pt_ptr->get<double>(path / "x") ;
    retPos._y = _pt_ptr->get<double>(path / "y") ;
    retPos._z = _pt_ptr->get<double>(path / "z") ;
    return retPos ;
}

/// matrix4f
template<>
const Eigen::Matrix4f Config::get<Eigen::Matrix4f>( const boost::property_tree::path &p ) const
{
    Eigen::Matrix4f m ;
    lock_guard<recursive_mutex> lck(_mtx) ;
    for ( int i = 0; i < m.rows(); ++i ) {
        for ( int j = 0; j < m.cols(); ++j ) {
            m(i, j) = _pt_ptr->get<float>(p / to_string(i*m.cols() + j).c_str()) ;
        }
    }
    return m ;
}

template <>
const boost::optional<Eigen::Matrix4f> Config::get_optional( const boost::property_tree::path &path ) const
{
    lock_guard<recursive_mutex> lck(_mtx) ;
    if ( _pt_ptr->get_optional<float>(path / "0") ) {
        return boost::optional<Eigen::Matrix4f>(this->get<Eigen::Matrix4f>(path)) ;
    }
    return boost::optional<Eigen::Matrix4f>() ;
}

template<>
void Config::put<Eigen::Matrix4f>( const boost::property_tree::path &p, const Eigen::Matrix4f &m )
{
    lock_guard<recursive_mutex> lck(_mtx) ;
    for ( int i = 0; i < m.rows(); ++i ) {
        for ( int j = 0; j < m.cols(); ++j ) {
            _pt_ptr->put(p / to_string(i*m.cols() + j).c_str(), m(i, j)) ;
        }
    }
    _mapPtree[_fileName].bDirty = true ;
}

/// matrix4d
template<>
const Eigen::Matrix4d Config::get<Eigen::Matrix4d>( const boost::property_tree::path &p ) const
{
    Eigen::Matrix4d m ;
    lock_guard<recursive_mutex> lck(_mtx) ;
    for ( int i = 0; i < m.rows(); ++i ) {
        for ( int j = 0; j < m.cols(); ++j ) {
            m(i, j) = _pt_ptr->get<float>(p / to_string(i*m.cols() + j).c_str()) ;
        }
    }
    return m ;
}

template <>
const boost::optional<Eigen::Matrix4d> Config::get_optional( const boost::property_tree::path &path ) const
{
    lock_guard<recursive_mutex> lck(_mtx) ;
    if ( _pt_ptr->get_optional<double>(path / "0") ) {
        return boost::optional<Eigen::Matrix4d>(this->get<Eigen::Matrix4d>(path)) ;
    }
    return boost::optional<Eigen::Matrix4d>() ;
}

template<>
void Config::put<Eigen::Matrix4d>( const boost::property_tree::path &p, const Eigen::Matrix4d &m )
{
    lock_guard<recursive_mutex> lck(_mtx) ;
    for ( int i = 0; i < m.rows(); ++i ) {
        for ( int j = 0; j < m.cols(); ++j ) {
            _pt_ptr->put(p / to_string(i*m.cols() + j).c_str(), m(i, j)) ;
        }
    }
    _mapPtree[_fileName].bDirty = true ;
}

/// PosInfo
template<>
void Config::put<PosInfo>( const boost::property_tree::path &p, const PosInfo &posInfo )
{
    lock_guard<recursive_mutex> lck(_mtx) ;
    put(p, posInfo.pos) ;
    put(p, posInfo.axle) ;
    _pt_ptr->put(p / "posType", static_cast<int>(posInfo.posType)) ;
    _pt_ptr->put(p / "positionerRotate", posInfo.positionerRotate) ;
    _pt_ptr->put(p / "bEnabled", posInfo.bEnabled) ;
    _pt_ptr->put(p / "strSeamAlgo", posInfo.strSeamAlgo ) ;
}

template<>
const PosInfo Config::get<PosInfo>( const boost::property_tree::path &p ) const
{
    PosInfo posInfo ;
    lock_guard<recursive_mutex> lck(_mtx) ;
    posInfo.pos = get<RobotPos>(p) ;
    posInfo.axle= get<RobotAxle>(p) ;
    posInfo.posType = static_cast<PosType>(_pt_ptr->get<int>(p / "posType")) ;
    posInfo.positionerRotate = _pt_ptr->get<int>(p / "positionerRotate") ;
    posInfo.bEnabled = _pt_ptr->get<bool>(p / "bEnabled") ;
    posInfo.strSeamAlgo = _pt_ptr->get<string>(p / "strSeamAlgo", "" ) ;
    return posInfo ;
}

/// UV
template<>
void Config::put<UV>( const boost::property_tree::path &p, const UV &uv )
{
    _pt_ptr->put(p / "u", uv.u) ;
    _pt_ptr->put(p / "v", uv.v) ;
}

template<>
const UV Config::get<UV>( const boost::property_tree::path &p ) const
{
    UV uv ;
    uv.u = _pt_ptr->get<double>(p / "u") ;
    uv.v = _pt_ptr->get<double>(p / "v") ;
    return uv ;
}
//XZ
template<>
void Config::put<XZ>( const boost::property_tree::path &p, const XZ &xz )
{
    _pt_ptr->put(p / "x", xz.x) ;
    _pt_ptr->put(p / "z", xz.z) ;
}

template<>
const XZ Config::get<XZ>( const boost::property_tree::path &p ) const
{
    XZ xz ;
    xz.x = _pt_ptr->get<double>(p / "x") ;
    xz.z = _pt_ptr->get<double>(p / "z") ;
    return xz ;
}

void Config::sync()
{
    lock_guard<recursive_mutex> lck(_mtx) ;
    if ( !_bWritten ) return ;
    auto &refName = _mapPtree[_fileName].refName ;
    //cout << "sync refName:" << refName << endl << flush ;
    if ( _mapPtree[_fileName].bDirty ) {
        //cout << "写入[" << refName << "]" << &(_mapPtree[_fileName].bDirty) << endl << flush ;
        boost::property_tree::write_info( refName, *_pt_ptr ) ;
        //cout << "写入配置文件" << endl ;
        _mapPtree[_fileName].bDirty = false ;
    }
}

void Config::clear()
{
    lock_guard<recursive_mutex> lck(_mtx) ;
    _pt_ptr->clear() ;
}

