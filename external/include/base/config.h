#ifndef _CONFIG_H
#define _CONFIG_H
#include <string>
#include <map>
#include <mutex>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <boost/filesystem.hpp>
#include <boost/signals2/signal.hpp>
#include <Eigen/Dense>
#include "walgo/point.h"
#include "base/data_types.h"

using namespace walgo;

class Config
{
   struct ConfigInfo
   {
      std::shared_ptr<boost::property_tree::ptree> pt_ptr ;
      bool bDirty ;
      //std::string logicName ;
      std::string refName ;
      std::recursive_mutex mtx ;
      boost::signals2::signal<void ()> refChanged_sig ;
      bool useRobot;
   } ;

public:
   Config( const decltype(ConfigInfo::pt_ptr->get_child_optional("")) &pt ) ;

   Config( const char *pstr = "", bool bWritten = true , bool useRobot = true) ;

   Config( const std::string &fileName, bool bWritten = true, bool useRobot = true) ;

   /// 更新指向的配置文件[使用时的逻辑名不变，所有逻辑文件指向的配置文件将会发生改变]
   /// 如Config("sys").changeRef("sys_kuka") 则在所有使用Config("sys")的地方，实际使用的将是"sys_kuka"
   /// @param [in] fileName 指向新的配置文件
   void changeRef( const std::string &fileName ) ;

   /// 更新指向的配置文件[使用时的逻辑名不变，所有逻辑文件指向的配置文件将会发生改变]
   /// 如Config("xxx").changeRobot("kuka") 则在所有使用Config("xxx")的地方，实际使用的将是"xxx_kuka"
   /// @param [in] robotName 指向新的配置文件
   static void changeRobot( const std::string &robotName ) ;

   ~Config() ;

   bool del( const boost::property_tree::path &p ) ;

   auto get_child_optional( const boost::property_tree::path &path ) const -> decltype(ConfigInfo::pt_ptr->get_child_optional(path)) ;

   // 获取节点数据
   template <typename T>
   const T get( const boost::property_tree::path &path) const {
      std::lock_guard<std::recursive_mutex> lck(*_mtx_ptr) ;
      //std::cout << "获取:" << _mapPtree[_fileName].refName << std::endl ;
      return _pt_ptr->template get<T>(path) ;
   }

   // 获取节点数据, 如果失败则返回默认值
   template <typename T>
   const T get( const boost::property_tree::path &path, T value) const {
      std::lock_guard<std::recursive_mutex> lck(*_mtx_ptr) ;
      return _pt_ptr->template get<T>(path, value) ;
   }

   template <typename T>
   const boost::optional<T> get_optional( const boost::property_tree::path &path ) const {
      std::lock_guard<std::recursive_mutex> lck(*_mtx_ptr) ;
      return _pt_ptr->template get_optional<T>(path) ;
   }

   template <typename T>
   void put( const boost::property_tree::path &path, const T &value) {
      std::lock_guard<std::recursive_mutex> lck(*_mtx_ptr) ;
      _pt_ptr->put( path, value ) ;
      _mapPtree[_fileName].bDirty = true ;
   }

   void sync() ;

   void clear() ;

   template <typename ostream>
   friend ostream &operator<<( ostream &os, const Config &config) {
      boost::property_tree::write_info(os, *(config._pt_ptr)) ;
      return os ;
   }
private:
   std::string _fileName ;
   std::shared_ptr<boost::property_tree::ptree> _pt_ptr ;
   static std::map<std::string, ConfigInfo> _mapPtree ;
   static std::recursive_mutex _mtx ;
   static std::string _robotName ;
   std::recursive_mutex *_mtx_ptr ;
   boost::signals2::connection _connRefChanged ;
   size_t _countRef ;
   bool _bWritten ;
private:
   void updateRef() ;
} ;

/// RobotPos
template<>
const RobotPos Config::get<RobotPos>( const boost::property_tree::path &path) const ;

template <>
const RobotPos Config::get<RobotPos>( const boost::property_tree::path &path, RobotPos value) const ;

template <>
const boost::optional<RobotPos> Config::get_optional<RobotPos>( const boost::property_tree::path &path ) const ;

template <>
void Config::put<RobotPos>( const boost::property_tree::path &path, const RobotPos &value) ;

/// RobotAxle
template<>
const RobotAxle Config::get<RobotAxle>( const boost::property_tree::path &path) const ;

template <>
const RobotAxle Config::get<RobotAxle>( const boost::property_tree::path &path, RobotAxle value) const ;

template <>
const boost::optional<RobotAxle> Config::get_optional<RobotAxle>( const boost::property_tree::path &path ) const ;

template <>
void Config::put<RobotAxle>( const boost::property_tree::path &path, const RobotAxle &value) ;

/// point3d

template <>
const point3d<double> Config::get<point3d<double>>( const boost::property_tree::path &path) const ;

/// matrix4f
template<>
const boost::optional<Eigen::Matrix4f> Config::get_optional<Eigen::Matrix4f>( const boost::property_tree::path &p ) const ;

template<>
const Eigen::Matrix4f Config::get<Eigen::Matrix4f>( const boost::property_tree::path &p ) const ;

template<>
void Config::put<Eigen::Matrix4f>( const boost::property_tree::path &p, const Eigen::Matrix4f &m ) ;

/// Matrix4d
template<>
const boost::optional<Eigen::Matrix4d> Config::get_optional<Eigen::Matrix4d>( const boost::property_tree::path &p ) const ;

template<>
const Eigen::Matrix4d Config::get<Eigen::Matrix4d>( const boost::property_tree::path &p ) const ;

template<>
void Config::put<Eigen::Matrix4d>( const boost::property_tree::path &p, const Eigen::Matrix4d &m ) ;

/// PosInfo
template<>
void Config::put<PosInfo>( const boost::property_tree::path &p, const PosInfo &posInfo ) ;

template<>
const PosInfo Config::get<PosInfo>( const boost::property_tree::path &p ) const ;

/// UV
template<>
void Config::put<UV>( const boost::property_tree::path &p, const UV &uv ) ;

template<>
const UV Config::get<UV>( const boost::property_tree::path &p ) const ;

/// XZ
template<>
void Config::put<XZ>( const boost::property_tree::path &p, const XZ &xz ) ;

template<>
const XZ Config::get<XZ>( const boost::property_tree::path &p ) const ;

#endif

