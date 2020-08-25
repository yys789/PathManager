/*
 * utils.h
 *
 *  Created on: Apr 10, 2016
 *      Author: zhian
 */

#ifndef UTILS_H_2
#define UTILS_H_2

#include <map>
#include <string>

bool ReadConfig(const std::string& fname, std::map<std::string, int>& config);

bool GetConfigEntry(const std::map<std::string, int>& config, const char* key, int& val);

bool GetConfigEntry(const std::map<std::string, int>& config, std::string& key, int& val);

bool MyReadConfig(const std::string& fname, const std::string& motorTypeSeamNo, std::map<std::string,int>& config);

#endif /* UTILS_H_ */
