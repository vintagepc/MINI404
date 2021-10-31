/*
	ArgHelper.h - Convenience class for turning the -append
    CLI arg into things we can use within the context of Mini404

	Copyright 2021 VintagePC <https://github.com/vintagepc/>

 	This file is part of Mini404.

	Mini404 is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	Mini404 is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with Mini404.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "ArgHelper.h"
#include "ScriptHost.h"
#include <iostream>
#include <sstream>
#include "../opengl/Config.h"
#include "../opengl/KeyController.h"
#include "../opengl/PrintVisualType.h"

void ArgHelper::SetArgs(std::string strArgs){
	std::istringstream s(strArgs);
	std::string strArg;
	while (std::getline(s,strArg, ',')) {
		auto iSplit = strArg.find('=');
		if (iSplit!=std::string::npos)
		{
			m_map.insert({strArg.substr(0,iSplit), strArg.substr(iSplit+1, strArg.length())});
		}
		else 
		{
			m_map.insert({strArg,"true"});
		}
	}
}

bool ArgHelper::Parse() {
	if (m_map.count("scripthelp"))
	{
		ScriptHost::PrintScriptHelp(false);
		return false;
	} else if (m_map.count("scripthelpmd")) {
        ScriptHost::PrintScriptHelp(true);
        return false;
    } else if (m_map.count("keyhelp")) {
		KeyController::GetController().PrintKeys(false);
		return false;
	} else if (m_map.count("keyhelpmd")) {
		KeyController::GetController().PrintKeys(true);
		return false;
	}

	if (m_map.count("colour-extrusion")) {
		Config::Get().SetColourE(true);
	}

	if (m_map.count("extrusion")) {
		auto &strValue = m_map.at("extrusion");
		auto eType = PrintVisualType::LINE;
		//Line|Quad_Avg|Quad_HR|Tube_Avg|Tube_HR
		if (strValue=="Quad_Avg") {
			eType = PrintVisualType::QUAD;
		} else if (strValue=="Quad_HR") {
			eType = PrintVisualType::QUAD_HIGHRES;
		}else if (strValue=="Tube_Avg") {
			eType = PrintVisualType::TUBE;
		}else if (strValue=="Tube_HR") {
			eType = PrintVisualType::TUBE_HIGHRES;
		}
		Config::Get().SetExtrusionMode(eType);
	}

	return true;
}

bool ArgHelper::IsArg(std::string strArg)
{
	return m_map.count(strArg)>0;
}

const char* ArgHelper::GetString(std::string strArg){
	if (m_map.count(strArg)){
		return m_map.at(strArg).c_str();
	} else {
		return nullptr;
	}
}

extern "C" {

	// Sets the arguments for future use. 
	extern void arghelper_setargs(const char* args){
		ArgHelper::Get().SetArgs(args);
	}

	// Processes the args after startup. Return code indicates whether
	// the machine should continue execution (T) or quit (F)
	// e.g. after some kind of help output.  
	extern bool arghelper_parseargs(){
		return ArgHelper::Get().Parse();
	}

	extern bool arghelper_is_arg(const char* arg)
	{
		return ArgHelper::Get().IsArg(arg);
	}

	extern const char* arghelper_get_string(const char* arg)
	{
		return ArgHelper::Get().GetString(arg);
	}

}
