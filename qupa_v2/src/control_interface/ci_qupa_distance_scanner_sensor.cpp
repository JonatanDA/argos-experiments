/**
 * @file <argos3/plugins/robots/qupa/control_interface/ci_qupa_distance_scanner_sensor.cpp>
 *
 * @author Jose Santos
 * Gabriel Madroñero 
 * - <emails >
 */

#include "ci_qupa_distance_scanner_sensor.h"

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#endif

namespace argos {
   
  /****************************************/
  /****************************************/

  const CCI_QupaDistanceScannerSensor::TReadingsMap& CCI_QupaDistanceScannerSensor::GetReadingsMap(){
    return m_tReadingsMap;
  }

  /****************************************/
  /****************************************/

  const CCI_QupaDistanceScannerSensor::TReadingsMap& CCI_QupaDistanceScannerSensor::GetShortReadingsMap(){
    return m_tShortReadingsMap;
  }

  /****************************************/
  /****************************************/

  const CCI_QupaDistanceScannerSensor::TReadingsMap& CCI_QupaDistanceScannerSensor::GetLongReadingsMap(){
    return m_tLongReadingsMap;
  }

   
#ifdef ARGOS_WITH_LUA
  void CCI_QupaDistanceScannerSensor::CreateLuaState(lua_State* pt_lua_state) {
    CLuaUtility::OpenRobotStateTable (pt_lua_state, "distance_scanner");
    CLuaUtility::StartTable          (pt_lua_state, "short_range"     );
    CLuaUtility::EndTable            (pt_lua_state                    );
    CLuaUtility::StartTable          (pt_lua_state, "long_range"      );
    CLuaUtility::EndTable            (pt_lua_state                    );
    CLuaUtility::CloseRobotStateTable(pt_lua_state                    );
  }
#endif

  /****************************************/
  /****************************************/

#ifdef ARGOS_WITH_LUA
  void CCI_QupaDistanceScannerSensor::ReadingsToLuaState(lua_State* pt_lua_state) {
    lua_getfield(pt_lua_state, -1, "distance_scanner");
    CLuaUtility::StartTable(pt_lua_state, "short_range");
    int nCounter = 1;
    for(TReadingsMap::iterator it = m_tShortReadingsMap.begin();
	it != m_tShortReadingsMap.end(); ++it) {
      CLuaUtility::StartTable(pt_lua_state, nCounter);
      CLuaUtility::AddToTable(pt_lua_state, "angle",    it->first);
      CLuaUtility::AddToTable(pt_lua_state, "distance", it->second);
      CLuaUtility::EndTable(pt_lua_state);
      ++nCounter;
    }
    CLuaUtility::EndTable(pt_lua_state);
    CLuaUtility::StartTable(pt_lua_state, "long_range");
    nCounter = 1;
    for(TReadingsMap::iterator it = m_tLongReadingsMap.begin();
	it != m_tLongReadingsMap.end(); ++it) {
      CLuaUtility::StartTable(pt_lua_state, nCounter);
      CLuaUtility::AddToTable(pt_lua_state, "angle",    it->first);
      CLuaUtility::AddToTable(pt_lua_state, "distance", it->second);
      CLuaUtility::EndTable(pt_lua_state);
      ++nCounter;
    }
    CLuaUtility::EndTable(pt_lua_state);
    lua_pop(pt_lua_state, 1);
  }
#endif


  /****************************************/
  /****************************************/

}
