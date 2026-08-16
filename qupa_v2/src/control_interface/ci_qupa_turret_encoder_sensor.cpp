/**
 * @file <argos3/plugins/robots/qupa/control_interface/ci_qupa_turret_encoder_sensor.cpp>
 *
 * @author Jose Santos
 * Gabriel Madroñero 
 * - <emails >
 */

#include "ci_qupa_turret_encoder_sensor.h"

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#endif

namespace argos {
   
   /****************************************/
    /****************************************/
   
   const CRange<CRadians> CCI_QupaTurretEncoderSensor::ANGULAR_RANGE(CRadians(-ARGOS_PI), CRadians(ARGOS_PI));
   
   /****************************************/
   /****************************************/
   
   const CRadians& CCI_QupaTurretEncoderSensor::GetRotation() const {
     return m_cRotation;
   }

   /****************************************/
   /****************************************/
   
#ifdef ARGOS_WITH_LUA
   void CCI_QupaTurretEncoderSensor::CreateLuaState(lua_State* pt_lua_state) {
      CLuaUtility::OpenRobotStateTable(pt_lua_state, "turret");
      CLuaUtility::AddToTable(pt_lua_state, "rotation",  m_cRotation);
      CLuaUtility::CloseRobotStateTable(pt_lua_state);
   }
#endif

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_QupaTurretEncoderSensor::ReadingsToLuaState(lua_State* pt_lua_state) {
      lua_getfield(pt_lua_state, -1, "turret");
      lua_pushnumber(pt_lua_state, m_cRotation.GetValue());
      lua_setfield(pt_lua_state, -2, "rotation");
      lua_pop(pt_lua_state, 1);
   }
#endif


   /****************************************/
   /****************************************/

}
