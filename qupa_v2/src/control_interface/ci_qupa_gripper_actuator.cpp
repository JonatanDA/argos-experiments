/**
 * @file <argos3/plugins/robots/qupa/control_interface/ci_qupa_gripper_actuator.cpp>
 *
 * @author Jose Santos
 * Gabriel Madroñero 
 * - <emails >
 */

#include "ci_qupa_gripper_actuator.h"

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#endif

namespace argos {

  /****************************************/
  /****************************************/
   
  CRadians CCI_QupaGripperActuator::LOCKED_NEGATIVE(-ARGOS_PI * 0.5f);
  CRadians CCI_QupaGripperActuator::LOCKED_POSITIVE(ARGOS_PI * 0.5f);
  CRadians CCI_QupaGripperActuator::UNLOCKED(0.0f);

  CRange<CRadians> APERTURE_RANGE(CCI_QupaGripperActuator::LOCKED_NEGATIVE,
				  CCI_QupaGripperActuator::LOCKED_POSITIVE);
   
  /****************************************/
  /****************************************/

#ifdef ARGOS_WITH_LUA
  /*
   * The stack must have no values
   */
  int LuaGripperLockPositive(lua_State* pt_lua_state) {
    /* Get wheel speed from stack */
    if(lua_gettop(pt_lua_state) != 0) {
      return luaL_error(pt_lua_state, "robot.gripper.lock_positive() expects no arguments");
    }
    /* Perform action */
    CLuaUtility::GetDeviceInstance<CCI_QupaGripperActuator>(pt_lua_state, "gripper")->LockPositive();
    return 0;
  }

  /*
   * The stack must have no values
   */
  int LuaGripperLockNegative(lua_State* pt_lua_state) {
    /* Get wheel speed from stack */
    if(lua_gettop(pt_lua_state) != 0) {
      return luaL_error(pt_lua_state, "robot.gripper.lock_negative() expects no arguments");
    }
    /* Perform action */
    CLuaUtility::GetDeviceInstance<CCI_QupaGripperActuator>(pt_lua_state, "gripper")->LockNegative();
    return 0;
  }

  /*
   * The stack must have no values
   */
  int LuaGripperUnlock(lua_State* pt_lua_state) {
    /* Get wheel speed from stack */
    if(lua_gettop(pt_lua_state) != 0) {
      return luaL_error(pt_lua_state, "robot.gripper.unlock() expects no arguments");
    }
    /* Perform action */
    CLuaUtility::GetDeviceInstance<CCI_QupaGripperActuator>(pt_lua_state, "gripper")->Unlock();
    return 0;
  }
#endif

  /****************************************/
  /****************************************/

  CCI_QupaGripperActuator::CCI_QupaGripperActuator() :
    m_cAperture(UNLOCKED) {}

  /****************************************/
  /****************************************/

  void CCI_QupaGripperActuator::SetAperture(const CRadians& c_aperture) {
    m_cAperture = c_aperture;
    m_cAperture.SignedNormalize();
    APERTURE_RANGE.TruncValue(m_cAperture);
  }

  /****************************************/
  /****************************************/

  void CCI_QupaGripperActuator::LockPositive() {
    SetAperture(LOCKED_POSITIVE);
  }

  /****************************************/
  /****************************************/

  void CCI_QupaGripperActuator::LockNegative() {
    SetAperture(LOCKED_NEGATIVE);
  }

  /****************************************/
  /****************************************/

  void CCI_QupaGripperActuator::Unlock() {
    SetAperture(UNLOCKED);
  }

  /****************************************/
  /****************************************/

#ifdef ARGOS_WITH_LUA
  void CCI_QupaGripperActuator::CreateLuaState(lua_State* pt_lua_state) {
    CLuaUtility::OpenRobotStateTable (pt_lua_state, "gripper"                               );
    CLuaUtility::AddToTable          (pt_lua_state, "_instance",     this                   );
    CLuaUtility::AddToTable          (pt_lua_state, "lock_positive", &LuaGripperLockPositive);
    CLuaUtility::AddToTable          (pt_lua_state, "lock_negative", &LuaGripperLockNegative);
    CLuaUtility::AddToTable          (pt_lua_state, "unlock",        &LuaGripperUnlock      );
    CLuaUtility::CloseRobotStateTable(pt_lua_state                                          );
  }
#endif

  /****************************************/
  /****************************************/

}
