/**
 * @file <argos3/plugins/robots/qupa/control_interface/ci_qupa_turret_encoder_sensor.h>
 *
 * @brief This file provides the common interface definition of the qupa turret encoder
 * sensor. The sensor provides a measure of the rotation of the turret.
 *
 * The turret rotation is expressed in radians [-pi,pi], counter-clockwise positive when
 * looking from above.
 *
 * @author Jose Santos
 * Gabriel Madroñero 
 * - <emails >
 */

#ifndef CCI_QUPA_TURRET_ENCODER_SENSOR_H
#define CCI_QUPA_TURRET_ENCODER_SENSOR_H

namespace argos {
   class CCI_QupaTurretEncoderSensor;
}

#include <argos3/core/control_interface/ci_sensor.h>
#include <argos3/core/utility/math/vector2.h>

namespace argos {

   class CCI_QupaTurretEncoderSensor : virtual public CCI_Sensor {

   public:

      static const CRange<CRadians> ANGULAR_RANGE;

   public:

      virtual ~CCI_QupaTurretEncoderSensor() {}

      const CRadians& GetRotation() const;

#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State* pt_lua_state);

      virtual void ReadingsToLuaState(lua_State* pt_lua_state);
#endif

   protected:

      CRadians m_cRotation;

      friend class CCI_QupaTurretActuator;
      friend class CRealQupaTurretActuator;

   };

}

#endif
