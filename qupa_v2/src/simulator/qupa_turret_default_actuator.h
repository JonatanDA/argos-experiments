/**
 * @file <argos3/plugins/robots/qupa/simulator/qupa_turret_default_actuator.h>
 *
 * @author Jose Santos
 * Gabriel Madroñero 
 * - <emails >
 */

#ifndef QUPA_TURRET_DEFAULT_ACTUATOR_H
#define QUPA_TURRET_DEFAULT_ACTUATOR_H

#include <string>
#include <map>

namespace argos {
   class CQupaTurretDefaultActuator;
}

#include "../control_interface/ci_qupa_turret_actuator.h"
#include "qupa_entity.h"
#include "qupa_turret_entity.h"
#include <argos3/core/simulator/actuator.h>

namespace argos {

   class CQupaTurretDefaultActuator : public CSimulatedActuator,
                                         public CCI_QupaTurretActuator {

   public:

      CQupaTurretDefaultActuator();
      virtual ~CQupaTurretDefaultActuator() {}

      virtual void SetRobot(CComposableEntity& c_entity);

      virtual void SetRotation(const CRadians& c_angle);
      virtual void SetRotationSpeed(SInt32 n_speed_pulses);
      virtual void SetMode(ETurretModes e_mode);

      virtual void Update();
      virtual void Reset();

   private:

      CQupaTurretEntity* m_pcTurretEntity;
      UInt32 m_unDesiredMode;

   };

}

#endif
