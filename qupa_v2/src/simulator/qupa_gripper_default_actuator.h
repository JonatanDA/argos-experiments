/**
 * @file <argos3/plugins/robots/qupa/simulator/qupa_gripper_default_actuator.h>
 *
 * @author Jose Santos
 * Gabriel Madroñero 
 * - <emails >
 */

#ifndef QUPA_GRIPPER_DEFAULT_ACTUATOR_H
#define QUPA_GRIPPER_DEFAULT_ACTUATOR_H

#include <string>
#include <map>

namespace argos {
   class CQupaGripperDefaultActuator;
}

#include "../control_interface/ci_qupa_gripper_actuator.h"
#include <argos3/plugins/simulator/entities/gripper_equipped_entity.h>
#include <argos3/core/simulator/actuator.h>

namespace argos {

   class CQupaGripperDefaultActuator : public CSimulatedActuator,
                                          public CCI_QupaGripperActuator {

   public:

      CQupaGripperDefaultActuator();

      virtual ~CQupaGripperDefaultActuator() {}

      virtual void SetRobot(CComposableEntity& c_entity);

      virtual void Update();
      virtual void Reset();

      virtual void EnableCheckForObjectGrippedRoutine() {}
      virtual void DisableCheckForObjectGrippedRoutine() {}

   private:

      CGripperEquippedEntity* m_pcGripperEquippedEntity;

   };

}

#endif
