/**
 * @file <argos3/plugins/robots/qupa/simulator/qupa_distance_scanner_default_actuator.h>
 *
 * @author Jose Santos
 * Gabriel Madroñero
 * - <emails >
 */

#ifndef QUPA_DISTANCE_SCANNER_DEFAULT_ACTUATOR_H
#define QUPA_DISTANCE_SCANNER_DEFAULT_ACTUATOR_H

#include <string>
#include <map>

namespace argos {
   class CQupaDistanceScannerDefaultActuator;
}

#include "../control_interface/ci_qupa_distance_scanner_actuator.h"
#include "qupa_entity.h"
#include "qupa_distance_scanner_equipped_entity.h"
#include <argos3/core/simulator/actuator.h>

namespace argos {

   class CQupaDistanceScannerDefaultActuator : public CSimulatedActuator,
                                                  public CCI_QupaDistanceScannerActuator {

   public:

  	  static const Real RPM_TO_RADIANS_PER_SEC;

      CQupaDistanceScannerDefaultActuator();
      virtual ~CQupaDistanceScannerDefaultActuator() {}

      virtual void SetRobot(CComposableEntity& c_entity);

      virtual void SetAngle(const CRadians& c_angle);
      virtual void SetRPM(Real f_rpm);

      virtual void Enable();
      virtual void Disable();

      virtual void Update();
      virtual void Reset();

   private:

      CQupaDistanceScannerEquippedEntity* m_pcDistanceScannerEquippedEntity;

      CRadians m_cDesiredRotation;
      Real     m_fDesiredRotationSpeed;
      UInt8    m_unDesiredMode;

   };

}

#endif
