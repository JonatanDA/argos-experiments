/**
 * @file <argos3/plugins/robots/qupa/simulator/qupa_turret_encoder_default_sensor.h>
 *
 * @author Jose Santos
 * Gabriel Madroñero 
 * - <emails >
 */

#ifndef QUPA_TURRET_ENCODER_DEFAULT_SENSOR_H
#define QUPA_TURRET_ENCODER_DEFAULT_SENSOR_H

#include <string>
#include <map>

namespace argos {
   class CQupaTurretEncoderDefaultSensor;
   class CComposableEntity;            // <-- añadido (forward declare)
}

#include "../control_interface/ci_qupa_turret_encoder_sensor.h"
#include "qupa_turret_encoder_default_sensor.h"   // (opcional quitar, pero no estorba)
#include "qupa_turret_entity.h"
#include <argos3/core/simulator/sensor.h>

namespace argos {

   class CQupaTurretEncoderDefaultSensor : public CCI_QupaTurretEncoderSensor,
                                           public CSimulatedSensor {

   public:
      CQupaTurretEncoderDefaultSensor();
      virtual ~CQupaTurretEncoderDefaultSensor() {}

      virtual void SetRobot(CComposableEntity& c_entity);
      virtual void Update();
      virtual void Reset();

      virtual void Enable();     // ya existía
      virtual void Disable();    // ya existía
      bool IsDisabled() const;   // <-- añadido

   private:
      CQupaTurretEntity* m_pcTurretEntity;

   protected:
      bool m_bEnabled = true;    // <-- añadido
   };

}

#endif
