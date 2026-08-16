/**
 * @file <argos3/plugins/robots/qupa/simulator/qupa_proximity_default_sensor.h>
 *
 * @author Jose Santos
 * Gabriel Madroñero 
 * - <emails >
 */

#ifndef QUPA_PROXIMITY_DEFAULT_SENSOR_H
#define QUPA_PROXIMITY_DEFAULT_SENSOR_H

#include <string>
#include <map>

namespace argos {
   class CQupaProximityDefaultSensor;
   class CComposableEntity;                 // <-- añadido
}

#include "../control_interface/ci_qupa_proximity_sensor.h"
#include <argos3/plugins/robots/generic/simulator/proximity_default_sensor.h>
#include <argos3/core/simulator/sensor.h>

namespace argos {

   class CQupaProximityDefaultSensor : public CCI_QupaProximitySensor,
                                       public CSimulatedSensor {

   public:
      CQupaProximityDefaultSensor();
      virtual ~CQupaProximityDefaultSensor();

      virtual void SetRobot(CComposableEntity& c_entity);
      virtual void Init(TConfigurationNode& t_tree);
      virtual void Update();
      virtual void Reset();

      virtual void Enable();               // se queda
      virtual void Disable();              // se queda
      bool IsDisabled() const;             // <-- añadido

   private:
      CProximityDefaultSensor* m_pcProximityImpl;

   protected:
      bool m_bEnabled = true;              // <-- añadido
   };

}

#endif
