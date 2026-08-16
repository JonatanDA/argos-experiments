/**
 * @file <argos3/plugins/robots/qupa/simulator/qupa_base_ground_rotzonly_sensor.h>
 *
 * @author Jose Santos
 * Gabriel Madroñero 
 * - <emails >
 */

#ifndef QUPA_BASE_GROUND_ROTZONLY_SENSOR_H
#define QUPA_BASE_GROUND_ROTZONLY_SENSOR_H

#include <string>
#include <map>

namespace argos {
   class CQupaBaseGroundRotZOnlySensor;
   class CGroundSensorEquippedEntity;
   class CFloorEntity;
   class CEmbodiedEntity;
   class CComposableEntity; 
}

#include "../control_interface/ci_qupa_base_ground_sensor.h"
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/sensor.h>

namespace argos {

   class CQupaBaseGroundRotZOnlySensor : public CCI_QupaBaseGroundSensor,
                                            public CSimulatedSensor {
      
   public:

      CQupaBaseGroundRotZOnlySensor();

      virtual ~CQupaBaseGroundRotZOnlySensor() {}

      virtual void SetRobot(CComposableEntity& c_entity);

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Update();

      virtual void Reset();

      /* ===== Compatibilidad con ARGoS sin Enable/Disable en CSimulatedSensor ===== */
      void Enable();                 // <-- añadido
      void Disable();                // <-- añadido
      bool IsDisabled() const;       // <-- añadido

   protected:

      /** Reference to embodied entity associated to this sensor */
      CEmbodiedEntity* m_pcEmbodiedEntity;

      /** Reference to floor entity */
      CFloorEntity* m_pcFloorEntity;

      /** Reference to ground sensor equipped entity associated to this sensor */
      CGroundSensorEquippedEntity* m_pcGroundSensorEntity;

      /** Random number generator */
      CRandom::CRNG* m_pcRNG;

      /** Whether to add noise or not */
      bool m_bAddNoise;

      /** Noise range */
      CRange<Real> m_cNoiseRange;

      /** Reference to the space */
      CSpace& m_cSpace;

      /** Estado de habilitación local del sensor (compatibilidad) */
      bool m_bEnabled = true;        // <-- añadido
   };

}

#endif
