/**
 * @file <argos3/plugins/robots/qupa/simulator/qupa_light_rotzonly_sensor.h>
 *
 * @author Jose Santos
 * Gabriel Madroñero 
 * - <emails >
 */

#ifndef QUPA_LIGHT_ROTZONLY_SENSOR_H
#define QUPA_LIGHT_ROTZONLY_SENSOR_H

#include <string>
#include <map>

namespace argos {
   class CQupaLightRotZOnlySensor;
   class CLightSensorEquippedEntity;
}

#include  "../control_interface/ci_qupa_light_sensor.h"
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/sensor.h>

namespace argos {

   class CQupaLightRotZOnlySensor : public CSimulatedSensor,
                                      public CCI_QupaLightSensor {

   public:

      CQupaLightRotZOnlySensor();

      virtual ~CQupaLightRotZOnlySensor() {}

      virtual void SetRobot(CComposableEntity& c_entity);

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Update();

      virtual void Reset();

      /* Compat ARGoS beta56 */
      void Enable();
      void Disable();
      bool IsDisabled() const;

      /**
       * Returns true if the rays must be shown in the GUI.
       * @return true if the rays must be shown in the GUI.
       */
      inline bool IsShowRays() {
         return m_bShowRays;
      }

      /**
       * Sets whether or not the rays must be shown in the GUI.
       * @param b_show_rays true if the rays must be shown, false otherwise
       */
      inline void SetShowRays(bool b_show_rays) {
         m_bShowRays = b_show_rays;
      }

   protected:

      /** Reference to embodied entity associated to this sensor */
      CEmbodiedEntity* m_pcEmbodiedEntity;

      /** Reference to light sensor equipped entity associated to this sensor */
      CLightSensorEquippedEntity* m_pcLightEntity;

      /** Reference to controllable entity associated to this sensor */
      CControllableEntity* m_pcControllableEntity;

      /** Flag to show rays in the simulator */
      bool m_bShowRays;

      /** Random number generator */
      CRandom::CRNG* m_pcRNG;

      /** Whether to add noise or not */
      bool m_bAddNoise;

      /** Noise range */
      CRange<Real> m_cNoiseRange;

      /** Reference to the space */
      CSpace& m_cSpace;

      bool m_bEnabled = true;
   };

}

#endif
