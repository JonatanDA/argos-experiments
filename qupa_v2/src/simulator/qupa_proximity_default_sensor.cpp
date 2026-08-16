/**
 * @file <argos3/plugins/robots/qupa/simulator/qupa_proximity_default_sensor.cpp>
 *
 * @authorJose Santos
 * Gabriel Madroñero 
 * - <emails >
 */

#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/simulator.h>

#include "qupa_proximity_default_sensor.h"

namespace argos {

   /****************************************/
   /****************************************/

   class CProximitySensorImpl : public CProximityDefaultSensor {

   public:

      virtual Real CalculateReading(Real f_distance) {
         /* Saturación a los 4cm (0.04m) según el sensor real */
         if(f_distance <= 0.01) {
            return 1.0;
         }
         /* Corte a los 50cm (0.50m) */
         else if(f_distance >= 0.20) {
            return 0.0;
         }
         /* Mapeo Lineal Suave: 
         Garantiza que a 4cm sea 1.0 y a 50cm sea 0.0 sin saltos. */
         else {
            return (0.20-f_distance) / (0.20-0.01);
         }
      }

   };

   /****************************************/
   /****************************************/

   CQupaProximityDefaultSensor::CQupaProximityDefaultSensor() :
      m_pcProximityImpl(new CProximitySensorImpl()) {}

   /****************************************/
   /****************************************/

   CQupaProximityDefaultSensor::~CQupaProximityDefaultSensor() {
      delete m_pcProximityImpl;
   }

   /****************************************/
   /****************************************/

   void CQupaProximityDefaultSensor::SetRobot(CComposableEntity& c_entity) {
      try {
         m_pcProximityImpl->SetRobot(c_entity);
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Can't set robot for the qupa proximity default sensor", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CQupaProximityDefaultSensor::Init(TConfigurationNode& t_tree) {
      m_pcProximityImpl->Init(t_tree);
      // AÑADE ESTA LÍNEA para limitar el número de sensores a 6
      m_tReadings.resize(6);
      Enable();// <-- añadido: activo por defecto
   }

   /****************************************/
   /****************************************/

   void CQupaProximityDefaultSensor::Update() {
      if(IsDisabled()) return;
      m_pcProximityImpl->Update();
      // Obtengo las lecturas del sensor de proximidad y las guardo en m_tReadings
      const std::vector<Real>& tImplReadings = m_pcProximityImpl->GetReadings();
      if(m_tReadings.size() != tImplReadings.size()) {
         m_tReadings.resize(tImplReadings.size());
      }

      for(size_t i = 0; i < m_tReadings.size(); ++i) {
         m_tReadings[i].Value = tImplReadings[i];
      }
   }  

   /****************************************/
   /****************************************/

   void CQupaProximityDefaultSensor::Reset() {
      m_pcProximityImpl->Reset();
      // AÑADE ESTE BUCLE para asegurar que tus 6 lecturas se reinicien
      for(size_t i = 0; i < m_tReadings.size(); ++i) {
         m_tReadings[i].Value = 0.0f;
      }
   }

   /****************************************/
   /****************************************/

   /*void CQupaProximityDefaultSensor::Enable() {
     CCI_Sensor::Enable();
     m_pcProximityImpl->Enable();
   } */

   /****************************************/
   /****************************************/

   /*void CQupaProximityDefaultSensor::Disable() {
     CCI_Sensor::Disable();
     m_pcProximityImpl->Disable();
   }*/

   void CQupaProximityDefaultSensor::Enable()  { m_bEnabled = true;  }  // <-- reemplazo
   void CQupaProximityDefaultSensor::Disable() { m_bEnabled = false; }  // <-- reemplazo
   bool CQupaProximityDefaultSensor::IsDisabled() const { return !m_bEnabled; } // <-- nuevo

   /****************************************/
   /****************************************/

   REGISTER_SENSOR(CQupaProximityDefaultSensor,
                   "qupa_proximity", "default",
                   "Carlo Pinciroli [ilpincy@gmail.com]",
                   "1.0",
                   "The qupa proximity sensor.",
                   "This sensor accesses the qupa proximity sensor. For a complete description\n"
                   "of its usage, refer to the ci_qupa_proximity_sensor.h interface. For the XML\n"
                   "configuration, refer to the default proximity sensor.\n",
                   "Usable"
		  );

}