/**
 * @file <argos3/plugins/robots/qupa/simulator/qupa_entity.h>
 *
 * @author Jose Santos
 * Gabriel Madroñero 
 * - <emails >
 */

#ifndef QUPA_ENTITY_H
#define QUPA_ENTITY_H

namespace argos {
   class CBatteryEquippedEntity;
   class CControllableEntity;
   class CQupaDistanceScannerEquippedEntity;
   class CEmbodiedEntity;
   class CQupaEntity;
   class CQupaTurretEntity;
   class CGripperEquippedEntity;
   class CGroundSensorEquippedEntity;
   class CLEDEquippedEntity;
   class CLightSensorEquippedEntity;
   class COmnidirectionalCameraEquippedEntity;
   class CPerspectiveCameraEquippedEntity;
   class CProximitySensorEquippedEntity;
   class CRABEquippedEntity;
}

#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/simulator/entities/wheeled_entity.h>


namespace argos {

   class CQupaEntity : public CComposableEntity {

   public:

      ENABLE_VTABLE();

   public:

      CQupaEntity();

      CQupaEntity(const std::string& str_id,
                     const std::string& str_controller_id,
                     const CVector3& c_position = CVector3(),
                     const CQuaternion& c_orientation = CQuaternion(),
                     Real f_rab_range = 3.0f,
                     size_t un_rab_data_size = 10,
                     const std::string& str_bat_model = "",
                     const CRadians& c_omnicam_aperture = ToRadians(CDegrees(70.0f)),
                     bool b_perspcam_front = true,
                     const CRadians& c_perspcam_aperture = ToRadians(CDegrees(30.0f)),
                     Real f_perspcam_focal_length = 0.035,
                     Real f_perspcam_range = 2.0);

      virtual void Init(TConfigurationNode& t_tree);
      virtual void Reset();
      virtual void UpdateComponents();

      inline CControllableEntity& GetControllableEntity() {
         return *m_pcControllableEntity;
      }

      inline const CControllableEntity& GetControllableEntity() const {
        return *m_pcControllableEntity;
      }

      inline CQupaDistanceScannerEquippedEntity& GetDistanceScannerEquippedEntity() {
         return *m_pcDistanceScannerEquippedEntity;
      }

      inline CQupaTurretEntity& GetTurretEntity() {
         return *m_pcTurretEntity;
      }

      inline CEmbodiedEntity& GetEmbodiedEntity() {
         return *m_pcEmbodiedEntity;
      }

      inline CGripperEquippedEntity& GetGripperEquippedEntity() {
         return *m_pcGripperEquippedEntity;
      }

      inline CGroundSensorEquippedEntity& GetGroundSensorEquippedEntity() {
         return *m_pcGroundSensorEquippedEntity;
      }

      inline CLEDEquippedEntity& GetLEDEquippedEntity() {
         return *m_pcLEDEquippedEntity;
      }

      inline CLightSensorEquippedEntity& GetLightSensorEquippedEntity() {
         return *m_pcLightSensorEquippedEntity;
      }

      inline COmnidirectionalCameraEquippedEntity& GetOmnidirectionalCameraEquippedEntity() {
         return *m_pcOmnidirectionalCameraEquippedEntity;
      }

      inline CPerspectiveCameraEquippedEntity& GetPerspectiveCameraEquippedEntity() {
         return *m_pcPerspectiveCameraEquippedEntity;
      }

      inline CProximitySensorEquippedEntity& GetProximitySensorEquippedEntity() {
         return *m_pcProximitySensorEquippedEntity;
      }

      inline CRABEquippedEntity& GetRABEquippedEntity() {
         return *m_pcRABEquippedEntity;
      }

      inline CWheeledEntity& GetWheeledEntity() {
         return *m_pcWheeledEntity;
      }

      inline CBatteryEquippedEntity& GetBatterySensorEquippedEntity() {
          return *m_pcBatteryEquippedEntity;
      }

      virtual std::string GetTypeDescription() const {
         return "qupa";
      }

   private:

      CControllableEntity*                   m_pcControllableEntity;
      CQupaDistanceScannerEquippedEntity* m_pcDistanceScannerEquippedEntity;
      CQupaTurretEntity*                  m_pcTurretEntity;
      CEmbodiedEntity*                       m_pcEmbodiedEntity;
      CGripperEquippedEntity*                m_pcGripperEquippedEntity;
      CGroundSensorEquippedEntity*           m_pcGroundSensorEquippedEntity;
      CLEDEquippedEntity*                    m_pcLEDEquippedEntity;
      CLightSensorEquippedEntity*            m_pcLightSensorEquippedEntity;
      COmnidirectionalCameraEquippedEntity*  m_pcOmnidirectionalCameraEquippedEntity;
      CPerspectiveCameraEquippedEntity*      m_pcPerspectiveCameraEquippedEntity;
      CProximitySensorEquippedEntity*        m_pcProximitySensorEquippedEntity;
      CRABEquippedEntity*                    m_pcRABEquippedEntity;
      CWheeledEntity*                        m_pcWheeledEntity;
      CBatteryEquippedEntity*                m_pcBatteryEquippedEntity;
   };

}

#endif
