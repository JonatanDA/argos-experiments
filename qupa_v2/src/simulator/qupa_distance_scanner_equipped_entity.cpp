/**
 * @file <argos3/plugins/robots/qupa/simulator/qupa_distance_scanner_equipped_entity.cpp>
 *
 * @author Jose Santos
 * Gabriel Madroñero
 */

#include "qupa_distance_scanner_equipped_entity.h"
#include <argos3/core/simulator/physics_engine/physics_engine.h>
#include <argos3/core/simulator/space/space.h>

namespace argos {

   /****************************************/
   /****************************************/

   CQupaDistanceScannerEquippedEntity::CQupaDistanceScannerEquippedEntity(CComposableEntity* pc_parent) :
      CEntity(pc_parent),
      m_unMode(MODE_OFF),
      m_fRotationSpeed(0.0f) {
      Disable();
   }
   
   /****************************************/
   /****************************************/

   CQupaDistanceScannerEquippedEntity::CQupaDistanceScannerEquippedEntity(CComposableEntity* pc_parent,
                                                                  const std::string& str_id) :
      CEntity(pc_parent, str_id),
      m_unMode(MODE_OFF),
      m_fRotationSpeed(0.0f) {
      Disable();
   }
   
   /****************************************/
   /****************************************/

   void CQupaDistanceScannerEquippedEntity::Reset() {
      m_unMode = MODE_OFF;
      m_cRotation = CRadians::ZERO;
      m_fRotationSpeed = 0.0f;
   }

   /****************************************/
   /****************************************/

   void CQupaDistanceScannerEquippedEntity::Update() {
      if(m_unMode == MODE_SPEED_CONTROL &&
         m_fRotationSpeed != 0.0f) {
         m_cRotation += CRadians(m_fRotationSpeed *
                                 CPhysicsEngine::GetSimulationClockTick());
         m_cRotation.UnsignedNormalize();
      }
   }

   /****************************************/
   /****************************************/

   void CQupaDistanceScannerEquippedEntity::SetRotation(const CRadians& c_rotation) {
      m_cRotation = c_rotation;
      m_cRotation.UnsignedNormalize();
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_ENTITY(CQupaDistanceScannerEquippedEntity);

   /****************************************/
   /****************************************/

}
