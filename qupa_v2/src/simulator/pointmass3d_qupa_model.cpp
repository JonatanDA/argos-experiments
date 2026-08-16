/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_differentialsteering_control.cpp>
 *
 * @author Jose Santos
 * Gabriel Madroñero
 */

#include "pointmass3d_qupa_model.h"
#include <argos3/core/utility/math/cylinder.h>

namespace argos {

   static const Real QUPA_RADIUS                   = 0.07f;
   static const Real QUPA_INTERWHEEL_DISTANCE      = 0.14f;
   static const Real QUPA_HEIGHT                   = 0.15f;

   enum QUPA_WHEELS {
      QUPA_LEFT_WHEEL = 0,
      QUPA_RIGHT_WHEEL = 1
   };

   /****************************************/
   /****************************************/

   CPointMass3DQupaModel::CPointMass3DQupaModel(CPointMass3DEngine& c_engine,
                                                      CQupaEntity& c_qupa) :
      CPointMass3DModel(c_engine, c_qupa.GetEmbodiedEntity()),
      m_cWheeledEntity(c_qupa.GetWheeledEntity()),
      m_fCurrentWheelVelocity(m_cWheeledEntity.GetWheelVelocities()) {
      /* Register the origin anchor update method */
      RegisterAnchorMethod(GetEmbodiedEntity().GetOriginAnchor(),
                           &CPointMass3DQupaModel::UpdateOriginAnchor);
      /* Get initial rotation */
      CRadians cTmp1, cTmp2;
      GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(m_cYaw, cTmp1, cTmp2);
   }

   /****************************************/
   /****************************************/

   void CPointMass3DQupaModel::Reset() {
      CPointMass3DModel::Reset();
      CRadians cTmp1, cTmp2;
      GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(m_cYaw, cTmp1, cTmp2);
      m_fAngularVelocity = 0.0;
   }

   /****************************************/
   /****************************************/


   void CPointMass3DQupaModel::UpdateFromEntityStatus() {
      m_cVelocity.Set((m_fCurrentWheelVelocity[QUPA_RIGHT_WHEEL] + m_fCurrentWheelVelocity[QUPA_LEFT_WHEEL])*0.5, 0.0, 0.0);
      m_cVelocity.RotateZ(m_cYaw);
      m_fAngularVelocity = (m_fCurrentWheelVelocity[QUPA_RIGHT_WHEEL] - m_fCurrentWheelVelocity[QUPA_LEFT_WHEEL]) / QUPA_INTERWHEEL_DISTANCE;
   }

   /****************************************/
   /****************************************/

   void CPointMass3DQupaModel::Step() {
      m_cPosition += m_cVelocity * m_cPM3DEngine.GetPhysicsClockTick();
      m_cYaw += CRadians(m_fAngularVelocity * m_cPM3DEngine.GetPhysicsClockTick());
   }

   /****************************************/
   /****************************************/

   void CPointMass3DQupaModel::CalculateBoundingBox() {
      GetBoundingBox().MinCorner.Set(
         GetEmbodiedEntity().GetOriginAnchor().Position.GetX() - QUPA_RADIUS,
         GetEmbodiedEntity().GetOriginAnchor().Position.GetY() - QUPA_RADIUS,
         GetEmbodiedEntity().GetOriginAnchor().Position.GetZ());
      GetBoundingBox().MaxCorner.Set(
         GetEmbodiedEntity().GetOriginAnchor().Position.GetX() + QUPA_RADIUS,
         GetEmbodiedEntity().GetOriginAnchor().Position.GetY() + QUPA_RADIUS,
         GetEmbodiedEntity().GetOriginAnchor().Position.GetZ() + QUPA_HEIGHT);
   }

   /****************************************/
   /****************************************/

   bool CPointMass3DQupaModel::CheckIntersectionWithRay(Real& f_t_on_ray,
                                                           const CRay3& c_ray) const {
      CCylinder m_cShape(QUPA_RADIUS,
                         QUPA_HEIGHT,
                         m_cPosition,
                         CVector3::Z);
      bool bIntersects = m_cShape.Intersects(f_t_on_ray, c_ray);
      return bIntersects;
   }

   /****************************************/
   /****************************************/

   void CPointMass3DQupaModel::UpdateOriginAnchor(SAnchor& s_anchor) {
      s_anchor.Position = m_cPosition;
      s_anchor.Orientation = CQuaternion(m_cYaw, CVector3::Z);
   }

   /****************************************/
   /****************************************/
   
   REGISTER_STANDARD_POINTMASS3D_OPERATIONS_ON_ENTITY(CQupaEntity, CPointMass3DQupaModel);

   /****************************************/
   /****************************************/

}
