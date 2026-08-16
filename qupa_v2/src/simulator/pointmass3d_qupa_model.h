/**
 * @file <argos3/plugins/simulator/robots/qupa/simulator/pointmass3d/pointmass3d_qupa_model.h>
 *
 * @author Jose Santos
 * Gabriel Madroñero
 */

#ifndef POINTMASS3D_QUPA_H
#define POINTMASS3D_QUPA_H

namespace argos {
   class CPointMass3DEngine;
   class CPointMass3DQupaModel;
   class CQupaEntity;
}

#include <argos3/plugins/simulator/physics_engines/pointmass3d/pointmass3d_model.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include "qupa_entity.h"

namespace argos {

   class CPointMass3DQupaModel : public CPointMass3DModel {

   public:
      
      CPointMass3DQupaModel(CPointMass3DEngine& c_engine,
                               CQupaEntity& c_qupa);

      virtual ~CPointMass3DQupaModel() {}
      
      virtual void Reset();

      virtual void UpdateFromEntityStatus();
      virtual void Step();

      virtual void CalculateBoundingBox();

      virtual bool CheckIntersectionWithRay(Real& f_t_on_ray,
                                            const CRay3& c_ray) const;

      virtual void UpdateOriginAnchor(SAnchor& s_anchor);

   private:

      /** Reference to the wheeled entity */
      CWheeledEntity& m_cWheeledEntity;

      /** The yaw of the qupa entity */
      CRadians m_cYaw;

      /** The yaw of the qupa entity */
      Real m_fAngularVelocity;

      /** Current wheel velocity */
      const Real* m_fCurrentWheelVelocity;
   };

}

#endif
