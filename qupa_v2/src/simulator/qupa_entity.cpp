/**
 * @file <argos3/plugins/robots/qupa/simulator/qupa_entity.cpp>
 *
 * @author Jose Santos
 * Gabriel Madroñero 
 * - <emails >
 */

#include "qupa_entity.h"

#include <argos3/core/utility/math/matrix/rotationmatrix3.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/plugins/simulator/entities/battery_equipped_entity.h>
#include <argos3/plugins/simulator/entities/rab_equipped_entity.h>
#include <argos3/plugins/simulator/entities/gripper_equipped_entity.h>
#include <argos3/plugins/simulator/entities/ground_sensor_equipped_entity.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>
#include <argos3/plugins/simulator/entities/light_sensor_equipped_entity.h>
#include <argos3/plugins/simulator/entities/omnidirectional_camera_equipped_entity.h>
#include <argos3/plugins/simulator/entities/perspective_camera_equipped_entity.h>
#include <argos3/plugins/simulator/entities/proximity_sensor_equipped_entity.h>
#include "qupa_distance_scanner_equipped_entity.h"
#include "qupa_turret_entity.h"

namespace argos {

   /****************************************/
   /****************************************/

   static const Real BODY_RADIUS                = 0.07f;
   static const Real BODY_HEIGHT                = 0.146f;

   static const Real LED_RING_RADIUS            = BODY_RADIUS - 0.03f;

   static const Real INTERWHEEL_DISTANCE        = 0.14f;
   static const Real HALF_INTERWHEEL_DISTANCE   = INTERWHEEL_DISTANCE * 0.5f;
   static const Real WHEEL_RADIUS               = 0.029112741f;

   static const Real PROXIMITY_SENSOR_RING_ELEVATION       = 0.03f;
   static const Real PROXIMITY_SENSOR_RING_RADIUS          = BODY_RADIUS;
   static const CRadians PROXIMITY_SENSOR_RING_START_ANGLE = CRadians((ARGOS_PI / 12.0f) * 0.5f);
   
   /* Rango de los rayos de proximidad 50cm */
   static const Real PROXIMITY_SENSOR_RING_RANGE           = 0.2f; 

   static const Real LED_RING_ELEVATION         = 0.10f; /*Original = 0.10f*/
   static const Real RAB_ELEVATION              = 0.08f;
   static const Real BEACON_ELEVATION           = 0.15f; /*Original = 0.13f*/

   static const Real GRIPPER_ELEVATION          = LED_RING_ELEVATION;

   static const CRadians LED_ANGLE_SLICE        = CRadians(ARGOS_PI / 6.0);
   static const CRadians HALF_LED_ANGLE_SLICE   = LED_ANGLE_SLICE * 0.5f;

   static const Real OMNIDIRECTIONAL_CAMERA_ELEVATION = 0.245f;

   //  ángulos de los 6 sensores de proximidad
   static const CRadians QUPA_PROXIMITY_SENSOR_ANGLES[6] = {
        CRadians(0.0f),           // Sensor 1: 0° (frontal)
        CRadians(-ARGOS_PI / 4.0f),   // Sensor 2: 45°
        CRadians(ARGOS_PI / 4.0f),   // Sensor 2: 45°
        CRadians(-ARGOS_PI / 2.0f),   // Sensor 3: 90° (lateral derecho)
        CRadians(ARGOS_PI / 2.0f),   // Sensor 3: 90° (lateral derecho)
        CRadians(ARGOS_PI),       // Sensor 5: 180° (trasero)

   };


   CQupaEntity::CQupaEntity() :
      CComposableEntity(nullptr),
      m_pcControllableEntity(nullptr),
      m_pcDistanceScannerEquippedEntity(nullptr),
      m_pcTurretEntity(nullptr),
      m_pcEmbodiedEntity(nullptr),
      m_pcGripperEquippedEntity(nullptr),
      m_pcGroundSensorEquippedEntity(nullptr),
      m_pcLEDEquippedEntity(nullptr),
      m_pcLightSensorEquippedEntity(nullptr),
      m_pcOmnidirectionalCameraEquippedEntity(nullptr),
      m_pcPerspectiveCameraEquippedEntity(nullptr),
      m_pcProximitySensorEquippedEntity(nullptr),
      m_pcRABEquippedEntity(nullptr),
      m_pcWheeledEntity(nullptr),
      m_pcBatteryEquippedEntity(nullptr) {
   }

   /****************************************/
   /****************************************/

   CQupaEntity::CQupaEntity(const std::string& str_id,
                                  const std::string& str_controller_id,
                                  const CVector3& c_position,
                                  const CQuaternion& c_orientation,
                                  Real f_rab_range,
                                  size_t un_rab_data_size,
                                  const std::string& str_bat_model,
                                  const CRadians& c_omnicam_aperture,
                                  bool b_perspcam_front,
                                  const CRadians& c_perspcam_aperture,
                                  Real f_perspcam_focal_length,
                                  Real f_perspcam_range) :
      CComposableEntity(nullptr, str_id),
      m_pcControllableEntity(nullptr),
      m_pcDistanceScannerEquippedEntity(nullptr),
      m_pcTurretEntity(nullptr),
      m_pcEmbodiedEntity(nullptr),
      m_pcGripperEquippedEntity(nullptr),
      m_pcGroundSensorEquippedEntity(nullptr),
      m_pcLEDEquippedEntity(nullptr),
      m_pcLightSensorEquippedEntity(nullptr),
      m_pcOmnidirectionalCameraEquippedEntity(nullptr),
      m_pcPerspectiveCameraEquippedEntity(nullptr),
      m_pcProximitySensorEquippedEntity(nullptr),
      m_pcRABEquippedEntity(nullptr),
      m_pcWheeledEntity(nullptr),
      m_pcBatteryEquippedEntity(nullptr) {
      try {
         /*
          * Create and init components
          */
         /*
          * Embodied entity
          * Better to put this first, because many other entities need this one
          */
         m_pcEmbodiedEntity = new CEmbodiedEntity(this, "body_0", c_position, c_orientation);
         AddComponent(*m_pcEmbodiedEntity);
         SAnchor& cTurretAnchor = m_pcEmbodiedEntity->AddAnchor("turret");
         // Eliminar la redeclaración de cPerspCamOrient y cPerspCamAnchor aquí
         // CQuaternion cPerspCamOrient(b_perspcam_front ? CRadians::ZERO : -CRadians::PI_OVER_TWO,
         //                             CVector3::Y);
         // SAnchor& cPerspCamAnchor = m_pcEmbodiedEntity->AddAnchor("perspective_camera",
         //                                                          CVector3(BODY_RADIUS, 0.0, BEACON_ELEVATION),
         //                                                          cPerspCamOrient);
         /* Wheeled entity and wheel positions (left, right) */
         m_pcWheeledEntity = new CWheeledEntity(this, "wheels_0", 2);
         AddComponent(*m_pcWheeledEntity);
         m_pcWheeledEntity->SetWheel(0, CVector3(0.0f,  HALF_INTERWHEEL_DISTANCE, 0.0f), WHEEL_RADIUS);
         m_pcWheeledEntity->SetWheel(1, CVector3(0.0f, -HALF_INTERWHEEL_DISTANCE, 0.0f), WHEEL_RADIUS);
         /* LED equipped entity, with LEDs [0-5] and beacon [6] */
         m_pcLEDEquippedEntity = new CLEDEquippedEntity(this, "leds_0");
         AddComponent(*m_pcLEDEquippedEntity);
         m_pcLEDEquippedEntity->AddLEDRing(
            CVector3(0.0f, 0.0f, LED_RING_ELEVATION),
            LED_RING_RADIUS,
            HALF_LED_ANGLE_SLICE,
            1,
            cTurretAnchor);
         m_pcLEDEquippedEntity->AddLED(
            CVector3(0.0f, 0.0f, BEACON_ELEVATION),
            cTurretAnchor);
         /* Proximity sensor equipped entity */
         m_pcProximitySensorEquippedEntity =
            new CProximitySensorEquippedEntity(this, "proximity_0");
         AddComponent(*m_pcProximitySensorEquippedEntity);
         // INICIO CAMBIO CORREGIDO: Configura 6 sensores individuales en lugar del anillo de 24
         for(size_t i = 0; i < 6; ++i) {
             m_pcProximitySensorEquippedEntity->AddSensor(
                 CVector3(PROXIMITY_SENSOR_RING_RADIUS * Cos(QUPA_PROXIMITY_SENSOR_ANGLES[i]),
                          PROXIMITY_SENSOR_RING_RADIUS * Sin(QUPA_PROXIMITY_SENSOR_ANGLES[i]),
                          PROXIMITY_SENSOR_RING_ELEVATION),
                 // CORRECTO: Pasa un CVector3 como dirección
                 CVector3(Cos(QUPA_PROXIMITY_SENSOR_ANGLES[i]),
                          Sin(QUPA_PROXIMITY_SENSOR_ANGLES[i]),
                          0.0f),
                 PROXIMITY_SENSOR_RING_RANGE,
                 m_pcEmbodiedEntity->GetOriginAnchor());
         }
         m_pcProximitySensorEquippedEntity->Enable();
         // FIN CAMBIO CORREGIDO
         /* Light sensor equipped entity */
         m_pcLightSensorEquippedEntity =
            new CLightSensorEquippedEntity(this, "light_0");
         AddComponent(*m_pcLightSensorEquippedEntity);
         m_pcLightSensorEquippedEntity->AddSensorRing(
            CVector3(0.0f, 0.0f, PROXIMITY_SENSOR_RING_ELEVATION),
            PROXIMITY_SENSOR_RING_RADIUS,
            PROXIMITY_SENSOR_RING_START_ANGLE,
            PROXIMITY_SENSOR_RING_RANGE,
            24, // Aquí podrías querer cambiar también los sensores de luz si solo quieres 6
            m_pcEmbodiedEntity->GetOriginAnchor());
         /* Gripper equipped entity */
         m_pcGripperEquippedEntity =
            new CGripperEquippedEntity(this,
                                       "gripper_0",
                                       CVector3(BODY_RADIUS, 0.0f, GRIPPER_ELEVATION),
                                       CVector3::X);
         AddComponent(*m_pcGripperEquippedEntity);
         /* Ground sensor equipped entity */
         m_pcGroundSensorEquippedEntity =
            new CGroundSensorEquippedEntity(this, "ground_0");
         AddComponent(*m_pcGroundSensorEquippedEntity);

         // --- Sensores de Escala de Grises (4 en total) - Índices [0] a [3] ---
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(0.063, 0.0116),
                                                   CGroundSensorEquippedEntity::TYPE_GRAYSCALE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(-0.063, 0.0116),
                                                   CGroundSensorEquippedEntity::TYPE_GRAYSCALE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(-0.063, -0.0116),
                                                   CGroundSensorEquippedEntity::TYPE_GRAYSCALE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(0.063, -0.0116),
                                                   CGroundSensorEquippedEntity::TYPE_GRAYSCALE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());

         // --- Sensores de Blanco y Negro (8 en total) - Índices [4] a [11] ---
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(0.08, 0.0),
                                                   CGroundSensorEquippedEntity::TYPE_BLACK_WHITE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(0.042, 0.065),
                                                   CGroundSensorEquippedEntity::TYPE_BLACK_WHITE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(0.0, 0.08),
                                                   CGroundSensorEquippedEntity::TYPE_BLACK_WHITE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(-0.042, 0.065),
                                                   CGroundSensorEquippedEntity::TYPE_BLACK_WHITE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(-0.08, 0.0),
                                                   CGroundSensorEquippedEntity::TYPE_BLACK_WHITE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(-0.042, -0.065),
                                                   CGroundSensorEquippedEntity::TYPE_BLACK_WHITE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(0.0, -0.08),
                                                   CGroundSensorEquippedEntity::TYPE_BLACK_WHITE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(0.042, -0.065),
                                                   CGroundSensorEquippedEntity::TYPE_BLACK_WHITE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());

         // <<< NUEVO SENSOR AÑADIDO AL FINAL - Índice [12] --- SEnsor del QUPA
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(0.063, 0.0),
                                                   CGroundSensorEquippedEntity::TYPE_GRAYSCALE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());
         /* Distance scanner */
         m_pcDistanceScannerEquippedEntity =
            new CQupaDistanceScannerEquippedEntity(this, "distance_scanner_0");
         AddComponent(*m_pcDistanceScannerEquippedEntity);
         /* RAB equipped entity */
         m_pcRABEquippedEntity =
            new CRABEquippedEntity(this,
                                   "rab_0",
                                   un_rab_data_size,
                                   f_rab_range,
                                   m_pcEmbodiedEntity->GetOriginAnchor(),
                                   *m_pcEmbodiedEntity,
                                   CVector3(0.0f, 0.0f, RAB_ELEVATION));
         AddComponent(*m_pcRABEquippedEntity);
         /* Omnidirectional camera equipped entity */
         m_pcOmnidirectionalCameraEquippedEntity =
            new COmnidirectionalCameraEquippedEntity(this,
                                                     "omnidirectional_camera",
                                                     c_omnicam_aperture,
                                                     CVector3(0.0f,
                                                              0.0f,
                                                              OMNIDIRECTIONAL_CAMERA_ELEVATION));
         AddComponent(*m_pcOmnidirectionalCameraEquippedEntity);
         m_pcOmnidirectionalCameraEquippedEntity->Enable();
         /* Perspective camera equipped entity */
         // Declarar cPerspCamOrient y cPerspCamAnchor aquí por primera vez
         CQuaternion cPerspCamOrient(b_perspcam_front ? CRadians::ZERO : -CRadians::PI_OVER_TWO,
                                     CVector3::Y);
         SAnchor& cPerspCamAnchor = m_pcEmbodiedEntity->AddAnchor("perspective_camera",
                                                                  CVector3(BODY_RADIUS, 0.0, BEACON_ELEVATION),
                                                                  cPerspCamOrient);
         m_pcPerspectiveCameraEquippedEntity =
            new CPerspectiveCameraEquippedEntity(this,
                                                 "perspective_camera_0",
                                                 c_perspcam_aperture,
                                                 f_perspcam_focal_length,
                                                 f_perspcam_range,
                                                 640, 480,
                                                 cPerspCamAnchor);
         AddComponent(*m_pcPerspectiveCameraEquippedEntity);
         /* Turret equipped entity */
         m_pcTurretEntity = new CQupaTurretEntity(this, "turret_0", cTurretAnchor);
         AddComponent(*m_pcTurretEntity);
         /* Battery equipped entity */
         m_pcBatteryEquippedEntity = new CBatteryEquippedEntity(this, "battery_0", str_bat_model);
         AddComponent(*m_pcBatteryEquippedEntity);
         /* Controllable entity
            It must be the last one, for actuators/sensors to link to composing entities correctly */
         m_pcControllableEntity = new CControllableEntity(this, "controller");
         AddComponent(*m_pcControllableEntity);
         m_pcControllableEntity->SetController(str_controller_id);
         /* Update components */
         UpdateComponents();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize entity \"" << GetId() << "\".", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CQupaEntity::Init(TConfigurationNode& t_tree) {
      try {
         /*
          * Init parent
          */
         CComposableEntity::Init(t_tree);
         /*
          * Create and init components
          */
         /*
          * Embodied entity
          * Better to put this first, because many other entities need this one
          */
         m_pcEmbodiedEntity = new CEmbodiedEntity(this);
         AddComponent(*m_pcEmbodiedEntity);
         m_pcEmbodiedEntity->Init(GetNode(t_tree, "body"));
         SAnchor& cTurretAnchor = m_pcEmbodiedEntity->AddAnchor("turret");
         /* Wheeled entity and wheel positions (left, right) */
         m_pcWheeledEntity = new CWheeledEntity(this, "wheels_0", 2);
         AddComponent(*m_pcWheeledEntity);
         m_pcWheeledEntity->SetWheel(0, CVector3(0.0f,  HALF_INTERWHEEL_DISTANCE, 0.0f), WHEEL_RADIUS);
         m_pcWheeledEntity->SetWheel(1, CVector3(0.0f, -HALF_INTERWHEEL_DISTANCE, 0.0f), WHEEL_RADIUS);
         /* LED equipped entity, with LEDs [0-5] and beacon [6] */
         m_pcLEDEquippedEntity = new CLEDEquippedEntity(this, "leds_0");
         AddComponent(*m_pcLEDEquippedEntity);
         m_pcLEDEquippedEntity->AddLEDRing(
            CVector3(0.0f, 0.0f, LED_RING_ELEVATION),
            LED_RING_RADIUS,
            HALF_LED_ANGLE_SLICE,
            1,
            cTurretAnchor);
         m_pcLEDEquippedEntity->AddLED(
            CVector3(0.0f, 0.0f, BEACON_ELEVATION),
            cTurretAnchor);
         /* Proximity sensor equipped entity */
         m_pcProximitySensorEquippedEntity =
            new CProximitySensorEquippedEntity(this, "proximity_0");
         AddComponent(*m_pcProximitySensorEquippedEntity);
         // INICIO CAMBIO CORREGIDO: Configura 6 sensores individuales en lugar del anillo de 24
         // Primero, verifica si hay una configuración de proximidad explícita en el XML
         // Si la hay, la clase CProximitySensorEquippedEntity la manejará.
         // Si no, agregaremos nuestros 6 sensores por defecto.
         if(NodeExists(t_tree, "proximity")) {
             m_pcProximitySensorEquippedEntity->Init(GetNode(t_tree, "proximity"));
         } else {
             for(size_t i = 0; i < 6; ++i) {
                 m_pcProximitySensorEquippedEntity->AddSensor(
                     CVector3(PROXIMITY_SENSOR_RING_RADIUS * Cos(QUPA_PROXIMITY_SENSOR_ANGLES[i]),
                              PROXIMITY_SENSOR_RING_RADIUS * Sin(QUPA_PROXIMITY_SENSOR_ANGLES[i]),
                              PROXIMITY_SENSOR_RING_ELEVATION),
                     // CORRECTO: Pasa un CVector3 como dirección
                     CVector3(Cos(QUPA_PROXIMITY_SENSOR_ANGLES[i]),
                              Sin(QUPA_PROXIMITY_SENSOR_ANGLES[i]),
                              0.0f),
                     PROXIMITY_SENSOR_RING_RANGE,
                     m_pcEmbodiedEntity->GetOriginAnchor());
             }
         }
         m_pcProximitySensorEquippedEntity->Enable();
         // FIN CAMBIO CORREGIDO
         /* Light sensor equipped entity */
         m_pcLightSensorEquippedEntity =
            new CLightSensorEquippedEntity(this, "light_0");
         AddComponent(*m_pcLightSensorEquippedEntity);
         m_pcLightSensorEquippedEntity->AddSensorRing(
            CVector3(0.0f, 0.0f, PROXIMITY_SENSOR_RING_ELEVATION),
            PROXIMITY_SENSOR_RING_RADIUS,
            PROXIMITY_SENSOR_RING_START_ANGLE,
            PROXIMITY_SENSOR_RING_RANGE,
            24, // Aquí podrías querer cambiar también los sensores de luz si solo quieres 6
            m_pcEmbodiedEntity->GetOriginAnchor());
         /* Gripper equipped entity */
         m_pcGripperEquippedEntity =
            new CGripperEquippedEntity(this,
                                       "gripper_0",
                                       CVector3(BODY_RADIUS, 0.0f, GRIPPER_ELEVATION),
                                       CVector3::X);
         AddComponent(*m_pcGripperEquippedEntity);
         /* Ground sensor equipped entity */
         m_pcGroundSensorEquippedEntity =
            new CGroundSensorEquippedEntity(this, "ground_0");
         AddComponent(*m_pcGroundSensorEquippedEntity);
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(0.063, 0.0116),
                                                   CGroundSensorEquippedEntity::TYPE_GRAYSCALE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(-0.063, 0.0116),
                                                   CGroundSensorEquippedEntity::TYPE_GRAYSCALE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(-0.063, -0.0116),
                                                   CGroundSensorEquippedEntity::TYPE_GRAYSCALE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(0.063, -0.0116),
                                                   CGroundSensorEquippedEntity::TYPE_GRAYSCALE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(0.08, 0.0),
                                                   CGroundSensorEquippedEntity::TYPE_BLACK_WHITE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(0.042, 0.065),
                                                   CGroundSensorEquippedEntity::TYPE_BLACK_WHITE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(0.0, 0.08),
                                                   CGroundSensorEquippedEntity::TYPE_BLACK_WHITE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(-0.042, 0.065),
                                                   CGroundSensorEquippedEntity::TYPE_BLACK_WHITE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(-0.08, 0.0),
                                                   CGroundSensorEquippedEntity::TYPE_BLACK_WHITE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(-0.042, -0.065),
                                                   CGroundSensorEquippedEntity::TYPE_BLACK_WHITE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(0.0, -0.08),
                                                   CGroundSensorEquippedEntity::TYPE_BLACK_WHITE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(0.042, -0.065),
                                                   CGroundSensorEquippedEntity::TYPE_BLACK_WHITE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());
         /* Distance scanner */
         m_pcDistanceScannerEquippedEntity =
            new CQupaDistanceScannerEquippedEntity(this, "distance_scanner_0");
         AddComponent(*m_pcDistanceScannerEquippedEntity);
         /* RAB equipped entity */
         Real fRange = 3.0f;
         GetNodeAttributeOrDefault(t_tree, "rab_range", fRange, fRange);
         UInt32 unDataSize = 10;
         GetNodeAttributeOrDefault(t_tree, "rab_data_size", unDataSize, unDataSize);
         m_pcRABEquippedEntity =
            new CRABEquippedEntity(this,
                                   "rab_0",
                                   unDataSize,
                                   fRange,
                                   m_pcEmbodiedEntity->GetOriginAnchor(),
                                   *m_pcEmbodiedEntity,
                                   CVector3(0.0f, 0.0f, RAB_ELEVATION));
         AddComponent(*m_pcRABEquippedEntity);
         /* Omnidirectional camera equipped entity */
         CDegrees cAperture(80.0f);
         GetNodeAttributeOrDefault(t_tree, "omnidirectional_camera_aperture", cAperture, cAperture);
         m_pcOmnidirectionalCameraEquippedEntity =
            new COmnidirectionalCameraEquippedEntity(this,
                                                     "omnidirectional_camera",
                                                     ToRadians(cAperture),
                                                     CVector3(0.0f,
                                                              0.0f,
                                                              OMNIDIRECTIONAL_CAMERA_ELEVATION));
         AddComponent(*m_pcOmnidirectionalCameraEquippedEntity);
         /* Perspective camera equipped entity */
         bool bPerspCamFront = true;
         GetNodeAttributeOrDefault(t_tree, "perspective_camera_front", bPerspCamFront, bPerspCamFront);
         Real fPerspCamFocalLength = 0.035;
         GetNodeAttributeOrDefault(t_tree, "perspective_camera_focal_length", fPerspCamFocalLength, fPerspCamFocalLength);
         Real fPerspCamRange = 2.0;
         GetNodeAttributeOrDefault(t_tree, "perspective_camera_range", fPerspCamRange, fPerspCamRange);
         cAperture.SetValue(30.0f);
         GetNodeAttributeOrDefault(t_tree, "perspective_camera_aperture", cAperture, cAperture);
         CQuaternion cPerspCamOrient(bPerspCamFront ? CRadians::ZERO : -CRadians::PI_OVER_TWO,
                                     CVector3::Y);
         SAnchor& cPerspCamAnchor = m_pcEmbodiedEntity->AddAnchor("perspective_camera",
                                                                  CVector3(BODY_RADIUS, 0.0, BEACON_ELEVATION),
                                                                  cPerspCamOrient);
         m_pcPerspectiveCameraEquippedEntity =
            new CPerspectiveCameraEquippedEntity(this,
                                                 "perspective_camera_0",
                                                 ToRadians(cAperture),
                                                 fPerspCamFocalLength,
                                                 fPerspCamRange,
                                                 640, 480,
                                                 cPerspCamAnchor);
         AddComponent(*m_pcPerspectiveCameraEquippedEntity);
         /* Turret equipped entity */
         m_pcTurretEntity = new CQupaTurretEntity(this, "turret_0", cTurretAnchor);
         AddComponent(*m_pcTurretEntity);
         /* Battery equipped entity */
         m_pcBatteryEquippedEntity = new CBatteryEquippedEntity(this, "battery_0");
         if(NodeExists(t_tree, "battery"))
            m_pcBatteryEquippedEntity->Init(GetNode(t_tree, "battery"));
         AddComponent(*m_pcBatteryEquippedEntity);
         /* Controllable entity
            It must be the last one, for actuators/sensors to link to composing entities correctly */
         m_pcControllableEntity = new CControllableEntity(this);
         AddComponent(*m_pcControllableEntity);
         m_pcControllableEntity->Init(GetNode(t_tree, "controller"));
         /* Update components */
         UpdateComponents();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize entity \"" << GetId() << "\".", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CQupaEntity::Reset() {
      /* Reset all components */
      CComposableEntity::Reset();
      /* Update components */
      UpdateComponents();
   }

   /****************************************/
   /****************************************/

#define UPDATE(COMPONENT) if(COMPONENT->IsEnabled()) COMPONENT->Update();

   void CQupaEntity::UpdateComponents() {
     UPDATE(m_pcDistanceScannerEquippedEntity);
     UPDATE(m_pcTurretEntity);
     UPDATE(m_pcGripperEquippedEntity);
     UPDATE(m_pcRABEquippedEntity);
     UPDATE(m_pcLEDEquippedEntity);
     UPDATE(m_pcBatteryEquippedEntity);

     // FALTABAN ESTAS:
     UPDATE(m_pcProximitySensorEquippedEntity);
     UPDATE(m_pcLightSensorEquippedEntity);
     UPDATE(m_pcGroundSensorEquippedEntity);
     UPDATE(m_pcOmnidirectionalCameraEquippedEntity);
     UPDATE(m_pcPerspectiveCameraEquippedEntity);
   }

   /****************************************/
   /****************************************/


   REGISTER_ENTITY(CQupaEntity,
                   "qupa",
                   "Carlo Pinciroli [ilpincy@gmail.com]",
                   "1.0",
                   "The qupa robot, developed in the Swarmanoid project.",
                   "The qupa is a wheeled robot developed in the Swarmanoid Project. It is a\n"
                   "modular robot with a rich set of sensors and actuators. For more information,\n"
                   "refer to the dedicated web page\n"
                   "(http://www.swarmanoid.org/swarmanoid_hardware.php).\n\n"
                   "REQUIRED XML CONFIGURATION\n\n"
                   "  <arena ...>\n"
                   "    ...\n"
                   "    <qupa id=\"fb0\">\n"
                   "      <body position=\"0.4,2.3,0.25\" orientation=\"45,0,0\" />\n"
                   "      <controller config=\"mycntrl\" />\n"
                   "    </qupa>\n"
                   "    ...\n"
                   "  </arena>\n\n"
                   "The 'id' attribute is necessary and must be unique among the entities. If two\n"
                   "entities share the same id, initialization aborts.\n"
                   "The 'body/position' attribute specifies the position of the bottom point of the\n"
                   "qupa in the arena. When the robot is untranslated and unrotated, the\n"
                   "bottom point is in the origin and it is defined as the middle point between\n"
                   "the two wheels on the XY plane and the lowest point of the robot on the Z\n"
                   "axis, that is the point where the wheels touch the floor. The attribute values\n"
                   "are in the X,Y,Z order.\n"
                   "The 'body/orientation' attribute specifies the orientation of the qupa. All\n"
                   "rotations are performed with respect to the bottom point. The order of the\n"
                   "angles is Z,Y,X, which means that the first number corresponds to the rotation\n"
                   "around the Z axis, the second around Y and the last around X. This reflects\n"
                   "the internal convention used in ARGoS, in which rotations are performed in\n"
                   "that order. Angles are expressed in degrees. When the robot is unrotated, it\n"
                   "is oriented along the X axis.\n"
                   "The 'controller/config' attribute is used to assign a controller to the\n"
                   "qupa. The value of the attribute must be set to the id of a previously\n"
                   "defined controller. Controllers are defined in the <controllers> XML subtree.\n\n"
                   "OPTIONAL XML CONFIGURATION\n\n"
                   "You can set the emission range of the range-and-bearing system. By default, a\n"
                   "message sent by a qupa can be received up to 3m. By using the 'rab_range'\n"
                   "attribute, you can change it to, i.e., 4m as follows:\n\n"
                   "  <arena ...>\n"
                   "    ...\n"
                   "    <qupa id=\"fb0\" rab_range=\"4\">\n"
                   "      <body position=\"0.4,2.3,0.25\" orientation=\"45,0,0\" />\n"
                   "      <controller config=\"mycntrl\" />\n"
                   "    </qupa>\n"
                   "    ...\n"
                   "  </arena>\n\n"
                   "You can also set the data sent at each time step through the range-and-bearing\n"
                   "system. By default, a message sent by a qupa is 10 bytes long. By using the\n"
                   "'rab_data_size' attribute, you can change it to, i.e., 20 bytes as follows:\n\n"
                   "  <arena ...>\n"
                   "    ...\n"
                   "    <qupa id=\"fb0\" rab_data_size=\"20\">\n"
                   "      <body position=\"0.4,2.3,0.25\" orientation=\"45,0,0\" />\n"
                   "      <controller config=\"mycntrl\" />\n"
                   "    </qupa>\n"
                   "    ...\n"
                   "  </arena>\n\n"
                   "You can also configure the battery of the robot. By default, the battery never\n"
                   "depletes. You can choose among several battery discharge models, such as\n"
                   "- time: the battery depletes by a fixed amount at each time step\n"
                   "- motion: the battery depletes according to how the robot moves\n"
                   "- time_motion: a combination of the above models.\n"
                   "You can define your own models too. Follow the examples in the file\n"
                   "argos3/src/plugins/simulator/entities/battery_equipped_entity.cpp.\n\n"
                   "  <arena ...>\n"
                   "    ...\n"
                   "    <qupa id=\"fb0\"\n"
                   "      <body position=\"0.4,2.3,0.25\" orientation=\"45,0,0\" />\n"
                   "      <controller config=\"mycntrl\" />\n"
                   "      <battery model=\"time\" factor=\"1e-5\"/>\n"
                   "    </qupa>\n"
                   "    ...\n"
                   "  </arena>\n\n"
                   "  <arena ...>\n"
                   "    ...\n"
                   "    <qupa id=\"fb0\"\n"
                   "      <body position=\"0.4,2.3,0.25\" orientation=\"45,0,0\" />\n"
                   "      <controller config=\"mycntrl\" />\n"
                   "      <battery model=\"motion\" pos_factor=\"1e-3\"\n"
                   "                              orient_factor=\"1e-3\"/>\n"
                   "    </qupa>\n"
                   "    ...\n"
                   "  </arena>\n\n"
                   "  <arena ...>\n"
                   "    ...\n"
                   "    <qupa id=\"fb0\"\n"
                   "      <body position=\"0.4,2.3,0.25\" orientation=\"45,0,0\" />\n"
                   "      <controller config=\"mycntrl\" />\n"
                   "      <battery model=\"time_motion\" time_factor=\"1e-5\"\n"
                   "                                   pos_factor=\"1e-3\"\n"
                   "                                   orient_factor=\"1e-3\"/>\n"
                   "    </qupa>\n"
                   "    ...\n"
                   "  </arena>\n\n"
                   "You can also change the aperture of the omnidirectional camera. The aperture is\n"
                   "set to 70 degrees by default. The tip of the omnidirectional camera is placed on\n"
                   "top of the robot (h=0.289), and with an aperture of 70 degrees the range on the\n"
                   "ground is r=h*tan(aperture)=0.289*tan(70)=0.794m. To change the aperture to 80\n"
                   "degrees, use the 'omnidirectional_camera_aperture' as follows:\n\n"
                   "  <arena ...>\n"
                   "    ...\n"
                   "    <qupa id=\"fb0\" omnidirectional_camera_aperture=\"80\">\n"
                   "      <body position=\"0.4,2.3,0.25\" orientation=\"45,0,0\" />\n"
                   "      <controller config=\"mycntrl\" />\n"
                   "    </qupa>\n"
                   "    ...\n"
                   "  </arena>\n\n"
                   "Finally, you can change the parameters of the perspective camera. You can set\n"
                   "its direction, aperture, focal length, and range with the attributes\n"
                   "'perspective_camera_front', 'perspective_camera_aperture',\n"
                   "'perspective_camera_focal_length', and 'perspective_camera_range', respectively.\n"
                   "The default values are: 'true' for front direction, 30 degrees for aperture,\n"
                   "0.035 for focal length, and 2 meters for range. When the direction is set to\n"
                   "'false', the camera looks up. This can be useful to see the eye-bot LEDs. Check\n"
                   "the following example:\n\n"
                   "  <arena ...>\n"
                   "    ...\n"
                   "    <qupa id=\"fb0\"\n"
                   "              perspective_camera_front=\"false\"\n"
                   "              perspective_camera_aperture=\"45\"\n"
                   "              perspective_camera_focal_length=\"0.07\"\n"
                   "              perspective_camera_range=\"10\">\n"
                   "      <body position=\"0.4,2.3,0.25\" orientation=\"45,0,0\" />\n"
                   "      <controller config=\"mycntrl\" />\n"
                   "    </qupa>\n"
                   "    ...\n"
                   "  </arena>\n\n"
                   ,
                   "Under development"
      );

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CQupaEntity);

   /****************************************/
   /****************************************/

}