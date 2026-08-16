#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>

using namespace argos;

class CRobotIDQTUserFunctions : public CQTOpenGLUserFunctions {
public:
    CRobotIDQTUserFunctions() {
        RegisterUserFunction<CRobotIDQTUserFunctions, CEPuckEntity>(&CRobotIDQTUserFunctions::Draw);
    }

    void Draw(CEPuckEntity& c_entity) {
        // Obtener ID del robot
        std::string strID = c_entity.GetId();
        // Dibujar el ID encima del robot
        DrawText(CVector3(0.0, 0.0, 0.1), strID);
    }
};

REGISTER_QTOPENGL_USER_FUNCTIONS(CRobotIDQTUserFunctions, "robot_id_qtuser_functions")
