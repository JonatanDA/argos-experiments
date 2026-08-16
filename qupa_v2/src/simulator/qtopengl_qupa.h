#ifndef QTOPENGL_QUPA_H
#define QTOPENGL_QUPA_H

namespace argos {
   class CQupaEntity;
}
#include <argos3/core/utility/datatypes/datatypes.h>
#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

namespace argos {

   class CQTOpenGLQupa {

   public:

      CQTOpenGLQupa();

      virtual ~CQTOpenGLQupa();

      virtual void Draw(CQupaEntity& c_entity);

   private:

      /**
       * @brief Renderiza el cuerpo del robot en una display list.
       */
      void Render();

      /**
       * @brief Dibuja los LEDs de la entidad del qupa.
       */
      void DrawLEDs(CQupaEntity& c_entity);

   private:

      /* Display list para el cuerpo del robot */
      GLuint m_unBodyList;
      
      /* Display list para un único LED (una esfera pequeña) */
      GLuint m_unLEDList;

      // Esta variable controlará la escala del modelo.
      Real m_fScale;
   };
}

#endif
