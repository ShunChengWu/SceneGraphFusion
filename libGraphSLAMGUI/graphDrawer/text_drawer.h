#pragma once
#include <Eigen/Core>
#include "../../Utils/define.h"
#include "../../libGUI3D/libGUI3D/glShader.hpp"
#include <vector>
#include <memory>
#include <map>

#ifdef COMPILE_WITH_FREETYPE
#include <ft2build.h>
#include FT_FREETYPE_H
#endif

namespace PSLAM {
    class TextDrawer {
        struct Character {
            GLuint TextureID;   // ID handle of the glyph texture
            Eigen::Vector2i Size;    // Size of glyph
            Eigen::Vector2i Bearing;  // Offset from baseline to left/top of glyph
            GLuint Advance;    // Horizontal offset to advance to next glyph
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

    public:

        TextDrawer();

        void Init();

        void Draw(const std::string& text, GLfloat x, GLfloat y, float width, float height,
                  GLfloat scale, const Eigen::Vector3f& color);

        float text_size = 1;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        bool bInited;
        std::unique_ptr<glUtil::Shader> mShader;
        unsigned int VAO,VBO;
        std::map<GLchar, Character> Characters;

        void buildText();

        void RenderText(std::string text, GLfloat x, GLfloat y, float width, float height,
                GLfloat scale, const Eigen::Vector3f& color);

        void buildFreeType();

        const std::string vsShader =
                "#version 330 core\n"
                "layout (location = 0) in vec4 vertex; // <vec2 pos, vec2 tex>\n"
                "out vec2 TexCoords;\n"
                "void main()\n"
                "{\n"
                "    gl_Position = vec4(vertex.xy, 0.0, 1.0);\n"
                "    TexCoords = vertex.zw;\n"
                "}";

        const std::string fsShader =
                "#version 330 core\n"
                "in vec2 TexCoords;\n"
                "out vec4 color;\n"
                "uniform sampler2D text;\n"
                "uniform vec3 textColor;\n"
                "void main()\n"
                "{\n"
                "    vec4 sampled = vec4(1.0, 1.0, 1.0, texture(text, TexCoords).r);\n"
                "    color = vec4(textColor, 1.0) * sampled;\n"
                "}";
    };
}