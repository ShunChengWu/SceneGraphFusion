//
// Created by sc on 6/5/20.
//

#include "text_drawer.h"

#include <iostream>

using namespace  PSLAM;

TextDrawer::TextDrawer():bInited(false){
}
void TextDrawer::Init() {
    buildText();
    buildFreeType();
    bInited = true;
}

void TextDrawer::Draw(const std::string& text, GLfloat x, GLfloat y, float width, float height,
          GLfloat scale, const Eigen::Vector3f& color){
    if(!bInited) throw std::runtime_error("TextDrawer: Draw was called before Init.\n");
    mShader->use();
    RenderText(text,x,y, width, height,scale,color);
}

void TextDrawer::buildText(){
    /// Text
    {
        // shader
        mShader.reset(new glUtil::Shader(vsShader,fsShader));
        mShader->use();
        // gl buffer
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 6 * 4, NULL, GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat), 0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }
}

void TextDrawer::RenderText(std::string text, GLfloat x, GLfloat y, float width, float height,
                GLfloat scale, const Eigen::Vector3f& color) {
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable( GL_BLEND );

    scale = text_size * scale;

    // Activate corresponding render state
    mShader->use();
    mShader->set("textColor", color);
    glActiveTexture(GL_TEXTURE0);
    glBindVertexArray(VAO);

    std::string::const_iterator c;

    {
        GLfloat x_t = x;
        /// Calculate bbox
        GLfloat x_min,x_max,y_min,y_max;
        x_min=x_max=y_min=y_max=0;
        for (c = text.begin(); c != text.end(); c++) {
            Character ch = Characters[*c];

            GLfloat xpos = x_t + ch.Bearing.x() * scale;
            GLfloat ypos = y - float(ch.Size.y() - ch.Bearing.y()) * scale;
            GLfloat w = ch.Size.x() * scale;
            GLfloat h = ch.Size.y() * scale;
            GLfloat vertices[6][4] = {
                    {xpos,     ypos + h, 0.0, 0.0},
                    {xpos,     ypos,     0.0, 1.0},
                    {xpos + w, ypos,     1.0, 1.0},

                    {xpos,     ypos + h, 0.0, 0.0},
                    {xpos + w, ypos,     1.0, 1.0},
                    {xpos + w, ypos + h, 1.0, 0.0}
            };

            for(auto & vertice : vertices){
                vertice[0] /= width;
                vertice[1] /= height;
                vertice[0] = vertice[0]*2 - 1;
                vertice[1] = vertice[1]*2 - 1;
            }
            if (c == text.begin()) {
                x_min = xpos;
                y_min = ypos;
            }
            auto cc = c;
            cc++;
            if (cc == text.end()) {
                x_max = xpos+w;
            }
            if(ypos+h>y_max)
                y_max = ypos+h;

            x_t += (ch.Advance >> 6) * scale;
        }

        if(x_min>0 && x_max>0 && y_min>0 && y_max>0) {
            auto ch = Characters[*text.begin()];
            const static float margin = 5;
            GLfloat xpos = x_min - margin * scale;
            GLfloat ypos = y_min - margin * scale;
            GLfloat w = x_max - x_min + 2*margin * scale;
            GLfloat h = y_max - y_min + 2*margin * scale;
            GLfloat vertices[6][4] = {
                    {xpos,     ypos + h, 0.0, 0.0},
                    {xpos,     ypos,     0.0, 1.0},
                    {xpos + w, ypos,     1.0, 1.0},

                    {xpos,     ypos + h, 0.0, 0.0},
                    {xpos + w, ypos,     1.0, 1.0},
                    {xpos + w, ypos + h, 1.0, 0.0}
            };

            for (auto &vertice : vertices) {
                vertice[0] /= width;
                vertice[1] /= height;
                vertice[0] = vertice[0] * 2 - 1;
                vertice[1] = vertice[1] * 2 - 1;
            }

            // Render glyph texture over quad
            glBindTexture(GL_TEXTURE_2D, ch.TextureID);
            // Update content of VBO memory
            glBindBuffer(GL_ARRAY_BUFFER, VBO);
            glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices),
                            vertices); // Be sure to use glBufferSubData and not glBufferData

            glBindBuffer(GL_ARRAY_BUFFER, 0);
            // Render quad
            mShader->set("textColor", {0.2, 0.2, 0.2});
//            Eigen::Vector3f color_;
//            color_.x() = 1.f-color.x();
//            color_.y() = 1.f-color.y();
//            color_.z() = 1.f-color.z();
//            mShader->set("textColor", color_);
            glDisable(GL_BLEND);
            glDrawArrays(GL_TRIANGLES, 0, 6);
            glEnable(GL_BLEND);
        }
    }
    glDisable(GL_DEPTH_TEST);


    mShader->set("textColor", color);
    // Iterate through all characters
    for (c = text.begin(); c != text.end(); c++) {
        Character ch = Characters[*c];

        GLfloat xpos = x + ch.Bearing.x() * scale;
        GLfloat ypos = y - float(ch.Size.y() - ch.Bearing.y()) * scale;

        GLfloat w = ch.Size.x() * scale;
        GLfloat h = ch.Size.y() * scale;

        // Update VBO for each character
//        std::cout << xpos << ", " << ypos << "; " << xpos+w << ", " << ypos+h << "\n";
        GLfloat vertices[6][4] = {
                {xpos,     ypos + h, 0.0, 0.0},
                {xpos,     ypos,     0.0, 1.0},
                {xpos + w, ypos,     1.0, 1.0},

                {xpos,     ypos + h, 0.0, 0.0},
                {xpos + w, ypos,     1.0, 1.0},
                {xpos + w, ypos + h, 1.0, 0.0}
        };

        for(auto & vertice : vertices){
            vertice[0] /= width;
            vertice[1] /= height;
            vertice[0] = vertice[0]*2 - 1;
            vertice[1] = vertice[1]*2 - 1;
        }

        // Render glyph texture over quad
        glBindTexture(GL_TEXTURE_2D, ch.TextureID);
        // Update content of VBO memory
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices),
                        vertices); // Be sure to use glBufferSubData and not glBufferData

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        // Render quad
        glDrawArrays(GL_TRIANGLES, 0, 6);
        // Now advance cursors for next glyph (note that advance is number of 1/64 pixels)
        x += (ch.Advance >> 6) *
             scale; // Bitshift by 6 to get value in pixels (2^6 = 64 (divide amount of 1/64th pixels by 64 to get amount of pixels))
    }
    glEnable(GL_DEPTH_TEST);
    glBindVertexArray(0);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void TextDrawer::buildFreeType(){
#ifdef COMPILE_WITH_FREETYPE
    const std::string fontPath = std::string(GUI_FOLDER_PATH) + "fonts/";
    /// FreeType
    {
        FT_Library ft;
        // All functions return a value different than 0 whenever an error occurred
        if (FT_Init_FreeType(&ft))
            throw std::runtime_error("ERROR::FREETYPE: Could not init FreeType Library");
        // Load font as face
        FT_Face face;
        if (FT_New_Face(ft, (fontPath + "Montserrat-Regular.ttf").c_str(), 0, &face))
            throw std::runtime_error("ERROR::FREETYPE: Failed to load font");
        // Set size to load glyphs as
        FT_Set_Pixel_Sizes(face, 0, 48);

        // Disable byte-alignment restriction
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

        // Load first 128 characters of ASCII set
        for (GLubyte c = 0; c < 128; c++) {
            // Load character glyph
            if (FT_Load_Char(face, c, FT_LOAD_RENDER)) {
                std::cout << "WARNING::FREETYTPE: Failed to load Glyph" << std::endl;
                continue;
            }
            // Generate texture
            GLuint texture;
            glGenTextures(1, &texture);
            glBindTexture(GL_TEXTURE_2D, texture);
            glTexImage2D(
                    GL_TEXTURE_2D,
                    0,
                    GL_RED,
                    face->glyph->bitmap.width,
                    face->glyph->bitmap.rows,
                    0,
                    GL_RED,
                    GL_UNSIGNED_BYTE,
                    face->glyph->bitmap.buffer
            );
            // Set texture options
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            // Now store character for later use
            Character character = {
                    texture,
                    {face->glyph->bitmap.width, face->glyph->bitmap.rows},
                    {face->glyph->bitmap_left, face->glyph->bitmap_top},
                    static_cast<GLuint>(face->glyph->advance.x)
            };
            Characters.insert(std::pair<GLchar, Character>(c, character));
        }
        // Destroy FreeType once we're finished
        FT_Done_Face(face);
        FT_Done_FreeType(ft);
    }
#endif
}