#ifndef TEXTURE_H
#define TEXTURE_H

#include <QVector>
#include <QTextStream>
#include <QVector3D>
#include <QVector2D>
#include <iostream>
#include "Library/View/cbmp.h"
#include <GLUT/glut.h>
#include <GL/freeglut.h>
#include <OpenGL/glu.h>
#include <OpenGL/gl.h>


class Texture
{
public:
    Texture();
    GLuint id;
    int height;
    int width;
    BMP *bmp;

    //GLuint LoadTexture( const char * filename );
};


#endif // TEXTURE_H
