#include "Texture.h"


Texture::Texture()
{
}



/*GLuint LoadTexture( const char * filename )
{

  GLuint texture;

  int width, height;

  unsigned char * data;

  FILE * file;

  file = fopen( filename, "rb" );

  if ( file == NULL ) return 0;
  width = 1024;
  height = 512;
  data = (unsigned char *)malloc( width * height * 3 );
  //int size = fseek(file,);
  fread( data, width * height * 3, 1, file );
  fclose( file );

 for(int i = 0; i < width * height ; ++i)
{
   int index = i*3;
   unsigned char B,R;
   B = data[index];
   R = data[index+2];

   data[index] = R;
   data[index+2] = B;

}


glGenTextures( 1, &texture );
glBindTexture( GL_TEXTURE_2D, texture );
glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE,GL_MODULATE );
glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,GL_LINEAR_MIPMAP_NEAREST );


glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,GL_LINEAR );
glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S,GL_REPEAT );
glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T,GL_REPEAT );
gluBuild2DMipmaps( GL_TEXTURE_2D, 3, width, height,GL_RGB, GL_UNSIGNED_BYTE, data );
free( data );

return texture;
}
*/

/*
    bmp = new BMP();
    cbmp c;
    bmp = c.readBMP(filename);
    height = bmp->bmHeight;
    width = bmp->bmWidth;

    qDebug("bmp h is %d w is %d",height,width);
    //qDebug(filename);
    //id = width;
    //GLuint formerUsedTex;
    //glGetIntegerv(GL_TEXTURE_2D, (GLint*)&formerUsedTex);

    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glGenTextures(1, &id);

        qDebug("texture id:%u using image %s\n", id, filename);

        glBindTexture(GL_TEXTURE_2D, id);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, bmp->bmWidth, bmp->bmHeight, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, bmp->bmBits);
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_BLEND);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        //glBindTexture(GL_TEXTURE_2D, formerUsedTex);
    glPopAttrib();


    delete bmp;
}
*/
