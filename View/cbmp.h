#ifndef CBMP_H
#define CBMP_H

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

typedef long LONG;
typedef unsigned char BYTE;
typedef unsigned int UINT;
typedef unsigned short WORD;
typedef unsigned long DWORD;
typedef void* LPVOID;

typedef struct tagBITMAPFILEHEADER
{
    WORD bfType;
    DWORD bfSize;
    WORD bfReserved1;
    WORD bfReserved2;
    DWORD bfOffbits;
}
BITMAPFILEHEADER;

typedef struct tagBITMAPINFOHEADER
{
    DWORD biSize;
    LONG biWidth;
    LONG biHeight;
    WORD biPlanes;
    WORD biBitCount;
    DWORD biCompression;
    DWORD biSizeImage;
    LONG biXPelsPerMeter;
    LONG biYPelsPerMeter;
    DWORD biClrUsed;
    DWORD biClrImportant;
}
BITMAPINFOHEADER;

typedef struct tagBMP
{
    LONG bmWidth;
    LONG bmHeight;
    LONG bmWidthBytes;
    WORD bmBitsPixel;
    LPVOID bmBits;
}
BMP;

class cbmp
{
public:
    cbmp();
    bool LoadBmp(BMP& bmp, const char* path);
    unsigned char* readBMP(const char* filename);
};

#endif // CBMP_H
