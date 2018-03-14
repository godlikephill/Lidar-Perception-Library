#include "cbmp.h"

cbmp::cbmp()
{
}

bool cbmp::LoadBmp(BMP &bmp, const char *path)
{
    BITMAPFILEHEADER fileHeader;
    BITMAPINFOHEADER infoHeader;
    FILE *fp;
    fp = fopen( path, "rb");
    if (fp == NULL)
        return false;

    //read file
    fread(&fileHeader, sizeof(BITMAPFILEHEADER), 1, fp);
    printf("nima %d",sizeof(BITMAPFILEHEADER));
    fread(&infoHeader, sizeof(BITMAPINFOHEADER), 1, fp);
    //init
    bmp.bmWidth = infoHeader.biWidth;
    bmp.bmHeight = infoHeader.biHeight;
    size_t lineSize = (infoHeader.biWidth * infoHeader.biBitCount + 31)/8;
    bmp.bmWidthBytes = lineSize;
    bmp.bmBitsPixel = infoHeader.biBitCount;
    //pixels
    size_t size = lineSize * infoHeader.biHeight;
    bmp.bmBits = malloc(size);
    fseek(fp, fileHeader.bfOffbits, SEEK_SET);
    fread(bmp.bmBits, size, 1, fp);

    fclose(fp);
    return true;
}

unsigned char* readBMP(const char* filename)
{
    int i;
    FILE* f = fopen(filename, "rb");
    unsigned char info[54];
    fread(info, sizeof(unsigned char), 54, f); // read the 54-byte header

    // extract image height and width from header
    int width = *(int*)&info[18];
    int height = *(int*)&info[22];

    int size = 3 * width * height;
    unsigned char* data = new unsigned char[size]; // allocate 3 bytes per pixel
    fread(data, sizeof(unsigned char), size, f); // read the rest of the data at once
    fclose(f);

    return data;
}
