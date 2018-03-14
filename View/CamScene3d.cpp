#include "CamScene3d.h"
#include <cmath>
#include "Library/View/CamView3d.h"

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

CamScene3d::CamScene3d() :
    mTranslation(3, 0.0),
    mRotation(3, 0.0),
    mScale(1.0) {
  setTranslation(0.0, 0.0, 0.0);
  setRotation(-10.0 * M_PI / 180.0, -5.0 * M_PI / 180.0, 0.0);
  setScale(1.0);
}

CamScene3d::~CamScene3d() {
}

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

void CamScene3d::setTranslation(double x, double y, double z) {
  if ((x != mTranslation[0]) || (y != mTranslation[1]) ||
      (z != mTranslation[2])) {
    mTranslation[0] = x;
    mTranslation[1] = y;
    mTranslation[2] = z;
    emit translationChanged(mTranslation);
  }
}

const std::vector<double>& CamScene3d::getTranslation() const {
  return mTranslation;
}

void CamScene3d::setRotation(double yaw, double pitch, double roll) {
  if ((yaw != mRotation[0]) || (pitch != mRotation[1]) ||
      (roll != mRotation[2])) {
    mRotation[0] = correctAngle(yaw);
    mRotation[1] = correctAngle(pitch);
    mRotation[2] = correctAngle(roll);
    emit rotationChanged(mRotation);
  }
}

const std::vector<double>& CamScene3d::getRotation() const {
  return mRotation;
}

void CamScene3d::setScale(double scale) {
  if (mScale != scale) {
    mScale = scale;
    emit scaleChanged(mScale);
  }
}

double CamScene3d::getScale() const {
  return mScale;
}

/******************************************************************************/
/*  Methods                                                                   */
/******************************************************************************/

void CamScene3d::setup(CamView3d& view) {
  glMatrixMode(GL_MODELVIEW);
  glScalef(mScale, mScale, mScale);
  glRotatef(mRotation[2] * 180.0 / M_PI, 1, 0, 0);
  glRotatef(mRotation[1] * 180.0 / M_PI, 0, 1, 0);
  glRotatef(mRotation[0] * 180.0 / M_PI, 0, 0, 1);
  glTranslatef(mTranslation[0], mTranslation[1], mTranslation[2]);
}


void CamScene3d::render(CamView3d& view) {
  emit render(view, *this);
}

double CamScene3d::correctAngle(double angle) const {
  if (angle >= 0.0)
    while (angle >= M_PI) angle -= 2.0 * M_PI;
  else
    while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}
