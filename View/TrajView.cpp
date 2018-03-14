#include "TrajView.h"

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

TrajView::TrajView(QWidget* parent) :
    QOpenGLWidget(parent),
    mFont(0),
    mMouse(2, 0),
    mViewport(4, 0),
    mProjection(16, 0.0),
    mModelview(16, 0.0) {
  setFont("/usr/share/fonts/truetype/msttcorefonts/Arial.ttf");
  connect(&mCamera, SIGNAL(positionChanged(const std::vector<double>&)), this,
    SLOT(cameraPositionChanged(const std::vector<double>&)));
  connect(&mCamera, SIGNAL(viewpointChanged(const std::vector<double>&)), this,
    SLOT(cameraViewpointChanged(const std::vector<double>&)));
  connect(&mCamera, SIGNAL(rangeChanged(const std::vector<double>&)), this,
    SLOT(cameraRangeChanged(const std::vector<double>&)));
  connect(&mScene, SIGNAL(translationChanged(const std::vector<double>&)), this,
    SLOT(sceneTranslationChanged(const std::vector<double>&)));
  connect(&mScene, SIGNAL(rotationChanged(const std::vector<double>&)), this,
    SLOT(sceneRotationChanged(const std::vector<double>&)));
  connect(&mScene, SIGNAL(scaleChanged(double)), this,
    SLOT(sceneScaleChanged(double)));
  connect(&TrajView::getInstance().getScene(), SIGNAL(render(TrajView&, Scene3d&)),this, SLOT(render(TrajView&, Scene3d&)));
  setBackgroundColor(Qt::white);
  }

TrajView::~TrajView() {
  if (mFont)
    delete mFont;
}

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

Camera& TrajView::getCamera() {
  return mCamera;
}

const Camera& TrajView::getCamera() const {
  return mCamera;
}

Scene3d& TrajView::getScene() {
  return mScene;
}

const Scene3d& TrajView::getScene() const {
  return mScene;
}

void TrajView::setColor(const QColor& color) {
  glColor4f(color.redF(), color.greenF(), color.blueF(), color.alphaF());
}

void TrajView::setColor(const Palette& palette, const QString& role) {
  setColor(palette.getColor(role));
}

void TrajView::setFont(const QString& filename) {
  if (mFontFilename != filename) {
    if (mFont) {
      delete mFont;
      mFont = 0;
    }
    mFontFilename = filename;
    emit fontChanged(mFontFilename);
    update();
  }
}

const QString& TrajView::getFont() const {
  return mFontFilename;
}


void TrajView::setBackgroundColor(const QColor& color) {
  mPalette.setColor("Background", color);
}


/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

std::vector<double> TrajView::unproject(const QPoint& point, double distance) {
  std::vector<double> result(3, 0.0);
  double near = mCamera.getRange()[0];
  double far = mCamera.getRange()[1];
  double z = (1.0 / near - 1.0 / distance) / (1.0 / near - 1.0 / far);
  gluUnProject(point.x(), -point.y(), z, &mModelview[0], &mProjection[0],
    &mViewport[0], &result[0], &result[1], &result[2]);
  return result;
}

void TrajView::render(double x, double y, double z, const QString& text,
    double minScale, double maxScale, bool faceX, bool faceY, bool faceZ) {
  if (!mFont) {
    QFileInfo fileInfo(mFontFilename);
    if (fileInfo.isFile() && fileInfo.isReadable()) {
      mFont = new FTPolygonFont(mFontFilename.toLatin1().constData());
      mFont->UseDisplayList(false);
      mFont->FaceSize(1);
    }
  }
  if (mFont) {
    glPushMatrix();
    glTranslatef(x, y, z);
    if (faceZ)
      glRotatef(-mScene.getRotation()[0] * 180.0 / M_PI, 0, 0, 1);
    if (faceY)
      glRotatef(-mScene.getRotation()[1] * 180.0 / M_PI, 0, 1, 0);
    if (faceX)
      glRotatef(-mScene.getRotation()[2] * 180.0 / M_PI, 1, 0, 0);
    glRotatef(90.0, 1, 0, 0);
    glRotatef(-90.0, 0, 1, 0);
    const double scale = std::max(minScale, std::min(maxScale,
      mScene.getScale()));
    glScalef(scale / mScene.getScale(), scale / mScene.getScale(),
      scale / mScene.getScale());
    mFont->Render(text.toLatin1().constData());
    glPopMatrix();
  }
}

void TrajView::mousePressEvent(QMouseEvent* event) {
  mMouse[0] = event->globalX();
  mMouse[1] = event->globalY();
}

void TrajView::mouseMoveEvent(QMouseEvent* event) {
  int deltaX = event->globalX() - mMouse[0];
  int deltaY = event->globalY() - mMouse[1];
  if (event->buttons() == Qt::LeftButton) {
      mScene.setRotation(
      mScene.getRotation()[0] - M_PI / width() * deltaX,
      mScene.getRotation()[1] + M_PI / height() * deltaY,
      mScene.getRotation()[2]);
  }
  else if (event->buttons() == Qt::RightButton) {
    QPoint mouseLocal = mapFromGlobal(QPoint(mMouse[0], mMouse[1]));
    QPoint eventLocal = mapFromGlobal(QPoint(event->globalPos()));
    double distance = mCamera.getViewpointDistance();
    std::vector<double> mouseUnprojected = unproject(mouseLocal, distance);
    std::vector<double> eventUnprojected = unproject(eventLocal, distance);
    mScene.setTranslation(
      mScene.getTranslation()[0] + (eventUnprojected[0] -
        mouseUnprojected[0]),
      mScene.getTranslation()[1] + (eventUnprojected[1] -
        mouseUnprojected[1]),
      mScene.getTranslation()[2] + (eventUnprojected[2] -
        mouseUnprojected[2]));
  }
  mMouse[0] = event->globalX();
  mMouse[1] = event->globalY();
}

void TrajView::wheelEvent(QWheelEvent* event) {
  double deltaScale = 1e-2;
  mScene.setScale(mScene.getScale() * (1.0 + deltaScale * event->delta() /
    8.0));
}

void TrajView::initializeGL() {
  glEnable(GL_DEPTH_TEST);
  glEnable (GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glShadeModel(GL_SMOOTH);
  glDepthFunc(GL_LEQUAL);
  glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
  emit createDisplayLists();
}

void TrajView::resizeGL(int width, int height) {
  glViewport(0, 0, width, height);
  mCamera.setup(*this, width, height);
}

void TrajView::paintGL() {
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  mCamera.setup(*this, width(), height());
  mScene.setup(*this);
  glGetIntegerv(GL_VIEWPORT, &mViewport[0]);
  glGetDoublev(GL_PROJECTION_MATRIX, &mProjection[0]);
  glGetDoublev(GL_MODELVIEW_MATRIX, &mModelview[0]);
  mScene.render(*this);
}

void TrajView::paintEvent(QPaintEvent* event) {
  QOpenGLWidget::paintEvent(event);
  emit updated();
}

void TrajView::cameraPositionChanged(const std::vector<double>& position) {
  update();
}

void TrajView::cameraViewpointChanged(const std::vector<double>& viewpoint) {
  update();
}

void TrajView::cameraRangeChanged(const std::vector<double>& range) {
  update();
}

void TrajView::sceneTranslationChanged(const std::vector<double>& translation) {
  update();
}

void TrajView::sceneRotationChanged(const std::vector<double>& rotation) {
  update();
}

void TrajView::sceneScaleChanged(double scale) {
  update();
}

void TrajView::resizeEvent(QResizeEvent* event) {
  QOpenGLWidget::resizeEvent(event);
  emit resized();
}


void TrajView::renderBackground() {
  glPushAttrib(GL_CURRENT_BIT);
  QColor backgroundColor = mPalette.getColor("Background");
  glClearColor(backgroundColor.redF(), backgroundColor.greenF(),
    backgroundColor.blueF(), 0.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPopAttrib();
}


void TrajView::render(TrajView& view, Scene3d& Scene3d) {
     renderBackground();
}
