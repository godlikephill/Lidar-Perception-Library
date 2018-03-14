#ifndef TRAJVIEW_H
#define TRAJVIEW_H

#include <vector>
#include <OpenGL/glu.h>
#include <OpenGL/gl.h>
#include <QOpenGLWidget>
#include <QtGui/QColor>
#include <QtCore/QString>
#include <QtCore/QPoint>
#include <cmath>
#include <QtCore/QFileInfo>
#include <QMouseEvent>
#include <QWheelEvent>
#include <FTGL/ftgl.h>
#include "Library/Base/Singleton.h"
#include "Library/View/Palette.h"
#include "Library/View/Camera.h"
#include "Library/View/Scene3d.h"


class FTPolygonFont;

/** The View3d class represents a 3d view.
    \brief 3d view
  */
class TrajView :
  public QOpenGLWidget,
  public Singleton<TrajView> {

Q_OBJECT

  /** \name Private constructors
    @{
    */
  /// Copy constructor
  TrajView(const TrajView& other);
  /// Assignment operator
  TrajView& operator = (const TrajView& other);
  /** @}
    */

public:
  /** \name Constructors/destructor
    @{
    */
  /// Constructs the view
  TrajView(QWidget* parent = 0);
  /// Destructor
  virtual ~TrajView();
  /** @}
    */

  /** \name Accessors
    @{
    */
  /// Returns the camera
  Camera& getCamera();
  /// Returns the camera
  const Camera& getCamera() const;
  /// Returns the scene
  Scene3d& getScene();
  /// Returns the scene
  const Scene3d& getScene() const;
  /// Sets a color
  void setColor(const QColor& color);
  /// Sets a color
  void setColor(const Palette& palette, const QString& role);
  /// Sets the font
  void setFont(const QString& filename);
  /// Returns the font
  const QString& getFont() const;
  /// Sets the background color
  void setBackgroundColor(const QColor& color);

  /** @}
    */

  /** \name Methods
    @{
    */
  /// Unproject a point
  std::vector<double> unproject(const QPoint& point, double distance);
  /// Render text in the scene
  void render(double x, double y, double z, const QString& text,
    double minScale = 0.0, double maxScale = 1.0, bool faceX = false,
    bool faceY = false, bool faceZ = false);
  /** @}
    */
   ///Camera mCamera;
protected:
  /** \name Protected methods
    @{
    */
  /// Mouse press event
  virtual void mousePressEvent(QMouseEvent* event);
  /// Mouse move event
  virtual void mouseMoveEvent(QMouseEvent* event);
  /// Mouse wheel event
  virtual void wheelEvent(QWheelEvent* event);
  /// Init the OpenGL engine
  virtual void initializeGL();
  /// Resize the OpenGL engine
  virtual void resizeGL(int width, int height);
  /// Paint in the OpenGL engine
  virtual void paintGL();
  /// Paint event
  virtual void paintEvent(QPaintEvent* event);
  /// Resize event
  virtual void resizeEvent(QResizeEvent* event);
  /// Render the background
  void renderBackground();
  /// Render tEgoCar
  void renderEgoCar();
  /** @}
    */

  /** \name Protected members
    @{
    */
  /// Filename for the font
  QString mFontFilename;
  /// Font object
  FTPolygonFont* mFont;
  /// Camera
  Camera mCamera;
  /// Scene
  Scene3d mScene;
  /// Mouse
  std::vector<int> mMouse;
  /// Viewport
  std::vector<int> mViewport;
  /// Projection
  std::vector<double> mProjection;
  /// Model view
  std::vector<double> mModelview;
  /// Palette
  Palette mPalette;

  /** @}
    */


protected slots:
  /** \name Qt slots
    @{
    */
  /// Camera position changed
  void cameraPositionChanged(const std::vector<double>& position);
  /// Camera viewpoint changed
  void cameraViewpointChanged(const std::vector<double>& viewpoint);
  /// Camera range changed
  void cameraRangeChanged(const std::vector<double>& range);
  /// Scene translation changed
  void sceneTranslationChanged(const std::vector<double>& translation);
  /// Scene rotation changed
  void sceneRotationChanged(const std::vector<double>& rotation);
  /// Scene scale changed
  void sceneScaleChanged(double scale);
  /// Render the scene
  void render(TrajView& view, Scene3d& scene);
  /** @}
    */

signals:
  /** \name Qt signals
    @{
    */
  /// Font changed
  void fontChanged(const QString& filename);
  /// Updated signal
  void updated();
  /// Clients should create display lists
  void createDisplayLists();
 /// Resized signal
  void resized();
  /** @}
    */

};


#endif // TRAJVIEW_H
