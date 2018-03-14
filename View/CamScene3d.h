#ifndef CAMSCENE3D_H
#define CAMSCENE3D_H

#include <vector>

#include <QtCore/QObject>

class CamView3d;

/** The Scene3d class represents a 3d scene.
    \brief 3d scene
  */
class CamScene3d :
  public QObject {

Q_OBJECT

  /** \name Private constructors
    @{
    */
  /// Copy constructor
  CamScene3d(const CamScene3d& other);
  /// Assignment operator
  CamScene3d& operator = (const CamScene3d& other);
  /** @}
    */

public:
  /** \name Constructors/destructor
    @{
    */
  /// Default constructor
  CamScene3d();
  /// Destructor
  virtual ~CamScene3d();
  /** @}
    */

  /** \name Accessors
    @{
    */
  /// Sets the scene translation
  void setTranslation(double x, double y, double z);
  /// Returns the scene translation
  const std::vector<double>& getTranslation() const;
  /// Sets the scene rotation
  void setRotation(double yaw, double pitch, double roll);
  /// Returns the scene rotation
  const std::vector<double>& getRotation() const;
  /// Sets the scene scale
  void setScale(double scale);
  /// Returns the scene scale
  double getScale() const;
  /** @}
    */

  /** \name Methods
    @{
    */
  /// Setup view
  void setup(CamView3d& view);
  /// Render the scene
  void render(CamView3d& view);
  /** @}
    */

protected:
  /** \name Protected methods
    @{
    */
  /// Correct angles
  double correctAngle(double angle) const;
  /** @}
    */

  /** \name Protected members
    @{
    */
  /// Scene translation
  std::vector<double> mTranslation;
  /// Scene rotation
  std::vector<double> mRotation;
  /// Scene scale
  double mScale;
  /** @}
    */

signals:
  /** \name Qt signals
    @{
    */
  /// Translation has changed
  void translationChanged(const std::vector<double>& translation);
  /// Rotation has changed
  void rotationChanged(const std::vector<double>& rotation);
  /// Scale has changed
  void scaleChanged(double scale);
  /// Render the scene
  void render(CamView3d& view, CamScene3d& scene);
  /** @}
    */

};

#endif // CAMSCENE3D_H
