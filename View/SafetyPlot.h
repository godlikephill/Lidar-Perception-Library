#ifndef SAFETYPLOT_H
#define SAFETYPLOT_H

#include "Library/View/qcustomplot.h"
#include "Library/Base/Singleton.h"

class SafetyPlot :
  public QCustomPlot,
  public Singleton<SafetyPlot> {

Q_OBJECT

public:
  /** \name Constructors/destructor
    @{
    */
  /// Constructs the view
  SafetyPlot();
  /// Destructor
   ~SafetyPlot();

protected:
  /** \name Protected methods
    @{
    */
protected slots:

signals:


};




#endif // SAFETYPLOT_H
