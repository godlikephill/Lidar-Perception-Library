#ifndef CARNUMPLOT_H
#define CARNUMPLOT_H

#include "Library/View/qcustomplot.h"
#include "Library/Base/Singleton.h"

class CarNumPlot :
  public QCustomPlot,
  public Singleton<CarNumPlot> {

Q_OBJECT

public:
  /** \name Constructors/destructor
    @{
    */
  /// Constructs the view
  CarNumPlot();
  /// Destructor
   ~CarNumPlot();

protected:
  /** \name Protected methods
    @{
    */
protected slots:

signals:


};


#endif // CARNUMPLOT_H
