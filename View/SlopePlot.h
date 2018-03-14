#ifndef SLOPEPLOT_H
#define SLOPEPLOT_H


#include "Library/View/qcustomplot.h"
#include "Library/Base/Singleton.h"

class SlopePlot :
  public QCustomPlot,
  public Singleton<SlopePlot> {

Q_OBJECT

public:
  /** \name Constructors/destructor
    @{
    */
  /// Constructs the view
  SlopePlot();
  /// Destructor
   ~SlopePlot();

protected:
  /** \name Protected methods
    @{
    */
protected slots:

signals:


};


#endif // SLOPEPLOT_H
