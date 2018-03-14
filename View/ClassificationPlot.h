#ifndef CLASSIFICATIONPLOT_H
#define CLASSIFICATIONPLOT_H

#include "Library/View/qcustomplot.h"
#include "Library/Base/Singleton.h"

class ClassificationPlot :
  public QCustomPlot,
  public Singleton<ClassificationPlot> {

Q_OBJECT

public:
  /** \name Constructors/destructor
    @{
    */
  /// Constructs the view
  ClassificationPlot();
  /// Destructor
   ~ClassificationPlot();

protected:
  /** \name Protected methods
    @{
    */
protected slots:

signals:


};

#endif // CLASSIFICATIONPLOT_H
