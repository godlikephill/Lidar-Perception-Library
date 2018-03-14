#include "SafetyPlot.h"


SafetyPlot::SafetyPlot()
{

    this->addGraph();// blue line
    this->graph(0)->setPen(QPen(QColor(40, 110, 255)));
    this->addGraph(); // red line
    this->graph(1)->setPen(QPen(QColor(255, 110, 40)));

    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    timeTicker->setTimeFormat("%m:%s");
    this->xAxis->setTicker(timeTicker);
    this->axisRect()->setupFullAxesBox();
    this->yAxis->setRange(0, 10);

    // make left and bottom axes transfer their ranges to right and top axes:
    connect(this->xAxis, SIGNAL(rangeChanged(QCPRange)), this->xAxis2, SLOT(setRange(QCPRange)));
    connect(this->yAxis, SIGNAL(rangeChanged(QCPRange)), this->yAxis2, SLOT(setRange(QCPRange)));
    //connect(&datatimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlot()));
}



SafetyPlot::~SafetyPlot()
{

}
