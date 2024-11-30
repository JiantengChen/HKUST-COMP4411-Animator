#ifndef INCLUDE_BSPLINE_CURVE_EVALUATOR_H
#define INCLUDE_BSPLINE_CURVE_EVALUATOR_H

#pragma warning(disable : 4786)

#include "CurveEvaluator.h"
class BSplineCurveEvaluator : public CurveEvaluator
{

public:
    void evaluateCurve(const std::vector<Point> &ptvCtrlPts,
                       std::vector<Point> &ptvEvaluatedCurvePts,
                       const float &fAniLength,
                       const bool &bWrap) const;
};

#endif