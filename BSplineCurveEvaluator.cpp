#include "BSplineCurveEvaluator.h"
#include "BezierCurveEvaluator.h"
#include <assert.h>

#include "modelerapp.h"
#include <Eigen/Dense>

void BSplineCurveEvaluator::evaluateCurve(const std::vector<Point> &ptvCtrlPts,
                                          std::vector<Point> &ptvEvaluatedCurvePts,
                                          const float &fAniLength,
                                          const bool &bWrap) const
{
    int iCtrlPtCount = ptvCtrlPts.size();
    ptvEvaluatedCurvePts.clear();

    int i;
    for (i = 0; i <= iCtrlPtCount - 4; i++)
    {
        Point p1 = ptvCtrlPts[i];
        Point p2 = ptvCtrlPts[i + 1];
        Point p3 = ptvCtrlPts[i + 2];
        Point p4 = ptvCtrlPts[i + 3];

        // v = (1/6) * transform * p
        // Q(t) = [t^3, t^2, t, 1] * base_beizer_matrix * v

        Eigen::Matrix<float, 4, 2> p;
        p << p1.x, p1.y,
            p2.x, p2.y,
            p3.x, p3.y,
            p4.x, p4.y;

        Eigen::Matrix<float, 4, 4> transform;
        transform << 1, 4, 1, 0,
            0, 4, 2, 0,
            0, 2, 4, 0,
            0, 1, 4, 1;

        Eigen::Matrix<float, 4, 2> v = (1.0 / 6.0) * transform * p;

        Point p1_ = Point(v(0, 0), v(0, 1));
        Point p2_ = Point(v(1, 0), v(1, 1));
        Point p3_ = Point(v(2, 0), v(2, 1));
        Point p4_ = Point(v(3, 0), v(3, 1));

        BezierCurveEvaluator::drawBezierSegment(p1_, p2_, p3_, p4_, ptvEvaluatedCurvePts);
    }

    if (bWrap)
    {
        Point p1, p2, p3, p4, p1_, p2_, p3_, p4_;
        Eigen::Matrix<float, 4, 2> p;
        Eigen::Matrix<float, 4, 4> transform;
        Eigen::Matrix<float, 4, 2> v;
        transform << 1, 4, 1, 0,
            0, 4, 2, 0,
            0, 2, 4, 0,
            0, 1, 4, 1;

        // segment 1
        p1 = ptvCtrlPts[i];
        p2 = ptvCtrlPts[i + 1];
        p3 = ptvCtrlPts[i + 2];
        p4 = ptvCtrlPts[0];
        p4.x += fAniLength;

        p << p1.x, p1.y,
            p2.x, p2.y,
            p3.x, p3.y,
            p4.x, p4.y;
        v = (1.0 / 6.0) * transform * p;

        p1_ = Point(v(0, 0), v(0, 1));
        p2_ = Point(v(1, 0), v(1, 1));
        p3_ = Point(v(2, 0), v(2, 1));
        p4_ = Point(v(3, 0), v(3, 1));
        BezierCurveEvaluator::drawBezierSegment(p1_, p2_, p3_, p4_, ptvEvaluatedCurvePts);

        for (int k = 0; k < ptvEvaluatedCurvePts.size(); k++)
        {
            if (ptvEvaluatedCurvePts[k].x > fAniLength)
            {
                ptvEvaluatedCurvePts[k].x -= fAniLength;
            }
        }

        // segment 2
        p1 = ptvCtrlPts[i + 1];
        p2 = ptvCtrlPts[i + 2];
        p3 = ptvCtrlPts[0];
        p4 = ptvCtrlPts[1];
        p3.x += fAniLength;
        p4.x += fAniLength;

        p << p1.x, p1.y,
            p2.x, p2.y,
            p3.x, p3.y,
            p4.x, p4.y;
        v = (1.0 / 6.0) * transform * p;

        p1_ = Point(v(0, 0), v(0, 1));
        p2_ = Point(v(1, 0), v(1, 1));
        p3_ = Point(v(2, 0), v(2, 1));
        p4_ = Point(v(3, 0), v(3, 1));
        BezierCurveEvaluator::drawBezierSegment(p1_, p2_, p3_, p4_, ptvEvaluatedCurvePts);

        for (int k = 0; k < ptvEvaluatedCurvePts.size(); k++)
        {
            if (ptvEvaluatedCurvePts[k].x > fAniLength)
            {
                ptvEvaluatedCurvePts[k].x -= fAniLength;
            }
        }

        // segment 3
        p1 = ptvCtrlPts[i + 2];
        p2 = ptvCtrlPts[0];
        p3 = ptvCtrlPts[1];
        p4 = ptvCtrlPts[2];
        p2.x += fAniLength;
        p3.x += fAniLength;
        p4.x += fAniLength;

        p << p1.x, p1.y,
            p2.x, p2.y,
            p3.x, p3.y,
            p4.x, p4.y;
        v = (1.0 / 6.0) * transform * p;

        p1_ = Point(v(0, 0), v(0, 1));
        p2_ = Point(v(1, 0), v(1, 1));
        p3_ = Point(v(2, 0), v(2, 1));
        p4_ = Point(v(3, 0), v(3, 1));

        BezierCurveEvaluator::drawBezierSegment(p1_, p2_, p3_, p4_, ptvEvaluatedCurvePts);

        for (int k = 0; k < ptvEvaluatedCurvePts.size(); k++)
        {
            if (ptvEvaluatedCurvePts[k].x > fAniLength)
            {
                ptvEvaluatedCurvePts[k].x -= fAniLength;
            }
        }
    }
    else
    {
        float x = fAniLength;
        ptvEvaluatedCurvePts.push_back({x, ptvCtrlPts[ptvCtrlPts.size() - 1].y});
        ptvEvaluatedCurvePts.push_back({0, ptvCtrlPts[0].y});
    }
}