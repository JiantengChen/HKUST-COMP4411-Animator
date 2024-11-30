#include "CatmullRomCurveEvaluator.h"
#include <assert.h>
#include "BezierCurveEvaluator.h"
#include "modelerapp.h"
#include <Eigen/Dense>

void CatmullRomCurveEvaluator::evaluateCurve(const std::vector<Point> &ptvCtrlPts,
                                             std::vector<Point> &ptvEvaluatedCurvePts,
                                             const float &fAniLength,
                                             const bool &bWrap) const
{

    if (ptvCtrlPts.size() < 4)
    {
        // return if there are less than 4 points
        // catmull rom curve requires at least 4 points
        BezierCurveEvaluator *bezierCurveEvaluator = new BezierCurveEvaluator();
        bezierCurveEvaluator->evaluateCurve(ptvCtrlPts, ptvEvaluatedCurvePts, fAniLength, bWrap);
        return;
    }

    int iCtrlPtCount = ptvCtrlPts.size();
    int lastIndex = ptvCtrlPts.size() - 1;
    float tension = ModelerApplication::Instance()->tension;

    ptvEvaluatedCurvePts.clear();

    Point p1, p2, p3, p4, p1_, p2_, p3_, p4_;
    Eigen::Matrix<float, 4, 2> p;
    Eigen::Matrix<float, 4, 4> transform;
    Eigen::Matrix<float, 4, 2> v;

    // first segment
    p1_ = ptvCtrlPts[0];
    p2_ = ptvCtrlPts[0] + ptvCtrlPts[1] - ptvCtrlPts[0];
    p3_ = ptvCtrlPts[1] - (ptvCtrlPts[2] - ptvCtrlPts[0]) * (tension / 3.0);
    p4_ = ptvCtrlPts[1];
    BezierCurveEvaluator::drawBezierSegment(p1_, p2_, p3_, p4_, ptvEvaluatedCurvePts);

    // regular segments
    int i;
    for (i = 0; i < iCtrlPtCount - 3; i++)
    {
        // Q = [t^3, t^2, t, 1] * 0.5 * base_matrix_catmull_rom * P
        // Q = [t^3, t^2, t, 1] * base_matrix_bezier * V
        // V = 0.5 * transform * P
        // base_matrix_bezier * transform = base_matrix_catmull_rom
        // left multiply by inv of base_matrix_bezier, get transform, then get V
        // base for catmull rom curve
        // -1 3 -3  1
        //  2 -5  4 -1
        // -1  0  1  0
        //  0  2  0  0
        // inv for base_matrix_bezier
        // 0 0 0 1
        // 0 0 1/3 1
        // 0 1/3 2/3 1
        // 1 1 1 1
        // transform
        // 0 2 0 0
        // -1/3 2 1/3 0
        // 0 1/3 2 -1/3
        // 0 0 2 0
        p1 = ptvCtrlPts[i];
        p2 = ptvCtrlPts[i + 1];
        p3 = ptvCtrlPts[i + 2];
        p4 = ptvCtrlPts[i + 3];

        p << p1.x, p1.y,
            p2.x, p2.y,
            p3.x, p3.y,
            p4.x, p4.y;

        transform << 0, 2, 0, 0,
            -1.0 / 3.0, 2, 1.0 / 3.0, 0,
            0, 1.0 / 3.0, 2, -1.0 / 3.0,
            0, 0, 2, 0;

        v = 0.5 * transform * p;

        p1_ = Point(v(0, 0), v(0, 1));
        p2_ = Point(v(1, 0), v(1, 1));
        p3_ = Point(v(2, 0), v(2, 1));
        p4_ = Point(v(3, 0), v(3, 1));

        // p1_ = p2;
        // p2_ = p2 + (p3 - p1) * (tension / 3.0);
        // p3_ = p3 - (p4 - p2) * (tension / 3.0);
        // p4_ = p3;

        BezierCurveEvaluator::drawBezierSegment(p1_, p2_, p3_, p4_, ptvEvaluatedCurvePts);
    }

    // last segment
    p1_ = ptvCtrlPts[lastIndex - 1];
    p2_ = ptvCtrlPts[lastIndex - 1] + (ptvCtrlPts[lastIndex] - ptvCtrlPts[lastIndex - 2]) * (tension / 3.0);
    p3_ = ptvCtrlPts[lastIndex] - (ptvCtrlPts[lastIndex] - ptvCtrlPts[lastIndex - 1]);
    p4_ = ptvCtrlPts[lastIndex];
    BezierCurveEvaluator::drawBezierSegment(p1_, p2_, p3_, p4_, ptvEvaluatedCurvePts);

    if (bWrap)
    {
        p1 = ptvCtrlPts[lastIndex - 1];
        p2 = ptvCtrlPts[lastIndex];
        p3 = ptvCtrlPts[0];
        p3.x += fAniLength;
        p4 = ptvCtrlPts[1];
        p4.x += fAniLength;

        p1_ = p2;
        p2_ = p2 + ((p3 - p1) * (tension / 3.0f));
        p3_ = p3 - ((p4 - p2) * (tension / 3.0f));
        p4_ = p3;
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
