#pragma once

#include <vector>

#include <Standard_Handle.hxx>
#include <gp_Ax1.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Edge.hxx>
#include <TColStd_PackedMapOfInteger.hxx>

class Geom_Curve;
class Geom_Surface;
class Geom_ToroidalSurface;
class Geom_CylindricalSurface;
class Geom_ConicalSurface;
class Geom_SurfaceOfRevolution;
class Geom_SphericalSurface;
class Geom_Plane;
class Geom_BSplineSurface;

namespace FeatureCommon
{

    /*
    * @brief find common edges between face1 and face2
    * @param face1
    * @param face2
    * @return common edges between face1 and face2
    */
    std::vector<TopoDS_Edge> FindCommonEdges(const TopoDS_Face& face1,
                                            const TopoDS_Face& face2);
    /*
     * @brief calculate Angle between Faces
     * @note face1 and face2 must share edge
     * @param face1
     * @param face2
     * @param[out] norm1
     * @param[out] norm2
     * @return angle between face1 and face2 if they share edge; 0 otherwise
     */
    double CalculateAngleBetweenFaces(const TopoDS_Face& face1, const TopoDS_Face& face2,
                                  gp_Dir& outNormal1, gp_Dir& outNormal2);
    /*
     * @brief is two conic are geometric same
     * @param faceA with conic
     * @param faceB with conic
     * @return true faceA and faceB are geometric same; false otherwise
     */
    bool IsSameConic(const TopoDS_Face& face, const TopoDS_Face& partner);

    /*
    * @brief calculate edge length
    * @param edge
    * @return length of edge
    */
    double CalculateEdgeLength(const TopoDS_Edge& edge);

    /*
     * @brief calculate face area
     * @param face
     * @return area of edge
     */
    double CalculateFaceArea(const TopoDS_Face& face);

    /*
    * @brief judge two edges have common vertex
    * @param edge0
    * @param edge1
    * @return true if they have common vertex; false otherwise
    */
    bool HaveCommonVertex(const TopoDS_Edge& edge0, const TopoDS_Edge& edge1);

    bool IsSingleSidedFace(const TopoDS_Face& face);

    /*
    * @brief sample point on curve
    * @param start param
    * @param end param
    * @param sample nums
    * @param[out] sampled points
    * @param[out] sampled params
    */
void SampleCurvePoints(const Handle(Geom_Curve) & C, double u0, double u1, int nSamples,
                       std::vector<gp_Pnt>& outPts, std::vector<double>* outParams = nullptr);

    /*
     * @brief compare two double numbers
     * @param a, num1
     * @param b, num2
     * @return -1, a<b; 0, a==b; 1, a>b
     */
    int DoubleCompare(const double a, const double b);

    /*
     * @brief print integer set
     * @param integer set
     * @param set name
     */
    void Print(const TColStd_PackedMapOfInteger& nums, const std::string& setName);

    /*
     * @brief judge if curve is circle, or circle like
     * @param curve
     * @param low param
     * @param high param
     * @param[out] axis
     * @param[out] radius
     * @param[out] confusion
     * @return true if curve is circle or circle like and calculate the direction, radius; false otherwise
     */
    bool IsCircle(Handle(Geom_Curve) curve, double low, double high, gp_Ax1& outAxis,
              double& outRadius, double& outConfusion);

    /*
     * @brief judge if edge is circle, or circle like
     * @param edge
     * @param[out] axis
     * @param[out] radius
     * @param[out] confusion
     * @return true if edge is circle or circle like and calculate the direction, radius; false
     * otherwise
     */
    bool IsCircle(const TopoDS_Edge& edge, gp_Ax1& outAxis, double& outRadius,
                  double& outConfusion);

    double LargeAngle();
    

    double Angular();

    double Confusion();

    // 角度转弧度
    double DegreesToRadians(double degrees);
    
    // 弧度转角度
    double RadiansToDegrees(double radians);

    bool IsBiCoaxial(const gp_Ax1& currentAxis, const gp_Ax1& axis, double angularTol,
                     double confusion);


    /*
     * @brief calculate curvature along curve and perpendicular curve on t
     * @param face
     * @param edge
     * @param t, parameter on edge
     * @param[out] UV
     * @param[out] along curvature
     * @param[out] perpendicular curvature
     * @return true if curvature defined; false otherwise
     */
    bool EvaluateCurvature(const TopoDS_Face& face, const TopoDS_Edge& edge, const double t,
                           gp_Pnt2d& UV, double& alongCurvature, double& perpendCurvature);

    /*
     * @brief calculate curvature along curve and perpendicular curve on t for analysis surface
     * @param face
     * @param edge
     * @param t, parameter on edge
     * @param[out] UV
     * @param[out] along curvature
     * @param[out] perpendicular curvature
     * @return true if curvature defined; false otherwise
     */
    bool EvaluateCurvatureAnalysis(const TopoDS_Face& face, const TopoDS_Edge& edge,
                                   const double t, gp_Pnt2d& UV, double& alongCurvature,
                                   double& perpendCurvature);

    /*
     * @brief calculate curvature along curve and perpendicular curve on t for analysis surface
     * @param face
     * @param edge
     * @param[out] along curvature
     * @param[out] perpendicular curvature
     * @return true if curvature defined; false otherwise
     */
    bool EvaluateCurvatureAnalysis(const TopoDS_Face& face, const TopoDS_Edge& edge,
                                   double& alongCurvature, double& perpendCurvature);

    /*
     * @brief judge surface or its basis surface is conical surface
     * @param surface
     * @return true if surface or its basis surface is conical surface; false otherwise
     */
    bool IsConicalSurface(Handle(Geom_Surface) surface);

    /*
     * @brief judge face is conical face
     * @param face
     * @return true if face is conical face; false otherwise
     */
    bool IsConicFace(const TopoDS_Face& face);

    /*
    * @brief judge surface or its basis surface is cylindrical surface
    * @param surface
    * @return true if surface or its basis surface is cylindrical surface; false otherwise
    */
    bool IsCylindricalSurface(Handle(Geom_Surface) surface);

    /*
     * @brief judge face is cylindrical face
     * @param face
     * @return true if face is cylindrical face; false otherwise
     */
    bool IsCylindricalFace(const TopoDS_Face& face);

    /*
     * @brief judge surface or its basis surface is offset surface
     * @param surface
     * @return true if surface or its basis surface is offset surface; false otherwise
     */
    bool IsOffsetSurface(Handle(Geom_Surface) surface);

    /*
     * @brief judge surface or its basis surface is toroidal surface
     * @param surface
     * @return true if surface or its basis surface is toroidal surface; false otherwise
     */
    bool IsToroidalSurface(Handle(Geom_Surface) surface);

    /*
     * @brief judge surface or its basis surface is spherical surface
     * @param surface
     * @return true if surface or its basis surface is spherical surface; false otherwise
     */
    bool IsSphericalSurface(Handle(Geom_Surface) surface);

    /*
     * @brief judge surface or its basis surface is bspline surface
     * @param surface
     * @return true if surface or its basis surface is bspline surface; false otherwise
     */
    bool IsBSplineSurface(Handle(Geom_Surface) surface);

    /*
     * @brief judge surface or its basis surface is extrusion
     * @param surface
     * @return true if surface or its basis surface is extrusion; false otherwise
     */
    bool IsExtrusionSurface(Handle(Geom_Surface) surface);

    /*
     * @brief judge face is extrusion
     * @param face
     * @return true if face is extrusion; false otherwise
     */
    bool IsExtrusionFace(const TopoDS_Face& face);

    /*
     * @brief judge surface or its basis surface is bezier surface
     * @param surface
     * @return true if surface or its basis surface is bezier surface; false otherwise
     */
    bool IsBezierSurface(Handle(Geom_Surface) surface);

    /*
     * @brief judge surface or its basis surface is surface of revolution
     * @param surface
     * @return true if surface or its basis surface is surface of revolution; false otherwise
     */
    bool IsSurfaceOfRevolution(Handle(Geom_Surface) surface);

    /*
     * @brief judge surface or its basis surface is plane
     * @param surface
     * @return true if surface or its basis surface is plane; false otherwise
     */
    bool IsPlane(Handle(Geom_Surface) surface);

    /*
     * @brief judge face is plane
     * @param face
     * @return true if face is plane; false otherwise
     */
    bool IsPlane(const TopoDS_Face& face);

    /*
    * @brief fetch toroidal surface from Geom_surfce
    * @param surface
    * @return Geom_ToroidalSurface
    */
    Handle(Geom_ToroidalSurface) FetchToroidalSurface(Handle(Geom_Surface) surface);

    /*
     * @brief fetch bspline surface from Geom_surfce
     * @param surface
     * @return Geom_BsplineSurface
     */
    Handle(Geom_BSplineSurface) FetchBsplineSurface(Handle(Geom_Surface) surface);

    /*
     * @brief fetch spherical surface from Geom_surfce
     * @param surface
     * @return Geom_SphericalSurface
     */
    Handle(Geom_SphericalSurface) FetchSphericalSurface(Handle(Geom_Surface) surface);

    /*
     * @brief fetch surface of revolution from Geom_surfce
     * @param surface
     * @return Geom_SurfaceOfRevolution
     */
    Handle(Geom_SurfaceOfRevolution) FetchSurfaceOfRevolution(Handle(Geom_Surface) surface);

    /*
     * @brief fetch cylindrical surface from Geom_surfce
     * @param surface
     * @return Geom_CylindricalSurface
     */
    Handle(Geom_CylindricalSurface) FetchCylindricalSurface(Handle(Geom_Surface) surface);

    /*
     * @brief fetch cylindrical surface from cylindrical face
     * @param face
     * @return Geom_CylindricalSurface
     */
    Handle(Geom_CylindricalSurface) FetchCylindricalSurface(const TopoDS_Face& face);

    /*
     * @brief fetch plane from face
     * @param face
     * @return Geom_Plane
     */
    Handle(Geom_Plane) FetchPlane(const TopoDS_Face& face);

    /*
     * @brief fetch plane from surface
     * @param surface
     * @return Geom_Plane
     */
    Handle(Geom_Plane) FetchPlane(Handle(Geom_Surface) surface);

    /*
     * @brief fetch conical surface from Geom_surfce
     * @param surface
     * @return Geom_ConicalSurface
     */
    Handle(Geom_ConicalSurface) FetchConicalSurface(Handle(Geom_Surface) surface);

    /*
     * @brief fetch conical surface from face
     * @param face
     * @return Geom_ConicalSurface
     */
    Handle(Geom_ConicalSurface) FetchConicalSurface(const TopoDS_Face& face);

    /**
     * 将点投影到曲面，获取最近的UV坐标
     * @param point 要投影的3D点
     * @param surface 目标曲面
     * @param uv 输出的UV坐标
     * @param tolerance 容差
     * @return 投影是否成功
     */
    bool ProjectPointToSurface(const gp_Pnt& point, const TopoDS_Face& face,
                               gp_Pnt2d& outUV, double tolerance = 1e-6);

    /*
     * @brief get mid uv by curve2d
     * @param face
     * @param edge
     * @param[out] outUV
     * @return true if calc success; false otherwise
     */
    bool GetMidUVByCurve2d(const TopoDS_Face& face, const TopoDS_Edge& edge,
                                          gp_Pnt2d& outUV);

    /*
    * @brief create othorgnal curve on mid point
    * @param face
    * @param edge
    * @return othognal curve
    */
    TopoDS_Edge CreateOthorgnalCurveOnMidPoint(const TopoDS_Face& face,
                                                  const TopoDS_Edge& edge);
    
    /*
     * @brief judge if curve is straight, or straight like
     * @param curve
     * @param parameter range of low
     * @param parameter range of high
     * @param outDir the direction of straight
     * @return true if curve is straight or straight like and calculate the direction; false
     * otherwise
     */
    bool IsStraight(Handle(Geom_Curve) curve, double low, double high, gp_Dir& outDir);

    /*
     * @brief judge if edge is straight, or straight like
     * @param edge
     * @param[out] axis of line
     * @return true if edge is straight or straight like and calculate the axis; false otherwise
     */
    bool IsStraight(const TopoDS_Edge& edge, gp_Ax1& outAxis);

    bool IsAxesCoplanar(const gp_Ax1& axis1, const gp_Ax1& axis2,
                         double angularTol = FeatureCommon::Angular(),
                         double linearTol = FeatureCommon::Confusion());

};
