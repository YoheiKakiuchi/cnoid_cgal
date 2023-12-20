#ifndef __CHOREONOID_CGAL_LIB_H__
#define __CHOREONOID_CGAL_LIB_H__

#include <cnoid/SceneGraph>
#include <cnoid/SceneDrawables>

#include "exportdecl.h"

namespace cnoid
{

typedef SgVectorArray<Vector3> SgVertexArrayDbl;
typedef ref_ptr<SgVertexArrayDbl> SgVertexArrayDblPtr;
class SgMeshDbl;
typedef ref_ptr<SgMeshDbl> SgMeshDblPtr;
class CGALMesh;
typedef ref_ptr<CGALMesh> CGALMeshPtr;

class CNOID_EXPORT SgMeshDbl : public SgMesh
{
public:
    SgMeshDbl();
    SgMeshDbl(const SgMesh &_sgmesh, CloneMap *cloneMap);
    SgMeshDbl(const SgMesh &_sgmesh) : SgMeshDbl(_sgmesh, nullptr) {};
    ~SgMeshDbl();

    bool hasVerticesDbl() const { return (dbl_vertices_ && !dbl_vertices_->empty()); }
    SgVertexArrayDbl* verticesDbl() { return dbl_vertices_; }
    const SgVertexArrayDbl* verticesDbl() const { return dbl_vertices_; }
    SgVertexArrayDbl* setVerticesDbl(SgVertexArrayDbl* vertices); // downC
    SgVertexArrayDbl* getOrCreateVerticesDbl(int size = -1); // upC

    bool upConvert();
    bool downConvert(bool resetOthers = false);

    static SgMeshDblPtr copyFrom(const SgMesh &_sgmesh);

    void transform(const Isometry3 &T);
protected:
    //SgMeshPtr original_;
    SgVertexArrayDblPtr dbl_vertices_;
};

class CNOID_EXPORT CGALMesh : public SgMeshDbl
{
public:
    CGALMesh();
    CGALMesh(const SgMesh &_sgmesh, CloneMap *cloneMap);
    CGALMesh(const SgMesh &_sgmesh) : CGALMesh(_sgmesh, nullptr) {};

    ~CGALMesh();
    static CGALMeshPtr copyFrom(const SgMesh &_sgmesh);
    static CGALMeshPtr copyFrom(const SgMesh &_sgmesh, const Isometry3 &T);

    bool updateSelfByCGAL();
    bool updateCGAL();
    void transform(const Isometry3 &T);
    double volume();

    bool booleanUnion(const CGALMeshPtr target);
    bool booleanUnion(const CGALMeshPtr target, const Isometry3 &T);
    bool booleanDifference(const CGALMeshPtr target);
    bool booleanDifference(const CGALMeshPtr target, const Isometry3 &T);
    bool booleanIntersection(const CGALMeshPtr target);
    bool booleanIntersection(const CGALMeshPtr target, const Isometry3 &T);

    CGALMeshPtr createByUnion(const CGALMeshPtr target) const;
    CGALMeshPtr createByUnion(const CGALMeshPtr target, const Isometry3 &T) const;
    CGALMeshPtr createByDifference(const CGALMeshPtr target) const;
    CGALMeshPtr createByDifference(const CGALMeshPtr target, const Isometry3 &T) const;
    CGALMeshPtr createByIntersection(const CGALMeshPtr target) const;
    CGALMeshPtr createByIntersection(const CGALMeshPtr target, const Isometry3 &T) const;

    bool checkInside(const Vector3 &p);
    bool checkInside(const Vector3f &p);
    bool checkInside(const SgPointSet &pt, std::vector<int> &_result);

    // for Octomap
    bool generateInsidePoints(double resolution, const std::vector<int> &start_end_xyz,
                              const Vector3 &offset, const Vector3 &scale, std::vector<Vector3> &result);
    // for MergeBoxes
    bool generateInsidePointsIndices(size_t x_length, size_t y_length, size_t z_length,
                                     const Vector3 &size, const Vector3 &offset, std::vector<int> &indices);
private:
    class CGALObj;
    CGALObj *object;
};

}
#endif // __CHOREONOID_CGAL_LIB_H__
