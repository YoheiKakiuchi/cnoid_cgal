#include "CnoidCGAL.h"
#include <cnoid/CloneMap>
#include <cnoid/MeshFilter>
#include "CGALUtil.h"

#ifdef DEBUG_MSG
#include <iostream>
#define DEBUG_STREAM(args) std::cerr << __PRETTY_FUNCTION__ << args << std::endl;
#define DEBUG_PRINT(args) std::cerr << args << std::endl;
#else
#define DEBUG_STREAM(args)
#define DEBUG_PRINT(args)
#endif

using namespace std;
using namespace cnoid;

//////
//
// SgMeshDbl
//
//////
SgMeshDbl::SgMeshDbl() : SgMesh(), dbl_vertices_(nullptr)
{
}
SgMeshDbl::SgMeshDbl(const SgMesh &_sgmesh, CloneMap *cloneMap) : SgMesh(_sgmesh, cloneMap)
{
    dbl_vertices_ = nullptr;
    upConvert();
}
SgMeshDbl::~SgMeshDbl()
{
    if(dbl_vertices_){
        dbl_vertices_->removeParent(this);
    }
}
SgMeshDblPtr SgMeshDbl::copyFrom(const SgMesh &_sgmesh)
{
    CloneMap cm;
    SgMeshDblPtr ptr(new SgMeshDbl(_sgmesh, &cm));
    return ptr;
}
SgVertexArrayDbl* SgMeshDbl::setVerticesDbl(SgVertexArrayDbl* vertices)
{
    if(dbl_vertices_){
        dbl_vertices_->removeParent(this);
    }
    dbl_vertices_ = vertices;
    if(vertices){
        vertices->setAttribute(Geometry);
        vertices->addParent(this);
    }
    return vertices;
}
SgVertexArrayDbl* SgMeshDbl::getOrCreateVerticesDbl(int size)
{
    if(size < 0) {
        if(upConvert()) {
            return dbl_vertices_;
        } else {
            return nullptr;
        }
    }
    if(!dbl_vertices_){
        setVerticesDbl(new SgVertexArrayDbl(size));
    } else if(size >= 0){
        dbl_vertices_->resize(size);
    }
    return dbl_vertices_;
}
bool SgMeshDbl::upConvert() // float->double
{
    if(!vertices_) {
        return false;
    }
    if(!dbl_vertices_) {
        getOrCreateVerticesDbl(vertices_->size());
    } else {
        dbl_vertices_->resize(vertices_->size());
    }
    // Copy float->double
    for(int i = 0;  i < dbl_vertices_->size(); i++) {
        const Vector3f &vf = vertices_->at(i);
        Vector3 &vd = dbl_vertices_->at(i);
        const float *flt  = vf.data();
        double *dbl = vd.data();
        for(int j = 0; j < 3; j++) {
            dbl[j] = flt[j];
        }
    }
    return true;
}
bool SgMeshDbl::downConvert(bool resetOthers) // double->float
{
    if(!dbl_vertices_) {
        return false;
    }
    if(!vertices_) {
        getOrCreateVertices(dbl_vertices_->size());
    } else {
        vertices_->resize(dbl_vertices_->size());
    }
    // Copy double->float
    for(int i = 0;  i < dbl_vertices_->size(); i++) {
        Vector3f &vf = vertices_->at(i);
        const Vector3 &vd = dbl_vertices_->at(i);
        float *flt  = vf.data();
        const double *dbl = vd.data();
        for(int j = 0; j < 3; j++) {
            flt[j] = dbl[j];
        }
    }
    if (resetOthers) {
        MeshFilter mf;
        mf.setNormalOverwritingEnabled(true);
        mf.generateNormals(this, 0.75f);

        //_mesh.getOrCreateColors(0);
        setColors(nullptr);
        colorIndices_.resize(0);
        setTexCoords(nullptr);
        texCoordIndices_.resize(0);
    }
    updateBoundingBox();
    return true;
}
void SgMeshDbl::transform(const Isometry3& T)
{
    if (hasNormals()) {
        Matrix3 R = T.linear();
        auto& n = *normals();
        for(size_t i=0; i < n.size(); ++i) {
            n[i] = (R * n[i].cast<Isometry3::Scalar>()).cast<Vector3f::Scalar>();
        }
    }
    if (hasVerticesDbl()) {
        auto& v = *verticesDbl();
        for(size_t i=0; i < v.size(); ++i) {
            v[i] = T.linear() * v[i] + T.translation();
        }
        downConvert();
    } else if (hasVertices()) {
        auto& v = *vertices();
        for(size_t i=0; i < v.size(); ++i) {
            v[i] = (T * v[i].cast<Isometry3::Scalar>()).cast<Vector3f::Scalar>();
        }
        upConvert();
    }
}

//////
//
// CGALMesh
//
//////
CGALMesh::CGALMesh() : SgMeshDbl()
{
    DEBUG_STREAM("");
    object = new CGALObj();
}
CGALMesh::CGALMesh(const SgMesh &_sgmesh, CloneMap *cloneMap) : SgMeshDbl(_sgmesh, cloneMap)
{
    DEBUG_STREAM("");
    object = new CGALObj(*this);
}
CGALMesh::~CGALMesh()
{
    if (!!object) {
        delete object;
    }
}
CGALMeshPtr CGALMesh::copyFrom(const SgMesh &_sgmesh)
{
    CloneMap cm;
    CGALMeshPtr ptr(new CGALMesh(_sgmesh, &cm));
    return ptr;
}
CGALMeshPtr CGALMesh::copyFrom(const SgMesh &_sgmesh, const Isometry3 &T)
{
    CloneMap cm;
    CGALMeshPtr ptr(new CGALMesh(_sgmesh, &cm));
    ptr->transform(T);
    ptr->updateCGAL();//???
    return ptr;
}
bool CGALMesh::updateSelfByCGAL()
{
    if (!object) return false;
    DEBUG_STREAM("");
    if(object->isValid()) {
        object->writeToMesh(*this);
        downConvert(true);
        return true;
    }
    return false;
}
bool CGALMesh::updateCGAL()
{
    if (!!object) {
        delete object;
    }
    DEBUG_STREAM("");
    object = new CGALObj(*this);
    return true;
}
void CGALMesh::transform(const Isometry3& T)
{
    if(!!object && object->isValid()) {
        DEBUG_STREAM(" trans");
        object->transform(T);
        DEBUG_STREAM(" writeToMesh");
        object->writeToMesh(*this);
        downConvert(true);
    }
}
double CGALMesh::volume()
{
    if(!!object) {
        return object->volume();
    }
    return 0.0;
}
bool CGALMesh::booleanUnion(const CGALMeshPtr target)
{
    if (!target->object->cgal_obj || !object->cgal_obj) {
        return false;
    }
    Nef_polyhedron *tmp = new Nef_polyhedron(*(object->cgal_obj)  +  *(target->object->cgal_obj));
    delete object->cgal_obj;
    object->cgal_obj = tmp;
    return updateSelfByCGAL();
}
bool CGALMesh::booleanUnion(const CGALMeshPtr target, const Isometry3 &T)
{
    if (!target->object->cgal_obj || !object->cgal_obj) {
        return false;
    }
    Nef_polyhedron newTgt( *(target->object->cgal_obj) );
    transformNef(newTgt, T);
    Nef_polyhedron *tmp = new Nef_polyhedron(*(object->cgal_obj)  +  newTgt);
    delete object->cgal_obj;
    object->cgal_obj = tmp;
    return updateSelfByCGAL();
}
bool CGALMesh::booleanDifference(const CGALMeshPtr target)
{
    if (!target->object->cgal_obj || !object->cgal_obj) {
        return false;
    }
    Nef_polyhedron *tmp = new Nef_polyhedron(*(object->cgal_obj)  -  *(target->object->cgal_obj));
    delete object->cgal_obj;
    object->cgal_obj = tmp;
    return updateSelfByCGAL();
}
bool CGALMesh::booleanDifference(const CGALMeshPtr target, const Isometry3 &T)
{
    if (!target->object->cgal_obj || !object->cgal_obj) {
        return false;
    }
    Nef_polyhedron newTgt( *(target->object->cgal_obj) );
    transformNef(newTgt, T);
    Nef_polyhedron *tmp = new Nef_polyhedron(*(object->cgal_obj)  -  newTgt);
    delete object->cgal_obj;
    object->cgal_obj = tmp;
    return updateSelfByCGAL();
}
bool CGALMesh::booleanIntersection(const CGALMeshPtr target)
{
    if (!target->object->cgal_obj || !object->cgal_obj) {
        return false;
    }
    Nef_polyhedron *tmp = new Nef_polyhedron(*(object->cgal_obj)  *  *(target->object->cgal_obj));
    delete object->cgal_obj;
    object->cgal_obj = tmp;
    return updateSelfByCGAL();
}
bool CGALMesh::booleanIntersection(const CGALMeshPtr target, const Isometry3 &T)
{
    if (!target->object->cgal_obj || !object->cgal_obj) {
        return false;
    }
    Nef_polyhedron newTgt( *(target->object->cgal_obj) );
    transformNef(newTgt, T);
    Nef_polyhedron *tmp = new Nef_polyhedron(*(object->cgal_obj)  *  newTgt);
    delete object->cgal_obj;
    object->cgal_obj = tmp;
    return updateSelfByCGAL();
}
CGALMeshPtr CGALMesh::createByUnion(const CGALMeshPtr target) const
{
    if (!target->object->cgal_obj || !object->cgal_obj) {
        return nullptr;
    }
    Nef_polyhedron *tmp = new Nef_polyhedron(*(object->cgal_obj)  +  *(target->object->cgal_obj));
    CGALMeshPtr ret(new CGALMesh());
    ret->object->cgal_obj = tmp;
    bool res = ret->updateSelfByCGAL();
    return ret;
}
CGALMeshPtr CGALMesh::createByUnion(const CGALMeshPtr target, const Isometry3 &T) const
{
    if (!target->object->cgal_obj || !object->cgal_obj) {
        return nullptr;
    }
    Nef_polyhedron newTgt( *(target->object->cgal_obj) );
    transformNef(newTgt, T);
    Nef_polyhedron *tmp = new Nef_polyhedron(*(object->cgal_obj)  +  newTgt);
    CGALMeshPtr ret(new CGALMesh());
    ret->object->cgal_obj = tmp;
    bool res = ret->updateSelfByCGAL();
    return ret;
}
CGALMeshPtr CGALMesh::createByDifference(const CGALMeshPtr target) const
{
    if (!target->object->cgal_obj || !object->cgal_obj) {
        return nullptr;
    }
    Nef_polyhedron *tmp = new Nef_polyhedron(*(object->cgal_obj)  -  *(target->object->cgal_obj));
    CGALMeshPtr ret(new CGALMesh());
    ret->object->cgal_obj = tmp;
    bool res = ret->updateSelfByCGAL();
    return ret;
}
CGALMeshPtr CGALMesh::createByDifference(const CGALMeshPtr target, const Isometry3 &T) const
{
    if (!target->object->cgal_obj || !object->cgal_obj) {
        return nullptr;
    }
    Nef_polyhedron newTgt( *(target->object->cgal_obj) );
    transformNef(newTgt, T);
    Nef_polyhedron *tmp = new Nef_polyhedron(*(object->cgal_obj)  -  newTgt);
    CGALMeshPtr ret(new CGALMesh());
    ret->object->cgal_obj = tmp;
    bool res = ret->updateSelfByCGAL();
    return ret;
}
CGALMeshPtr CGALMesh::createByIntersection(const CGALMeshPtr target) const
{
    if (!target->object->cgal_obj || !object->cgal_obj) {
        return nullptr;
    }
    Nef_polyhedron *tmp = new Nef_polyhedron(*(object->cgal_obj)  *  *(target->object->cgal_obj));
    CGALMeshPtr ret(new CGALMesh());
    ret->object->cgal_obj = tmp;
    bool res = ret->updateSelfByCGAL();
    return ret;
}
CGALMeshPtr CGALMesh::createByIntersection(const CGALMeshPtr target, const Isometry3 &T) const
{
    if (!target->object->cgal_obj || !object->cgal_obj) {
        return nullptr;
    }
    Nef_polyhedron newTgt( *(target->object->cgal_obj) );
    transformNef(newTgt, T);
    Nef_polyhedron *tmp = new Nef_polyhedron(*(object->cgal_obj)  *  newTgt);
    CGALMeshPtr ret(new CGALMesh());
    ret->object->cgal_obj = tmp;
    bool res = ret->updateSelfByCGAL();
    return ret;
}
bool CGALMesh::checkInside(const Vector3 &p)
{
    if(!!object) {
        return object->checkInside(p);
    }
    return false;
}
bool CGALMesh::checkInside(const Vector3f &p)
{
    if(!!object) {
        return object->checkInside(p);
    }
    return false;
}
bool CGALMesh::checkInside(const SgPointSet &pt, std::vector<int> &_result)
{
    if(!!object) {
        return object->checkInside(pt, _result);
    }
    return false;
}
bool CGALMesh::generateInsidePoints(double resolution, const std::vector<int> &start_end_xyz,
                                    const Vector3 &offset, const Vector3 &scale, std::vector<Vector3> &result)
{
    if(!!object && start_end_xyz.size() > 5) {
        return object->generateInsidePoints(start_end_xyz[0], start_end_xyz[1], start_end_xyz[2], start_end_xyz[3],
                                            start_end_xyz[4], start_end_xyz[5], resolution, result,
                                            offset.x(), offset.y(), offset.z(), scale.x(), scale.y(), scale.z());
    }
    return false;
}
bool CGALMesh::generateInsidePointsIndices(size_t x_length, size_t y_length, size_t z_length,
                                           const Vector3 &size, const Vector3 &offset, std::vector<int> &indices)
{
    if(!!object) {
        return object->generateInsidePointsIndices(x_length, y_length, z_length, size.x(), size.y(), size.z(),
                                                   indices, offset.x(), offset.y(), offset.z());
    }
    return false;
}
