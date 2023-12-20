/**
   @author YoheiKakiuchi
*/

#include <cnoid/PyUtil>
#include <cnoid/PyEigenTypes>
#include "../src/CnoidCGAL.h"
#include "../src/MergeBoxes.h"

using Matrix4RM = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;

typedef Eigen::Matrix<double, -1, -1, Eigen::RowMajor> MatrixRM;

using namespace cnoid;
namespace py = pybind11;

PYBIND11_MODULE(CGALMesh, m)
{
    m.doc() = "python-binding for using CGAL";

    py::module::import("cnoid.Util");

    py::class_< SgMeshDbl, SgMeshDblPtr, SgMesh > dbl_cls(m, "SgMeshDbl");
    dbl_cls.def(py::init<>())
    .def(py::init<const SgMesh &>())
    //
    .def("hasVerticesDbl", &SgMeshDbl::hasVerticesDbl)
    .def("setVerticesDbl", [](SgMeshDbl &self, Eigen::Ref<const MatrixRM> mat) {
        // mat.cols() == 3
        int size = mat.rows();
        SgVertexArrayDbl *va_ = self.getOrCreateVerticesDbl(size);
        for(int i = 0; i < size; i++) va_->at(i) = mat.row(i);  })
    .def_property_readonly("verticesDbl", [](SgMeshDbl &self) {
        SgVertexArrayDbl *va_ = self.verticesDbl();
        if (!va_) { MatrixRM mat(0, 3); return mat; }
        MatrixRM mat(va_->size(), 3);
        for(int i = 0; i < va_->size(); i ++) mat.row(i) = va_->at(i);
        return mat; })
    .def_property_readonly("sizeOfVerticesDbl", [](SgMeshDbl &self) { if (!self.verticesDbl()) return (std::size_t)0; return self.verticesDbl()->size(); })
    .def("appendVertex", [](SgMeshDbl &self, const Vector3 &v) { self.verticesDbl()->push_back(v); })
    .def("vertex", [](SgMeshDbl &self, int idx) { return self.verticesDbl()->at(idx); })
    .def("setVertex", [](SgMeshDbl &self, int idx, const Vector3 &v) { self.verticesDbl()->at(idx) = v; })
    //
    .def("upConvert", &SgMeshDbl::upConvert)
    .def("downConvert", &SgMeshDbl::downConvert)
    .def("transform", [](SgMeshDbl& self, Eigen::Ref<const Matrix4RM> T){ const Isometry3 tt(T); self.transform(tt); })
    ;

    py::class_< CGALMesh, CGALMeshPtr, SgMeshDbl > cgal_cls(m, "CGALMesh");
    cgal_cls.def(py::init<>())
    .def(py::init<const SgMesh &>())
    .def(py::init<const SgMeshDbl &>())
    .def_static("copyFrom", (CGALMeshPtr(*)(const SgMesh &))&CGALMesh::copyFrom)
    .def_static("copyFromWithTransform", (CGALMeshPtr(*)(const SgMesh &, const Isometry3 &))&CGALMesh::copyFrom)
    .def("transform", [](CGALMesh& self, Eigen::Ref<const Matrix4RM> T){ const Isometry3 tt(T); self.transform(tt); })
    .def("volume", &CGALMesh::volume)
    .def("booleanUnion", (bool(CGALMesh::*)(const CGALMeshPtr))&CGALMesh::booleanUnion)
    .def("booleanUnionWithTransform", [] (CGALMesh &self, CGALMeshPtr target, Eigen::Ref<const Matrix4RM> T) {
        const Isometry3 tt(T); return self.booleanUnion(target, tt); })
    .def("booleanDifference", (bool(CGALMesh::*)(const CGALMeshPtr))&CGALMesh::booleanDifference)
    .def("booleanDifferenceWithTransform", [] (CGALMesh &self, CGALMeshPtr target, Eigen::Ref<const Matrix4RM> T) {
        const Isometry3 tt(T); return self.booleanDifference(target, tt); })
    .def("booleanIntersection", (bool(CGALMesh::*)(const CGALMeshPtr))&CGALMesh::booleanIntersection)
    .def("booleanIntersectionWithTransform", [] (CGALMesh &self, CGALMeshPtr target, Eigen::Ref<const Matrix4RM> T) {
        const Isometry3 tt(T); return self.booleanIntersection(target, tt); })
    .def("createByUnion", (CGALMeshPtr(CGALMesh::*)(const CGALMeshPtr) const)&CGALMesh::createByUnion)
    .def("createByUnionWithTransform", [] (CGALMesh &self, CGALMeshPtr target, Eigen::Ref<const Matrix4RM> T) {
        const Isometry3 tt(T); return self.createByUnion(target, tt); })
    .def("createByDifference", (CGALMeshPtr(CGALMesh::*)(const CGALMeshPtr) const)&CGALMesh::createByDifference)
    .def("createByDifferenceWithTransform", [] (CGALMesh &self, CGALMeshPtr target, Eigen::Ref<const Matrix4RM> T) {
        const Isometry3 tt(T); return self.createByDifference(target, tt); })
    .def("createByIntersection", (CGALMeshPtr(CGALMesh::*)(const CGALMeshPtr) const)&CGALMesh::createByIntersection)
    .def("createByIntersectionWithTransform", [] (CGALMesh &self, CGALMeshPtr target, Eigen::Ref<const Matrix4RM> T) {
        const Isometry3 tt(T); return self.createByIntersection(target, tt); })
    .def("checkInside", [](CGALMesh &self, const Vector3 &v) { return self.checkInside(v); })
    .def("checkInside", [](CGALMesh &self, const Vector3f &v) { return self.checkInside(v); })
    .def("checkInside", [](CGALMesh &self, const SgPointSetPtr &pt) {
        std::vector<int> res; self.checkInside(*pt, res); return res;})
    .def("addPointsMergeBoxes", [] (CGALMesh &self, MergeBoxes &mboxes) {
                                    // set offset and boxSize to mboxes before calling this method
                                    std::vector<int> res; int _x, _y, _z; bool ret;
                                    mboxes.getSize(_x, _y, _z);
                                    ret = self.generateInsidePointsIndices(_x, _y, _z, mboxes.boxSize(), mboxes.offset(), res);
                                    if (!ret) { return 1; }
                                    ret = mboxes.setPoints(res);
                                    if (!ret) { return 2; }
                                    return 0; })
    ;

    py::class_< MergeBoxes > (m, "MergeBoxes")
    .def(py::init<size_t, size_t, size_t>())
    .def("setPoints", &MergeBoxes::setPoints)
    .def("mergePoints", &MergeBoxes::mergePoints)
    .def("addBoxPrimitives", &MergeBoxes::addBoxPrimitives)
    .def("resetBoxes", &MergeBoxes::resetBoxes)
    .def_property_readonly("sizeOfBoxes", &MergeBoxes::sizeOfBoxes)
    .def("getSize", [](MergeBoxes &self) { int _x, _y, _z;
            self.getSize(_x, _y, _z); std::vector<int> res;
            res.push_back(_x); res.push_back(_y); res.push_back(_z);
            return res; })
    .def_property("offset", [] (MergeBoxes &self) { return self.offset(); },
                  [] (MergeBoxes &self, const Vector3 &v) { self.offset() = v; })
    .def_property("boxSize", [] (MergeBoxes &self) { return self.boxSize(); },
                  [] (MergeBoxes &self, const Vector3 &v) { self.boxSize() = v; })
    ;
}
