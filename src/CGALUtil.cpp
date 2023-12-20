#include "CGALUtil.h"

// generating mesh
#include <CGAL/OFF_to_nef_3.h>
// writing mesh
#include <CGAL/boost/graph/convert_nef_polyhedron_to_polygon_mesh.h>
// Triangulation
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/mark_domain_in_triangulation.h> // not using ??
#include <CGAL/Polygon_2.h> // not using ??
// volume
#include <CGAL/Polygon_mesh_processing/measure.h>
//
#include <CGAL/Side_of_triangle_mesh.h>

//#include <CGAL/Polygon_mesh_processing/refine.h>
//#include <CGAL/Polygon_mesh_processing/fair.h>
//// triangulation
typedef CGAL::Triangulation_vertex_base_2<CGAL_Kernel> VertexBase;
typedef CGAL::Constrained_triangulation_face_base_2<CGAL_Kernel> FaceBase;
typedef CGAL::Triangulation_data_structure_2<VertexBase, FaceBase> TDS;
typedef CGAL::Exact_predicates_tag  Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<CGAL_Kernel, TDS, Itag> CDT;
typedef CDT::Point Point2;
typedef CGAL::Polygon_2<CGAL_Kernel> Polygon_2_;

#ifdef PCLSEARCH
#include <pcl/kdtree/kdtree_flann.h>
typedef pcl::PointXY PCL2D;
#endif

using namespace cnoid;
using namespace CGAL;

#ifdef DEBUG_MSG
#include <iostream>
#define DEBUG_STREAM(args) std::cerr << __PRETTY_FUNCTION__ << args << std::endl;
#define DEBUG_PRINT(args) std::cerr << args << std::endl;
#else
#define DEBUG_STREAM(args)
#define DEBUG_PRINT(args)
#endif

template <class Nef_3>
std::size_t
buildNefPolyhedron (const SgMeshDbl &mesh, Nef_3 &nef_union, bool verbose = true)
{
    // Nef
    typedef typename Nef_3::Kernel         Kernel;
    typedef typename Nef_3::Point_3        Point_3;
    typedef typename std::vector<Point_3>  Point_set;
    CGAL::Nef_nary_union_3<Nef_3>          nary_union;
    // input data structure
    typedef double                         Input_NT;
    typedef CGAL::Cartesian<Input_NT>      Input_kernel;
    typedef Input_kernel::Point_3          Input_point;
    typedef std::vector<Input_point>       Input_point_set;
    typedef Input_kernel::Vector_3         Input_vector;
    typedef typename Kernel::Kernel_tag Kernel_tag;
    typedef typename CGAL::number_type_converter_nef_3<Kernel_tag, Kernel> ntc;

    int discarded_facets = 0;
    std::size_t numVertices = mesh.verticesDbl()->size(); //
    Input_point_set V_inp;
    V_inp.reserve (numVertices);
    Point_set V;
    V.reserve (numVertices);
    // store vertices
    const SgVertexArrayDbl *vts = mesh.verticesDbl();
    for (std::size_t idx=0; idx < numVertices; idx++) {
        Input_point sp(vts->at(idx).x(), vts->at(idx).y(), vts->at(idx).z());
        V_inp.push_back (sp);
        V.push_back (ntc::convert(sp));
    }
    // for each facet
    int numTri = mesh.numTriangles();
    const SgIndexArray &idx_ary = mesh.triangleVertices();
    for (std::size_t idx = 0; idx < numTri; idx++) {
#define TRIANGLE_SIZE 3
        Input_point_set V_f_inp;
        V_f_inp.reserve(TRIANGLE_SIZE);
        Point_set V_f;
        V_f.reserve(TRIANGLE_SIZE);

        std::size_t v_idx = TRIANGLE_SIZE * idx;
        for (std::size_t jdx = 0; jdx < TRIANGLE_SIZE; jdx++) {
            int tmp_idx = idx_ary[v_idx + jdx];
            // TODO : check (tmp_idx < numVertices );
            V_f_inp.push_back (V_inp[tmp_idx]);
            V_f.push_back (V[tmp_idx]);
        }

        bool is_nef = false;
        CGAL_assertion_msg( V_f.size() >= 1 || !verbose, "empty vertex cycle");
        if ( V_f.size() >= 1 ) {
            Input_vector normal;
            normal_vector_newell_3(V_f_inp.begin(), V_f_inp.end(), normal);
            Nef_3 nef (V_f.begin(), V_f.end(), normal, verbose);
            if ( !nef.is_empty() ) {
                nary_union.add_polyhedron(nef);
                is_nef = true;
            }
        }
        if ( !is_nef ) {
            ++discarded_facets;
            if (verbose) {
                // message
            }
        }
    }

    nef_union = nary_union.get_union();
    CGAL::Mark_bounded_volumes<Nef_3> mbv (true);
    nef_union.delegate (mbv);

    return discarded_facets;
}

struct Plane
{
    Vector3 origin;
    Vector3 normal;
    Vector3 u_axis;
    Vector3 v_axis;
#ifdef PCLSEARCH
    bool onPlane(const Vector3 &pt, PCL2D &uv, double eps=1e-10)
    {
        bool res = true;
        double dist = std::fabs((pt - origin).dot(normal));
        if (dist > eps) {
            res = false;
        }
        double uu = (pt - origin).dot(u_axis);
        double vv = (pt - origin).dot(v_axis);
        uv.x = uu;
        uv.y = vv;
        return res;
    }
#endif
    bool onPlane(const Vector3 &pt, Vector2 &uv, double eps=1e-10)
    {
        bool res = true;
        double dist = std::fabs((pt - origin).dot(normal));
        if (dist > eps) {
            res = false;
        }
        double uu = (pt - origin).dot(u_axis);
        double vv = (pt - origin).dot(v_axis);
        uv.x() = uu;
        uv.y() = vv;
        return res;
    }
    double setPlane(Vector3 &v0, Vector3 &v1, Vector3 &v2, double eps=1e-10)
    {
        double res;
        Vector3 uu = (v1 - v0);
        Vector3 vv = (v2 - v0);
        uu.normalize();
        vv.normalize();
        Vector3 nn = uu.cross(vv);
        double norm = nn.norm();
        DEBUG_STREAM(" uu: " << uu.x() << " " << uu.y() << " " << uu.z());
        DEBUG_STREAM(" vv: " << vv.x() << " " << vv.y() << " " << vv.z());
        DEBUG_STREAM(" nn(" << norm << ") : " << nn.x() << " " << nn.y());
#if 0
        if (norm < eps) {
            Vector3 uu2 = (v0 - v1);
            Vector3 vv2 = (v2 - v1);
            uu2.normalize();
            vv2.normalize();
            Vector3 nn2 = uu2.cross(vv2);
            double norm2 = nn2.norm();
            if (norm2 > norm) {
                uu = uu2;
                //vv = vv2;
                if (nn.dot(nn2) < 0) {
                    nn = nn2;
                } else {
                    nn = -nn2;
                }
                origin = v1;
                res = norm2;
            } else {
                origin = v0;
                res = norm;
            }
        }
#endif
        origin = v0;
        res = norm;
        ////
        nn.normalize();
        normal = nn;
        u_axis = uu;
        v_axis = nn.cross(uu);
        return res;
    }
    void print()
    {
        DEBUG_STREAM(" origin: " << origin.x() << " " << origin.y() << " " << origin.z());
        DEBUG_STREAM(" normal: " << normal.x() << " " << normal.y() << " " << normal.z());
        DEBUG_STREAM(" u_axis: " << u_axis.x() << " " << u_axis.y() << " " << u_axis.z());
        DEBUG_STREAM(" v_axis: " << v_axis.x() << " " << v_axis.y() << " " << v_axis.z());
    }
};
#ifdef PCLSEARCH
void generateOnPlanePoints(const SgVertexArrayDblPtr varray, const std::vector<int> indices,
                           pcl::PointCloud<PCL2D>::Ptr ptlst)
{
#if 0
    Vector3 center(0, 0, 0);
    for(int i = 0; i < indices.size(); i++) {
        const Vector3 &v = SgVertexArrayDbl->at(indices[i]);
        center += v;
    }
    center *= (1.0/indices.size());
#endif
    Plane pl;
    // TODO : making a plane with larger area
    //double plres = pl.setPlane(varray->at(indices[0]), varray->at(indices[1]), varray->at(indices.back()));
    double plres = pl.setPlane(varray->at(indices[0]), varray->at(indices[1]), varray->at(indices[2]));
    ////
    //ptlst2.reserve(indices.size());
    ptlst->width = indices.size();
    ptlst->height = 1;
    ptlst->is_dense = false;
    ptlst->points.resize(indices.size());
    for(int i = 0; i < indices.size(); i++) {
        const Vector3 &v = varray->at(indices[i]);
        bool res = pl.onPlane(v, ptlst->points[i]);
        if (!res) {
            ////
        }
    }
}
void TriangulationPointXY(const pcl::PointCloud<PCL2D>::Ptr ptlst, std::vector<int> &result)
{
    // kdtree<points>
    // N point -> org_points(pcl)
    // N point -> Point2
    // vertex -> int
    pcl::KdTreeFLANN < PCL2D > kdtree;
    kdtree.setInputCloud(ptlst);

    Polygon_2_ polygon;
    for(int i = 0; i < ptlst->points.size(); i++) {
        Point2 pp(ptlst->points[i].x, ptlst->points[i].y);
        polygon.push_back(pp);
    }

    CDT cdt;
    cdt.insert_constraint(polygon.vertices_begin(), polygon.vertices_end(), true);

    std::unordered_map<CDT::Face_handle, bool> in_domain_map;
    boost::associative_property_map< std::unordered_map<CDT::Face_handle, bool> >
    in_domain(in_domain_map);

    CGAL::mark_domain_in_triangulation(cdt, in_domain);

    // vertex ->
    std::map<CDT::Vertex_handle, int> vtxmap;
    int vcntr = 0;
    for(CDT::Vertex_handle vv : cdt.finite_vertex_handles())
    {
        auto xx = (*vv);
        const CDT::Point &p = cdt.point(vv);
        //// serach_at_kdtree ( point -> index )
        PCL2D pt;
#ifdef USE_GMPQ
        double xx_ = p.x().to_double();
        double yy_ = p.y().to_double();
#else
        double xx_ = p.x();
        double yy_ = p.y();
#endif
        pt.x = xx_;
        pt.y = yy_;
        int k = 1;
        std::vector<int> indices;
        std::vector<float> distance;
        indices.resize(k);
        distance.resize(k);
        kdtree.nearestKSearch (pt, k, indices, distance);
        //
        vtxmap.insert(std::make_pair(vv, indices[0]));
        vcntr++;
    }

    int count=0;
    for (CDT::Face_handle f : cdt.finite_face_handles())
    {
        if ( get(in_domain, f) ) {
            //// using face
            auto v0 = (*f).vertex(0);
            auto v1 = (*f).vertex(1);
            auto v2 = (*f).vertex(2);
            result.push_back(vtxmap[v0]);
            result.push_back(vtxmap[v1]);
            result.push_back(vtxmap[v2]);
            ++count;
        }
    }
}
#endif
void generateOnPlanePoints(const SgVertexArrayDblPtr varray, const std::vector<int> indices,
                           std::vector<Vector2> &uvlst)
{
    Plane pl;
    //double plres = pl.setPlane(varray->at(indices[0]), varray->at(indices[1]), varray->at(indices.back()));
    DEBUG_STREAM("in(size) : " << indices.size());
#ifdef DEBUG_MSG
    //// check for vertex
    {
        std::size_t num = indices.size();
        for(std::size_t i = 0; i < num -1; i++) {
            const Vector3 &vi = varray->at(indices[i]);
            for(std::size_t j = i+1; j < num; j++) {
                const Vector3 &vj = varray->at(indices[j]);
                if (vi == vj) {
                    DEBUG_STREAM(" " << i << " == " << j);
                }
            }
        }
    }
#endif
    double plres = pl.setPlane(varray->at(indices[0]), varray->at(indices[1]), varray->at(indices[2]));
    pl.print();
    if (plres < 1e-10) {
        DEBUG_STREAM(" [WARNING] is valid plane? | " << plres);
    }
    uvlst.resize(indices.size());
    for(int i = 0; i < indices.size(); i++) {
        const Vector3 &v = varray->at(indices[i]);
        DEBUG_STREAM( i << " v (" << indices[i] << ") : " << v.x() << " " << v.y() << " " << v.z());
        bool res = pl.onPlane(v, uvlst[i]);
        DEBUG_STREAM(" res : " << res << " : " << uvlst[i].x() << " " << uvlst[i].y());
        if (!res) {
            DEBUG_STREAM(" [WARNING] point is not on plane?");
        }
    }
}
int nearestUV(const std::vector<Vector2> &uvlst, double u, double v, double &dist)
{
    double min_dist = 1e15;
    int min_index = -1;
    for(int i = 0; i < uvlst.size(); i++) {
        double _u = u - uvlst[i].x();
        double _v = v - uvlst[i].y();
        double _dist = _u * _u + _v * _v;
        if (_dist < min_dist) {
            min_index = i;
            min_dist = _dist;
        }
    }
    dist = min_dist;
    return min_index;
}
void TriangulationPointXY(const std::vector<Vector2> &uvlst, std::vector<int> &result)
{
    Polygon_2_ polygon;
    for(int i = 0; i < uvlst.size(); i++) {
        Point2 pp(uvlst[i].x(), uvlst[i].y());
        polygon.push_back(pp);
    }
    CDT cdt;
    cdt.insert_constraint(polygon.vertices_begin(), polygon.vertices_end(), true);
    std::unordered_map<CDT::Face_handle, bool> in_domain_map;
    boost::associative_property_map< std::unordered_map<CDT::Face_handle, bool> >
    in_domain(in_domain_map);
    CGAL::mark_domain_in_triangulation(cdt, in_domain);

    std::map<CDT::Vertex_handle, int> vtxmap;
    int vcntr = 0;
    for(CDT::Vertex_handle vv : cdt.finite_vertex_handles())
    {
        auto xx = (*vv);
        const CDT::Point &p = cdt.point(vv);
        double min_dist;
#ifdef USE_GMPQ
        int min_idx = nearestUV(uvlst, p.x().to_double(), p.y().to_double(), min_dist);
#else
        double xx_ = CGAL::to_double(p.x());
        double yy_ = CGAL::to_double(p.y());
        int min_idx = nearestUV(uvlst, xx_, yy_, min_dist);
#endif
        ////
        vtxmap.insert(std::make_pair(vv, min_idx));
        vcntr++;
    }
    int count=0;
    for (CDT::Face_handle f : cdt.finite_face_handles())
    {
        if ( get(in_domain, f) ) {
            //// using face
            auto v0 = (*f).vertex(0);
            auto v1 = (*f).vertex(1);
            auto v2 = (*f).vertex(2);
            result.push_back(vtxmap[v0]);
            result.push_back(vtxmap[v1]);
            result.push_back(vtxmap[v2]);
            ++count;
        }
    }
}
//void SurfaceMeshToMeshDbl(Surface_mesh_ &mesh_out, SgMeshDbl &_mesh, bool verbose = false)
inline void SurfaceMeshToMeshDbl(Surface_mesh_ &mesh_out, SgMeshDbl &_mesh, bool verbose = false)
{
    //// post_processing surface_mesh
    DEBUG_STREAM(" ver: " << mesh_out.number_of_vertices());
    DEBUG_STREAM(" fcs: " << mesh_out.number_of_faces());
    SgVertexArrayDblPtr varray(new SgVertexArrayDbl());
    varray->resize(mesh_out.number_of_vertices());
    {
        long cntr = 0;
        for(auto it = mesh_out.vertices_begin(); it != mesh_out.vertices_end(); it++, cntr++) {
            std::size_t ii = (std::size_t)(*it);
            if (ii != cntr) {
                DEBUG_STREAM(" [WARNING] vertices index is not equal to order? " << ii << " != " << cntr);
            }
            Surface_mesh_::Point &pt = mesh_out.point(*it);
            DEBUG_STREAM(cntr << " " << (*it) << " " << pt.x() << " " << pt.y() << " " << pt.z());
            Vector3 &vv = varray->at(cntr);
#ifdef USE_GMPQ
            vv.x() = pt.x().to_double();
            vv.y() = pt.y().to_double();
            vv.z() = pt.z().to_double();
#else
            vv.x() = pt.x();
            vv.y() = pt.y();
            vv.z() = pt.z();
#endif
        }
    }
    std::vector<int> indices;
    indices.reserve(mesh_out.number_of_faces() * 3);
    {
        for(auto it = mesh_out.faces_begin(); it != mesh_out.faces_end(); it++) {
            std::vector<int> tmp_idx;
            auto st = mesh_out.halfedge(*it);
            auto nx = mesh_out.next(st);
            tmp_idx.push_back((int)mesh_out.target(st));
            while(nx != st) {
                tmp_idx.push_back((int)mesh_out.target(nx));
                nx = mesh_out.next(nx);
            }
            if (tmp_idx.size() > 3) {
                DEBUG_STREAM(" more than 3 vertices");
                std::vector<int> local_indices;
#ifdef PCLSEARCH
                pcl::PointCloud<PCL2D>::Ptr points2d(new pcl::PointCloud<PCL2D>());
                generateOnPlanePoints(varray, tmp_idx, points2d);
                TriangulationPointXY(points2d, local_indices);
#else
                std::vector<Vector2> uvlst;
                generateOnPlanePoints(varray, tmp_idx, uvlst);
                TriangulationPointXY(uvlst, local_indices);
#endif
                for(std::size_t i = 0; i < local_indices.size(); i++) {
                    DEBUG_STREAM(" local idx" << i << " : " << local_indices[i] << " ==> " << tmp_idx[local_indices[i]]);
                    indices.push_back( tmp_idx[local_indices[i]] );
                }
            } else if (tmp_idx.size() < 3) {
                DEBUG_STREAM(" [WARNING] invalid mesh size : " << tmp_idx.size());
            } else {
                indices.push_back(tmp_idx[0]);
                indices.push_back(tmp_idx[1]);
                indices.push_back(tmp_idx[2]);
            }
        }
    }
    //// Write down to mesh-object
    _mesh.setVerticesDbl(varray);
    _mesh.faceVertexIndices() = indices;
}
void writeNefToMesh(Nef_polyhedron &nef, SgMeshDbl &_mesh, bool verbose = false)
{
    Surface_mesh_ mesh_out;
    CGAL::convert_nef_polyhedron_to_polygon_mesh(nef, mesh_out);
    CGAL::Polygon_mesh_processing::triangulate_faces(mesh_out);

    SurfaceMeshToMeshDbl(mesh_out, _mesh, verbose);
}
#if 0
//// TODO: mesh processing
typedef Polyhedron::Vertex_handle Vertex_handle;
void writeNefToMesh(Nef_polyhedron &nef, SgMeshDbl &_mesh, bool verbose = false)
{
    Polyhedron poly;
    nef.convert_to_Polyhedron(poly);
    std::vector<Polyhedron::Facet_handle>  new_facets;
    std::vector<Vertex_handle> new_vertices;
    CGAL::Polygon_mesh_processing::refine(poly, faces(poly),
                                          std::back_inserter(new_facets),
                                          std::back_inserter(new_vertices),
                                          CGAL::parameters::density_control_factor(2.));
    CGAL::Polygon_mesh_processing::triangulate_faces(poly);

    Nef_polyhedron new_nef(poly);
    Surface_mesh_ mesh_out;
    CGAL::convert_nef_polyhedron_to_polygon_mesh(new_nef, mesh_out);

    SurfaceMeshToMeshDbl(mesh_out, _mesh, verbose);
}
#endif
////
//
//
//
////
CGALMesh::CGALObj::CGALObj(const SgMeshDbl &_mesh)
{
    cgal_obj = nullptr;
    readFromMesh(_mesh, false);
}
int CGALMesh::CGALObj::readFromMesh(const SgMeshDbl &_mesh, bool verbose)
{
    Nef_polyhedron *nef = new Nef_polyhedron();
    error_count = buildNefPolyhedron(_mesh, *nef, verbose);
    if (!!cgal_obj) {
        delete cgal_obj;
    }
    cgal_obj = nef;
    return error_count;
}
void CGALMesh::CGALObj::writeToMesh(SgMeshDbl &_mesh, bool verbose)
{
    if (!!cgal_obj) {
        writeNefToMesh(*cgal_obj, _mesh, verbose);
    }
}
void CGALMesh::CGALObj::transform(const Isometry3 &T)
{
    if(!!cgal_obj) {
        transformNef(*cgal_obj, T);
    }
}
double CGALMesh::CGALObj::volume()
{
    if(!!cgal_obj) {
#ifdef USE_GMPQ
        CGAL::Surface_mesh<CGAL::Cartesian<double>::Point_3> mesh_out;
        // CGAL::Surface_mesh<CGAL::Exact_predicates_exact_constructions_kernel::Point_3> mesh_out;
        CGAL::convert_nef_polyhedron_to_polygon_mesh(*cgal_obj, mesh_out);
        return CGAL::to_double(CGAL::Polygon_mesh_processing::volume(mesh_out));
#else
        Polyhedron plh;
        cgal_obj->convert_to_polyhedron(plh);
        auto ret = CGAL::Polygon_mesh_processing::volume(plh);
        return CGAL::to_double(ret);
#endif
    }
    return 0.0;
}
bool CGALMesh::CGALObj::checkInside(const Vector3 &pin)
{
    if(!!cgal_obj) {
       Polyhedron plh;
       cgal_obj->convert_to_polyhedron(plh);
       CGAL::Side_of_triangle_mesh<Polyhedron, CGAL_Kernel> inside(plh);
       //CGAL::Side_of_triangle_mesh<Nef_polyhedron, CGAL_Kernel> inside(*cgal_obj);
       Polyhedron::Point_3 p(pin.x(), pin.y(), pin.z());
       CGAL::Bounded_side res = inside(p);
        if (res == CGAL::ON_BOUNDED_SIDE) {
            return true;
        } else if (res == CGAL::ON_BOUNDARY) {
            return true;
        }
        return false;
    }
    return false;
}
bool CGALMesh::CGALObj::checkInside(const Vector3f &p)
{
    Vector3 pd(p.x(), p.y(), p.z());
    return checkInside(pd);
}
bool CGALMesh::CGALObj::checkInside(const SgPointSet &pt, std::vector<int> &_result)
{
    if(!!cgal_obj) {
        _result.resize(0);
        Polyhedron plh;
        cgal_obj->convert_to_polyhedron(plh);
        CGAL::Side_of_triangle_mesh<Polyhedron, CGAL_Kernel> inside(plh);
        //CGAL::Side_of_triangle_mesh<Nef_polyhedron, CGAL_Kernel> inside(*cgal_obj);
        const SgVertexArray *ary = pt.vertices();
        for(int i = 0; i < ary->size(); i++) {
            const Vector3f &pin = ary->at(i);
            Polyhedron::Point_3 p(pin.x(), pin.y(), pin.z());
            CGAL::Bounded_side res = inside(p);
            if (res == CGAL::ON_BOUNDED_SIDE) {
                _result.push_back(1);
            } else if (res == CGAL::ON_BOUNDARY) {
                _result.push_back(2);
            } else {
                _result.push_back(0);
            }
        }
        return true;
    }
    return false;
}
bool CGALMesh::CGALObj::generateInsidePoints(int st_x, int ed_x, int st_y, int ed_y, int st_z, int ed_z, double resolution,
                                             std::vector<Vector3> &result, double off_x, double off_y, double off_z, double scl_x, double scl_y, double scl_z)
{
    if(!!cgal_obj) {
        result.resize(0);
        Polyhedron plh;
        cgal_obj->convert_to_polyhedron(plh);
        CGAL::Side_of_triangle_mesh<Polyhedron, CGAL_Kernel> inside(plh);
        double resolution_2 = resolution/2;
        for(int xx = st_x; xx <= ed_x; xx++) {
            double org_x = ( resolution_2 + resolution*xx );
            double map_ptx = org_x + off_x;
            double obj_ptx = scl_x * org_x + off_x;
            for(int yy = st_y; yy <= ed_y; yy++) {
                double org_y = ( resolution_2 + resolution*yy );
                double map_pty = org_y + off_y;
                double obj_pty = scl_y * org_y + off_y;
                for(int zz = st_z; zz <= ed_z; zz++) {
                    double org_z = ( resolution_2 + resolution*zz );
                    double map_ptz = org_z + off_z;
                    double obj_ptz = scl_z * org_z + off_z;
                    Polyhedron::Point_3 p(obj_ptx, obj_pty, obj_ptz);
                    CGAL::Bounded_side res = inside(p);
                    if (res == CGAL::ON_BOUNDED_SIDE) {
                        Vector3 v(map_ptx, map_pty, map_ptz);
                        result.push_back(v);
                    } else if (res == CGAL::ON_BOUNDARY) {
                        Vector3 v(map_ptx, map_pty, map_ptz);
                        result.push_back(v);
                    } else {
                        // _result.push_back(0);
                    }
                }
            }
        }
        return true;
    }
    return false;
}
bool CGALMesh::CGALObj::generateInsidePointsIndices(size_t _x_len, size_t _y_len, size_t _z_len,
                                                    double size_x, double size_y, double size_z,
                                                    std::vector<int> &indices, double off_x, double off_y, double off_z)
{
    if(!!cgal_obj) {
        indices.resize(0);
        Polyhedron plh;
        cgal_obj->convert_to_polyhedron(plh);
        CGAL::Side_of_triangle_mesh<Polyhedron, CGAL_Kernel> inside(plh);
        int counter = 0;
        for(size_t zz = 0; zz < _z_len; zz++) {
            double obj_ptz = size_z * zz + off_z;
            for(size_t yy = 0; yy < _y_len; yy++) {
                double obj_pty = size_y * yy + off_y;
                for(size_t xx = 0; xx < _x_len; xx++) {
                    double obj_ptx = size_x * xx + off_x;
                    Polyhedron::Point_3 p(obj_ptx, obj_pty, obj_ptz);
                    CGAL::Bounded_side res = inside(p);
                    if (res == CGAL::ON_BOUNDED_SIDE) {
                        indices.push_back(counter);
                        counter++;
                    } else if (res == CGAL::ON_BOUNDARY) {
                        indices.push_back(counter);
                        counter++;
                    } else {
                        indices.push_back(-1);
                    }
                }
            }
        }
        return true;
    }
    return false;
}
