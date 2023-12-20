#include "CnoidOctomap.h"
#include <cnoid/MeshGenerator>

#include <iostream>

using namespace cnoid;

SgOctomap::SgOctomap(float resolution) : octomap::OcTree(resolution), offset_(Vector3::Zero()), scale_(1, 1, 1)
{
}

SgOctomap::~SgOctomap()
{
}

void SgOctomap::addPointSet(const SgPointSet &pt)
{
    octomap::Pointcloud octo_pc;
    const SgVertexArray *ary = pt.vertices();

    for(int i = 0; i < ary->size(); i++) {
        const Vector3f &p = ary->at(i);
        octo_pc.push_back(p.x(), p.y(),  p.z());
    }
    octomap::point3d origin(0., 0., 0.);
    this->insertPointCloud (octo_pc, origin);
}

void SgOctomap::addPoints(const std::vector<Vector3> &pt)
{
    octomap::Pointcloud octo_pc;
    for(int i = 0; i < pt.size(); i++) {
        const Vector3 &p = pt[i];
        octo_pc.push_back(p.x(), p.y(),  p.z());
    }
    octomap::point3d origin(0., 0., 0.);
    this->insertPointCloud (octo_pc, origin);
}

void SgOctomap::addBoxPrimitives(SgGroupPtr sgg, SgMaterial *mat)
{
    double scl_x = this->scale_.x();
    double scl_y = this->scale_.y();
    double scl_z = this->scale_.z();
    Vector3 _offset(this->offset_.x(), this->offset_.y(), this->offset_.z());

    for(auto it = this->begin_leafs(); it != this->end_leafs(); it++) {
        double sz = it.getSize();
        {
            OcTree::NodeType &nd = *it;
            if ( nd.getLogOdds() < 0.0) {
                continue;
            }
#if 0
            double n0 = nd.getLogOdds(); //
            double n1 = nd.getMaxChildLogOdds();
            double n2 = nd.getMeanChildLogOdds();
            double n3 = nd.getOccupancy ();
            double n4 = nd.getValue (); //

            double xx = it.getX();
            double yy = it.getY();
            double zz = it.getZ();
            std::cout << sz << " : " << xx << ", " << yy << ", " << zz  << std::endl;
            std::cout << "     param: " << n0 << ", " << n1 << ", " << n2 << ", " << n3 << ", " << n4 << std::endl;

            //param: -0.405465, -3.40282e+38, -inf, 0.4, -0.405465
            //param: 0.847298, -3.40282e+38, -inf, 0.7, 0.847298
            if (n4 < 0.0) {
                continue;
            }
#endif
        }
        MeshGenerator mg;
        Vector3 size(sz * scl_x, sz * scl_y, sz * scl_z);
        Vector3 pos(it.getX(), it.getY(), it.getZ());
        pos += _offset;

        SgMesh *mesh = mg.generateBox(size);
        SgPosTransform *trs = new SgPosTransform();

        SgShape *sp = new SgShape();
        sp->setMesh(mesh);
        sp->setMaterial(mat);

        trs->setTranslation(pos);
        trs->addChild(sp);

        sgg->addChild(trs);
    }
}
