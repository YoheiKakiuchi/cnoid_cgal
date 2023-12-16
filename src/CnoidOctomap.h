#ifndef __CHOREONOID_OCTOMAP_LIB_H__
#define __CHOREONOID_OCTOMAP_LIB_H__

#include <cnoid/SceneGraph>
#include <cnoid/SceneDrawables>

#include <octomap/octomap.h>

#include "exportdecl.h"

namespace cnoid
{

class SgOctomap;
typedef ref_ptr<SgOctomap> SgOctomapPtr;

class CNOID_EXPORT SgOctomap : public octomap::OcTree, public Referenced
{
public:
    SgOctomap(float resolution);
    ~SgOctomap();

    void addPointSet(const SgPointSet &pt);
    void addPoints(const std::vector<Vector3> &pt);
    // void addVertexArray()

    void addBoxPrimitives(SgGroupPtr sgg, SgMaterial *mat = nullptr);

    Vector3 offset;
    Vector3 scale;
};

}
#endif
