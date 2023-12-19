#include "MergeBoxes.h"
#include <cnoid/MeshGenerator>

using namespace cnoid;

MergeBoxes::MergeBoxes(size_t _xx, size_t _yy, size_t _zz) : size_x(_xx), size_y(_yy), size_z(_zz), size_xy(_xx*_yy)
{
}

bool MergeBoxes::setPoints(std::vector<int> &_points)
{
    if ( points.size() != size_x * size_y * size_z ) {
        return false;
    }
    points = _points;
    return true;
}

void MergeBoxes::getSize(int &_x, int &_y, int &_z)
{
    _x = size_x; _y = size_y; _z = size_z;
}

void MergeBoxes::mergePoints()
{
    for (size_t cur_idx = 0; cur_idx < points.size(); cur_idx++) {
        if (points[cur_idx] < 0) { // there is no point
            continue;
        }
        int x_, y_, z_;
        index_to_coords(cur_idx, x_, y_, z_);
#if 0
        std::cout << std::endl;
        std::cout << "idx: " << cur_idx;
        std::cout << ", " << x_;
        std::cout << ", " << y_;
        std::cout << ", " << z_ << std::endl;
#endif
        Direction dir = canExpand(x_, y_, z_);
#if 0
        std::cout << "dir: " << dir << std::endl;
#endif
        if (dir == Direction::None) { // isolated point
            continue;
        }
        //
        int sx_, ex_, sy_, ey_, sz_, ez_;
        sx_ = ex_ = x_;
        sy_ = ey_ = y_;
        sz_ = ez_ = z_;
        size_t expand_idx = cur_idx;
        switch(dir) {
        case Direction::DIR_P_X:
            ex_ += 1;
            expand_idx += 1;
            break;
        case Direction::DIR_M_X:
            sx_ -= 1;
            expand_idx -= 1;
            break;
        case Direction::DIR_P_Y:
            ey_ += 1;
            expand_idx += size_y;
            break;
        case Direction::DIR_M_Y:
            sy_ -= 1;
            expand_idx -= size_y;
            break;
        case Direction::DIR_P_Z:
            ez_ += 1;
            expand_idx += size_xy;
            break;
        case Direction::DIR_M_Z:
            sz_ -= 1;
            expand_idx -= size_xy;
            break;
        }
#if 0
        std::cout << "(" << sx_;
        std::cout << ", " << ex_ << ")";
        std::cout << "(" << sy_;
        std::cout << ", " << ey_ << ")";
        std::cout << "(" << sz_;
        std::cout << ", " << ez_ << ")" << std::endl;
#endif
        Box cur_box(sx_, ex_, sy_, ey_, sz_, ez_);
        cur_box.id = points[cur_idx];
        points[cur_idx] = -1;
        points[expand_idx] = -1;

        while (true) {
            Direction cur_dir = canExpand(cur_box);
#if 0
            std::cout << "ld: " << cur_dir << std::endl;
#endif
            if (cur_dir == Direction::None) {
                break;
            }
#if 0
            std::cout << "s: ";
            for(int i = 0; i < points.size(); i++) {
                std::cout << " " << points[i];
            }
            std::cout << std::endl;
#endif
            expand(cur_box, cur_dir);
#if 0
            std::cout << "e: ";
            for(int i = 0; i < points.size(); i++) {
                std::cout << " " << points[i];
            }
            std::cout << std::endl;
#endif
        }
        boxes.push_back(cur_box);
    }
}

void MergeBoxes::addBoxPrimitives(SgGroupPtr sgg, const Vector3 &offset, const Vector3 &size, SgMaterial *mat)
{
    double sz_x = size.x();
    double sz_y = size.y();
    double sz_z = size.z();

    for(size_t i = 0; i < boxes.size(); i++) {
        Box &cur_box_ = boxes[i];
        Vector3 size_;
        Vector3 pos_;
        pos_ += offset;
        //
        MeshGenerator mg;
        SgMesh *mesh_ = mg.generateBox(size_);
        SgPosTransform *trs_ = new SgPosTransform();
        SgShape *sp_ = new SgShape();
        //
        sp_->setMesh(mesh_);
        sp_->setMaterial(mat);
        //
        trs_->setTranslation(pos_);
        trs_->addChild(sp_);
        //
        sgg->addChild(trs_);
    }
}
