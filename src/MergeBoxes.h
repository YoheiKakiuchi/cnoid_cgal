#ifndef __CHOREONOID_MERGE_BOXES_LIB_H__
#define __CHOREONOID_MERGE_BOXES_LIB_H__

#include <vector>
#include <cnoid/SceneDrawables>

#include "exportdecl.h"

namespace cnoid
{

class CNOID_EXPORT MergeBoxes
{

public:
    MergeBoxes(size_t _xx, size_t _yy, size_t _zz);

    bool setPoints(std::vector<int> &_points);
    void getSize(int &_x, int &_y, int &_z);
    void mergePoints();

    //SceneDrawables
    void addBoxPrimitives(SgGroupPtr sgg, const Vector3 &offset, const Vector3 &size, SgMaterial *mat = nullptr);

protected:
    size_t size_x;
    size_t size_y;
    size_t size_z;

    size_t size_xy;

    std::vector<int> points;

public:
    class Box
    {
    public:
        Box(int _sx, int _ex, int _sy, int _ey, int _sz, int _ez) :
            sx(_sx), sy(_sy), sz(_sz), ex(_ex), ey(_ey), ez(_ez)
        { }
        // s?, e? >= 0
        // sx <= ex
        // sx = N, ex = N  :==>  size = 1 (at index N)
        int sx; // index of start
        int sy;
        int sz;
        int ex; // index of end
        int ey;
        int ez;
        int id;
        inline void getSize(int &_x, int &_y, int &_z)
        {
            _x = ex - sx + 1;
            _y = ey - sy + 1;
            _z = ez - sz + 1;
        }
        inline void getCenter(float &_x, float &_y, float &_z)
        {
            _x = (sx + ex)*0.5;
            _y = (sy + ey)*0.5;
            _z = (sz + ez)*0.5;
        }
        inline void getActualCenter(double &_x, double &_y, double &_z,
                                    const Vector3 &offset, const Vector3 &size)
        {
            float _cx, _cy, _cz;
            getCenter(_cx, _cy, _cz);

            _x = _cx * size.x() + offset.x();
            _y = _cy * size.y() + offset.y();
            _z = _cz * size.z() + offset.z();
        }
    };

    enum Direction {
        None,
        DIR_P_X,
        DIR_P_Y,
        DIR_P_Z,
        DIR_M_X,
        DIR_M_Y,
        DIR_M_Z,
    };

public:
    std::vector<Box> boxes;

protected:
    inline Direction canExpand(size_t _x, size_t _y, size_t _z)
    {
        size_t idx = (size_xy * _z) + ( size_x * _y ) + _x;
        //size_t max_idx = points.size();

        if (_x > 0) {
            if (points[idx-1] >= 0) {
                return Direction::DIR_M_X;
            }
        }
        if (_x < size_x -1) {
            if (points[idx+1] >= 0) {
                return Direction::DIR_P_X;
            }
        }
        if (_y > 0) {
            if (points[idx-size_x] >= 0) {
                return Direction::DIR_M_Y;
            }
        }
        if (_y < size_y -1) {
            if (points[idx+size_x] >= 0) {
                return Direction::DIR_P_Y;
            }
        }
        if (_z > 0) {
            if (points[idx-size_xy] >= 0) {
                return Direction::DIR_M_Z;
            }
        }
        if (_z < size_z -1) {
            if (points[idx+size_xy] >= 0) {
                return Direction::DIR_P_Z;
            }
        }
        return Direction::None;
    }
    inline Direction canExpand(Box &_box)
    {
        if (canExpand_YZ(_box.ex+1, _box.sy, _box.ey, _box.sz, _box.ez)) {
            return Direction::DIR_P_X;
        }
        if (canExpand_YZ(_box.sx-1, _box.sy, _box.ey, _box.sz, _box.ez)) {
            return Direction::DIR_M_X;
        }
        if (canExpand_XZ(_box.ey+1, _box.sx, _box.ex, _box.sz, _box.ez)) {
            return Direction::DIR_P_Y;
        }
        if (canExpand_XZ(_box.sy-1, _box.sx, _box.ex, _box.sz, _box.ez)) {
            return Direction::DIR_M_Y;
        }
        if (canExpand_XY(_box.ez+1, _box.sx, _box.ex, _box.sy, _box.ey)) {
            return Direction::DIR_P_Z;
        }
        if (canExpand_XY(_box.sz-1, _box.sx, _box.ex, _box.sy, _box.ey)) {
            return Direction::DIR_M_Z;
        }
        return Direction::None;
    }
    inline void expand(Box &_box, Direction _dir)
    {
        switch(_dir) {
        case Direction::DIR_P_X:
            removeId_YZ(_box.ex+1, _box.sy, _box.ey, _box.sz, _box.ez);
            _box.ex += 1;
            break;
        case Direction::DIR_M_X:
            removeId_YZ(_box.sx-1, _box.sy, _box.ey, _box.sz, _box.ez);
            _box.sx -= 1;
            break;
        case Direction::DIR_P_Y:
            removeId_XZ(_box.ey+1, _box.sx, _box.ex, _box.sz, _box.ez);
            _box.ey += 1;
            break;
        case Direction::DIR_M_Y:
            removeId_XZ(_box.sy-1, _box.sx, _box.ex, _box.sz, _box.ez);
            _box.sy -= 1;
            break;
        case Direction::DIR_P_Z:
            removeId_XY(_box.ez+1, _box.sx, _box.ex, _box.sy, _box.ey);
            _box.ez += 1;
            break;
        case Direction::DIR_M_Z:
            removeId_XY(_box.sz-1, _box.sx, _box.ex, _box.sy, _box.ey);
            _box.sz -= 1;
            break;
        }
    }
    inline void removeId_YZ(int _xx, int _sy, int _ey, int _sz, int _ez)
    {
        for(int zz = _sz; zz < _ez+1; zz++) {
            size_t idx = size_xy * zz + size_x * _sy + _xx;
            for(int yy = _sy; yy < _ey+1; yy++) {
                points[idx] = -1;
                idx += size_y;
            }
        }
    }
    inline void removeId_XZ(int _yy, int _sx, int _ex, int _sz, int _ez)
    {
        for(int zz = _sz; zz < _ez+1; zz++) {
            size_t idx = size_xy * zz + size_x * _yy + _sx;
            for(int xx = _sx; xx < _ex+1; xx++) {
                points[idx] = -1;
                idx += 1;
            }
        }
    }
    inline void removeId_XY(int _zz, int _sx, int _ex, int _sy, int _ey)
    {
        for(int yy = _sy; yy < _ey+1; yy++) {
            size_t idx = size_xy * _zz + size_x * yy + _sx;
            for(int xx = _sx; xx < _ex+1; xx++) {
                points[idx] = -1;
                idx += 1;
            }
        }
    }
    inline bool canExpand_YZ(int _xx, int _sy, int _ey, int _sz, int _ez)
    {
        if(_xx < 0 || _xx >= size_x) return false;
        for(int zz = _sz; zz < _ez+1; zz++) {
            size_t idx = size_xy * zz + size_x * _sy + _xx;
            for(int yy = _sy; yy < _ey+1; yy++) {
                if (points[idx] < 0) {
                    return false;
                }
                idx += size_y;
            }
        }
        return true;
    }
    inline bool canExpand_XZ(int _yy, int _sx, int _ex, int _sz, int _ez)
    {
        if(_yy < 0 || _yy >= size_y) return false;
        for(int zz = _sz; zz < _ez+1; zz++) {
            size_t idx = size_xy * zz + size_x * _yy + _sx;
            for(int xx = _sx; xx < _ex+1; xx++) {
                if (points[idx] < 0) {
                    return false;
                }
                idx += 1;
            }
        }
        return true;
    }
    inline bool canExpand_XY(int _zz, int _sx, int _ex, int _sy, int _ey)
    {
        if(_zz < 0 || _zz >= size_z) return false;
        for(int yy = _sy; yy < _ey+1; yy++) {
            size_t idx = size_xy * _zz + size_x * yy + _sx;
            for(int xx = _sx; xx < _ex+1; xx++) {
                if (points[idx] < 0) {
                    return false;
                }
                idx += 1;
            }
        }
        return true;
    }
    inline void index_to_coords(size_t _index, int &res_x, int &res_y, int &res_z)
    {
        int _tmp = _index % size_xy;
        res_z = _index / size_xy;
        res_y = _tmp / size_x;
        res_x = _tmp % size_x;
    }
};

}

#endif
