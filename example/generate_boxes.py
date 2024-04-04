# jupyter console --kernel=choreonoid
exec(open('/choreonoid_ws/install/share/irsl_choreonoid/sample/irsl_import.py').read())

def convertToBoxes(cgalmesh, dim=None, dim_x=1, dim_y=1, dim_z=1, material=None):
    bbx_=cgalmesh.boundingBox()
    ##
    if dim is not None:
        dim_x = dim
        dim_y = dim
        dim_z = dim
    ##
    bbs_=bbx_.size()
    mb_size = fv(bbs_[0]/dim_x, bbs_[1]/dim_y, bbs_[2]/dim_z)
    mb_offset = bbx_.min() + mb_size/2
    ##
    mb=MergeBoxes(dim_x, dim_y, dim_z)
    mb.boxSize=mb_size
    mb.offset=mb_offset
    res=cgalmesh.addPointsMergeBoxes(mb)
    ##
    if res == 0:
        mb.mergePoints()
        ##mb.sizeOfBoxes
        gg  = cutil.SgGroup()
        if material is None:
            material = cutil.SgMaterial()
        mb.addBoxPrimitives(gg, material)
        return gg

def generateOctomap(cgalmesh, resolution=0.001, st_x=0, ed_x=100, st_y=0, ed_y=100,
                    st_z=0, ed_z=100, off_x=0.0, off_y=0.0, off_z=0.0, size_hint=None, scale=None):
    if size_hint is not None:
        cgalmesh.updateBundingBox()
        bb=cgalmesh.boundingBox()
        bmin = bb.min()
        bmax = bb.max()
        bsize = bb.size()
        mx = max(bsize)
        hh = bsize/mx
        ##
        sz = size_hint/math.pow(hh[0] * hh[1] * hh[2], 1.0/3)

        xs = int(hh[0] * sz)
        ys = int(hh[1] * sz)
        zs = int(hh[2] * sz)
        ##print('{} {} {}'.format(xs, ys, zs))
        print('size_hint is not implemented yet!')
        return

    off_ = fv(off_x, off_y, off_z, dtype='float32')
    pts=cutil.SgPointSet()
    pts.setVertices(fv((0, 0, 0), dtype='float32'))
    ##
    idx_lst=[0]
    cntr=1
    for xx in range(st_x, ed_x):
        for yy in range(st_y, ed_y):
            for zz in range(st_z, ed_z):
                idx_lst.append(cntr)
                cntr += 1
                pts.appendVertex(resolution * fv(xx+0.5, yy+0.5, zz+0.5, dtype='float32') + off_)
    result=cgalmesh.checkInside(pts)
    ##
    veclst=[]
    for res, idx in zip(result, idx_lst):
        if res > 0:
            veclst.append(pts.vertex(idx))
    ##
    pts.setVertices(npa(veclst, dtype='float32'))
    ##
    oct_=SgOctomap(resolution)
    oct_.addPointSet(pts)
    #oct_.prune()
    #oct_.expand()
    sg_group  = cutil.SgGroup()
    sg_mat    = cutil.SgMaterial()
    oct_.addBoxPrimitives(sg_group, sg_mat)
    return (sg_group, pts, off_)

###
#di=DrawInterface()
#
#cgalmesh = mkshapes.loadMesh('/home/irsl/src/robot_assembler_config_IRSL/meshes/x_series/short_frame_mm.stl', meshScale=0.001)
#sg_g_box = cgal.convertToBoxes(cgalmesh, dim=20)
#(sg_g_oct, pts, off_) = cgal.generateOctomap(cgalmesh)
#
#di.addObject(sg_g_box)
#
#di.addObject(sg_g_oct)
###
