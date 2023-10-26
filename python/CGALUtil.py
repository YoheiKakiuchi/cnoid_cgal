## import
import cnoid.Util as cutil
from irsl_choreonoid.irsl_draw_object import coordsWrapper as coordsWrapper
from irsl_choreonoid.draw_coords import GeneralDrawInterfaceWrapped as DrawInterface
import irsl_choreonoid.make_shapes as mkshapes
#from cnoid.CGALMesh import SgMeshDbl
from cnoid.CGALMesh import CGALMesh

def cgalShape(shape, material=None, wrapped=True, rawShape=False, coords=None, **kwargs):
    shape.setMesh(CGALMesh(shape.mesh))

    if material is None:
        mat = mkshapes.generateMaterial(**kwargs)
        if mat is not None:
            shape.setMaterial(mat)
    elif isinstance(material, cutil.SgMaterial):
        shape.setMaterial(material)

    if rawShape:
        return shape

    ret = cutil.SgPosTransform()
    ret.addChild(shape)
    if wrapped:
        ret = coordsWrapper(ret, original_object=shape)
        if coords is not None:
            ret.newcoords(coords)
    else:
        if coords is not None:
            ret.setPosition(coords.cnoidPosition)
    return ret

#def loadScene(fname, wrapped=True, rawShape=False, coords=None, **kwargs):
def loadScene(fname, wrapped=True, rawShape=False, coords=None, **kwargs):
    return cgalShape(mkshapes.loadScene(fname, rawShape=True, **kwargs),
                     wrapped=wrapped, rawShape=rawShape, coords=coords, material=False, **kwargs)
#def loadMesh(fname, wrapped=True, rawShape=False, coords=None, **kwargs):
def loadMesh(fname, wrapped=True, rawShape=False, coords=None, **kwargs):
    return cgalShape(mkshapes.loadMesh(fname, rawShape=True, **kwargs),
                     wrapped=wrapped, rawShape=rawShape, coords=coords, material=False, **kwargs)
#def makeBox(x, y = None, z = None, wrapped=True, rawShape=False, coords=None, **kwargs):
def makeBox(x, y = None, z = None, wrapped=True, rawShape=False, coords=None, **kwargs):
    return cgalShape(mkshapes.makeBox(x, y, z, rawShape=True, **kwargs),
                     wrapped=wrapped, rawShape=rawShape, coords=coords, material=False, **kwargs)
#def makeCylinder(radius, height, wrapped=True, rawShape=False, coords=None, **kwargs):
def makeCylinder(radius, height, wrapped=True, rawShape=False, coords=None, **kwargs):
    return cgalShape(mkshapes.makeCylinder(radius, height, rawShape=True, **kwargs),
                     wrapped=wrapped, rawShape=rawShape, coords=coords, material=False, **kwargs)
#def makeSphere(radius, wrapped=True, rawShape=False, coords=None, **kwargs):
def makeSphere(radius, wrapped=True, rawShape=False, coords=None, **kwargs):
    return cgalShape(mkshapes.makeSphere(radius, rawShape=True, **kwargs),
                     wrapped=wrapped, rawShape=rawShape, coords=coords, material=False, **kwargs)
#def makeCone(radius, height, wrapped=True, rawShape=False, coords=None, **kwargs):
def makeCone(radius, height, wrapped=True, rawShape=False, coords=None, **kwargs):
    return cgalShape(mkshapes.makeCone(radius, height, rawShape=True, **kwargs),
                     wrapped=wrapped, rawShape=rawShape, coords=coords, material=False, **kwargs)
#def makeCapsule(radius, height, wrapped=True, rawShape=False, coords=None, **kwargs):
def makeCapsule(radius, height, wrapped=True, rawShape=False, coords=None, **kwargs):
    return cgalShape(mkshapes.makeCapsule(radius, height, rawShape=True, **kwargs),
                     wrapped=wrapped, rawShape=rawShape, coords=coords, material=False, **kwargs)
#def makeTorus(radius, corssSectionRadius, beginAngle = None, endAngle = None, wrapped=True, rawShape=False, coords=None, **kwargs):
def makeTorus(radius, corssSectionRadius, beginAngle = None, endAngle = None, wrapped=True, rawShape=False, coords=None, **kwargs):
    return cgalShape(mkshapes.makeTorus(radius, corssSectionRadius, beginAngle=beginAngle, endAngle=endAngle, rawShape=True, **kwargs),
                     wrapped=wrapped, rawShape=rawShape, coords=coords, material=False, **kwargs)
#def makeExtrusion(crossSection, spine, wrapped=True, rawShape=False, coords=None, **kwargs):
def makeExtrusion(crossSection, spine, wrapped=True, rawShape=False, coords=None, **kwargs):
    return cgalShape(mkshapes.makeExtrusion(crossSection, spine, rawShape=True, **kwargs),
                     wrapped=wrapped, rawShape=rawShape, coords=coords, material=False, **kwargs)
#def makeTetrahedron(base_width, base_height, height, base_center=None, center_x=None, center_y=None, wrapped=True, rawShape=False, coords=None, **kwargs):
def makeTetrahedron(base_width, base_height, height, base_center=None, center_x=None, center_y=None, wrapped=True, coords=None, **kwargs):
    return cgalShape(mkshapes.makeTetrahedron(base_width, base_height, height, base_center, center_x, center_y, rawShape=True, **kwargs),
                     wrapped=wrapped, coords=coords, material=False, **kwargs)

def transformForBooleanOperation(object0, object1, SgRoot=None):
    res0 = mkshapes.extractShapes(object0)
    res1 = mkshapes.extractShapes(object1)
    if len(res0) == 0 or len(res1) == 0:
        raise Exception(f'{object0} : num shapes is {len(res0)}, {object1} : num shapes is {len(res1)}')

    res0 = res0[0] ## pick first one
    res1 = res1[0] ## pick first one
    shape0 = res0[0]
    shape1 = res1[0]
    if not isinstance(shape0.mesh, CGALMesh) or not isinstance(shape1.mesh, CGALMesh):
        raise Exception(f'shape0 : {shape0.mesh} : {type(shape0.mesh)}, shape1 : {shape1.mesh} : {type(shape1.mesh)}')

    if SgRoot is None:
        if res0[1] is None or res1[1] is None:
            return (shape0, shape1, None)
        else:
            return (shape0, shape1, res0[1].transformation(res1[1]))
    elif type(SgRoot) is DrawInterface:
        SgRoot = SgRoot.SgPosTransform

    cds0=None
    cds1=None
    res = mkshapes.extractShapes(SgRoot)
    for obj, cds in res:
        if obj is shape0:
            cds0 = cds
        elif obj is shape1:
            cds1 = cds
    if cds0 is None or cds1 is None:
        raise Exception(f'cds0 : {cds0}, cds1 : {cds1}')
    if cds0.equal(cds1, 1e-10):
        return (shape0, shape1, None)
    else:
        return (shape0, shape1, cds0.transformation(cds1))

def booleanUnion(object0, object1, SgRoot=None, update=True):
    if isinstance(object0, coordsWrapper):
        obj0 = object0.target
    else:
        obj0 = object0
    if isinstance(object1, coordsWrapper):
        obj1 = object1.target
    else:
        obj1 = object1
    shape0, shape1, cds = transformForBooleanOperation(obj0, obj1, SgRoot=SgRoot)
    if cds is None:
        shape0.mesh.booleanUnion(shape1.mesh)
    else:
        shape0.mesh.booleanUnionWithTransform(shape1.mesh, cds.cnoidPosition)
    if update:
        shape0.mesh.notifyUpdate()
        #if isinstance(object0, coordsWrapper):
        #    object0.updateTarget()
        #elif hasattr(object0, 'notifyUpdate'):
        #    object0.notifyUpdate()
    return object0

def createByUnion(object0, object1, SgRoot=None):
    if isinstance(object0, coordsWrapper):
        obj0 = object0.target
    else:
        obj0 = object0
    if isinstance(object1, coordsWrapper):
        obj1 = object1.target
    else:
        obj1 = object1
    shape0, shape1, cds = transformForBooleanOperation(obj0, obj1, SgRoot=SgRoot)
    if cds is None:
        res = shape0.mesh.createByUnion(shape1.mesh)
    else:
        res = shape0.mesh.createByUnionWithTransform(shape1.mesh, cds.cnoidPosition)
    if isinstance(object0, coordsWrapper):
        trans = cutil.SgPosTransform()
        sp = cutil.SgShape()
        sp.setMesh(res)
        sp.setMaterial(shape0.material)
        trans.addChild(sp)
        ret = coordsWrapper(trans, original_object=sp)
        ret.newcoords(object0)
        return ret
    elif isinstance(object0, cutil.SgPosTransform):
        trans = cutil.SgPosTransform()
        trans.setPosition(object0.T)
        sp = cutil.SgShape()
        sp.setMesh(res)
        sp.setMaterial(shape0.material)
        trans.addChild(sp)
        return trans
    return res

def booleanDifference(object0, object1, SgRoot=None, update=True):
    if isinstance(object0, coordsWrapper):
        obj0 = object0.target
    else:
        obj0 = object0
    if isinstance(object1, coordsWrapper):
        obj1 = object1.target
    else:
        obj1 = object1
    shape0, shape1, cds = transformForBooleanOperation(obj0, obj1, SgRoot=SgRoot)
    if cds is None:
        shape0.mesh.booleanDifference(shape1.mesh)
    else:
        shape0.mesh.booleanDifferenceWithTransform(shape1.mesh, cds.cnoidPosition)
    if update:
        shape0.mesh.notifyUpdate()
        #if isinstance(object0, coordsWrapper):
        #    object0.updateTarget()
        #elif hasattr(object0, 'notifyUpdate'):
        #    object0.notifyUpdate()
    return object0

def createByDifference(object0, object1, SgRoot=None):
    if isinstance(object0, coordsWrapper):
        obj0 = object0.target
    else:
        obj0 = object0
    if isinstance(object1, coordsWrapper):
        obj1 = object1.target
    else:
        obj1 = object1
    shape0, shape1, cds = transformForBooleanOperation(obj0, obj1, SgRoot=SgRoot)
    if cds is None:
        res = shape0.mesh.createByDifference(shape1.mesh)
    else:
        res = shape0.mesh.createByDifferenceWithTransform(shape1.mesh, cds.cnoidPosition)
    if isinstance(object0, coordsWrapper):
        trans = cutil.SgPosTransform()
        sp = cutil.SgShape()
        sp.setMesh(res)
        sp.setMaterial(shape0.material)
        trans.addChild(sp)
        ret = coordsWrapper(trans, original_object=sp)
        ret.newcoords(object0)
        return ret
    elif isinstance(object0, cutil.SgPosTransform):
        trans = cutil.SgPosTransform()
        trans.setPosition(object0.T)
        sp = cutil.SgShape()
        sp.setMesh(res)
        sp.setMaterial(shape0.material)
        trans.addChild(sp)
        return trans
    return res

def booleanIntersection(object0, object1, SgRoot=None, update=True):
    if isinstance(object0, coordsWrapper):
        obj0 = object0.target
    else:
        obj0 = object0
    if isinstance(object1, coordsWrapper):
        obj1 = object1.target
    else:
        obj1 = object1
    shape0, shape1, cds = transformForBooleanOperation(obj0, obj1, SgRoot=SgRoot)
    if cds is None:
        shape0.mesh.booleanIntersection(shape1.mesh)
    else:
        shape0.mesh.booleanIntersectionWithTransform(shape1.mesh, cds.cnoidPosition)
    if update:
        shape0.mesh.notifyUpdate()
        #if isinstance(object0, coordsWrapper):
        #    object0.updateTarget()
        #elif hasattr(object0, 'notifyUpdate'):
        #    object0.notifyUpdate()
    return object0

def createByIntersection(object0, object1, SgRoot=None):
    if isinstance(object0, coordsWrapper):
        obj0 = object0.target
    else:
        obj0 = object0
    if isinstance(object1, coordsWrapper):
        obj1 = object1.target
    else:
        obj1 = object1
    shape0, shape1, cds = transformForBooleanOperation(obj0, obj1, SgRoot=SgRoot)
    if cds is None:
        res = shape0.mesh.createByIntersection(shape1.mesh)
    else:
        res = shape0.mesh.createByIntersectionWithTransform(shape1.mesh, cds.cnoidPosition)
    if isinstance(object0, coordsWrapper):
        trans = cutil.SgPosTransform()
        sp = cutil.SgShape()
        sp.setMesh(res)
        sp.setMaterial(shape0.material)
        trans.addChild(sp)
        ret = coordsWrapper(trans, original_object=sp)
        ret.newcoords(object0)
        return ret
    elif isinstance(object0, cutil.SgPosTransform):
        trans = cutil.SgPosTransform()
        trans.setPosition(object0.T)
        sp = cutil.SgShape()
        sp.setMesh(res)
        sp.setMaterial(shape0.material)
        trans.addChild(sp)
        return trans
    return res
