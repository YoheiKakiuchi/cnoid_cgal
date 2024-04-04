# jupyter console --kernel=choreonoid
exec(open('/choreonoid_ws/install/share/irsl_choreonoid/sample/irsl_import.py').read())
from cnoid.CGALMesh import SgOctomap
di=DrawInterface()

oct_=SgOctomap(0.1)

pp=mkshapes.makePoints(fv( ( 0.05,  0.05, 0.05),
                           ( 0.05,  0.15, 0.05),
                           ( 0.05,  0.05, 0.15),
                           ( 0.05,  0.15, 0.15),
                           ( 0.15,  0.05, 0.05),
                           ##( 0.15,  0.15, 0.05),
                           ( 0.15,  0.05, 0.15),
                           ( 0.15,  0.15, 0.15),
                           dtype='float32' ), rawShape=True)
oct_.addPointSet(pp)
oct_.prune()
sg_group = cutil.SgGroup()
sg_mat = cutil.SgMaterial()
oct_.addBoxPrimitives(sg_group, sg_mat)

di.addObject(sg_group)
