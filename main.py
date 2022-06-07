from zlib import Z_NO_COMPRESSION
from viewerGL import ViewerGL
import glutils
from mesh import Mesh
from cpe3d import Object3D, Camera, Transformation3D, Text
import numpy as np
import OpenGL.GL as GL
import pyrr

def main():
    
    Stage=open('stage.txt','r')
    stage=Stage.readlines()
    Stage.close()
    map=[]
    for nbligne in stage:
        map.append([])
    ligneM=0
    for ligne in stage:
        ligne=ligne[1:-2]
        print(ligne)
        ligne=ligne.split()
        for i in ligne:
            map[ligneM].append(int(i))
        ligneM+=1
    
    xcord=-32
    zcord=-32

    viewer = ViewerGL()

    viewer.set_camera(Camera())
    viewer.cam.transformation.translation.y = 2
    viewer.cam.transformation.rotation_center = viewer.cam.transformation.translation.copy()

    program3d_id = glutils.create_program_from_file('shader.vert', 'shader.frag')
    programGUI_id = glutils.create_program_from_file('gui.vert', 'gui.frag')

    m = Mesh.load_obj('stegosaurus.obj')
    m.normalize()
    m.apply_matrix(pyrr.matrix44.create_from_scale([2, 2.5, 2, 1]))
    n = Mesh.load_obj('stegosaurus.obj')
    n.normalize()
    n.apply_matrix(pyrr.matrix44.create_from_scale([2, 2, 2, 1]))
    texturew = glutils.load_texture('Wall.jpg')
    textures = glutils.load_texture('stegosaurus.jpg')
    for ligne in map:
        # print('a')
        for bloc in ligne:
            # print('b')
            if bloc==1:
                tr = Transformation3D(euler=pyrr.euler.create(), center=pyrr.Vector3(), translation=pyrr.Vector3([xcord,2.5,zcord]))
                o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texturew, tr)
                viewer.add_object(o)
            if bloc==2:
                tr = Transformation3D(euler=pyrr.euler.create(), center=pyrr.Vector3(), translation=pyrr.Vector3([xcord,0,zcord]))
                o = Object3D(n.load_to_gpu(), n.get_nb_triangles(), program3d_id, textures, tr)
                viewer.add_object(o)
            print(xcord, zcord)
            xcord+=2
        xcord = -32
        zcord+=2         

    m = Mesh()
    p0, p1, p2, p3 = [-32, 0, -32], [32, 0, -32], [32, 0, 32], [-32, 0, 32]
    n, c = [0, 1, 0], [1, 1, 1]
    t0, t1, t2, t3 = [0, 0], [1, 0], [1, 1], [0, 1]
    m.vertices = np.array([[p0 + n + c + t0], [p1 + n + c + t1], [p2 + n + c + t2], [p3 + n + c + t3]], np.float32)
    m.faces = np.array([[0, 1, 2], [0, 2, 3]], np.uint32)
    texture = glutils.load_texture('Flor.jpg')
    o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, Transformation3D())
    viewer.add_object(o)

    viewer.run()


if __name__ == '__main__':
    main()