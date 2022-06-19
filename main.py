#---------------------------------------------------------
# Créateur : CHRONOWSKI Amaury / JOBERT--ROLLIN Gabin 
# Sur la base d'un programme créé par DUPONT Thibault
# Crée le : 10/06/2022
# Programme : Permet de lancer un jeu de Shooter classique
#---------------------------------------------------------

# Bibliothèques générales
from zlib import Z_NO_COMPRESSION
import glutils
import numpy as np
import OpenGL.GL as GL
import pyrr

# bibliothèques interne projet
from mesh import Mesh
from cpe3d import Object3D, Camera, Transformation3D, Text
from viewerGL import ViewerGL




def main():
# Fonction qui lance le programme
    
    # Ouverture du fichier où la disposition de l'espace de jeu est stocké (enseble de 16 lignes de 16 valeurs pour faire un carrée, qui a la valeur 0, le vide, 1, un mur, 2, une cible)
    Stage=open('stage.txt','r')
    stage=Stage.readlines()
    Stage.close()

    # Variable ou sera stocké chaque information de l'espace de jeu
    map=[]

    # Transformation du fichier en information exploitable
    for nbligne in stage:
        map.append([])
    ligneM=0
    for ligne in stage:
        ligne=ligne.split()
        for element in ligne:
            map[ligneM].append(int(element))
        ligneM+=1
    
    # Coordonnées de départ pour le placement des éléments
    xcord=-15
    zcord=-15

    # Création de la caméra qui sera le joueur
    viewer = ViewerGL()
    viewer.set_camera(Camera())

    # Initialisation des shaders
    program3d_id = glutils.create_program_from_file('shader.vert', 'shader.frag')
    programGUI_id = glutils.create_program_from_file('gui.vert', 'gui.frag')

    # Initialisation de la structure et texture du "cube" qui sera utilisée pour faire les murs
    m = Mesh.load_obj('cube.obj')
    m.normalize()
    vaoM=m.load_to_gpu()
    m.apply_matrix(pyrr.matrix44.create_from_scale([1, 1.25, 1, 1]))
    textureW = glutils.load_texture('Wall.jpg')
    
    # Initialisation de la structure et texture du "stegosaurus" sera utilisée pour faire les cibles
    n = Mesh.load_obj('stegosaurus.obj')
    n.normalize()
    vaoN=n.load_to_gpu()
    n.apply_matrix(pyrr.matrix44.create_from_scale([1, 1, 1, 1]))
    textureS = glutils.load_texture('stegosaurus.jpg')

    # Palcemant des éléments selon les données, on fait ligne par ligne élément par élément
    for ligne in map:
        for bloc in ligne:
            if bloc==1: # Le 1 Represente les murs, donc on place un mur
                tr = Transformation3D(euler=pyrr.euler.create(), center=pyrr.Vector3(), translation=pyrr.Vector3([xcord,1,zcord]))
                o = Object3D(vaoM, m.get_nb_triangles(), program3d_id, textureW, tr)
                viewer.add_object(o)
            if bloc==2: # Le 2 Represente les cibles, donc on place une cible
                tr = Transformation3D(euler=pyrr.euler.create(), center=pyrr.Vector3(), translation=pyrr.Vector3([xcord,0.5,zcord]))
                o = Object3D(vaoN, n.get_nb_triangles(), program3d_id, textureS, tr)
                viewer.add_object(o)
                viewer.add_cible(o)
            if bloc==0 and ( xcord==-15 or xcord==15 or zcord==-15 or zcord==15):# Ici on fait en sorte de déplacer la caméra sur le bord du labyrinthe ou il y a du vide
                viewer.cam.transformation.translation.x = xcord
                viewer.cam.transformation.translation.y = 1
                viewer.cam.transformation.translation.z = zcord
                viewer.cam.transformation.rotation_center = viewer.cam.transformation.translation.copy()
            xcord+=2
        xcord = -15
        zcord+=2         
    
    # Création du sol
    m = Mesh()
    p0, p1, p2, p3 = [-16, 0, -16], [16, 0, -16], [16, 0, 16], [-16, 0, 16]
    n, c = [0, 1, 0], [1, 1, 1]
    t0, t1, t2, t3 = [0, 0], [1, 0], [1, 1], [0, 1]
    m.vertices = np.array([[p0 + n + c + t0], [p1 + n + c + t1], [p2 + n + c + t2], [p3 + n + c + t3]], np.float32)
    m.faces = np.array([[0, 1, 2], [0, 2, 3]], np.uint32)
    texture = glutils.load_texture('Flor.jpg')
    o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, Transformation3D())
    viewer.add_object(o)

    # Création du menu de démarage
    vao = Text.initalize_geometry()
    texture = glutils.load_texture('fontB.jpg')
    o = Text('+', np.array([-0.1, -0.05], np.float32), np.array([0.1, 0.05], np.float32), vao, 2, programGUI_id, texture)
    viewer.add_object(o)
    o = Text('time', np.array([0.8, 0.8], np.float32), np.array([1, 1], np.float32), vao, 2, programGUI_id, texture)
    viewer.add_object(o)
    o = Text('Shhoter Basique', np.array([-0.8, 0.3], np.float32), np.array([0.8, 0.8], np.float32), vao, 2, programGUI_id, texture)
    viewer.add_object(o)
    o = Text('new Game', np.array([-0.5, -0.2], np.float32), np.array([0.5, 0.3], np.float32), vao, 2, programGUI_id, texture)
    viewer.add_object(o)
    

    # Lance le jeu
    viewer.run()


if __name__ == '__main__':
    main()