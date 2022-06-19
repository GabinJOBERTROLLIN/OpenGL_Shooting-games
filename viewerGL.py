#---------------------------------------------------------
# Créateur : CHRONOWSKI Amaury / JOBERT--ROLLIN Gabin 
# Sur la base d'un programme créé par DUPONT Thibault
# Crée le : 10/06/2022
# Programme : Permet de lancer un jeu de Shooter classique
#---------------------------------------------------------

#!/usr/bin/env python3

# Bibliothèques générales
import collections
import OpenGL.GL as GL
import glfw
import pyrr
import numpy as np
from sympy import euler
import time

# bibliothèques interne projet
from cpe3d import Object3D



global xCoord,yCoord
xCoord=0
yCoord=0



class ViewerGL:
    def __init__(self):
        # initialisation de la librairie GLFW
        glfw.init()

        # paramétrage du context OpenGL
        glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
        glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
        glfw.window_hint(glfw.OPENGL_FORWARD_COMPAT, GL.GL_TRUE)
        glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)

        # création et paramétrage de la fenêtre
        glfw.window_hint(glfw.RESIZABLE, False)
        self.window = glfw.create_window(1600, 1600, 'Shooter basique', None, None)

        # paramétrage de la fonction de gestion des évènements
        glfw.set_key_callback(self.window, self.key_callback)
        glfw.set_mouse_button_callback(self.window,self.Mouse_button_callback)

        # activation du context OpenGL pour la fenêtre
        glfw.make_context_current(self.window)
        glfw.swap_interval(1)

        # activation de la gestion de la profondeur
        GL.glEnable(GL.GL_DEPTH_TEST)

        # choix de la couleur de fond
        GL.glClearColor(0.5, 0.6, 0.9, 1.0)
        print(f"OpenGL: {GL.glGetString(GL.GL_VERSION).decode('ascii')}")
        self.objs = []
        self.cible=[]
        self.touch = {}



    def collision(self,cam,objet): 
    # Fonction qui gere la collision entre le joueur et les objets de l'enviornement;
    # Entrées : les coordonnées de la caméra avec le prochain déplacment, les coordonnées d'un objet 
        x=cam[0]
        z=cam[2]
        xobj=objet[0]
        zobj=objet[2]
        yobj=objet[1]
        xobjmin=xobj-1
        xobjmax=xobj+1
        zobjmin=zobj-1
        zobjmax=zobj+1

        # Vérifie si le prochain déplacment ne met pas la camméra dans un objet et enlève l'objet sol 
        # retourne si il n'y a pas collision (True) ou s'il y a collision (False)
        if (xobjmin<=x and x<=xobjmax) and (zobjmin<=z and z<=zobjmax) and yobj>0: 
            return False
        else:
            return True



    def run(self):
    # boucle d'affichage
        self.timeStart=-1
        self.partiEnCours=False
        coord=[0,0,0]
        while not glfw.window_should_close(self.window):
            GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT) # nettoyage de la fenêtre : fond et profondeur

            if self.partiEnCours==True:
                self.updateMouseCoord(self.window) # Rotate la caméra en fonction du déplacement de la sourie
                coord=self.update_key() # Récupère le déplacmant que va faire le joueur en fonction des touches 
                self.objs[-3].value=str(int(time.time()-self.timeStart))
            pose=self.cam.transformation.translation + pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.cam.transformation.rotation_euler), pyrr.Vector3(coord)) # Calcule la futrue position du joueur qui sera utilisé dans le scolision
            
            # Rregarde s'il y a colision pour chaque element du labyrinthe, sort s'il y a colision avec un seul élément 
            for obj in self.objs[:-4]:
                ok=self.collision(pose,obj.transformation.translation)
                if not ok:
                    break

            if ok: # Déplace le joueur s'il n'y a pas collision
                self.cam.transformation.translation += \
                    pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.cam.transformation.rotation_euler), pyrr.Vector3(coord))
                self.cam.transformation.translation[1]=1
                self.cam.transformation.rotation_center = self.cam.transformation.translation

            # Affiche tout les éléments au joueurs
            for obj in self.objs:
                GL.glUseProgram(obj.program)
                if isinstance(obj, Object3D):
                    self.update_camera(obj.program)
                obj.draw()

            # changement de buffer d'affichage pour éviter un effet de scintillement
            glfw.swap_buffers(self.window)

            # gestion des évènements
            glfw.poll_events()

    

    def add_object(self, obj):
        self.objs.append(obj)

    def add_cible(self,obj):
        self.cible.append(obj)

    def set_camera(self, cam):
        self.cam = cam
    

#CALLBACKS KEY AND MOUSE  
    def Mouse_button_callback(self,window, button, action,mods):
    # Mouse Callback
        global xCoord,yCoord
        R=0.6
        tousDetruit=True
        if button ==glfw.MOUSE_BUTTON_LEFT and action == glfw.PRESS:
            if self.partiEnCours==False:
                print ("bouton click")
                self.objs[-1].visible=False
                self.objs[-2].visible=False
                self.timeStart=time.time()
                self.partiEnCours=True
                glfw.set_cursor_pos(self.window,800,800)
                xCoord=800
                yCoord=800
            else:
                for obj in self.cible:
                    trObj=np.array(obj.transformation.translation).copy()
                   
                    AC=np.array(trObj)-np.array(self.cam.transformation.translation)
                    rot=np.array(self.cam.transformation.rotation_euler)

                    rot=[]
                    yaw=self.cam.transformation.rotation_euler[pyrr.euler.index().yaw]
                    pitch=self.cam.transformation.rotation_euler[pyrr.euler.index().pitch]
                    rot.append(np.sin(yaw))
                    rot.append(-np.sin(pitch)*np.cos(yaw))
                    rot.append(-np.cos(pitch)*np.cos(yaw))
                    b=2*np.dot(AC,rot)/(np.dot(rot,rot))
                    c=(np.dot(AC,AC)-R*R)/(np.dot(rot,rot))
                    
                    if b*b-4*c>=-0.1:
                        print("touche")
                        obj.visible=False
                    if obj.visible==True:
                        tousDetruit=False
                if tousDetruit==True:
                    self.partiEnCours=False


                    
                

    def key_callback(self, win, key, scancode, action, mods):
    # sortie du programme si appui sur la touche 'échappement'
        if key == glfw.KEY_ESCAPE and action == glfw.PRESS:
            glfw.set_window_should_close(win, glfw.TRUE)
        self.touch[key] = action
    
    def updateMouseCoord(self,win):
    # actualise les coordonnées de la caméra pour qu'elle suive le curseur, téléporte le curseur si il est bloqué à droite ou gauche de l'écran.
        global xCoord
        global yCoord
        x,y=glfw.get_cursor_pos(win)
        newX=(x-xCoord)/300
        newY=(y-yCoord)/300

        xCoord=x
        yCoord=y
        
        
        self.cam.transformation.rotation_euler[pyrr.euler.index().roll] += newY
        self.cam.transformation.rotation_euler[pyrr.euler.index().yaw] += newX
        if xCoord<=0:
            glfw.set_cursor_pos(win,1600,yCoord)
            xCoord=1600
        if xCoord>1600:
            glfw.set_cursor_pos(win,1,yCoord)
            xCoord=1

    def update_camera(self, prog):
        GL.glUseProgram(prog)
        # Récupère l'identifiant de la variable pour le programme courant
        loc = GL.glGetUniformLocation(prog, "translation_view")
        # Vérifie que la variable existe
        if (loc == -1) :
            print("Pas de variable uniforme : translation_view")
        # Modifie la variable pour le programme courant
        translation = -self.cam.transformation.translation
        GL.glUniform4f(loc, translation.x, translation.y, translation.z, 0)

        # Récupère l'identifiant de la variable pour le programme courant
        loc = GL.glGetUniformLocation(prog, "rotation_center_view")
        # Vérifie que la variable existe
        if (loc == -1) :
            print("Pas de variable uniforme : rotation_center_view")
        # Modifie la variable pour le programme courant
        rotation_center = self.cam.transformation.rotation_center
        GL.glUniform4f(loc, rotation_center.x, rotation_center.y, rotation_center.z, 0)

        rot = pyrr.matrix44.create_from_eulers(-self.cam.transformation.rotation_euler)
        loc = GL.glGetUniformLocation(prog, "rotation_view")
        if (loc == -1) :
            print("Pas de variable uniforme : rotation_view")
        GL.glUniformMatrix4fv(loc, 1, GL.GL_FALSE, rot)
    
        loc = GL.glGetUniformLocation(prog, "projection")
        if (loc == -1) :
            print("Pas de variable uniforme : projection")
        GL.glUniformMatrix4fv(loc, 1, GL.GL_FALSE, self.cam.projection)


    def update_key(self): 
    # Récupère les information des touche permetant le délcement et retourne le déplacemant à faire
        coord=[0,0,0]
        if glfw.KEY_W in self.touch and self.touch[glfw.KEY_W] > 0:
            coord[2]+=-0.2

        if glfw.KEY_S in self.touch and self.touch[glfw.KEY_S] > 0:
            coord[2]+=0.2

        if glfw.KEY_A in self.touch and self.touch[glfw.KEY_A] > 0:
            coord[0]+=-0.2

        if glfw.KEY_D in self.touch and self.touch[glfw.KEY_D] > 0:
            coord[0]+=0.2
        return coord

