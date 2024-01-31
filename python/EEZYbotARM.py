import serial
import tkinter as tk
import time 
from helpers import get_ArduinoUno_ports, ResettableTimer, File
import io
import os, sys
from PIL import Image, ImageTk
import continuous_threading
import numpy as np
#import threading

DIRPATH = 'python'
#IMAGEPATH = os.path.join(os.path.dirname(__file__), "images")
IMAGEPATH = "images"
SCALE = 0.3
DELAY_DEFAULT = 30

def create_serial():
    ports = get_ArduinoUno_ports()
    if len(ports) > 0:
        try:
            return serial.Serial(ports[0], 460800, timeout=1)
        except serial.serialutil.SerialException:
            pass

def scale(x):
    return round(x*SCALE)

###################################################################################
# Classe de base pour les objets à dessiner
###################################################################################
class Objet():
    def __init__(self, canvas):
        self.canvas  = canvas
    def calcul(self):
        pass
    def delete(self):
        pass
    def mousemove(self, event):
        pass

class Point(Objet):
    def __init__(self, canvas, pos, coul="black"):
        super().__init__(canvas) 

        # On met l'objet dans la liste des objets du canvas
        #self.canvas.add_objet(self)

        # Position en pixels "repère"
        self.x = pos[0]
        self.y = pos[1]
        self.r = 1
        self.c = coul

        # On dessine l'objet
        self.draw()

    def delete(self):
        if hasattr(self, 'id'): # l'objet a déja été dessiné
            self.canvas.delete(self.id)

    def draw(self):
        x, y = self.canvas.r2c(self.x, self.y)
        self.id = self.canvas.create_oval((x-self.r, y-self.r),
                                          (x+self.r, y+self.r),
                                               fill = self.c)

    def set_pos(self, pos):
        self.x = pos[0]
        self.y = pos[1]


class Vecteur(Objet):
    def __init__(self, canvas, pos, val, coul="black"):
        super().__init__(canvas) 

        # On met l'objet dans la liste des objets du canvas
        #self.canvas.add_objet(self)

        # Position en pixels "repère"
        self.x = pos[0]
        self.y = pos[1]
        self.px = val[0]
        self.py = val[1]
        self.r = 2
        self.c = coul

        # On dessine l'objet
        self.draw()

    def delete(self):
        if hasattr(self, 'id'): # l'objet a déja été dessiné
            self.canvas.delete(self.id)

    def draw(self):
        x0, y0 = self.canvas.r2c(self.x, self.y)
        x1, y1 = self.canvas.r2c(self.x+self.px, self.y+self.py)
        self.id = self.canvas.create_line(x0, y0,
                                          x1, y1,
                                          arrow = tk.LAST,
                                          fill = self.c,
                                          width = 2
                                          )

    def set_pos(self, pos):
        self.x = pos[0]
        self.y = pos[1]

    def set_val(self, val):
        self.px = val[0]
        self.py = val[1]

    

class Repere(Objet):
    def __init__(self, canvas, sp, spo = None):
        super().__init__(canvas) 
        self.sp = sp # sprite
        self.spo = spo # sprite Origine si pas celle du sprite sp
        self.r = 100
        self.orig = Point(canvas, (0, 0))
        self.axex = Vecteur(canvas, (0, 0), (0, 0), coul="red")
        self.axey = Vecteur(canvas, (0, 0), (0, 0), coul="green")
        self.calcul()
        # self.orig = Point(canvas, (sp.x, sp.y))
        # a = np.radians(sp.ax+sp.a)
        # c = np.cos(a)
        # s = np.sin(a)
        
        # self.axex = Vecteur(canvas, (sp.x, sp.y), 
        #                     (r*c, r*s))
        # self.axey = Vecteur(canvas, (sp.x, sp.y), 
        #                     (r*s, -r*c))


    def delete(self):
        self.orig.delete()
        self.axex.delete()
        self.axey.delete()

    def draw(self):
        self.orig.draw()
        self.axex.draw()
        self.axey.draw()
    
    def get_angle(self):
        return self.a

    def calcul(self):
        if self.spo is not None:
            x, y = self.spo.get_pos()
        else:
            x, y = self.sp.x, self.sp.y
        self.orig.set_pos((x, y))
        self.axex.set_pos((x, y))
        self.axey.set_pos((x, y))
        self.a = self.sp.a
        a = np.radians(self.sp.a)
        c = np.cos(a)
        s = np.sin(a)
        self.axex.set_val((self.r*c, self.r*s))
        self.axey.set_val((-self.r*s, self.r*c))

class Angle(Objet):
    def __init__(self, canvas, r0, r1, nom=""):
        super().__init__(canvas)
        self.r0 = r0
        self.r1 = r1
        self.r = 50
        self.nom = nom

    def delete(self):
        if hasattr(self, 'id'): # l'objet a déja été dessiné
            self.canvas.delete(self.id)
        if hasattr(self, 'id1'): # l'objet a déja été dessiné
            self.canvas.delete(self.id1)

    def draw(self):
        x0, y0 = self.r0.orig.x - self.r, self.r0.orig.y - self.r
        x1, y1 = self.r0.orig.x + self.r, self.r0.orig.y + self.r
        x0, y0 = self.canvas.r2c(x0, y0)
        x1, y1 = self.canvas.r2c(x1, y1)
        a0, a1 = self.r0.get_angle(), self.r1.get_angle()
        
        self.id = self.canvas.create_arc(x0, y0,
                                         x1, y1,
                                         style = tk.ARC,
                                         start = a0,
                                         extent = a1 - a0,
                                         outline = "blue",
                                         width = 2
                                         )
        if abs(a0-a1) > 1:
            a1 = np.radians(a1)
            xc, yc = self.r0.orig.x, self.r0.orig.y
            x1, y1 = xc + self.r*np.cos(a1), yc + self.r*np.sin(a1)
            x0, y0 = x1 + 3*np.sin(a1), y1 - 3*np.cos(a1)
            x0, y0 = self.canvas.r2c(x0, y0)
            x1, y1 = self.canvas.r2c(x1, y1)
            if a1 - a0 < 0:
                x0, y0, x1, y1 = x1, y1, x0, y0
            self.id1 = self.canvas.create_line (x0, y0,
                                                x1, y1,
                                                arrow = tk.LAST,
                                                fill = "blue"
                                                )


class Rectangle(Objet):
    def __init__(self, canvas, x0, y0, w, h, bg0, bg1):
        super().__init__(canvas)

        # On met l'objet dans la liste des objets du canvas
        self.canvas.objets.append(self)

        # Paramètres géométriques
        self.x0 = x0
        self.y0 = y0
        self.w = w
        self.h = h

        # Couleurs
        self.bg0 = bg0
        self.bg1 = bg1
        self.bg = self.bg0 # Couleur courante

        # Point cliqué
        self.dx = self.dy = None

        # On dessine l'objet
        self.draw()


    def est_dedans(self, x, y):
        """ Renvoie True si le point (x,y) est à l'intérieur de l'objet
        """
        return self.x0 < x < self.x0 + self.w and self.y0 < y < self.y0 + self.h


    def mousemove(self, event):
        """ Actions réalisées quand la souris est déplacée
        """
        x, y = event.x, event.y

        # Changement de couleur
        if self.est_dedans(x, y):
            self.bg = self.bg1
        else:
            self.bg = self.bg0

        # Drag
        if self.dx is not None: # drag en cours
            self.x0 = x - self.dx
            self.y0 = y - self.dy

        self.draw()


    def mousedown(self, event):
        """ Actions réalisées quand le bouton de la souris est appuyé
        """
        x, y = event.x, event.y

        # Début du drag : on mémorise la position relative du curseur de la souris
        if self.est_dedans(x, y):
            self.dx = event.x - self.x0
            self.dy = event.y - self.y0


    def mouseup(self, event):
        """ Actions réalisées quand le bouton de la souris est relâché
        """
        x, y = event.x, event.y
        # Fin du drag
        self.dx = self.dy = None


    def draw(self):
        if hasattr(self, 'id'): # l'objet a déja été dessiné
            self.canvas.delete(self.id)
        self.id = self.canvas.create_rectangle((self.x0, self.y0),
                                               (self.x0 + self.w, self.y0 + self.h),
                                               fill = self.bg)


class Sprite(Objet):
    def __init__(self, canvas, fname, pts, pos, ang = 0, ax = 0):
        """ canvas : tk.Canvas
            fname  : nom du fichier image
            pts    : (x,y) en pixel "image" des points caractéristiques du solide 
                         (le 1er est l'origine du repère "local")
            pos    : (x,y) en pixel repère "référentiel" de l'origine du repère "local"
            ang     : position angulaire en degrès du repère "local" par rapport au repère "référentiel"
            ax      : angle en degrés de l'axe x par rapport à l'image
        """

        super().__init__(canvas)
        self.nom = fname
        # On met l'objet dans la liste des objets du canvas
        #self.canvas.add_objet(self)

        # image
        self.img = Image.open(os.path.join(IMAGEPATH, fname))
        width, height = self.img.size
        self.img = self.img.resize((scale(width), scale(height)))
        # self.tkimage = ImageTk.PhotoImage(self.img)
        # self.id = self.canvas.create_image((0,0), image = self.tkimage)
        #print("taille :", width, height)

        # Paramètres géométriques (constantes)
        #  Origine (par rapport au centre image) en pixels "repère"
        self.cx = scale(width/2-pts[0][0])
        self.cy = scale(pts[0][1]-height/2)
        self.ax = ax
        #print("centre :", self.cx, self.cy)

        #  Autres points caractéristiques en pixels "repère"
        #    dans le repère local du solide
        #    avec angle en degrés et rayon
        self.pts = []
        for p in pts[1:]:
            x = p[0]-pts[0][0]
            y = pts[0][1]-p[1]
            a = np.degrees(np.arctan2(y, x)) + self.ax
            r = np.sqrt(x**2 + y**2)
            x = scale(x)
            y = scale(y)
            r = scale(r)
            self.pts.append((x, y, a, r))

        # Position et orientation initiales
        self.a = ang
        self.x, self.y = pos  # en pixels "repère"
        self.pos_cnv = self.canvas.r2c(self.x, self.y)
        self.modif_a = True
        self.modif_p = True

        # Point cliqué
        self.dx = self.dy = None # en pixels "canvas"

        # On dessine l'objet
        self.calcul()
        self.draw()

    def set_a(self, a):
        if a != self.a:
            self.a = a
            self.modif_a = True
        else:
            self.modif_a = False

    def get_centre(self):
        return self.cx, self.cy

    def get_pos(self):
        return self.x, self.y

    def get_point(self, num):
        """ Renvoie les coordonnées du point num
            dans le repère dessin
        """
        x, y = self.get_pos() 
        a = np.radians(self.a + self.pts[num][2])
        r = self.pts[num][3]
        px = x + r*np.cos(a)
        py = y + r*np.sin(a)
        return px, py

    # def rotation_centre(self, x, y, a):
    #     self.cx = x
    #     self.cy = y
    #     self.a = a
    #     self.draw()

    def est_dedans(self, x, y):
        """ Renvoie True si le point (x,y) est à l'intérieur de l'objet
        """
        return False


    def mousemove(self, event):
        """ Actions réalisées quand la souris est déplacée
        """
        x, y = event.x, event.y

        # Drag
        if self.dx is not None: # drag en cours
            self.x = x - self.dx
            self.y = y - self.dy


    def mousedown(self, event):
        """ Actions réalisées quand le bouton de la souris est appuyé
        """
        x, y = event.x, event.y

        # Début du drag : on mémorise la position relative du curseur de la souris
        if self.est_dedans(x, y):
            self.dx = event.x - self.x
            self.dy = event.y - self.y


    def mouseup(self, event):
        """ Actions réalisées quand le bouton de la souris est relâché
        """
        x, y = event.x, event.y
        # Fin du drag
        self.dx = self.dy = None


    def calcul(self):
        if True:#self.modif_a:
            # Rotation (autour du centre de l'image, sinon bug)
            img = self.img.rotate(self.a + self.ax, 
                                resample = Image.BICUBIC,
                                expand = True,
                                )
        else:
            img = self.img

        # Translation centre image --> centre rotation
        _x, _y = self.x, self.y
        x, y = self.get_pos()
        if self.modif_a or x != _x or y != _y:
            cx, cy = self.get_centre()
            self.modif_p = True
            t = np.radians(self.a + self.ax)
            c, s = np.cos(t), np.sin(t)
            r = np.array(((c, -s),
                        (s,  c) ))
            v = np.array((cx, cy))
            v = r.dot(v) - v  + np.array((x + cx, y + cy))
            v = v.tolist()
            #self.x, self.y = v
            self.pos_cnv = self.canvas.r2c(*v)
        else:
            self.modif_p = False

        self.tkimage = ImageTk.PhotoImage(img)
        #print("sprite r :", self.x + self.cx, self.y + self.cy)
        

    def delete(self):
        if hasattr(self, 'id'):# and self.modif_a: # l'objet a déja été dessiné
            self.canvas.delete(self.id)

    def draw(self):
        # if hasattr(self, 'id'): # l'objet a déja été dessiné
        #     self.canvas.moveto(self.id, *self.pos_cnv)
        if True:#self.modif_a or self.modif_p:
            self.id = self.canvas.create_image(
                self.pos_cnv, image = self.tkimage)
        
        

class Linked_Sprite(Sprite):
    def __init__(self, canvas, fname, sp, pt, pts, pos, ang = 0, ax = 0):#, dang = 0):
        self.sp = sp
        self.pt = pt
        super().__init__(canvas, fname, pts, pos, ang, ax)#, dang)
        
    def get_pos(self):
        self.x, self.y = self.sp.get_point(self.pt)
        return self.x, self.y



###################################################################################
# Classe définissant le canvas où sont dessinés les objets
###################################################################################
class Dessin(tk.Canvas):
    def __init__(self, fen, width, height, orig, bg):
        """ fen : Frame contenante
            width, height : dimensions du canvas en pixels
            orig : origine du repère en pixels "canvas"
        """

        super().__init__(fen, width = width, height=height, bg=bg)
        self.objets = []
        self.orig = orig # pixel "canvas"
        self.fen = fen
        fen.update_idletasks()

        # Liaisons des événements à des méthodes
        self.bind('<Motion>', self.mousemove)
        self.bind('<Button-1>', self.mousedown)
        self.bind('<ButtonRelease-1>', self.mouseup)

    def add_objet(self, o):
        self.objets.append(o)

    def add_objets(self, o):
        self.objets.extend(o)

    def c2r(self, x, y):
        """ Conversion pixels "canvas" --> pixels "repère"
        """
        return x-self.orig[0], self.winfo_reqheight()-y+self.orig[1]

    def r2c(self, x, y):
        """ Conversion pixels "repère" --> pixels "canvas"
        """
        return x+self.orig[0], self.winfo_reqheight()-y-self.orig[1]

    def draw(self):
        for o in self.objets:
            o.calcul()

        for o in self.objets:
            o.delete()
        
        for o in self.objets:
            if self.fen.draw_rep.get() or (not isinstance(o, Repere) and not isinstance(o, Angle)):
                o.draw()
        #self.after_idle(self.update)

    def mousemove(self, event):
        for o in self.objets:
            o.mousemove(event)

    def mousedown(self, event):
        for o in self.objets:
            o.mousedown(event)

    def mouseup(self, event):
        for o in self.objets:
            o.mouseup(event)





class Cmd(continuous_threading.PausableThread):
    def __init__(self, gui):
        super().__init__()
        self.gui = gui
        # File de commandes à envoyer
        self.f = File()
        self.after = None

    def _run(self): # Boucle principale
        if self.gui.ser is None:
            return
        
        # émission commandes
        if not self.f.is_empty():
            c = self.f.dequeue()
            #print(">>>", c)
            self.gui.ser.write(bytes("<"+c+">", 'ascii'))
            if c != "of":
                self.gui.S.reset() # redemarrage du timer d'extinction
            
            if self.after is not None:
                self.after()
                self.after = None    

        # reception données
        if self.gui.ser is not None and self.gui.ser.in_waiting:
            self.gui.receive(self.gui.ser.readline())

        

    def add(self, cmd, after = None):
        self.f.enqueue(cmd)
        self.after = after



###################################################################################
# Widgets de contrôle
###################################################################################
class SpinBoxLabel(tk.Frame):
    def __init__(self, parent, text = "", **kargs):
        super().__init__(parent)
        l = tk.Label(self,
                    text = text)
        l.pack( side = tk.LEFT, 
                fill = tk.Y,
                expand = False)
        self.sb = tk.Spinbox(self, **kargs)
        self.sb.pack(side = tk.RIGHT, 
               expand = True,
               pady = 2, 
               fill = tk.BOTH)




###################################################################################
# Classe définissant l'objet représentant la fenêtre principale de l'application
###################################################################################
class Application(tk.Tk):
    def __init__(self, w, h):
        super().__init__()
        self.title("EEZYbotARM contrôle")   # Le titre de la fenêtre
        self.w = w
        self.h = h
        self.minsize(self.w, self.h)      # taille de fenêtre
        self.geometry(f"{self.w}x{self.h}")
        self.update()

        # Une méthode séparée pour construire le contenu de la fenêtre
        self.createWidgets()

        # Connexion au robot
        self.connecter()

        # Timer d'extinction des moteurs
        self.S = ResettableTimer(self.interval.get(), self.switch_off)
        self.S.run()

        # Thread de commandes
        self.T = Cmd(self)
        self.T.start()

        self.protocol("WM_DELETE_WINDOW", self.on_closing)
    
    def on_closing(self):
        #print("on_closing")
        self.switch_off()
        #print("   *")
        self.deconnecter()
        #print("   *")
        self.destroy()
        #print("   *")

    def mng_connexion(self):
        if self.ser:
            self.deconnecter()
        else:
            self.connecter()

    def connecter(self):
        self.ser = create_serial()
        self.disable_widgets(self.ser is None)
        

    def deconnecter(self):
        if not self.ser is None:
            s = self.ser
            self.ser = None
            s.close()
            
        self.disable_widgets(True)


    def get_size(self):
        return self.winfo_width(), self.winfo_height()

    # Méthode de création des widgets
    def createWidgets(self):
        # Création des widgets
        self.dessin = Dessin(self, width = self.winfo_width(),
                                   height = self.winfo_height()-100,
                                   orig = (210, 140),
                                   bg = "ivory")
        self.bati = Sprite(self.dessin, 'bati.png', 
                          [(418, 112), (256, 38)], (0, 0), 0)
        self.bras = Sprite(self.dessin, 'bras.png', 
                          [(72, 522), (389, 44)], (0, 0), 0, -56.5)
        self.abras = Linked_Sprite(self.dessin, 'avbras.png', 
                                   self.bras, 0,
                                   [(144, 241), (444, 788), (28, 28)], (0, 0), 0, 61)
        self.rabras = Sprite(self.dessin, 'renvoi_avbras.png',
                             [(145, 241), (29, 28)], (0, 0), 0, 61)#, 180)
        self.babras1 = Linked_Sprite(self.dessin, 'biellette_avbras2.png', 
                                   self.rabras, 0,
                                   [(30, 508), (348, 30)], (0, 0), 0, -56.5)
        self.bpoignet1 = Linked_Sprite(self.dessin, 'biellette_bras.png', 
                                        self.bati, 0, #self.bati.pts[0]
                                        [(30, 508), (348, 30)], (0, 0), 0, -56.5)
        self.bascule = Linked_Sprite(self.dessin, 'lien.png', 
                                   self.bras, 0,
                                   [(193, 147), (32, 61), (332, 28)], (0, 0), 0, 0)#, 40)
        self.poignet = Linked_Sprite(self.dessin, 'poignet.png', 
                                   self.abras, 0,
                                   [(31, 150), (171,30)], (0, 0), 0, 0)
        self.bpoignet2 = Linked_Sprite(self.dessin, 'biellette_avbras.png', 
                                        self.bascule, 1,
                                        [(31, 31), (333, 577)], self.bati.pts[0][:2], 0, 61)
        
        self.r_ref = Repere(self.dessin, self.bati)
        self.r_bras = Repere(self.dessin, self.bras)
        self.r_abras = Repere(self.dessin, self.abras)
        self.r_refb = Repere(self.dessin, self.bati, self.abras)

        self.a_a = Angle(self.dessin, self.r_ref, self.r_bras)
        self.a_b = Angle(self.dessin, self.r_refb, self.r_abras)
        self.dessin.add_objets([self.abras,
                                self.bras,
                                self.babras1,
                                self.rabras, 
                                self.bascule,
                                self.poignet,
                                self.bpoignet2,
                                self.bati, 
                                self.bpoignet1,
                                self.r_ref,
                                self.r_bras,
                                self.r_abras,
                                self.r_refb,
                                self.a_a,
                                self.a_b
                                ])
        self.dessin.pack()
        

        r = 1
        # Coordonnées
        fc = tk.Frame(self)
        lf = tk.LabelFrame(fc, text = "Position", font='Helvetica 10 bold')
        lf.pack( side = tk.TOP, 
                #fill = tk.BOTH,
                padx = 2,
                expand = False,
                anchor = tk.W)

        l = tk.Label(lf, text = "consigne")
        l.pack( side = tk.TOP, 
                #fill = tk.BOTH,
                padx = 2,
                expand = False,
                anchor = tk.W)

        self.coord_x = tk.IntVar()
        self.coord_x_sb = SpinBoxLabel(lf, text = "x =",
                                        textvariable = self.coord_x, 
                                        command = self.move_to,
                                        from_ = 0, to = 300,
                                        increment = 10,
                                        width = 4)
        self.coord_x_sb.sb.bind("<Return>", self.move_to) 
        self.coord_x_sb.pack(side = tk.TOP, expand = True, fill = tk.BOTH)

        self.coord_y = tk.IntVar(value=0)
        self.coord_y_sb = SpinBoxLabel(lf, text = "y =",
                                        textvariable = self.coord_y, 
                                        from_ = 0, to = 300,
                                        command = self.move_to,
                                        increment = 10,
                                        width = 4 )
        self.coord_y_sb.sb.bind("<Return>", self.move_to) 
        self.coord_y_sb.pack(side = tk.BOTTOM, expand = True, fill = tk.BOTH)
        fc.pack(side = tk.LEFT, padx = 5)

        # Angles ################
        fa = tk.Frame(self)
        lf = tk.LabelFrame(fa, text = "Angles", font='Helvetica 10 bold')
        lf.pack( side = tk.TOP, 
                #fill = tk.BOTH,
                padx = 2,
                expand = False,
                anchor = tk.W)

        fc = tk.Frame(lf)
        l = tk.Label(fc, text = "consigne")
        l.pack( side = tk.TOP, 
                #fill = tk.BOTH,
                padx = 2,
                expand = False,
                anchor = tk.W)
        self.angle_a = tk.IntVar()
        self.angle_a_sb = SpinBoxLabel(fc, text = "a =",
                                        textvariable = self.angle_a, 
                                        command = self.rotate_a,
                                        from_ = 40, to = 136,
                                        increment = 10,
                                        width = 4)
        self.angle_a_sb.sb.bind("<Return>", self.rotate_a)
        self.angle_a_sb.pack(side = tk.TOP, expand = True, fill = tk.BOTH)

        self.angle_b = tk.IntVar()
        self.angle_b_sb = SpinBoxLabel(fc, text = "b =",
                                        textvariable = self.angle_b, 
                                        command = self.rotate_b,
                                        from_ = -65, to = 19,
                                        increment = 10,
                                        width = 4)
        self.angle_b_sb.sb.bind("<Return>", self.rotate_b)
        self.angle_b_sb.pack(side = tk.BOTTOM, expand = True, fill = tk.BOTH)
        
        fc.pack(side = tk.LEFT, padx = 5)

        # Angles mesurés
        fc = tk.Frame(lf)
        l = tk.Label(fc, text = "mesure")
        l.pack( side = tk.TOP, 
                #fill = tk.BOTH,
                padx = 2,
                expand = False,
                anchor = tk.W)
        self.angle_mes_a = tk.IntVar()
        self.angle_mes_a_sb = tk.Label(fc, text = "a =", state='disabled',
                                        textvariable = self.angle_mes_a
                                        )
        self.angle_mes_a_sb.pack(side = tk.TOP, expand = True, fill = tk.BOTH)

        self.angle_mes_b = tk.IntVar()
        self.angle_mes_b_sb = tk.Label(fc, text = "b =", state='disabled',
                                        textvariable = self.angle_mes_b
                                        )
        self.angle_mes_b_sb.pack(side = tk.BOTTOM, expand = True, fill = tk.BOTH)
        
        fc.pack(side = tk.LEFT, padx = 5)

        fa.pack(side = tk.LEFT, padx = 5)

        # Commandes
        cm = tk.Frame(self)
        l = tk.Label(cm, text = "Commande :", font='Helvetica 10 bold')
        l.pack( side = tk.TOP, 
                #fill = tk.BOTH,
                padx = 2,
                expand = False,
                anchor = tk.W)
        self.commande = tk.StringVar()
        self.entree = tk.Entry(cm, textvariable = self.commande)
        self.entree.bind("<Return>", self.send_command) 
        self.entree.pack(side = tk.TOP, 
                         padx = 2, 
                         fill = tk.X,
                         expand = True, 
                         anchor = tk.W)

        cm.pack(side = tk.LEFT, padx = 2, fill = tk.X, expand = False, anchor = tk.N)

        # Zone de droite
        zd = tk.Frame(self)

        # Barre de boutons
        fb = tk.Frame(zd)
        # Un bouton pour centrer
        self.centerButton = tk.Button(fb, text = "Centrer",
                                      command = self.centrer)
        self.centerButton.pack(side = tk.LEFT)

        # Un bouton pour éteindre les moteurs
        self.offButton = tk.Button(fb, text = "Éteindre",
                                   command = self.switch_off)
        self.offButton.pack(side = tk.LEFT)

        # Un bouton pour connecter/déconnecter
        self.msg_conn = tk.StringVar()
        self.connectButton = tk.Button(fb, text = "", textvariable = self.msg_conn,
                                       command = self.mng_connexion)
        self.connectButton.pack(side = tk.LEFT)

        fb.pack(side = tk.TOP)


        # Réglages
        fr = tk.Frame(zd)
        self.interval = tk.IntVar(value=DELAY_DEFAULT)
        self.interval_sb = SpinBoxLabel(fr, text = "Délai extinction (s) :",
                                        textvariable = self.interval, 
                                        from_ = 0, to = 200,
                                        command = self.update_timer,
                                        increment = 1,
                                        width = 4 )
        self.interval_sb.sb.bind("<Return>", self.update_timer) 
        self.interval_sb.pack(side = tk.BOTTOM, expand = True, fill = tk.BOTH)

        self.boucle_fermee = tk.BooleanVar(value=False)
        self.boucle_fermee_cb = tk.Checkbutton(fr, text = "Boucle fermée", 
                                        variable = self.boucle_fermee,
                                        command=self.setBF)
        self.boucle_fermee_cb.pack(side = tk.BOTTOM, expand = False, fill = None, anchor=tk.W)

        self.draw_rep = tk.BooleanVar(value=True)
        self.draw_rep_cb = tk.Checkbutton(fr, text = "Dessiner repères", 
                                        variable = self.draw_rep,
                                        command = self.dessin.draw)
        self.draw_rep_cb.pack(side = tk.BOTTOM, expand = False, fill = None, anchor=tk.W)

        fr.pack(side = tk.TOP)

        zd.pack()

        self.dessin.draw()

    def update_timer(self):
        self.S.set_interval(self.interval.get())
        self.S.reset()

    def disable_widgets(self, dis):
        if dis:
            st = "disabled"
            self.msg_conn.set("Connexion")
        else:
            st = "normal"
            self.msg_conn.set("Déconnexion")
        
        self.centerButton.config(state=st)
        self.offButton.config(state=st)
        self.entree.config(state=st)
        self.entree.config(state=st)
        self.coord_x_sb.sb.config(state=st)
        self.coord_y_sb.sb.config(state=st)
        self.angle_a_sb.sb.config(state=st)
        self.angle_b_sb.sb.config(state=st)

    def setBF(self):
        if self.boucle_fermee.get():
            self.angle_mes_a_sb.configure(state = 'normal')
            self.angle_mes_b_sb.configure(state = 'normal')
        else:
            self.angle_mes_a_sb.configure(state = 'disabled')
            self.angle_mes_b_sb.configure(state = 'disabled')

    def centrer(self, val=0):
        self.send('cc')

    def switch_off(self, val=0):
        self.S.stop()
        self.send("of", after = self.T.stop)
        #self.T.stop()
        #print("OFF")
        
    def switch_on(self, val=0):
        #print("ON")
        self.centrer()

    def send_command(self, val):
        self.send(self.commande.get())
        self.entree.delete(0, 'end')

    def move_to(self, val = None):
        x = str(self.coord_x.get())
        y = str(self.coord_y.get())
        self.send("xy " + x + " " + y)

    def rotate_a(self, val=None):
        a = str(self.angle_a.get())
        self.send("ad " + a)

    def rotate_b(self, val=None):
        b = str(self.angle_b.get())
        self.send("bd " + b)

    def receive(self, cmd):
        """ Réception données
        """
        #print(cmd)
        
        try:
            cmd = bytes.decode(cmd, 'utf-8')
            typ, val = cmd.split(" ", 1)
        except:
            return

        if typ == "_aa":
            try:
                a, b = val.split(",")
            except:
                return
            a, b = int(a), int(b)
            self.angle_a.set(a)
            self.angle_b.set(b)

            if not self.boucle_fermee.get():
                self.bras.set_a(a)
                self.abras.set_a(b)
                self.rabras.set_a(b)
                self.babras1.set_a(a)
                self.bpoignet1.set_a(a)
                self.bpoignet2.set_a(b)
                self.dessin.draw()

        # Angles mesurés
        #  format : "_am a b"
        #  avec a et b = angles mesurés en degrés
        elif typ == "_am":
            try:
                a, b = val.split(",")
            except:
                return
            
            if a == 'N' or not self.boucle_fermee.get():
                a = self.angle_a.get()
            else:
                a = round(float(a))

            if b == 'N' or not self.boucle_fermee.get():
                b = self.angle_b.get()
            else:
                b = round(float(b))

            if self.boucle_fermee.get():
                self.angle_mes_a.set(a)
                self.angle_mes_b.set(b)
            
            # On redessine ...
            self.bras.set_a(a)
            self.abras.set_a(b)
            self.rabras.set_a(b)
            self.babras1.set_a(a)
            self.bpoignet1.set_a(a)
            self.bpoignet2.set_a(b)
            self.dessin.draw()

        elif typ == "_xy":
            try:
                x, y = val.split(",")
            except:
                return
            x, y = int(x), int(y)
            self.coord_x.set(x)
            self.coord_y.set(y)

    def send(self, cmd, after = None):
        self.T.start()
        self.T.add(cmd, after = after)


###################################################################################
app = Application(590, 620)
try:
    app.mainloop()
    time.sleep(2)
    app.switch_on()
finally:
    #print("fin")
    app.switch_off()
    #print("  +")
    if app.ser is not None:
        #print("  +")
        app.ser.close()
    #print("fini")
    sys.exit()
