"""
Soft body simulation. Mesh.

@author: 
    Juan Jose Potes Gomez
    Julie Alejandra Ibarra
    Cristian Camilo Jimenez
"""
import pygame
from pygame.locals import *

from OpenGL.GL import *
from OpenGL.GLU import *

import numpy as np
import matplotlib.pyplot as plt
import copy

# Funcion para pasar angulo de radianes a grados y viceversa
def conv_ang(angulo, tipo):
    if tipo == "rad":
        conv = (angulo * np.pi)/180
        return conv
    if tipo == "grad":
        conv = (angulo * 180)/np.pi
        return conv

# Funcion que grafica un circulo con OpenGL
def circle(xc,yc,radio,clr):
    n = 0
    glBegin(GL_POLYGON)
    glColor3f(clr[0], clr[1], clr[2])
    nsides = 20
    while(n <= nsides):
        angle = 2*np.pi*n/nsides
        x = xc+radio*np.cos(angle)
        y = yc+radio*np.sin(angle)
        glVertex2f(x,y)
        n += 1
    glEnd()

# Funcion que grafica una linea con openGL
def linea(x1,y1,x2,y2,r,g,b):
    glBegin(GL_LINES)
    glColor3f(r,g,b)
    glVertex2f(x1,y1)
    glVertex2f(x2,y2)
    glEnd()

# Funcion para graficar el plano 2d de referencia
def plano():
    linea(0,-ejeY,0,ejeY,0.8,0.8,0.8)
    linea(-ejeX,0,ejeX,0,0.8,0.8,0.8)

# Funcion que grafica las lineas entre los puntos de la malla
def lineas_malla():
    global malla
    for i in range(0, len(malla)):
        for j in range(0, len(malla[0])):
            x0 = malla[i][j].pos[0]
            y0 = malla[i][j].pos[1]
            
            if(i < (len(malla)-1)):
                x1 = malla[i+1][j].pos[0]
                y1 = malla[i+1][j].pos[1]
                linea(x0,y0,x1,y1,1,1,1)
            
            if(j < (len(malla[0])-1)):
                x2 = malla[i][j+1].pos[0]
                y2 = malla[i][j+1].pos[1]
                linea(x0,y0,x2,y2,1,1,1)


# Funcion que retorna el valor de la fuerza externa segun la opcion escogida por el usuario y el tiempo
def fuerza_ext(tp, eje):
    # F = -8d(t)
    if tp < 0.05:
        if(eje == "x"): 
            return fuer_mag * np.cos(fuer_dir)
        elif(eje == "y"):
            return fuer_mag * np.sin(fuer_dir)
    else:
        return 0

# Funcion que retorna la distancia al punto de equilibrio (Coordenadas originales)
def dist_eq(i,j,eje):
    global malla_orig, malla
    if(eje == "y"):
        d = malla[i][j].pos[1] - malla_orig[i][j].pos[1]
        return d
    if(eje == "x"):
        d = malla[i][j].pos[0] - malla_orig[i][j].pos[0]
        return d

# Funcion que retorna el valor de la aceleracion para Y
def f_ay(i, j, tiempo, aplicfuerza):
    global malla
    # Se evalua si se aplica la fuerza externa
    if(aplicfuerza == True):
        f_externa = fuerza_ext(tiempo, "y")
    else:
        f_externa = 0
    
    # Componentes de fuerzas en Y de las particulas de arriba y abajo
    if((i-1) < 0):
        # Si el punto esta en el borde superior
        arriba = 0
        abajo = (k*(dist_eq(i+1,j,"y") - dist_eq(i,j,"y"))) + (c*(malla[i+1][j].v[1] - malla[i][j].v[1]))
    elif((i+1) >= len(malla)):
        # Si el punto esta en el borde inferior
        arriba = -(k*(dist_eq(i,j,"y") - dist_eq(i-1,j,"y"))) - (c*(malla[i][j].v[1] - malla[i-1][j].v[1]))
        abajo = 0
    else:
        # Si el punto no esta en el borde superior ni inferior
        arriba = -(k*(dist_eq(i,j,"y") - dist_eq(i-1,j,"y"))) - (c*(malla[i][j].v[1] - malla[i-1][j].v[1]))
        abajo = (k*(dist_eq(i+1,j,"y") - dist_eq(i,j,"y"))) + (c*(malla[i+1][j].v[1] - malla[i][j].v[1]))
    
    # Componentes de fuerzas en Y de las particulas de la izquierda y la derecha
    if((j-1) < 0):
        # Si el punto esta en el borde izquierdo
        izquierda = 0
        derecha = (k*(dist_eq(i,j+1,"y") - dist_eq(i,j,"y"))) + (c*(malla[i][j+1].v[1] - malla[i][j].v[1]))
    elif((j+1) >= len(malla[0])):
        # Si el punto esta en el borde derecho
        izquierda = -(k*(dist_eq(i,j,"y") - dist_eq(i,j-1,"y"))) - (c*(malla[i][j].v[1] - malla[i][j-1].v[1]))
        derecha = 0
    else:
        # Si el punto no esta en el borde izquierdo y derecho
        izquierda = - (k*(dist_eq(i,j,"y") - dist_eq(i,j-1,"y"))) - (c*(malla[i][j].v[1] - malla[i][j-1].v[1]))
        derecha = (k*(dist_eq(i,j+1,"y") - dist_eq(i,j,"y"))) + (c*(malla[i][j+1].v[1] - malla[i][j].v[1]))
    
    # Se suman todas las fuerzas en Y
    resul = (arriba + abajo + derecha + izquierda + f_externa)/m
    return resul

# Funcion que retorna el valor de la aceleracion para X
def f_ax(i, j, tiempo, aplicfuerza):
    global malla
    # Se evalua si se aplica la fuerza externa
    if(aplicfuerza == True):
        f_externa = fuerza_ext(tiempo, "x")
    else:
        f_externa = 0
    
    # Componentes de fuerzas en X de las particulas de la izquierda y la derecha
    if((j-1) < 0):
        # Si el punto esta en el borde izquierdo
        izquierda = 0
        derecha = (k*(dist_eq(i,j+1,"x") - dist_eq(i,j,"x"))) + (c*(malla[i][j+1].v[0] - malla[i][j].v[0]))
    elif((j+1) >= len(malla[0])):
        # Si el punto esta en el borde derecho
        izquierda = -(k*(dist_eq(i,j,"x") - dist_eq(i,j-1,"x"))) - (c*(malla[i][j].v[0] - malla[i][j-1].v[0]))
        derecha = 0
    else:
        # Si el punto no esta en el borde izquierdo y derecho
        izquierda = - (k*(dist_eq(i,j,"x") - dist_eq(i,j-1,"x"))) - (c*(malla[i][j].v[0] - malla[i][j-1].v[0]))
        derecha = (k*(dist_eq(i,j+1,"x") - dist_eq(i,j,"x"))) + (c*(malla[i][j+1].v[0] - malla[i][j].v[0]))
    
    # Componentes de fuerzas en X de las particulas de arriba y abajo
    if((i-1) < 0):
        # Si el punto esta en el borde superior
        arriba = 0
        abajo = (k*(dist_eq(i+1,j,"x") - dist_eq(i,j,"x"))) + (c*(malla[i+1][j].v[0] - malla[i][j].v[0]))
    elif((i+1) >= len(malla)):
        # Si el punto esta en el borde inferior
        arriba = -(k*(dist_eq(i,j,"x") - dist_eq(i-1,j,"x"))) - (c*(malla[i][j].v[0] - malla[i-1][j].v[0]))
        abajo = 0
    else:
        # Si el punto no esta en el borde superior ni inferior
        arriba = -(k*(dist_eq(i,j,"x") - dist_eq(i-1,j,"x"))) - (c*(malla[i][j].v[0] - malla[i-1][j].v[0]))
        abajo = (k*(dist_eq(i+1,j,"x") - dist_eq(i,j,"x"))) + (c*(malla[i+1][j].v[0] - malla[i][j].v[0]))
    
    # Se suman todas las fuerzas en X
    resul = (izquierda + derecha + arriba + abajo + f_externa)/m
    return resul
        
# Definicion de clase particula
class Punto:
    
    # Constructor de la clase
    def __init__(self, pos, rad, masa, ang_vel, vel, acel, color, fuerza, fijo, ubic):
        self.pos = pos
        self.rad = rad
        self.masa = masa
        self.ang = ang_vel
        self.vel = vel
        self.v = np.array([[self.vel*np.cos(self.ang)],[self.vel*np.sin(self.ang)]])
        self.acel = acel
        self.color = color
        self.fuerza = fuerza
        self.fijo = fijo
        self.ubic = ubic
    
    # Set para indicar si el punto esta fijo o no
    def set_fijo(self,fijo):
        self.fijo = fijo
    
    # Set para indicar si al punto le afecta la fuerza externa
    def set_aplifuerza(self,fuerza):
        self.fuerza = fuerza
    
    # Set para cambiar el color del punto
    def set_color(self,color):
        self.color = color
    
    # Metodo para mostrar la particula en la posicion respectiva
    def graficar(self):
        circle(self.pos[0],self.pos[1],self.rad,self.color)
        p_dx = self.pos[0] + self.v[0]
        p_dy = self.pos[1] + self.v[1]
        linea(self.pos[0],self.pos[1],p_dx,p_dy,self.color[0],self.color[1],self.color[2])
        
        
    # Metodo que hace los calculos para determinar la nueva posicion con respecto a un intervalo de tiempo
    def avanzar(self,ht):
        global t
        
        if(self.fijo == False):
            # Datos en Y
            ay = self.acel[1]
            v_y = self.v[1]
            y = self.pos[1]
            
            self.acel[1] = f_ay(self.ubic[0],self.ubic[1],t,self.fuerza)
            self.pos[1] = ay * (ht * ht) + (v_y * ht) + y
            self.v[1] = ay * ht + v_y
            
            
            # Datos en X
            ax = self.acel[0]
            v_x = self.v[0]
            x = self.pos[0]
            
            self.acel[0] = f_ax(self.ubic[0],self.ubic[1],t,self.fuerza)
            self.pos[0] = ax * (ht * ht) + (v_x * ht) + x
            self.v[0] = ax * ht + v_x
    
    
            self.vel = np.sqrt((self.v[0]**2)+(self.v[1]**2))
            self.ang = np.arctan(self.v[1]/self.v[0])

ejeX = 8
ejeY = 8

# Variables para la simulacion
k = 0.5
m = 1 # gramo
c = 0.02
r = 0.1

#Variables del tiempo
ht = 0.01 
t = 0
t_max = 100

# Variables para la fuerza externa
fuer_mag = -8
fuer_dir = conv_ang(150,"rad")

#Variables para la creación de la malla
x_i = 1.0
y_i = -1.0

tam_i = 6
tam_j = 5

# Se declara la matriz de puntos
malla = np.empty((tam_i,tam_j), dtype=object)
for i in range(0, len(malla)):
    if(y_i < -tam_i):
        y_i = -1.0
    for j in range(0, len(malla[0])):
        if(x_i > tam_j):
            x_i = 1.0
        p_base = Punto([x_i,y_i], r, m, 0, 0, [0,0],[0,1,1], False, False, [i,j])
        malla[i][j] = copy.deepcopy(p_base) 
        x_i += 1.0
    y_i -= 1.0

# Se definen los puntos fijos
malla[0][0].set_fijo(True)
malla[0][0].set_color([1,0,0])
malla[0][1].set_fijo(True)
malla[0][1].set_color([1,0,0])
malla[1][0].set_fijo(True)
malla[1][0].set_color([1,0,0])
malla[5][0].set_fijo(True)
malla[5][0].set_color([1,0,0])

# Se define el punto al que se le aplica la fuerza
malla[2][4].set_aplifuerza(True)
malla[2][4].set_color([1,1,0])

malla_aux = copy.deepcopy(malla)
malla_orig = copy.deepcopy(malla)

# Funcion principal
def main():
    global t, malla, malla_aux
    running = True
    paused = False
    pygame.init()
    display=(600,600)
    pygame.display.set_mode(display, DOUBLEBUF|OPENGL)
    gluOrtho2D(-1,ejeX,-ejeY,1)
    
    # Ciclo para que se vaya visualizando la simulacion
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                running=False
                break
            if event.type == pygame.KEYDOWN:
                if event.key == K_SPACE:
                    if paused == True:
                        paused = False
                    elif paused == False:
                        paused = True
        # Se ejecuta mientras que el tiempo total sea menor al tiempo maximo definido
        if( running == True and t <= t_max and paused == False):
            # Se limpia la pantalla de OpenGL
            glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
            
            # Se avanza cada punto de la malla con una malla auxiliar
            for i in range(0, len(malla)):
                for j in range(0, len(malla[0])):
                    malla_aux[i][j].avanzar(ht)
                  
            # Se pasan los valores movidos de la malla auxiliar a la original
            malla = copy.deepcopy(malla_aux)
            
            # Se grafican los puntos de la malla
            for i in range(0, len(malla)):
                for j in range(0, len(malla[0])):
                    malla[i][j].graficar()
            
            # Se grafican las lineas de la malla
            lineas_malla()
            
            pygame.display.flip() # Mostrar pantalla
            pygame.time.wait(1)
            print("Tiempo procesado: ",t," s")
            t+=ht
        elif(t > t_max and paused == False and running == True):
            running = False
            break

    print("Tiempo total simulado = ",t)
    pygame.quit()

main()