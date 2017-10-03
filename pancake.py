#!/usr/bin/env python

import time
import random
import sys
import os
# from np import *
import numpy as np
import vtk

## Show graphically the Towers of Hanoi problem

Lmax = 1.0                        # max radius of disc
Lmin = 0.2                        # min radius of disc
Npnck = 10                        # height of the total stack of discs
Hpnck = 1.0                       # Height of the peg
Hmove = 2*Hpnck                   # height of the parabole that makes
                                        # a disc during the move
tsleep = 0.006                       # sleep time while performing the
                                        # visualization (but not video)
framerate = 20                          # Nbr of frames during a move
mkvideo = 1                             # make video or not
render = 1                              # render or not (for debugging)

## Height a single disc
Hpnck1 = Hpnck/Npnck
# Create the Renderer, RenderWindow, and RenderWindowInteractor
ren = vtk.vtkRenderer()
renWin = vtk.vtkRenderWindow()
renWin.AddRenderer(ren)
renWin.SetSize(640, 480)

## This is an alternative to set_rand_color()
## First we generate a list of colors by taking
## first RGB and then interpolations between them , and so on
colors = np.zeros((3,3))
colors[0] = np.array([1,0,0])
colors[1] = np.array([0,1,0])
colors[2] = np.array([0,0,1])
while True:
    n = colors.shape[0]
    if n>=Npnck:
        break
    colors = np.resize(colors,[2*n,3])
    for k in range(n):
        k1 = (k+1)%n
        colors[n+k] = 0.5*(colors[k]+colors[k1])

## After, the `set_color()' for `j' takes the `j'-th
## color from the generated list
def set_color(actor,j):
    prop = actor.GetProperty()
    prop.SetColor(colors[j])

## The disc contains its index (in [0,ndisc)) and
## its position (peg and position)
class pnck:
    def __init__(self,j,pos):
        self.j = j
        ## self.pos = pos
        ## create te actor
        cube = vtk.vtkCubeSource()
        L = Lmin+(Lmax-Lmin)/(Npnck-1)*j
        cube.SetXLength(L)
        cube.SetZLength(L)
        cube.SetYLength(Hpnck1)

        cubeMapper = vtk.vtkPolyDataMapper()
        cubeMapper.SetInputConnection(cube.GetOutputPort() )
        cubeMapper.ScalarVisibilityOff()

        cube_actor = vtk.vtkActor()
        cube_actor.SetMapper(cubeMapper)
        self.actor = cube_actor
        ## set_rand_color(cube_actor)
        set_color(cube_actor,j)
        ren.AddActor(cube_actor)

## The discs are stored in a list of 3 lists
pancakes = []
## Initially all discs are in peg 0
for k in range(Npnck):
    pancakes.append(pnck(k,k))
random.shuffle(pancakes)

for k in range(Npnck):
    pancakes[k].actor.AddPosition(0,k*Hpnck1,0)

def pnck_print():
    print "pncks = ",
    for k in range(Npnck):
        print " %d" % pancakes[k].j,
    print

frame = 0
def myrender():
    global frame
    if render:
        renWin.Render()
        if mkvideo:
            w2i.Modified()
            ## generate the fram filename
            assert os.path.isdir("./YUV")
            tiff = "./YUV/frame.%d.tiff" % frame
            yuv = "./YUV/frame.%d.yuv" % frame
            writer.SetFileName(tiff)
            writer.Write()
            ## Convert to YUV and gzip
            os.system("convert %s %s ; gzip -f %s" % (tiff,yuv,yuv))
            os.unlink(tiff)
            frame += 1
        else:
            time.sleep(tsleep)

## color the background
ren.SetBackground(0.7,0.7,0.7)

## prepare the camera
cam = ren.GetActiveCamera()
cam.SetViewUp(0.,1.,0.);
xfrom = np.array([-1,0.5,3])
xfrom *= 1.
cam.SetPosition(xfrom)
cam.SetFocalPoint(0,0.5*Hpnck,0)

## This is used to store the frames
## for creating a movie
w2i = vtk.vtkWindowToImageFilter()
w2i.SetInput(renWin)
w2i.Update()

## The TIFF writer
writer = vtk.vtkTIFFWriter()
writer.SetInputConnection(w2i.GetOutputPort())
writer.SetCompressionToJPEG()

if render:
    renWin.Render()

def invert(npnck):
    theta = 180.0/framerate
    for k in range(framerate):
        yc1 = (Npnck-npnck+0.5)*Hpnck1
        yc2 = (Npnck-0.5)*Hpnck1
        yc = 0.5*(yc1+yc2)
        for p in range(npnck):
            q = Npnck-1-p
            yq = (q+0.5)*Hpnck1
            pnck = pancakes[q].actor
            pnck.SetOrigin(0,yc-yq,0)
            pnck.RotateZ(theta)
        myrender()
    for p in range(npnck/2):
        q1 = Npnck-npnck+p
        q2 = Npnck-1-p
        ## print "exchange %d %d" % (q1,q2)
        tmp = pancakes[q1]
        pancakes[q1] = pancakes[q2]
        pancakes[q2] = tmp
    for p in range(Npnck):
        pnck = pancakes[p].actor
        pnck.SetOrigin(0,0,0)
        pnck.SetOrientation(0,0,0)
        pnck.SetPosition(0,p*Hpnck1,0)
    for k in range(framerate/3):
        myrender()

pnck_print()
for k in range(0,Npnck-1):
    ## look for the larger in range [k,Npnck)
    jmax = k
    for j in range(k+1,Npnck):
        if pancakes[j].j>pancakes[jmax].j:
            jmax = j
    invert(Npnck-jmax)
    pnck_print()
    invert(Npnck-k)
    pnck_print()
