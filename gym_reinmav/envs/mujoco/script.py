import xml.etree.ElementTree as ET
import numpy as np
import matplotlib.pyplot as plt
import os,sys, time

cwd_path = '/home/sjinang/reinmav-gym/gym_reinmav/envs/mujoco'+'/'

def run():
    
    tree = ET.parse(cwd_path+'assets/quadrotor_hovering_copy.xml')
    root = tree.getroot()

    worldbody = root.find('worldbody')

    W=20
    D=20
    NW = 15
    ND = 15

    for floor in worldbody:
        # floor.attrib['size']=str(W/2)+' '+str(D/2)+' .2'
        floor.attrib['size']='100 100 .2'
    #    floor.attrib['pos']=str(W/2)+' '+str(D/2)+' 0'
        break

    obsts = [((0)+int(W*i/NW)-W/2+np.random.uniform()*W/NW,(0)+int(D*j/ND)-D/2+np.random.uniform()*D/ND) for i in range(NW) for j in range(ND)]
    obsts = [(obst[0],obst[1]) for obst in obsts if abs(obst[0])>=1 or abs(obst[1])>=1]

    for obst in obsts:
        new = ET.SubElement(worldbody, 'geom')
        new.set('type','box')
        new.set('pos',str(obst[0])+' '+str(obst[1])+' 0')
        new.set('size','0.2 0.2 2.5')
        new.set('rgba','0.7 0.3 0.3 1')

    tree.write(cwd_path+'assets/quadrotor_hovering_test.xml') 

