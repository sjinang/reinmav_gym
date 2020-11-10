import xml.etree.ElementTree as ET
import numpy as np
import matplotlib.pyplot as plt
import os,sys, time

cd_path = os.path.dirname(os.path.abspath(__file__))

def run(v,goal,threshold):
    
    tree = ET.parse(cd_path+'/assets/quadrotor_env.xml')
    root = tree.getroot()

    worldbody = root.find('worldbody')

    W=20
    D=20
    NW = 4
    ND = 4

    for floor in worldbody:
        # floor.attrib['size']=str(W/2)+' '+str(D/2)+' .2'
        floor.attrib['size']='10 10 .2'
    #    floor.attrib['pos']=str(W/2)+' '+str(D/2)+' 0'
        break

    if v==1:
        obsts = [((0)+int(W*i/NW)-W/2+np.random.uniform()*W/NW,(0)+int(D*j/ND)-D/2+np.random.uniform()*D/ND) for i in range(NW) for j in range(ND)]
        obsts = [(obst[0],obst[1]) for obst in obsts if abs(obst[0])>=1 or abs(obst[1])>=1]

        for obst in obsts:
            new = ET.SubElement(worldbody, 'geom')
            new.set('type','box')
            new.set('pos',str(obst[0])+' '+str(obst[1])+' 1.25')
            new.set('size','0.2 0.2 1.25')
            new.set('rgba','0.55 0.3 0.3 1')

    new = ET.SubElement(worldbody, 'site')
    new.set('type','cylinder')
    new.set('pos',str(goal[0])+' '+str(goal[1])+' 1.25')
    new.set('size',str(threshold)+' 1.25')
    new.set('rgba','1.0 0. 0. 0.4')

    tree.write(cd_path+'/assets/quadrotor_env_modified.xml') 