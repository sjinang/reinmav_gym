import xml.etree.ElementTree as ET
import numpy as np
import matplotlib.pyplot as plt
import os,sys, time

cd_path = os.path.dirname(os.path.abspath(__file__))

def run():
    
    tree = ET.parse(cd_path+'/assets/quadrotor_env.xml')
    root = tree.getroot()

    worldbody = root.find('worldbody')

    W=5*2
    D=5*2
    NW = 5
    ND = 5
    
    obsts = [((0)+int(W*i/NW)-W/2+np.random.uniform()*W/NW,(0)+int(D*j/ND)-D/2+np.random.uniform()*D/ND) for i in range(NW) for j in range(ND)]
    obsts = [(obst[0],obst[1]) for obst in obsts if abs(obst[0])>=1 or abs(obst[1])>=1]

    for obst in obsts:
        new = ET.SubElement(worldbody, 'geom')
        new.set('type','box')
        new.set('pos',str(obst[0])+' '+str(obst[1])+' 1.25')
        new.set('size','0.2 0.2 1.25')
        new.set('rgba','0.55 0.3 0.3 1')

    tree.write(cd_path+'/assets/quadrotor_env_modified.xml') 