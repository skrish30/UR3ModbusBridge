#!usr/bin/env python
import numpy as np

def distance_between_plane(w,n):
    v = w-pa
    print(n)
    a = project(v,n)
    return np.linalg.norm(a)

def get_normal(pa,pb,pc):
    v1 = pb-pa
    v2 = pc-pa
    return np.cross(v1,v2)

def normalize(v):
    vhat = v / np.linalg.norm(v)
    return vhat

#Project Vector A onto Vector parallel to Vector B 
def project(v, n):
    nhat = normalize(n)
    return np.dot(v,n)/ np.linalg.norm(n) * nhat

def angle_between(v1,v2):
    # print(v1)
    # print(v2)
    cos_theta = np.dot(v1,v2)/ (np.linalg.norm(v1) * np.linalg.norm(v2))
    #Determine if it is theta is +/-
    sign = np.sign(np.cross(v2,v1)[2])
    return sign * (np.arccos(cos_theta) * 180/np.pi)
