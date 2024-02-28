import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def rodrigues_rot(P, n0, n1):
    
    # If P is only 1d array (coords of single point), fix it to be matrix
    if P.ndim == 1:
        P = P[np.newaxis,:]
    
    # Get vector of rotation k and angle theta
    n0 = n0/np.linalg.norm(n0)
    n1 = n1/np.linalg.norm(n1)
    k = np.cross(n0,n1)
    k = k/np.linalg.norm(k)
    theta = np.arccos(np.dot(n0,n1))
    
    # Compute rotated points
    P_rot = np.zeros((len(P),3))
    for i in range(len(P)):
        P_rot[i] = P[i]*np.cos(theta) + np.cross(k,P[i])*np.sin(theta) + k*np.dot(k,P[i])*(1-np.cos(theta))

    return P_rot
def fit_circle_2d(x, y, w=[]):
    
    A = np.array([x, y, np.ones(len(x))]).T
    b = x**2 + y**2
    
    # Modify A,b for weighted least squares
    if len(w) == len(x):
        W = np.diag(w)
        A = np.dot(W,A)
        b = np.dot(W,b)
    
    # Solve by method of least squares
    c, residual, N, s = np.linalg.lstsq(A,b,rcond=None)
    
    # Get circle parameters from solution c
    xc = c[0]/2
    yc = c[1]/2
    r = np.sqrt(c[2] + xc**2 + yc**2)
    return xc, yc, r, residual

number_of_measurements=424 #hier Anzahl der Messungen eingeben
i=0
points_list=[]
while i<number_of_measurements: #liest die Posen aus den .frame-Files, die von ICP und GraphSLAM generiert wurden aus, nimmt die letzte Zeile und wählt dann die Wert aus, die die Translation angeben
    if i>99:
        name="scan"+str(i)+".frames"
    elif i<10:
        name="scan00"+str(i)+".frames"
    else:
        name="scan0"+str(i)+".frames"
    with open("%s" % name) as file:
        for line in file:
            pass
        last_line=line
    line_list=[]
    for number in last_line.split():
        num=float(number)
        line_list.append(num)
    points_list.append(line_list)
    i=i+1
#print (points_list[0])


pointsum=0
i=0
pointarray=np.array([points_list[i][12],points_list[i][13],points_list[i][14]])
current_point=np.array([])
i=1
while i<number_of_measurements:
    current_point=np.array([points_list[i][12],points_list[i][13],points_list[i][14]])
    i=i+1
    pointarray=np.vstack((pointarray,current_point))
#print(pointarray)

P_mean=pointarray.mean(axis=0)
#print(P_mean)
P_centered=pointarray-P_mean
U,s,V=np.linalg.svd(P_centered)
print(V)
normal=V[2,:]
d=-np.dot(P_mean,normal)
P_xy=rodrigues_rot(P_centered, normal, [0,0,1])
xc,yc,r,residual=fit_circle_2d(P_xy[:,0], P_xy[:,1])
print("Fitted circle has radius " + str(r) + " cm") #da wird der berechnete Radius ausgegeben
print("Residual: " + str(np.sqrt(residual) / number_of_measurements))


def generate_circle_by_vectors(t, C, r, n, u):
    n = n/np.linalg.norm(n)
    u = u/np.linalg.norm(u)
    P_circle = r*np.cos(t)[:,np.newaxis]*u + r*np.sin(t)[:,np.newaxis]*np.cross(n,u) + C
    return P_circle

def angle_between(u, v, n=None):
    if n is None:
        return np.arctan2(np.linalg.norm(np.cross(u,v)), np.dot(u,v))
    else:
        return np.arctan2(np.dot(n,np.cross(u,v)), np.dot(u,v))


C = rodrigues_rot(np.array([xc,yc,0]), [0,0,1], normal) + P_mean
t = np.linspace(0, 2*np.pi, 100)
u = pointarray[0] - C
P_fitcircle = generate_circle_by_vectors(t, C, r, normal, u)


fig = plt.figure(figsize=(15,15))
ax = fig.add_subplot(1,1,1,projection='3d')
ax.plot(*pointarray.T, ls='', marker='o', alpha=0.5, label='Cluster points P')
i=0
#while i < number_of_measurements:
 #   ax.scatter(pointarray[i][0],pointarray[i][1],pointarray[i][2])
  #  i=i+1
 #--- Plot fitting plane
xx, yy = np.meshgrid(np.linspace(-6,6,11), np.linspace(-2,10,11))
zz = (-normal[0]*xx - normal[1]*yy - d) / normal[2]
ax.plot_surface(xx, yy, zz, rstride=2, cstride=2, color='y' ,alpha=0.2, shade=False)

#--- Plot fitting circle
ax.plot(*P_fitcircle.T, color='k', ls='--', lw=2, label='Fitting circle')
#ax.plot(*P_fitarc.T, color='k', ls='-', lw=3, label='Fitting arc')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend() 
cx=C[:,0]
cy=C[:,1]
cz=C[:,2]
#print(C)
ax.scatter(cx,cy,cz)
ax.scatter(cx+normal[0],cy+normal[1],cz+normal[2])
ax.set_aspect('auto', 'datalim')
ax.set_xlim3d(-20, 20)
ax.set_ylim3d(-20, 20)
ax.set_zlim3d(-20, 20)
plt.show() #hier wird der ganze Spaß dann geplottet

