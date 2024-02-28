from cmath import sqrt
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

x=12.8845 #[cm] Radius measured x 
ux=0.79969286 #[cm] Uncertanty of radius x

y=14.0177 #[cm] Radius measured y
uy=1.12347022 #[cm] Uncertanty of radius y

z=0.9745 #[cm] Radius measured z
uz=0.02354371 #[cm] Uncertanty of radius z

# Find all the valid real solutions, greedy
voxel_per_axis = 40 # 100 is a good value
xs = np.arange(x - ux, x + ux, 2*ux/voxel_per_axis)
ys = np.arange(y - uy, y + uy, 2*uy/voxel_per_axis)
zs = np.arange(z - uz, z + uz, 2*uz/voxel_per_axis)
solutions = np.empty([xs.size, ys.size, zs.size, 3], complex)
real_solutions = np.empty([xs.size, ys.size, zs.size], bool)
color_solutions = np.empty([xs.size, ys.size, zs.size])
min_real_color = float("inf")
best_dist = float("inf")
best_indices = np.empty([1,1,1])
for i in range(xs.size): 
  for j in range(ys.size):
    for k in range(zs.size):
      solutions[i][j][k][0] = sqrt(0.5*(-xs[i]*xs[i]+ys[j]*ys[j]+zs[k]*zs[k])) 
      solutions[i][j][k][1] = sqrt(0.5*(xs[i]*xs[i]-ys[j]*ys[j]+zs[k]*zs[k])) 
      solutions[i][j][k][2] = sqrt(0.5*(xs[i]*xs[i]+ys[j]*ys[j]-zs[k]*zs[k])) 
      # Distance based color scheme, looks distorted when using small uncertanties
      #color_solutions[i][j][k] = np.sqrt((xs[i] - x)**2 + (ys[j] - y)**2 + (zs[k] - z)**2)
      # Index based color scheme, colors do not represent 'true' distance, but clearly emphasize the 'best estimate'
      color_solutions[i][j][k] = np.sqrt((i - xs.size/2)**2 + (j - ys.size/2)**2 + (k - zs.size/2)**2)
      if (np.iscomplex(solutions[i][j][k])[0] or np.iscomplex(solutions[i][j][k])[1] or np.iscomplex(solutions[i][j][k])[2]):
        real_solutions[i][j][k] = False
        color_solutions[i][j][k] = 0
      else: 
        real_solutions[i][j][k] = True
        if (color_solutions[i][j][k] < min_real_color):
          min_real_color = color_solutions[i][j][k]
          best_indices = [i,j,k]

color_solutions = color_solutions - min_real_color
color_solutions = (color_solutions) / (np.max(color_solutions))
colormap = plt.cm.viridis(color_solutions)
print(best_indices)
print(solutions[best_indices[0]][best_indices[1]][best_indices[2]])


# TODO: Store values of original x,y,z radii during the iteration to use them for gaussian error estimation
# Translation parameters
tx= sqrt(0.5*(-x*x+y*y+z*z)) 
ty= sqrt(0.5*(x*x-y*y+z*z)) 
tz= sqrt(0.5*(x*x+y*y-z*z)) 

# Gauss stuff
dtxdx = -0.7071068 * x/sqrt(-x*x+y*y+z*z) 
dtxdy = 0.7071068 * y/sqrt(-x*x+y*y+z*z)
dtxdz = 0.7071068 * z/sqrt(-x*x+y*y+z*z)
dtydx = 0.7071068 * x/sqrt(x*x-y*y+z*z)
dtydy = -0.7071068 * y/sqrt(x*x-y*y+z*z)
dtydz = 0.7071068 * z/sqrt(x*x-y*y+z*z)
dtzdx = 0.7071068 * x/sqrt(x*x+y*y-z*z)
dtzdy = 0.7071068 * y/sqrt(x*x+y*y-z*z)
dtzdz = -0.7071068 * z/sqrt(x*x+y*y-z*z)

# # Uncertanty propagation
dx = sqrt( (dtxdx*ux)**2 + (dtxdy*uy)**2 + (dtxdz*uz)**2 )
dy = sqrt( (dtydx*ux)**2 + (dtydy*uy)**2 + (dtydz*uz)**2 )
dz = sqrt( (dtzdx*ux)**2 + (dtzdy*uy)**2 + (dtzdz*uz)**2 )

print(f'Tx= {tx} +- {dx}')
print(f'Ty= {ty} +- {dy}')
print(f'Tz= {tz} +- {dz}')
ax = plt.figure().add_subplot(projection='3d')
ax.voxels(real_solutions,
          facecolors=colormap,
          linewidth=0.5)
plt.show()
