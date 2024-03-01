#include <complex>
#include <limits>
#include <iostream>
#include <boost/multi_array.hpp>

int main(int argc, char const *argv[])
{
  if (argc < 8)
  {
    std::cout << "Too few arguments. Provide the following:" << std::endl;
    std::cout << "x\tRadius around x axis" << std::endl;
    std::cout << "y\tRadius around y axis" << std::endl;
    std::cout << "z\tRadius around z axis" << std::endl;
    std::cout << "ux\tConfidence band of x radius (as in x +- ux)" << std::endl;
    std::cout << "uy\tConfidence band of y radius (as in y +- uy)" << std::endl;
    std::cout << "uz\tConfidence band of z radius (as in z +- uz)" << std::endl;
    std::cout << "res\tResolution of calibration space. Try 100.\nUsage:" << std::endl;
    std::cout <<  "./sphere_extrinsics <x> <y> <z> <ux> <uy> <uz> <res>" << std::endl;
  }
  double x, y, z, ux, uy, uz;
  x = std::atof(argv[1]);
  ux = std::atof(argv[4]);
  y = std::atof(argv[2]);
  uy = std::atof(argv[5]);
  z = std::atof(argv[3]);
  uz = std::atof(argv[6]);;
  int voxel_per_axis = std::atoi(argv[7]);;

  std::cout << "Setting up calibration space using:" << std::endl;
  std::cout << "Radius x = " << x << " +- " << ux << std::endl
    << "Radius y = " << y << " +- " << uy << std::endl
    << "Radius z = " << z << " +- " << uz << std::endl
    << voxel_per_axis << " voxel per axis" << std::endl;  
  double* xs = new double[voxel_per_axis];
  double step_x = 2*ux/voxel_per_axis;
  
  double* ys = new double[voxel_per_axis];
  double step_y = 2*uy/voxel_per_axis;
  
  double* zs = new double[voxel_per_axis];
  double step_z = 2*uz/voxel_per_axis;

  for (int i = 0; i < voxel_per_axis; ++i) 
  {
    xs[i]= x-ux + i*step_x;
    ys[i]= y-uy + i*step_y;
    zs[i]= z-uz + i*step_z;
  }

  std::cout << "Setting up complex solution space..." << std::endl;
  boost::multi_array<std::complex<double>, 4> solutions(boost::extents[voxel_per_axis][voxel_per_axis][voxel_per_axis][3]);
  int best_indices[3];
  double best_dist = std::numeric_limits<double>::max();
  int voxel_center = voxel_per_axis / 2;
  std::cout << "Calibrating now...";
  std::cout.flush(); 
  for (int i = 0; i < voxel_per_axis; i++)
  {
    for (int j = 0; j < voxel_per_axis; j++)
    {
      for (int k = 0; k < voxel_per_axis; k++)
      {
        //double dist2 = (xs[i]-x)*(xs[i]-x) + (ys[j]-y)*(ys[j]-y) + (zs[k]-z)*(zs[k]-z);
        double dist2 = (i - voxel_center)*(i - voxel_center)+(j - voxel_center)*(j - voxel_center)+(k - voxel_center)*(k - voxel_center);
        if (best_dist < dist2) continue;
        solutions[i][j][k][0] = std::sqrt(0.5*(-xs[i]*xs[i]+ys[j]*ys[j]+zs[k]*zs[k]));
        if (solutions[i][j][k][0].imag() != 0 || std::isnan(solutions[i][j][k][0].real())) continue;
        solutions[i][j][k][1] = std::sqrt(0.5*(xs[i]*xs[i]-ys[j]*ys[j]+zs[k]*zs[k]));
        if (solutions[i][j][k][1].imag() != 0 || std::isnan(solutions[i][j][k][1].real())) continue;
        solutions[i][j][k][2] = std::sqrt(0.5*(xs[i]*xs[i]+ys[j]*ys[j]-zs[k]*zs[k]));
        if (solutions[i][j][k][2].imag() != 0 || std::isnan(solutions[i][j][k][2].real())) continue;
        if (dist2 < best_dist)
        {
          best_dist = dist2;
          best_indices[0] = i;
          best_indices[1] = j;
          best_indices[2] = k;
        }
      }  
    }
    std::cout << ".";
    std::cout.flush();
  }
  std::cout << std::endl;
  auto best = solutions[best_indices[0]][best_indices[1]][best_indices[2]];
  std::cout << "Best solution:\n"
            << "tx = " << best[0].real() << "\n"
            << "ty = " << best[1].real() << "\n"
            << "tz = " << best[2].real()
            << std::endl;
  return 0;
}
