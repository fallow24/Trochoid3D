# Trochoid3D

This will be used as a motion model for spherical mobile mapping systems.
Also, the model assumes movement due to rolling without slip in arbitrary planes.
The ground normal at any time is considered as rotation around that normal will not lead to translation of the balls center. 

![Input velocities and resulting sensor trajectory](https://github.com/fallow24/Trochoid3D/blob/main/img/schematics.png)

The sensors rotation around the balls center is described through the rotation derivative which correspond to local gyro measurements $\vec{\omega}^r$   
$$\dot{\mathbf{R}}_r = \vec{\omega}^r_{\textrm{x}} \cdot \mathbf{R}_r$$
where the cross-product matrix is defined as 
$$\vec{\omega}_{\times} = \begin{pmatrix}0 & -\omega_3 & \omega_2\\\omega_3 & 0 & -\omega_1\\-\omega_2 & \omega_1 & 0\end{pmatrix} .$$
Go study group theory and learn about the Lie algebra $\mathfrak{so}(3)$ which is the tangent space to $\textrm{SO}(3)$ at its identity, to find out why this works. 
Anyways, through the inverse rotation we transform the local measurements into the global frame, leading to 
$$\vec{\omega} = \mathbf{R}_r^{-1} \cdot \vec{\omega}^r$$
and
$$\vec{s} = \mathbf{R}_r^{-1} \cdot \vec{s}^{\,r} + \,\vec{p}$$
with 
$$\vec{p}(t) = \int_0^t \vec{v}(\tau) d\tau ,$$
where the velocity of the balls center over ground with normal $\vec{n}$ is
$$\vec{v} = r_s \vec{\omega} \times \vec{n} .$$
The combined model:
$$\vec{s} = \mathbf{R}_r^{-1} \cdot \vec{s}^{\,r} + \int \left( \left[ \mathbf{R}_r^{-1} \cdot r_s \cdot \vec{\omega}^r \right] \times \vec{n} \right)$$
expands, not ommiting the time dependence, to:
$$\vec{s}(t) = \left[ \int_0^t \vec{\omega}^r_{\times}(\tau)\mathbf{R}_r(\tau)d\tau \right]^{-1} \vec{s}^{\,r} + \int_0^t \left( \left[ \bigg\{ \int_0^t \vec{\omega}^r_{\times}(\tau)\mathbf{R}_r(\tau)d\tau \bigg\}^{-1} r_s \vec{\omega}^r(t) \right]\times \vec{n}(t) \right)$$
which is the kinematic model to be solved numerically for any arbitrary gyro measurements.