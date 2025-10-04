// Dynamics/rigid_body_eom.hpp
#pragma once
#include <array>
#include <cmath>

std::array<double, 12> rigid_body_eom(
    const std::array<double, 12> &x, // [px,py,pz, u,v,w, φ,θ,ψ, p,q,r]
    const std::array<double, 6>
        &forces_moments, // [Fx, Fy, Fz, L, M, N] in body frame
    double mass,
    const std::array<double, 6> &inertia) { // [Ixx, Iyy, Izz, Ixy, Ixz, Iyz]

  // Unpack state
  double px = x[0], py = x[1], pz = x[2];      // Position (NED)
  double u = x[3], v = x[4], w = x[5];         // Body velocities
  double phi = x[6], theta = x[7], psi = x[8]; // Euler angles
  double p = x[9], q = x[10], r = x[11];       // Body rates

  // Unpack forces and moments from single array
  double Fx = forces_moments[0], Fy = forces_moments[1], Fz = forces_moments[2];
  double L = forces_moments[3], M = forces_moments[4], N = forces_moments[5];

  // Unpack full inertia tensor
  double Ixx = inertia[0], Iyy = inertia[1], Izz = inertia[2];
  double Ixy = inertia[3], Ixz = inertia[4], Iyz = inertia[5];

  // Equations of motion
  std::array<double, 12> xdot = {};

  // Position derivatives (NED frame) - Kinematic equations
  xdot[0] = u * cos(theta) * cos(psi) +
            v * (sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi)) +
            w * (cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi));
  xdot[1] = u * cos(theta) * sin(psi) +
            v * (sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi)) +
            w * (cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi));
  xdot[2] =
      -u * sin(theta) + v * sin(phi) * cos(theta) + w * cos(phi) * cos(theta);

  // Velocity derivatives (body frame) - Newton's laws
  xdot[3] = (Fx / mass) + (r * v - q * w); // u_dot
  xdot[4] = (Fy / mass) + (p * w - r * u); // v_dot
  xdot[5] = (Fz / mass) + (q * u - p * v); // w_dot

  // Euler angle derivatives - Kinematic equations
  xdot[6] = p + q * sin(phi) * tan(theta) + r * cos(phi) * tan(theta); // φ_dot
  xdot[7] = q * cos(phi) - r * sin(phi);                               // θ_dot
  xdot[8] = q * sin(phi) / cos(theta) + r * cos(phi) / cos(theta);     // ψ_dot

  // Angular rate derivatives - Full Euler's equations with products of inertia
  double denom = Ixx * Izz - Ixz * Ixz;
  xdot[9] = ((Izz * L + Ixz * N) - (Izz * (Iyy - Izz) - Ixz * Ixz) * q * r -
             Ixz * (Ixx - Iyy + Izz) * p * q +
             (Izz * Iyz + Ixz * Ixy) * (q * q - r * r) +
             (Ixz * Iyz - Izz * Ixy) * p * r) /
            denom; // p_dot

  xdot[10] = (M - (Ixx - Izz) * p * r - Ixz * (p * p - r * r) - Ixy * p * q +
              Iyz * q * r) /
             Iyy; // q_dot

  xdot[11] = ((Ixx * N + Ixz * L) - (Ixx * (Ixx - Iyy) + Ixz * Ixz) * p * q -
              Ixz * (Iyy - Izz - Ixx) * q * r +
              (Ixx * Ixy - Ixz * Iyz) * (p * p - r * r) +
              (Ixz * Ixy - Ixx * Iyz) * p * r) /
             denom; // r_dot

  return xdot;
}