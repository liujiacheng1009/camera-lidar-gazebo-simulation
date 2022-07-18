import numpy as np 
import math
import sys 
from pyquaternion import Quaternion

def Euler2A(phi, theta, psi):
  Rx_phi = [[1, 0, 0], [0, math.cos(phi), -math.sin(phi)], [0, math.sin(phi), math.cos(phi)]]
  Ry_theta = [[math.cos(theta), 0, math.sin(theta)], [0, 1, 0], [-math.sin(theta), 0, math.cos(theta)]]
  Rz_psi = [[math.cos(psi), -math.sin(psi), 0], [math.sin(psi), math.cos(psi), 0], [0, 0, 1]]
  A = np.matmul(Rz_psi, np.matmul(Ry_theta, Rx_phi))
  return A

def AxisAngle(A):
    det = np.linalg.det(A)
    if det < 0.99999999 or det > 1.00000001:
        print("Matrica prosledjena f-ji AxisAngle nema determinantu 1, vec:")
        print(det)
        sys.exit(1)
        
    A1 = A - np.eye(3)
    p = np.cross(A1[0], A1[1])
    p = p / np.linalg.norm(p)
    u = A1[0]
    u = u / np.linalg.norm(u)
    u_prime = A.dot(u)
    u_prime = u_prime / np.linalg.norm(u_prime)
    phi = np.arccos(u.dot(u_prime))
    if np.linalg.det(np.array([u, u_prime, p])) < 0:
        p = -p
    return p, phi


def Rodrigez(p, fi):
  p_intensity = math.sqrt(np.dot(p, p))
  p_unit = np.array([p[0] / p_intensity, p[1] / p_intensity, p[2] / p_intensity])

  pT = np.transpose([p_unit])
  ppT = np.matmul(pT, [p_unit])
  identity = np.eye(3, dtype=float)
  pX = np.array([[0, -p_unit[2], p_unit[1]], [p_unit[2], 0, -p_unit[0]], [-p_unit[1], p_unit[0], 0]])

  A = ppT + math.cos(fi) * (identity - ppT) + math.sin(fi) * pX

  return A


def A2Euler(A):
  det = np.linalg.det(A)
  if det < 0.99999999 or det > 1.00000001:
    print("Matrica prosledjena f-ji A2Euler nema determinantu 1, vec:")
    print(det)
    sys.exit(1)

  psi = -100
  theta = -100
  fi = -100

  if A[2, 0] < 1:
    if A[2,0] > -1:
      psi = np.arctan2(A[1,0], A[0, 0])
      theta = math.asin(-A[2,0])
      fi = np.arctan2(A[2, 1], A[2, 2])
    else:
      psi = np.arctan2(-A[0, 1], A[1, 1])
      theta = math.pi / 2
      fi = 0
  else:
    psi = np.arctan2(-A[0, 1], A[1, 1])
    theta = -math.pi / 2
    fi = 0

  return fi, theta, psi

def AxisAngle2Q(p, phi):
    w = np.cos(phi / 2)
    p = p / np.linalg.norm(p)
    im = np.sin(phi / 2) * p
    q = Quaternion(imaginary=im, real=w)
    return q


def Q2AxisAngle(q):
    q = q.normalised
    if q.real < 0:
        q = -q
    phi = 2 * np.arccos(q.real)
    if q.real == 1:
        p = np.eye(1, 3)
    else:
        p = q.imaginary
        p = p / np.linalg.norm(p)

    return p, phi

def slerp(q1, q2, t, tm):
    dot = q1.conjugate.real * q2.real
    if dot < 0:
        q1 = -q1
        dot = -dot
    if dot > 0.95:
        return q1

    phi = np.arccos(dot)
    qs = (np.sin(phi * (1 - t / tm)) / np.sin(phi)) * q1 + (np.sin(phi * (t / tm)) / np.sin(phi)) * q2
    return qs

# board 4, 0.8,1.6,0.2,1.6,0   -x 3 -y 0 -z 2.5 -R 0.2 -P 1.57 -Y 0
# board 4, -0.8,1.6,-0.2,1.6,0 -x 6 -y -1 -z 3.5 -R -0.2 -P 1.57 -Y 0

point = np.array([3, 0 , 2.5])
euler_angles = np.array([0.2,1.57,0])
A = Euler2A(euler_angles[0], euler_angles[1], euler_angles[2])
p, phi = AxisAngle(A)
q = AxisAngle2Q(p, phi)

point2 = np.array([6,-1,3.5])
euler_angles2 = np.array([-0.2,1.57,0])
A2 = Euler2A(euler_angles2[0], euler_angles2[1], euler_angles2[2])
p2, phi2 = AxisAngle(A2)
q2 = AxisAngle2Q(p2, phi2)

N = 8
for i in range(N):
    qs = slerp(q, q2, i, N-1)
    p_s, phi_s = Q2AxisAngle(qs) 
    A_s = Rodrigez(p_s, phi_s)
    phi, theta, psi = A2Euler(A_s)
    c = (1 - i / (N-1)) * point + (i / (N-1)) * point2
    print(phi, theta, psi,c)


# 0.2 1.57 0 3 0 2.5 
# 0.136 1.57 0 3.429 -0.143 2.643
# 0.079 1.57 0 3.857 -0.286 2.786
# 0.026 1.57 0 4.286 -0.429 2.929
# -0.026 1.57 0 4.714 -0.571 3.071
# -0.079 1.57 0 5.143 -0.714 3.214
# -0.136 1.57 0 5.571 -0.857 3.357
# -0.2 1.57 0 6 -1 3.5 
