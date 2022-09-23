import numpy as np


class Quat:
    def __init__(self, re, imag):
        self.w = re
        self.re = re
        self.imag = imag
        self.x, self.y, self.z = imag[0], imag[1], imag[2]
        self.angle = np.arccos(re) * 2
        imaglen = np.linalg.norm(imag)
        self.axis = imag / imaglen if imaglen != 0 else imag
        self.length = np.linalg.norm(np.append(imag, re))

    def inv(self):
        lensq = self.length * self.length
        return Quat(self.re / lensq, -self.imag / lensq)

    def __add__(self, other):
        return Quat(self.re + other.re, self.imag + other.imag)

    def __mul__(self, other):
        if isinstance(other, Quat):
            re = (self.re * other.re) - np.dot(self.imag, other.imag)
            imag = (self.re * other.imag) + (other.re * self.imag) + np.cross(self.imag, other.imag)
            return Quat(re, imag)

        elif isinstance(other, float) or isinstance(other, int):
            return Quat(self.re * other, self.imag * other)

    def __rmul__(self, lhs):
        if isinstance(lhs, float) or isinstance(lhs, int):
            return self * lhs

    def dot(self, other):
        return (self.re * self.re) + np.dot(self.imag, other.imag)

    def conj(self):
        return Quat(self.re, -self.imag)

    def __truediv__(self, other):
        if isinstance(other, Quat):
            return self * other.inv()

        elif isinstance(other, float) or isinstance(other, int):
            return self * (1. / other)

    def __str__(self):
        return str(self.w) + " + " + str(self.x) + "i + " + str(self.y) + "j + " + str(self.z) + "k"

    def __eq__(self, other):
        return self.re == other.self and self.imag == other.imag

    def to_float(self):
        assert self.x == self.y == self.z == 0.
        return self.w

    def rotate_vector(self, v):
        return (self * Quat(0, v) * self.inv()).imag

    @staticmethod
    def from_axis_angle(axis, angle):
        assert len(axis) == 3 and angle < np.pi  # must be a convex angle
        re = np.cos(angle / 2)
        imag = np.sin(angle / 2) * axis / np.linalg.norm(axis)
        return Quat(re, imag)


def angle(q1:Quat, q2:Quat):
    return np.arccos(q1.dot(q2) / (q1.length * q2.length))


def SBisect(q1:Quat, q2:Quat):
    q = (q1 + q2)
    return q / q.length


def SDouble(q1:Quat, q2:Quat):
    return (2 * q1.dot(q2) * q2) - q1


def Slerp(q1:Quat, q2:Quat, u):
    omega = angle(q1, q2)
    return ((np.sin((1-u) * omega) * q1) + (np.sin(omega * u) * q2)) / np.sin(omega)


def cubic_quat_spline(quat_pts, t):
    '''
        NOTE: REMEMBER TO INCLUDE PHANTOM POINTS AT THE BEGINNING AND END OF THE LIST

        quat_pts: a list of tuples (t_i, q_i) ordered s.t. t_i < t_j for i < j and q_i are the points to interpolate
        t: The value from 0 to t_max to evaluate the spline at.
    '''
    assert quat_pts[0][0] <= t <= quat_pts[-1][0]
    assert len(quat_pts) >= 4

    # 1. Find the segment to evaluate on.
    b0, b3, prev, nxt = None, None, None, None
    for i in range(1, len(quat_pts)-1):
        q1, q2 = quat_pts[i], quat_pts[i+1]
        if q1[0] <= t <= q2[0]:
            b0, b3 = q1, q2
            prev = quat_pts[i-1]
            next = quat_pts[i+2]
            break

    assert b0 is not None

    # 2. Generate other 2 control points
    b1 = Slerp(b0, SBisect(SDouble(prev, b0), b3), 1. / 3.)# if prev != None else ?
    b2 = Slerp(b3, SBisect(b0, SDouble(nxt, b3)), 1. / 3.)# if nxt != None else ?

    # 3. TODO: Squash t into interval and evaluate spline using 6 SLERPS


def euler_to_rotmat(theta_z, theta_y, theta_x):
    rotmatZ = np.array([[np.cos(theta_z), -np.sin(theta_z), 0],
                        [np.sin(theta_z), np.cos(theta_z), 0],
                        [0, 0, 1]])

    rotmatY = np.array([[np.cos(theta_y), 0, np.sin(theta_y)],
                        [0, 1, 0],
                        [-np.sin(theta_y), 0, np.cos(theta_y)]])

    rotmatX = np.array([[1, 0, 0],
                        [0, np.cos(theta_x), -np.sin(theta_x)],
                        [0, np.sin(theta_x), np.cos(theta_x)]])

    return np.dot(rotmatZ, np.dot(rotmatY, rotmatX))


def euler_to_quat(theta_z, theta_y, theta_x):
    R = euler_to_rotmat(theta_z, theta_y, theta_x)
    w = 0.5 * np.sqrt(R[0][0] + R[1][1] + R[2][2] + 1)
    x = (R[2][1] - R[1][2]) / (4 * w)
    y = (R[0][2] - R[2][0]) / (4 * w)
    z = (R[1][0] - R[0][1]) / (4 * w)
    return Quat(w, np.array([x, y, z]))



