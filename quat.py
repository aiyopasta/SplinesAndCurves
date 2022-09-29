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

    def __sub__(self, other):
        return self + (-1 * other)

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
        return (self.re * other.re) + np.dot(self.imag, other.imag)

    def conj(self):
        return Quat(self.re, -self.imag)

    def __truediv__(self, other):
        if isinstance(other, Quat):
            return self * other.inv()

        elif isinstance(other, float) or isinstance(other, int):
            return self * (1. / other)

    def __repr__(self):
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


def Bisect(v1, v2):
    return (v1 + v2) / 2.


def Double(v1, v2):
    return v1 + (2 * (v2 - v1))


def SDouble(q1:Quat, q2:Quat):
    return (2 * q1.dot(q2) * q2) - q1


def Slerp(q1:Quat, q2:Quat, u):
    omega = angle(q1, q2)
    return ((np.sin((1-u) * omega) * q1) + (np.sin(omega * u) * q2)) / np.sin(omega)


def lerp(a, b, t):
    assert 0 <= t <= 1
    return ((1 - t) * a) + (t * b)


def angular_lerp(a, b, t):
    a %= 2 * np.pi
    b %= 2 * np.pi
    if a == b:
        return a

    elif a < b:
        if b - a <= np.pi:
            return lerp(a, b, t)

        # Else b - a > 180
        return (a - lerp(0, (2 * np.pi) - b + a, t)) % (2 * np.pi)

    # Else a > b
    if a - b <= np.pi:
        return (a - lerp(0, a - b, t)) % (2 * np.pi)

    # Else a - b > 180
    return (a + lerp(0, (2 * np.pi) - a + b, t)) % (2 * np.pi)


def euler_angle_lerp(euler1, euler2, t):
    assert len(euler1) == len(euler2) == 3
    return np.array([angular_lerp(euler1[i], euler2[i], t) for i in range(3)])


def cubic_quat_spline(quat_pts, t):
    '''
        NOTE: REMEMBER TO INCLUDE PHANTOM POINTS AT THE BEGINNING AND END OF THE LIST

        quat_pts: a list of tuples (t_i, q_i) ordered s.t. t_i < t_j for i < j and q_i are the points to interpolate
        t: The value from 0 to t_max to evaluate the spline at.
    '''
    assert quat_pts[0][0] <= t <= quat_pts[-1][0]
    assert len(quat_pts) >= 4

    # 1. Find the segment to evaluate on.
    b0, b3, prev, nxt, left_idx = None, None, None, None, -1
    for i in range(1, len(quat_pts)-2):
        q1, q2 = quat_pts[i], quat_pts[i+1]
        if q1[0] <= t <= q2[0]:
            b0, b3 = q1[1], q2[1]
            prev = quat_pts[i-1][1]
            nxt = quat_pts[i+2][1]
            left_idx = i
            break

    assert left_idx != -1

    # 2. Generate other 2 control points
    b1 = Slerp(b0, SBisect(SDouble(prev, b0), b3), 1. / 3.)
    b2 = Slerp(b3, SBisect(b0, SDouble(nxt, b3)), 1. / 3.)

    # 3. Squash t into interval and evaluate spline using 6 SLERPS
    tmin, tmax = quat_pts[left_idx][0], quat_pts[left_idx+1][0]
    u = (t - tmin) / (tmax - tmin)
    middleSlerp = Slerp(b1, b2, u)
    return Slerp(Slerp(Slerp(b0, b1, u), middleSlerp, u), Slerp(middleSlerp, Slerp(b2, b3, u), u), u)


# TODO: Implement
def cubic_euler_spline(euler_pts, t):
    assert euler_pts[0][0] <= t <= euler_pts[-1][0]
    assert len(euler_pts) >= 4

    # 1. Find the segment to evaluate on.
    b0, b3, prev, nxt, left_idx = None, None, None, None, -1
    for i in range(1, len(euler_pts)-2):
        p1, p2 = euler_pts[i], euler_pts[i+1]
        if p1[0] <= t <= p2[0]:
            b0, b3 = p1[1], p2[1]
            prev = euler_pts[i-1][1]
            nxt = euler_pts[i+2][1]
            left_idx = i
            break

    assert left_idx != -1

    # 2. Generate other 2 control points
    b1 = euler_angle_lerp(b0, Bisect(Double(prev, b0), b3), 1. / 3.)
    b2 = euler_angle_lerp(b3, Bisect(b0, Double(nxt, b3)), 1. / 3.)

    # 3. Squash t into interval and evaluate spline using 6 LERPS
    tmin, tmax = euler_pts[left_idx][0], euler_pts[left_idx+1][0]
    u = (t - tmin) / (tmax - tmin)
    middleLerp = euler_angle_lerp(b1, b2, u)
    return euler_angle_lerp(euler_angle_lerp(euler_angle_lerp(b0, b1, u), middleLerp, u), euler_angle_lerp(middleLerp, euler_angle_lerp(b2, b3, u), u), u)



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



