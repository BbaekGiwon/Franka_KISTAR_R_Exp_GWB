#include "fk_solver.h"

#include <cmath>

namespace
{
constexpr double kPi = 3.14159265358979323846;

struct Transform
{
    double r[3][3];
    double p[3];
};

Transform identity()
{
    Transform t{};
    t.r[0][0] = 1.0;
    t.r[1][1] = 1.0;
    t.r[2][2] = 1.0;
    return t;
}

Transform compose(const Transform &a, const Transform &b)
{
    Transform out{};
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            out.r[i][j] = a.r[i][0] * b.r[0][j] + a.r[i][1] * b.r[1][j] + a.r[i][2] * b.r[2][j];
        }
        out.p[i] = a.r[i][0] * b.p[0] + a.r[i][1] * b.p[1] + a.r[i][2] * b.p[2] + a.p[i];
    }
    return out;
}

Transform make_rpy_xyz(double roll, double pitch, double yaw, double x, double y, double z)
{
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);

    Transform t{};
    // URDF rpy: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    t.r[0][0] = cy * cp;
    t.r[0][1] = cy * sp * sr - sy * cr;
    t.r[0][2] = cy * sp * cr + sy * sr;
    t.r[1][0] = sy * cp;
    t.r[1][1] = sy * sp * sr + cy * cr;
    t.r[1][2] = sy * sp * cr - cy * sr;
    t.r[2][0] = -sp;
    t.r[2][1] = cp * sr;
    t.r[2][2] = cp * cr;

    t.p[0] = x;
    t.p[1] = y;
    t.p[2] = z;
    return t;
}

Transform axis_rotation(double ax, double ay, double az, double q)
{
    const double c = std::cos(q);
    const double s = std::sin(q);
    const double v = 1.0 - c;

    Transform t = identity();
    t.r[0][0] = ax * ax * v + c;
    t.r[0][1] = ax * ay * v - az * s;
    t.r[0][2] = ax * az * v + ay * s;

    t.r[1][0] = ay * ax * v + az * s;
    t.r[1][1] = ay * ay * v + c;
    t.r[1][2] = ay * az * v - ax * s;

    t.r[2][0] = az * ax * v - ay * s;
    t.r[2][1] = az * ay * v + ax * s;
    t.r[2][2] = az * az * v + c;
    return t;
}

void rotmat_to_quat_wxyz(const double r[3][3], float quat[4])
{
    const double trace = r[0][0] + r[1][1] + r[2][2];
    double qw;
    double qx;
    double qy;
    double qz;

    if (trace > 0.0)
    {
        const double s = std::sqrt(trace + 1.0) * 2.0;
        qw = 0.25 * s;
        qx = (r[2][1] - r[1][2]) / s;
        qy = (r[0][2] - r[2][0]) / s;
        qz = (r[1][0] - r[0][1]) / s;
    }
    else if ((r[0][0] > r[1][1]) && (r[0][0] > r[2][2]))
    {
        const double s = std::sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2.0;
        qw = (r[2][1] - r[1][2]) / s;
        qx = 0.25 * s;
        qy = (r[0][1] + r[1][0]) / s;
        qz = (r[0][2] + r[2][0]) / s;
    }
    else if (r[1][1] > r[2][2])
    {
        const double s = std::sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2.0;
        qw = (r[0][2] - r[2][0]) / s;
        qx = (r[0][1] + r[1][0]) / s;
        qy = 0.25 * s;
        qz = (r[1][2] + r[2][1]) / s;
    }
    else
    {
        const double s = std::sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2.0;
        qw = (r[1][0] - r[0][1]) / s;
        qx = (r[0][2] + r[2][0]) / s;
        qy = (r[1][2] + r[2][1]) / s;
        qz = 0.25 * s;
    }

    const double norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    quat[0] = static_cast<float>(qw / norm);
    quat[1] = static_cast<float>(qx / norm);
    quat[2] = static_cast<float>(qy / norm);
    quat[3] = static_cast<float>(qz / norm);
}

Transform fixed_then_revolute(double roll, double pitch, double yaw,
                              double x, double y, double z,
                              double ax, double ay, double az,
                              double q)
{
    return compose(make_rpy_xyz(roll, pitch, yaw, x, y, z), axis_rotation(ax, ay, az, q));
}

double to_radian_from_encoder(int16_t enc)
{
    return static_cast<double>(enc) * (kPi / 8192.0);
}

void solve_thumb(const double q[4], float pos[3], float quat[4])
{
    Transform t = identity();
    t = compose(t, make_rpy_xyz(0.0, 0.0, 0.0, 0.0, 0.0, 0.071));
    t = compose(t, make_rpy_xyz(-0.0873, 0.0, 0.0, -0.02325, 0.01224, -0.02988));
    t = compose(t, fixed_then_revolute(-1.5708, 0.0, 0.0, 0.0197, 0.002, 0.0, 0.0, 1.0, 0.0, q[0]));
    t = compose(t, fixed_then_revolute(0.0, 0.0, 0.0, -0.002, 0.0, 0.036, 0.0, 0.0, 1.0, q[1]));
    t = compose(t, fixed_then_revolute(0.0, 0.0, 0.0, 0.002, 0.0, 0.0102, 0.0, 1.0, 0.0, q[2]));
    t = compose(t, fixed_then_revolute(0.0, 0.0, 0.0, 0.0, 0.0, 0.0335, 0.0, 1.0, 0.0, q[3]));
    t = compose(t, make_rpy_xyz(0.0, 0.0, 0.0, -0.002, 0.0, 0.010));

    pos[0] = static_cast<float>(t.p[0]);
    pos[1] = static_cast<float>(t.p[1]);
    pos[2] = static_cast<float>(t.p[2]);
    rotmat_to_quat_wxyz(t.r, quat);
}

void solve_finger_common(const double base_roll, const double base_pitch, const double base_yaw,
                         const double base_x, const double base_y, const double base_z,
                         const double q[4], float pos[3], float quat[4])
{
    Transform t = identity();
    t = compose(t, make_rpy_xyz(0.0, 0.0, 0.0, 0.0, 0.0, 0.071));
    t = compose(t, make_rpy_xyz(base_roll, base_pitch, base_yaw, base_x, base_y, base_z));
    t = compose(t, fixed_then_revolute(0.0, 0.0, 0.0, 0.02325, 0.0, 0.0047, 1.0, 0.0, 0.0, q[0]));
    t = compose(t, fixed_then_revolute(0.0, 0.0, 0.0, 0.002, 0.0, 0.0195, 0.0, 1.0, 0.0, q[1]));
    t = compose(t, fixed_then_revolute(0.0, 0.0, 0.0, 0.0, 0.0, 0.0335, 0.0, 1.0, 0.0, q[2]));
    t = compose(t, make_rpy_xyz(0.0, 0.0, 0.0, 0.0, 0.0, 0.0267));
    t = compose(t, fixed_then_revolute(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, q[3]));
    t = compose(t, make_rpy_xyz(0.0, 0.0, 0.0, -0.002, 0.0, 0.010));

    pos[0] = static_cast<float>(t.p[0]);
    pos[1] = static_cast<float>(t.p[1]);
    pos[2] = static_cast<float>(t.p[2]);
    rotmat_to_quat_wxyz(t.r, quat);
}
} // namespace

void compute_hand_fk_from_joint_pos(const int16_t joint_pos[Hand_DOF], float tip_pos[4][3], float tip_quat_wxyz[4][4])
{
    double q[Hand_DOF];
    for (int i = 0; i < Hand_DOF; ++i)
    {
        q[i] = to_radian_from_encoder(joint_pos[i]);
    }

    solve_thumb(&q[0], tip_pos[0], tip_quat_wxyz[0]);
    solve_finger_common(-0.0873, 0.0, 0.0, -0.02325, 0.02935, -0.0004, &q[4], tip_pos[1], tip_quat_wxyz[1]);
    solve_finger_common(0.0, 0.0, 0.0, -0.02325, 0.0, 0.0, &q[8], tip_pos[2], tip_quat_wxyz[2]);
    solve_finger_common(0.0873, 0.0, 0.0, -0.02325, -0.02935, -0.0004, &q[12], tip_pos[3], tip_quat_wxyz[3]);
}
