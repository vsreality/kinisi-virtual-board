/* Physical sensor geometry only. No estimator state or correction lives here. */
#ifndef KINISI_SIM_IMU_MOUNT_H
#define KINISI_SIM_IMU_MOUNT_H
#include <math.h>
#include <string.h>

typedef struct {
    double offset[3], board_to_chassis[9];
    int centered_identity;
} sim_imu_mount_t;
#define IMU_MOUNT_IDENTITY {{0., 0., 0.}, {1., 0., 0., 0., 1., 0., 0., 0., 1.}, 1}

static int sim_mount_set(sim_imu_mount_t *mount, const double offset[3], const double matrix[9]) {
    if (!offset || !matrix) {
        return 0;
    }
    for (unsigned i = 0; i < 3; ++i) {
        if (!isfinite(offset[i])) return 0;
    }
    for (unsigned i = 0; i < 9; ++i) {
        if (!isfinite(matrix[i])) return 0;
    }
    for (unsigned i = 0; i < 3; ++i) {
        for (unsigned j = 0; j < 3; ++j) {
            double dot = 0.;
            for (unsigned k = 0; k < 3; ++k) {
                dot += matrix[3 * i + k] * matrix[3 * j + k];
            }
            if (fabs(dot - (i == j ? 1. : 0.)) > 1e-6) return 0;
        }
    }
    double det = matrix[0] * (matrix[4] * matrix[8] - matrix[5] * matrix[7])
               - matrix[1] * (matrix[3] * matrix[8] - matrix[5] * matrix[6])
               + matrix[2] * (matrix[3] * matrix[7] - matrix[4] * matrix[6]);
    if (fabs(det - 1.) > 1e-6) return 0;

    sim_imu_mount_t next = IMU_MOUNT_IDENTITY;
    memcpy(next.offset, offset, sizeof(next.offset));
    memcpy(next.board_to_chassis, matrix, sizeof(next.board_to_chassis));
    for (unsigned i = 0; i < 3; ++i) {
        if (offset[i] != 0.) next.centered_identity = 0;
    }
    for (unsigned i = 0; i < 9; ++i) {
        if (matrix[i] != (i % 4 == 0 ? 1. : 0.)) next.centered_identity = 0;
    }
    *mount = next;
    return 1;
}

static void sim_mount_apply(const sim_imu_mount_t *mount, const double acceleration[3],
                            const double gyro[3], double yaw_acceleration,
                            double board_acceleration[3], double board_gyro[3]) {
    /* Identity bypass preserves the legacy centered stream bit for bit. */
    if (mount->centered_identity) {
        memcpy(board_acceleration, acceleration, 3 * sizeof(double));
        memcpy(board_gyro, gyro, 3 * sizeof(double));
        return;
    }
    /* a_chip = a_origin + alpha x r + omega x (omega x r).
     * Planar chassis: omega=(0,0,w), alpha=(0,0,alpha_z). Gravity
     * is already included in acceleration; a height offset adds no term here. */
    double w = gyro[2], x = mount->offset[0], y = mount->offset[1];
    double chip[3] = {
        acceleration[0] - yaw_acceleration * y - w * w * x,
        acceleration[1] + yaw_acceleration * x - w * w * y,
        acceleration[2]
    };
    for (unsigned i = 0; i < 3; ++i) {
        board_acceleration[i] = 0.;
        board_gyro[i] = 0.;
        for (unsigned j = 0; j < 3; ++j) {
            board_acceleration[i] += mount->board_to_chassis[3 * j + i] * chip[j];
            board_gyro[i] += mount->board_to_chassis[3 * j + i] * gyro[j];
        }
    }
}
#endif
