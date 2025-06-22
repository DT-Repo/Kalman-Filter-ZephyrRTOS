#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

#include <zsl/orientation/fusion/fusion.h>
#include <zsl/orientation/orientation.h>
#include <zsl/zsl.h>
#include <zsl/instrumentation.h>

#define DEBUG_KALMAN 0

#if DEBUG_KALMAN
#include "data.h"
#endif

#define M_PI 3.14159265358979323846264338327950288
#define degToRad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)
#define radToDeg(angleInRadians) ((angleInRadians) * 180.0 / M_PI)

// Config settings for the extended Kalman filter.
static zsl_real_t _kalm_P[16] = {
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0};

static struct zsl_fus_kalm_cfg kalm_cfg = {
    .var_g = degToRad(0.015) * 23.7171,     // sqrt(562.5) //SQRT(ODR[HZ])
    .var_a = (230 * 1e-6 * 9.81) * 23.7171, // sqrt(562.5) //SQRT(ODR[HZ])
    .var_m = 0.01,
    .P = {
        .sz_rows = 4,
        .sz_cols = 4,
        .data = _kalm_P,
    },
};

static struct zsl_fus_drv kalm_drv = {

    .init_handler = zsl_fus_kalm_init,
    .feed_handler = zsl_fus_kalm_feed,
    .error_handler = zsl_fus_kalm_error,
    .config = &kalm_cfg,
};

void fusion_init(struct zsl_fus_drv *drv)
{
	/* Init filter at 100 Hz. */
	drv->init_handler(100.0, drv->config);
}

void fusion_demo(struct zsl_fus_drv *drv)
{
    struct zsl_quat q = {.r = 1.0, .i = 0.0, .j = 0.0, .k = 0.0};
    struct zsl_euler e = {0};
    uint32_t ns = 0;

    int rc = 0;

    const struct device *const icm20948 = DEVICE_DT_GET_ONE(invensense_icm20948);

    struct sensor_value accel[3];
    struct sensor_value gyro[3];
    struct sensor_value magn[3];

    rc = sensor_sample_fetch(icm20948);

    rc = sensor_channel_get(icm20948, SENSOR_CHAN_ACCEL_XYZ,
                            accel);
    rc = sensor_channel_get(icm20948, SENSOR_CHAN_GYRO_XYZ,
                            gyro);
    rc = sensor_channel_get(icm20948, SENSOR_CHAN_MAGN_XYZ,
                            magn);

    ZSL_VECTOR_DEF(av, 3);
    ZSL_VECTOR_DEF(mv, 3);
    ZSL_VECTOR_DEF(gv, 3);

    av.data[0] = sensor_value_to_double(&accel[0]);
    av.data[1] = sensor_value_to_double(&accel[1]);
    av.data[2] = sensor_value_to_double(&accel[2]);

    mv.data[0] = sensor_value_to_double(&magn[0]);
    mv.data[1] = sensor_value_to_double(&magn[1]);
    mv.data[2] = sensor_value_to_double(&magn[2]);

    gv.data[0] = sensor_value_to_double(&gyro[0]);
    gv.data[1] = sensor_value_to_double(&gyro[1]);
    gv.data[2] = sensor_value_to_double(&gyro[2]);

    // ZSL_INSTR_START(ns);
    drv->feed_handler(&av, &mv, &gv, NULL, &q, drv->config);
    // ZSL_INSTR_STOP(ns);
    // printf("Took: %d ns\n", ns);

    // zsl_quat_print(&q);
    zsl_quat_to_euler(&q, &e);
    e.x *= 180. / ZSL_PI;
    e.y *= 180. / ZSL_PI;
    e.z *= 180. / ZSL_PI;
    zsl_eul_print(&e);
}
