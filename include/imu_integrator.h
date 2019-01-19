#pragma once

struct imu_delta_s {
    float dt;
    float delta_ang[3];
    float delta_vel[3];
};

extern struct pubsub_topic_s imu_deltas_topic;

void imu_integrator_trigger(void);
