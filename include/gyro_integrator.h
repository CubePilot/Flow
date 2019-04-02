#pragma once

struct gyro_delta_s {
    float dt;
    float delta_gyro[3];
};

extern struct pubsub_topic_s gyro_deltas_topic;
void gyro_integrator_trigger(void);
