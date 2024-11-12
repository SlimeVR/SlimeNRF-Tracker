#ifndef SLIMENRF_SENSOR_FUSIONS
#define SLIMENRF_SENSOR_FUSIONS

#include "fusion/fusion.h"
#include "fusion/sensorfusion.h"
#include "fusion/vqf.h"

enum fusion {
    FUSION_NONE,
    FUSION_FUSION,
    FUSION_MOTIONSENSE,
	FUSION_VQF
};

const char *fusion_names[] = {
    "None",
    "x-io Technologies Fusion",
    "NXP SensorFusion",
    "VQF"
};
const sensor_fusion_t *sensor_fusions[] = {
    NULL,
    &sensor_fusion_fusion,
    &sensor_fusion_motionsense,
    &sensor_fusion_vqf
};

#endif