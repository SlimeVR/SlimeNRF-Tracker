/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RETAINED_H_
#define RETAINED_H_

//#include "../Fusion/Fusion/Fusion.h"
//#include "../vqf-c/src/vqf.h"

#include <inttypes.h>

/* Example of validatable retained data. */
struct retained_data {
	uint8_t reboot_counter;
	uint8_t paired_addr[8];

	float accelBias[3];
	float gyroBias[3];
	float magBAinv[4][3];

	bool fusion_data_stored;
	uint8_t fusion_data[364]; // MAX(sizeof(FusionAhrs)+sizeof(FusionOffset), sizeof(vqf_state_t))

	/* CRC used to validate the retained data.  This must be
	 * stored little-endian, and covers everything up to but not
	 * including this field.
	 */
	uint32_t crc;
};

/* For simplicity in the sample just allow anybody to see and
 * manipulate the retained state.
 */
extern struct retained_data retained;

/* Check whether the retained data is valid, and if not reset it.
 *
 * @return true if and only if the data was valid and reflects state
 * from previous sessions.
 */
bool retained_validate(void);

/* Update any generic retained state and recalculate its checksum so
 * subsequent boots can verify the retained state.
 */
void retained_update(void);

#endif /* RETAINED_H_ */
