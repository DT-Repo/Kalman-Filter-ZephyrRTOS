/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "../util/data.c"

int main(void)
{
	printf("Kalman filter START\n");
	while (1)
	{
		fusion_demo(&kalm_drv);
	}
	return 0;
}
