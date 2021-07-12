// Copyright 2013, Fredrik Hultin.
// Copyright 2013, Jakob Bornecrantz.
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Functions imported ad-hoc for OpenHMD compilation */

#include "openhmdi.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

uint64_t ohmd_monotonic_per_sec(ohmd_context* ctx)
{
	return ctx->monotonic_ticks_per_sec;
}

/*
 * Grabbed from druntime, good thing it's BOOST v1.0 as well.
 */
uint64_t ohmd_monotonic_conv(uint64_t ticks, uint64_t srcTicksPerSecond, uint64_t dstTicksPerSecond)
{
	// This would be more straightforward with floating point arithmetic,
	// but we avoid it here in order to avoid the rounding errors that that
	// introduces. Also, by splitting out the units in this way, we're able
	// to deal with much larger values before running into problems with
	// integer overflow.
	return ticks / srcTicksPerSecond * dstTicksPerSecond +
		ticks % srcTicksPerSecond * dstTicksPerSecond / srcTicksPerSecond;
}

/* Dummy OpenHMD functions */
void* ohmd_allocfn(ohmd_context* ctx, const char* e_msg, size_t size)
{
    (void)(ctx);
    (void)(e_msg);

    return malloc (size);
}

