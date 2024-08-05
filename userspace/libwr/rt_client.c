/*
 * Example mini-ipc client
 *
 * Copyright (C) 2011 CERN (www.cern.ch)
 * Author: Alessandro Rubini <rubini@gnudd.com>
 *
 * Released in the public domain
 */

/*
 * This file must *only* be used by wrsw_hal, even if it lives in libwr
 * Only wr_phytool uses it, for strange hacking, but it interferes with
 * hal. Nobody uses phytool in production, so that's ok. I'll add some
 * check, maybe, someday.
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/time.h>

#include "minipc.h"

#define RTIPC_EXPORT_STRUCTURES
#include <rt_ipc.h>

#define RTS_MAILBOX_ADDR "mem:10007000"

#define RTS_TIMEOUT 200 /* ms */

static struct minipc_ch *client;

// #define VERBOSE
#include <libwr/wrs-msg.h>

/* Queries the RT CPU PLL state */
int rts_get_state(struct rts_pll_state *state)
{
	int ret = minipc_call(client, RTS_TIMEOUT,
				 &rtipc_rts_get_state_struct, state);

	if (ret < 0)
		return ret;

#ifdef VERBOSE
	{
		int i;

		pr_info("RTS State Dump:\n");
		pr_info("CurrentRef: %d Mode: %d Flags: %x\n",
			   state->current_ref, state->mode, state->flags);
		i = 17;
		pr_info("Ch%d: setpoint: %dps current: %dps "
			 "loopback: %dps flags: %x\n", i,
			 state->channels[i].phase_setpoint,
			 state->channels[i].phase_current,
			 state->channels[i].phase_loopback,
			 state->channels[i].flags);
	}
#endif
	return 0;
}

/* Sets the RT subsystem mode (Boundary Clock or Grandmaster) */
int rts_set_mode(int mode)
{
	int rval;
	int ret = minipc_call(client, RTS_TIMEOUT,
			      &rtipc_rts_set_mode_struct, &rval, mode);

	if (ret < 0)
		return ret;

	return rval;
}

int rts_ptracker_set_average_samples(int channel, int avg_samples)
{
	printf("Setting navg @ port %d (wri%d) navg = %d\n,", channel + 1, channel + 1,avg_samples);

	int rval;
	int ret = minipc_call(client, RTS_TIMEOUT,
			      &rtipc_rts_set_average_samples_struct, &rval,channel, avg_samples);

	if (ret < 0)
		return ret;

	return rval;
}


/* Sets the phase setpoint on a given channel */
int rts_adjust_phase(int channel, int32_t phase_setpoint)
{
	int rval;
	int ret = minipc_call(client, RTS_TIMEOUT,
			      &rtipc_rts_adjust_phase_struct, &rval,
			      channel, phase_setpoint);

	if (ret < 0)
		return ret;

	return rval;
}

/* Reference channel configuration (BC mode only) */
int rts_lock_channel(int channel, int priority)
{
	int rval;
	int ret = minipc_call(client, RTS_TIMEOUT,
			      &rtipc_rts_lock_channel_struct, &rval,
			      channel, priority);

	if (ret < 0)
		return ret;

	return rval;
}

int rts_enable_ptracker(int channel, int enable)
{
	int rval;
	int ret = minipc_call(client, RTS_TIMEOUT,
			      &rtipc_rts_enable_ptracker_struct, &rval,
			      channel, enable);

	if (ret < 0)
		return ret;

	return rval;
}
int rts_enable_ptracker_log(int channel, int enable)
{
	int rval;
	int ret = minipc_call(client, RTS_TIMEOUT,
			      &rtipc_rts_enable_ptracker_rge_struct, &rval,
			      channel, enable);

	if (ret < 0)
		return ret;

	return rval;
}

int rts_debug_command(int command, int value)
{
	int rval;
	int ret = minipc_call(client, RTS_TIMEOUT,
			      &rtipc_rts_debug_command_struct, &rval,
			      command, value);

	if (ret < 0)
		return ret;

	return rval;
}

int rts_connect(char *logfilename)
{
	static FILE *f;

	minipc_set_poll(25000); /* 25ms, default is 10ms */
	client = minipc_client_create(RTS_MAILBOX_ADDR, 0 /* not verbose */ );
	if (!client)
		return -1;
	if (!f && logfilename) {

		f = fopen(logfilename, "a");
		if (!f) /* ignore error for logs */
			return 0;
		setvbuf(f, NULL, _IONBF, 0);
		minipc_set_logfile(client, f);
	}
	return 0;
}
