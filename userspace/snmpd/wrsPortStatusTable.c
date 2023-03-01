#include "wrsSnmp.h"
#include "snmp_shmem.h"
#include "wrsPortStatusTable.h"

/* Our data: per-port information */
struct wrsPortStatusTable_s wrsPortStatusTable_array[WRS_N_PORTS];

static char *slog_obj_name;
static char *wrsPortStatusSfpError_str = "wrsPortStatusSfpError";

static struct pickinfo wrsPortStatusTable_pickinfo[] = {
	FIELD(wrsPortStatusTable_s, ASN_UNSIGNED, index), /* not reported */
	FIELD(wrsPortStatusTable_s, ASN_OCTET_STR, wrsPortStatusPortName),
	FIELD(wrsPortStatusTable_s, ASN_INTEGER, wrsPortStatusLink),
	FIELD(wrsPortStatusTable_s, ASN_INTEGER, wrsPortStatusConfiguredMode),
	FIELD(wrsPortStatusTable_s, ASN_INTEGER, wrsPortStatusLocked),
	FIELD(wrsPortStatusTable_s, ASN_OCTET_STR, wrsPortStatusPeer_obsolete),
	FIELD(wrsPortStatusTable_s, ASN_OCTET_STR, wrsPortStatusSfpVN),
	FIELD(wrsPortStatusTable_s, ASN_OCTET_STR, wrsPortStatusSfpPN),
	FIELD(wrsPortStatusTable_s, ASN_OCTET_STR, wrsPortStatusSfpVS),
	FIELD(wrsPortStatusTable_s, ASN_INTEGER, wrsPortStatusSfpInDB),
	FIELD(wrsPortStatusTable_s, ASN_INTEGER, wrsPortStatusSfpGbE),
	FIELD(wrsPortStatusTable_s, ASN_INTEGER, wrsPortStatusSfpError),
	FIELD(wrsPortStatusTable_s, ASN_COUNTER, wrsPortStatusPtpTxFrames),
	FIELD(wrsPortStatusTable_s, ASN_COUNTER, wrsPortStatusPtpRxFrames),
	FIELD(wrsPortStatusTable_s, ASN_INTEGER, wrsPortStatusMonitor),
	FIELD(wrsPortStatusTable_s, ASN_INTEGER, wrsPortStatusSfpDom),
	FIELD(wrsPortStatusTable_s, ASN_INTEGER, wrsPortStatusSfpTemp),
	FIELD(wrsPortStatusTable_s, ASN_INTEGER, wrsPortStatusSfpVcc),
	FIELD(wrsPortStatusTable_s, ASN_INTEGER, wrsPortStatusSfpTxBias),
	FIELD(wrsPortStatusTable_s, ASN_INTEGER, wrsPortStatusSfpTxPower),
	FIELD(wrsPortStatusTable_s, ASN_INTEGER, wrsPortStatusSfpRxPower),
	FIELD(wrsPortStatusTable_s, ASN_INTEGER, wrsPortStatusT24p),
	FIELD(wrsPortStatusTable_s, ASN_INTEGER, wrsPortStatusT24pValid),
};


time_t wrsPortStatusTable_data_fill(unsigned int *n_rows)
{
	unsigned ii, i, ppi_i;
	unsigned retries = 0;
	static time_t time_update;
	time_t time_cur;
	static int n_rows_local = 0;

	/* number of rows does not change for wrsPortStatusTable */
	if (n_rows)
		*n_rows = n_rows_local;

	time_cur = get_monotonic_sec();
	if (time_update
	    && time_cur - time_update < WRSPORTSTATUSTABLE_CACHE_TIMEOUT) {
		/* cache not updated, return last update time */
		return time_update;
	}
	time_update = time_cur;

	struct hal_port_state *port_state;
	memset(wrsPortStatusTable_array, 0, sizeof(wrsPortStatusTable_array));

	/* check whether shmem is available */
	if (!shmem_ready_hald()) {
		/* there was an update, return current time */
		snmp_log(LOG_ERR,  "SNMP: " SL_ER
			"%s: Unable to read HAL shmem\n", __func__);
		n_rows_local = 0;
		return time_cur;
	} else {
		n_rows_local = WRS_N_PORTS;
	}

	if (n_rows)
		*n_rows = n_rows_local;

	/* read data, with the sequential lock to have all data consistent */
	while (1) {
		ii = wrs_shm_seqbegin(hal_head);
		for (i = 0; i < hal_nports_local; ++i) {
			int mainPortState;
			struct wrsPortStatusTable_s *wrsPortStatusTable=&wrsPortStatusTable_array[i];

			/* Assume that number of ports does not change between
			 * reads */
			snprintf(wrsPortStatusTable->wrsPortStatusPortName, 10,
				 "wri%d", i + 1);
			port_state = hal_lookup_port(hal_ports,
					hal_nports_local,
					wrsPortStatusTable->wrsPortStatusPortName);
			if(!port_state) {
				/* It looks like we're in strange situation
				 * that HAL is up but hal_ports is not filled
				 */
				continue;
			}

			wrsPortStatusTable->wrsPortStatusMonitor =
							port_state->monitor;

			/* wrsPtpT24p */
			wrsPortStatusTable->wrsPortStatusT24p = port_state->t2_phase_transition;
			wrsPortStatusTable->wrsPortStatusT24pValid = port_state->t24p_from_config;

			/* No need to copy all ports structures, only what
			 * we're interested in.
			 * Keep value 0 for Not available
			 * values defined as WRS_PORT_STATUS_LINK_*
			*/
			wrsPortStatusTable->wrsPortStatusLink =
					1 + state_up(port_state);
			mainPortState=get_port_state(port_state);
			if (mainPortState== HAL_PORT_STATE_DISABLED ||
					mainPortState== HAL_PORT_STATE_INIT) {
				wrsPortStatusTable->wrsPortStatusSfpError =
					  WRS_PORT_STATUS_SFP_ERROR_PORT_DOWN;
				/* if port is in initialization state or disabled don't fill
				 * other fields */
				continue;
			}
			/* Keep value 0 for Not available */
			wrsPortStatusTable->wrsPortStatusLocked =
							1 + port_state->locked;
			if (port_state->sfpPresent && (port_state->calib.sfp.flags & SFP_FLAG_IN_DB)) {
				wrsPortStatusTable->wrsPortStatusSfpInDB =
					WRS_PORT_STATUS_SFP_IN_DB_IN_DATA_BASE;
			} else if (port_state->sfpPresent) {
				wrsPortStatusTable->wrsPortStatusSfpInDB =
					WRS_PORT_STATUS_SFP_IN_DB_NOT_IN_DATA_BASE;
			}
			if (port_state->sfpPresent && (port_state->calib.sfp.flags & SFP_FLAG_1GbE)) {
				wrsPortStatusTable->wrsPortStatusSfpGbE =
					WRS_PORT_STATUS_SFP_GBE_LINK_GBE;
			} else if (port_state->sfpPresent) {
				wrsPortStatusTable->wrsPortStatusSfpGbE =
					WRS_PORT_STATUS_SFP_GBE_LINK_NOT_GBE;
			}
			strncpy(wrsPortStatusTable->wrsPortStatusSfpVN,
				port_state->calib.sfp.vendor_name,
				sizeof(wrsPortStatusTable->wrsPortStatusSfpVN));
			strncpy(wrsPortStatusTable->wrsPortStatusSfpPN,
				port_state->calib.sfp.part_num,
				sizeof(wrsPortStatusTable->wrsPortStatusSfpPN));
			strncpy(wrsPortStatusTable->wrsPortStatusSfpVS,
				port_state->calib.sfp.vendor_serial,
				sizeof(wrsPortStatusTable->wrsPortStatusSfpVS));

			/* Copy DOM data for SFP */
			if (hal_shmem->read_sfp_diag) {
				if (port_state->has_sfp_diag) {
					wrsPortStatusTable->wrsPortStatusSfpDom = WRS_PORT_STATUS_SFP_DOM_ENABLE;
					/* temp in C */
					wrsPortStatusTable->wrsPortStatusSfpTemp = ntohs(*port_state->calib.sfp_dom_raw.temp)/256;
					/* vcc in mV */
					wrsPortStatusTable->wrsPortStatusSfpVcc = ntohs(*port_state->calib.sfp_dom_raw.vcc)/10;
					/* tx_bias in uA */
					wrsPortStatusTable->wrsPortStatusSfpTxBias = ntohs(*port_state->calib.sfp_dom_raw.tx_bias)*2;
					/* tx_pow in uW */
					wrsPortStatusTable->wrsPortStatusSfpTxPower = ntohs(*port_state->calib.sfp_dom_raw.tx_pow)/10;
					/* rx_pow in uW */
					wrsPortStatusTable->wrsPortStatusSfpRxPower = ntohs(*port_state->calib.sfp_dom_raw.rx_pow)/10;
				} else {
					wrsPortStatusTable->wrsPortStatusSfpDom = WRS_PORT_STATUS_SFP_DOM_NOT_SUPPORTED;
				}
			} else {
				wrsPortStatusTable->wrsPortStatusSfpDom = WRS_PORT_STATUS_SFP_DOM_DISABLE;
			}
		}

		retries++;
		if (retries > 100) {
			snmp_log(LOG_ERR,  "SNMP: " SL_ER
				"%s: Unable to read HAL, too many retries\n",
				 __func__);
			retries = 0;
			}
		if (!wrs_shm_seqretry(hal_head, ii))
			break; /* consistent read */
		usleep(1000);
	}

	/* keep the log printouts outside wrs_shm_seqbegin and wrs_shm_seqretry
	 * since it introduces delays and might lead to an infinite loop of
	 * retries */
	slog_obj_name = wrsPortStatusSfpError_str;
	for (i = 0; i < hal_nports_local; ++i) {
		struct wrsPortStatusTable_s *wrsPortStatusTable=&wrsPortStatusTable_array[i];

		/* If info about wrsPortStatusSfpGbE is not filled skip further
		 * checking. NOTE: there is no need to check the fill of others
		 * like:
		 * - wrsPortStatusSfpInDB
		 */
		if (wrsPortStatusTable->wrsPortStatusSfpGbE == 0) {
			/* if this is not filled, it means SFP is not plugged,
			 * so there is no error on that port */
			wrsPortStatusTable->wrsPortStatusSfpError = WRS_PORT_STATUS_SFP_ERROR_SFP_OK;
			continue;
		}
		/* Don't check if WRS_PORT_STATUS_SFP_ERROR_PORT_DOWN */
		if(wrsPortStatusTable->wrsPortStatusSfpError == WRS_PORT_STATUS_SFP_ERROR_PORT_DOWN) {
			continue;
		}

		/* sfp error when SFP is not 1 GbE or
		  * (port is not "non-wr", "none" mode and sfp not in data base)
		  * port down, is set above
		  * (WRS_PORT_STATUS_SFP_ERROR_PORT_DOWN) */
		wrsPortStatusTable->wrsPortStatusSfpError = WRS_PORT_STATUS_SFP_ERROR_SFP_OK;
		
		snmp_log(LOG_DEBUG, "SNMP: " SL_DEBUG
			" reading ports name %s link %d, "
			"locked %d\n",
			wrsPortStatusTable->wrsPortStatusPortName,
			wrsPortStatusTable->wrsPortStatusLink,
			wrsPortStatusTable->wrsPortStatusLocked);
		
		if (wrsPortStatusTable->wrsPortStatusMonitor == WRS_PORT_STATUS_MONITOR_DISABLE)
		{
			snmp_log(LOG_DEBUG, "SNMP: " SL_DEBUG " ignoring any "
				"problems on port %s as monitoring is disabled\n",
				wrsPortStatusTable->wrsPortStatusPortName);
			continue;
		}
		if (wrsPortStatusTable->wrsPortStatusSfpGbE == WRS_PORT_STATUS_SFP_GBE_LINK_NOT_GBE) {
			/* error, SFP is not 1 GbE */
			wrsPortStatusTable->wrsPortStatusSfpError = WRS_PORT_STATUS_SFP_ERROR_SFP_ERROR;
			snmp_log(LOG_ERR, "SNMP: " SL_ER  " %s: "
				  "SFP in port %d (wri%d) is not for Gigabit Ethernet\n",
				  slog_obj_name, i + 1, i + 1);
		}
		if (wrsPortStatusTable->wrsPortStatusSfpInDB == WRS_PORT_STATUS_SFP_IN_DB_NOT_IN_DATA_BASE) {
			/* error, port is not non-wr mode and sfp not in data base */
			wrsPortStatusTable->wrsPortStatusSfpError = WRS_PORT_STATUS_SFP_ERROR_SFP_ERROR;
			snmp_log(LOG_ERR, "SNMP: " SL_ER  " %s: "
				  "SFP in port %d (wri%d) is not in the database. "
				  "Change the SFP or declare port as non-wr or none\n",
				  slog_obj_name, i + 1, i + 1);
		}

	}

	retries = 0;
	/* check whether shmem is available */
	if (!shmem_ready_ppsi()) {
		/* there was an update, return current time */
		snmp_log(LOG_ERR,   "SNMP: " SL_ER
                        "%s: Unable to read PPSI shmem\n",
			 __func__);
		return time_cur;
	}

	/* fill wrsPortStatusPtpTxFrames and wrsPortStatusPtpRxFrames
	 * ptp_tx_count and ptp_rx_count statistics in PPSI are collected per
	 * ppi instance. Since there can be more than one instance per physical
	 * port, proper counters has to be added. */
	while (1) {
		ii = wrs_shm_seqbegin(ppsi_head);
		/* Match port name with interface name of ppsi instance.
		 * More than one ppsi_iface_name can match to
		 * wrsPortStatusTable_array[i].wrsPortStatusPortName, but only one can
		 * match way round */
		for (ppi_i = 0; ppi_i < *ppsi_ppi_nlinks; ppi_i++) {
			/* (ppsi_ppi + ppi_i)->iface_name is a pointer in
			 * shmem, so we have to follow it
			 * NOTE: ppi->cfg.port_name cannot be used instead,
			 * because it is not used when ppsi is configured from
			 * cmdline */
			struct pp_instance *ppi=ppsi_ppi + ppi_i;
			portDS_t *portDS = (portDS_t *) wrs_shm_follow(ppsi_head,ppi->portDS);
			char *ppsi_iface_name = (char *) wrs_shm_follow(ppsi_head,ppi->iface_name);

			for (i = 0; i < hal_nports_local; ++i) {
				struct wrsPortStatusTable_s *wrsPortStatusTable=&wrsPortStatusTable_array[i];


				if (!strncmp(wrsPortStatusTable->wrsPortStatusPortName,
					     ppsi_iface_name, 12)) {
					int configuredMode=WRS_PORT_STATUS_CONFIGURED_MODE_UNKNOWN;

					wrsPortStatusTable->wrsPortStatusPtpTxFrames +=
							ppi->ptp_tx_count;
					wrsPortStatusTable->wrsPortStatusPtpRxFrames +=
							ppi->ptp_rx_count;

					/* Update wrsPortStatusConfiguredMode */
					if ( ppi->protocol_extension==PPSI_EXT_WR ) {
						if ( wrsPortStatusTable->wrsPortStatusMonitor == WRS_PORT_STATUS_MONITOR_DISABLE) {
							configuredMode=WRS_PORT_STATUS_CONFIGURED_MODE_NON_WR;
						} else {
							if ( (ppsi_defaultDS->externalPortConfigurationEnabled &&
								  ppi->externalPortConfigurationPortDS.desiredState==PPS_MASTER)
								|| portDS->masterOnly) {
								// MASTER
								configuredMode=WRS_PORT_STATUS_CONFIGURED_MODE_MASTER;
							} else if ( (ppsi_defaultDS->externalPortConfigurationEnabled &&
							ppi->externalPortConfigurationPortDS.desiredState==PPS_SLAVE)
								|| ppsi_defaultDS->slaveOnly) {
								// SLAVE
								configuredMode=WRS_PORT_STATUS_CONFIGURED_MODE_SLAVE;
							} else if ( (ppsi_defaultDS->externalPortConfigurationEnabled &&
							(ppi->externalPortConfigurationPortDS.desiredState==PPS_TIMESCALE_SLAVE ))
								|| ppsi_defaultDS->slaveOnly) {
								// SLAVE
								configuredMode=WRS_PORT_STATUS_CONFIGURED_MODE_SLAVE;
							}else if ( !ppsi_defaultDS->externalPortConfigurationEnabled &&
									!portDS->masterOnly &&
									!ppsi_defaultDS->slaveOnly ) {
								// AUTO
								configuredMode=WRS_PORT_STATUS_CONFIGURED_MODE_AUTO;
							}
						}
					} else {
						configuredMode=WRS_PORT_STATUS_CONFIGURED_MODE_NONE;
					}
					wrsPortStatusTable->wrsPortStatusConfiguredMode=configuredMode;
					/* speed up a little, break here */
					break;
				}
			}
		}
		retries++;
		if (retries > 100) {
			snmp_log(LOG_ERR, "SNMP: " SL_ER
				 "%s: Unable to read PPSI, too many retries\n",
				 __func__);
			retries = 0;
			break;
			}
		if (!wrs_shm_seqretry(ppsi_head, ii))
			break; /* consistent read */
		usleep(1000);
	}

	/* there was an update, return current time */
	return time_cur;
}

#define TT_OID WRSPORTSTATUSTABLE_OID
#define TT_PICKINFO wrsPortStatusTable_pickinfo
#define TT_DATA_FILL_FUNC wrsPortStatusTable_data_fill
#define TT_DATA_ARRAY wrsPortStatusTable_array
#define TT_GROUP_NAME "wrsPortStatusTable"
#define TT_INIT_FUNC init_wrsPortStatusTable
#define TT_CACHE_TIMEOUT WRSPORTSTATUSTABLE_CACHE_TIMEOUT


#include "wrsTableTemplate.h"
