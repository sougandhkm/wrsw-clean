#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <unistd.h>
#include <time.h>
#include <sys/timex.h>
#include <inttypes.h>
#include <ppsi/ppsi.h>
#include <libwr/shmem.h>
#include <libwr/hal_shmem.h>
#include <libwr/switch_hw.h>
#include <libwr/wrs-msg.h>
#include <libwr/pps_gen.h>
#include "../../kernel/wbgen-regs/ppsg-regs.h"
#include <fpga_io.h>
#include <minipc.h>
#include <signal.h>
#include <ppsi-wrs.h>


#include "term.h"
#include <time_lib.h>

#define PTP_EXPORT_STRUCTURES
#include "ptpd_exports.h"

#define SHOW_GUI		0
#define SHOW_SLAVE_PORTS	1
#define SHOW_MASTER_PORTS	(1<<1)
#define SHOW_OTHER_PORTS	(1<<2) /* non-wr and none */
#define SHOW_SERVO		(1<<3)
#define SHOW_TEMPERATURES	(1<<4)
#define WEB_INTERFACE		(1<<5)
#define SHOW_WR_TIME		(1<<6)

/* for convenience when any or all ports needs a print statement */
#define SHOW_ALL_PORTS		(SHOW_SLAVE_PORTS | SHOW_MASTER_PORTS | \
				SHOW_OTHER_PORTS)
/* for convenience with -a option */
#define SHOW_ALL		(SHOW_ALL_PORTS | SHOW_SERVO | \
				SHOW_TEMPERATURES | SHOW_WR_TIME)


#define MAX_INST_SERVO 18

struct inst_servo_t {
	struct pp_instance * ppi;              /* pointer to the ppi instance */
	int                  valid_servo_data; /* 1 means servo data are vaild */
	TimeInterval         offsetFromMaster; /* currentDS.offsetFromMaster */
	TimeInterval         meanDelay;        /* currentDS.meanDelay */
	TimeInterval         delayAsymmetry;   /* portDS.delayAsymmetry */
	RelativeDifference	 scaledDelayCoefficient; /* AsymmetryCorrectionPortDS.scaledDelayCoefficient */
	TimeInterval         egressLatency;    /* timestampCorrectionPortDS.egressLatency */
	TimeInterval         ingressLatency;   /* timestampCorrectionPortDS.ingressLatency */
	TimeInterval         semistaticLatency;/* timestampCorrectionPortDS.semistaticLatency */
	TimeInterval	     constantAsymmetry;/* asymmetryCorrectionPortDS.constantAsymmetry */
	struct pp_servo      servo_snapshot;  /* image of a the ppsi servo */
	void *               servo_ext_snapshot; /* image of the extension servo */
};

/* protocol extension data */
#define IS_PROTO_EXT_INFO_AVAILABLE( proto_id ) \
	((proto_id < sizeof(proto_ext_info)/sizeof(struct proto_ext_info_t)) \
			&& (proto_ext_info[proto_id].valid==1))

struct proto_ext_info_t {
	int valid;
	char *ext_name; /* Extension name */
	char short_ext_name; /* Very short extension name - just one character */
	int servo_ext_size; /* Size of the extension */
	int ipc_cmd_tacking; /* Command to enable/disable servo tacking*/
	int track_onoff;     /* Tracking on/off */
	time_t lastt;
	int last_count;
};

static struct proto_ext_info_t proto_ext_info [] = {
		[PPSI_EXT_NONE] = {
				.valid=1,
				.ext_name="PTP",
				.short_ext_name='P',
				.ipc_cmd_tacking=-1, /* Invalid */
		},

#if CONFIG_HAS_EXT_WR
		[PPSI_EXT_WR] = {
				.valid=1,
				.ext_name="White-Rabbit",
				.short_ext_name='W',
				.servo_ext_size=sizeof(struct wr_data),
				.ipc_cmd_tacking=PTPDEXP_COMMAND_WR_TRACKING,
				.track_onoff = 1,
		},
#endif
#if CONFIG_HAS_EXT_L1SYNC
		[PPSI_EXT_L1S] = {
				.valid=1,
				.ext_name="L1Sync",
				.short_ext_name='L',
				.servo_ext_size=sizeof(struct l1e_data),
				.ipc_cmd_tacking=PTPDEXP_COMMAND_L1SYNC_TRACKING,
				.track_onoff = 1,
		},
#endif
#if CONFIG_HAS_EXT_CUSTOM
		[PPSI_EXT_CUSTOM] = {
				.valid=1,
				.ext_name="Custom",
				.short_ext_name='C',
				.servo_ext_size=sizeof(struct l1e_data),
				.ipc_cmd_tacking=PTPDEXP_COMMAND_L1SYNC_TRACKING,
				.track_onoff = 1,
		},
#endif

};

typedef struct {
	struct pp_instance *ppi;
	portDS_t *portDS;
}pp_instance_ptr_t;

static pp_instance_ptr_t instances[PP_MAX_LINKS];


static struct inst_servo_t servos[MAX_INST_SERVO];

int mode = SHOW_GUI;
Boolean showCal = 0;
static struct minipc_ch *ptp_ch;

static struct wrs_shm_head *hal_head;
struct hal_port_state *hal_ports;
/* local copy of port state */
static struct hal_port_state hal_ports_local_copy[HAL_MAX_PORTS];
static int hal_nports_local;
static struct wrs_shm_head *ppsi_head;
static struct pp_globals *ppg;
static void *ppg_arch;
static 	defaultDS_t *defaultDS;
static pid_t ptp_ch_pid; /* pid of ppsi connected via minipc */
static struct hal_temp_sensors *temp_sensors;
/* local copy of temperature sensor readings */
static struct hal_temp_sensors temp_sensors_local;

static uint64_t seconds;
static uint32_t nanoseconds;
/* ignore checking if process is alive */
static int ignore_alive;

/* define size of pp_instance_state_to_name as a last element + 1 */
#define PP_INSTANCE_STATE_MAX (sizeof(pp_instance_state_to_name)/sizeof(char *))

/* define conversion array for the field state in the struct pp_instance */
static char *pp_instance_state_to_name[] = {
	/* from ppsi/include/ppsi/ieee1588_types.h, enum pp_std_states */
	/* PPS_END_OF_TABLE = 0 */
	[PPS_END_OF_TABLE] =      "EOT       ",
	[PPS_INITIALIZING] =      "INITING   ",
	[PPS_FAULTY] =            "FAULTY    ",
	[PPS_DISABLED] =          "DISABLED  ",
	[PPS_LISTENING] =         "LISTENING ",
	[PPS_PRE_MASTER] =        "PRE_MASTER",
	[PPS_MASTER] =            "MASTER    ",
	[PPS_PASSIVE] =           "PASSIVE   ",
	[PPS_UNCALIBRATED] =      "UNCALIBRAT",
	[PPS_SLAVE] =             "SLAVE     ",
	NULL
	};

#define EMPTY_EXTENSION_STATE_NAME "          "

#if CONFIG_HAS_EXT_L1SYNC
static char * l1e_instance_extension_state[]={
		[__L1SYNC_MISSING   ] = "INVALID   ",
		[L1SYNC_DISABLED    ] = "DISABLED  ",
		[L1SYNC_IDLE        ] = "IDLE      ",
		[L1SYNC_LINK_ALIVE  ] = "LINK ALIVE",
		[L1SYNC_CONFIG_MATCH] = "CFG MATCH ",
		[L1SYNC_UP          ] = "UP        ",
		NULL
};
#define L1S_INSTANCE_EXTENSION_STATE_MAX (sizeof (l1e_instance_extension_state)/sizeof(char *) )

#endif

static char * timind_mode_state[] = {
		[WRH_TM_GRAND_MASTER]=     "GM",
		[WRH_TM_FREE_MASTER]=      "FR",
		[WRH_TM_BOUNDARY_CLOCK]=   "BC",
		[WRH_TM_DISABLED]=         "--",
		NULL
	};

static char * pll_locking_state[] = {
		[WRH_TM_LOCKING_STATE_NONE]=     "NONE    ",
		[WRH_TM_LOCKING_STATE_LOCKING]=  "LOCKING ",
		[WRH_TM_LOCKING_STATE_LOCKED]=   "LOCKED  ",
		[WRH_TM_LOCKING_STATE_HOLDOVER]= "HOLDOVER",
		[WRH_TM_LOCKING_STATE_ERROR]=    "ERROR   ",
		NULL
	};

#if CONFIG_HAS_EXT_WR
static char * wr_instance_extension_state[]={
		[WRS_IDLE ]=              "IDLE      ",
		[WRS_PRESENT] =           "WR_PRESENT",
		[WRS_S_LOCK] =            "WR_S_LOCK ",
		[WRS_M_LOCK] =            "WR_M_LOCK ",
		[WRS_LOCKED] =            "WR_LOCKED ",
		[WRS_CALIBRATION] =       "WR_CAL-ION",
		[WRS_CALIBRATED] =        "WR_CAL-ED ",
		[WRS_RESP_CALIB_REQ] =    "WR_RSP_CAL",
		[WRS_WR_LINK_ON] =        "WR_LINK_ON",
		NULL
};
#define WR_INSTANCE_EXTENSION_STATE_MAX (sizeof (wr_instance_extension_state)/sizeof(char *) )

#endif

static char *prot_detection_state_name[]={
		"NONE   ", /* No meaning. No extension present */
		"WA_MSG ", /* Waiting first message */
		"PD_IPRG", /* Protocol detection  */
		"EXT_ON ", /* Protocol detected */
		"EXT_OFF" /* Protocol not detected */
};

/* prototypes */
int read_instances(void);

static inline int extensionStateColor( struct pp_instance *ppi) {
	if ( ppi->protocol_extension==PPSI_EXT_NONE) {
		return C_GREEN; /* No extension */
	}
	switch (ppi->extState) {
	case PP_EXSTATE_ACTIVE :
		return C_GREEN;
	case PP_EXSTATE_PTP :
		return C_WHITE;
	case PP_EXSTATE_DISABLE :
	default:
		return C_RED;
	}
}

char *getStateAsString(char *p[], int index) {
	int i,len;
	char *errMsg="?????????????????????";

	len=strlen(p[0]);
	for (i=0; ;i++) {
		if ( p[i]==NULL )
			return errMsg+strlen(errMsg)-len;
		if ( i==index)
			return p[index];
	}
}

int64_t interval_to_picos(TimeInterval interval)
{
	return (interval * 1000) >>  TIME_INTERVAL_FRACBITS;
}

int64_t pp_time_to_picos(struct pp_time *ts)
{
	return ts->secs * PP_NSEC_PER_SEC
		+ ((ts->scaled_nsecs * 1000 + 0x8000) >> TIME_INTERVAL_FRACBITS);
}

TimeInterval pp_time_to_interval (struct pp_time *pptime) {
	return pptime->scaled_nsecs;
}

char *optimized_pp_time_toString(struct pp_time *pptime, char *buf ) {
	char lbuf[128];

	if ( pptime->secs )
		sprintf(buf,"%16s sec",timeToString(pptime,lbuf));
	else
		sprintf(buf,"%16s nsec",timeIntervalToString(pp_time_to_interval(pptime),lbuf));
	return buf;
}

#if 0
static double alpha_to_double(int32_t alpha) {
  double f ;
  int neg = alpha<0;

  if(neg) alpha= ~alpha+1;
  f= (double)alpha/(double)(1LL<<FIX_ALPHA_FRACBITS);
  return  neg ? -f : f;
}
#endif

void help(char *prgname)
{
	fprintf(stderr, "%s: Use: \"%s [<options>] <cmd> [<args>]\n",
		prgname, prgname);
	fprintf(stderr,
		"  The program has the following options\n"
		"  -h   print help\n"
		"  -i   show White Rabbit time.\n"
		"	   very close\n"
		"  -m   show master ports\n"
		"  -s   show slave ports\n"
		"  -o   show other ports\n"
		"  -e   show servo statistics\n"
		"  -t   show temperatures\n"
		"  -a   show all (same as -i -m -s -o -e -t options)\n"
		"  -b   black and white output\n"
		"  -w   web interface mode\n"
		"  -c   show calibration values on GUI"
		"  -H <dir> Open shmem dumps from the given directory\n"
		"\n"
		"During execution the user can enter 'q' to exit the program\n"
		"and 't' to toggle printing of state information on/off\n");
	exit(1);
}

int read_hal(void){
	unsigned ii;
	unsigned retries = 0;

	/* read data, with the sequential lock to have all data consistent */
	while (1) {
		ii = wrs_shm_seqbegin(hal_head);
		memcpy(hal_ports_local_copy, hal_ports,
		       hal_nports_local*sizeof(struct hal_port_state));
		memcpy(&temp_sensors_local, temp_sensors,
		       sizeof(*temp_sensors));
		retries++;
		if (retries > 100)
			return -1;
		if (!wrs_shm_seqretry(hal_head, ii))
			break; /* consistent read */
		usleep(1000);
	}

	return 0;
}

int read_instances(void) {
	struct pp_instance *pp_array;
	int l;

	bzero(instances,sizeof(instances));

	if ( !(pp_array = wrs_shm_follow(ppsi_head, ppg->pp_instances)) )
		return -1;

	for (l = 0; l < ppg->nlinks; l++) {
		instances[l].ppi=&pp_array[l];
		if ( ! (instances[l].portDS=wrs_shm_follow(ppsi_head, instances[l].ppi->portDS)) )
			return -1;
	}
	return 0;
}

int read_servo(void){

	unsigned int i, servoIdx;


	/* Clear servo structure */
	for (i=0; i<MAX_INST_SERVO; i++) {
		if (servos[i].ppi ) {
			if ( servos[i].servo_ext_snapshot )
				free(servos[i].servo_ext_snapshot);
		}
	}
	bzero(&servos, sizeof(servos));

	servoIdx=0;
	for (i = 0; i < ppg->nlinks; i++) {
		struct pp_instance *ppi = instances[i].ppi;

		/* we are only interested  on instances in SLAVE state */
		if (ppi->state == PPS_SLAVE ) {
			struct inst_servo_t *servo=&servos[servoIdx++];
			int alloc_size=IS_PROTO_EXT_INFO_AVAILABLE(ppi->protocol_extension) ?
					proto_ext_info[ppi->protocol_extension].servo_ext_size :
					0;

			/* Allocate extension data memory if needed */
			if ( alloc_size > 0 )
				if ( !(servo->servo_ext_snapshot=malloc(alloc_size)) )
					return -1;

			while (1) {
				unsigned ii = wrs_shm_seqbegin(ppsi_head);
				unsigned retries = 0;
				struct pp_servo *ppsi_servo;

				/* Copy common data */
				if ( !(ppsi_servo = wrs_shm_follow(ppsi_head, ppi->servo)) )
						break;
				memcpy(&servo->servo_snapshot, ppsi_servo, sizeof(struct pp_servo));
				
				servo->meanDelay=pp_time_to_interval(&servo->servo_snapshot.meanDelay);   /* currentDS.meanDelay */
				servo->offsetFromMaster=pp_time_to_interval(&servo->servo_snapshot.offsetFromMaster);/* currentDS.offsetFromMaster */

				/* Copy extension servo data */
				if ( servo->servo_ext_snapshot ) {
					void *ppsi_servo_ext;

					if ( !(ppsi_servo_ext = wrs_shm_follow(ppsi_head, ppi->ext_data)) )
							break;
					memcpy(servo->servo_ext_snapshot, ppsi_servo_ext,alloc_size);
				}

				/* Copy extra interesting data */
				{
					currentDS_t *currenDS;

					if ( !(currenDS = wrs_shm_follow(ppsi_head, ppg->currentDS) ) )
							break;

					//servo->offsetFromMaster=currenDS->offsetFromMaster; /* currentDS.offsetFromMaster */
					//servo->meanDelay=currenDS->meanDelay;    /* currentDS.meanDelay */
				}
				{
					portDS_t *portDS;

					if ( !(portDS = wrs_shm_follow(ppsi_head, ppi->portDS) ) )
							break;
					servo->delayAsymmetry=portDS->delayAsymmetry;   /* portDS.delayAsymmetry */
				}
				servo->scaledDelayCoefficient=ppi->asymmetryCorrectionPortDS.scaledDelayCoefficient; /* AsymmetryCorrectionPortDS.scaledDelayCoefficient */
				servo->constantAsymmetry=ppi->asymmetryCorrectionPortDS.constantAsymmetry;/* asymmetryCorrectionPortDS.constantAsymmetry */
				servo->egressLatency=ppi->timestampCorrectionPortDS.egressLatency;   /* timestampCorrectionPortDS.egressLatency */
				servo->ingressLatency=ppi->timestampCorrectionPortDS.ingressLatency;  /* timestampCorrectionPortDS.ingressLatency */
				servo->semistaticLatency=ppi->timestampCorrectionPortDS.semistaticLatency;  /* timestampCorrectionPortDS.semistaticLatency */
				if (!wrs_shm_seqretry(ppsi_head, ii)) {
					servo->valid_servo_data=1;
					break; /* consistent read */
				}
				retries++;
				if (retries > 100)
					break;
			}
			if ( servo->valid_servo_data ) {
				servo->ppi=ppi;
			} else {
				if ( servo->servo_ext_snapshot ) {
					free (servo->servo_ext_snapshot);
				}
			}
		}
	}
	return 0;
}

void ppsi_connect_minipc(void)
{
	if (ptp_ch) {
		/* close minipc, if connected before */
		minipc_close(ptp_ch);
	}
	ptp_ch = minipc_client_create("ptpd", 0);
	if (!ptp_ch) {
		pr_error("Can't establish WRIPC connection to the PTP "
			 "daemon!\n");
		exit(1);
	}
	/* store pid of ppsi connected via minipc */
	ptp_ch_pid = ppsi_head->pid;
}

void init_shm(void)
{
	struct hal_shmem_header *h;
	int ret;
	int n_wait = 0;
	while ((ret = wrs_shm_get_and_check(wrs_shm_hal, &hal_head)) != 0) {
		n_wait++;
		if (ret == WRS_SHM_OPEN_FAILED) {
			pr_error("Unable to open HAL's shm !\n");
		}
		if (ret == WRS_SHM_WRONG_VERSION) {
			pr_error("Unable to read HAL's version!\n");
		}
		if (ret == WRS_SHM_INCONSISTENT_DATA) {
			pr_error("Unable to read consistent data from HAL's "
				 "shmem!\n");
		}
		if (n_wait > 10) {
			/* timeout! */
			exit(-1);
		}
		sleep(1);
	}

	if (hal_head->version != HAL_SHMEM_VERSION) {
		pr_error("Unknown HAL's shm version %i (known is %i)\n",
			 hal_head->version, HAL_SHMEM_VERSION);
		exit(1);
	}
	h = (void *)hal_head + hal_head->data_off;
	/* Assume number of ports does not change in runtime */
	hal_nports_local = h->nports;
	if (hal_nports_local > HAL_MAX_PORTS) {
		pr_error("Too many ports reported by HAL. %d vs %d "
			 "supported\n", hal_nports_local, HAL_MAX_PORTS);
		exit(1);
	}
	/* Even after HAL restart, HAL will place structures at the same
	 * addresses. No need to re-dereference pointer at each read.
	 */
	hal_ports = wrs_shm_follow(hal_head, h->ports);
	if (!hal_ports) {
		pr_error("Unable to follow hal_ports pointer in HAL's "
			 "shmem\n");
		exit(1);
	}
	temp_sensors = &(h->temp);

	n_wait = 0;
	while ((ret = wrs_shm_get_and_check(wrs_shm_ptp, &ppsi_head)) != 0) {
		n_wait++;
		if (ret == WRS_SHM_OPEN_FAILED) {
			pr_error("Unable to open PPSI's shm !\n");
		}
		if (ret == WRS_SHM_WRONG_VERSION) {
			pr_error("Unable to read PPSI's version!\n");
		}
		if (ret == WRS_SHM_INCONSISTENT_DATA) {
			pr_error("Unable to read consistent data from PPSI's "
				 "shmem!\n");
		}
		if (n_wait > 10) {
			/* timeout! */
			exit(-1);
		}
		sleep(1);
	}

	/* check hal's shm version */
	if (ppsi_head->version != WRS_PPSI_SHMEM_VERSION) {
		pr_error("Unknown PPSI's shm version %i (known is %i)\n",
			 ppsi_head->version, WRS_PPSI_SHMEM_VERSION);
		exit(1);
	}
	ppg = (void *)ppsi_head + ppsi_head->data_off;

	/* Access to ppg arch data */
	if ( ppg->arch_data!=NULL)
		ppg_arch=wrs_shm_follow(ppsi_head, ppg->arch_data);

	/* Access to defaultDS data */
	defaultDS = wrs_shm_follow(ppsi_head, ppg->defaultDS);
	if (!defaultDS) {
		pr_error("Unable to follow defaultDS pointer in PPSI's shmem\n");
		exit(1);
	}

	if ( read_instances()==-1 )
		exit(1);

	ppsi_connect_minipc();
}

static struct desired_state_t{
	char *str_state;
	int state;
} desired_states[] = {
	{ "initializing", PPS_INITIALIZING},
	{ "faulty", PPS_FAULTY},
	{ "disabled", PPS_DISABLED},
	{ "listening", PPS_LISTENING},
	{ "pre-master", PPS_PRE_MASTER},
	{ "master", PPS_MASTER},
	{ "passive", PPS_PASSIVE},
	{ "uncalibrated", PPS_UNCALIBRATED},
	{ "slave", PPS_SLAVE},
	{ "timescale_slave", PPS_TIMESCALE_SLAVE},
	{}
};

void show_ports(int hal_alive, int ppsi_alive)
{
	int i, j;
	uint32_t tai_l,tmp2,nsec;
	struct timeval sw, hw;
	struct timex timex_val;
	struct tm *tm;
	char datestr[32];
	struct hal_port_state *port_state;
	int vlan_i;
	int nvlans;
	int *p;

	struct PPSG_WB *pps=(struct PPSG_WB *)(_fpga_base_virt+FPGA_BASE_PPS_GEN);

	if (!hal_alive) {
		if (mode == SHOW_GUI)
			term_cprintf(C_RED, "HAL is dead!\n");
		else if (mode == SHOW_ALL)
			printf("HAL is dead!\n");
		return;
	}

	if (mode == SHOW_GUI) {
		//First get all the times at the "same instant"
		do {
			tai_l = pps->CNTR_UTCLO;
			nsec = pps->CNTR_NSEC * 16; /* we count a 16.5MHz */
			tmp2 = pps->CNTR_UTCLO;
		} while((tmp2 != tai_l));
		gettimeofday(&sw, NULL);

		hw.tv_usec = nsec/1000;
		hw.tv_sec  = (time_t)(tai_l);

		tm = gmtime(&(hw.tv_sec));
		strftime(datestr, sizeof(datestr), "%Y-%m-%d %H:%M:%S", tm);
		term_cprintf(C_BLUE, "WR time (TAI)    : ");
		term_cprintf(C_WHITE, "%s.%06li", datestr,hw.tv_usec);

		term_cprintf(C_BLUE, "   Leap seconds: ");
		bzero(&timex_val,sizeof(timex_val));
		if (adjtimex(&timex_val) < 0) {
			term_cprintf(C_WHITE, "error\n");
		} else {
			p = (int *)(&timex_val.stbcnt) + 1;
			term_cprintf(C_WHITE, "%3d\n", *p);
		}

		tm = gmtime(&(sw.tv_sec));
		strftime(datestr, sizeof(datestr), "%Y-%m-%d %H:%M:%S", tm);
		term_cprintf(C_BLUE, "Switch time (UTC): ");
		term_cprintf(C_WHITE, "%s.%06li", datestr,sw.tv_usec);

		term_cprintf(C_BLUE, "   TAI-UTC     : ");
		{
			struct timeval diff;
			int neg=0;

			neg=timeval_subtract(&diff, &hw, &sw);
			term_cprintf(C_WHITE, "%c%li.%06li\n",neg?'-':'+',labs(diff.tv_sec),labs(diff.tv_usec));

		}

		if ( ppsi_alive && ppg_arch!=NULL) {
			term_cprintf(C_BLUE, "TimingMode: ");
			term_cprintf(C_WHITE, "%s",getStateAsString(timind_mode_state,((wrs_arch_data_t *)ppg_arch)->timingMode));
			term_cprintf(C_BLUE, "    PLL locking state: ");
			term_cprintf(C_WHITE, "%s\n",getStateAsString(pll_locking_state,((wrs_arch_data_t *)ppg_arch)->timingModeLockingState));
		}
		term_cprintf(C_CYAN, "----- HAL ---|---------------------------------- PPSI --------------------------------------------------------\n");
		term_cprintf(C_CYAN, " Iface| Freq |Inst|     Name     |   Config   | MAC of peer port  |    PTP/EXT/PDETECT States    | Pro | VLANs\n");
		term_cprintf(C_CYAN, "------+------+----+--------------+------------+-------------------+------------------------------+-----+------\n");

	}
	if (mode & (SHOW_SLAVE_PORTS|SHOW_MASTER_PORTS)) {
		printf("PORTS ");
	}

	for (i = 0; i < hal_nports_local; i++) {
		char if_name[10];
		int print_port = 0;
		int instance_port = 0;
		int color;

		snprintf(if_name, 10, "wri%d", i + 1);

		port_state = hal_lookup_port(hal_ports_local_copy,
						hal_nports_local, if_name);
		if (!port_state)
			continue;

		if (mode == SHOW_GUI) {
			/* check if link is up */
			if (state_up(port_state))
				term_cprintf(C_GREEN, " %-5s", if_name);
			else
				term_cprintf(C_RED, "*%-5s", if_name);
			term_cprintf(C_CYAN, "| ");
			if (port_state->locked)
				term_cprintf(C_GREEN, "Lock ");
			else
				term_cprintf(C_RED, "     ");

			term_cprintf(C_CYAN, "|");

			instance_port = 0;
			/*
			 * Actually, what is interesting is the PTP state.
			 * For this lookup, the port in ppsi shmem
			 */
			if ( ppsi_alive ) {
				for (j = 0; j < ppg->nlinks; j++) {
					char str_config[15];
					pp_instance_ptr_t *ppi_pt=&instances[j];
					struct pp_instance *ppi=ppi_pt->ppi;
					int proto_extension=ppi->protocol_extension;
					struct proto_ext_info_t *pe_info= IS_PROTO_EXT_INFO_AVAILABLE(proto_extension) ? &proto_ext_info[proto_extension] :  &proto_ext_info[0] ;

					if (strcmp(if_name,
							ppi->cfg.iface_name)) {
						/* Instance not for this interface
						 * skip */
						continue;
					}
					if (instance_port > 0) {
						term_cprintf(C_CYAN, "\n      |      |");
					}
					instance_port++;
					// Evaluate the instance configuration
					strcpy(str_config,"unknown");
					if ( defaultDS->slaveOnly) {
						strncpy(str_config,"slaveOnly",sizeof(str_config)-1);
					} else {
						if ( defaultDS->externalPortConfigurationEnabled ) {
							int s=0;
							for ( s=0; s<sizeof(desired_states)/sizeof(struct desired_state_t); s++ ) {
								if (desired_states[s].state == ppi->externalPortConfigurationPortDS.desiredState) {
									strncpy(str_config,desired_states[s].str_state,sizeof(str_config)-1);
									break;
								}
							}

						} else {
							if ( ppi_pt->portDS->masterOnly ) {
								strncpy(str_config,"masterOnly",sizeof(str_config)-1);
							} else {
								strncpy(str_config,"auto",sizeof(str_config)-1);
							}
						}
					}
					str_config[sizeof(str_config)-1]=0; // Force the string to be well terminated
					/* print instance number */
					term_cprintf(C_WHITE, " %2d ", j);
					term_cprintf(C_CYAN, "|");
					/* print instance name */
					term_cprintf(C_WHITE, "%-14s",ppi->cfg.port_name);
					term_cprintf(C_CYAN, "|");
					term_cprintf(C_WHITE, "%-12s",str_config);
					term_cprintf(C_CYAN, "| ");

					/* Note: we may have more pp instances per port */
/*					if (state_up(port_state->state)) */ {
						unsigned char *p = ppi->activePeer;
						char * extension_state_name=EMPTY_EXTENSION_STATE_NAME;

						term_cprintf(C_WHITE, "%02x:%02x"
								 ":%02x:%02x:%02x:%02x ",
								 p[0], p[1], p[2], p[3],
								 p[4], p[5]);
						term_cprintf(C_CYAN, "| ");
						term_cprintf(C_GREEN, "%s/",getStateAsString(pp_instance_state_to_name,ppi->state));
						/* print extension state */
						switch (ppi->protocol_extension ) {
#if CONFIG_HAS_EXT_WR
						case PPSI_EXT_WR :
						{
							portDS_t *portDS;

							extension_state_name=getStateAsString(wr_instance_extension_state,-1); // Default value

							if ( (portDS = wrs_shm_follow(ppsi_head, ppi->portDS) ) ) {
								struct wr_dsport *extPortDS;

								if ( (extPortDS = wrs_shm_follow(ppsi_head, portDS->ext_dsport) ) )
										extension_state_name=getStateAsString(wr_instance_extension_state,extPortDS->state);
							}
							break;
						}
#endif
#if CONFIG_HAS_EXT_L1SYNC
						case PPSI_EXT_L1S :
						{
							portDS_t *portDS;

							extension_state_name=getStateAsString(l1e_instance_extension_state,-1); // Default value
							if ( (portDS = wrs_shm_follow(ppsi_head, ppi->portDS) ) ) {
								l1e_ext_portDS_t *extPortDS;

								if ( (extPortDS = wrs_shm_follow(ppsi_head, portDS->ext_dsport) ) ) {
										extension_state_name=getStateAsString(l1e_instance_extension_state,extPortDS->basic.L1SyncState);
								}
							}
							break;
						}
#endif
						}
						term_cprintf(C_GREEN, "%s/%s",extension_state_name,getStateAsString(prot_detection_state_name,ppi->pdstate));
					} // else {
//						term_cprintf(C_WHITE, "                  ");
//						term_cprintf(C_CYAN, "|");
//						term_cprintf(C_WHITE, "                      ");
//					}
					term_cprintf(C_CYAN, "| ");
					if (ppi->proto == PPSI_PROTO_RAW) {
						term_cprintf(C_WHITE, "R");
					} else if (ppi->proto
						   == PPSI_PROTO_UDP) {
						term_cprintf(C_WHITE, "U");
					} else if (ppi->proto
						   == PPSI_PROTO_VLAN) {
						term_cprintf(C_WHITE, "V");
					} else {
						term_cprintf(C_WHITE, "?");
					}
					color=extensionStateColor(ppi);
					term_cprintf(color, "-%c",pe_info->short_ext_name);

					nvlans = ppi->nvlans;
					term_cprintf(C_CYAN, " | ");
					for (vlan_i = 0; vlan_i < nvlans; vlan_i++) {
						term_cprintf(C_WHITE, "%d",
								ppi->vlans[vlan_i]);
						if (vlan_i < nvlans - 1)
							term_cprintf(C_WHITE, ",");
					}
				}
			}
			if (!instance_port || !ppsi_alive) {
				term_cprintf(C_WHITE, " -- ");
				term_cprintf(C_CYAN, "|              |            |                   |                              |     |");
			}
			term_cprintf(C_WHITE, "\n");
		} else if (mode & WEB_INTERFACE) {
			printf("%s ", state_up(port_state)
				? "up" : "down");
			printf("%s ", port_state->locked
				? "Locked" : "NoLock");
			printf("%s ", port_state->calib.rx_calibrated
				&& port_state->calib.tx_calibrated
				? "Calibrated" : "Uncalibrated");
		} else if (print_port) {
			printf("port:%s ", if_name);
			printf("lnk:%d ", state_up(port_state));
			printf("lock:%d ", port_state->locked);
			print_port = 0;
		}
	}
	if (mode == SHOW_GUI) {
		term_cprintf(C_BLUE, "Pro - Protocol mapping: V-Ethernet over "
			     "VLAN; U-UDP; R-Ethernet\n");
	}
}
void show_calibration(){
	if ((mode == SHOW_GUI) && (showCal )){
	
	term_cprintf(C_CYAN, "\n--------------------------- Calibration values ----------------------------\n");

	TimeInterval delaysMM[MAX_INST_SERVO];
	TimeInterval delaysMS[MAX_INST_SERVO];
	TimeInterval offsetsFromMaster = 0;
	int i;
	int portCounter = 0;
	for (i=0; i<MAX_INST_SERVO; i++){

		if (servos[i].servo_snapshot.state == WRH_TRACK_PHASE){
				offsetsFromMaster  += servos[i].offsetFromMaster;
				delaysMM[0] = servos[i].servo_snapshot.delayMM.scaled_nsecs;
				delaysMS[0] = servos[i].servo_snapshot.delayMS.scaled_nsecs;
				portCounter = portCounter + 1;
		}		
		if (servos[i].servo_snapshot.state == WRH_MONITOR_PHASE){
				offsetsFromMaster  += servos[i].offsetFromMaster;
				delaysMM[1] = servos[i].servo_snapshot.delayMM.scaled_nsecs;
				delaysMS[1] = servos[i].servo_snapshot.delayMS.scaled_nsecs;
				portCounter = portCounter + 1;
		}

		if (portCounter ==2){
			break;
			}
	}

	if (portCounter !=2){
			term_cprintf(C_RED,"Waiting for ports in MONITOR_PHASE and TRACK_PHASE states\n");
			return;
		}	
	char buf[128];
	float a = (float) -2*offsetsFromMaster + delaysMM[0] -delaysMM[1] -2*(delaysMS[0] - delaysMS[1]);
	float b = (float) offsetsFromMaster - delaysMM[0] + delaysMS[0] -delaysMS[1] ;

	float alpha1 = a/b;
	term_cprintf(C_CYAN," | ");term_cprintf(C_BLUE,  "Port %s (active port) ",servos[0].ppi->cfg.iface_name);
	term_cprintf(C_WHITE,"alpha: %s\n",gcvt(alpha1,10,buf)); 
	// term_cprintf(C_WHITE,"delaysMM1: %s\n",timeIntervalToString(delaysMM[0],buf));
	// term_cprintf(C_WHITE,"AdelaysMS1: %s\n",timeIntervalToString(delaysMS[0],buf));

	b = (float) offsetsFromMaster + delaysMM[1] + delaysMS[0] -delaysMS[1] ;
	alpha1 = a/b;
	term_cprintf(C_CYAN," | ");term_cprintf(C_BLUE,  "Port %s (monitor port) ",servos[1].ppi->cfg.iface_name);
	term_cprintf(C_WHITE,"alpha: %s\n",gcvt(alpha1,10,buf)); 
	// term_cprintf(C_WHITE,"delaysMM2: %s\n",timeIntervalToString(delaysMM[1],buf));
	// term_cprintf(C_WHITE,"AdelaysMS2: %s\n",timeIntervalToString(delaysMS[1],buf));
	// term_cprintf(C_WHITE,"offsetFromMaster: %s\n",timeIntervalToString(offsetsFromMaster,buf));
	}
}
void show_servo(struct inst_servo_t *servo, int alive)
{

	wrh_servo_t * wr_servo;
	wr_servo_ext_t * wr_servo_ext=NULL;

	char buf[128];
	wrh_servo_t * l1e_servo;
	int proto_extension=servo->ppi->extState!=PP_EXSTATE_DISABLE ? servo->ppi->protocol_extension : PPSI_EXT_NONE;
	struct proto_ext_info_t *pe_info= IS_PROTO_EXT_INFO_AVAILABLE(proto_extension) ? &proto_ext_info[proto_extension] :  &proto_ext_info[0] ;

	wr_servo= (servo->ppi->protocol_extension==PPSI_EXT_WR && servo->ppi->extState==PP_EXSTATE_ACTIVE) ?
			( wrh_servo_t* ) servo->servo_ext_snapshot : NULL;
	if ( wr_servo ) {
		wr_servo_ext= &((struct wr_data *)wr_servo)->servo_ext;
	}
	l1e_servo= (servo->ppi->protocol_extension==PPSI_EXT_L1S && servo->ppi->extState==PP_EXSTATE_ACTIVE) ?
			( wrh_servo_t * ) servo->servo_ext_snapshot : NULL;

	if (mode == SHOW_GUI) {
		term_cprintf(C_CYAN, "\n--------------------------- Synchronization status ----------------------------\n");
	}

	if (!alive) {
		if (mode == SHOW_GUI)
			term_cprintf(C_RED, "PPSI is dead!\n");
		return;
	}

	if (mode == SHOW_GUI) {

		if (!(servo->servo_snapshot.flags & PP_SERVO_FLAG_VALID)) {
			term_cprintf(C_RED,
				     "Master mode or sync info not valid\n");
			return;
		}

		term_cprintf(C_BLUE, "Servo state:          ");
		if (pe_info->lastt && time(NULL) - pe_info->lastt > 5) {
			term_cprintf(C_RED, "--- not updating ---\n");
		} else {
			term_cprintf(C_WHITE, "%s:%s: %s%s\n",
				     servo->ppi->cfg.iface_name,
					 pe_info->ext_name,
					 servo->servo_snapshot.servo_state_name,
					 servo->servo_snapshot.flags & PP_SERVO_FLAG_WAIT_HW ?
				     " (wait for hw)" : "");
		}

		/* "tracking disabled" is just a testing tool */
		if (wr_servo  && !wr_servo->tracking_enabled)
			term_cprintf(C_RED, "Tracking forcibly disabled\n");
		term_cprintf(C_CYAN, "\n +- Timing parameters RGE ------------------------------------------------------\n");

		term_cprintf(C_CYAN," | ");term_cprintf(C_BLUE,  "meanDelay        : ");
		term_cprintf(C_WHITE, "%16s nsec\n", timeIntervalToString(servo->meanDelay,buf) );

		term_cprintf(C_CYAN," | ");term_cprintf(C_BLUE,  "delayMS          : ");
		term_cprintf(C_WHITE,"%s\n",optimized_pp_time_toString(&servo->servo_snapshot.delayMS,buf));
		{
			struct pp_time *delayMM= wr_servo_ext ?
					&wr_servo_ext->rawDelayMM :
					&servo->servo_snapshot.delayMM;
			term_cprintf(C_CYAN," | ");term_cprintf(C_BLUE,  "delayMM          : ");
			term_cprintf(C_WHITE,"%s\n",optimized_pp_time_toString(delayMM,buf));
		}

		//term_cprintf(C_BLUE, "Estimated link length:     ");
		/* (RTT - deltas) / 2 * c / ri
		 c = 299792458 - speed of light in m/s
		 ri = 1.4682 - refractive index for fiber g.652. However,
			       experimental measurements using long (~5km) and
			       short (few m) fibers gave a value 1.4827
		 */
		//term_cprintf(C_WHITE, "%10.2f meters\n",
		//	crtt / 2 / 1e6 * 299.792458 / 1.4827);


		term_cprintf(C_CYAN," | ");term_cprintf(C_BLUE,  "delayAsymmetry   : ");
		term_cprintf(C_WHITE, "%16s nsec\n",   timeIntervalToString(servo->delayAsymmetry,buf));
		term_cprintf(C_CYAN," | ");term_cprintf(C_BLUE,  "delayCoefficient : ");
		term_cprintf(C_WHITE, "%s", relativeDifferenceToString(servo->scaledDelayCoefficient,buf));
		term_cprintf(C_BLUE,  " fpa : ");
		term_cprintf(C_WHITE, "%lld",servo->scaledDelayCoefficient);
		term_cprintf(C_WHITE, "\n");
		term_cprintf(C_CYAN," | ");term_cprintf(C_BLUE,  "ingressLatency   : ");
		term_cprintf(C_WHITE, "%16s nsec\n",   timeIntervalToString(servo->ingressLatency,buf));
		term_cprintf(C_CYAN," | ");term_cprintf(C_BLUE,  "egressLatency    : ");
		term_cprintf(C_WHITE, "%16s nsec\n",   timeIntervalToString(servo->egressLatency,buf));
		term_cprintf(C_CYAN," | ");term_cprintf(C_BLUE,  "semistaticLatency: ");
		term_cprintf(C_WHITE, "%16s nsec\n",   timeIntervalToString(servo->semistaticLatency,buf));

		/*if (0) {
			term_cprintf(C_BLUE, "Fiber asymmetry:   ");
			term_cprintf(C_WHITE, "%.3f nsec\n",
				ss.fiber_asymmetry/1000.0);
		}*/

		term_cprintf(C_CYAN," | ");term_cprintf(C_BLUE, "offsetFromMaster : ");
		term_cprintf(C_WHITE, "%16s nsec\n", timeIntervalToString (servo->offsetFromMaster,buf));

		if ( wr_servo ) {
			term_cprintf(C_CYAN," | ");term_cprintf(C_BLUE, "Phase setpoint   : ");
			term_cprintf(C_WHITE, "%16.3f nsec\n",wr_servo->cur_setpoint_ps/1000.0);

			term_cprintf(C_CYAN," | ");term_cprintf(C_BLUE, "Skew             : ");
			term_cprintf(C_WHITE, "%16.3f nsec\n",wr_servo->skew_ps/1000.0);
		}

		if ( l1e_servo ) {
			term_cprintf(C_CYAN," | ");term_cprintf(C_BLUE, "Phase setpoint   : ");
			term_cprintf(C_WHITE, "%16.3f nsec\n",l1e_servo->cur_setpoint_ps/1000.0);

			term_cprintf(C_CYAN," | ");term_cprintf(C_BLUE, "Skew             : ");
			term_cprintf(C_WHITE, "%16.3f nsec\n",l1e_servo->skew_ps/1000.0);
		}
		term_cprintf(C_CYAN," | ");term_cprintf(C_BLUE, "Update counter   : ");
		term_cprintf(C_WHITE, "%16u times\n", servo->servo_snapshot.update_count);
		if (servo->servo_snapshot.update_count != pe_info->last_count) {
			pe_info->lastt = time(NULL);
			pe_info->last_count = servo->servo_snapshot.update_count;
		}

		if ( wr_servo ) {
			term_cprintf(C_CYAN," | ");term_cprintf(C_BLUE, "Master PHY delays ");
			term_cprintf(C_BLUE, "TX: ");
			term_cprintf(C_WHITE,"%s",optimized_pp_time_toString(&wr_servo_ext->delta_txm,buf));
			term_cprintf(C_BLUE, "  RX: ");
			term_cprintf(C_WHITE,"%s\n",optimized_pp_time_toString(&wr_servo_ext->delta_rxm,buf));

			term_cprintf(C_CYAN," | ");term_cprintf(C_BLUE, "Slave  PHY delays ");
			term_cprintf(C_BLUE, "TX: ");
			term_cprintf(C_WHITE,"%s",optimized_pp_time_toString(&wr_servo_ext->delta_txs,buf));
			term_cprintf(C_BLUE, "  RX: ");
			term_cprintf(C_WHITE,"%s\n",optimized_pp_time_toString(&wr_servo_ext->delta_rxs,buf));
			printf("\n");
		}
	} else {
		/* TJP: commented out fields are present on the SPEC,
		 *      does the switch have similar fields?
		 */
		printf("SERVO ");

		
/*		printf("lnk:");*/
/*		printf("rx:");*/
/*		printf("tx:");*/
		printf("wri:%s ",servo->ppi->cfg.iface_name);
		printf("sv:%d ", servo->servo_snapshot.flags & PP_SERVO_FLAG_VALID ? 1 : 0);
		printf("ss:'%s' ", servo->servo_snapshot.servo_state_name);
/*		printf("aux:");*/
		printf("md:%s ", timeIntervalToString(servo->meanDelay,buf));
		printf("dms:%s ", timeToString(&servo->servo_snapshot.delayMS,buf));
		if ( wr_servo ) {
			int64_t crtt= wr_servo->delayMM_ps - pp_time_to_picos(&wr_servo_ext->delta_txm) -
					pp_time_to_picos(&wr_servo_ext->delta_rxm) - pp_time_to_picos(&wr_servo_ext->delta_txs) -
					pp_time_to_picos(&wr_servo_ext->delta_rxs);

			printf("lock:%i ", wr_servo->tracking_enabled);
			printf("dtxm:%s ", timeToString(&wr_servo_ext->delta_txm,buf));
			printf("drxm:%s ", timeToString(&wr_servo_ext->delta_rxm,buf));
			printf("dtxs:%s ", timeToString(&wr_servo_ext->delta_txs,buf));
			printf("drxs:%s ", timeToString(&wr_servo_ext->delta_rxs,buf));
		/* (RTT - deltas) / 2 * c / ri
		 c = 299792458 - speed of light in m/s
		 ri = 1.4682 - refractive index for fiber g.652. However,
			       experimental measurements using long (~5km) and
			       short (few m) fibers gave a value 1.4827
		 */
		//printf("ll:%d ",
		//       (int) (crtt / 2 / 1e6 * 299.792458 / 1.4827 * 100));
			printf("crtt:%llu ", crtt);
			printf("setp:%d ", wr_servo->cur_setpoint_ps);
		}
		if ( l1e_servo ) {
			printf("lock:%i ", l1e_servo->tracking_enabled);
			printf("setp:%d ", l1e_servo->cur_setpoint_ps);
		}
		printf("asym:%s ", timeIntervalToString(servo->delayAsymmetry,buf));
		printf("cko:%s ", timeIntervalToString(servo->offsetFromMaster,buf));
/*		printf("hd:");*/
/*		printf("md:");*/
/*		printf("ad:");*/
		printf("ucnt:%u ", servo->servo_snapshot.update_count);
		/* SPEC shows temperature, but that can be selected separately
		 * in this program
		 */
	}
}

void show_servos(int alive) {

	int i;

	for (i=0; i<MAX_INST_SERVO; i++)
		if (servos[i].ppi )
			show_servo(&servos[i], alive);
}


void show_temperatures(void)
{
	if ((mode == SHOW_GUI) || (mode & WEB_INTERFACE)) {
		if (mode == SHOW_GUI) {
/*                                              -------------------------------------------------------------------------------*/
			term_cprintf(C_CYAN, "\n-------------------------------- Temperatures ---------------------------------\n");
		} else {
			term_cprintf(C_CYAN, "\nTemperatures:\n");
		}

		term_cprintf(C_BLUE, "FPGA: ");
		term_cprintf(C_WHITE, "%2.2f ",
			     temp_sensors_local.fpga/256.0);
		term_cprintf(C_BLUE, "PLL: ");
		term_cprintf(C_WHITE, "%2.2f ",
			     temp_sensors_local.pll/256.0);
		term_cprintf(C_BLUE, "PSL: ");
		term_cprintf(C_WHITE, "%2.2f ",
			     temp_sensors_local.psl/256.0);
		term_cprintf(C_BLUE, "PSR: ");
		term_cprintf(C_WHITE, "%2.2f\n",
			     temp_sensors_local.psr/256.0);
	} else {
		printf("TEMP ");
		printf("fpga:%2.2f ", temp_sensors_local.fpga/256.0);
		printf("pll:%2.2f ", temp_sensors_local.pll/256.0);
		printf("psl:%2.2f ", temp_sensors_local.psl/256.0);
		printf("psr:%2.2f", temp_sensors_local.psr/256.0);
	}
}

void show_time(void)
{
	printf("TIME sec:%lld nsec:%d ", seconds, nanoseconds);
}

void show_all(void)
{
	int hal_alive;
	int ppsi_alive;

	if (mode == SHOW_GUI) {
		term_clear();
		term_pcprintf(1, 1, C_BLUE, "WR Switch Sync Monitor ");
		term_cprintf(C_WHITE, "%s", __GIT_VER__);
		term_cprintf(C_BLUE, " [q = quit]\n\n");
	}

	hal_alive = (hal_head->pid && (kill(hal_head->pid, 0) == 0))
								+ ignore_alive;
	ppsi_alive = (ppsi_head->pid && (kill(ppsi_head->pid, 0) == 0))
								+ ignore_alive;

	if (mode & SHOW_WR_TIME) {
		if (ppsi_alive)
			show_time();
		else if (mode == SHOW_ALL)
			printf("PPSI is dead!\n");
	}

	if ((mode & (SHOW_ALL_PORTS|WEB_INTERFACE)) || mode == SHOW_GUI) {
		show_ports(hal_alive,ppsi_alive);
	}

	if (mode & SHOW_SERVO || mode == SHOW_GUI) {
		show_servos(ppsi_alive);
		show_calibration();
	}

	if (mode & (SHOW_TEMPERATURES | WEB_INTERFACE) || mode == SHOW_GUI) {
		if (hal_alive)
			show_temperatures();
	}

	if (!(mode & WEB_INTERFACE || mode == SHOW_GUI)) {
		/* the newline for all in non-GUI or non-WEB mode... */
		printf("\n");
	}
	fflush(stdout);
}

static void enable_disable_tracking(int proto_extension) {

	if ( IS_PROTO_EXT_INFO_AVAILABLE(proto_extension)  ) {
		struct proto_ext_info_t *pe_info= &proto_ext_info[proto_extension];

		if ( pe_info->ipc_cmd_tacking!=-1 ) {
			int rval;

			pe_info->track_onoff = 1-pe_info->track_onoff;
			if (ptp_ch_pid != ppsi_head->pid) {
				/* ppsi was restarted since minipc
				 * connection, reconnect now */
				ppsi_connect_minipc();
			}
			minipc_call(ptp_ch, 200, &__rpcdef_cmd,
				&rval, pe_info->ipc_cmd_tacking,pe_info->track_onoff);
		}
	}
}

int main(int argc, char *argv[])
{
	int opt;
	int usecolor = 1;

	/* try a pps_gen based approach */
	uint64_t last_seconds = 0;

	wrs_msg_init(argc, argv, LOG_USER);

	while ((opt = getopt(argc, argv, "himcsoetabwqvH:")) != -1) {
		switch(opt)
		{
			case 'h':
				help(argv[0]);
			case 'i':
				mode |= SHOW_WR_TIME;
				break;
			case 'c':
				showCal = 1;
				mode = SHOW_GUI;
				break;
			case 's':
				mode |= SHOW_SLAVE_PORTS;
				break;
			case 'm':
				mode |= SHOW_MASTER_PORTS;
				break;
			case 'o':
				mode |= SHOW_OTHER_PORTS;
				break;
			case 'e':
				mode |= SHOW_SERVO;
				break;
			case 't':
				mode |= SHOW_TEMPERATURES;
				break;
			case 'a':
				mode |= SHOW_ALL;
				break;
			case 'b':
				usecolor = 0;
				break;
			case 'w':
				mode |= WEB_INTERFACE;
				break;
			case 'H':
				wrs_shm_set_path(optarg);
				/* ignore WRS_SHM_LOCKED flag */
				wrs_shm_ignore_flag_locked(1);
				ignore_alive = 1;
				break;
			case 'q': break; /* done in wrs_msg_init() */
			case 'v': break; /* done in wrs_msg_init() */
			default:
				help(argv[0]);
		}
	}

	init_shm();

	if (shw_fpga_mmap_init() < 0) {
		pr_error("Can't initialize FPGA mmap\n");
		exit(1);
	}

	if (mode & WEB_INTERFACE) {
		shw_pps_gen_read_time(&seconds, &nanoseconds);
		read_servo();
		read_hal();
		show_all();
		exit(0);
	}

	term_init(usecolor);
	setvbuf(stdout, NULL, _IOFBF, 4096);

	/* main loop */
	for(;;)
	{
		if (term_poll(10)) {
			int c = term_get();

			switch (c) {
			case 'q':
				goto quit;
			case 'w' :
				enable_disable_tracking (PPSI_EXT_WR);
				break;
			case 'l' :
				enable_disable_tracking (PPSI_EXT_L1S);
				break;
			}
		}

		shw_pps_gen_read_time(&seconds, &nanoseconds);
		if (seconds != last_seconds) {
			read_servo();
			read_hal();
			show_all();

			last_seconds = seconds;
		}

		/* If we got broken pipe or anything, exit */
		if (ferror(stdout))
			exit(1);
	}

	quit:;
	term_restore();
	setlinebuf(stdout);
	printf("\n");
	return 0;
}
