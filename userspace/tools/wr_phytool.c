/*\
 * WR Switch PHY testing tool
 	 Tomasz Wlostowski / 2012

	 WARNING: This is just my internal code for testing some timing-related stuff.
	 				  I'll document it and clean it up in the future, but for now, please
	 				  don't ask questions. sorry
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <assert.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <signal.h>

#define __EXPORTED_HEADERS__ /* prevent a #warning notice from linux/types.h */
//#define _LINUX_IF_ETHER_H /* prevent linux/if_ether.h that redefines ethhdr  */
#include <linux/mii.h>
#include <libwr/shw_io.h>
#include "../libwr/i2c_sfp.h"


#include <regs/endpoint-regs.h>
#include <regs/endpoint-mdio.h>

#undef PACKED
#include <libwr/ptpd_netif.h>
#define __EXPORTED_HEADERS__ /* prevent a #warning notice from linux/types.h */
#define _LINUX_IF_ETHER_H /* prevent linux/if_ether.h that redefines ethhdr  */
#include <linux/mii.h>

#include <rt_ipc.h>
#include <libwr/switch_hw.h>

#include <libwr/shmem.h>
#include <libwr/hal_shmem.h>
#include <libwr/wrs-msg.h>
#include <libwr/util.h>

#define WRS3_FPGA_BASE 0x10000000
#define WRS3_FPGA_SIZE 0x100000

void *fpga;

static struct EP_WB _ep_wb;


#define EP_REG(regname) ((uint32_t)((void*)&_ep_wb.regname - (void*)&_ep_wb))
#define IDX_TO_EP(x) (0x30000 + ((x) * 0x400))

#define fpga_writel(val, addr) *(volatile uint32_t *)(fpga + (addr)) = val
#define fpga_readl(addr) (*(volatile uint32_t *)(fpga + (addr)))

static int hal_nports_local;
static struct wrs_shm_head *hal_head;
static struct hal_port_state *hal_ports;

int hal_shm_init(void)
{
	int ii;
	struct hal_shmem_header *h;
	int n_wait = 0;
	int ret;

	/* wait for HAL */
	while ((ret = wrs_shm_get_and_check(wrs_shm_hal, &hal_head)) != 0) {
		n_wait++;
		if (n_wait > 10) {
			if (ret == WRS_SHM_OPEN_FAILED) {
				fprintf(stderr, "wr_phytool: Unable to open "
					"HAL's shm !\n");
			}
			if (ret == WRS_SHM_WRONG_VERSION) {
				fprintf(stderr, "wr_phytool: Unable to read "
					"HAL's version!\n");
			}
			if (ret == WRS_SHM_INCONSISTENT_DATA) {
				fprintf(stderr, "wr_phytool: Unable to read "
					"consistent data from HAL's shmem!\n");
			}
			return(-1);
		}
		sleep(1);
	}

	/* check hal's shm version */
	if (hal_head->version != HAL_SHMEM_VERSION) {
		fprintf(stderr, "wr_mon: unknown hal's shm version %i "
			"(known is %i)\n",
			hal_head->version, HAL_SHMEM_VERSION);
		return -1;
	}

	h = (void *)hal_head + hal_head->data_off;

	while (1) { /* wait forever for HAL to produce consistent nports */
		ii = wrs_shm_seqbegin(hal_head);
		/* Assume number of ports does not change in runtime */
		hal_nports_local = h->nports;
		if (!wrs_shm_seqretry(hal_head, ii))
			break;
		fprintf(stderr, "INFO: wr_phytool Wait for HAL.\n");
		sleep(1);
	}

	/* Even after HAL restart, HAL will place structures at the same
	 * addresses. No need to re-dereference pointer at each read. */
	hal_ports = wrs_shm_follow(hal_head, h->ports);
	if (hal_nports_local > HAL_MAX_PORTS) {
		fprintf(stderr,
			"FATAL: wr_phytool Too many ports reported by HAL."
			"%d vs %d supported\n",
			hal_nports_local, HAL_MAX_PORTS);
		return -1;
	}
	return 0;
}

uint32_t pcs_read(int ep, uint32_t reg)
{
	fpga_writel(EP_MDIO_CR_ADDR_W(reg), IDX_TO_EP(ep) + EP_REG(MDIO_CR));
	while(! (fpga_readl(IDX_TO_EP(ep) + EP_REG(MDIO_ASR)) & EP_MDIO_ASR_READY));
	return EP_MDIO_CR_DATA_R(fpga_readl(IDX_TO_EP(ep) + EP_REG(MDIO_ASR)));
}

void pcs_write(int ep, uint32_t reg, uint32_t val)
{
	fpga_writel(EP_MDIO_CR_ADDR_W(reg) | EP_MDIO_CR_DATA_W(val) | EP_MDIO_CR_RW, IDX_TO_EP(ep) + EP_REG(MDIO_CR));
	while(! (fpga_readl(IDX_TO_EP(ep) + EP_REG(MDIO_ASR)) & EP_MDIO_ASR_READY));
}

void dump_pcs_regs(int ep, int argc, char *argv[])
{
	int i;
	printf("PCS registers dump for endpoint %d:\n", ep);
	for(i=0;i<20;i++)
		printf("R%d = 0x%08x\n",i,pcs_read(ep, i));
}

void write_pcs_reg(int ep, int argc, char *argv[])
{
	int reg, data;
	if (argc < 4) {
		printf("Not enough parameters %d\n", argc);
		exit(1);
	}
	sscanf(argv[3], "%x", &reg);
	sscanf(argv[4], "%x", &data);
	pcs_write(ep, reg, data);
	printf("R%d = 0x%08x\n",reg, pcs_read(ep, reg));
}


void tx_cal(int ep, int argc, char *argv[])
{
	int on_off = atoi(argv[3]) ? 1 : 0;
	printf("TX cal pattern %s\n", on_off ? "ON":"OFF");

	pcs_write(ep, 16, on_off);
	printf("rdbk:%x\n",pcs_read(ep, 16));
}

void try_autonegotiation(int ep, int argc, char *argv[])
{
	pcs_write(ep, MII_ADVERTISE, 0x1a0);
	pcs_write(ep, MII_BMCR, BMCR_ANENABLE | BMCR_ANRESTART);

	printf("Autonegotiation: ");
	for(;;)
	{
		printf(".");
		fflush(stdout);
		if(pcs_read(ep, MII_BMSR) & BMSR_ANEGCOMPLETE)
			break;
		sleep(1);
	}
	printf("\nComplete: LPA %x\n", pcs_read(ep, MII_LPA));
}

int get_bitslide(int ep)
{
	return (pcs_read(ep, 16) >> 4) & 0x1f;
}

int get_phase(int ep, int *phase)
{
	uint32_t dmsr=fpga_readl(IDX_TO_EP(ep) + EP_REG(DMSR));
	if(dmsr & EP_DMSR_PS_RDY)
	{
		*phase = (dmsr & 0xffffff) / 1024;
//		fprintf(stderr ,"NewPhase: %d\n", *phase);

		return 1;
	}
	return 0;
}

#define MAX_BITSLIDES 20


static	struct {
		int occupied;
		int phase_min, phase_max, phase_dev;
		int rx_ahead;
		int delta;
		int hits;
} bslides[MAX_BITSLIDES];

int bslide_bins(void)
{
	int i, hits = 0;

	for(i=0;i<MAX_BITSLIDES;i++)
		if(bslides[i].occupied) hits++;

	return hits;
}

#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))
void bslide_update(int phase, int delta, int ahead, int bs)
{
	bslides[bs].occupied = 1;
	bslides[bs].phase_min = MIN(phase, bslides[bs].phase_min);
	bslides[bs].phase_max = MAX(phase, bslides[bs].phase_max);
	bslides[bs].phase_dev = bslides[bs].phase_max - bslides[bs].phase_min;
	bslides[bs].delta = delta;
	bslides[bs].rx_ahead = ahead;
	bslides[bs].hits++;
}

static int quit = 0;

void sighandler(int sig)
{
	quit = 1;
}

static void print_cal_stats(void)
{
	int i,last_occupied = -1;
	printf("Calibration statistics: \n");
	printf("bitslide[UIs] | Delta[ns] | Ahead[bool] | phase_min[tics] | phase_max[tics] | phase_dev[tics] | hits | delta_prev[ps] \n");
	for(i=0;i<MAX_BITSLIDES;i++)
		if(bslides[i].occupied)
		{
			printf("%-15d %-11d %-13d %-17d %-17d %-17d %-6d %d\n",
				i,
				bslides[i].delta,
				bslides[i].rx_ahead,
				bslides[i].phase_min,
				bslides[i].phase_max,
				bslides[i].phase_dev,
				bslides[i].hits,
				(last_occupied >= 0) ? bslides[i].delta-bslides[last_occupied].delta:0
			);

			last_occupied = i;
		}
	printf("\n");
}

void calc_trans(int ep, int argc, char *argv[])
{
	struct wr_tstamp ts_tx, ts_rx;
	struct wr_socket *sock;
	FILE *f_log = NULL;
	struct wr_sockaddr sock_addr, from;
	int bitslide,phase, i;
	struct hal_port_state *port;

	memset(&ts_tx, 0, sizeof(ts_tx));
	memset(&ts_rx, 0, sizeof(ts_rx));
	signal (SIGINT, sighandler);
	for(i=0;i<MAX_BITSLIDES;i++)
	{
		bslides[i].occupied = 0;
		bslides[i].phase_min = 1000000;
		bslides[i].phase_max= -1000000;
	}

	if(argc >= 3)
		f_log = fopen(argv[3], "wb");

	snprintf(sock_addr.if_name, sizeof(sock_addr.if_name), "wri%d", ep + 1);
	sock_addr.family = PTPD_SOCK_RAW_ETHERNET; // socket type
	sock_addr.ethertype = 12345;
	memset(sock_addr.mac, 0xff, 6);

	assert(ptpd_netif_init() == 0);
	assert(hal_shm_init() == 0);
	port = hal_lookup_port(hal_ports, hal_nports_local,
			       sock_addr.if_name);
	sock = ptpd_netif_create_socket(PTPD_SOCK_RAW_ETHERNET, 0, &sock_addr,
					port);

//	fpga_writel(EP_DMCR_N_AVG_W(1024) | EP_DMCR_EN, IDX_TO_EP(ep) + EP_REG(DMCR));

	if( rts_connect(NULL) < 0)
	{
		printf("Can't connect to the RT subsys\n");
		return;
	}


	while(!quit)
	{
		char buf[64];
		struct wr_sockaddr to;
		struct rts_pll_state pstate;

	/*	pcs_write(ep, MII_BMCR, BMCR_PDOWN);
		usleep(10000);
		pcs_write(ep, MII_BMCR, 0);
		pcs_read(ep, MII_BMSR);*/


		while(! (pcs_read(ep, MII_BMSR) & BMSR_LSTATUS)) usleep(10000);


		usleep(200000);
		bitslide = get_bitslide(ep);
		rts_enable_ptracker(ep, 0);
		rts_enable_ptracker(ep, 1);
		usleep(1000000);

//		get_phase(ep, &phase);
		rts_get_state(&pstate);

		phase = pstate.channels[ep].phase_loopback;
		printf("phase %d flags %x\n", phase,  pstate.channels[ep].flags);

	//		ptpd_netif_get_dmtd_phase(sock, &phase);



		memset(to.mac, 0xff, 6);
		to.ethertype = 12345;
		to.family = PTPD_SOCK_RAW_ETHERNET; // socket type

		ptpd_netif_sendto(sock, &to, buf, 64, &ts_tx);
		int n = ptpd_netif_recvfrom(sock, &from, buf, 64, &ts_rx,
					    port);


		if(n>0)
		{
			int delta = ts_rx.nsec * 1000 + ts_rx.phase - ts_tx.nsec * 1000;
			bslide_update(phase, delta, ts_rx.raw_ahead, bitslide);
			printf("delta %d (adv %d), bitslide: %d bins: %d phase %d, rxphase %d\n", delta, ts_rx.raw_ahead, bitslide, bslide_bins(), phase, ts_rx.phase);

			if(f_log) fprintf(f_log, "%d %d %d %d %d\n", bitslide, ts_tx.nsec, ts_rx.raw_nsec, ts_rx.raw_phase, ts_rx.raw_ahead);
		}
		usleep(500000);
	}

	if(f_log) fclose(f_log);
	print_cal_stats();
}



void analyze_phase_log(int ep, int argc, char *argv[])
{
	FILE *f_log = NULL;
	int bitslide,phase, i,trans_point;

	f_log=fopen(argv[3], "r");

	if(argc>4)
		trans_point = atoi(argv[4]);
	else
		trans_point = 1000;

	for(i=0;i<MAX_BITSLIDES;i++)
	{
		bslides[i].occupied = 0;
		bslides[i].phase_min = 1000000;
		bslides[i].phase_max= -1000000;
	}


	while(!feof(f_log))
	{
			struct wr_tstamp ts_tx, ts_rx;
			fscanf(f_log, "%d %d %d %d %d\n", &bitslide, &ts_tx.nsec, &ts_rx.raw_nsec, &ts_rx.raw_phase, &ts_rx.raw_ahead);

			ts_rx.nsec = ts_rx.raw_nsec;
			phase = ts_rx.raw_phase;

			ptpd_netif_linearize_rx_timestamp(&ts_rx, ts_rx.raw_phase, ts_rx.raw_ahead, trans_point, 16000);

			int delta = ts_rx.nsec * 1000 + ts_rx.phase - ts_tx.nsec * 1000;
			bslide_update(phase, delta, ts_rx.raw_ahead, bitslide);

	}

	printf("Transition point: %d ps. \n", trans_point);
	print_cal_stats();
	fclose(f_log);
}

void pps_adjustment_test(int ep, int argc, char *argv[])
{
	struct wr_tstamp ts_tx, ts_rx;
	struct wr_socket *sock;
	struct wr_sockaddr sock_addr, from;
	int adjust_count = 0;
	struct hal_port_state *port;

	memset(&ts_tx, 0, sizeof(ts_tx));
	memset(&ts_rx, 0, sizeof(ts_rx));
	signal (SIGINT, sighandler);

	snprintf(sock_addr.if_name, sizeof(sock_addr.if_name), "wri%d", ep + 1);
	sock_addr.family = PTPD_SOCK_RAW_ETHERNET; // socket type
	sock_addr.ethertype = 12345;
	memset(sock_addr.mac, 0xff, 6);

	assert(ptpd_netif_init() == 0);
	assert(hal_shm_init() == 0);
	port = hal_lookup_port(hal_ports, hal_nports_local,
			       sock_addr.if_name);
	sock = ptpd_netif_create_socket(PTPD_SOCK_RAW_ETHERNET, 0, &sock_addr,
					port);

	while(!quit)
	{
		char buf[64];
		struct wr_sockaddr to;

		memset(to.mac, 0xff, 6);
		to.ethertype = 12345;
		to.family = PTPD_SOCK_RAW_ETHERNET; // socket type

		if(adjust_count == 0)
		{
			ptpd_netif_adjust_counters(1,0);// 500000000);
			adjust_count = 8;
		}

//		if(!ptpd_netif_adjust_in_progress())
		{
			ptpd_netif_sendto(sock, &to, buf, 64, &ts_tx);
			ptpd_netif_recvfrom(sock, &from, buf, 64, &ts_rx,
					    port);
			printf("TX timestamp: correct %d %12lld:%12d\n", ts_tx.correct, ts_tx.sec, ts_tx.nsec);
			printf("RX timestamp: correct %d %12lld:%12d\n", ts_rx.correct, ts_rx.sec, ts_rx.nsec);
			adjust_count --;
		}// else printf("AdjustInProgress\n");

		sleep(1);
	}
}

char * getRtsTimingMode(int mode) {
	static struct {
		char *name;
		int mode;
	} *m, modes[] = {
		{"TIME_GM", RTS_MODE_GM_EXTERNAL},
		{"TIME_FM", RTS_MODE_GM_FREERUNNING},
		{"TIME_BC", RTS_MODE_BC},
		{"TIME_DS", RTS_MODE_DISABLED},
		{"???????",-1}
	};

	m=modes;
	do {
		if ( m->mode==mode )
			break;
		m++;
	} while (m->mode!=-1);
	return m->name;
}


#define TXCAL_ST_STARTUP 0
#define TXCAL_ST_WAIT_SPLL 1
#define TXCAL_ST_WAIT_PTRACKER 2
#define TXCAL_ST_READY 3

struct port_state
{
	int ep;
	int state;
	int tx_phase_bias;
	int tx_phase;
	struct bitslide_bin
	{
		int samples;
		int acc;
		int rx_phase;
	} bins[20];
};

struct key_value
	{
		char *comment;
		char *key;
		char *value;
		struct key_value *next;
	};

struct config_file
{
	
	struct key_value *head;
};

struct key_value *cfg_find_key(struct config_file *cfg, const char *key, int create)
{
	struct key_value *kv = cfg->head, *kv_prev = cfg->head;

	while (kv)
	{
		if (kv->key && !strcasecmp(key, kv->key))
		{
			printf("found key %s %p\n", key, kv);
			return kv;
		}
		kv_prev = kv;
		kv = kv->next;
	}

	if (!create)
		return NULL;

	if (!cfg->head)
	{
		cfg->head = malloc(sizeof(struct key_value));
		kv = cfg->head;
	}
	else
	{
		kv_prev->next = malloc(sizeof(struct key_value));
		kv = kv_prev->next;
	}

	printf("create %s %p\n", key, kv);

	kv->key = strdup(key);
	kv->value = NULL;
	kv->comment = NULL;
	kv->next = NULL;

	return kv;
}

int cfg_get_int(struct config_file *cfg, const char *key, int *value)
{
	struct key_value *kv = cfg_find_key(cfg, key, 0);

	if (!kv)
		return 0;

	*value = atoi(kv->value);
	return 1;
}

struct config_file *cfg_open(const char *filename, int overwrite)
{
	FILE *f = fopen(filename, "rb");
	if (!f)
	{

		if (overwrite)
		{
			struct config_file *cfg = malloc(sizeof(struct config_file));

			f = fopen(filename, "wb");

			cfg->head = NULL;

			return cfg;
		}
		return NULL;
	}

	struct config_file *cfg = malloc(sizeof(struct config_file));

	cfg->head = NULL;

	while (!feof(f))
	{
		char str[1024];
		int rv = (int) fgets(str, 1024, f);
		int i, pos = -1;

		if (rv == 0)
			break;

		for (i = 0; i < strlen(str); i++)
		{
			if (str[i] == '=' && pos < 0)
				pos = i;
			if (str[i] == '\n' || str[i] == '\r')
				str[i] = 0;
		}

		struct key_value *kv = malloc(sizeof(struct key_value));
		if (pos < 0 || strlen(str) == 0 || str[0] == '#')
		{
			kv->comment = strdup(str);
			kv->key = NULL;
			kv->value = NULL;
		}
		else
		{
			kv->comment = NULL;
			kv->key = strndup(str, pos);
			kv->value = strdup(str + pos + 1);
			kv->next = NULL;
			printf("KV: %s = %s\n", kv->key, kv->value);
		}

		if (!cfg->head)
		{
			cfg->head = kv;
		}
		else
		{
			struct key_value *last = cfg->head;
			while (last->next)
				last = last->next;
			last->next = kv;
		}
	}

	return cfg;
}

int cfg_set_int(struct config_file *cfg, const char *key, int value)
{
	struct key_value *kv = cfg_find_key(cfg, key, 1);

	if (!kv)
		return 0;

	char buf[1024];
	snprintf(buf, 1024, "%d", value);

	kv->value = strdup(buf);
	return 1;
}

int cfg_save(struct config_file *cfg, const char *filename)
{
	FILE *f = fopen(filename, "wb");

	if (!f)
	{
		printf("Can't open '%s' for writing.\n", filename);
		return -1;
	}

	struct key_value *kv = cfg->head;

	while (kv)
	{
		if (kv->comment)
			fprintf(f, "%s\n", kv->comment);
		else
			fprintf(f, "%s=%s\n", kv->key, kv->value);

		kv = kv->next;
	}

	fclose(f);
	return 0;
}

int calibrate_tx_delay_fsm(struct port_state *state)
{
	struct rts_pll_state pstate;

	switch (state->state)
	{
	case TXCAL_ST_STARTUP:
	{
		printf("Port %d: start TX calibration\n", state->ep);

		pcs_write(state->ep, 0, MDIO_MCR_PDOWN);
		pcs_write(state->ep, 0, MDIO_MCR_LOOPBACK); // re-set the PHY and configure in loopback mode
		pcs_write(state->ep, 19, 1);								// configure DDMTD RX CLK source to TXOUTCLK (TX PMA clock) for calibration

		rts_enable_ptracker(state->ep, 0);
		rts_get_state(&pstate);

		if (pstate.mode != RTS_MODE_GM_FREERUNNING)
		{
			printf("Setup SPLL to free-running mode\n");
			rts_set_mode(RTS_MODE_GM_FREERUNNING);
		}
		state->state = TXCAL_ST_WAIT_SPLL;
		break;
	}

	case TXCAL_ST_WAIT_SPLL:
	{
		rts_get_state(&pstate);
		if (pstate.flags & RTS_DMTD_LOCKED)
		{
			rts_enable_ptracker(state->ep, 1);
			state->state = TXCAL_ST_WAIT_PTRACKER;
		}
		break;
	}

	case TXCAL_ST_WAIT_PTRACKER:
	{
		rts_get_state(&pstate);
		if (pstate.channels[state->ep].flags & CHAN_PMEAS_READY)
		{
			state->tx_phase = pstate.channels[state->ep].phase_loopback;
			printf("Port %d TX Phase = %d ps\n", state->ep, state->tx_phase);
			state->state = TXCAL_ST_READY;
		}
		break;
	}
	}

	return (state->state == TXCAL_ST_READY);
}

#define RXCAL_ST_STARTUP 0
#define RXCAL_ST_RESET_PMA 1
#define RXCAL_ST_WAIT_LINK 2
#define RXCAL_ST_WAIT_TX_PHASE 3
#define RXCAL_ST_WAIT_RX_PHASE 4
#define RXCAL_ST_DONE 5

#define RXCAL_MIN_SAMPLES 2

int calibrate_rx_delay_fsm(struct port_state *state, int use_external_loopback)
{
	struct rts_pll_state pstate;
	int i;
	int min_samples_per_bin = 1000000;
	int empty_bins = 0, full_bins = 0;

	switch (state->state)
	{
	case RXCAL_ST_STARTUP:
	{
		printf("Port %d: start RX calibration\n", state->ep);

		state->state = RXCAL_ST_RESET_PMA;
		break;
	}

	case RXCAL_ST_RESET_PMA:
	{
		pcs_write(state->ep, 0, MDIO_MCR_PDOWN);
		pcs_write(state->ep, 0, (use_external_loopback == state->ep) ? 0 : MDIO_MCR_LOOPBACK); // re-set the PHY and configure in loopback mode
		pcs_write(state->ep, 19, 1);																													 // configure DDMTD RX CLK source to TXOUTCLK (TX PMA clock) for calibration

		rts_enable_ptracker(state->ep, 0);
		state->state = RXCAL_ST_WAIT_LINK;
		break;
	}

	case RXCAL_ST_WAIT_LINK:
	{
		if (pcs_read(state->ep, 1) & MDIO_MSR_LSTATUS)
		{
			int bitslide = get_bitslide(state->ep);

			if (state->bins[bitslide].samples > RXCAL_MIN_SAMPLES)
			{
				// not interesting, we've got enough phase samples in this bin
				state->state = RXCAL_ST_RESET_PMA;
			}
			else
			{
				state->state = RXCAL_ST_WAIT_TX_PHASE;
				rts_enable_ptracker(state->ep, 1);
			}
		}

		break;
	}

	case RXCAL_ST_WAIT_TX_PHASE:
	{
		rts_get_state(&pstate);
		if (pstate.channels[state->ep].flags & CHAN_PMEAS_READY)
		{
			state->tx_phase = pstate.channels[state->ep].phase_loopback;

			rts_enable_ptracker(state->ep, 0);
			pcs_write(state->ep, 19, 0); // configure DDMTD RX CLK source to RXRECCLK (RX recovered PMA clock)
			rts_enable_ptracker(state->ep, 1);

			state->state = RXCAL_ST_WAIT_RX_PHASE;
		}
		break;
	}

	case RXCAL_ST_WAIT_RX_PHASE:
	{
		rts_get_state(&pstate);
		if (pstate.channels[state->ep].flags & CHAN_PMEAS_READY)
		{
			int rx_phase = (pstate.channels[state->ep].phase_loopback - (state->tx_phase - state->tx_phase_bias));
			int bitslide = get_bitslide(state->ep);

			while (rx_phase < 0)
				rx_phase += 800;
			while (rx_phase > 800)
				rx_phase -= 800;

			printf("RXph %d TXPh %d bs %d [", state->tx_phase, rx_phase, bitslide);
			state->state = RXCAL_ST_RESET_PMA;

			if (state->bins[bitslide].samples <= RXCAL_MIN_SAMPLES)
			{
				state->bins[bitslide].acc += rx_phase;
				state->bins[bitslide].samples++;
			}

			for (i = 0; i < 20; i++)
			{
				printf("%d ", state->bins[i].samples);
				if (state->bins[i].samples < min_samples_per_bin)
					min_samples_per_bin = state->bins[i].samples;

				if (state->bins[i].samples > RXCAL_MIN_SAMPLES)
					full_bins++;

				if (state->bins[i].samples == 0)
					empty_bins++;
			}

			printf("] min = %d, empty bins = %d \n", min_samples_per_bin, empty_bins);
			if (full_bins == 19)
			{
				int avg = 0;

				for (i = 0; i < 20; i++)
				{
					if (state->bins[i].samples > RXCAL_MIN_SAMPLES)
					{
						state->bins[i].acc /= state->bins[i].samples;
						avg += state->bins[i].acc;
					}
				}

				avg /= full_bins;

				printf("Calibration offsets for port %d:\n", state->ep);

				for (i = 0; i < 20; i++)
				{
					if (state->bins[i].samples > RXCAL_MIN_SAMPLES)
					{
						state->bins[i].acc -= avg;
						printf("bitslide %d: %d ps\n", i, state->bins[i].acc);
					}
				}

				state->state = RXCAL_ST_DONE;
				return 1;
			}

			break;
		}

	case RXCAL_ST_DONE:
		return 1;
	}
	}

	return 0;
}

struct port_state ports[8];

void run_rx_calibration(void)
{
	int i;

	for (i = 0; i < 8; i++)
	{
		int j;
		ports[i].state = RXCAL_ST_STARTUP;
		ports[i].tx_phase_bias = ports[i].tx_phase;
		for (j = 0; j < 20; j++)
		{
			ports[i].bins[j].samples = 0;
			ports[i].bins[j].acc = 0;
			ports[i].bins[j].rx_phase = 0;
		}

		ports[i].ep = i;
	}

	int done;

	do
	{
		done = 1;
		for (i = 0; i < 8; i++)
		{
			if (!calibrate_rx_delay_fsm(&ports[i], -1))
				done = 0;
		}
		usleep(10000);

	} while (!done);
}

#if 0
void gtx_phase_test(int ep, int argc, char *argv[])
{
  int i,j;

	if(	rts_connect(NULL) < 0)
	{
		printf("Can't connect to the RT subsys\n");
		return;
	}

  for(i=0;i<8;i++)
  {
    ports[i].state = TXCAL_ST_STARTUP;
    ports[i].ep = i;
  }

  int done;

  do {
    done = 1;
    for(i=0;i<8;i++)
    {
      if( !calibrate_tx_delay_fsm(&ports[i] ) )
        done = 0;
    }
    usleep(10000);

  } while (!done);

  int full_calib_needed = 0;

  struct config_file *cfg = cfg_open("/wr/etc/gtx-calibration.conf", 0);

  if(!cfg)
  {
    printf("No GTX calibration file found. Creating one!\n");
    cfg = cfg_open("/wr/etc/gtx-calibration.conf", 1);


    full_calib_needed = 1;


    for(i=0;i<8;i++)
    {
      char key[1024];
      snprintf(key, 1024, "tx_bias_port%d", i);
      cfg_set_int(cfg, key, ports[i].tx_phase);
    }
  }

  if (full_calib_needed)
  {
    run_rx_calibration();


    for(i=0;i<8;i++)
    {
      for(j =0;j<20;j++)
      {
        char key[1024];
        snprintf(key, 1024, "rx_adjust_port%d_bitslide%d_valid", i, j);
        cfg_set_int(cfg, key, ports[i].bins[j].samples > RXCAL_MIN_SAMPLES ? 1 : 0);

        snprintf(key, 1024, "rx_adjust_port%d_bitslide%d", i, j);
        cfg_set_int(cfg, key, ports[i].bins[j].acc );
      }
    }


  }

  cfg_save(cfg, "/wr/etc/gtx-calibration.conf");

  run_rx_calibration();
}
#endif

int measure_loobpack_phase(int ep)
{
	struct rts_pll_state pstate;
	rts_enable_ptracker(ep, 0);
	rts_enable_ptracker(ep, 1);
	rts_get_state(&pstate);
	
	while(! (pstate.channels[ep].flags & CHAN_PMEAS_READY ) )
	{
		shw_udelay(10);
		rts_get_state(&pstate);
	}
	
	return pstate.channels[ep].phase_loopback;
}

void phase_test_mode0(int ep)
{

//			shw_sfp_set_generic(ep, 0, SFP_LED_WRMODE1);

			pcs_write( ep, 19, 0 | 2 | (1<<2));

      pcs_write( ep, 0, MDIO_MCR_PDOWN);
      shw_udelay(10);
      pcs_write( ep, 0, MDIO_MCR_LOOPBACK);
      shw_udelay(10);

			pcs_write(ep, 0, pcs_read(ep, 0) | MDIO_MCR_RESET);
			pcs_write(ep, 16, 0);
			shw_udelay(10);
			//shw_sfp_set_tx_disable(ep, 0);
			//shw_sfp_set_generic(ep, 1, SFP_LED_WRMODE1);
			pcs_write(ep, 0, pcs_read(ep, 0) & ~(MDIO_MCR_RESET));
			shw_udelay(10000);

			pcs_write(ep, 19, 1 << 14); // enable TXOUTCCLK clock source for DDMTD

			//printf("DBG1: 0x%x\n", pcs_read(ep, 19));
			//printf("DBG0: 0x%x\n", pcs_read(ep, 18));


			  //printf("BMSR %x %x\n", pcs_read(ep, 1), pcs_read(ep, 0xa));

				int tx_phase = measure_loobpack_phase( ep );

			//int tx_phase = 0;

       pcs_write(ep, 19, 0);
		  	shw_udelay(1000);
      	int loop_phase = measure_loobpack_phase( ep );
				int dbg0_int = pcs_read(ep, 18);
        int bitslide = get_bitslide(ep);

			 //printf("loop phase %d [%d] bs %d\n", loop_phase, loop_phase % 800, bitslide);

        pcs_write( ep, 0, 0);

				shw_udelay(1000);
	      
        int int_loop_phase = measure_loobpack_phase( ep );
          bitslide = get_bitslide(ep);

			int dbg0_ext = pcs_read(ep, 18);
			  //printf("BMSR %x %x\n", pcs_read(ep, 1), pcs_read(ep, 0xa));
			 printf("p %d %d %d %x %x %d\n", tx_phase, loop_phase, int_loop_phase, dbg0_int, dbg0_ext, bitslide);

		
	
}

int force_bitslide(int ep, uint16_t expected_pattern, int loopback)
{
	uint16_t pattern;
	int n = 0;
	do {
		pcs_write( ep, 19, 0 | 2 | (1<<2));

		pcs_write( ep, 0, MDIO_MCR_PDOWN);
		shw_udelay(10);
		pcs_write( ep, 0, loopback ? MDIO_MCR_LOOPBACK : 0);
		shw_udelay(10);

		pcs_write(ep, 0, pcs_read(ep, 0) | MDIO_MCR_RESET);
		pcs_write(ep, 16, 0);
		shw_udelay(10);
		
		pcs_write(ep, 0, pcs_read(ep, 0) & ~(MDIO_MCR_RESET));
		shw_udelay(1000);
		pattern = pcs_read(ep, 18);
		n++;
	} while ( pattern != expected_pattern );

	return n;
}

void phase_test_mode1(int ep)
{
	uint16_t expected_pattern = 0x50bc;
	int n=0;

	n = force_bitslide( ep, expected_pattern, 1 );

	pcs_write(ep, 19, 1 << 14); // enable TXOUTCCLK clock source for DDMTD
	int tx_phase = measure_loobpack_phase( ep );

  pcs_write(ep, 19, 0);
 	shw_udelay(1000);
 	
 	int loop_phase = measure_loobpack_phase( ep );

  //pcs_write( ep, 0, 0);
	n = force_bitslide( ep, expected_pattern, 0 );

	shw_udelay(1000);
  int int_loop_phase = measure_loobpack_phase( ep );

  printf("p %d %d %d %d\n", tx_phase, loop_phase, int_loop_phase,n);
}

#define PCS_DBG_RESET_TX (1<<0)
#define PCS_DBG_RESET_TX_DONE (1<<0)
#define PCS_DBG_TX_ENABLE (1<<1)
#define PCS_DBG_RX_ENABLE (1<<2)
#define PCS_DBG_RESET_RX (1<<3)

#define PCS_DBG_DMTD_SOURCE_TXOUTCLK (1<<14)
#define PCS_DBG_LINK_UP (1<<1)
#define PCS_DBG_LINK_ALIGNED (1<<2)

#define MDIO_DBG1 19
#define MDIO_DBG0 18

int force_tx_phase(int ep, int center, int tolerance)
{
	int n_attempts = 0;

	fprintf(stderr, "Forcing TX phase on EP %d...\n ", ep);
	for (;;)
	{
		n_attempts++;
		pcs_write(ep, MDIO_DBG1, PCS_DBG_DMTD_SOURCE_TXOUTCLK | PCS_DBG_RESET_TX); // enable TXOUTCCLK clock source for DDMTD
		shw_udelay(1000);
		pcs_write(ep, MDIO_DBG1, PCS_DBG_DMTD_SOURCE_TXOUTCLK); // enable TXOUTCCLK clock source for DDMTD
		shw_udelay(1000);

		//	printf("PCS reset done : %d", pcs_read(ep, MDIO_DBG0) & PCS_DBG_RESET_TX_DONE ? 1: 0);

		int tx_phase = measure_loobpack_phase(ep);
		//fprintf(stderr,"txPhase %d\n", tx_phase);

		int n = tx_phase - center + 10000;

		if (n > 10000 - tolerance && n < 10000 + tolerance)
		{
			fprintf(stderr, "locked on %d ps after %d attempts.\n", tx_phase, n_attempts);
			return 1;
		}
	}

	return 0;
	//  printf("p %d\n", tx_phase );
}

int phase_test_mode2(int ep)
{
	// disable the PHY
	pcs_write(ep, MDIO_DBG1, PCS_DBG_RESET_TX); // enable TXOUTCCLK clock source for DDMTD

	force_tx_phase(0, 80, 500);

	for (;;)
	{
		pcs_write(ep, MDIO_DBG1, 0);
		shw_udelay(1000);
		pcs_write(ep, MDIO_DBG1, PCS_DBG_TX_ENABLE);
		shw_udelay(1000);
		int n_attempts = 0;

		for (;;)
		{
			shw_sfp_set_tx_disable(ep, 0);

			n_attempts++;
			pcs_write(ep, MDIO_DBG1, PCS_DBG_TX_ENABLE | PCS_DBG_RESET_RX);
			shw_udelay(1000);
			pcs_write(ep, MDIO_DBG1, PCS_DBG_TX_ENABLE);
			shw_udelay(1000);

			while (!(pcs_read(ep, MDIO_DBG0) & PCS_DBG_LINK_UP))
			{
				shw_udelay(1000);
			}

			//fprintf(stderr,"Link up\n");

			if (pcs_read(ep, MDIO_DBG0) & PCS_DBG_LINK_ALIGNED && n_attempts > 10)
				break;
		}
		//fprintf(stderr,"OK\n");

		int loopback_phase = measure_loobpack_phase(ep);

		fprintf(stderr, "Link aligned after %d attempts, loopback phase = %d ps\n", n_attempts, loopback_phase);
	}
	return 0;
}

void gtx_phase_test(int ep, int argc, char *argv[])
{
	struct rts_pll_state pstate;

	if (argc < 3)
	{
		printf("Mode expected\n");
		return;
	}
	int mode = atoi(argv[3]);

	printf("Mode : %d ep %d\n", mode, ep);

	//	printf("setmode rv %d\n", rts_set_mode(RTS_MODE_GM_FREERUNNING));
	if (rts_connect(NULL) < 0)
	{
		printf("Can't connect to the RT subsys\n");
		return;
	}

	
	printf("Waiting for PLL to lock in master mode\n");
	
		shw_io_init();
		shw_fpga_mmap_init();
		shw_sfp_buses_init();

		rts_set_mode(RTS_MODE_GM_FREERUNNING);

		while (1)
		{
			rts_get_state(&pstate);
			if (pstate.flags & RTS_DMTD_LOCKED)
				break;
			sleep(1);
		}

		shw_sfp_set_tx_disable(ep, 0);

	
		if( mode == 0 )
		{
			for(;;) phase_test_mode0(ep); 
		}
		
		
		if( mode == 1 )
		{ 
			for(;;) phase_test_mode1(ep); 
			 
		}

		if( mode == 2 )
		{ 
			for(;;) phase_test_mode2(ep); 
			 
		}
		return;
}

void ts_comand(int ep, int argc,char *argv[])
{	
	struct rts_pll_state pstate;
	int i;

	assert(hal_shm_init() == 0); /* to get hal_nports_local */

	if(	rts_connect(NULL) < 0)
	{
		printf("Can't connect to the RT subsys\n");
		exit(1);
	}

	rts_get_state(&pstate);
	
	if (!strcmp(argv[3], "track"))
	{
		printf("Enabling phase logging @ port %d (wri%d)\n", ep + 1, ep + 1);
		rts_enable_ptracker_log(ep, 1);
	}
	else if (!strcmp(argv[3], "set_navg"))
	{
		int navg = atoi(argv[4]);

		printf("Setting navg @ port %d (wri%d) navg =  %d\n,", ep + 1, ep + 1,navg);
		rts_ptracker_set_average_samples(ep, navg);
	}

}

void rt_command(int ep, int argc, char *argv[])
{
/* ep is 0..17 */

	struct rts_pll_state pstate;
	int i;

	assert(hal_shm_init() == 0); /* to get hal_nports_local */

	if(	rts_connect(NULL) < 0)
	{
		printf("Can't connect to the RT subsys\n");
		exit(1);
	}

	rts_get_state(&pstate);

	if(!strcmp(argv[3], "show"))
	{
		printf("RTS State Dump [%d physical ports]:\n",
		       hal_nports_local);
		printf("CurrentRef: %d Mode: %s (%d) Flags: %x\n", pstate.current_ref, getRtsTimingMode(pstate.mode), pstate.mode, pstate.flags);
		for (i = 0; i < hal_nports_local; i++)
			printf("wri%-2d: setpoint: %-8dps current: %-8dps "
			       "loopback: %-8dps flags: %x\n", i + 1,
			       pstate.channels[i].phase_setpoint,
			       pstate.channels[i].phase_current,
			       pstate.channels[i].phase_loopback,
			       pstate.channels[i].flags);
	} else if (!strcmp(argv[3], "lock"))
	{
		printf("locking to: port %d wri%d\n", ep + 1, ep + 1);
		rts_get_state(&pstate);
		printf("setmode rv %d\n", rts_set_mode(RTS_MODE_BC));
		printf("lock rv %d\n", rts_lock_channel(ep, 0));
	} else if (!strcmp(argv[3], "fr"))
	{
		printf("Enabling free-running master timing mode\n");
		printf("rv: %d\n", rts_set_mode(RTS_MODE_GM_FREERUNNING));
	}
	else if (!strcmp(argv[3], "gm"))
	{
		printf("Enabling grand master timing mode\n");
		printf("rv: %d\n", rts_set_mode(RTS_MODE_GM_EXTERNAL));
	}
	else if (!strcmp(argv[3], "ds"))
	{
		printf("Disable timing mode\n");
		printf("rv: %d\n", rts_set_mode(RTS_MODE_DISABLED));
	}else if (!strcmp(argv[3], "track"))
	{
		printf("Enabling ptracker @ port %d (wri%d)\n", ep + 1, ep + 1);
		rts_enable_ptracker(ep, 1);
	}else if (!strcmp(argv[3], "trackrge"))
	{
		printf("Enabling ptracker @ port %d (wri%d)\n", ep + 1, ep + 1);
		rts_enable_ptracker_log(ep, 1);
	}
}

int measure_t24p(int endpoint, uint32_t *value);

void cmd_t24p_new(int ep, int argc, char *argv[])
{
	uint32_t value;
	measure_t24p( ep, &value );
}


struct {
	char *cmd;
	char *params;
	char *desc;
	void (*func)(int, int, char *argv[]);
} commands[] = {
	{
	"txcal",
	"[0/1]",
	"enable/disable transmission of a calibration pattern",
	tx_cal},
	{
	"autoneg",
	"",
	"perform autonegotiation procedure",
	try_autonegotiation},

	{
	"t24p",
	"",
	"determine transition point (new algo)",
	cmd_t24p_new},

	{
	"dump",
	"",
	"dump PCS regs ",
	dump_pcs_regs},
	{
	"wr",
	"",
	"write PCS reg ",
	write_pcs_reg},

	{
	"analyze",
	"log_file [transition_point]",
	"analyze phase log",
	analyze_phase_log},

	{
	"ppsadj",
	"",
	"PPS adjustment test",
	pps_adjustment_test},
	
	{
        "gtx_phase",
	"[0/1/2]",
	"gtx phase test [test modes: 0,1,2]",
	gtx_phase_test},

	{
	"rt",
	"",
	"RT subsystem command [show,lock,[gm,fr,ds],track]",
	rt_command},
	{
	"ts",
	"",
	"Timescale comand usage ts ep [track ,set_navg [avg_samples]]",
	ts_comand
	},
	{NULL}

};



/* New calibrator for the transition phase value. A major pain in the ass for
   the folks who frequently rebuild their gatewares. The idea is described
   below:
   - lock the PLL to the master
   - scan the whole phase shifter range
   - at each scanning step, generate a fake RX timestamp.
   - check if the rising edge counter is ahead of the falling edge counter
     (added a special bit for it in the TSU).
   - determine phases at which positive/negative transitions occur
   - transition phase value is in the middle between the rising and falling
     edges.
   
   This calibration procedure is fast enough to be run on slave nodes whenever
   the link goes up. For master mode, the core must be run at least once as a
   slave to calibrate itself and store the current transition phase value in
   the EEPROM.
*/

/* how finely we scan the phase shift range to determine where we have the bit
 * flip */
#define CAL_SCAN_STEP 100

/* deglitcher threshold (to remove 1->0->1 flip bit glitches that might occur
   due to jitter) */
#define CAL_DEGLITCH_THRESHOLD 5

/* we scan at least one clock period to look for rising->falling edge transition
   plus some headroom */
#define CAL_SCAN_RANGE (REF_CLOCK_PERIOD_PS + \
		(3 * CAL_DEGLITCH_THRESHOLD * CAL_SCAN_STEP))

#define TD_WAIT_INACTIVE	0
#define TD_GOT_TRANSITION	1
#define TD_DONE			2

/* Number of retries for rxts_calibration_update
 * value found experimentally */
#define CALIB_RETRIES 1000


/* state of transition detector */
struct trans_detect_state {
	int prev_val;
	int sample_count;
	int state;
	int trans_phase;
};

/* finds the transition in the value of flip_bit and returns phase associated
   with it. If no transition phase has been found yet, returns 0. Non-zero
   polarity means we are looking for positive transitions, 0 - negative
   transitions */
static int lookup_transition(struct trans_detect_state *state, int flip_bit,
			     int phase, int polarity)
{
	if (polarity)
		polarity = 1;

	switch (state->state) {
	case TD_WAIT_INACTIVE:
		/* first, wait until we have at least CAL_DEGLITCH_THRESHOLD of
		   inactive state samples */
		if (flip_bit != polarity)
			state->sample_count++;
		else
			state->sample_count = 0;

		if (state->sample_count >= CAL_DEGLITCH_THRESHOLD) {
			state->state = TD_GOT_TRANSITION;
			state->sample_count = 0;
		}

		break;

	case TD_GOT_TRANSITION:
		if (flip_bit != polarity)
			state->sample_count = 0;
		else {
			state->sample_count++;
			if (state->sample_count >= CAL_DEGLITCH_THRESHOLD) {
				state->state = TD_DONE;
				state->trans_phase =
				    phase -
				    CAL_DEGLITCH_THRESHOLD * CAL_SCAN_STEP;
			}
		}
		break;

	case TD_DONE:
		return 1;
		break;
	}
	return 0;
}

static struct trans_detect_state det_rising, det_falling;
static int cal_cur_phase;
static int cal_endpoint;
static struct rts_pll_state cal_pstate;


static void rxts_calibration_init(int endpoint)
{
	if(	rts_connect(NULL) < 0)
	{
		printf("Can't connect to the RT subsys\n");
		exit(1);
	}

	cal_endpoint = endpoint;
}

static void cal_spll_set_phase_shift(int endpoint, int shift )
{
	printf("adjp ep %d shift %d\n", endpoint, shift );
	rts_adjust_phase(endpoint, shift);
}

static int cal_spll_shifter_busy()
{
	rts_get_state(&cal_pstate);
	
	return ( cal_pstate.channels[cal_endpoint].flags & CHAN_SHIFTING ? 1 : 0);
}

/* Starts RX timestamper calibration process state machine. Invoked by
   ptpnetif's check lock function when the PLL has already locked, to avoid
   complicating the API of ptp-noposix/ppsi. */

void rxts_calibration_start(void)
{
	cal_cur_phase = 0;
	det_rising.prev_val = det_falling.prev_val = -1;
	det_rising.state = det_falling.state = TD_WAIT_INACTIVE;
	det_rising.sample_count = 0;
	det_falling.sample_count = 0;
	det_rising.trans_phase = 0;
	det_falling.trans_phase = 0;
	cal_spll_set_phase_shift(cal_endpoint, 0);
}

/* Updates RX timestamper state machine. Non-zero return value means that
   calibration is done. */
int rxts_calibration_update(uint32_t *t24p_value)
{
	int32_t ttrans = 0;

	if (cal_spll_shifter_busy(cal_endpoint))
		return 0;

	/* generate a fake RX timestamp and check if falling edge counter is
	   ahead of rising edge counter */
	int flip = cal_ep_timestamper_cal_pulse(cal_endpoint);

	/* look for transitions (with deglitching) */
	lookup_transition(&det_rising, flip, cal_cur_phase, 1);
	lookup_transition(&det_falling, flip, cal_cur_phase, 0);

	if (cal_cur_phase >= CAL_SCAN_RANGE) {
		if (det_rising.state != TD_DONE || det_falling.state != TD_DONE) 
		{
			printf("RXTS calibration error.\n");
			return -1;
		}

		/* normalize */
		while (det_falling.trans_phase >= REF_CLOCK_PERIOD_PS)
			det_falling.trans_phase -= REF_CLOCK_PERIOD_PS;
		while (det_rising.trans_phase >= REF_CLOCK_PERIOD_PS)
			det_rising.trans_phase -= REF_CLOCK_PERIOD_PS;

		/* Use falling edge as second sample of rising edge */
		if (det_falling.trans_phase > det_rising.trans_phase)
			ttrans = det_falling.trans_phase - REF_CLOCK_PERIOD_PS/2;
		else if(det_falling.trans_phase < det_rising.trans_phase)
			ttrans = det_falling.trans_phase + REF_CLOCK_PERIOD_PS/2;
		ttrans += det_rising.trans_phase;
		ttrans /= 2;

		/*normalize ttrans*/
		if(ttrans < 0) ttrans += REF_CLOCK_PERIOD_PS;
		if(ttrans >= REF_CLOCK_PERIOD_PS) ttrans -= REF_CLOCK_PERIOD_PS;


		printf("RXTS calibration: R@%dps, F@%dps, transition@%dps\n",
			  det_rising.trans_phase, det_falling.trans_phase,
			  ttrans);

		*t24p_value = (uint32_t)ttrans;
		return 1;
	}

	printf("Try phase %d flip %d\n", cal_cur_phase, flip );
	cal_cur_phase += CAL_SCAN_STEP;

	

	cal_spll_set_phase_shift(cal_endpoint, cal_cur_phase);

	return 0;
}

int cal_ep_timestamper_cal_pulse()
{
	uint32_t tscr = fpga_readl( IDX_TO_EP(cal_endpoint) + EP_REG(TSCR) ); 
	
	tscr |= EP_TSCR_RX_CAL_START;

	fpga_writel(tscr, IDX_TO_EP(cal_endpoint) + EP_REG(TSCR));
	shw_udelay(1000);

	tscr = fpga_readl( IDX_TO_EP(cal_endpoint) + EP_REG(TSCR) ); 
	return tscr & EP_TSCR_RX_CAL_RESULT ? 1 : 0;
}


/* legacy function for 'calibration force' command */
int measure_t24p(int endpoint, uint32_t *value)
{
	int rv, initial_mode, initial_current_ref;

	printf("Measuring t24p for endpoint %d\n", endpoint );

	rxts_calibration_init( endpoint );
	
	/* read/remember mode/reference before the measurment,
	   to be restored at the end */
	rts_get_state(&cal_pstate);
	initial_mode = cal_pstate.mode;
	initial_current_ref = cal_pstate.current_ref;
	

	printf("Waiting for link...\n");
	while(! (pcs_read(cal_endpoint, MII_BMSR) & BMSR_LSTATUS))
	{
		shw_udelay(1000);
	}
			
	rts_set_mode( RTS_MODE_BC );
	rts_lock_channel( endpoint, 0 );
	printf("Locking PLL...\n");
	while (1)
	{
		rts_get_state(&cal_pstate);
		//printf("PState flags %x\n", cal_pstate.flags );
		if (cal_pstate.flags & RTS_REF_LOCKED)
			break;
		shw_udelay(100000);
	}
	printf("\n");

	printf("Calibrating RX timestamper...\n");
	rxts_calibration_start();

	while (!(rv = rxts_calibration_update(value))) ;

	/* Go back to old settings of reference while still in BC mode */
	rts_lock_channel( initial_current_ref, 0 );

	/* Go back to the previous mode of PLL, only if it's not BC mode
	   NOTE: the rts_set_mode() function initializes softPLL to
	   DISABLED mode if it is called with RTS_MODE_BC argument), it
	   needs to be followed by rts_lock_channel() function which
	   initializes spll with SLAVE mode... */
	if (initial_mode != RTS_MODE_BC)
		rts_set_mode( initial_mode );

	return rv;
}


int main(int argc, char **argv)
{
	int i;
	int ep;

	wrs_msg_init(1, argv, LOG_USER); /* only use argv[0]: no cmdline */

	if(argc<3)
	{

		printf("phytool - GTX serdes testing program\n");
		printf("usage: %s endpoint command [parameters]\n", argv[0]);
		printf("Commands:\n");
		for(i=0; commands[i].cmd;i++)
			printf("%-20s %-20s: %s\n", commands[i].cmd, commands[i].params, commands[i].desc);
		return 0;
	}

	fpga = create_map(WRS3_FPGA_BASE, WRS3_FPGA_SIZE);
	if (!fpga) {
		printf("%s: Unable to mmap\n", argv[0]);
		exit(1);
	}

	for(i=0; commands[i].cmd;i++)
		if(!strcmp(commands[i].cmd, argv[2]))
		{
			/* first parameter is an enpoint number 1..18 */
			ep = atoi(argv[1]) - 1;
			if (ep < 0) {
				printf("Wrong endpoint number %d\n", ep + 1);
				exit(1);
			}
			/* pass endpoint as number 0..17 */
			commands[i].func(ep, argc, argv);
			return 0;
		}

	printf("Unrecognized command '%s'\n", argv[2]);
	return -1;
}
