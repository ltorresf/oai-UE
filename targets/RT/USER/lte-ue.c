/*
 * Licensed to the OpenAirInterface (OAI) Software Alliance under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The OpenAirInterface Software Alliance licenses this file to You under
 * the OAI Public License, Version 1.0  (the "License"); you may not use this file
 * except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.openairinterface.org/?page_id=698
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *-------------------------------------------------------------------------------
 * For more information about the OpenAirInterface (OAI) Software Alliance:
 *      contact@openairinterface.org
 */

/*! \file lte-ue.c
 * \brief threads and support functions for real-time LTE UE target
 * \author R. Knopp, F. Kaltenberger, Navid Nikaein
 * \date 2015
 * \version 0.1
 * \company Eurecom
 * \email: knopp@eurecom.fr,florian.kaltenberger@eurecom.fr, navid.nikaein@eurecom.fr
 * \note
 * \warning
 */
#include "lte-softmodem.h"

#include "rt_wrapper.h"

#ifdef OPENAIR2
#include "LAYER2/MAC/defs.h"
#include "RRC/LITE/extern.h"
#endif
#include "PHY_INTERFACE/extern.h"

#undef MALLOC //there are two conflicting definitions, so we better make sure we don't use it at all
//#undef FRAME_LENGTH_COMPLEX_SAMPLES //there are two conflicting definitions, so we better make sure we don't use it at all

#include "PHY/extern.h"
#include "SCHED/extern.h"
#include "LAYER2/MAC/extern.h"
#include "LAYER2/MAC/proto.h"

#include "UTIL/LOG/log_extern.h"
#include "UTIL/OTG/otg_tx.h"
#include "UTIL/OTG/otg_externs.h"
#include "UTIL/MATH/oml.h"
#include "UTIL/LOG/vcd_signal_dumper.h"
#include "UTIL/OPT/opt.h"

#include "T.h"

extern double cpuf;

#define FRAME_PERIOD    100000000ULL
#define DAQ_PERIOD      66667ULL
#define FIFO_PRIORITY   40

typedef enum {
    pss=0,
    pbch=1,
    si=2
} sync_mode_t;

void init_UE_threads(PHY_VARS_UE *UE);
void *UE_thread(void *arg);
void init_UE(int nb_inst);

int32_t **rxdata;
int32_t **txdata;

#define KHz (1000UL)
#define MHz (1000*KHz)

typedef struct eutra_band_s {
    int16_t band;
    uint32_t ul_min;
    uint32_t ul_max;
    uint32_t dl_min;
    uint32_t dl_max;
    lte_frame_type_t frame_type;
} eutra_band_t;

typedef struct band_info_s {
    int nbands;
    eutra_band_t band_info[100];
} band_info_t;

band_info_t bands_to_scan;

static const eutra_band_t eutra_bands[] = {
    { 1, 1920    * MHz, 1980    * MHz, 2110    * MHz, 2170    * MHz, FDD},
    { 2, 1850    * MHz, 1910    * MHz, 1930    * MHz, 1990    * MHz, FDD},
    { 3, 1710    * MHz, 1785    * MHz, 1805    * MHz, 1880    * MHz, FDD},
	//{ 3, 1710    * MHz, 1785    * MHz, 1835    * MHz, 1855    * MHz, FDD},
    { 4, 1710    * MHz, 1755    * MHz, 2110    * MHz, 2155    * MHz, FDD},
    { 5,  824    * MHz,  849    * MHz,  869    * MHz,  894    * MHz, FDD},
    { 6,  830    * MHz,  840    * MHz,  875    * MHz,  885    * MHz, FDD},
    { 7, 2500    * MHz, 2570    * MHz, 2620    * MHz, 2690    * MHz, FDD},
    { 8,  880    * MHz,  915    * MHz,  925    * MHz,  960    * MHz, FDD},
    { 9, 1749900 * KHz, 1784900 * KHz, 1844900 * KHz, 1879900 * KHz, FDD},
    {10, 1710    * MHz, 1770    * MHz, 2110    * MHz, 2170    * MHz, FDD},
    {11, 1427900 * KHz, 1452900 * KHz, 1475900 * KHz, 1500900 * KHz, FDD},
    {12,  698    * MHz,  716    * MHz,  728    * MHz,  746    * MHz, FDD},
    {13,  777    * MHz,  787    * MHz,  746    * MHz,  756    * MHz, FDD},
    {14,  788    * MHz,  798    * MHz,  758    * MHz,  768    * MHz, FDD},
    {17,  704    * MHz,  716    * MHz,  734    * MHz,  746    * MHz, FDD},
    {20,  832    * MHz,  862    * MHz,  791    * MHz,  821    * MHz, FDD},
    {22, 3510    * MHz, 3590    * MHz, 3410    * MHz, 3490    * MHz, FDD},
    {33, 1900    * MHz, 1920    * MHz, 1900    * MHz, 1920    * MHz, TDD},
    {34, 2010    * MHz, 2025    * MHz, 2010    * MHz, 2025    * MHz, TDD},
    {35, 1850    * MHz, 1910    * MHz, 1850    * MHz, 1910    * MHz, TDD},
    {36, 1930    * MHz, 1990    * MHz, 1930    * MHz, 1990    * MHz, TDD},
    {37, 1910    * MHz, 1930    * MHz, 1910    * MHz, 1930    * MHz, TDD},
    {38, 2570    * MHz, 2620    * MHz, 2570    * MHz, 2630    * MHz, TDD},
    {39, 1880    * MHz, 1920    * MHz, 1880    * MHz, 1920    * MHz, TDD},
    {40, 2300    * MHz, 2400    * MHz, 2300    * MHz, 2400    * MHz, TDD},
    {41, 2496    * MHz, 2690    * MHz, 2496    * MHz, 2690    * MHz, TDD},
    {42, 3400    * MHz, 3600    * MHz, 3400    * MHz, 3600    * MHz, TDD},
    {43, 3600    * MHz, 3800    * MHz, 3600    * MHz, 3800    * MHz, TDD},
    {44, 703    * MHz, 803    * MHz, 703    * MHz, 803    * MHz, TDD},
};

void init_thread(int sched_runtime, int sched_deadline, int sched_fifo, cpu_set_t *cpuset, char * name) {
	int procID_init_thread = gettid();

#ifdef DEADLINE_SCHEDULER
    if (sched_runtime!=0) {
        struct sched_attr attr= {0};
        attr.size = sizeof(attr);
        attr.sched_policy = SCHED_DEADLINE;
        attr.sched_runtime  = sched_runtime;
        attr.sched_deadline = sched_deadline;
        attr.sched_period   = 0;
        AssertFatal(sched_setattr(0, &attr, 0) == 0,
                    "[SCHED] %s thread: sched_setattr failed %s \n", name, strerror(errno));
        LOG_I(HW,"[SCHED][eNB] %s deadline thread %lu started on CPU %d\n",
              name, (unsigned long)gettid(), sched_getcpu());
    }

#else
    if (CPU_COUNT(cpuset) > 0)
        AssertFatal( 0 == pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), cpuset), "");
    struct sched_param sp;
    sp.sched_priority = sched_fifo;
    AssertFatal(pthread_setschedparam(pthread_self(),SCHED_FIFO,&sp)==0,
                "Can't set thread priority, Are you root?\n");
    /* Check the actual affinity mask assigned to the thread */
    cpu_set_t *cset=CPU_ALLOC(CPU_SETSIZE);
    if (0 == pthread_getaffinity_np(pthread_self(), CPU_ALLOC_SIZE(CPU_SETSIZE), cset)) {
      char txt[512]={0};
      for (int j = 0; j < CPU_SETSIZE; j++)
        if (CPU_ISSET(j, cset))
	  sprintf(txt+strlen(txt), " %d ", j);
      LOG_I(PHY,"[PID-%d] CPU Affinity of thread %s is %s\n",procID_init_thread, name, txt);
    }
    CPU_FREE(cset);
#endif

    // Lock memory from swapping. This is a process wide call (not constraint to this thread).
    mlockall(MCL_CURRENT | MCL_FUTURE);
    pthread_setname_np( pthread_self(), name );

    // LTS: this sync stuff should be wrong
    //printf("waiting for sync (%s)\n",name);
    LOG_I(PHY,"[PID-%d] Waiting for sync (%s)\n",procID_init_thread,name);
    pthread_mutex_lock(&sync_mutex);
    //printf("Locked sync_mutex, waiting (%s)\n",name);
    LOG_I(PHY,"[PID-%d] Locked sync_mutex, waiting (%s)\n",procID_init_thread,name);
    while (sync_var<0)
        pthread_cond_wait(&sync_cond, &sync_mutex);
    pthread_mutex_unlock(&sync_mutex);
    //printf("started %s as PID: %ld\n",name, gettid());
    LOG_I(PHY,"[PID-%d] started %s as PID: %ld\n",procID_init_thread,name, gettid());
}

void init_UE(int nb_inst)
{
  int inst;
  for (inst=0; inst < nb_inst; inst++) {
    //    UE->rfdevice.type      = NONE_DEV;
    PHY_VARS_UE *UE = PHY_vars_UE_g[inst][0];
    AssertFatal(0 == pthread_create(&UE->proc.pthread_ue,
                                    &UE->proc.attr_ue,
                                    UE_thread,
                                    (void*)UE), "");
  }

  printf("UE threads created by %ld\n", gettid());
#if 0
#if defined(ENABLE_USE_MME)
  extern volatile int start_UE;
  while (start_UE == 0) {
    sleep(1);
  }
#endif
#endif
}


int freq_offset=0,carrier_offset=0;//LA
/*!
 * \brief This is the UE synchronize thread.
 * It performs band scanning and synchonization.
 * \param arg is a pointer to a \ref PHY_VARS_UE structure.
 * \returns a pointer to an int. The storage is not on the heap and must not be freed.
 */
static void *UE_thread_synch(void *arg) {
	int procID_sync = gettid(); //LA
	printf("**************************************************** Start : [UE_thread_synch] [PID: %d] ****************************************************\n",procID_sync);
    static int __thread UE_thread_synch_retval;
    int i, hw_slot_offset;
    PHY_VARS_UE *UE = (PHY_VARS_UE*) arg;
    int current_band = 0;
    int current_offset = 0;
    sync_mode_t sync_mode = pbch;
    int CC_id = UE->CC_id;
    int	set_band = 0;
    //CC_id = 1;
//    int freq_offset=0,carrier_offset=0;//LA
    char threadname[128];


    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    if ( threads.iq != -1 )
        CPU_SET(threads.iq, &cpuset);
    // this thread priority must be lower that the main acquisition thread
    //sprintf(threadname, "sync UE %d\n", UE->Mod_id);
    sprintf(threadname, "sync_UE_%d", UE->Mod_id);
    init_thread(100000, 500000, FIFO_PRIORITY-1, &cpuset, threadname);

    UE->is_synchronized = 0;

    if (UE->UE_scan == 0) {
    	LOG_I(PHY,"[%d] Band scanning disabled. (UE->UE_scan = 0).\n",procID_sync);
        int ind;
        for ( ind=0;
                ind < sizeof(eutra_bands) / sizeof(eutra_bands[0]);
                ind++) {
            current_band = eutra_bands[ind].band;
            LOG_D(PHY, "Scanning band %d, dl_min %"PRIu32", ul_min %"PRIu32"\n", current_band, eutra_bands[ind].dl_min,eutra_bands[ind].ul_min);
            if ( eutra_bands[ind].dl_min <= downlink_frequency[0][0] && eutra_bands[ind].dl_max >= downlink_frequency[0][0] ) {
                for (i=0; i<4; i++)
                    uplink_frequency_offset[CC_id][i] = eutra_bands[ind].ul_min - eutra_bands[ind].dl_min;
                break;
            }
        }
        AssertFatal( ind < sizeof(eutra_bands) / sizeof(eutra_bands[0]), "Can't find EUTRA band for frequency");

        LOG_I( PHY, "[%d] [SCHED][UE] Check absolute frequency DL %"PRIu32", UL %"PRIu32" (oai_exit %d, rx_num_channels %d)\n", procID_sync,
               downlink_frequency[0][0], downlink_frequency[0][0]+uplink_frequency_offset[0][0],
               oai_exit, openair0_cfg[0].rx_num_channels);

        for (i=0; i<openair0_cfg[UE->rf_map.card].rx_num_channels; i++) {
            openair0_cfg[UE->rf_map.card].rx_freq[UE->rf_map.chain+i] = downlink_frequency[CC_id][i];
            openair0_cfg[UE->rf_map.card].tx_freq[UE->rf_map.chain+i] =
                downlink_frequency[CC_id][i]+uplink_frequency_offset[CC_id][i];
            openair0_cfg[UE->rf_map.card].autocal[UE->rf_map.chain+i] = 1;
            if (uplink_frequency_offset[CC_id][i] != 0) //
                openair0_cfg[UE->rf_map.card].duplex_mode = duplex_mode_FDD;
            else //FDD
                openair0_cfg[UE->rf_map.card].duplex_mode = duplex_mode_TDD;
        }
        sync_mode = pbch;

    } else {
    	LOG_I(PHY,"[%d] Band scanning enabled. (UE->UE_scan = 1).\n",procID_sync);
        current_band=0;
        for (i=0; i<openair0_cfg[UE->rf_map.card].rx_num_channels; i++) {
            //downlink_frequency[UE->rf_map.card][UE->rf_map.chain+i] = bands_to_scan.band_info[CC_id].dl_min;
            //uplink_frequency_offset[UE->rf_map.card][UE->rf_map.chain+i] = bands_to_scan.band_info[CC_id].ul_min-bands_to_scan.band_info[CC_id].dl_min;
            downlink_frequency[UE->rf_map.card][UE->rf_map.chain+i] = bands_to_scan.band_info[set_band].dl_min;
            uplink_frequency_offset[UE->rf_map.card][UE->rf_map.chain+i] = bands_to_scan.band_info[set_band].ul_min-bands_to_scan.band_info[set_band].dl_min;
            openair0_cfg[UE->rf_map.card].rx_freq[UE->rf_map.chain+i] = downlink_frequency[CC_id][i];
            openair0_cfg[UE->rf_map.card].tx_freq[UE->rf_map.chain+i] = downlink_frequency[CC_id][i]+uplink_frequency_offset[CC_id][i];
            openair0_cfg[UE->rf_map.card].rx_gain[UE->rf_map.chain+i] = UE->rx_total_gain_dB;

            //LA:
            LOG_I(PHY, "[%d] UE_scan = %d. i = %d, downlink_frequency[%d][%d] = %"PRIu32" MHz.\n",procID_sync,UE->UE_scan,i,(UE->rf_map.card),(UE->rf_map.chain+i),downlink_frequency[UE->rf_map.card][UE->rf_map.chain+i]/1000000);
            LOG_I(PHY, "[%d] UE_scan = %d. i = %d, uplink_frequency_offset[%d][%d] = %"PRIi32" MHz.\n",procID_sync,UE->UE_scan,i,(UE->rf_map.card),(UE->rf_map.chain+i),uplink_frequency_offset[UE->rf_map.card][UE->rf_map.chain+i]/1000000);
            LOG_I(PHY, "[%d] UE_scan = %d. i = %d, openair0_cfg[%d].rx_freq[%d] = %f MHz.\n",procID_sync,UE->UE_scan,i,(UE->rf_map.card),(UE->rf_map.chain+i),openair0_cfg[UE->rf_map.card].rx_freq[UE->rf_map.chain+i]/1000000);
            LOG_I(PHY, "[%d] UE_scan = %d. i = %d, openair0_cfg[%d].tx_freq[%d] = %f MHz.\n",procID_sync,UE->UE_scan,i,(UE->rf_map.card),(UE->rf_map.chain+i),openair0_cfg[UE->rf_map.card].tx_freq[UE->rf_map.chain+i]/1000000);
            LOG_I(PHY, "[%d] UE_scan = %d. i = %d, openair0_cfg[%d].rx_gain[%d] = %f.\n",procID_sync,UE->UE_scan,i,(UE->rf_map.card),(UE->rf_map.chain+i),openair0_cfg[UE->rf_map.card].rx_gain[UE->rf_map.chain+i]);

/*            LOG_I(PHY, "[%d] UE_scan = %d. i = %d, bands_to_scan.band_info[%d].band = %d.\n",procID_sync,UE->UE_scan,i,CC_id,bands_to_scan.band_info[CC_id].band);
            LOG_I(PHY, "[%d] UE_scan = %d. i = %d, bands_to_scan.band_info[%d].ul_min = %"PRIu32" MHz.\n",procID_sync,UE->UE_scan,i,CC_id,bands_to_scan.band_info[CC_id].ul_min/1000000);
            LOG_I(PHY, "[%d] UE_scan = %d. i = %d, bands_to_scan.band_info[%d].ul_max = %"PRIu32" MHz.\n",procID_sync,UE->UE_scan,i,CC_id,bands_to_scan.band_info[CC_id].ul_max/1000000);
            LOG_I(PHY, "[%d] UE_scan = %d. i = %d, bands_to_scan.band_info[%d].dl_min = %"PRIu32" MHz.\n",procID_sync,UE->UE_scan,i,CC_id,bands_to_scan.band_info[CC_id].dl_min/1000000);
            LOG_I(PHY, "[%d] UE_scan = %d. i = %d, bands_to_scan.band_info[%d].dl_max = %"PRIu32" MHz.\n",procID_sync,UE->UE_scan,i,CC_id,bands_to_scan.band_info[CC_id].dl_max/1000000);
            LOG_I(PHY, "[%d] UE_scan = %d. i = %d, bands_to_scan.band_info[%d].frame_type = %d.\n",procID_sync,UE->UE_scan,i,CC_id,bands_to_scan.band_info[CC_id].frame_type); */

            LOG_I(PHY, "[%d] UE_scan = %d. i = %d, bands_to_scan.band_info[%d].band = %d.\n",procID_sync,UE->UE_scan,i,set_band,bands_to_scan.band_info[set_band].band);
			LOG_I(PHY, "[%d] UE_scan = %d. i = %d, bands_to_scan.band_info[%d].ul_min = %"PRIu32" MHz.\n",procID_sync,UE->UE_scan,i,set_band,bands_to_scan.band_info[set_band].ul_min/1000000);
			LOG_I(PHY, "[%d] UE_scan = %d. i = %d, bands_to_scan.band_info[%d].ul_max = %"PRIu32" MHz.\n",procID_sync,UE->UE_scan,i,set_band,bands_to_scan.band_info[set_band].ul_max/1000000);
			LOG_I(PHY, "[%d] UE_scan = %d. i = %d, bands_to_scan.band_info[%d].dl_min = %"PRIu32" MHz.\n",procID_sync,UE->UE_scan,i,set_band,bands_to_scan.band_info[set_band].dl_min/1000000);
			LOG_I(PHY, "[%d] UE_scan = %d. i = %d, bands_to_scan.band_info[%d].dl_max = %"PRIu32" MHz.\n",procID_sync,UE->UE_scan,i,set_band,bands_to_scan.band_info[set_band].dl_max/1000000);
			LOG_I(PHY, "[%d] UE_scan = %d. i = %d, bands_to_scan.band_info[%d].frame_type = %d.\n",procID_sync,UE->UE_scan,i,set_band,bands_to_scan.band_info[set_band].frame_type);

            //LOG_I( PHY, "[%d] Bandwith of signal: %"PRIu32" MHz.\n",procID_sync,(bands_to_scan.band_info[CC_id].dl_max-bands_to_scan.band_info[CC_id].dl_min)/1000000);
            LOG_I( PHY, "[%d] Bandwith of signal: %"PRIu32" MHz.\n",procID_sync,(bands_to_scan.band_info[set_band].dl_max-bands_to_scan.band_info[set_band].dl_min)/1000000);
            LOG_I(PHY,"[%d] samples_per_tti = %d.\n",procID_sync,UE->frame_parms.samples_per_tti);

            //LA:Setting the new Tx/Rx frequency values in the USRP configuration
            UE->rfdevice.trx_set_freq_func(&UE->rfdevice,&openair0_cfg[0],0);
        }
    }

    //    AssertFatal(UE->rfdevice.trx_start_func(&UE->rfdevice) == 0, "Could not start the device\n");

    while (oai_exit==0) {
        AssertFatal ( 0== pthread_mutex_lock(&UE->proc.mutex_synch), "");
        while (UE->proc.instance_cnt_synch < 0)
            // the thread waits here most of the time
            pthread_cond_wait( &UE->proc.cond_synch, &UE->proc.mutex_synch );
        AssertFatal ( 0== pthread_mutex_unlock(&UE->proc.mutex_synch), "");

        switch (sync_mode) {
        case pss:
        {
        	LOG_I(PHY,">>>>>>>>>>>>>>>>>>>>>>>>>> [%d] Start case: PSS <<<<<<<<<<<<<<<<<<<<<<<<<<\n",procID_sync);//LA
            LOG_I(PHY,"[%d] [SCHED][UE] Scanning band %d (%d), freq %u\n",procID_sync,bands_to_scan.band_info[current_band].band, current_band,bands_to_scan.band_info[current_band].dl_min+current_offset);
            lte_sync_timefreq(UE,current_band,bands_to_scan.band_info[current_band].dl_min+current_offset);
            current_offset += 20000000; // increase by 20 MHz

            if (current_offset > bands_to_scan.band_info[current_band].dl_max-bands_to_scan.band_info[current_band].dl_min) {
                current_band++;
                current_offset=0;
            }

            if (current_band==bands_to_scan.nbands) {
                current_band=0;
                oai_exit=1;
            }

            for (i=0; i<openair0_cfg[UE->rf_map.card].rx_num_channels; i++) {
                downlink_frequency[UE->rf_map.card][UE->rf_map.chain+i] = bands_to_scan.band_info[current_band].dl_min+current_offset;
                uplink_frequency_offset[UE->rf_map.card][UE->rf_map.chain+i] = bands_to_scan.band_info[current_band].ul_min-bands_to_scan.band_info[0].dl_min + current_offset;

                openair0_cfg[UE->rf_map.card].rx_freq[UE->rf_map.chain+i] = downlink_frequency[CC_id][i];
                openair0_cfg[UE->rf_map.card].tx_freq[UE->rf_map.chain+i] = downlink_frequency[CC_id][i]+uplink_frequency_offset[CC_id][i];
                openair0_cfg[UE->rf_map.card].rx_gain[UE->rf_map.chain+i] = UE->rx_total_gain_dB;
                if (UE->UE_scan_carrier) {
                    openair0_cfg[UE->rf_map.card].autocal[UE->rf_map.chain+i] = 1;
                }

            }
            LOG_I(PHY,">>>>>>>>>>>>>>>>>>>>>>>>>> [%d] End case: PSS <<<<<<<<<<<<<<<<<<<<<<<<<<\n",procID_sync);//LA
            break;
        }
        case pbch:
        {
        	printf(">>>>>>>>>>>>>>>>>>>>>>>>>> [%d] Start case: PBCH <<<<<<<<<<<<<<<<<<<<<<<<<<\n",procID_sync);//LA
#if DISABLE_LOG_X
            printf("[UE thread Synch] Running Initial Synch (mode %d)\n",UE->mode);
#else
//LA1            LOG_I(PHY, "[%d] Running Initial Synch (UE->mode = %d) [0:normal_txrx].\n",procID_sync,UE->mode);
#endif
//LA            LOG_I(PHY,"[%d] samples_per_tti = %d.\n",procID_sync,UE->frame_parms.samples_per_tti);
            if (initial_sync( UE, UE->mode ) == 0) {	//LA: Initial synchronization succeed
            		//LA: Once it is synchronized, the main change is that the parameter UE->is_synchronized changes from 0 to 1. Then this loop is no longer executed.
                hw_slot_offset = (UE->rx_offset<<1) / UE->frame_parms.samples_per_tti;
                LOG_I( HW, "[%d] Got synch: hw_slot_offset %d, carrier off %d Hz, rxgain %d dB (DL %u, UL %u), UE_scan_carrier %d\n", procID_sync,
                       hw_slot_offset,
                       freq_offset,
                       UE->rx_total_gain_dB,
                       downlink_frequency[0][0]+freq_offset,
                       downlink_frequency[0][0]+uplink_frequency_offset[0][0]+freq_offset,
                       UE->UE_scan_carrier );


				// rerun with new cell parameters and frequency-offset
				for (i=0; i<openair0_cfg[UE->rf_map.card].rx_num_channels; i++) {
					openair0_cfg[UE->rf_map.card].rx_gain[UE->rf_map.chain+i] = UE->rx_total_gain_dB;//-USRP_GAIN_OFFSET;
					if (UE->UE_scan_carrier == 1) {
						if (freq_offset >= 0)
							openair0_cfg[UE->rf_map.card].rx_freq[UE->rf_map.chain+i] += abs(UE->common_vars.freq_offset);
						else
							openair0_cfg[UE->rf_map.card].rx_freq[UE->rf_map.chain+i] -= abs(UE->common_vars.freq_offset);
						openair0_cfg[UE->rf_map.card].tx_freq[UE->rf_map.chain+i] = openair0_cfg[UE->rf_map.card].rx_freq[UE->rf_map.chain+i]+uplink_frequency_offset[CC_id][i];
						downlink_frequency[CC_id][i] = openair0_cfg[CC_id].rx_freq[i];
						freq_offset=0;
					}
				}

				// reconfigure for potentially different bandwidth
				switch(UE->frame_parms.N_RB_DL) {
				case 6:
					openair0_cfg[UE->rf_map.card].sample_rate =1.92e6;
					openair0_cfg[UE->rf_map.card].rx_bw          =.96e6;
					openair0_cfg[UE->rf_map.card].tx_bw          =.96e6;
					//            openair0_cfg[0].rx_gain[0] -= 12;
					break;
				case 25:
					openair0_cfg[UE->rf_map.card].sample_rate =7.68e6;
					openair0_cfg[UE->rf_map.card].rx_bw          =2.5e6;
					openair0_cfg[UE->rf_map.card].tx_bw          =2.5e6;
					//            openair0_cfg[0].rx_gain[0] -= 6;
					break;
				case 50:
					openair0_cfg[UE->rf_map.card].sample_rate =15.36e6;
					openair0_cfg[UE->rf_map.card].rx_bw          =5.0e6;
					openair0_cfg[UE->rf_map.card].tx_bw          =5.0e6;
					//            openair0_cfg[0].rx_gain[0] -= 3;
					break;
				case 100:
					openair0_cfg[UE->rf_map.card].sample_rate=30.72e6;
					openair0_cfg[UE->rf_map.card].rx_bw=10.0e6;
					openair0_cfg[UE->rf_map.card].tx_bw=10.0e6;
					//            openair0_cfg[0].rx_gain[0] -= 0;
					break;
				}

				UE->rfdevice.trx_set_freq_func(&UE->rfdevice,&openair0_cfg[0],0);
				//UE->rfdevice.trx_set_gains_func(&openair0,&openair0_cfg[0]);
				//UE->rfdevice.trx_stop_func(&UE->rfdevice);
				sleep(1);
				init_frame_parms(&UE->frame_parms,1);
				dump_frame_parms(&UE->frame_parms);
				/*if (UE->rfdevice.trx_start_func(&UE->rfdevice) != 0 ) {
					LOG_E(HW,"Could not start the device\n");
					oai_exit=1;
				}*/


                if (UE->UE_scan_carrier == 1) {
					UE->UE_scan_carrier = 0;
                } else {
                    AssertFatal ( 0== pthread_mutex_lock(&UE->proc.mutex_synch), "");
                    //printf("Before: UE->is_synchronized = %d\n",UE->is_synchronized);
                    UE->is_synchronized = 1;
                    //printf("After: UE->is_synchronized = %d\n",UE->is_synchronized);
                    AssertFatal ( 0== pthread_mutex_unlock(&UE->proc.mutex_synch), "");

                    if( UE->mode == rx_dump_frame ) {
                        FILE *fd;
                        if ((UE->proc.proc_rxtx[0].frame_rx&1) == 0) {  // this guarantees SIB1 is present
                            if ((fd = fopen("rxsig_frame0.dat","w")) != NULL) {
                                fwrite((void*)&UE->common_vars.rxdata[0][0],
                                       sizeof(int32_t),
                                       10*UE->frame_parms.samples_per_tti,
                                       fd);
                                LOG_I(PHY,"Dummping Frame ... bye bye \n");
                                fclose(fd);
                                exit(0);
                            } else {
                                LOG_E(PHY,"Cannot open file for writing\n");
                                exit(0);
                            }
                        } else {
                            AssertFatal ( 0== pthread_mutex_lock(&UE->proc.mutex_synch), "");
                            UE->is_synchronized = 0;
                            AssertFatal ( 0== pthread_mutex_unlock(&UE->proc.mutex_synch), "");

                        }
                    }
                }
            } else {
                // initial sync failed
                // calculate new offset and try again
 /*           	if (UE->UE_scan == 1) {

            		if (UE->UE_scan_carrier == 1) {
            			LOG_I(PHY,"[%d] carrier_cnt = %d, CC_id = %d, frequency = %"PRIu32".\n",procID_sync,carrier_cnt,CC_id,downlink_frequency[0][0]);
            			LOG_I(PHY,"[%d] Initial sync failed. Calculating new +/-100-Hz offset and trying again.\n",procID_sync);
						if (freq_offset >= 0)
						freq_offset += 100;
						freq_offset *= -1;

						if (abs(freq_offset) > 7500) {
							//LOG_I( PHY, "[%d] No cell synchronization found after scanning 15-kHz BW (OFDM carrier BW); Scanning next carrier: %d of %d.\n",procID_sync,
							//							2+carrier_offset/15000,(bands_to_scan.band_info[CC_id].dl_max-bands_to_scan.band_info[CC_id].dl_min)/15000);
							LOG_I( PHY, "[%d] No cell synchronization found after scanning 15-kHz BW (OFDM carrier BW); Scanning next carrier: %d of %d.\n",procID_sync,
																					2+carrier_offset/15000,(bands_to_scan.band_info[set_band].dl_max-bands_to_scan.band_info[set_band].dl_min)/15000);
							freq_offset=0;
							carrier_offset+=15000;
							//pss_corr_freq[carrier_cnt] = pss_corr_foffset[carrier_cnt] + bands_to_scan.band_info[CC_id].dl_min;
							pss_corr_freq[carrier_cnt] = pss_corr_foffset[carrier_cnt] + bands_to_scan.band_info[set_band].dl_min;
							carrier_cnt++; //LA
							//if (carrier_offset > (bands_to_scan.band_info[CC_id].dl_max-bands_to_scan.band_info[CC_id].dl_min)) {
							if (carrier_offset > (bands_to_scan.band_info[set_band].dl_max-bands_to_scan.band_info[set_band].dl_min)) {
							//if (carrier_offset > 60000) {
								//Storing variable in MATLAB file
								write_output("PSS_correlation_peaks.m","peak_val",pss_corr_peaks,5000,1,2);
								write_output("PSS_correlation_time_offset.m","time_offset",pss_corr_toffset,5000,1,2);
								write_output("PSS_correlation_freq.m","freq",pss_corr_freq,5000,1,2);
								write_output("PSS_correlation_freq_offset.m","freq_offset",pss_corr_foffset,5000,1,2);
								write_output("PSS_correlation_sequence_index.m","pss_index",pss_corr_seq,5000,1,2);
								LOG_I(PHY,"Dumping max correlation values after PSS synchronization centered at each possible OFDM carrier... bye bye \n");
		                        //FILE *fd;
		                        //if ((fd = fopen("rxsig_frame0.dat","w"))!=NULL) {
		                        //    fwrite((void*)&UE->common_vars.rxdata[0][0],sizeof(int32_t),10*UE->frame_parms.samples_per_tti,fd);
		                        //    LOG_I(PHY,"Dumping max correlation values after PSS synchronization centered at each possible OFDM carrier... bye bye \n");
		                        //    fclose(fd);
		                        //    exit(0);
		                        //}
		                        //Exiting the whole program execution
								mac_xface->macphy_exit("No cell synchronization found, abandoning");
								return &UE_thread_synch_retval; // not reached
							}
						}

            		}

            	}
 */

            	LOG_I(PHY,"[%d] Initial sync failed. Calculating new +/-100-Hz offset and trying again.\n",procID_sync);
                if (UE->UE_scan_carrier == 1) {
                    if (freq_offset >= 0)
                        freq_offset += 100;
                    freq_offset *= -1;

                    if (abs(freq_offset) > 7500) {
                        LOG_I( PHY, "[%d] No cell synchronization found after scanning 15-kHz BW (OFDM carrier BW); abandoning.\n",procID_sync );
                        FILE *fd;
                        if ((fd = fopen("rxsig_frame0.dat","w"))!=NULL) {
                        //if ((fd = fopen("rxsig_frame0.m","w"))!=NULL) {
                            fwrite((void*)&UE->common_vars.rxdata[0][0],
                                   sizeof(int32_t),
                                   10*UE->frame_parms.samples_per_tti,
                                   fd);
                            LOG_I(PHY,"Dumping Frame ... bye bye \n");
                            fclose(fd);
                            exit(0);
                        }
                        mac_xface->macphy_exit("No cell synchronization found, abandoning");
                        return &UE_thread_synch_retval; // not reached
                    }

                }
#if DISABLE_LOG_X
                printf("[initial_sync] trying carrier off %d Hz, rxgain %d (DL %u, UL %u)\n",
                       freq_offset,
                       UE->rx_total_gain_dB,
                       downlink_frequency[0][0]+freq_offset,
                       downlink_frequency[0][0]+uplink_frequency_offset[0][0]+freq_offset );
#else
/*                LOG_I(PHY,"[%d] Trying carrier off %d Hz, rxgain %d (DL %u, UL %u)\n",procID_sync,
                       freq_offset,
                       UE->rx_total_gain_dB,
                       downlink_frequency[0][0]+freq_offset,
                       downlink_frequency[0][0]+uplink_frequency_offset[0][0]+freq_offset );*/
                LOG_I(PHY,"[%d] Trying carrier offset %d Hz, rxgain %d (DL %u, UL %u)\n",procID_sync,
                       freq_offset+carrier_offset,
                       UE->rx_total_gain_dB,
                       downlink_frequency[0][0]+freq_offset+carrier_offset,
                       downlink_frequency[0][0]+uplink_frequency_offset[0][0]+freq_offset+carrier_offset );
#endif

                for (i=0; i<openair0_cfg[UE->rf_map.card].rx_num_channels; i++) {
                	//openair0_cfg[UE->rf_map.card].rx_freq[UE->rf_map.chain+i] = downlink_frequency[CC_id][i]+freq_offset;
                	//openair0_cfg[UE->rf_map.card].tx_freq[UE->rf_map.chain+i] = downlink_frequency[CC_id][i]+uplink_frequency_offset[CC_id][i]+freq_offset;
                	openair0_cfg[UE->rf_map.card].rx_freq[UE->rf_map.chain+i] = downlink_frequency[CC_id][i]+freq_offset+carrier_offset;
                    openair0_cfg[UE->rf_map.card].tx_freq[UE->rf_map.chain+i] = downlink_frequency[CC_id][i]+uplink_frequency_offset[CC_id][i]+freq_offset+carrier_offset;
                    openair0_cfg[UE->rf_map.card].rx_gain[UE->rf_map.chain+i] = UE->rx_total_gain_dB;//-USRP_GAIN_OFFSET;
                    if (UE->UE_scan_carrier==1)
                        openair0_cfg[UE->rf_map.card].autocal[UE->rf_map.chain+i] = 1;
                }
                //Setting the new Tx/Rx frequency values in the USRP configuration
                UE->rfdevice.trx_set_freq_func(&UE->rfdevice,&openair0_cfg[0],0);
            }// initial_sync=0
            printf(">>>>>>>>>>>>>>>>>>>>>>>>>> [%d] End case: PBCH <<<<<<<<<<<<<<<<<<<<<<<<<<\n",procID_sync);//LA
            break;
        }
        case si:
        default:
            break;
        }

        AssertFatal ( 0== pthread_mutex_lock(&UE->proc.mutex_synch), "");
        // indicate readiness
        UE->proc.instance_cnt_synch--;
//LA        LOG_I(PHY,"[%d] UE->proc.instance_cnt_synch = %d, samples_per_tti = %d.\n",procID_sync,UE->proc.instance_cnt_synch,UE->frame_parms.samples_per_tti);
        AssertFatal ( 0== pthread_mutex_unlock(&UE->proc.mutex_synch), "");

        VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME( VCD_SIGNAL_DUMPER_FUNCTIONS_UE_THREAD_SYNCH, 0 );
    }  // while !oai_exit

    printf("**************************************************** End : [UE_thread_synch] [PID: %d] ****************************************************\n",procID_sync);
    return &UE_thread_synch_retval;
}

int tttime = 0;
#include "subframe2file.pb-c.h"
#include <time.h>

void populate_protobufc_proc(UeRxTxProc *proc,UE_rxtx_proc_t *proc_data) {
	proc->subframe_rx=proc_data->subframe_rx;

	return;
}

void populate_protobufc_ue(PhyVarsUe *ue_raw, PHY_VARS_UE *UE_data) {
	Frameparms frame_p = FRAMEPARMS__INIT;

	frame_p.n_rb_dl = UE_data->frame_parms.N_RB_DL;
	frame_p.n_rb_ul = UE_data->frame_parms.N_RB_UL;
	ue_raw->ue_scan = UE_data->UE_scan;
	ue_raw->frame_parms=&frame_p;
	return;
}


void dump_phy_vars_ue(PHY_VARS_UE *UE_data) {
	//printf("frame_parms->N_RB_DL=%u\n",frame_parms->N_RB_DL);
}

void dump_ue_rxtx_proc(UE_rxtx_proc_t *proc_data) {
	printf("proc_data->proc_id=%d\n",proc_data->proc_id);
	printf("proc_data->CC_id=%u\n",proc_data->CC_id);
	printf("proc_data->timestamp_tx=%ld\n",proc_data->timestamp_tx);
	printf("proc_data->subframe_tx=%d\n",proc_data->subframe_tx);
	printf("proc_data->subframe_rx=%d\n",proc_data->subframe_rx);
	printf("proc_data->frame_tx=%d\n",proc_data->frame_tx);
	printf("proc_data->frame_rx=%d\n",proc_data->frame_rx);
	printf("proc_data->instance_cnt_rxtx=%d\n",proc_data->instance_cnt_rxtx);
	printf("proc_data->pthread_rxtx=%lu\n",proc_data->pthread_rxtx);
	//excluded: attr_rxtx
	//excluded: cond_rxtx
	//excluded: mutex_rxtx
	//excluded: sched_param_rxtx
	printf("proc_data->instance_cnt_slot1_dl_processing=%d\n",proc_data->instance_cnt_slot1_dl_processing);
	printf("proc_data->pthread_slot1_dl_processing=%lu\n",proc_data->pthread_slot1_dl_processing);
	//excluded: attr_slot1_dl_processing
	//excluded: cond_slot1_dl_processing
	//excluded: mutex_slot1_dl_processing
	printf("proc_data->chan_est_pilot0_slot1_available=%u\n",proc_data->chan_est_pilot0_slot1_available);
	printf("proc_data->chan_est_slot1_available=%u\n",proc_data->chan_est_slot1_available);
	printf("proc_data->llr_slot1_available=%u\n",proc_data->llr_slot1_available);
	printf("proc_data->dci_slot0_available=%u\n",proc_data->dci_slot0_available);
	printf("proc_data->first_symbol_available=%u\n",proc_data->first_symbol_available);
	//excluded: sched_param_fep_slot1
	printf("proc_data->sub_frame_start=%d\n",proc_data->sub_frame_start);
	printf("proc_data->sub_frame_step=%d\n",proc_data->sub_frame_step);
	printf("proc_data->gotIQs=%llu\n",proc_data->gotIQs);

}

void dump_subframe_to_file(void *argum){
    struct rx_tx_thread_data *rx_data = argum;
    UE_rxtx_proc_t *proc_data = rx_data->proc;
    PHY_VARS_UE    *UE_data   = rx_data->UE;

    //String that contains current time to be used in filename
    time_t	rawtime=time(NULL);
    struct tm *tm = localtime(&rawtime);
    char s[20];
    char filename[50];
    strftime(s, sizeof(s), "%Y%m%d.%H%M%S_", tm);	//s contains 16 char
    //printf("now: %s\n", s);

    RxTxThreadData rxd = RX_TX_THREAD_DATA__INIT;

    //Populating context data structure for RX/TX portion of subframe processing
    UeRxTxProc	procd = UE_RX_TX_PROC__INIT;

    procd.proc_id=proc_data->proc_id;
    procd.cc_id=proc_data->CC_id;
    procd.timestamp_tx=proc_data->timestamp_tx;
    procd.subframe_tx=proc_data->subframe_tx;
    procd.subframe_rx=proc_data->subframe_rx;
    procd.frame_tx=proc_data->frame_tx;
    procd.frame_rx=proc_data->frame_rx;
    procd.instance_cnt_rxtx=proc_data->instance_cnt_rxtx;
    procd.pthread_rxtx=proc_data->pthread_rxtx;
    //excluded: attr_rxtx
    //excluded: cond_rxtx
    //excluded: mutex_rxtx
    //excluded: sched_param_rxtx
    procd.instance_cnt_slot1_dl_processing=proc_data->instance_cnt_slot1_dl_processing;
    procd.pthread_slot1_dl_processing=proc_data->pthread_slot1_dl_processing;
    //excluded: attr_slot1_dl_processing
    //excluded: cond_slot1_dl_processing
    //excluded: mutex_slot1_dl_processing
	procd.chan_est_pilot0_slot1_available=proc_data->chan_est_pilot0_slot1_available;
	procd.chan_est_slot1_available=proc_data->chan_est_slot1_available;
	procd.llr_slot1_available=proc_data->llr_slot1_available;
	procd.dci_slot0_available=proc_data->dci_slot0_available;
	procd.first_symbol_available=proc_data->first_symbol_available;
	//excluded: sched_param_fep_slot1
	procd.sub_frame_start=proc_data->sub_frame_start;
	procd.sub_frame_step=proc_data->sub_frame_step;
	procd.gotiqs=proc_data->gotIQs;

	dump_ue_rxtx_proc(proc_data);

    //Populating top-level PHY Data Structure for UE
    PhyVarsUe	ued	= PHY_VARS_UE__INIT;
    ued.mod_id = UE_data->Mod_id;
    ued.cc_id = UE_data->CC_id;
    //Openair0RfMap rf_map
    //RunMode mode
    ued.ue_scan = UE_data->UE_scan;
    ued.ue_scan_carrier = UE_data->UE_scan_carrier;
	ued.is_synchronized = UE_data->is_synchronized;
	//UeProc proc
	ued.no_timing_correction = UE_data->no_timing_correction;
    ued.tx_total_gain_db = UE_data->tx_total_gain_dB;
    ued.rx_total_gain_db = UE_data->rx_total_gain_dB;
    //rx_gain_max
    //rx_gain_med
    //rx_gain_byp
    //tx_power_dBm
    //tx_total_RE
    ued.tx_power_max_dbm = UE_data->tx_power_max_dBm;
    ued.n_connected_enb = UE_data->n_connected_eNB;
	ued.ho_initiated = UE_data->ho_initiated;
	ued.ho_triggered = UE_data->ho_triggered;
	//PhyMeasurements measurements
	//frame_parms -> DONE
	//Frameparms frame_parms_before_ho
	//UeCommon common_vars
	//current_thread_id
	//UePdsch *pdsch_vars[RX_NB_TH_MAX][NUMBER_OF_CONNECTED_eNB_MAX+1]
	//UePdschFlp *pdsch_vars_flp[NUMBER_OF_CONNECTED_eNB_MAX+1]
	//UePdsch *pdsch_vars_SI[NUMBER_OF_CONNECTED_eNB_MAX+1]
	//UePdsch *pdsch_vars_ra[NUMBER_OF_CONNECTED_eNB_MAX+1]
	//UePdsch *pdsch_vars_p[NUMBER_OF_CONNECTED_eNB_MAX+1]
	//UePdsch *pdsch_vars_MCH[NUMBER_OF_CONNECTED_eNB_MAX]
	//UePbch*pbch_vars[NUMBER_OF_CONNECTED_eNB_MAX]
	//UePdcch *pdcch_vars[RX_NB_TH_MAX][NUMBER_OF_CONNECTED_eNB_MAX]
	//UePrach*prach_vars[NUMBER_OF_CONNECTED_eNB_MAX]
	//UeDlsch *dlsch[RX_NB_TH_MAX][NUMBER_OF_CONNECTED_eNB_MAX][2]
	//UeDlsch *ulsch[NUMBER_OF_CONNECTED_eNB_MAX]
	//UeDlsch *dlsch_SI[NUMBER_OF_CONNECTED_eNB_MAX
	//UeDlsch *dlsch_ra[NUMBER_OF_CONNECTED_eNB_MAX]
	//UeDlsch *dlsch_p[NUMBER_OF_CONNECTED_eNB_MAX]
	//UeDlsch *dlsch_MCH[NUMBER_OF_CONNECTED_eNB_MAX]
	//EnbDlsch *dlsch_eNB[NUMBER_OF_CONNECTED_eNB_MAX]
	ued.imsimod1024 = UE_data->IMSImod1024;
	ued.pf = UE_data->PF;
	ued.po = UE_data->PO;
	//sr[10]
	//pucch_sel[10]
	//pucch_payload[22]
	//UE_mode[NUMBER_OF_CONNECTED_eNB_MAX]
	//lte_gold_table[7][20][2][14]
	//lte_gold_uespec_port5_table[20][38]
	//lte_gold_uespec_table[2][20][2][21]
	//lte_gold_mbsfn_table[10][3][42]
	//X_u[64][839]
	ued.high_speed_flag = UE_data->high_speed_flag;
	ued.perfect_ce = UE_data->perfect_ce;
	ued.ch_est_alpha = UE_data->ch_est_alpha;
	//generate_ul_signal[NUMBER_OF_CONNECTED_eNB_MAX]
	//UeScanInfo scan_info[NB_BANDS_MAX]
	//ulsch_no_allocation_counter[NUMBER_OF_CONNECTED_eNB_MAX]
	//ulsch_Msg3_active[NUMBER_OF_CONNECTED_eNB_MAX]
	//ulsch_Msg3_frame[NUMBER_OF_CONNECTED_eNB_MAX]
	//ulsch_Msg3_subframe[NUMBER_OF_CONNECTED_eNB_MAX]
	//PrachResources *prach_resources[NUMBER_OF_CONNECTED_eNB_MAX]
	ued.turbo_iterations = UE_data->turbo_iterations;
	ued.turbo_cntl_iterations = UE_data->turbo_cntl_iterations;
	// total_TBS[NUMBER_OF_CONNECTED_eNB_MAX] 
	// total_TBS_last[NUMBER_OF_CONNECTED_eNB_MAX] 
	// bitrate[NUMBER_OF_CONNECTED_eNB_MAX] 
	// total_received_bits[NUMBER_OF_CONNECTED_eNB_MAX] 
	// dlsch_errors[NUMBER_OF_CONNECTED_eNB_MAX] 
	// dlsch_errors_last[NUMBER_OF_CONNECTED_eNB_MAX] 
	// dlsch_received[NUMBER_OF_CONNECTED_eNB_MAX] 
	// dlsch_received_last[NUMBER_OF_CONNECTED_eNB_MAX] 
	// dlsch_fer[NUMBER_OF_CONNECTED_eNB_MAX] 
	// dlsch_SI_received[NUMBER_OF_CONNECTED_eNB_MAX] 
	// dlsch_SI_errors[NUMBER_OF_CONNECTED_eNB_MAX] 
	// dlsch_ra_received[NUMBER_OF_CONNECTED_eNB_MAX] 
	// dlsch_ra_errors[NUMBER_OF_CONNECTED_eNB_MAX] 
	// dlsch_p_received[NUMBER_OF_CONNECTED_eNB_MAX] 
	// dlsch_p_errors[NUMBER_OF_CONNECTED_eNB_MAX] 
	// dlsch_mch_received_sf[MAX_MBSFN_AREA][NUMBER_OF_CONNECTED_eNB_MAX] 
	// dlsch_mch_received[NUMBER_OF_CONNECTED_eNB_MAX] 
	// dlsch_mcch_received[MAX_MBSFN_AREA][NUMBER_OF_CONNECTED_eNB_MAX] 
	// dlsch_mtch_received[MAX_MBSFN_AREA][NUMBER_OF_CONNECTED_eNB_MAX] 
	// dlsch_mcch_errors[MAX_MBSFN_AREA][NUMBER_OF_CONNECTED_eNB_MAX] 
	// dlsch_mtch_errors[MAX_MBSFN_AREA][NUMBER_OF_CONNECTED_eNB_MAX] 
	// dlsch_mcch_trials[MAX_MBSFN_AREA][NUMBER_OF_CONNECTED_eNB_MAX] 
	// dlsch_mtch_trials[MAX_MBSFN_AREA][NUMBER_OF_CONNECTED_eNB_MAX] 
	// current_dlsch_cqi[NUMBER_OF_CONNECTED_eNB_MAX] 
	// first_run_timing_advance[NUMBER_OF_CONNECTED_eNB_MAX]
	ued.generate_prach =UE_data->generate_prach;
	ued.prach_cnt =UE_data->prach_cnt;
	ued.prach_preambleindex =UE_data->prach_PreambleIndex;
	ued.decode_sib =UE_data->decode_SIB;
	ued.decode_mib =UE_data->decode_MIB;
	ued.rx_offset =UE_data->rx_offset;
	ued.rx_offset_diff =UE_data->rx_offset_diff;
	ued.time_sync_cell =UE_data->time_sync_cell;
	ued.timing_advance =UE_data->timing_advance;
	ued.hw_timing_advance =UE_data->hw_timing_advance;
	ued.n_ta_offset =UE_data->N_TA_offset;
	ued.is_secondary_ue=UE_data->is_secondary_ue;
	ued.has_valid_precoder=UE_data->has_valid_precoder;
	//**ul_precoder_S_UE
	ued.log2_maxp =UE_data->log2_maxp;
	ued.mac_enabled =UE_data->mac_enabled;
	ued.init_averaging =UE_data->init_averaging;
	//*sinr_dB
	//*sinr_CQI_dB
	ued.sinr_eff =UE_data->sinr_eff;
	ued.n0 =UE_data->N0;



    Frameparms frame_p = FRAMEPARMS__INIT;
    frame_p.n_rb_dl = UE_data->frame_parms.N_RB_DL;
    frame_p.n_rb_ul = UE_data->frame_parms.N_RB_UL;
    frame_p.n_rbg = UE_data->frame_parms.N_RBG;
    frame_p.n_rbgs = UE_data->frame_parms.N_RBGS;
    frame_p.nid_cell = UE_data->frame_parms.Nid_cell;
    frame_p.nid_cell_mbsfn = UE_data->frame_parms.Nid_cell_mbsfn;
    frame_p.ncp = UE_data->frame_parms.Ncp;
    frame_p.ncp_ul = UE_data->frame_parms.Ncp_UL;
    frame_p.nushift = UE_data->frame_parms.nushift;
    frame_p.frame_type = UE_data->frame_parms.frame_type;
    frame_p.tdd_config = UE_data->frame_parms.tdd_config;
    frame_p.tdd_config_s = UE_data->frame_parms.tdd_config_S;
    frame_p.srsx = UE_data->frame_parms.srsX;
    frame_p.node_id = UE_data->frame_parms.node_id;
    frame_p.freq_idx = UE_data->frame_parms.freq_idx;
    frame_p.mode1_flag = UE_data->frame_parms.mode1_flag;
    frame_p.threequarter_fs = UE_data->frame_parms.threequarter_fs;
    frame_p.ofdm_symbol_size = UE_data->frame_parms.ofdm_symbol_size;
    frame_p.nb_prefix_samples = UE_data->frame_parms.nb_prefix_samples;
    frame_p.nb_prefix_samples0 = UE_data->frame_parms.nb_prefix_samples0;
    frame_p.first_carrier_offset = UE_data->frame_parms.first_carrier_offset;
	frame_p.samples_per_tti = UE_data->frame_parms.samples_per_tti;
	frame_p.symbols_per_tti = UE_data->frame_parms.symbols_per_tti;
	frame_p.dl_symbols_in_s_subframe = UE_data->frame_parms.dl_symbols_in_S_subframe;
	frame_p.ul_symbols_in_s_subframe = UE_data->frame_parms.ul_symbols_in_S_subframe;
	frame_p.nb_antennas_tx = UE_data->frame_parms.nb_antennas_tx;
	frame_p.nb_antennas_rx = UE_data->frame_parms.nb_antennas_rx;
	frame_p.nb_antenna_ports_enb = UE_data->frame_parms.nb_antenna_ports_eNB;
	//Pending: prach_config_common
	//Pending: pucch_config_common
	//Pending: pdsch_config_common
	//Pending: pusch_config_common
	//Pending: phich_config_common
	//Pending: soundingrs_ul_config_common
	//Pending: ul_power_control_config_common
	frame_p.num_mbsfn_config = UE_data->frame_parms.num_MBSFN_config;
	//Excluded: MBSFN_config
	frame_p.maxharq_msg3tx = UE_data->frame_parms.maxHARQ_Msg3Tx;
	frame_p.siwindowsize = UE_data->frame_parms.SIwindowsize;
	frame_p.siperiod = UE_data->frame_parms.SIPeriod;

	frame_p.n_pcfich_reg = 4;
	frame_p.pcfich_reg = malloc (sizeof (uint32_t) * frame_p.n_pcfich_reg);
	for (int i = 0; i < frame_p.n_pcfich_reg; i++)
		frame_p.pcfich_reg[i] = UE_data->frame_parms.pcfich_reg[i];
	frame_p.pcfich_first_reg_idx = UE_data->frame_parms.pcfich_first_reg_idx;

	frame_p.n_phich_reg_outer=	MAX_NUM_PHICH_GROUPS;
	PhichReg **phichreg;
	phichreg = malloc(sizeof(PhichReg*)*frame_p.n_phich_reg_outer);
	for (int j = 0; j < frame_p.n_phich_reg_outer; j++) {
		//printf("j=%d ",j);
		phichreg[j]=malloc (sizeof(PhichReg));
		phich_reg__init(phichreg[j]);
		phichreg[j]->n_phich_reg_inner=3;
		phichreg[j]->phich_reg_inner=malloc (sizeof(uint32_t)*phichreg[j]->n_phich_reg_inner);
		//printf("phichreg[%d]->n_phich_reg_inner : %lu\n",j,phichreg[j]->n_phich_reg_inner);
		for (int k = 0; k < phichreg[j]->n_phich_reg_inner; k++) {
			phichreg[j]->phich_reg_inner[k] = UE_data->frame_parms.phich_reg[j][k];
			//printf("val[%d][%d] : %d\n",j,k,phichreg[j]->phich_reg_inner[k]);
		}
	}
	frame_p.phich_reg_outer = phichreg;

	//Pending: MBSFN_SubframeConfig
    ued.frame_parms=&frame_p;

    //populate_protobufc_proc(&procd,proc_data);
    //populate_protobufc_ue(&ued,UE_data);

    rxd.proc=&procd;
    rxd.ue=&ued;

	void *buf;
	unsigned len;

	len = rx_tx_thread_data__get_packed_size(&rxd);
	printf("len = %d\n",len);
	buf = malloc(len);
	rx_tx_thread_data__pack(&rxd,buf);		//serializes the message.


    snprintf(filename,50,"%s%s%d-%d.%d_%d%s",s,"UE_",UE_data->frame_parms.Nid_cell,proc_data->frame_rx,proc_data->subframe_rx,tttime,".dat");
    printf("Filename: %s, subframe: %d\n",filename,proc_data->subframe_rx);
    FILE *fd;
    if ((fd = fopen(filename,"w"))!=NULL ) {
    	fprintf(stderr,"Writing %d serialized bytes\n",len);
    	fwrite(buf,len,1,fd);
    	fclose(fd);
    }
    free(buf);
    return;
}

/*!
 * \brief This is the UE thread for RX subframe n and TX subframe n+4.
 * This thread performs the phy_procedures_UE_RX() on every received slot.
 * then, if TX is enabled it performs TX for n+4.
 * \param arg is a pointer to a \ref PHY_VARS_UE structure.
 * \returns a pointer to an int. The storage is not on the heap and must not be freed.
 */

static void *UE_thread_rxn_txnp4(void *arg) {
	int procID_rxn_txnp4 = gettid();
	printf("**************************************************** Start : [UE_thread_rxn_txnp4] [PID: %d] ****************************************************\n",procID_rxn_txnp4);
    static __thread int UE_thread_rxtx_retval;
    struct rx_tx_thread_data *rtd = arg;	//LA: I may be intested in saving this file, since it contains both UE and proc pointers.
    UE_rxtx_proc_t *proc = rtd->proc;
    PHY_VARS_UE    *UE   = rtd->UE;
    int ret;


    proc->instance_cnt_rxtx=-1;	//LA: Instance count for RXn-TXnp4 processing thread. Always  = 0 for UE_thread_rxn_txnp4 processing.
    proc->subframe_rx=proc->sub_frame_start;

    char threadname[256];
    sprintf(threadname,"UE_%d_proc_%d", UE->Mod_id, proc->sub_frame_start);
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);

    if ( (proc->sub_frame_start+1)%RX_NB_TH == 0 && threads.one != -1 )
        CPU_SET(threads.one, &cpuset);
    if ( (proc->sub_frame_start+1)%RX_NB_TH == 1 && threads.two != -1 )
        CPU_SET(threads.two, &cpuset);
    if ( (proc->sub_frame_start+1)%RX_NB_TH == 2 && threads.three != -1 )
        CPU_SET(threads.three, &cpuset);
            //CPU_SET(threads.three, &cpuset);
    init_thread(900000,1000000 , FIFO_PRIORITY-1, &cpuset,
                threadname);

    while (!oai_exit) {
    	//LA: what do we lock here? Why do we do it?
        if (pthread_mutex_lock(&proc->mutex_rxtx) != 0) {
          LOG_E( PHY, "[SCHED][UE] error locking mutex for UE RXTX\n" );
          exit_fun("nothing to add");
        }
        //LA: Why do we wait here for?
        while (proc->instance_cnt_rxtx < 0) {	//LA: Initialized with a negative value when processing each Rx subframe, so the condition should normally be fulfilled.
        	//LA:when is it changed to a positive value?
          // most of the time, the thread is waiting here
        //LA: there must be a condition here that locks reading the UE values unless something doesn't finishes first
          pthread_cond_wait( &proc->cond_rxtx, &proc->mutex_rxtx );
        }
        //Only when this mutex is unlocked, we can proceed.
        if (pthread_mutex_unlock(&proc->mutex_rxtx) != 0) {
          LOG_E( PHY, "[SCHED][UE] error unlocking mutex for UE RXn_TXnp4\n" );
          exit_fun("nothing to add");
        }

        //LA: Why do we have two different timers? what is their importance?
        initRefTimes(t2);	//LA: What are the values of t2 and t3? where are they defined?
        initRefTimes(t3);
        pickTime(current);
        updateTimes(proc->gotIQs, &t2, 10000, "Delay to wake up UE_Thread_Rx (case 2)");

        //LA: Saving a subframe in the time domain.
        if (tttime >= 100 && tttime <= 115){
        	dump_subframe_to_file(rtd);
        }
        tttime++;
        //printf("int32_t = %d, UE_rxtx_proc_t = %d, PHY_VARS_UE = %d\n",sizeof(int32_t), sizeof(UE_rxtx_proc_t),sizeof(PHY_VARS_UE));
        //LA: end of saving subframe.

        // Process Rx data for one sub-frame. //LA: The subframe should have been stored in "rxp", which is a memory subsection of the pointer UE
        //LA: In the case of FDD, the value returned is SF_DL for all subframes. This function is only relevant in TDD.
        lte_subframe_t sf_type = subframe_select( &UE->frame_parms, proc->subframe_rx);
        if ((sf_type == SF_DL) ||
                (UE->frame_parms.frame_type == FDD) ||
                (sf_type == SF_S)) {

            if (UE->frame_parms.frame_type == TDD) {
                LOG_D(PHY, "%s,TDD%d,%s: calling UE_RX\n",
                      threadname,
                      UE->frame_parms.tdd_config,
                      (sf_type==SF_DL? "SF_DL" :
                       (sf_type==SF_UL? "SF_UL" :
                        (sf_type==SF_S ? "SF_S"  : "UNKNOWN_SF_TYPE"))));
            } else {
                //LOG_D(PHY, "%s,%s,%s: calling UE_RX\n",
                	LOG_I(PHY, "[PID-%d] proc->instance_cnt_rxtx = %d, threadname = %s, UE->frame_parms.frame_type = %s, subframe-type = %s: calling UE_RX\n",
                		procID_rxn_txnp4,
					proc->instance_cnt_rxtx,
                      threadname,
                      (UE->frame_parms.frame_type==FDD? "FDD":
                       (UE->frame_parms.frame_type==TDD? "TDD":"UNKNOWN_DUPLEX_MODE")),
                      (sf_type==SF_DL? "SF_DL" :
                       (sf_type==SF_UL? "SF_UL" :
                        (sf_type==SF_S ? "SF_S"  : "UNKNOWN_SF_TYPE"))));
            }
#ifdef UE_SLOT_PARALLELISATION
            phy_procedures_slot_parallelization_UE_RX( UE, proc, 0, 0, 1, UE->mode, no_relay, NULL );
#else
            phy_procedures_UE_RX( UE,	//LA: [phy_vars_ue] Pointer to UE variables on which to act
            			proc, 		//LA: Pointer to RXn_TXnp4 proc information
            			0, 			//LA: Local id of eNB on which to act
					0, 			//LA: [abstraction_flag] Indicator of PHY abstraction
					1, 			//LA: [do_pdcch_flag] if = 1, then deactivate reception until we scan pdcch
					UE->mode, 	//LA: [mode] calibration/debug mode
					no_relay, 	//LA: [r_type] indicates the relaying operation: "no relaying"
					NULL );		//LA: [phy_vars_rn] pointer to RN variables (no relay nodes)
#endif
        }

#if UE_TIMING_TRACE
        start_meas(&UE->generic_stat);
#endif
        if (UE->mac_enabled==1) {

            ret = mac_xface->ue_scheduler(UE->Mod_id,
                                          proc->frame_rx,
                                          proc->subframe_rx,
                                          proc->frame_tx,
                                          proc->subframe_tx,
                                          subframe_select(&UE->frame_parms,proc->subframe_tx),
                                          0,
                                          0/*FIXME CC_id*/);
            if ( ret != CONNECTION_OK) {
                char *txt;
                switch (ret) {
                case CONNECTION_LOST:
                    txt="RRC Connection lost, returning to PRACH";
                    break;
                case PHY_RESYNCH:
                    txt="RRC Connection lost, trying to resynch";
                    break;
                case RESYNCH:
                    txt="return to PRACH and perform a contention-free access";
                    break;
                default:
                    txt="UNKNOWN RETURN CODE";
                };
                LOG_E( PHY, "[UE %"PRIu8"] Frame %"PRIu32", subframe %u %s\n",
                       UE->Mod_id, proc->frame_rx, proc->subframe_tx,txt );
            }
        }
#if UE_TIMING_TRACE
        stop_meas(&UE->generic_stat);
#endif

        // Prepare the future Tx data

        if ((subframe_select( &UE->frame_parms, proc->subframe_tx) == SF_UL) ||
                (UE->frame_parms.frame_type == FDD) )
            if (UE->mode != loop_through_memory)
                phy_procedures_UE_TX(UE,proc,0,0,UE->mode,no_relay);


        if ((subframe_select( &UE->frame_parms, proc->subframe_tx) == SF_S) &&
                (UE->frame_parms.frame_type == TDD))
            if (UE->mode != loop_through_memory)
                phy_procedures_UE_S_TX(UE,0,0,no_relay);
        updateTimes(current, &t3, 10000, "Delay to process sub-frame (case 3)");

        if (pthread_mutex_lock(&proc->mutex_rxtx) != 0) {
          LOG_E( PHY, "[SCHED][UE] error locking mutex for UE RXTX\n" );
          exit_fun("noting to add");
        }
        proc->instance_cnt_rxtx--;
        if (pthread_mutex_unlock(&proc->mutex_rxtx) != 0) {
          LOG_E( PHY, "[SCHED][UE] error unlocking mutex for UE RXTX\n" );
          exit_fun("noting to add");
        }
    }

    printf("**************************************************** End : [UE_thread_rxn_txnp4] [PID: %d] ****************************************************\n",procID_rxn_txnp4);
// thread finished
    free(arg);
    return &UE_thread_rxtx_retval;
}

/*!
 * \brief This is the main UE thread.
 * This thread controls the other three UE threads:
 * - UE_thread_rxn_txnp4 (even subframes)
 * - UE_thread_rxn_txnp4 (odd subframes)
 * - UE_thread_synch
 * \param arg unused
 * \returns a pointer to an int. The storage is not on the heap and must not be freed.
 */

void *UE_thread(void *arg) {

	int procID_UE_thread = gettid();
	printf("**************************************************** Start : [UE_thread] [PID: %d] ****************************************************\n",procID_UE_thread);
    PHY_VARS_UE *UE = (PHY_VARS_UE *) arg;
    //  int tx_enabled = 0;
    int dummy_rx[UE->frame_parms.nb_antennas_rx][UE->frame_parms.samples_per_tti] __attribute__((aligned(32)));
    openair0_timestamp timestamp,timestamp1;
    void* rxp[NB_ANTENNAS_RX], *txp[NB_ANTENNAS_TX];
    int start_rx_stream = 0;
    int i;
    char threadname[128];
    int th_id;

    static uint8_t thread_idx = 0;

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    if ( threads.iq != -1 )
        CPU_SET(threads.iq, &cpuset);
    init_thread(100000, 500000, FIFO_PRIORITY, &cpuset,
                "UHD Threads");
    if (oaisim_flag == 0)
        AssertFatal(0== openair0_device_load(&(UE->rfdevice), &openair0_cfg[0]), "");
    UE->rfdevice.host_type = BBU_HOST;	//LA: this means that our processing is done in the baseband (Baseband Unit). What does this imply for the code?
    sprintf(threadname, "Main UE %d", UE->Mod_id);
    pthread_setname_np(pthread_self(), threadname);
    init_UE_threads(UE);

#ifdef NAS_UE
    MessageDef *message_p;
    message_p = itti_alloc_new_message(TASK_NAS_UE, INITIALIZE_MESSAGE);
    itti_send_msg_to_task (TASK_NAS_UE, UE->Mod_id + NB_eNB_INST, message_p);
#endif

    int sub_frame=-1;
    //int cumulated_shift=0;
//LA    int la=0;//LA, la1=0, la2=0;
    AssertFatal(UE->rfdevice.trx_start_func(&UE->rfdevice) == 0, "Could not start the device\n");
    while (!oai_exit) {
    	//LAla++;
//LA    	la++;
//LA    	printf("Cycle = %d\n",la);
        AssertFatal ( 0== pthread_mutex_lock(&UE->proc.mutex_synch), "");
        //LA: Instance count for synch thread. If <0,
        int instance_cnt_synch = UE->proc.instance_cnt_synch;
        //LA: Indicator that UE is synchronized to an eNB (i.e., the PSS, SSS, and PBCH sequences were decoded)
        int is_synchronized    = UE->is_synchronized;
        AssertFatal ( 0== pthread_mutex_unlock(&UE->proc.mutex_synch), "");

        if (is_synchronized == 0) {
//        	printf("la = %d, UE->is_synchronized = %d\n",la,UE->is_synchronized);
        	//printf("UE->is_synchronized = %d\n",UE->is_synchronized);
            if (instance_cnt_synch < 0) {  // we can invoke the synch
//LA                printf("[<0] instance_cnt_synch = %d\n",instance_cnt_synch);
            	// grab 10 ms of signal and wakeup synch thread
                for (int i=0; i<UE->frame_parms.nb_antennas_rx; i++)
                    rxp[i] = (void*)&UE->common_vars.rxdata[i][0];
//LA                printf("[<0] loop_through_memory = %d, UE->mode = %d\n", loop_through_memory,UE->mode);

                if (UE->mode != loop_through_memory)
                    AssertFatal( UE->frame_parms.samples_per_tti*10 ==
                                 UE->rfdevice.trx_read_func(&UE->rfdevice,
                                                            &timestamp,
                                                            rxp,
                                                            UE->frame_parms.samples_per_tti*10,
                                                            UE->frame_parms.nb_antennas_rx), "");
		AssertFatal ( 0== pthread_mutex_lock(&UE->proc.mutex_synch), "");
                instance_cnt_synch = ++UE->proc.instance_cnt_synch;
                if (instance_cnt_synch == 0) {
                	//LA: UE->proc.cond_synch: condition variable for UE synch thread;
                    AssertFatal( 0 == pthread_cond_signal(&UE->proc.cond_synch), "");
                } else {
                    LOG_E( PHY, "[SCHED][UE] UE sync thread busy!!\n" );
                    exit_fun("nothing to add");
                }
		AssertFatal ( 0== pthread_mutex_unlock(&UE->proc.mutex_synch), "");
            } else {
//LA            	printf("[=0] instance_cnt_synch = %d\n",instance_cnt_synch);
#if OAISIM
              (void)dummy_rx; /* avoid gcc warnings */
              usleep(500);
#else
//LA              printf("[=0] loop_through_memory = %d, UE->mode = %d, N_RB_DL = %d\n", loop_through_memory,UE->mode,UE->frame_parms.N_RB_DL);
                // grab 10 ms of signal into dummy buffer
                if (UE->mode != loop_through_memory) {
                    for (int i=0; i<UE->frame_parms.nb_antennas_rx; i++)
                        rxp[i] = (void*)&dummy_rx[i][0];
                    for (int sf=0; sf<10; sf++) {
 //LA                   	printf("before: sf = %d, samples_per_tti = %d, nb_antennas_rx = %d.\n",sf,UE->frame_parms.samples_per_tti,UE->frame_parms.nb_antennas_rx);
                    	//dump_frame_parms((LTE_DL_FRAME_PARMS*) UE->frame_parms);

//LA						printf("before: frame_parms->N_RB_DL=%d\n",UE->frame_parms.N_RB_DL);
/*LA						printf("frame_parms->N_RB_UL=%d\n",UE->frame_parms.N_RB_UL);
						//printf("frame_parms->Nid_cell=%d\n",UE->frame_parms.Nid_cell);
						printf("frame_parms->Ncp=%d\n",UE->frame_parms.Ncp);
						printf("frame_parms->Ncp_UL=%d\n",UE->frame_parms.Ncp_UL);
						//printf("frame_parms->nushift=%d\n",UE->frame_parms.nushift);
						printf("frame_parms->frame_type=%d\n",UE->frame_parms.frame_type);
						//printf("frame_parms->tdd_config=%d\n",UE->frame_parms.tdd_config);
						//printf("frame_parms->tdd_config_S=%d\n",UE->frame_parms.tdd_config_S);
						//printf("frame_parms->mode1_flag=%d\n",UE->frame_parms.mode1_flag);
						//printf("frame_parms->nb_antenna_ports_eNB=%d\n",UE->frame_parms.nb_antenna_ports_eNB);
						printf("frame_parms->nb_antennas_tx=%d\n",UE->frame_parms.nb_antennas_tx);
						printf("frame_parms->nb_antennas_rx=%d\n",UE->frame_parms.nb_antennas_rx);
						printf("frame_parms->ofdm_symbol_size=%d\n",UE->frame_parms.ofdm_symbol_size);
						printf("frame_parms->nb_prefix_samples=%d\n",UE->frame_parms.nb_prefix_samples);
						printf("frame_parms->nb_prefix_samples0=%d\n",UE->frame_parms.nb_prefix_samples0);
						printf("frame_parms->first_carrier_offset=%d\n",UE->frame_parms.first_carrier_offset);
						printf("frame_parms->samples_per_tti=%d\n",UE->frame_parms.samples_per_tti);
						printf("frame_parms->symbols_per_tti=%d\n",UE->frame_parms.symbols_per_tti);
*/
                    	//printf("rx = { %"PRIi16" , %"PRIi16" %"PRIi16" %"PRIi16" }\n",(int16_t *)(rxp),(int16_t *)(rxp+16),(int16_t *)(rxp+32),(int16_t *)(rxp+48));
                        //	    printf("Reading dummy sf %d\n",sf);
                    	//AssertFatal ( 0== pthread_mutex_lock(&UE->proc.mutex_synch), ""); //LA
                    	UE->rfdevice.trx_read_func(&UE->rfdevice,&timestamp,rxp,UE->frame_parms.samples_per_tti,UE->frame_parms.nb_antennas_rx);
						//AssertFatal ( 0== pthread_mutex_unlock(&UE->proc.mutex_synch), ""); //LA
//LA						printf("after: sf = %d, number of samples read = %d\n",sf,la2);
//LA						printf("after: frame_parms->N_RB_DL=%d\n",UE->frame_parms.N_RB_DL);
						//dump_frame_parms((LTE_DL_FRAME_PARMS*) UE->frame_parms);

                    }
                }
#endif
            }

        } // UE->is_synchronized==0
        else {
        	LOG_I(PHY,"[PID-%d] Frame synchronization succeed.\n",procID_UE_thread);
            if (start_rx_stream==0) {	//LA: This condition is only met the first time that sync is achieved, in order to perform timing offset
                start_rx_stream=1;
                if (UE->mode != loop_through_memory) {	//LA: this condition is normally met
                    if (UE->no_timing_correction==0) {	//LA: shouldn't the UE do timing correction? -> false, then do it.
                    		//LA: Performing timing correcting
                        LOG_I(PHY,"[PID-%d] Resynchronizing RX by rx_offset=%d samples (mode = %d, 0:normal_txrx)\n",procID_UE_thread,UE->rx_offset,UE->mode);
                        //LA: this function receives "rx_offset" samples and stores them in "rxdata." It returns "rx_offset"
                        //LA: after reading them, it can then start reading received samples without having a timing offset
                        //LA: i.e., once we read some values from the buffer, they are freed and the next ones become available for reading
                        AssertFatal(UE->rx_offset ==
                                    UE->rfdevice.trx_read_func(&UE->rfdevice,
                                                               &timestamp,
                                                               (void**)UE->common_vars.rxdata,
                                                               UE->rx_offset,
                                                               UE->frame_parms.nb_antennas_rx),"");
                    }
                    UE->rx_offset=0;
                    UE->time_sync_cell=0;
                    //UE->proc.proc_rxtx[0].frame_rx++;
                    //UE->proc.proc_rxtx[1].frame_rx++;
                    for (th_id=0; th_id < RX_NB_TH; th_id++) {
                    		//LA: "proc_rxtx[0-1]" = set of scheduling variables RXn-TXnp4 threads
                        UE->proc.proc_rxtx[th_id].frame_rx++;		//LA: "frame_rx" = frame to act upon for reception
                    }

                    // read in first symbol	//LA: the samples are overwritten in "rxdata"
                    AssertFatal (UE->frame_parms.ofdm_symbol_size+UE->frame_parms.nb_prefix_samples0 ==
                                 UE->rfdevice.trx_read_func(&UE->rfdevice,
                                                            &timestamp,
                                                            (void**)UE->common_vars.rxdata,
                                                            UE->frame_parms.ofdm_symbol_size+UE->frame_parms.nb_prefix_samples0,
                                                            UE->frame_parms.nb_antennas_rx),"");
                    //LA: This function implements the OFDM front end processor on reception (FEP) -> which basically performs DFT
                    //LA: UE, OFDM symbol within slot = 0,slot number = 0, sample offset within rxdata = 0, prefix included, always zero.
                    slot_fep(UE,0, 0, 0, 0, 0);
                } //UE->mode != loop_through_memory
                else
                    rt_sleep_ns(1000*1000);

            } else {
            		//LA: This condition is met once per while iteration. In each while iteration, one subframe is processed.
                sub_frame++;
                sub_frame%=10;
                UE_rxtx_proc_t *proc = &UE->proc.proc_rxtx[thread_idx];
                // update thread index for received subframe
                UE->current_thread_id[sub_frame] = thread_idx;

                LOG_I(PHY,"[PID-%d] Processing Subframe = %d, thread Idx = %d \n",procID_UE_thread, sub_frame, UE->current_thread_id[sub_frame]);

                //LA: "thread_idx" can be either 0 or 1, depending whether we are processing an odd or even subframe
                thread_idx++;
                if(thread_idx>=RX_NB_TH)
                    thread_idx = 0;


                if (UE->mode != loop_through_memory) {
                	//LA: UE->mode is normally normal_txrx=0 and not loop_through_memory=8. So normally this portion of the code is executed.
                	//LA: rxp points to a memory sector offset of the original UE. UE is the most important variable. At what point should I write it down onto a file?
                	//LA: since the memory section allocated to UE continuously changes due to updates in the data it contains (e.g. new subframes received)
                    for (i=0; i<UE->frame_parms.nb_antennas_rx; i++)
                        rxp[i] = (void*)&UE->common_vars.rxdata[i][UE->frame_parms.ofdm_symbol_size+
                                 UE->frame_parms.nb_prefix_samples0+
                                 sub_frame*UE->frame_parms.samples_per_tti];
                    for (i=0; i<UE->frame_parms.nb_antennas_tx; i++)
                        txp[i] = (void*)&UE->common_vars.txdata[i][((sub_frame+2)%10)*UE->frame_parms.samples_per_tti];

                    int readBlockSize, writeBlockSize;
                    if (sub_frame<9) {
                        readBlockSize=UE->frame_parms.samples_per_tti;
                        writeBlockSize=UE->frame_parms.samples_per_tti;
                    } else {
                        // set TO compensation to zero
                        UE->rx_offset_diff = 0;
                        // compute TO compensation that should be applied for this frame
                        if ( UE->rx_offset < 5*UE->frame_parms.samples_per_tti  &&
                                UE->rx_offset > 0 )
                            UE->rx_offset_diff = -1 ;
                        if ( UE->rx_offset > 5*UE->frame_parms.samples_per_tti &&
                                UE->rx_offset < 10*UE->frame_parms.samples_per_tti )
                            UE->rx_offset_diff = 1;

                        //LOG_D(PHY,"AbsSubframe %d.%d SET rx_off_diff to %d rx_offset %d \n",proc->frame_rx,sub_frame,UE->rx_offset_diff,UE->rx_offset);
                        LOG_D(PHY,"[PID-%d] AbsSubframe %d.%d SET rx_off_diff to %d rx_offset %d \n",procID_UE_thread,proc->frame_rx,sub_frame,UE->rx_offset_diff,UE->rx_offset);
                        readBlockSize=UE->frame_parms.samples_per_tti -
                                      UE->frame_parms.ofdm_symbol_size -
                                      UE->frame_parms.nb_prefix_samples0 -
                                      UE->rx_offset_diff;
                        writeBlockSize=UE->frame_parms.samples_per_tti -
                                       UE->rx_offset_diff;
                    }

                    //LA: [Rx] here we read a whole subframe and store it in "rxp", which is basically a sector mapped to &UE->common_vars.rxdat
                    //LA: with certain offset depending on the current frame that is being processed.
                    AssertFatal(readBlockSize ==
                                UE->rfdevice.trx_read_func(&UE->rfdevice,
                                                           &timestamp,
                                                           rxp,
                                                           readBlockSize,	//LA: it reads one TTI (one subframe) per operation
                                                           UE->frame_parms.nb_antennas_rx),"");
                    printf("Timestamp: %"PRIi64"\n",timestamp);
                    AssertFatal( writeBlockSize ==
                                 UE->rfdevice.trx_write_func(&UE->rfdevice,
                                         timestamp+
                                         (2*UE->frame_parms.samples_per_tti) -
                                         UE->frame_parms.ofdm_symbol_size-UE->frame_parms.nb_prefix_samples0 -
                                         openair0_cfg[0].tx_sample_advance,
                                         txp,
                                         writeBlockSize,
                                         UE->frame_parms.nb_antennas_tx,
                                         1),"");
                    if( sub_frame==9) {
                        // read in first symbol of next frame and adjust for timing drift
                        int first_symbols=writeBlockSize-readBlockSize;
                        if ( first_symbols > 0 )
                            AssertFatal(first_symbols ==
                                        UE->rfdevice.trx_read_func(&UE->rfdevice,
                                                                   &timestamp1,
                                                                   (void**)UE->common_vars.rxdata,
                                                                   first_symbols,	//LA: this is the length
                                                                   UE->frame_parms.nb_antennas_rx),"");
                        if ( first_symbols <0 )
                            LOG_E(PHY,"can't compensate: diff =%d\n", first_symbols);
                    }
                    pickTime(gotIQs);
                    // operate on thread sf mod 2
                    AssertFatal(pthread_mutex_lock(&proc->mutex_rxtx) ==0,"");
                    if(sub_frame == 0) {
                        //UE->proc.proc_rxtx[0].frame_rx++;
                        //UE->proc.proc_rxtx[1].frame_rx++;
                        for (th_id=0; th_id < RX_NB_TH; th_id++) {
                            UE->proc.proc_rxtx[th_id].frame_rx++;	//LA: in the next iteration it will act upon the next Rx frame
                        }
                    }
                    //UE->proc.proc_rxtx[0].gotIQs=readTime(gotIQs);
                    //UE->proc.proc_rxtx[1].gotIQs=readTime(gotIQs);
                    for (th_id=0; th_id < RX_NB_TH; th_id++) {
                        UE->proc.proc_rxtx[th_id].gotIQs=readTime(gotIQs);
                    }
                    proc->subframe_rx=sub_frame;
                    proc->subframe_tx=(sub_frame+4)%10;
                    proc->frame_tx = proc->frame_rx + (proc->subframe_rx>5?1:0);
                    proc->timestamp_tx = timestamp+
                                         (4*UE->frame_parms.samples_per_tti)-
                                         UE->frame_parms.ofdm_symbol_size-UE->frame_parms.nb_prefix_samples0;

                    proc->instance_cnt_rxtx++;
                    //LOG_D( PHY, "[SCHED][UE %d] UE RX instance_cnt_rxtx %d subframe %d !!\n", UE->Mod_id, proc->instance_cnt_rxtx,proc->subframe_rx);
                    LOG_I( PHY, "[PID-%d][SCHED][UE %d]  UE RX instance_cnt_rxtx %d subframe %d !!\n",procID_UE_thread, UE->Mod_id,proc->instance_cnt_rxtx,proc->subframe_rx);
                    if (proc->instance_cnt_rxtx == 0) {	//LA: this would only be =0 if it was previously =1. How does this is valid in a general case, either for 1 or 0?
                      if (pthread_cond_signal(&proc->cond_rxtx) != 0) {
                        LOG_E( PHY, "[SCHED][UE %d] ERROR pthread_cond_signal for UE RX thread\n", UE->Mod_id);
                        exit_fun("nothing to add");
                      }
                    } else {
                    	//LA: What does this mean?
                      LOG_E( PHY, "[PID-%d][SCHED][UE %d] UE RX thread busy (Instance counter: %d)!!\n", procID_UE_thread,UE->Mod_id, proc->instance_cnt_rxtx);
                      if (proc->instance_cnt_rxtx > 2)
                        exit_fun("instance_cnt_rxtx > 2");
                    }

                    AssertFatal (pthread_cond_signal(&proc->cond_rxtx) ==0 ,"");
                    AssertFatal(pthread_mutex_unlock(&proc->mutex_rxtx) ==0,"");
                    initRefTimes(t1);
                    initStaticTime(lastTime);
                    updateTimes(lastTime, &t1, 20000, "Delay between two IQ acquisitions (case 1)");
                    pickStaticTime(lastTime);

                } else {
                    printf("Processing subframe %d",proc->subframe_rx);
                    getchar();
                }
            } // start_rx_stream==1
        } // UE->is_synchronized==1

    } // while !oai_exit
    printf("**************************************************** End : [UE_thread] [PID: %d] ****************************************************\n",procID_UE_thread);
    return NULL;
}

/*!
 * \brief Initialize the UE theads.
 * Creates the UE threads:
 * - UE_thread_rxtx0
 * - UE_thread_rxtx1
 * - UE_thread_synch
 * - UE_thread_fep_slot0
 * - UE_thread_fep_slot1
 * - UE_thread_dlsch_proc_slot0
 * - UE_thread_dlsch_proc_slot1
 * and the locking between them.
 */
void init_UE_threads(PHY_VARS_UE *UE) {
	int procID_init_UE_threads = gettid();
    struct rx_tx_thread_data *rtd;

    pthread_attr_init (&UE->proc.attr_ue);
    pthread_attr_setstacksize(&UE->proc.attr_ue,8192);//5*PTHREAD_STACK_MIN);

    pthread_mutex_init(&UE->proc.mutex_synch,NULL);
    pthread_cond_init(&UE->proc.cond_synch,NULL);

    // the threads are not yet active, therefore access is allowed without locking
    int nb_threads=RX_NB_TH;
    for (int i=0; i<nb_threads; i++) {
        rtd = calloc(1, sizeof(struct rx_tx_thread_data));
        if (rtd == NULL) abort();
        rtd->UE = UE;
        rtd->proc = &UE->proc.proc_rxtx[i];

        pthread_mutex_init(&UE->proc.proc_rxtx[i].mutex_rxtx,NULL);
        pthread_cond_init(&UE->proc.proc_rxtx[i].cond_rxtx,NULL);
        UE->proc.proc_rxtx[i].sub_frame_start=i;
        UE->proc.proc_rxtx[i].sub_frame_step=nb_threads;
        LOG_I(PHY,"[PID-%d] Init_UE_threads: rtd->proc->sub_frame_start (rtd) = %d, UE->proc.proc_rxtx[%d].sub_frame_start (proc) = %d, nb_threads = %d, i = %d\n",procID_init_UE_threads,rtd->proc->sub_frame_start,i,UE->proc.proc_rxtx[i].sub_frame_start,nb_threads, i);
        pthread_create(&UE->proc.proc_rxtx[i].pthread_rxtx, NULL, UE_thread_rxn_txnp4, rtd);

#ifdef UE_SLOT_PARALLELISATION
        //pthread_mutex_init(&UE->proc.proc_rxtx[i].mutex_slot0_dl_processing,NULL);
        //pthread_cond_init(&UE->proc.proc_rxtx[i].cond_slot0_dl_processing,NULL);
        //pthread_create(&UE->proc.proc_rxtx[i].pthread_slot0_dl_processing,NULL,UE_thread_slot0_dl_processing, rtd);

        pthread_mutex_init(&UE->proc.proc_rxtx[i].mutex_slot1_dl_processing,NULL);
        pthread_cond_init(&UE->proc.proc_rxtx[i].cond_slot1_dl_processing,NULL);
        pthread_create(&UE->proc.proc_rxtx[i].pthread_slot1_dl_processing,NULL,UE_thread_slot1_dl_processing, rtd);
#endif

    }
    pthread_create(&UE->proc.pthread_synch,NULL,UE_thread_synch,(void*)UE);
}


#ifdef OPENAIR2
void fill_ue_band_info(void) {

    UE_EUTRA_Capability_t *UE_EUTRA_Capability = UE_rrc_inst[0].UECap->UE_EUTRA_Capability;
    int i,j;

    bands_to_scan.nbands = UE_EUTRA_Capability->rf_Parameters.supportedBandListEUTRA.list.count;

    for (i=0; i<bands_to_scan.nbands; i++) {

        for (j=0; j<sizeof (eutra_bands) / sizeof (eutra_bands[0]); j++)
            if (eutra_bands[j].band == UE_EUTRA_Capability->rf_Parameters.supportedBandListEUTRA.list.array[i]->bandEUTRA) {
                memcpy(&bands_to_scan.band_info[i],
                       &eutra_bands[j],
                       sizeof(eutra_band_t));

                printf("Band %d (%lu) : DL %u..%u Hz, UL %u..%u Hz, Duplex %s \n",
                       bands_to_scan.band_info[i].band,
                       UE_EUTRA_Capability->rf_Parameters.supportedBandListEUTRA.list.array[i]->bandEUTRA,
                       bands_to_scan.band_info[i].dl_min,
                       bands_to_scan.band_info[i].dl_max,
                       bands_to_scan.band_info[i].ul_min,
                       bands_to_scan.band_info[i].ul_max,
                       (bands_to_scan.band_info[i].frame_type==FDD) ? "FDD" : "TDD");
                break;
            }
    }
}
#endif

int setup_ue_buffers(PHY_VARS_UE **phy_vars_ue, openair0_config_t *openair0_cfg) {

    int i, CC_id;
    LTE_DL_FRAME_PARMS *frame_parms;
    openair0_rf_map *rf_map;

    for (CC_id=0; CC_id<MAX_NUM_CCs; CC_id++) {
        rf_map = &phy_vars_ue[CC_id]->rf_map;

        AssertFatal( phy_vars_ue[CC_id] !=0, "");
        frame_parms = &(phy_vars_ue[CC_id]->frame_parms);

        // replace RX signal buffers with mmaped HW versions
        rxdata = (int32_t**)malloc16( frame_parms->nb_antennas_rx*sizeof(int32_t*) );
        txdata = (int32_t**)malloc16( frame_parms->nb_antennas_tx*sizeof(int32_t*) );

        for (i=0; i<frame_parms->nb_antennas_rx; i++) {
            LOG_I(PHY, "Mapping UE CC_id %d, rx_ant %d, freq %u on card %d, chain %d\n",
                  CC_id, i, downlink_frequency[CC_id][i], rf_map->card, rf_map->chain+i );
            free( phy_vars_ue[CC_id]->common_vars.rxdata[i] );
            rxdata[i] = (int32_t*)malloc16_clear( 307200*sizeof(int32_t) );
            phy_vars_ue[CC_id]->common_vars.rxdata[i] = rxdata[i]; // what about the "-N_TA_offset" ? // N_TA offset for TDD
        }

        for (i=0; i<frame_parms->nb_antennas_tx; i++) {
            LOG_I(PHY, "Mapping UE CC_id %d, tx_ant %d, freq %u on card %d, chain %d\n",
                  CC_id, i, downlink_frequency[CC_id][i], rf_map->card, rf_map->chain+i );
            free( phy_vars_ue[CC_id]->common_vars.txdata[i] );
            txdata[i] = (int32_t*)malloc16_clear( 307200*sizeof(int32_t) );
            phy_vars_ue[CC_id]->common_vars.txdata[i] = txdata[i];
        }

        // rxdata[x] points now to the same memory region as phy_vars_ue[CC_id]->common_vars.rxdata[x]
        // txdata[x] points now to the same memory region as phy_vars_ue[CC_id]->common_vars.txdata[x]
        // be careful when releasing memory!
        // because no "release_ue_buffers"-function is available, at least rxdata and txdata memory will leak (only some bytes)
    }
    return 0;
}

