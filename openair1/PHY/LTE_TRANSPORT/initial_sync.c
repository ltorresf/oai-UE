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

/*! \file PHY/LTE_TRANSPORT/initial_sync.c
* \brief Routines for initial UE synchronization procedure (PSS,SSS,PBCH and frame format detection)
* \author R. Knopp, F. Kaltenberger
* \date 2011
* \version 0.1
* \company Eurecom
* \email: knopp@eurecom.fr,kaltenberger@eurecom.fr
* \note
* \warning
*/
#include "PHY/types.h"
#include "PHY/defs.h"
#include "PHY/extern.h"
#include "SCHED/defs.h"
#include "SCHED/extern.h"
#include "defs.h"
#include "extern.h"

//LA:
#include "../../../targets/RT/USER/rt_wrapper.h"

#include "common_lib.h"
extern openair0_config_t openair0_cfg[];

//LA1#define DEBUG_INITIAL_SYNCH

int pbch_detection(PHY_VARS_UE *ue, runmode_t mode)
{

  uint8_t l,pbch_decoded,frame_mod4,pbch_tx_ant,dummy;
  LTE_DL_FRAME_PARMS *frame_parms=&ue->frame_parms;
  char phich_resource[6];

#ifdef DEBUG_INITIAL_SYNCH
  LOG_I(PHY,"[UE%d] Initial sync: starting PBCH detection (rx_offset %d)\n",ue->Mod_id,
        ue->rx_offset);
#endif

  //LA: The next two for-loops convert the first subframe to the frequency domain
  //LA Converts the first 7 OFDM symbols of slot 0 to the frequency domain
  for (l=0; l<frame_parms->symbols_per_tti/2; l++) {

    slot_fep(ue,
	     l,
	     0, //LA: Slot number (0..19)
	     ue->rx_offset,
	     0,
	     1);
  }
  //LA Converts the first 7 OFDM symbols of slot 1 to the frequency domain
  for (l=0; l<frame_parms->symbols_per_tti/2; l++) {

    slot_fep(ue,
	     l,
	     1, //LA: Slot number (0..19)
	     ue->rx_offset,
	     0,
	     1);
  }
  slot_fep(ue,
	   0,	//LA: OFDM symbol 0
	   2,	//LA: Slot number (0..19)
	   ue->rx_offset,
	   0,
	   1);

  //LA: this function fills the PHY_VARS_UE->PHY_measurement structure
  lte_ue_measurements(ue,
		      ue->rx_offset,
		      0,
                      0,
		      0,
                      0);


  if (ue->frame_parms.frame_type == TDD) {
    ue_rrc_measurements(ue,
			2,
			0);
  }
  else {
    ue_rrc_measurements(ue,
			0,
			0);
  }
#ifdef DEBUG_INITIAL_SYNCH
  LOG_I(PHY,"[UE %d] RX RSSI %d dBm, digital (%d, %d) dB, linear (%d, %d), avg rx power %d dB (%d lin), RX gain %d dB\n",
        ue->Mod_id,
        ue->measurements.rx_rssi_dBm[0] - ((ue->frame_parms.nb_antennas_rx==2) ? 3 : 0),
        ue->measurements.rx_power_dB[0][0],
        ue->measurements.rx_power_dB[0][1],
        ue->measurements.rx_power[0][0],
        ue->measurements.rx_power[0][1],
        ue->measurements.rx_power_avg_dB[0],
        ue->measurements.rx_power_avg[0],
        ue->rx_total_gain_dB);

  LOG_I(PHY,"[UE %d] N0 %d dBm digital (%d, %d) dB, linear (%d, %d), avg noise power %d dB (%d lin)\n",
        ue->Mod_id,
        ue->measurements.n0_power_tot_dBm,
        ue->measurements.n0_power_dB[0],
        ue->measurements.n0_power_dB[1],
        ue->measurements.n0_power[0],
        ue->measurements.n0_power[1],
        ue->measurements.n0_power_avg_dB,
        ue->measurements.n0_power_avg);
#endif

  pbch_decoded = 0;

  for (frame_mod4=0; frame_mod4<4; frame_mod4++) {
	  //LA: rx_pbch function returns the number of Tx antennas at the eNB (we obtain 2)
    pbch_tx_ant = rx_pbch(&ue->common_vars,
                          ue->pbch_vars[0],
                          frame_parms,
                          0,
                          SISO,
                          ue->high_speed_flag,
                          frame_mod4);

    if ((pbch_tx_ant>0) && (pbch_tx_ant<=2)) {
      pbch_decoded = 1;
      LOG_I(PHY,"PBCH Decoded using SISO configuration\n");
      break;
    }

    pbch_tx_ant = rx_pbch(&ue->common_vars,
                          ue->pbch_vars[0],
                          frame_parms,
                          0,
                          ALAMOUTI,
                          ue->high_speed_flag,
                          frame_mod4);

    if ((pbch_tx_ant>0) && (pbch_tx_ant<=2)) {
      pbch_decoded = 1;
      LOG_I(PHY,"PBCH Decoded using Alamouti (Tx Diversity) configuration\n");
      break;
    }
  }


  if (pbch_decoded) {

    frame_parms->nb_antenna_ports_eNB = pbch_tx_ant;

    // set initial transmission mode to 1 or 2 depending on number of detected TX antennas at the eNodeB
    frame_parms->mode1_flag = (pbch_tx_ant==1);
    LOG_I(PHY,"Number of eNB Tx antennas detected = %d, Transmission mode set to: TM = %s\n",pbch_tx_ant, (pbch_tx_ant==1) ? "SISO" : "Tx Diversity");
    // openair_daq_vars.dlsch_transmission_mode = (pbch_tx_ant>1) ? 2 : 1;


    // flip byte endian on 24-bits for MIB
    //    dummy = ue->pbch_vars[0]->decoded_output[0];
    //    ue->pbch_vars[0]->decoded_output[0] = ue->pbch_vars[0]->decoded_output[2];
    //    ue->pbch_vars[0]->decoded_output[2] = dummy;

    // now check for Bandwidth of Cell
    dummy = (ue->pbch_vars[0]->decoded_output[2]>>5)&7;

    switch (dummy) {

    case 0 :
    	LOG_I(PHY,"case 0: dummy = 0. Changing N_RB_DL to 6\n");
      frame_parms->N_RB_DL = 6;
      break;

    case 1 :
    	LOG_I(PHY,"case 1: dummy = 1. Changing N_RB_DL to 15\n");
      frame_parms->N_RB_DL = 15;
      break;

    case 2 :
    	LOG_I(PHY,"case 2: dummy = 2. Changing N_RB_DL to 25\n");
      frame_parms->N_RB_DL = 25;
      break;

    case 3 :
    	LOG_I(PHY,"case 3: dummy = 3. Changing N_RB_DL to 50\n");
      frame_parms->N_RB_DL = 50;
      break;

    case 4 :
    	LOG_I(PHY,"case 4: dummy = 4. Changing N_RB_DL to 75\n");
      frame_parms->N_RB_DL = 75;
      break;

    case 5:
    	LOG_I(PHY,"case 5: dummy = 5. Changing N_RB_DL to 100\n");
      frame_parms->N_RB_DL = 100;
      break;

    default:
      LOG_E(PHY,"[UE%d] Initial sync: PBCH decoding: Unknown N_RB_DL\n",ue->Mod_id);
      return -1;
      break;
    }


    // now check for PHICH parameters
    frame_parms->phich_config_common.phich_duration = (PHICH_DURATION_t)((ue->pbch_vars[0]->decoded_output[2]>>4)&1);
    dummy = (ue->pbch_vars[0]->decoded_output[2]>>2)&3;

    switch (dummy) {
    case 0:
      frame_parms->phich_config_common.phich_resource = oneSixth;
      sprintf(phich_resource,"1/6");
      break;

    case 1:
      frame_parms->phich_config_common.phich_resource = half;
      sprintf(phich_resource,"1/2");
      break;

    case 2:
      frame_parms->phich_config_common.phich_resource = one;
      sprintf(phich_resource,"1");
      break;

    case 3:
      frame_parms->phich_config_common.phich_resource = two;
      sprintf(phich_resource,"2");
      break;

    default:
      LOG_E(PHY,"[UE%d] Initial sync: Unknown PHICH_DURATION\n",ue->Mod_id);
      return -1;
      break;
    }

    for(int i=0; i<RX_NB_TH;i++)
    {
        ue->proc.proc_rxtx[i].frame_rx =   (((ue->pbch_vars[0]->decoded_output[2]&3)<<6) + (ue->pbch_vars[0]->decoded_output[1]>>2))<<2;
        ue->proc.proc_rxtx[i].frame_rx =   (((ue->pbch_vars[0]->decoded_output[2]&3)<<6) + (ue->pbch_vars[0]->decoded_output[1]>>2))<<2;

#ifndef USER_MODE
        // one frame delay
        ue->proc.proc_rxtx[i].frame_rx ++;
#endif
        ue->proc.proc_rxtx[i].frame_tx = ue->proc.proc_rxtx[0].frame_rx;
    }
#ifdef DEBUG_INITIAL_SYNCH
    LOG_I(PHY,"[UE%d] Initial sync: pbch decoded sucessfully mode1_flag %d, eNB_tx_ant %d, frame %d, N_RB_DL %d, phich_duration %d, phich_resource %s!\n",
          ue->Mod_id,
          frame_parms->mode1_flag,
          pbch_tx_ant,
          ue->proc.proc_rxtx[0].frame_rx,
          frame_parms->N_RB_DL,
          frame_parms->phich_config_common.phich_duration,
          phich_resource);  //frame_parms->phich_config_common.phich_resource);
#endif
    return(0);
  } else {
    return(-1);
  }

}

char phich_string[13][4] = {"","1/6","","1/2","","","one","","","","","","two"};
char duplex_string[2][4] = {"FDD","TDD"};
char prefix_string[2][9] = {"NORMAL","EXTENDED"};

//LA: if returns ret=0, it has done synchronization. Otherwise it has failed and a new frequency must be chosen.
int initial_sync(PHY_VARS_UE *ue, runmode_t mode)
{
	int procID_initial_sync = gettid();
//LA1	printf("-------------------------- Start: [initial_sync] [PID: %d] --------------------------\n",procID_initial_sync);

  int32_t sync_pos,sync_pos2,sync_pos_slot;
  int32_t metric_fdd_ncp=0,metric_fdd_ecp=0,metric_tdd_ncp=0,metric_tdd_ecp=0;
  uint8_t phase_fdd_ncp,phase_fdd_ecp,phase_tdd_ncp,phase_tdd_ecp;
  uint8_t flip_fdd_ncp,flip_fdd_ecp,flip_tdd_ncp,flip_tdd_ecp;
  //  uint16_t Nid_cell_fdd_ncp=0,Nid_cell_fdd_ecp=0,Nid_cell_tdd_ncp=0,Nid_cell_tdd_ecp=0;
  LTE_DL_FRAME_PARMS *frame_parms = &ue->frame_parms;
  int ret=-1;
  int aarx,rx_power=0;
  /*#ifdef OAI_USRP
  __m128i *rxdata128;
  #endif*/
  //  LOG_I(PHY,"**************************************************************\n");
  // First try FDD normal prefix
//LA1  printf(".......................... [1/4] Ncp=NORMAL, frame_type=FDD ..........................\n");
  frame_parms->Ncp=NORMAL;
  frame_parms->frame_type=FDD;
  init_frame_parms(frame_parms,1);
  /*
  write_output("rxdata0.m","rxd0",ue->common_vars.rxdata[0],10*frame_parms->samples_per_tti,1,1);
  exit(-1);
  */
  sync_pos = lte_sync_time(ue->common_vars.rxdata,frame_parms,(int *)&ue->common_vars.eNb_id);

  //  write_output("rxdata1.m","rxd1",ue->common_vars.rxdata[0],10*frame_parms->samples_per_tti,1,1);
  //LA: sync_pos2 is the beginning of the OFDM symbol considering the CP-prefix
  if (sync_pos >= frame_parms->nb_prefix_samples)
    sync_pos2 = sync_pos - frame_parms->nb_prefix_samples;
  else
    sync_pos2 = sync_pos + FRAME_LENGTH_COMPLEX_SAMPLES - frame_parms->nb_prefix_samples;

#ifdef DEBUG_INITIAL_SYNCH
  //LOG_I(PHY,"[UE%d] Initial sync : Estimated PSS position %d, Nid2 %d\n",ue->Mod_id,sync_pos,ue->common_vars.eNb_id);
  LOG_I(PHY,"[%d] Estimated PSS position = %"PRIi32", sync_pos2 = %"PRIi32", eNb_id = %d\n",procID_initial_sync,sync_pos,sync_pos2,ue->common_vars.eNb_id);
#endif

  // SSS detection

  // PSS is hypothesized in last symbol of first slot in Frame
  sync_pos_slot = (frame_parms->samples_per_tti>>1) - frame_parms->ofdm_symbol_size - frame_parms->nb_prefix_samples;

  if (sync_pos2 >= sync_pos_slot)
    ue->rx_offset = sync_pos2 - sync_pos_slot;
  else
    ue->rx_offset = FRAME_LENGTH_COMPLEX_SAMPLES + sync_pos2 - sync_pos_slot;

  LOG_I(PHY,"[%d] sync_pos_slot = %"PRIi32", timing offset = %d\n",procID_initial_sync,sync_pos_slot,ue->rx_offset);

  if (((sync_pos2 - sync_pos_slot) >=0 ) && ((sync_pos2 - sync_pos_slot) < ((FRAME_LENGTH_COMPLEX_SAMPLES-frame_parms->samples_per_tti/2)))) {
#ifdef DEBUG_INITIAL_SYNCH
    //LOG_I(PHY,"Calling sss detection (FDD normal CP).\n");
    LOG_I(PHY,"[%d] Calling SSS detection (FDD normal CP).\n",procID_initial_sync);
#endif
    rx_sss(ue,&metric_fdd_ncp,&flip_fdd_ncp,&phase_fdd_ncp);
    frame_parms->nushift  = frame_parms->Nid_cell%6;

   if (flip_fdd_ncp==1)
      ue->rx_offset += (FRAME_LENGTH_COMPLEX_SAMPLES>>1);

    init_frame_parms(&ue->frame_parms,1);
    lte_gold(frame_parms,ue->lte_gold_table[0],frame_parms->Nid_cell);
    LOG_I(PHY, "[1/4] Ncp=NORMAL, frame_type=FDD. Before PBCH detection: N_RB_DL = %d \n",ue->frame_parms.N_RB_DL);
    ret = pbch_detection(ue,mode);
    //   write_output("rxdata2.m","rxd2",ue->common_vars.rxdata[0],10*frame_parms->samples_per_tti,1,1);

#ifdef DEBUG_INITIAL_SYNCH
    //LOG_I(PHY,"FDD Normal prefix: CellId %d metric %d, phase %d, flip %d, pbch %d\n",frame_parms->Nid_cell,metric_fdd_ncp,phase_fdd_ncp,flip_fdd_ncp,ret);
    LOG_I(PHY,"[%d] CellId = %d, metric = %d, phase = %d, flip = %d, pbch = %d\n",procID_initial_sync,frame_parms->Nid_cell,metric_fdd_ncp,phase_fdd_ncp,flip_fdd_ncp,ret);
#endif
  } else {
#ifdef DEBUG_INITIAL_SYNCH
    LOG_I(PHY,"[%d] SSS error condition: sync_pos %d, sync_pos_slot %d\n",procID_initial_sync,sync_pos, sync_pos_slot);
#endif
  }


  if (ret==-1) {

    // Now FDD extended prefix
//LA1	  printf(".......................... [2/4] Ncp=EXTENDED, frame_type=FDD ..........................\n");
    frame_parms->Ncp=EXTENDED;
    frame_parms->frame_type=FDD;
    init_frame_parms(frame_parms,1);

    if (sync_pos < frame_parms->nb_prefix_samples)
      sync_pos2 = sync_pos + FRAME_LENGTH_COMPLEX_SAMPLES - frame_parms->nb_prefix_samples;
    else
      sync_pos2 = sync_pos - frame_parms->nb_prefix_samples;

    // PSS is hypothesized in last symbol of first slot in Frame
    sync_pos_slot = (frame_parms->samples_per_tti>>1) - frame_parms->ofdm_symbol_size - (frame_parms->nb_prefix_samples);

    if (sync_pos2 >= sync_pos_slot)
      ue->rx_offset = sync_pos2 - sync_pos_slot;
    else
      ue->rx_offset = FRAME_LENGTH_COMPLEX_SAMPLES + sync_pos2 - sync_pos_slot;

    //msg("nb_prefix_samples %d, rx_offset %d\n",frame_parms->nb_prefix_samples,ue->rx_offset);

    if (((sync_pos2 - sync_pos_slot) >=0 ) &&
        ((sync_pos2 - sync_pos_slot) < ((FRAME_LENGTH_COMPLEX_SAMPLES-frame_parms->samples_per_tti/2)))) {

      rx_sss(ue,&metric_fdd_ecp,&flip_fdd_ecp,&phase_fdd_ecp);
      frame_parms->nushift  = frame_parms->Nid_cell%6;

      if (flip_fdd_ecp==1)
        ue->rx_offset += (FRAME_LENGTH_COMPLEX_SAMPLES>>1);

      init_frame_parms(&ue->frame_parms,1);
      lte_gold(frame_parms,ue->lte_gold_table[0],frame_parms->Nid_cell);
      LOG_I(PHY, "[2/4] Ncp=EXTENDED, frame_type=FDD. Before PBCH detection: N_RB_DL = %d \n",ue->frame_parms.N_RB_DL);
      ret = pbch_detection(ue,mode);
      //     write_output("rxdata3.m","rxd3",ue->common_vars.rxdata[0],10*frame_parms->samples_per_tti,1,1);
#ifdef DEBUG_INITIAL_SYNCH
      LOG_I(PHY,"FDD Extended prefix: CellId %d metric %d, phase %d, flip %d, pbch %d\n",
            frame_parms->Nid_cell,metric_fdd_ecp,phase_fdd_ecp,flip_fdd_ecp,ret);
#endif
    } else {
#ifdef DEBUG_INITIAL_SYNCH
      LOG_I(PHY,"FDD Extended prefix: SSS error condition: sync_pos %d, sync_pos_slot %d\n", sync_pos, sync_pos_slot);
#endif
    }

    if (ret==-1) {
      // Now TDD normal prefix
    	//printf(".......................... [3/4] Ncp=NORMAL, frame_type=TDD ..........................\n");
      frame_parms->Ncp=NORMAL;
      frame_parms->frame_type=TDD;
      init_frame_parms(frame_parms,1);

      if (sync_pos >= frame_parms->nb_prefix_samples)
        sync_pos2 = sync_pos - frame_parms->nb_prefix_samples;
      else
        sync_pos2 = sync_pos + FRAME_LENGTH_COMPLEX_SAMPLES - frame_parms->nb_prefix_samples;

      // PSS is hypothesized in 2nd symbol of third slot in Frame (S-subframe)
      sync_pos_slot = frame_parms->samples_per_tti +
                      (frame_parms->ofdm_symbol_size<<1) +
                      frame_parms->nb_prefix_samples0 +
                      (frame_parms->nb_prefix_samples);

      if (sync_pos2 >= sync_pos_slot)
        ue->rx_offset = sync_pos2 - sync_pos_slot;
      else
        ue->rx_offset = (FRAME_LENGTH_COMPLEX_SAMPLES>>1) + sync_pos2 - sync_pos_slot;

      rx_sss(ue,&metric_tdd_ncp,&flip_tdd_ncp,&phase_tdd_ncp);

      if (flip_tdd_ncp==1)
        ue->rx_offset += (FRAME_LENGTH_COMPLEX_SAMPLES>>1);

      frame_parms->nushift  = frame_parms->Nid_cell%6;
      init_frame_parms(&ue->frame_parms,1);

      lte_gold(frame_parms,ue->lte_gold_table[0],frame_parms->Nid_cell);
      LOG_I(PHY, "[3/4] Ncp=NORMAL, frame_type=TDD. Before PBCH detection: N_RB_DL = %d \n",ue->frame_parms.N_RB_DL);
      ret = pbch_detection(ue,mode);
      //      write_output("rxdata4.m","rxd4",ue->common_vars.rxdata[0],10*frame_parms->samples_per_tti,1,1);

#ifdef DEBUG_INITIAL_SYNCH
      LOG_I(PHY,"TDD Normal prefix: CellId %d metric %d, phase %d, flip %d, pbch %d\n",
            frame_parms->Nid_cell,metric_tdd_ncp,phase_tdd_ncp,flip_tdd_ncp,ret);
#endif

      if (ret==-1) {
        // Now TDD extended prefix
//LA1    	  printf(".......................... [4/4] Ncp=EXTENDED, frame_type=TDD ..........................\n");
        frame_parms->Ncp=EXTENDED;
        frame_parms->frame_type=TDD;
        init_frame_parms(frame_parms,1);
        sync_pos2 = sync_pos - frame_parms->nb_prefix_samples;

        if (sync_pos >= frame_parms->nb_prefix_samples)
          sync_pos2 = sync_pos - frame_parms->nb_prefix_samples;
        else
          sync_pos2 = sync_pos + FRAME_LENGTH_COMPLEX_SAMPLES - frame_parms->nb_prefix_samples;

        // PSS is hypothesized in 2nd symbol of third slot in Frame (S-subframe)
        sync_pos_slot = frame_parms->samples_per_tti + (frame_parms->ofdm_symbol_size<<1) + (frame_parms->nb_prefix_samples<<1);

        if (sync_pos2 >= sync_pos_slot)
          ue->rx_offset = sync_pos2 - sync_pos_slot;
        else
          ue->rx_offset = (FRAME_LENGTH_COMPLEX_SAMPLES>>1) + sync_pos2 - sync_pos_slot;

        rx_sss(ue,&metric_tdd_ecp,&flip_tdd_ecp,&phase_tdd_ecp);
        frame_parms->nushift  = frame_parms->Nid_cell%6;

        if (flip_tdd_ecp==1)
          ue->rx_offset += (FRAME_LENGTH_COMPLEX_SAMPLES>>1);

        init_frame_parms(&ue->frame_parms,1);
        lte_gold(frame_parms,ue->lte_gold_table[0],frame_parms->Nid_cell);
        LOG_I(PHY, "[4/4] Ncp=EXTENDED, frame_type=TDD. Before PBCH detection: N_RB_DL = %d \n",ue->frame_parms.N_RB_DL);
        ret = pbch_detection(ue,mode);

	//	write_output("rxdata5.m","rxd5",ue->common_vars.rxdata[0],10*frame_parms->samples_per_tti,1,1);
#ifdef DEBUG_INITIAL_SYNCH
        LOG_I(PHY,"TDD Extended prefix: CellId %d metric %d, phase %d, flip %d, pbch %d\n",
              frame_parms->Nid_cell,metric_tdd_ecp,phase_tdd_ecp,flip_tdd_ecp,ret);
#endif
      }
    }
  }

  /* Consider threturnis is a false detection if the offset is > 1000 Hz */
  if( (abs(ue->common_vars.freq_offset) > 150) && (ret == 0) )
  {
	  ret=-1;
#if DISABLE_LOG_X
	  printf("Ignore MIB with high freq offset [%d Hz] estimation \n",ue->common_vars.freq_offset);
#else
	  LOG_E(HW, "Ignore MIB with high freq offset [%d Hz] estimation. N_RB_DL = %d \n",ue->common_vars.freq_offset,ue->frame_parms.N_RB_DL);
#endif
  }

  if (ret==0) {  // PBCH found so indicate sync to higher layers and configure frame parameters

    //#ifdef DEBUG_INITIAL_SYNCH
#if DISABLE_LOG_X
    printf("[UE%d] In synch, rx_offset %d samples\n",ue->Mod_id, ue->rx_offset);
#else
    LOG_I(PHY, "[UE%d] In synch, rx_offset %d samples\n",ue->Mod_id, ue->rx_offset);
#endif
    //#endif

    if (ue->UE_scan_carrier == 0) {

    #if UE_AUTOTEST_TRACE
      LOG_I(PHY,"[UE  %d] AUTOTEST Cell Sync : frame = %d, rx_offset %d, freq_offset %d \n",
              ue->Mod_id,
              ue->proc.proc_rxtx[0].frame_rx,
              ue->rx_offset,
              ue->common_vars.freq_offset );
    #endif

// send sync status to higher layers later when timing offset converge to target timing
#if OAISIM
      if (ue->mac_enabled==1) {
	LOG_I(PHY,"[UE%d] Sending synch status to higher layers\n",ue->Mod_id);
	//mac_resynch();
	mac_xface->dl_phy_sync_success(ue->Mod_id,ue->proc.proc_rxtx[0].frame_rx,0,1);//ue->common_vars.eNb_id);
	ue->UE_mode[0] = PRACH;
      }
      else {
	ue->UE_mode[0] = PUSCH;
      }
#endif

      generate_pcfich_reg_mapping(frame_parms);
      generate_phich_reg_mapping(frame_parms);


      ue->pbch_vars[0]->pdu_errors_conseq=0;

    }

#if DISABLE_LOG_X
    printf("[UE %d] Frame %d RRC Measurements => rssi %3.1f dBm (dig %3.1f dB, gain %d), N0 %d dBm,  rsrp %3.1f dBm/RE, rsrq %3.1f dB\n",ue->Mod_id,
	  ue->proc.proc_rxtx[0].frame_rx,
	  10*log10(ue->measurements.rssi)-ue->rx_total_gain_dB,
	  10*log10(ue->measurements.rssi),
	  ue->rx_total_gain_dB,
	  ue->measurements.n0_power_tot_dBm,
	  10*log10(ue->measurements.rsrp[0])-ue->rx_total_gain_dB,
	  (10*log10(ue->measurements.rsrq[0])));


    printf("[UE %d] Frame %d MIB Information => %s, %s, NidCell %d, N_RB_DL %d, PHICH DURATION %d, PHICH RESOURCE %s, TX_ANT %d\n",
	  ue->Mod_id,
	  ue->proc.proc_rxtx[0].frame_rx,
	  duplex_string[ue->frame_parms.frame_type],
	  prefix_string[ue->frame_parms.Ncp],
	  ue->frame_parms.Nid_cell,
	  ue->frame_parms.N_RB_DL,
	  ue->frame_parms.phich_config_common.phich_duration,
	  phich_string[ue->frame_parms.phich_config_common.phich_resource],
	  ue->frame_parms.nb_antenna_ports_eNB);
#else
    //LOG_I(PHY, "[UE %d] Frame %d RRC Measurements => rssi = %3.1f dBm (dig %3.1f dB, rx_total_gain_dB = %d), Estimated_Noise_power = %d dBm,  rsrp %3.1f dBm/RE, rsrq %3.1f dB\n",ue->Mod_id,
    LOG_I(PHY, "[UE %d] Frame %d RRC Measurements => rssi = %3.1f dBm (dig %3.1f dBm, rx_total_gain_dB = %d), Estimated_Noise_power = %d dBm,  rsrp =  %3.1f dBm/RE, rsrq = %3.1f dB\n",ue->Mod_id,
	  ue->proc.proc_rxtx[0].frame_rx,
	  10*log10(ue->measurements.rssi)-ue->rx_total_gain_dB,
	  10*log10(ue->measurements.rssi),
	  ue->rx_total_gain_dB,
	  ue->measurements.n0_power_tot_dBm,
	  10*log10(ue->measurements.rsrp[0])-ue->rx_total_gain_dB,
	  (10*log10(ue->measurements.rsrq[0])));

    LOG_I(PHY, "[UE %d] Frame %d MIB Information => %s, %s, NidCell %d, N_RB_DL %d, PHICH DURATION %d, PHICH RESOURCE %s, TX_ANT %d\n",
	  ue->Mod_id,
	  ue->proc.proc_rxtx[0].frame_rx,
	  duplex_string[ue->frame_parms.frame_type],
	  prefix_string[ue->frame_parms.Ncp],
	  ue->frame_parms.Nid_cell,
	  ue->frame_parms.N_RB_DL,
	  ue->frame_parms.phich_config_common.phich_duration,
	  phich_string[ue->frame_parms.phich_config_common.phich_resource],
	  ue->frame_parms.nb_antenna_ports_eNB);
#endif

#if defined(OAI_USRP) || defined(EXMIMO) || defined(OAI_BLADERF) || defined(OAI_LMSSDR)
#  if DISABLE_LOG_X
    printf("[UE %d] Frame %d Measured Carrier Frequency %.0f Hz (offset %d Hz)\n",
	  ue->Mod_id,
	  ue->proc.proc_rxtx[0].frame_rx,
	  openair0_cfg[0].rx_freq[0]-ue->common_vars.freq_offset,
	  ue->common_vars.freq_offset);
#  else
    LOG_I(PHY, "[UE %d] Frame %d Measured Carrier Frequency %.0f Hz (offset %d Hz)\n",
	  ue->Mod_id,
	  ue->proc.proc_rxtx[0].frame_rx,
	  openair0_cfg[0].rx_freq[0]-ue->common_vars.freq_offset,
	  ue->common_vars.freq_offset);
#  endif
#endif
  } else {
#ifdef DEBUG_INITIAL_SYNC
    LOG_I(PHY,"[UE%d] Initial sync : PBCH not ok\n",ue->Mod_id);
    LOG_I(PHY,"[UE%d] Initial sync : Estimated PSS position %d, Nid2 %d\n",ue->Mod_id,sync_pos,ue->common_vars.eNb_id);
    /*      LOG_I(PHY,"[UE%d] Initial sync: (metric fdd_ncp %d (%d), metric fdd_ecp %d (%d), metric_tdd_ncp %d (%d), metric_tdd_ecp %d (%d))\n",
          ue->Mod_id,
          metric_fdd_ncp,Nid_cell_fdd_ncp,
          metric_fdd_ecp,Nid_cell_fdd_ecp,
          metric_tdd_ncp,Nid_cell_tdd_ncp,
          metric_tdd_ecp,Nid_cell_tdd_ecp);*/
    LOG_I(PHY,"[UE%d] Initial sync : Estimated Nid_cell %d, Frame_type %d\n",ue->Mod_id,
          frame_parms->Nid_cell,frame_parms->frame_type);
#endif

    ue->UE_mode[0] = NOT_SYNCHED;
    ue->pbch_vars[0]->pdu_errors_last=ue->pbch_vars[0]->pdu_errors;
    ue->pbch_vars[0]->pdu_errors++;
    ue->pbch_vars[0]->pdu_errors_conseq++;

  }

  // gain control
  if (ret!=0) { //we are not synched (PBCH not found), so we cannot use rssi measurement (which is based on channel estimates)
    rx_power = 0;

    // do a measurement on the best guess of the PSS
    for (aarx=0; aarx<frame_parms->nb_antennas_rx; aarx++)
      rx_power += signal_energy(&ue->common_vars.rxdata[aarx][sync_pos2],
				frame_parms->ofdm_symbol_size+frame_parms->nb_prefix_samples);

    /*
    // do a measurement on the full frame
    for (aarx=0; aarx<frame_parms->nb_antennas_rx; aarx++)
      rx_power += signal_energy(&ue->common_vars.rxdata[aarx][0],
				frame_parms->samples_per_tti*10);
    */

    // we might add a low-pass filter here later
    ue->measurements.rx_power_avg[0] = rx_power/frame_parms->nb_antennas_rx;

    ue->measurements.rx_power_avg_dB[0] = dB_fixed(ue->measurements.rx_power_avg[0]);

#ifdef DEBUG_INITIAL_SYNCH
  LOG_I(PHY,"[UE%d] Initial sync : Estimated power: %d dB\n",ue->Mod_id,ue->measurements.rx_power_avg_dB[0] );
#endif

//#ifndef OAI_USRP //LA
#ifndef OAI_BLADERF
#ifndef OAI_LMSSDR
  phy_adjust_gain(ue,
		  ue->measurements.rx_power_avg_dB[0], //LA: estimated received signal power (sum of all TX/RX antennas, time average, in dB)
		  0);
#endif
#endif
//#endif

  }
  else {

//#ifndef OAI_USRP
#ifndef OAI_BLADERF
#ifndef OAI_LMSSDR
  phy_adjust_gain(ue,
		  dB_fixed(ue->measurements.rssi), //LA: RRC measurement
		  0);
#endif
#endif
//#endif

  }

//LA1  printf("-------------------------- End: [initial_sync] [PID: %d] --------------------------\n",procID_initial_sync);
  //  exit_fun("debug exit");
  return ret;
}

