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

#include "defs.h"
#include "log.h"

uint16_t dl_S_table_normal[10]={3,9,10,11,12,3,9,10,11,6};
uint16_t dl_S_table_extended[10]={3,8,9,10,3,8,9,5,0,0};

void set_S_config(LTE_DL_FRAME_PARMS *fp) {

  int X = fp->srsX;

  fp->ul_symbols_in_S_subframe=(1+X);

  if ((fp->Ncp==EXTENDED) && (fp->tdd_config_S>7))
    AssertFatal(1==0,"Illegal S subframe configuration for Extended Prefix mode\n");

  fp->dl_symbols_in_S_subframe = (fp->Ncp==NORMAL)?dl_S_table_normal[fp->tdd_config_S] : dl_S_table_extended[fp->tdd_config_S];

  
}

int init_frame_parms(LTE_DL_FRAME_PARMS *frame_parms,uint8_t osf)
{

  uint8_t log2_osf;

#if DISABLE_LOG_X
  printf("Initializing frame parms for N_RB_DL %d, Ncp %d, osf %d\n",frame_parms->N_RB_DL,frame_parms->Ncp,osf);
#else
  //LA:LOG_I(PHY,"Initializing frame parms for N_RB_DL = %d, Ncp = %d, osf = %d\n",frame_parms->N_RB_DL,frame_parms->Ncp,osf);
#endif

  if (frame_parms->Ncp==EXTENDED) {
    frame_parms->nb_prefix_samples0=512;
    frame_parms->nb_prefix_samples = 512;
    frame_parms->symbols_per_tti = 12;
  } else {
    frame_parms->nb_prefix_samples0 = 160;
    frame_parms->nb_prefix_samples = 144;
    frame_parms->symbols_per_tti = 14;
      
  }


  switch(osf) {	//LA: "oversampling factor" set to 1, when called by main OAI-UE (i.e. normal sampling (no oversampling))
  case 1:
    log2_osf = 0;
    break;

  case 2:
    log2_osf = 1;
    break;

  case 4:
    log2_osf = 2;
    break;

  case 8:
    log2_osf = 3;
    break;

  case 16:
    log2_osf = 4;
    break;

  default:
    printf("Illegal oversampling %d\n",osf);
    return(-1);
  }
  //printf("[lte_parms] frame_parms->N_RB_DL = %d\n",frame_parms->N_RB_DL);
  switch (frame_parms->N_RB_DL) {

  case 100:
    if (osf>1) {
      printf("Illegal oversampling %d for N_RB_DL %d\n",osf,frame_parms->N_RB_DL);
      return(-1);
    }

    if (frame_parms->threequarter_fs) {
      frame_parms->ofdm_symbol_size = 1536;
      frame_parms->samples_per_tti = 23040;
      frame_parms->first_carrier_offset = 1536-600;
      frame_parms->nb_prefix_samples=(frame_parms->nb_prefix_samples*3)>>2;
      frame_parms->nb_prefix_samples0=(frame_parms->nb_prefix_samples0*3)>>2;
    }
    else {
      frame_parms->ofdm_symbol_size = 2048;
      frame_parms->samples_per_tti = 30720;
      frame_parms->first_carrier_offset = 2048-600;
    }
    frame_parms->N_RBGS = 4;
    frame_parms->N_RBG = 25;
    break;

  case 75:
    if (osf>1) {
      printf("Illegal oversampling %d for N_RB_DL %d\n",osf,frame_parms->N_RB_DL);
      return(-1);
    }


    frame_parms->ofdm_symbol_size = 1536;
    frame_parms->samples_per_tti = 23040;
    frame_parms->first_carrier_offset = 1536-450;
    frame_parms->nb_prefix_samples=(frame_parms->nb_prefix_samples*3)>>2;
    frame_parms->nb_prefix_samples0=(frame_parms->nb_prefix_samples0*3)>>2;
    frame_parms->N_RBGS = 4;
    frame_parms->N_RBG = 25;
    break;

  case 50:
    if (osf>1) {
      printf("Illegal oversampling %d for N_RB_DL %d\n",osf,frame_parms->N_RB_DL);
      return(-1);
    }

    frame_parms->ofdm_symbol_size = 1024*osf;
    frame_parms->samples_per_tti = 15360*osf;
    frame_parms->first_carrier_offset = frame_parms->ofdm_symbol_size - 300;
    frame_parms->nb_prefix_samples>>=(1-log2_osf);
    frame_parms->nb_prefix_samples0>>=(1-log2_osf);
    frame_parms->N_RBGS = 3;
    frame_parms->N_RBG = 17;
    break;

  case 25:
    if (osf>2) {
      printf("Illegal oversampling %d for N_RB_DL %d\n",osf,frame_parms->N_RB_DL);
      return(-1);
    }

    frame_parms->ofdm_symbol_size = 512*osf;


    frame_parms->samples_per_tti = 7680*osf;
    frame_parms->first_carrier_offset = frame_parms->ofdm_symbol_size - 150;
    frame_parms->nb_prefix_samples>>=(2-log2_osf);
    frame_parms->nb_prefix_samples0>>=(2-log2_osf);
    frame_parms->N_RBGS = 2;
    frame_parms->N_RBG = 13;


    break;

  case 15:
    frame_parms->ofdm_symbol_size = 256*osf;
    frame_parms->samples_per_tti = 3840*osf;
    frame_parms->first_carrier_offset = frame_parms->ofdm_symbol_size - 90;
    frame_parms->nb_prefix_samples>>=(3-log2_osf);
    frame_parms->nb_prefix_samples0>>=(3-log2_osf);
    frame_parms->N_RBGS = 2;
    frame_parms->N_RBG = 8;
    break;

  case 6:
    frame_parms->ofdm_symbol_size = 128*osf;
    frame_parms->samples_per_tti = 1920*osf;
    frame_parms->first_carrier_offset = frame_parms->ofdm_symbol_size - 36;
    frame_parms->nb_prefix_samples>>=(4-log2_osf);
    frame_parms->nb_prefix_samples0>>=(4-log2_osf);
    frame_parms->N_RBGS = 1;
    frame_parms->N_RBG = 6;
    break;

  default:
    printf("init_frame_parms: Error: Number of resource blocks (N_RB_DL %d) undefined, frame_parms = %p \n",frame_parms->N_RB_DL, frame_parms);
    return(-1);
    break;
  }

  //LA:printf("lte_parms.c: Setting N_RB_DL to %d, ofdm_symbol_size %d\n",frame_parms->N_RB_DL, frame_parms->ofdm_symbol_size);
//  LOG_I(PHY,"Setting: N_RB_DL = %d, Ncp = %d, osf = %d, ofdm_symbol_size = %d.\n",frame_parms->N_RB_DL,frame_parms->Ncp,osf,frame_parms->ofdm_symbol_size);


  if (frame_parms->frame_type == TDD) set_S_config(frame_parms);

  //  frame_parms->tdd_config=3;
  return(0);
}


void dump_frame_parms(LTE_DL_FRAME_PARMS *frame_parms)
{
  printf("frame_parms->N_RB_DL=%"PRIu8"\n",frame_parms->N_RB_DL);
  printf("frame_parms->N_RB_UL=%"PRIu8"\n",frame_parms->N_RB_UL);
  printf("frame_parms->N_RBG=%u\n",frame_parms->N_RBG);
  printf("frame_parms->N_RBGS=%u\n",frame_parms->N_RBGS);
  printf("frame_parms->Nid_cell=%u\n",frame_parms->Nid_cell);
  printf("frame_parms->Nid_cell_mbsfn=%u\n",frame_parms->Nid_cell_mbsfn);
  printf("frame_parms->Ncp=%d\n",frame_parms->Ncp);
  printf("frame_parms->Ncp_UL=%d\n",frame_parms->Ncp_UL);
  printf("frame_parms->nushift=%d\n",frame_parms->nushift);
  printf("frame_parms->frame_type=%d\n",frame_parms->frame_type);
  printf("frame_parms->tdd_config=%u\n",frame_parms->tdd_config);
  printf("frame_parms->tdd_config_S=%u\n",frame_parms->tdd_config_S);
  printf("frame_parms->srsX=%u\n",frame_parms->srsX);
  printf("frame_parms->node_id=%"PRIu8"\n",frame_parms->node_id);
  printf("frame_parms->freq_idx=%"PRIu8"\n",frame_parms->freq_idx);
  //some excluded here
  printf("frame_parms->mode1_flag=%d\n",frame_parms->mode1_flag);
  printf("frame_parms->threequarter_fs=%"PRIu8"\n",frame_parms->threequarter_fs);
  printf("frame_parms->ofdm_symbol_size=%d\n",frame_parms->ofdm_symbol_size);
	printf("frame_parms->nb_prefix_samples=%d\n",frame_parms->nb_prefix_samples);
	printf("frame_parms->nb_prefix_samples0=%d\n",frame_parms->nb_prefix_samples0);
	printf("frame_parms->first_carrier_offset=%d\n",frame_parms->first_carrier_offset);
	printf("frame_parms->samples_per_tti=%d\n",frame_parms->samples_per_tti);
	printf("frame_parms->symbols_per_tti=%d\n",frame_parms->symbols_per_tti);
	printf("frame_parms->dl_symbols_in_S_subframe=%"PRIu16"\n",frame_parms->dl_symbols_in_S_subframe);
	printf("frame_parms->ul_symbols_in_S_subframe=%"PRIu16"\n",frame_parms->ul_symbols_in_S_subframe);
	printf("frame_parms->nb_antennas_tx=%d\n",frame_parms->nb_antennas_tx);
	  printf("frame_parms->nb_antennas_rx=%d\n",frame_parms->nb_antennas_rx);
  printf("frame_parms->nb_antenna_ports_eNB=%d\n",frame_parms->nb_antenna_ports_eNB);
  //common config excluded
  printf("frame_parms->num_MBSFN_config=%d\n",frame_parms->num_MBSFN_config);
  for(int i=0; i<MAX_MBSFN_AREA;i++)
	  printf("frame_parms->MBSFN_config[%d]: radioframeAllocationPeriod = %d, radioframeAllocationOffset = %d, fourFrames_flag = %d, mbsfn_SubframeConfig = %d\n",
			  i,frame_parms->MBSFN_config[i].radioframeAllocationPeriod,frame_parms->MBSFN_config[i].radioframeAllocationOffset,frame_parms->MBSFN_config[i].fourFrames_flag,frame_parms->MBSFN_config[i].mbsfn_SubframeConfig);

  printf("frame_parms->maxHARQ_Msg3Tx=%"PRIu8"\n",frame_parms->maxHARQ_Msg3Tx);
  printf("frame_parms->SIwindowsize=%"PRIu8"\n",frame_parms->SIwindowsize);
    printf("frame_parms->SIPeriod=%"PRIu16"\n",frame_parms->SIPeriod);
  printf("frame_parms->pcfich_reg={%"PRIu16",%"PRIu16",%"PRIu16",%"PRIu16"}\n",frame_parms->pcfich_reg[0],frame_parms->pcfich_reg[1],frame_parms->pcfich_reg[2],frame_parms->pcfich_reg[3]);
  printf("frame_parms->pcfich_first_reg_idx=%"PRIu8"\n",frame_parms->pcfich_first_reg_idx);
  printf("frame_parms->phich_reg[0][1][2]={[%u,%u,%u][%u,%u,%u][%u,%u,%u][%u,%u,%u]}\n",
		  frame_parms->phich_reg[0][0],frame_parms->phich_reg[0][1],frame_parms->phich_reg[0][2],
		  frame_parms->phich_reg[1][0],frame_parms->phich_reg[1][1],frame_parms->phich_reg[1][2],
		  frame_parms->phich_reg[2][0],frame_parms->phich_reg[2][1],frame_parms->phich_reg[2][2],
		  frame_parms->phich_reg[7][0],frame_parms->phich_reg[7][1],frame_parms->phich_reg[7][2]);

  printf("frame_parms->pdsch_config_common.referenceSignalPower=%"PRIi8" dBm\n",frame_parms->pdsch_config_common.referenceSignalPower);
  printf("frame_parms->pdsch_config_common.p_b=%"PRIu8"\n",frame_parms->pdsch_config_common.p_b);
  //printf("openair0_cfg[UE->rf_map.card].rx_num_channels = %d\n",openair0_cfg[UE->rf_map.card].rx_num_channels);
}

