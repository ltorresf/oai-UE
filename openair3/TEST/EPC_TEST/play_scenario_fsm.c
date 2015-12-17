/*******************************************************************************
    OpenAirInterface
    Copyright(c) 1999 - 2014 Eurecom

    OpenAirInterface is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.


    OpenAirInterface is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with OpenAirInterface.The full GNU General Public License is
    included in this distribution in the file called "COPYING". If not,
    see <http://www.gnu.org/licenses/>.

  Contact Information
  OpenAirInterface Admin: openair_admin@eurecom.fr
  OpenAirInterface Tech : openair_tech@eurecom.fr
  OpenAirInterface Dev  : openair4g-devel@lists.eurecom.fr

  Address      : Eurecom, Campus SophiaTech, 450 Route des Chappes, CS 50193 - 06904 Biot Sophia Antipolis cedex, FRANCE

 *******************************************************************************/

/*
                                play_scenario_fsm.c
                                -------------------
  AUTHOR  : Lionel GAUTHIER
  COMPANY : EURECOM
  EMAIL   : Lionel.Gauthier@eurecom.fr
 */
#include <stdio.h>
#include <sys/time.h>
#include <pthread.h>

#include "intertask_interface.h"
#include "platform_types.h"
#include "assertions.h"
#include "play_scenario.h"
#include "s1ap_ies_defs.h"
#include "play_scenario_s1ap_eNB_defs.h"
#include "timer.h"

//------------------------------------------------------------------------------
extern int                    g_max_speed;
//------------------------------------------------------------------------------
et_scenario_t    *g_scenario  = NULL;
pthread_mutex_t   g_fsm_lock  = PTHREAD_MUTEX_INITIALIZER;
et_fsm_state_t    g_fsm_state = ET_FSM_STATE_NULL;
uint32_t          g_constraints = ET_BIT_MASK_MATCH_SCTP_STREAM | ET_BIT_MASK_MATCH_SCTP_SSN;
//------------------------------------------------------------------------------
int timeval_subtract (struct timeval * const result, struct timeval * const a, struct timeval * const b)
{
  struct timeval  b2;
  int nsec = 0;
  b2.tv_sec   = b->tv_sec;
  b2.tv_usec = b->tv_usec;


  /* Perform the carry for the later subtraction by updating y. */
  if (a->tv_usec < b2.tv_usec) {
    nsec = (b2.tv_usec - a->tv_usec) / 1000000 + 1;
    b2.tv_usec -= 1000000 * nsec;
    b2.tv_sec += nsec;
  }
  if (a->tv_usec - b2.tv_usec > 1000000) {
    nsec = (a->tv_usec - b2.tv_usec) / 1000000;
    b2.tv_usec += 1000000 * nsec;
    b2.tv_sec -= nsec;
  }

  /* Compute the time remaining to wait.
     tv_usec is certainly positive. */
  result->tv_sec = a->tv_sec - b2.tv_sec;
  result->tv_usec = a->tv_usec - b2.tv_usec;

  LOG_D(ENB_APP, "timeval_subtract(%ld.%06d, %ld.%06d)=%ld.%06d\n", a->tv_sec, a->tv_usec, b->tv_sec, b->tv_usec, result->tv_sec, result->tv_usec);

  return a->tv_sec < b2.tv_sec;
}


//------------------------------------------------------------------------------
void et_scenario_wait_rx_packet(et_packet_t * const packet)
{
  packet->status = ET_PACKET_STATUS_SCHEDULED_FOR_RECEIVING;
  g_fsm_state    = ET_FSM_STATE_WAITING_RX_EVENT;
  if (timer_setup (ET_FSM_STATE_WAITING_RX_EVENT_DELAY_SEC, 0, TASK_S1AP, INSTANCE_DEFAULT, TIMER_ONE_SHOT,
        packet, &packet->timer_id) < 0) {
    AssertFatal(0, " Can not start waiting RX event timer\n");
  }
  LOG_D(ENB_APP, "Waiting RX packet num %d\n", packet->packet_number);
}
//------------------------------------------------------------------------------
void et_scenario_schedule_tx_packet(et_packet_t * const packet)
{
  s1ap_eNB_instance_t *s1ap_eNB_instance = NULL;
  struct timeval  now                   = { .tv_sec = 0, .tv_usec = 0 };
  struct timeval  offset_last_tx_packet = { .tv_sec = 0, .tv_usec = 0 };
  struct timeval  offset_last_rx_packet = { .tv_sec = 0, .tv_usec = 0 };
  struct timeval  offset_tx_rx          = { .tv_sec = 0, .tv_usec = 0 };
  struct timeval  offset                = { .tv_sec = 0, .tv_usec = 0 };
  int             last_packet_was_tx    = 0;
  int             we_are_too_early      = 0;

  AssertFatal(NULL != packet, "packet argument is NULL");
  s1ap_eNB_instance = et_s1ap_eNB_get_instance(packet->enb_instance);
  AssertFatal(NULL != s1ap_eNB_instance, "Cannot get s1ap_eNB_instance_t for eNB instance %d", packet->enb_instance);

  LOG_D(ENB_APP, "%s\n", __FUNCTION__);
  g_fsm_state = ET_FSM_STATE_WAITING_TX_EVENT;

  switch (packet->sctp_hdr.chunk_type) {
    case SCTP_CID_DATA:
      // check if we can send it now
      // TODO: BUG we have to discard in scenario all packets that cannot be processed (SACK, COOKIEs, etc)
      AssertFatal(gettimeofday(&now, NULL) == 0, "gettimeofday failed");
      timeval_subtract(&offset_last_tx_packet,&now,&g_scenario->time_last_tx_packet);
      timeval_subtract(&offset_last_rx_packet,&now,&g_scenario->time_last_rx_packet);
      LOG_D(ENB_APP, "offset_last_tx_packet=%ld.%06d\n", offset_last_tx_packet.tv_sec, offset_last_tx_packet.tv_usec);
      LOG_D(ENB_APP, "offset_last_rx_packet=%ld.%06d\n", offset_last_rx_packet.tv_sec, offset_last_rx_packet.tv_usec);

      last_packet_was_tx = timeval_subtract(&offset_tx_rx,&offset_last_tx_packet,&offset_last_rx_packet);
      if (last_packet_was_tx) {
        LOG_D(ENB_APP, "last_packet_was_tx\n");
        we_are_too_early = timeval_subtract(&offset,&offset_last_tx_packet,&packet->time_relative_to_last_sent_packet);
        LOG_D(ENB_APP, "we_are_too_early=%d, offset=%ld.%06d\n", we_are_too_early, offset.tv_sec, offset.tv_usec);
      } else {
        LOG_D(ENB_APP, "last_packet_was_rx\n");
        we_are_too_early = timeval_subtract(&offset,&offset_last_rx_packet,&packet->time_relative_to_last_received_packet);
        LOG_D(ENB_APP, "we_are_too_early=%d, offset=%ld.%06d\n", we_are_too_early, offset.tv_sec, offset.tv_usec);
      }
      if ((0 < we_are_too_early) && (0 == g_max_speed)){
        // set timer
        if (offset.tv_sec < 0) offset.tv_sec = -offset.tv_sec;
        if (offset.tv_usec < 0) {
          offset.tv_usec = offset.tv_usec + 1000000;
          offset.tv_sec  -= 1;
        }

        LOG_D(ENB_APP, "Send packet num %u original frame number %u in %ld.%06d sec\n",
            packet->packet_number, packet->original_frame_number, offset.tv_sec, offset.tv_usec);

        packet->status = ET_PACKET_STATUS_SCHEDULED_FOR_SENDING;
        if (timer_setup (offset.tv_sec, offset.tv_usec, TASK_S1AP, INSTANCE_DEFAULT, TIMER_ONE_SHOT,packet, &packet->timer_id) < 0) {
          AssertFatal(0, " Can not start TX event timer\n");
        }
        // Done g_fsm_state = ET_FSM_STATE_WAITING_TX_EVENT;
      } else {
        LOG_D(ENB_APP, "Send packet num %u original frame number %u immediately\n", packet->packet_number, packet->original_frame_number);
        // send immediately
        AssertFatal(0 == gettimeofday(&packet->timestamp_packet, NULL), "gettimeofday() Failed");

        g_scenario->time_last_tx_packet.tv_sec  = packet->timestamp_packet.tv_sec;
        g_scenario->time_last_tx_packet.tv_usec = packet->timestamp_packet.tv_usec;

        et_s1ap_eNB_itti_send_sctp_data_req(
            packet->enb_instance,
            packet->sctp_hdr.u.data_hdr.assoc_id,
            packet->sctp_hdr.u.data_hdr.payload.binary_stream,
            packet->sctp_hdr.u.data_hdr.payload.binary_stream_allocated_size,
            packet->sctp_hdr.u.data_hdr.stream);
        packet->status = ET_PACKET_STATUS_SENT;
        g_scenario->next_packet    = g_scenario->next_packet->next;
        g_fsm_state = ET_FSM_STATE_RUNNING;
      }
      break;
    case SCTP_CID_INIT:
    case SCTP_CID_INIT_ACK:
      AssertFatal(0, "Invalid case TX packet SCTP_CID_INIT or SCTP_CID_INIT_ACK");
      break;
    default:
      AssertFatal(0, "Invalid case TX packet SCTP_CID %d", packet->sctp_hdr.chunk_type);
  }
}
//------------------------------------------------------------------------------
et_fsm_state_t et_scenario_fsm_notify_event_state_running(et_event_t event)
{

  switch (event.code){
    case ET_EVENT_TICK:
      while (NULL != g_scenario->next_packet) {
        LOG_D(ENB_APP, "EVENT_TICK: Considering packet num %d:\n", g_scenario->next_packet->packet_number);
        switch (g_scenario->next_packet->sctp_hdr.chunk_type) {
          case SCTP_CID_DATA :
            // no init in this scenario, may be sub-scenario
            if (g_scenario->next_packet->action == ET_PACKET_ACTION_S1C_SEND) {
              if (g_scenario->next_packet->status == ET_PACKET_STATUS_NONE) {
                et_scenario_schedule_tx_packet(g_scenario->next_packet);
                pthread_mutex_unlock(&g_fsm_lock);

                et_event_t continue_event;
                continue_event.code = ET_EVENT_TICK;
                et_scenario_fsm_notify_event(continue_event);

                return g_fsm_state;
              } else if (g_scenario->next_packet->status != ET_PACKET_STATUS_SCHEDULED_FOR_SENDING) {
                AssertFatal(0, "Invalid packet status %d", g_scenario->next_packet->status);
              }
            } else if (g_scenario->next_packet->action == ET_PACKET_ACTION_S1C_RECEIVE) {
              if (g_scenario->next_packet->status == ET_PACKET_STATUS_RECEIVED) {
                g_scenario->next_packet    = g_scenario->next_packet->next;

              } else if (g_scenario->next_packet->status == ET_PACKET_STATUS_NONE) {
                et_scenario_wait_rx_packet(g_scenario->next_packet);
                pthread_mutex_unlock(&g_fsm_lock);
                return g_fsm_state;
              } else {
                AssertFatal(0, "Invalid packet status %d", g_scenario->next_packet->status);
              }
            } else {
              AssertFatal(0, "Invalid packet action %d", g_scenario->next_packet->action);
            }
            break;

          case SCTP_CID_INIT:
          case SCTP_CID_INIT_ACK:
          case SCTP_CID_HEARTBEAT:
          case SCTP_CID_HEARTBEAT_ACK:
          case SCTP_CID_COOKIE_ECHO:
          case SCTP_CID_COOKIE_ACK:
          case SCTP_CID_ECN_ECNE:
          case SCTP_CID_ECN_CWR:
            LOG_D(ENB_APP, "EVENT_TICK: Ignoring packet num %d SCTP CID %s\n",
                g_scenario->next_packet->packet_number,
                et_chunk_type_cid2str(g_scenario->next_packet->sctp_hdr.chunk_type));
            g_scenario->next_packet->status = ET_PACKET_STATUS_NOT_TAKEN_IN_ACCOUNT;
            g_scenario->next_packet = g_scenario->next_packet->next;
            break;

          case SCTP_CID_ABORT:
          case SCTP_CID_SHUTDOWN:
          case SCTP_CID_SHUTDOWN_ACK:
          case SCTP_CID_ERROR:
          case SCTP_CID_SHUTDOWN_COMPLETE:
            AssertFatal(0, "The scenario should be cleaned (packet %s cannot be processed at this time)",
                et_chunk_type_cid2str(g_scenario->next_packet->sctp_hdr.chunk_type));
            break;

          default:
            LOG_D(ENB_APP, "EVENT_TICK: Ignoring packet num %d SCTP CID %s\n",
                g_scenario->next_packet->packet_number,
                et_chunk_type_cid2str(g_scenario->next_packet->sctp_hdr.chunk_type));
            g_scenario->next_packet->status = ET_PACKET_STATUS_NOT_TAKEN_IN_ACCOUNT;
            g_scenario->next_packet = g_scenario->next_packet->next;
        }
      }
      fprintf(stderr, "No Packet found in this scenario: %s\n", g_scenario->name);
      g_fsm_state = ET_FSM_STATE_NULL;
      pthread_mutex_unlock(&g_fsm_lock);
      return g_fsm_state;
      break;
    case ET_EVENT_RX_PACKET_TIME_OUT:
      AssertFatal(0, "Event ET_EVENT_RX_PACKET_TIME_OUT not handled in FSM state ET_FSM_STATE_RUNNING");
      break;
    case ET_EVENT_TX_TIMED_PACKET:
      AssertFatal(0, "Event ET_EVENT_TX_TIMED_PACKET not handled in FSM state ET_FSM_STATE_RUNNING");
      break;
    case ET_EVENT_RX_S1AP:
      et_s1ap_process_rx_packet(&event.u.s1ap_data_ind);
      break;
    default:
      AssertFatal(0, "Case event %d not handled in ET_FSM_STATE_RUNNING", event.code);
  }
  pthread_mutex_unlock(&g_fsm_lock);
  return 0;
}

//------------------------------------------------------------------------------
et_fsm_state_t et_scenario_fsm_notify_event_state_waiting_tx(et_event_t event)
{
  int rv = 0;
  switch (event.code){
    case ET_EVENT_TICK:
      fprintf(stdout, "EVENT_TICK: waiting for tx event\n");
      break;

    case ET_EVENT_RX_S1AP:
      rv = et_s1ap_process_rx_packet(&event.u.s1ap_data_ind);
      break;

    case ET_EVENT_TX_TIMED_PACKET:
      // send immediately
      AssertFatal(0 == gettimeofday(&event.u.tx_timed_packet->timestamp_packet, NULL), "gettimeofday() Failed");

      g_scenario->time_last_tx_packet.tv_sec  = event.u.tx_timed_packet->timestamp_packet.tv_sec;
      g_scenario->time_last_tx_packet.tv_usec = event.u.tx_timed_packet->timestamp_packet.tv_usec;

      et_s1ap_eNB_itti_send_sctp_data_req(
          event.u.tx_timed_packet->enb_instance,
          event.u.tx_timed_packet->sctp_hdr.u.data_hdr.assoc_id,
          event.u.tx_timed_packet->sctp_hdr.u.data_hdr.payload.binary_stream,
          event.u.tx_timed_packet->sctp_hdr.u.data_hdr.payload.binary_stream_allocated_size,
          event.u.tx_timed_packet->sctp_hdr.u.data_hdr.stream);
      event.u.tx_timed_packet->status = ET_PACKET_STATUS_SENT;
      g_scenario->next_packet    = event.u.tx_timed_packet->next;
      g_fsm_state = ET_FSM_STATE_RUNNING;
      break;

    case ET_EVENT_RX_PACKET_TIME_OUT:
    default:
      AssertFatal(0, "Case event %d not handled in ET_FSM_STATE_WAITING_TX", event.code);
  }
  pthread_mutex_unlock(&g_fsm_lock);
  return 0;
}

//------------------------------------------------------------------------------
et_fsm_state_t et_scenario_fsm_notify_event_state_waiting_rx(et_event_t event)
{
  int rv = 0;
  switch (event.code){
    case ET_EVENT_TICK:
      fprintf(stdout, "EVENT_TICK: waiting for rx event\n");
      break;

    case ET_EVENT_RX_PACKET_TIME_OUT:
      fprintf(stderr, "Error The following packet is not received:\n");
      et_display_packet(event.u.rx_packet_time_out);
      AssertFatal(0, "Waited packet not received");
      break;

    case ET_EVENT_RX_S1AP:
      rv = et_s1ap_process_rx_packet(&event.u.s1ap_data_ind);
      // waited packet
      if (rv == 0) {
        g_fsm_state = ET_FSM_STATE_RUNNING;
      }
      break;

    case ET_EVENT_TX_TIMED_PACKET:
    default:
      AssertFatal(0, "Case event %d not handled in ET_FSM_STATE_WAITING_RX", event.code);
  }
  pthread_mutex_unlock(&g_fsm_lock);
  return 0;
}

//------------------------------------------------------------------------------
et_fsm_state_t et_scenario_fsm_notify_event_state_connecting_s1c(et_event_t event)
{

  switch (event.code){
    case ET_EVENT_TICK:
      break;

    case ET_EVENT_S1C_CONNECTED:
      // hack simulate we have been able to get the right timing values for STCP connect
      AssertFatal(gettimeofday(&g_scenario->time_last_rx_packet, NULL) == 0, "gettimeofday failed");

      while (NULL != g_scenario->next_packet) {
        switch (g_scenario->next_packet->sctp_hdr.chunk_type) {
          case SCTP_CID_DATA :
            // no init in this scenario, may be sub-scenario
            if (g_scenario->next_packet->action == ET_PACKET_ACTION_S1C_SEND) {
              et_scenario_schedule_tx_packet(g_scenario->next_packet);
              pthread_mutex_unlock(&g_fsm_lock);

              et_event_t continue_event;
              continue_event.code = ET_EVENT_TICK;
              et_scenario_fsm_notify_event(continue_event);

              return g_fsm_state;
            } else if (g_scenario->next_packet->action == ET_PACKET_ACTION_S1C_RECEIVE) {
              if (g_scenario->next_packet->status == ET_PACKET_STATUS_RECEIVED) {
                g_scenario->last_rx_packet = g_scenario->next_packet;
                g_scenario->time_last_rx_packet = g_scenario->last_rx_packet->timestamp_packet;
                g_scenario->next_packet    = g_scenario->next_packet->next;

              } else if (g_scenario->next_packet->status == ET_PACKET_STATUS_NONE) {
                et_scenario_wait_rx_packet(g_scenario->next_packet);
                pthread_mutex_unlock(&g_fsm_lock);
                return g_fsm_state;
              } else {
                AssertFatal(0, "Invalid packet status %d", g_scenario->next_packet->status);
              }
            } else {
              AssertFatal(0, "Invalid packet action %d", g_scenario->next_packet->action);
            }
             break;

          case SCTP_CID_INIT:
          case SCTP_CID_INIT_ACK:
          case SCTP_CID_HEARTBEAT:
          case SCTP_CID_HEARTBEAT_ACK:
          case SCTP_CID_COOKIE_ECHO:
          case SCTP_CID_COOKIE_ACK:
          case SCTP_CID_ECN_ECNE:
          case SCTP_CID_ECN_CWR:
            g_scenario->next_packet->status = ET_PACKET_STATUS_NOT_TAKEN_IN_ACCOUNT;
            g_scenario->next_packet = g_scenario->next_packet->next;
            break;

          case SCTP_CID_ABORT:
          case SCTP_CID_SHUTDOWN:
          case SCTP_CID_SHUTDOWN_ACK:
          case SCTP_CID_ERROR:
          case SCTP_CID_SHUTDOWN_COMPLETE:
            AssertFatal(0, "The scenario should be cleaned (packet %s cannot be processed at this time)",
                et_chunk_type_cid2str(g_scenario->next_packet->sctp_hdr.chunk_type));
            break;

          default:
            g_scenario->next_packet->status = ET_PACKET_STATUS_NOT_TAKEN_IN_ACCOUNT;
            g_scenario->next_packet = g_scenario->next_packet->next;
        }
      }
      fprintf(stderr, "No Packet found in this scenario: %s\n", g_scenario->name);
      g_fsm_state = ET_FSM_STATE_NULL;
      pthread_mutex_unlock(&g_fsm_lock);
      return g_fsm_state;
      break;

    default:
      AssertFatal(0, "Case event %d not handled in ET_FSM_STATE_CONNECTING_S1C", event.code);
  }
  pthread_mutex_unlock(&g_fsm_lock);
  return 0;
}
//------------------------------------------------------------------------------
et_fsm_state_t et_scenario_fsm_notify_event_state_null(et_event_t event)
{
  switch (event.code){
    case ET_EVENT_TICK:
      break;

    case ET_EVENT_INIT:
      AssertFatal(NULL == g_scenario, "Current scenario not ended");
      g_scenario = event.u.init.scenario;
      g_scenario->next_packet            = g_scenario->list_packet;
      g_scenario->last_rx_packet         = NULL;
      g_scenario->last_tx_packet         = NULL;

      while (NULL != g_scenario->next_packet) {
        switch (g_scenario->next_packet->sctp_hdr.chunk_type) {

          case SCTP_CID_DATA :
            // no init in this scenario, may be sub-scenario, ...
            if (g_scenario->next_packet->action == ET_PACKET_ACTION_S1C_SEND) {
              et_scenario_schedule_tx_packet(g_scenario->next_packet);
              pthread_mutex_unlock(&g_fsm_lock);

              et_event_t continue_event;
              continue_event.code = ET_EVENT_TICK;
              et_scenario_fsm_notify_event(continue_event);

              return g_fsm_state;
            } else if (g_scenario->next_packet->action == ET_PACKET_ACTION_S1C_RECEIVE) {
              if (g_scenario->next_packet->status == ET_PACKET_STATUS_RECEIVED) {
                g_scenario->last_rx_packet = g_scenario->next_packet;
                g_scenario->time_last_rx_packet = g_scenario->last_rx_packet->timestamp_packet;
                g_scenario->next_packet    = g_scenario->next_packet->next;

              } else if (g_scenario->next_packet->status == ET_PACKET_STATUS_NONE) {
                et_scenario_wait_rx_packet(g_scenario->next_packet);
                pthread_mutex_unlock(&g_fsm_lock);
                return g_fsm_state;
              } else {
                AssertFatal(0, "Invalid packet status %d", g_scenario->next_packet->status);
              }
            } else {
              AssertFatal(0, "Invalid packet action %d", g_scenario->next_packet->action);
            }
            break;

          case SCTP_CID_INIT:
          case SCTP_CID_INIT_ACK:
            g_scenario->enb_properties       = (Enb_properties_array_t *)et_enb_config_get();
            g_scenario->hash_old_ue_mme_id2ue_mme_id = hashtable_create (256,NULL,NULL);
            g_scenario->hash_mme2association_id      = hashtable_create (256,NULL,NULL);
            // Try to register each eNB
            g_scenario->registered_enb       = 0;
            g_fsm_state            = ET_FSM_STATE_CONNECTING_S1C;
            AssertFatal(gettimeofday(&g_scenario->time_last_tx_packet, NULL) == 0, "gettimeofday failed");
            et_eNB_app_register (g_scenario->enb_properties);
            pthread_mutex_unlock(&g_fsm_lock);
            return g_fsm_state;
            break;

          case SCTP_CID_HEARTBEAT:
          case SCTP_CID_HEARTBEAT_ACK:
          case SCTP_CID_COOKIE_ECHO:
          case SCTP_CID_COOKIE_ACK:
          case SCTP_CID_ECN_ECNE:
          case SCTP_CID_ECN_CWR:
            g_scenario->next_packet->status = ET_PACKET_STATUS_NOT_TAKEN_IN_ACCOUNT;
            g_scenario->next_packet = g_scenario->next_packet->next;
            break;

          case SCTP_CID_ABORT:
          case SCTP_CID_SHUTDOWN:
          case SCTP_CID_SHUTDOWN_ACK:
          case SCTP_CID_ERROR:
          case SCTP_CID_SHUTDOWN_COMPLETE:
            AssertFatal(0, "The scenario should be cleaned (packet %s cannot be processed at this time)",
                et_chunk_type_cid2str(g_scenario->next_packet->sctp_hdr.chunk_type));
            break;

          default:
            g_scenario->next_packet->status = ET_PACKET_STATUS_NOT_TAKEN_IN_ACCOUNT;
            g_scenario->next_packet = g_scenario->next_packet->next;
        }
      }
      fprintf(stderr, "No Useful packet found in this scenario: %s\n", g_scenario->name);
      g_fsm_state = ET_FSM_STATE_NULL;
      pthread_mutex_unlock(&g_fsm_lock);
      return g_fsm_state;
      break;

    default:
      AssertFatal(0, "Case event %d not handled in ET_FSM_STATE_NULL", event.code);
  }
  return 0;
}

//------------------------------------------------------------------------------
et_fsm_state_t et_scenario_fsm_notify_event(et_event_t event)
{
  AssertFatal((event.code >= ET_EVENT_START) && (event.code < ET_EVENT_END), "Unknown et_event_t.code %d", event.code);

  pthread_mutex_lock(&g_fsm_lock);
  switch (g_fsm_state){
    case ET_FSM_STATE_NULL: return et_scenario_fsm_notify_event_state_null(event); break;
    case ET_FSM_STATE_CONNECTING_S1C: return et_scenario_fsm_notify_event_state_connecting_s1c(event); break;
    case ET_FSM_STATE_WAITING_TX_EVENT: return et_scenario_fsm_notify_event_state_waiting_tx(event); break;
    case ET_FSM_STATE_WAITING_RX_EVENT: return et_scenario_fsm_notify_event_state_waiting_rx(event); break;
    case ET_FSM_STATE_RUNNING: return et_scenario_fsm_notify_event_state_running(event); break;
    default:
      AssertFatal(0, "Case fsm_state %d not handled", g_fsm_state);
  }
  pthread_mutex_unlock(&g_fsm_lock);
  return g_fsm_state;
}
