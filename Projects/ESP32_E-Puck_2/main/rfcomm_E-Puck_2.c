

#define __BTSTACK_FILE__ "rfcomm_E-Puck_2.c"

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/xtensa_api.h"
#include "freertos/queue.h"
#include "btstack.h"

#define RFCOMM_SERVER_CHANNEL 1
#define HEARTBEAT_PERIOD_MS 100

static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

static uint16_t rfcomm_channel_id;
static uint8_t  rfcomm_send_credit = 0;
static uint8_t  spp_service_buffer[4000];
static btstack_packet_callback_registration_t hci_event_callback_registration;



static void spp_service_setup(void){

    // register for HCI events
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    l2cap_init();

    rfcomm_init();
    rfcomm_register_service_with_initial_credits(packet_handler, RFCOMM_SERVER_CHANNEL, 0xffff,1);  // reserved channel, mtu limited by l2cap

    // init SDP, create record for SPP and register with SDP
    sdp_init();
    memset(spp_service_buffer, 0, sizeof(spp_service_buffer));
    spp_create_sdp_record(spp_service_buffer, 0x10001, RFCOMM_SERVER_CHANNEL, "SPP Counter");
    sdp_register_service(spp_service_buffer);
    printf("SDP service record size: %u\n", de_get_len(spp_service_buffer));
}

static btstack_timer_source_t heartbeat;
static char lineBuffer[4000];
static uint32_t packet_length = 0;
static void  heartbeat_handler(struct btstack_timer_source *ts){
    static int counter = 0;

    // if (rfcomm_send_credit){
    //     rfcomm_grant_credits(rfcomm_channel_id, 1);
    //     rfcomm_send_credit = 0;
    // }
    // if (rfcomm_channel_id){
    //     sprintf(lineBuffer, "BTstack counter %04u\n", ++counter);
    //     printf("%s", lineBuffer);

    //     rfcomm_request_can_send_now_event(rfcomm_channel_id);
    // }

    btstack_run_loop_set_timer(ts, HEARTBEAT_PERIOD_MS);
    btstack_run_loop_add_timer(ts);
} 

static void one_shot_timer_setup(void){
    // set one-shot timer
    heartbeat.process = &heartbeat_handler;
    btstack_run_loop_set_timer(&heartbeat, HEARTBEAT_PERIOD_MS);
    btstack_run_loop_add_timer(&heartbeat);
}

static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);


    bd_addr_t event_addr;
    uint8_t   rfcomm_channel_nr;
    static int32_t  mtu;
    int i;

    static int32_t nb_times_sent = 0;
    static int32_t remaining_bytes;

    switch (packet_type) {
        case HCI_EVENT_PACKET:
            switch (hci_event_packet_get_type(packet)) {
                case HCI_EVENT_PIN_CODE_REQUEST:
                    // inform about pin code request
                    printf("Pin code request - using '0000'\n");
                    hci_event_pin_code_request_get_bd_addr(packet, event_addr);
                    gap_pin_code_response(event_addr, "0000");
                    break;

                case HCI_EVENT_USER_CONFIRMATION_REQUEST:
                    // ssp: inform about user confirmation request
                    printf("SSP User Confirmation Request with numeric value '%06"PRIu32"'\n", little_endian_read_32(packet, 8));
                    printf("SSP User Confirmation Auto accept\n");
                    break;

                case RFCOMM_EVENT_INCOMING_CONNECTION:
                    // data: event (8), len(8), address(48), channel (8), rfcomm_cid (16)
                    rfcomm_event_incoming_connection_get_bd_addr(packet, event_addr); 
                    rfcomm_channel_nr = rfcomm_event_incoming_connection_get_server_channel(packet);
                    rfcomm_channel_id = rfcomm_event_incoming_connection_get_rfcomm_cid(packet);
                    printf("RFCOMM channel %u requested for %s\n", rfcomm_channel_nr, bd_addr_to_str(event_addr));
                    rfcomm_accept_connection(rfcomm_channel_id);
                    break;
               
                case RFCOMM_EVENT_CHANNEL_OPENED:
                    // data: event(8), len(8), status (8), address (48), server channel(8), rfcomm_cid(16), max frame size(16)
                    if (rfcomm_event_channel_opened_get_status(packet)) {
                        printf("RFCOMM channel open failed, status %u\n", rfcomm_event_channel_opened_get_status(packet));
                    } else {
                        rfcomm_channel_id = rfcomm_event_channel_opened_get_rfcomm_cid(packet);
                        mtu = rfcomm_event_channel_opened_get_max_frame_size(packet);
                        printf("RFCOMM channel open succeeded. New RFCOMM Channel ID %u, max frame size %u\n", rfcomm_channel_id, mtu);
                    }
                    break;
                case RFCOMM_EVENT_CAN_SEND_NOW:
                    vTaskDelay(1);
                    rfcomm_send(rfcomm_channel_id, (uint8_t*) lineBuffer, packet_length);
                    if((remaining_bytes - mtu) <= 0){
                        packet_length = remaining_bytes;
                        remaining_bytes = 0;
                    }else{
                        packet_length = mtu;
                        remaining_bytes -= mtu;
                    }

                    if (packet_length > 0){
                        rfcomm_request_can_send_now_event(rfcomm_channel_id);
                    }else{
                        nb_times_sent +=1;
                        remaining_bytes = 4000;
                        if(nb_times_sent < 49){
                            rfcomm_request_can_send_now_event(rfcomm_channel_id);
                        }
                    }
                    break;
               
                case RFCOMM_EVENT_CHANNEL_CLOSED:
                    printf("RFCOMM channel closed\n");
                    rfcomm_channel_id = 0;
                    break;
                
                default:
                    break;
            }
            break;

        case RFCOMM_DATA_PACKET:
            vTaskDelay(1);
            printf("RCV: %d characters '",size);
            // for (i=0;i<size;i++){
            //     putchar(packet[i]);
            // }
            printf("'\n");
            //rfcomm_send_credit = 1;
            rfcomm_grant_credits(rfcomm_channel_id, 1);

            //packet_length = size;
            //strcpy(lineBuffer, (char*)packet);
            //rfcomm_send(rfcomm_channel_id, (uint8_t*) lineBuffer, packet_length); 
            if(packet[0] == 'n'){
                uint32_t j = 0;
                for(j=0;j<4000;j++){
                    lineBuffer[j]=(char)j;
                }
                remaining_bytes = j;
                nb_times_sent = 0;
                printf("remaining_bytes = %d \n", remaining_bytes);
                printf("mtu = %d \n", mtu);
                if((remaining_bytes - mtu) <= 0){
                    packet_length = remaining_bytes;
                    remaining_bytes = 0;
                }else{
                    packet_length = mtu ;
                    remaining_bytes -= mtu;
                }

                rfcomm_request_can_send_now_event(rfcomm_channel_id);
            }
            
            break;

        default:
            break;
    }
}

int btstack_setup(int argc, const char * argv[]){

    one_shot_timer_setup();
    spp_service_setup();

    gap_discoverable_control(1);
    gap_ssp_set_io_capability(SSP_IO_CAPABILITY_DISPLAY_YES_NO);
    gap_set_local_name("SPP Counter 00:00:00:00:00:00");

    // turn on!
    hci_power_control(HCI_POWER_ON);

    return 0;
}

