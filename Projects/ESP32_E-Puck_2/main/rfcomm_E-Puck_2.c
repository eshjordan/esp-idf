/*

File    : rfcomm_E-Puck_2.c
Author  : Eliot Ferragni
Date    : 12 november 2017
REV 1.1

Functions to comtrol and use the bluetooth stack
*/

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
#include "freertos/semphr.h"
#include "btstack.h"

#include "rfcomm_E-Puck_2.h"

#define RFCOMM_SERVER_CHANNEL 1
#define HEARTBEAT_PERIOD_MS 1

static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void bluetooth_send(uint16_t channel_id, int32_t max_frame_size);
static void reset_blue_tx(void);

static uint16_t rfcomm_channel_id;
static uint8_t  spp_service_buffer[150];
static btstack_packet_callback_registration_t hci_event_callback_registration;

static uint8_t  blue_rx_buffer[BLUE_RX_BUFFER_SIZE];

static uint8_t  blue_tx_buffer[BLUE_TX_BUFFER_SIZE];
static uint8_t* ptr_to_send_blue_tx = blue_tx_buffer;
static uint8_t* ptr_to_fill_blue_tx = blue_tx_buffer;

//shared variables between threads
static uint16_t nb_to_send_blue_tx = 0;
static uint16_t remaining_size_blue_tx = BLUE_TX_BUFFER_SIZE;
static SemaphoreHandle_t xWriteBluetooth = NULL;

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
    spp_create_sdp_record(spp_service_buffer, 0x10001, RFCOMM_SERVER_CHANNEL, "e-puck2");
    sdp_register_service(spp_service_buffer);
    printf("SDP service record size: %u\n", de_get_len(spp_service_buffer));
}

static btstack_timer_source_t heartbeat;
static void  heartbeat_handler(struct btstack_timer_source *ts){

    if(xSemaphoreTake(xWriteBluetooth, (TickType_t)10) == pdTRUE){

        if(nb_to_send_blue_tx){
            xSemaphoreGive(xWriteBluetooth);
            rfcomm_request_can_send_now_event(rfcomm_channel_id);
        }
        xSemaphoreGive(xWriteBluetooth);

    }else{
        printf("semaphore not free heartbeat\n");
    }

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
                    bluetooth_send(rfcomm_channel_id, mtu);
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
            printf("<==================RCV: %d characters =====================>",size);
            printf("'\n");
            rfcomm_grant_credits(rfcomm_channel_id, 1);
            
            break;

        default:
            break;
    }
}

static void reset_blue_tx(void){
    ptr_to_send_blue_tx = blue_tx_buffer;
    ptr_to_fill_blue_tx = blue_tx_buffer;
    remaining_size_blue_tx = BLUE_TX_BUFFER_SIZE;
    nb_to_send_blue_tx = 0;
}

static void bluetooth_send(uint16_t channel_id, int32_t max_frame_size){
    static uint32_t envoyes = 0;

    static uint16_t credits = 0;
    static uint16_t size_to_send = 0;
    //usefull to avoid a watchdog trigger when the sending is very slow
    vTaskDelay(1);  

    if(xSemaphoreTake(xWriteBluetooth, (TickType_t)1000) == pdTRUE){
        size_to_send = nb_to_send_blue_tx;
        if(nb_to_send_blue_tx > max_frame_size){
            size_to_send = max_frame_size;
        }
        xSemaphoreGive(xWriteBluetooth);
    }
    credits = rfcomm_get_outgoing_credits(channel_id);
    printf("credits restants = %d\n",credits);
    if(credits <= 2){
        //stop the sending during 100ms in order to let the computer digest the datas and 
        //free new credits for us => continue the sending only if we have more than 2 credits
        //send an empty packet to tell to the computer to refresh the credits (needed at least for OS X)
        vTaskDelay(100 / portTICK_PERIOD_MS);
        rfcomm_send(channel_id, ptr_to_send_blue_tx, 0);
    }else{
        rfcomm_send(channel_id, ptr_to_send_blue_tx, size_to_send);

        if(xSemaphoreTake(xWriteBluetooth, (TickType_t)1000) == pdTRUE){
            nb_to_send_blue_tx -= size_to_send;
            xSemaphoreGive(xWriteBluetooth);
        }
        ptr_to_send_blue_tx += size_to_send;
        printf(" %d bytes sent, remaining = %d\n",size_to_send, nb_to_send_blue_tx);
    }

    if(nb_to_send_blue_tx > 0){
        printf("must send another\n");
        rfcomm_request_can_send_now_event(channel_id);
    }else{
        //reset the pointers
        if(xSemaphoreTake(xWriteBluetooth, (TickType_t)1000) == pdTRUE){
            reset_blue_tx();
            xSemaphoreGive(xWriteBluetooth);
        }
        printf("must not send => reset ptr\n");
    }
}

bool bluetooth_write(uint8_t* buffer, uint16_t buffer_len){
    static bool ret = true;
    static uint16_t i = 0;

    if(rfcomm_channel_id){
        if(xSemaphoreTake(xWriteBluetooth, (TickType_t)1000) == pdTRUE){
            if(remaining_size_blue_tx >= buffer_len){

                //increment counters for the tx buffer
                nb_to_send_blue_tx += buffer_len;
                remaining_size_blue_tx -= buffer_len;

                //copy datas into the tx buffer
                for(i = 0 ; i < buffer_len ; i++){
                    //the pointer is incremented after the copy of the value
                    *ptr_to_fill_blue_tx++ = buffer[i];
                }
                
                xSemaphoreGive(xWriteBluetooth);

                printf("wrote %d bytes to send buffer\n", buffer_len);
                printf("remaining size = %d, nb_to_send = %d\n",remaining_size_blue_tx, nb_to_send_blue_tx);
                
                ret = true;
            }else{
                xSemaphoreGive(xWriteBluetooth);
                printf("not enough space\n");
                ret = false;
            }
        }else{
            printf("semaphore not free bluetooth_write\n");
            ret = false;
        }
        
    }else{
        printf("bluetooth is not connected\n");
        ret = false;
    }

    return ret;
}

uint16_t bluetooth_read(uint8_t* buffer, uint16_t buffer_len){
    return 4;
}

int btstack_setup(int argc, const char * argv[]){

    xWriteBluetooth = xSemaphoreCreateBinary();
    xSemaphoreGive(xWriteBluetooth);

    one_shot_timer_setup();
    spp_service_setup();

    gap_discoverable_control(1);
    gap_ssp_set_io_capability(SSP_IO_CAPABILITY_DISPLAY_ONLY);
    gap_set_local_name("e-puck2 00:00:00:00:00:00");

    // turn on!
    hci_power_control(HCI_POWER_ON);

    return 0;
}

