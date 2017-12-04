/*

File    : rfcomm_E-Puck_2.c
Author  : Eliot Ferragni
Date    : 17 november 2017
REV 1.2

Functions to control and use the bluetooth stack
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

#include "rfcomm_e-puck2.h"
#include "button_e-puck2.h"

#define SERVICE_RECORD_1            0x10001   //Service Class ID List + ServiceName
#define SERVICE_RECORD_2            0x10002   //
#define RFCOMM_SERVER_CHANNEL_1     1         //channel 1
#define RFCOMM_SERVER_CHANNEL_2     2         //channel 2
#define RFCOMM_NAME_1               "11"
#define RFCOMM_NAME_2               "22"
#define HEARTBEAT_PERIOD_MS         1   //call at each loop
#define INITIAL_INCOMMING_CREDITS   1
#define OUTGOING_CREDITS_THRESHOLD  2
#define DELAY_10_TICKS              10
#define DELAY_1000_TICKS            1000                  


//internal functions
static void spp_service_setup(void);
static void heartbeat_handler(struct btstack_timer_source *ts);
static void one_shot_timer_setup(void);
static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void reset_blue_rx(void);
static void bluetooth_receive(uint16_t channel_id, uint8_t* buffer, uint16_t buffer_len, int32_t max_frame_size);
static void reset_blue_tx(void);
static void bluetooth_send(uint16_t channel_id, int32_t max_frame_size);


typedef struct {
    uint16_t remote_id;                //id assigned by the computer
    uint16_t server_id;         //internal id
    uint8_t  spp_service_buffer[150];   //buffer to store the service config
    uint32_t service_record;            //Config
    int32_t  mtu;
    uint8_t  blue_rx_buffer[BLUE_RX_BUFFER_SIZE];   //rx buffer
    uint8_t  blue_tx_buffer[BLUE_TX_BUFFER_SIZE];   //tx buffer
    ///////////////////////////////////////variables shared between threads/////////////////////////////////////////
    //rx variables
    uint16_t nb_to_read_blue_rx;
    uint16_t remaining_size_blue_rx;
    uint8_t  grant_incomming_credit;
    uint8_t  incomming_credits;
    uint8_t  nb_incomming_credit_to_grant;
    uint8_t* ptr_to_receive_blue_rx;
    uint8_t* ptr_to_read_blue_rx;
    SemaphoreHandle_t xReadBluetooth;
    //tx variables
    uint16_t nb_to_send_blue_tx;
    uint16_t remaining_size_blue_tx;
    uint8_t* ptr_to_send_blue_tx;
    uint8_t* ptr_to_fill_blue_tx;
    SemaphoreHandle_t xWriteBluetooth;
} rfcomm_user_channel_t;

////////////////////////////////////////////internal variables/////////////////////////////////////////////////
static btstack_timer_source_t heartbeat;
static btstack_packet_callback_registration_t hci_event_callback_registration;

///////////////////////////////////////variables shared between threads/////////////////////////////////////////
rfcomm_user_channel_t rf_channel[2];
//power mode variables
static uint8_t update_power_mode = 0;
static uint8_t power_mode_bluetooth_state = 0;
static SemaphoreHandle_t xPowerBluetooth = NULL;

//discoverability variables
static uint8_t update_discoverable = 0;
static uint8_t discoverable_bluetooth_state = 0;
static SemaphoreHandle_t xDiscoverableBluetooth = NULL;

//connectivity variables
static uint8_t update_connectable = 0;
static uint8_t connectable_bluetooth_state = 0;
static SemaphoreHandle_t xConnectableBluetooth = NULL;

static void init_rf_channels_struct(rfcomm_user_channel_t* channel){
   uint8_t i = 0;

   for(i = 0 ; i < 2 ; i++){
        //rx variables
        channel[i].nb_to_read_blue_rx = 0;
        channel[i].remaining_size_blue_rx = BLUE_RX_BUFFER_SIZE;
        channel[i].grant_incomming_credit = 0;
        channel[i].incomming_credits = INITIAL_INCOMMING_CREDITS;
        channel[i].nb_incomming_credit_to_grant = 0;
        channel[i].ptr_to_receive_blue_rx = channel[i].blue_rx_buffer;
        channel[i].ptr_to_read_blue_rx = channel[i].blue_rx_buffer;
        channel[i].xReadBluetooth = NULL;
        //tx variables
        channel[i].nb_to_send_blue_tx = 0;
        channel[i].remaining_size_blue_tx = BLUE_TX_BUFFER_SIZE;
        channel[i].ptr_to_send_blue_tx = channel[i].blue_tx_buffer;
        channel[i].ptr_to_fill_blue_tx = channel[i].blue_tx_buffer;
        channel[i].xWriteBluetooth = NULL;
   }
    channel[0].server_id = RFCOMM_SERVER_CHANNEL_1;
    channel[0].service_record = SERVICE_RECORD_1;

    channel[1].server_id = RFCOMM_SERVER_CHANNEL_2;
    channel[1].service_record = SERVICE_RECORD_2;
}

/* 
 * Set the spp service for the bluetooth
*/
static void spp_service_setup(void){

    // register for HCI events
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    l2cap_init();

    rfcomm_init();
    // reserved channel, mtu limited by l2cap
    rfcomm_register_service_with_initial_credits(packet_handler, rf_channel[0].server_id, 0xffff,INITIAL_INCOMMING_CREDITS);
    rfcomm_register_service_with_initial_credits(packet_handler, rf_channel[1].server_id, 0xffff,INITIAL_INCOMMING_CREDITS);

    // init SDP, create record for SPP and register with SDP
    sdp_init();
    memset(rf_channel[0].spp_service_buffer, 0, sizeof(rf_channel[0].spp_service_buffer));
    memset(rf_channel[1].spp_service_buffer, 0, sizeof(rf_channel[1].spp_service_buffer));
    spp_create_sdp_record(rf_channel[0].spp_service_buffer, rf_channel[0].service_record, rf_channel[0].server_id, RFCOMM_NAME_1);
    spp_create_sdp_record(rf_channel[1].spp_service_buffer, rf_channel[1].service_record, rf_channel[1].server_id, RFCOMM_NAME_2);
    sdp_register_service(rf_channel[0].spp_service_buffer);
    sdp_register_service(rf_channel[1].spp_service_buffer);
    printf("SDP service record size: %u\n", de_get_len(rf_channel[0].spp_service_buffer));
    printf("SDP service2 record size: %u\n", de_get_len(rf_channel[1].spp_service_buffer));
}

/* 
 * Handler called peridically by the main of btstack
 * by doing specific stuff here, we make sure it will be sync with
 * the thread of btstack. Unpredictable behaviors appear when functions of
 * btstac are called outside the btstack thread
*/
static void heartbeat_handler(struct btstack_timer_source *ts){

    //check if a send event is required
    if(xSemaphoreTake(rf_channel[0].xWriteBluetooth, (TickType_t)DELAY_10_TICKS) == pdTRUE){

        if(rf_channel[0].nb_to_send_blue_tx){
            xSemaphoreGive(rf_channel[0].xWriteBluetooth);
            rfcomm_request_can_send_now_event(rf_channel[0].remote_id);
        }
        xSemaphoreGive(rf_channel[0].xWriteBluetooth);

    }
    //check if incomming credits should be given
    if(xSemaphoreTake(rf_channel[0].xReadBluetooth, (TickType_t)DELAY_10_TICKS) == pdTRUE){
        if(rf_channel[0].grant_incomming_credit){
            rf_channel[0].grant_incomming_credit = NO_UPDATE;
            rf_channel[0].incomming_credits += rf_channel[0].nb_incomming_credit_to_grant;
            rfcomm_grant_credits(rf_channel[0].remote_id, rf_channel[0].nb_incomming_credit_to_grant);
        }
        xSemaphoreGive(rf_channel[0].xReadBluetooth);
    }
    //check if we need to change the state of the power of the bluetooth
    if(xSemaphoreTake(xPowerBluetooth, (TickType_t)DELAY_10_TICKS) == pdTRUE){
        if(update_power_mode){
            update_power_mode = NO_UPDATE;
            hci_power_control(power_mode_bluetooth_state);
        }
        xSemaphoreGive(xPowerBluetooth);
    }
    //check if we need to change the state of the discoverability of the bluetooth
    if(xSemaphoreTake(xDiscoverableBluetooth, (TickType_t)DELAY_10_TICKS) == pdTRUE){
        if(update_discoverable){
            update_discoverable = NO_UPDATE;
            gap_discoverable_control(discoverable_bluetooth_state);
        }
        xSemaphoreGive(xDiscoverableBluetooth);
    }
    //check if we need to change the state of the connectivity of the bluetooth
    if(xSemaphoreTake(xConnectableBluetooth, (TickType_t)DELAY_10_TICKS) == pdTRUE){
        if(update_connectable){
            update_connectable = NO_UPDATE;
            gap_connectable_control(connectable_bluetooth_state);
        }
        xSemaphoreGive(xConnectableBluetooth);
    }
    //reset the one shot timer to call this handler.
    btstack_run_loop_set_timer(ts, HEARTBEAT_PERIOD_MS);
    btstack_run_loop_add_timer(ts);
} 

/* 
 * Configure the heartbeat handler
*/
static void one_shot_timer_setup(void){
    // set one-shot timer
    heartbeat.process = &heartbeat_handler;
    btstack_run_loop_set_timer(&heartbeat, HEARTBEAT_PERIOD_MS);
    btstack_run_loop_add_timer(&heartbeat);
}

/* 
 * Function to handle the events related with the bluetooth communication
*/
static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);

    bd_addr_t event_addr;

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
                    rf_channel[0].server_id = rfcomm_event_incoming_connection_get_server_channel(packet);
                    rf_channel[0].remote_id = rfcomm_event_incoming_connection_get_rfcomm_cid(packet);
                    printf("RFCOMM channel %u requested for %s\n", rf_channel[0].server_id, bd_addr_to_str(event_addr));
                    rfcomm_accept_connection(rf_channel[0].remote_id);
                    break;
               
                case RFCOMM_EVENT_CHANNEL_OPENED:
                    // data: event(8), len(8), status (8), address (48), server channel(8), rfcomm_cid(16), max frame size(16)
                    if (rfcomm_event_channel_opened_get_status(packet)) {
                        printf("RFCOMM channel open failed, status %u\n", rfcomm_event_channel_opened_get_status(packet));
                    } else {
                        rf_channel[0].remote_id = rfcomm_event_channel_opened_get_rfcomm_cid(packet);
                        rf_channel[0].mtu = rfcomm_event_channel_opened_get_max_frame_size(packet);
                        rf_channel[0].nb_incomming_credit_to_grant = (BLUE_RX_BUFFER_SIZE / rf_channel[0].mtu);
                        rf_channel[0].incomming_credits = INITIAL_INCOMMING_CREDITS;
                        printf("RFCOMM channel open succeeded. New RFCOMM Channel ID %u, max frame size %u\n", rf_channel[0].remote_id, rf_channel[0].mtu);
                        bluetooth_discoverable_control(DISABLE);
                        bluetooth_connectable_control(DISABLE);
                    }
                    break;
                case RFCOMM_EVENT_CAN_SEND_NOW:
                    printf("channel = %d\n",channel);
                    bluetooth_send(rf_channel[0].remote_id, rf_channel[0].mtu);
                    break;
               
                case RFCOMM_EVENT_CHANNEL_CLOSED:
                    printf("RFCOMM channel closed\n");
                    rf_channel[0].remote_id = 0;
                    bluetooth_connectable_control(ENABLE);
                    break;
                
                default:
                    break;
            }
            break;

        case RFCOMM_DATA_PACKET:
            printf("channel = %d\n",channel);
            log_rfcomm("<==================RCV: %d characters =====================>",size);
            log_rfcomm("'\n");
            
            bluetooth_receive(rf_channel[0].remote_id, packet, size, rf_channel[0].mtu);
            
            break;

        default:
            break;
    }
}
/* 
 * Reset the pointers and counters of the blue_rx_buffer
*/
static void reset_blue_rx(void){
    rf_channel[0].ptr_to_receive_blue_rx = rf_channel[0].blue_rx_buffer;
    rf_channel[0].ptr_to_read_blue_rx = rf_channel[0].blue_rx_buffer;
    rf_channel[0].remaining_size_blue_rx = BLUE_RX_BUFFER_SIZE;
    rf_channel[0].nb_to_read_blue_rx = 0;
}

/* 
 * Function triggered by a receive bluetooth event which copy the received datas into the blue_rx_buffer
*/
static void bluetooth_receive(uint16_t channel_id, uint8_t* buffer, uint16_t buffer_len, int32_t max_frame_size){

    static uint16_t i = 0;

    log_rfcomm("incomming_credits = %d\n",rf_channel[0].incomming_credits);

    if(xSemaphoreTake(rf_channel[0].xReadBluetooth, (TickType_t)DELAY_1000_TICKS) == pdTRUE){
        log_rfcomm("received %d characters \n",buffer_len);
        rf_channel[0].remaining_size_blue_rx -= buffer_len;
        rf_channel[0].nb_to_read_blue_rx += buffer_len;
        rf_channel[0].incomming_credits--;

        for(i = 0 ; i < buffer_len ; i++){
            //the pointer is incremented after the copy of the value
            *rf_channel[0].ptr_to_receive_blue_rx++ = buffer[i];
        }
        log_rfcomm("remaining size = %d\n",rf_channel[0].remaining_size_blue_rx);
        xSemaphoreGive(rf_channel[0].xReadBluetooth);
    }
}

int16_t bluetooth_read(uint8_t* buffer, uint16_t buffer_len){

    static uint16_t i = 0;
    static uint16_t size_to_read = 0;

    if(rf_channel[0].remote_id){
        if(xSemaphoreTake(rf_channel[0].xReadBluetooth, (TickType_t)DELAY_1000_TICKS) == pdTRUE){
            if(rf_channel[0].nb_to_read_blue_rx){

                size_to_read = rf_channel[0].nb_to_read_blue_rx;
                if(rf_channel[0].nb_to_read_blue_rx > buffer_len){
                    size_to_read = buffer_len;
                }

                log_rfcomm("read %d characters\n",rf_channel[0].nb_to_read_blue_rx);
                rf_channel[0].nb_to_read_blue_rx -= size_to_read;
                rf_channel[0].remaining_size_blue_rx += size_to_read;

                for(i = 0 ; i < size_to_read ; i++){
                    //the pointer is incremented after the copy of the value
                    buffer[i] = *rf_channel[0].ptr_to_read_blue_rx++;
                }

                log_rfcomm("remaining size = %d, nb to read = %d\n",rf_channel[0].remaining_size_blue_rx, rf_channel[0].nb_to_read_blue_rx);

                if(!rf_channel[0].nb_to_read_blue_rx){
                    if(!rf_channel[0].grant_incomming_credit && !rf_channel[0].incomming_credits){
                        rf_channel[0].grant_incomming_credit = UPDATE;
                        log_rfcomm("grant credit bluetooth_read\n");
                    }
                    log_rfcomm("no more to read => reset ptr\n");
                    reset_blue_rx();
                }
                xSemaphoreGive(rf_channel[0].xReadBluetooth);
                return size_to_read;
            }else{
                xSemaphoreGive(rf_channel[0].xReadBluetooth);
                log_rfcomm("nothing received\n");
                return 0;
            }
        }else{
            log_rfcomm("semaphore not free bluetooth_read\n");
            return TASK_COLLISION;
        }
    }else{
        log_rfcomm("bluetooth is not connected\n");
        return BLUETOOTH_NOT_CONNECTED;
    }
}

/* 
 * Reset the pointers and counters of the blue_tx_buffer
*/
static void reset_blue_tx(void){
    rf_channel[0].ptr_to_send_blue_tx = rf_channel[0].blue_tx_buffer;
    rf_channel[0].ptr_to_fill_blue_tx = rf_channel[0].blue_tx_buffer;
    rf_channel[0].remaining_size_blue_tx = BLUE_TX_BUFFER_SIZE;
    rf_channel[0].nb_to_send_blue_tx = 0;
}

/* 
 * Function triggered by a send bluetooth event which send the datas over bluetooth
*/
static void bluetooth_send(uint16_t channel_id, int32_t max_frame_size){

    static uint16_t credits = 0;
    static uint16_t size_to_send = 0;

    if(xSemaphoreTake(rf_channel[0].xWriteBluetooth, (TickType_t)DELAY_1000_TICKS) == pdTRUE){
        size_to_send = rf_channel[0].nb_to_send_blue_tx;
        if(rf_channel[0].nb_to_send_blue_tx > max_frame_size){
            size_to_send = max_frame_size;
        }
        xSemaphoreGive(rf_channel[0].xWriteBluetooth);
    }
    credits = rfcomm_get_outgoing_credits(rf_channel[0].remote_id);
    log_rfcomm("outgoing credits = %d\n",credits);
    if(credits <= OUTGOING_CREDITS_THRESHOLD){
        //stop the sending during 100ms in order to let the computer digest the datas and 
        //free new credits for us => continue the sending only if we have more than 2 credits
        //send an empty packet to tell to the computer to refresh the credits (needed at least for OS X)
        vTaskDelay(100 / portTICK_PERIOD_MS);
        rfcomm_send(channel_id, rf_channel[0].ptr_to_send_blue_tx, 0);
    }else{
        rfcomm_send(channel_id, rf_channel[0].ptr_to_send_blue_tx, size_to_send);

        if(xSemaphoreTake(rf_channel[0].xWriteBluetooth, (TickType_t)DELAY_1000_TICKS) == pdTRUE){
            rf_channel[0].nb_to_send_blue_tx -= size_to_send;
            xSemaphoreGive(rf_channel[0].xWriteBluetooth);
        }
        rf_channel[0].ptr_to_send_blue_tx += size_to_send;
        log_rfcomm(" %d bytes sent, remaining = %d\n",size_to_send, rf_channel[0].nb_to_send_blue_tx);
    }

    if(rf_channel[0].nb_to_send_blue_tx > 0){
        log_rfcomm("must send another\n");
        rfcomm_request_can_send_now_event(channel_id);
    }else{
        //reset the pointers
        if(xSemaphoreTake(rf_channel[0].xWriteBluetooth, (TickType_t)DELAY_1000_TICKS) == pdTRUE){
            reset_blue_tx();
            xSemaphoreGive(rf_channel[0].xWriteBluetooth);
        }
        log_rfcomm("must not send => reset ptr\n");
    }
}

int8_t bluetooth_write(uint8_t* buffer, uint16_t buffer_len){
    static uint16_t i = 0;

    if(rf_channel[0].remote_id){
        if(xSemaphoreTake(rf_channel[0].xWriteBluetooth, (TickType_t)DELAY_1000_TICKS) == pdTRUE){
            if(rf_channel[0].remaining_size_blue_tx >= buffer_len){

                //increment counters for the tx buffer
                rf_channel[0].nb_to_send_blue_tx += buffer_len;
                rf_channel[0].remaining_size_blue_tx -= buffer_len;

                //copy datas into the tx buffer
                for(i = 0 ; i < buffer_len ; i++){
                    //the pointer is incremented after the copy of the value
                    *rf_channel[0].ptr_to_fill_blue_tx++ = buffer[i];
                }
                
                xSemaphoreGive(rf_channel[0].xWriteBluetooth);

                log_rfcomm("wrote %d bytes to send buffer\n", buffer_len);
                log_rfcomm("remaining size = %d, nb_to_send = %d\n",rf_channel[0].remaining_size_blue_tx, rf_channel[0].nb_to_send_blue_tx);
                
                return DATAS_WRITTEN;
            }else{
                xSemaphoreGive(rf_channel[0].xWriteBluetooth);
                log_rfcomm("not enough space\n");
                return BUFFER_FULL;
            }
        }else{
            log_rfcomm("semaphore not free bluetooth_write\n");
            return TASK_COLLISION;
        }
        
    }else{
        log_rfcomm("bluetooth is not connected\n");
        return BLUETOOTH_NOT_CONNECTED;
    }
}

void bluetooth_power_control(HCI_POWER_MODE power_mode){
    if(xSemaphoreTake(xPowerBluetooth, (TickType_t)DELAY_1000_TICKS) == pdTRUE){
        power_mode_bluetooth_state = power_mode;
        update_power_mode = UPDATE;
        xSemaphoreGive(xPowerBluetooth);
    }
}

void bluetooth_discoverable_control(CONTROL_STATE state){
    if(xSemaphoreTake(xDiscoverableBluetooth, (TickType_t)DELAY_1000_TICKS) == pdTRUE){
        discoverable_bluetooth_state = state;
        update_discoverable = UPDATE;
        xSemaphoreGive(xDiscoverableBluetooth);
    }
}

void bluetooth_connectable_control(CONTROL_STATE state){
    if(xSemaphoreTake(xConnectableBluetooth, (TickType_t)DELAY_1000_TICKS) == pdTRUE){
        connectable_bluetooth_state = state;
        update_connectable = UPDATE;
        xSemaphoreGive(xConnectableBluetooth);
    }
}

void example_echo_bluetooth_task(void *pvParameter){
  uint8_t test_buf[2000];
  uint16_t size = 2000;

  while(1){
    vTaskDelay(10 / portTICK_PERIOD_MS);
    int16_t rcv = bluetooth_read(test_buf, size);
    if(rcv > 0){
      while(bluetooth_write(test_buf,rcv) != DATAS_WRITTEN){
        vTaskDelay(10 / portTICK_PERIOD_MS);
      }
    }
  }
}

/* 
 * Function to configure and initiate the bluetooth communication
 * called by btstack_main located in /components/btstack/main.c
*/
int btstack_setup(int argc, const char * argv[]){

    init_rf_channels_struct(rf_channel);

    rf_channel[0].xReadBluetooth = xSemaphoreCreateBinary();
    xSemaphoreGive(rf_channel[0].xReadBluetooth);

    rf_channel[0].xWriteBluetooth = xSemaphoreCreateBinary();
    xSemaphoreGive(rf_channel[0].xWriteBluetooth);

    xPowerBluetooth = xSemaphoreCreateBinary();
    xSemaphoreGive(xPowerBluetooth);

    xDiscoverableBluetooth = xSemaphoreCreateBinary();
    xSemaphoreGive(xDiscoverableBluetooth);

    xConnectableBluetooth = xSemaphoreCreateBinary();
    xSemaphoreGive(xConnectableBluetooth);
    
    one_shot_timer_setup();
    spp_service_setup();

    gap_ssp_set_io_capability(SSP_IO_CAPABILITY_DISPLAY_ONLY);
    gap_set_local_name("e-puck2 00:00:00:00:00:00");

    //enable the discoverability of the bluetooth if the button is pressed during the startup
    if(button_is_pressed()){
        gap_discoverable_control(ENABLE);
    }

    // turn on!
    hci_power_control(HCI_POWER_ON);

    return 0;
}

