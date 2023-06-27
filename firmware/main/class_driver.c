/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "usb/usb_host.h"

#include "midi_translator.h"

#define CLIENT_NUM_EVENT_MSG        5

#define ACTION_OPEN_DEV             0x01
#define ACTION_GET_DEV_INFO         0x02
#define ACTION_GET_DEV_DESC         0x04
#define ACTION_GET_CONFIG_DESC      0x08
#define ACTION_GET_STR_DESC         0x10
#define ACTION_CLOSE_DEV            0x20
#define ACTION_EXIT                 0x40

typedef struct {
    usb_host_client_handle_t client_hdl;
    uint8_t dev_addr;
    usb_device_handle_t dev_hdl;
    uint32_t actions;
    esp_timer_handle_t midi_timer_hdl;
    uint64_t midi_timer_period;
    uint64_t new_midi_timer_period;
} class_driver_t;



extern void led_connect_effect_start(void);
extern void led_disconnect_effect_start(void);

extern int uart_send_data(struct uart_midi_event_packet ev);


static const char *TAG = "CLASS";

static void client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg)
{
    class_driver_t *driver_obj = (class_driver_t *)arg;
    switch (event_msg->event) {
        case USB_HOST_CLIENT_EVENT_NEW_DEV:
            if (driver_obj->dev_addr == 0) {
                driver_obj->dev_addr = event_msg->new_dev.address;
                //Open the device next
                driver_obj->actions |= ACTION_OPEN_DEV;
            }
            break;
        case USB_HOST_CLIENT_EVENT_DEV_GONE:
            if (driver_obj->dev_hdl != NULL) {
                //Cancel any other actions and close the device next
                driver_obj->actions = ACTION_CLOSE_DEV;
            }
            break;
        default:
            //Should never occur
            abort();
    }
}

static void action_open_dev(class_driver_t *driver_obj)
{
    assert(driver_obj->dev_addr != 0);
    ESP_LOGI(TAG, "Opening device at address %d", driver_obj->dev_addr);
    ESP_ERROR_CHECK(usb_host_device_open(driver_obj->client_hdl, driver_obj->dev_addr, &driver_obj->dev_hdl));
    //Get the device's information next
    driver_obj->actions &= ~ACTION_OPEN_DEV;
    driver_obj->actions |= ACTION_GET_DEV_INFO;
}

static void action_get_info(class_driver_t *driver_obj)
{
    assert(driver_obj->dev_hdl != NULL);
    ESP_LOGI(TAG, "Getting device information");
    usb_device_info_t dev_info;
    ESP_ERROR_CHECK(usb_host_device_info(driver_obj->dev_hdl, &dev_info));
    ESP_LOGI(TAG, "\t%s speed", (dev_info.speed == USB_SPEED_LOW) ? "Low" : "Full");
    ESP_LOGI(TAG, "\tbConfigurationValue %d", dev_info.bConfigurationValue);
    //Todo: Print string descriptors

    //Get the device descriptor next
    driver_obj->actions &= ~ACTION_GET_DEV_INFO;
    driver_obj->actions |= ACTION_GET_DEV_DESC;
}


unsigned long loopcounter = 0;


static esp_err_t midi_find_intf_and_ep_desc(class_driver_t *driver_obj, const usb_ep_desc_t **in_ep, const usb_ep_desc_t **out_ep, int* intf_num)
{
    bool interface_found = false;
    const usb_config_desc_t *config_desc;
    const usb_device_desc_t *device_desc;
    int intf_idx = -1;
    int desc_offset = 0;
    

    // Get required descriptors
    ESP_ERROR_CHECK(usb_host_get_device_descriptor(driver_obj->dev_hdl, &device_desc));
    ESP_ERROR_CHECK(usb_host_get_active_config_descriptor(driver_obj->dev_hdl, &config_desc));

    #define USB_SUBCLASS_COMMON        0x02
    #define USB_DEVICE_PROTOCOL_IAD    0x01

    if ((device_desc->bDeviceClass == USB_CLASS_MISC) && (device_desc->bDeviceSubClass == USB_SUBCLASS_COMMON) &&
        (device_desc->bDeviceProtocol == USB_DEVICE_PROTOCOL_IAD)) {


        const usb_standard_desc_t *this_desc = (const usb_standard_desc_t *)config_desc;
        do {

            ESP_LOGI(TAG, "TRY");
            intf_idx++;

            this_desc = usb_parse_next_descriptor_of_type(
                this_desc, config_desc->wTotalLength, USB_B_DESCRIPTOR_TYPE_INTERFACE, &desc_offset);

            if (this_desc == NULL){
                ESP_LOGI(TAG, "INTERFACE NOT FOUND");
                break; // Reached end of configuration descriptor
            }

            const usb_intf_desc_t *intf_desc = (const usb_intf_desc_t *)this_desc;
            
            #define USB_SUBCLASS_MIDISTREAMING 0x03
            if (intf_desc->bInterfaceClass == USB_CLASS_AUDIO && intf_desc->bInterfaceSubClass == USB_SUBCLASS_MIDISTREAMING && intf_desc->bInterfaceProtocol == 0x00){

                ESP_LOGI(TAG, "MIDI INTERFACE FOUND %d", intf_idx);
                *intf_num = intf_idx;
                interface_found = true;

            }
            
        } while (!interface_found);


        if (interface_found){

            usb_intf_desc_t* intf_desc = usb_parse_interface_descriptor(config_desc, intf_idx, 0, &desc_offset);

            int temp_offset = desc_offset;
            for (int i = 0; i < 2; i++) {
                const usb_ep_desc_t *this_ep = usb_parse_endpoint_descriptor_by_index(intf_desc, i, config_desc->wTotalLength, &desc_offset);
                assert(this_ep);
                if (USB_EP_DESC_GET_EP_DIR(this_ep)) {
                    *in_ep = this_ep;
                    ESP_LOGI(TAG, "IN EP FOUND %d", this_ep->bEndpointAddress);
                } else {
                    *out_ep = this_ep;
                    ESP_LOGI(TAG, "OUT EP FOUND %d", this_ep->bEndpointAddress);
                }
                desc_offset = temp_offset;
            }

            return ESP_OK;

        }
    } 
    else{
        ESP_LOGI(TAG, "NOT IAD");

             const usb_standard_desc_t *this_desc = (const usb_standard_desc_t *)config_desc;
        do {

            ESP_LOGI(TAG, "TRY");
            intf_idx++;

            this_desc = usb_parse_next_descriptor_of_type(
                this_desc, config_desc->wTotalLength, USB_B_DESCRIPTOR_TYPE_INTERFACE, &desc_offset);

            if (this_desc == NULL){
                ESP_LOGI(TAG, "INTERFACE NOT FOUND");
                break; // Reached end of configuration descriptor
            }

            const usb_intf_desc_t *intf_desc = (const usb_intf_desc_t *)this_desc;
            
            #define USB_SUBCLASS_MIDISTREAMING 0x03
            if (intf_desc->bInterfaceClass == USB_CLASS_AUDIO && intf_desc->bInterfaceSubClass == USB_SUBCLASS_MIDISTREAMING && intf_desc->bInterfaceProtocol == 0x00){

                ESP_LOGI(TAG, "MIDI INTERFACE FOUND %d", intf_idx);
                *intf_num = intf_idx;
                interface_found = true;

            }
            
        } while (!interface_found);


        if (interface_found){

            usb_intf_desc_t* intf_desc = usb_parse_interface_descriptor(config_desc, intf_idx, 0, &desc_offset);

            int temp_offset = desc_offset;
            for (int i = 0; i < 2; i++) {
                const usb_ep_desc_t *this_ep = usb_parse_endpoint_descriptor_by_index(intf_desc, i, config_desc->wTotalLength, &desc_offset);
                assert(this_ep);
                if (USB_EP_DESC_GET_EP_DIR(this_ep)) {
                    *in_ep = this_ep;
                    ESP_LOGI(TAG, "IN EP FOUND %d", this_ep->bEndpointAddress);
                } else {
                    *out_ep = this_ep;
                    ESP_LOGI(TAG, "OUT EP FOUND %d", this_ep->bEndpointAddress);
                }
                desc_offset = temp_offset;
            }

            return ESP_OK;

        }
    }

    return ESP_ERR_NOT_FOUND;
}


static usb_transfer_t *transfer;

static void transfer_cb(usb_transfer_t *transfer)
{
    //This is function is called from within usb_host_client_handle_events(). Don't block and try to keep it short
    struct class_driver_t *class_driver_obj = (struct class_driver_t *)transfer->context;
    printf("OUT: Transfer status %d, actual number of bytes transferred %d\n", transfer->status, transfer->actual_num_bytes);

}

static usb_transfer_t *in_transfer;

static void transform_midi_packet(struct uart_midi_event_packet *uart_ev)
{
    ESP_LOGI(TAG, "MIDI packet pre-transform: 0x%02X 0x%02X 0x%02X", uart_ev->byte1, uart_ev->byte2, uart_ev->byte3);
    if ((uart_ev->byte1 & 0xF0) == 0xB0 && uart_ev->byte2 >= 0x15 && uart_ev->byte2 <= 0x1A) // MIDI CC
    {
        uint8_t cc_on_channel_zero = uart_ev->byte1 - 9;
        uint8_t channel_shift = uart_ev->byte2 - 0x15;
        uart_ev->byte1 = cc_on_channel_zero + channel_shift;
        uart_ev->byte2 = 0x5F; // Model:Samples track level CC
    }
    ESP_LOGI(TAG, "MIDI packet post-transform: 0x%02X 0x%02X 0x%02X", uart_ev->byte1, uart_ev->byte2, uart_ev->byte3);
}


static void in_transfer_cb(usb_transfer_t *in_transfer)
{
    //This is function is called from within usb_host_client_handle_events(). Don't block and try to keep it short
    class_driver_t *class_driver_obj = (class_driver_t *)in_transfer->context;
    //printf("IN: Transfer status %d, actual number of bytes transferred %d\n", in_transfer->status, in_transfer->actual_num_bytes);

    //printf("IN: %d %d %d %d", in_transfer->data_buffer[0], in_transfer->data_buffer[1], in_transfer->data_buffer[2], in_transfer->data_buffer[3]);


    struct usb_midi_event_packet usb_ev = {
        .byte0 = in_transfer->data_buffer[0],
        .byte1 = in_transfer->data_buffer[1],
        .byte2 = in_transfer->data_buffer[2],
        .byte3 = in_transfer->data_buffer[3]
    };

    struct uart_midi_event_packet uart_ev = usb_midi_to_uart(usb_ev);

    // control tempo based on mod wheel value
    if (uart_ev.byte1 == 0xB9)
    {
        // pulseRateMicroSec = 60 * 1000000 / minBPM * pulseRatePPQN + ((maxBPM - minBPM) * activeMidiCcIntervals * pulseRatePPQN / totalMidiCcIntervals)
        // minBPM = 70, maxBPM = 180, pulseRatePPQN = 24 (as per MIDI clock spec), total MIDI CC intervals = 128
        float new_period = ((float)60 * 1000000) / ((float)70 * 24 + ((180 - 70) * (uart_ev.byte3) * 24)/128);
        class_driver_obj->new_midi_timer_period = (uint64_t)new_period;
    }
    
    ESP_LOGI(TAG, "USB -> MIDI USB in: 0x%02X 0x%02X 0x%02X 0x%02X", usb_ev.byte0, usb_ev.byte1, usb_ev.byte2, usb_ev.byte3);
    transform_midi_packet(&uart_ev);
    uart_send_data(uart_ev);


    usb_host_transfer_submit(in_transfer);

}


static void action_get_dev_desc(class_driver_t *driver_obj)
{
    assert(driver_obj->dev_hdl != NULL);
    ESP_LOGI(TAG, "Getting device descriptor");
    const usb_device_desc_t *dev_desc;
    ESP_ERROR_CHECK(usb_host_get_device_descriptor(driver_obj->dev_hdl, &dev_desc));
    usb_print_device_descriptor(dev_desc);
    //Get the device's config descriptor next
    driver_obj->actions &= ~ACTION_GET_DEV_DESC;
    driver_obj->actions |= ACTION_GET_CONFIG_DESC;

}

static void action_get_config_desc(class_driver_t *driver_obj)
{
    assert(driver_obj->dev_hdl != NULL);
    ESP_LOGI(TAG, "Getting config descriptor");
    const usb_config_desc_t *config_desc;
    ESP_ERROR_CHECK(usb_host_get_active_config_descriptor(driver_obj->dev_hdl, &config_desc));
    usb_print_config_descriptor(config_desc, NULL);


    const usb_ep_desc_t *in_ep = NULL;
    const usb_ep_desc_t *out_ep = NULL;
    int intf_num = NULL;
    midi_find_intf_and_ep_desc(driver_obj, &in_ep, &out_ep, &intf_num);

    ESP_LOGI(TAG, "SUKU claim interface %d nice", intf_num);
    ESP_ERROR_CHECK(usb_host_interface_claim(driver_obj->client_hdl, driver_obj->dev_hdl, intf_num, 0));


    usb_host_transfer_alloc(4, 0, &transfer);

    //Send an OUT transfer to EP1
    memset(transfer->data_buffer, 0xAA, 4);
    transfer->num_bytes = 4;
    transfer->device_handle = driver_obj->dev_hdl;
    transfer->bEndpointAddress = out_ep->bEndpointAddress;
    transfer->callback = transfer_cb;
    transfer->context = (void *)&driver_obj;

    //SETUP IN TRANSFER
    usb_host_transfer_alloc(USB_EP_DESC_GET_MPS(in_ep), 0, &in_transfer);
    memset(in_transfer->data_buffer, 0xAA, 4);
    in_transfer->num_bytes = USB_EP_DESC_GET_MPS(in_ep);
    in_transfer->device_handle = driver_obj->dev_hdl;
    in_transfer->bEndpointAddress = in_ep->bEndpointAddress;
    in_transfer->callback = in_transfer_cb;
    in_transfer->context = (void *)driver_obj;


    ESP_LOGI(TAG, "SUKU start IN transfer on ep %02x nice", in_ep->bEndpointAddress);
    usb_host_transfer_submit(in_transfer);

    //Get the device's string descriptors next
    driver_obj->actions &= ~ACTION_GET_CONFIG_DESC;
    driver_obj->actions |= ACTION_GET_STR_DESC;
}

static void action_get_str_desc(class_driver_t *driver_obj)
{

    if (loopcounter == 0){
         assert(driver_obj->dev_hdl != NULL);
        usb_device_info_t dev_info;
        ESP_ERROR_CHECK(usb_host_device_info(driver_obj->dev_hdl, &dev_info));
        if (dev_info.str_desc_manufacturer) {
            ESP_LOGI(TAG, "Getting Manufacturer string descriptor");
            usb_print_string_descriptor(dev_info.str_desc_manufacturer);
        }
        if (dev_info.str_desc_product) {
            ESP_LOGI(TAG, "Getting Product string descriptor");
            usb_print_string_descriptor(dev_info.str_desc_product);
        }
        if (dev_info.str_desc_serial_num) {
            ESP_LOGI(TAG, "Getting Serial Number string descriptor");
            usb_print_string_descriptor(dev_info.str_desc_serial_num);
        }   
    }


    // if (loopcounter%10 == 1){

    //     usb_host_transfer_submit(transfer);

    // }


    loopcounter++;

    //Nothing to do until the device disconnects
    //driver_obj->actions &= ~ACTION_GET_STR_DESC;
}

static void aciton_close_dev(class_driver_t *driver_obj)
{
    ESP_ERROR_CHECK(usb_host_device_close(driver_obj->client_hdl, driver_obj->dev_hdl));
    driver_obj->dev_hdl = NULL;
    driver_obj->dev_addr = 0;
    //We need to exit the event handler loop
    driver_obj->actions &= ~ACTION_CLOSE_DEV;
    driver_obj->actions |= ACTION_EXIT;
}


static void timer_cb(void *arg)
{
    class_driver_t *driver_obj = (class_driver_t *)arg;
    struct uart_midi_event_packet clock =
    {
        .length = 1,
        .byte1 = 0xF8,
        .byte2 = 0,
        .byte3 = 0,
    };
    uart_send_data(clock);

    if (driver_obj->new_midi_timer_period != driver_obj->midi_timer_period)
    {
        ESP_LOGI(TAG, "Period changed from %llu -> %llu", driver_obj->midi_timer_period, driver_obj->new_midi_timer_period);
        esp_timer_restart(driver_obj->midi_timer_hdl, driver_obj->new_midi_timer_period);
        driver_obj->midi_timer_period = driver_obj->new_midi_timer_period;
    }
}


static void setup_timer_for_midi_clock(class_driver_t *driver_obj)
{
    esp_timer_create_args_t timer_args = {
        .callback = timer_cb,
        .arg = (void *)driver_obj,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "MIDI clock",
        .skip_unhandled_events = false,
    };
    driver_obj->midi_timer_period = 28409;
    driver_obj->new_midi_timer_period = 28409;
    esp_timer_create(&timer_args, &driver_obj->midi_timer_hdl);
}

static void start_midi_clock(class_driver_t *driver_obj)
{
    esp_timer_start_periodic(driver_obj->midi_timer_hdl, driver_obj->midi_timer_period);
    ESP_LOGI(TAG, "MIDI clock started");
}

static void stop_midi_clock(class_driver_t *driver_obj)
{
    esp_timer_stop(driver_obj->midi_timer_hdl);
    esp_timer_delete(driver_obj->midi_timer_hdl);
}


void class_driver_task(void *arg)
{
    SemaphoreHandle_t signaling_sem = (SemaphoreHandle_t)arg;
    class_driver_t driver_obj = {0};

    //Wait until daemon task has installed USB Host Library
    xSemaphoreTake(signaling_sem, portMAX_DELAY);

    ESP_LOGI(TAG, "Registering Client");
    usb_host_client_config_t client_config = {
        .is_synchronous = false,    //Synchronous clients currently not supported. Set this to false
        .max_num_event_msg = CLIENT_NUM_EVENT_MSG,
        .async = {
            .client_event_callback = client_event_cb,
            .callback_arg = (void *)&driver_obj,
        },
    };
    ESP_ERROR_CHECK(usb_host_client_register(&client_config, &driver_obj.client_hdl));
 
    setup_timer_for_midi_clock(&driver_obj);

    while (1) {

        //ESP_LOGI(TAG, "actions: %d, loopcounter: %d", driver_obj.actions, loopcounter);
        
        usb_host_client_handle_events(driver_obj.client_hdl, 10);



        if (driver_obj.actions & ACTION_OPEN_DEV) {
            action_open_dev(&driver_obj);
            start_midi_clock(&driver_obj);
        }
        if (driver_obj.actions & ACTION_GET_DEV_INFO) {
            action_get_info(&driver_obj);
        }
        if (driver_obj.actions & ACTION_GET_DEV_DESC) {
            action_get_dev_desc(&driver_obj);
            led_connect_effect_start();
        }
        if (driver_obj.actions & ACTION_GET_CONFIG_DESC) {
            action_get_config_desc(&driver_obj);
        }
        if (driver_obj.actions & ACTION_GET_STR_DESC) {
            action_get_str_desc(&driver_obj);
        }
        if (driver_obj.actions & ACTION_CLOSE_DEV) {
            aciton_close_dev(&driver_obj);
        }
        if (driver_obj.actions & ACTION_EXIT) {
            break;
        }
    }

    stop_midi_clock(&driver_obj);

    ESP_LOGI(TAG, "Deregistering Client");
    ESP_ERROR_CHECK(usb_host_client_deregister(driver_obj.client_hdl));

    //Wait to be deleted
    xSemaphoreGive(signaling_sem);
    vTaskSuspend(NULL);
}
