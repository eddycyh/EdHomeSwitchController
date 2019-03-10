/*
 weather.c - Weather data retrieval from api.openweathermap.org

 This file is part of the ESP32 Everest Run project
 https://github.com/krzychb/esp32-everest-run

 Copyright (c) 2016 Krzysztof Budzynski <krzychb@gazeta.pl>
 This work is licensed under the Apache License, Version 2.0, January 2004
 See the file LICENSE for details.
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"

#include <stdio.h>
#include <string.h>

#include "external.h"
#include "http.h"
#include "jsmn.h"

static const char* TAG = "External";

/* Constants that aren't configurable in menuconfig
   Typically only LOCATION_ID may need to be changed
 */
#define WEB_SERVER "192.168.0.199"

static uint8_t relayStatus[8] = {0,0,0,0,0,0,0,0,0};
static uint8_t prevStatus[8] = {0,0,0,0,0,0,0,0};

void UpdateRelayStatus(uint8_t* currStatus)
{
    for(int i = 0; i < 8; i++)
        relayStatus[i] = currStatus[i];
}

static const char *get_request_on[8] = {"GET /on1 HTTP/1.1\n"
    "Host: "WEB_SERVER"\n"
    "User-Agent: esp-idf/1.0 esp32\n"
    "\n", 
    "GET /on2 HTTP/1.1\n"
    "Host: "WEB_SERVER"\n"
    "User-Agent: esp-idf/1.0 esp32\n"
    "\n", 
    "GET /on3 HTTP/1.1\n"
    "Host: "WEB_SERVER"\n"
    "User-Agent: esp-idf/1.0 esp32\n"
    "\n", 
    "GET /on4 HTTP/1.1\n"
    "Host: "WEB_SERVER"\n"
    "User-Agent: esp-idf/1.0 esp32\n"
    "\n",
    "GET /on5 HTTP/1.1\n"
    "Host: "WEB_SERVER"\n"
    "User-Agent: esp-idf/1.0 esp32\n"
    "\n",
    "GET /on6 HTTP/1.1\n"
    "Host: "WEB_SERVER"\n"
    "User-Agent: esp-idf/1.0 esp32\n"
    "\n", 
    "GET /on7 HTTP/1.1\n"
    "Host: "WEB_SERVER"\n"
    "User-Agent: esp-idf/1.0 esp32\n"
    "\n", 
    "GET /on8 HTTP/1.1\n"
    "Host: "WEB_SERVER"\n"
    "User-Agent: esp-idf/1.0 esp32\n"
    "\n"};

static const char *get_request_off[8] = {"GET /off1 HTTP/1.1\n"
    "Host: "WEB_SERVER"\n"
    "User-Agent: esp-idf/1.0 esp32\n"
    "\n", 
    "GET /off2 HTTP/1.1\n"
    "Host: "WEB_SERVER"\n"
    "User-Agent: esp-idf/1.0 esp32\n"
    "\n",
    "GET /off3 HTTP/1.1\n"
    "Host: "WEB_SERVER"\n"
    "Connection: close\n"
    "User-Agent: esp-idf/1.0 esp32\n"
    "\n",
    "GET /off4 HTTP/1.1\n"
    "Host: "WEB_SERVER"\n"
    "Connection: close\n"
    "User-Agent: esp-idf/1.0 esp32\n"
    "\n",
    "GET /off5 HTTP/1.1\n"
    "Host: "WEB_SERVER"\n"
    "Connection: close\n"
    "User-Agent: esp-idf/1.0 esp32\n"
    "\n",
    "GET /off6 HTTP/1.1\n"
    "Host: "WEB_SERVER"\n"
    "Connection: close\n"
    "User-Agent: esp-idf/1.0 esp32\n"
    "\n",
    "GET /off7 HTTP/1.1\n"
    "Host: "WEB_SERVER"\n"
    "Connection: close\n"
    "User-Agent: esp-idf/1.0 esp32\n"
    "\n",
    "GET /off8 HTTP/1.1\n"
    "Host: "WEB_SERVER"\n"
    "Connection: close\n"
    "User-Agent: esp-idf/1.0 esp32\n"
    "\n"};

static http_client_data http_client = {0};

static void process_chunk(uint32_t *args)
{
    http_client_data* client = (http_client_data*)args;

    int proc_buf_new_size = client->proc_buf_size + client->recv_buf_size;
    char *copy_from;

    if (client->proc_buf == NULL){
        client->proc_buf = malloc(proc_buf_new_size);
        copy_from = client->proc_buf;
    } else {
        proc_buf_new_size -= 1; // chunks of data are '\0' terminated
        client->proc_buf = realloc(client->proc_buf, proc_buf_new_size);
        copy_from = client->proc_buf + proc_buf_new_size - client->recv_buf_size;
    }
    if (client->proc_buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory");
    }
    client->proc_buf_size = proc_buf_new_size;
    memcpy(copy_from, client->recv_buf, client->recv_buf_size);
}

static void disconnected(uint32_t *args)
{
    http_client_data* client = (http_client_data*)args;
    bool weather_data_phrased = false;

    const char * response_body = find_response_body(client->proc_buf);
    if (response_body) {
        ESP_LOGI(TAG, "INFORMATION SUCCEED");
    } else {
        ESP_LOGE(TAG, "No HTTP header found");
    }

    free(client->proc_buf);
    client->proc_buf = NULL;
    client->proc_buf_size = 0;
    ESP_LOGD(TAG, "Free heap %u", xPortGetFreeHeapSize());
}

static void http_request_task(void *pvParameter)
{
    while(1) {
        for (int i = 0; i < 8; i++)
        {
            if(relayStatus[i] != prevStatus[i])
            {
                prevStatus[i] = relayStatus[i];
                if(!relayStatus[i])
                {
                    http_client_request(&http_client, WEB_SERVER, get_request_off[i]);
                }
                else
                {
                    http_client_request(&http_client, WEB_SERVER, get_request_on[i]);
                }
            }
        }
        vTaskDelay(200 / portTICK_RATE_MS);
    }
}

void initialise_external_connection(void)
{
    xTaskCreate(&http_request_task, "http_request_external_task", 2 * 2048, NULL, 5, NULL);
    ESP_LOGI(TAG, "HTTP request for External devices task started");
}
