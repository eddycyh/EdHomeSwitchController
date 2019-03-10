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

#include "version.h"
#include "edhome_ota.h"
#include "http.h"
#include "jsmn.h"

static const char* TAG = "VERSION";
static int version = 0;
/* Constants that aren't configurable in menuconfig
   Typically only LOCATION_ID may need to be changed
 */

int getCurrentVersion(void)
{
    return version;
}

static const char *get_request_version = {"GET /HomeSwitchController/version.txt HTTP/1.1\n"
    "Host: "OTA_SERVER_IP":"OTA_SERVER_PORT"\n"
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
    
    for(int j = 0; j < client->proc_buf_size; j++)
    {
        if( j != 0 && client->proc_buf[j] == '=')
        {
            if(client->proc_buf[j - 1] == 'N')
            {
                ESP_LOGI(TAG, "VERSION DETECTED!");
                version = ((client->proc_buf[j + 1] & 0x0F) * 100) + ((client->proc_buf[j + 2] & 0x0F) * 10) + (client->proc_buf[j+3] & 0x0F);
                if(version != 0)
                {
                    if(version > SOFTWARE_VERSION)
                    {
                        ESP_LOGI(TAG, "Start OTA current version %d to version %d...", SOFTWARE_VERSION, version);
                        initialise_edhome_ota();
                    }
                    else
                    {
                        ESP_LOGI(TAG, "CURRENT VERSION %d IS LATEST VERSION", version);
                    }
                }
            }
        }
    }

    free(client->proc_buf);
    client->proc_buf = NULL;
    client->proc_buf_size = 0;
    ESP_LOGD(TAG, "Free heap %u", xPortGetFreeHeapSize());
}

static void http_request_version_task(void *pvParameter)
{
    while(1) {
        ESP_LOGI(TAG, "PROCESSING VERSION");
        http_client_request_443(&http_client, OTA_SERVER_IP, get_request_version);
        vTaskDelay(10000 / portTICK_RATE_MS);
    }
}

void initialise_version_connection(void)
{
    http_client_on_process_chunk(&http_client, process_chunk);
    http_client_on_disconnected(&http_client, disconnected);
    xTaskCreate(&http_request_version_task, "http_request_version_task", 2 * 2048, NULL, 5, NULL);
    ESP_LOGI(TAG, "HTTP request for version task started");
}
