/* 
 weather.h - Weather data retrieval from api.openweathermap.org

 This file is part of the ESP32 Everest Run project
 https://github.com/krzychb/esp32-everest-run

 Copyright (c) 2016 Krzysztof Budzynski <krzychb@gazeta.pl>
 This work is licensed under the Apache License, Version 2.0, January 2004
 See the file LICENSE for details.
*/

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

int getCurrentVersion(void);
void initialise_version_connection(void);

#ifdef __cplusplus
}
#endif
