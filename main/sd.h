#pragma once

#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "esp_log.h"
#include "uart.h"

void sd_init(void);
void sd_deinit(void);
void sd_pau(void);
void sd_pla(void);
void sd_ne(void);
void sd_la(void);
