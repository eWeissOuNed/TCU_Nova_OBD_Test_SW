#include "sdcard.h"
#include "../config.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <dirent.h>
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include "esp_timer.h"

static const char    *TAG    = "SDCARD";
static sdmmc_card_t  *s_card = NULL;
static bool           s_mounted = false;

/* ── enable pin helpers ──────────────────────────────────────────────────── */
static void sd_power(bool on)
{
    if (on) {
        gpio_config_t cfg = {
            .pin_bit_mask = 1ULL << CFG_SD_EN_GPIO,
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&cfg);
        gpio_set_level(CFG_SD_EN_GPIO, 0);   /* active-low: 0 = enabled */
        ESP_LOGI(TAG, "EN pin → LOW (card powered)");
    } else {
        gpio_set_direction(CFG_SD_EN_GPIO, GPIO_MODE_INPUT); /* high-Z */
        ESP_LOGI(TAG, "EN pin → HIGH-Z (card off via pull-up)");
    }
}

/* ── public API ──────────────────────────────────────────────────────────── */
esp_err_t sdcard_init(void)
{
    if (s_mounted) {
        ESP_LOGW(TAG, "already mounted");
        return ESP_OK;
    }

    sd_power(true);                   /* always pull EN low before init */
    vTaskDelay(pdMS_TO_TICKS(100));  /* let card power and SPI lines settle */

    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files              = 5,
        .allocation_unit_size   = 16 * 1024,
    };

    sdmmc_host_t host       = SDSPI_HOST_DEFAULT();
    host.slot               = CFG_SPI_HOST;

    sdspi_device_config_t slot_cfg = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_cfg.gpio_cs  = CFG_SD_CS;
    slot_cfg.host_id  = CFG_SPI_HOST;

    esp_err_t ret = esp_vfs_fat_sdspi_mount(CFG_SD_MOUNT, &host,
                                             &slot_cfg, &mount_cfg, &s_card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Mount failed: %s", esp_err_to_name(ret));
        sd_power(false);
        return ret;
    }

    s_mounted = true;
    sdmmc_card_print_info(stdout, s_card);   /* prints card size, speed, etc. */
    ESP_LOGI(TAG, "Mounted at %s  CS=GPIO%d", CFG_SD_MOUNT, CFG_SD_CS);
    return ESP_OK;
}

esp_err_t sdcard_deinit(void)
{
    if (!s_mounted) return ESP_OK;
    esp_vfs_fat_sdcard_unmount(CFG_SD_MOUNT, s_card);
    s_card    = NULL;
    s_mounted = false;
    sd_power(false);
    ESP_LOGI(TAG, "Unmounted");
    return ESP_OK;
}

#define TEST_FILE  CFG_SD_MOUNT "/TEST.TXT"

esp_err_t sdcard_write(const char *msg)
{
    if (!s_mounted) return ESP_ERR_INVALID_STATE;

    static uint32_t seq = 0;
    int64_t uptime_ms = esp_timer_get_time() / 1000;

    FILE *f = fopen(TEST_FILE, "a");
    if (!f) { ESP_LOGE(TAG, "fopen write failed"); return ESP_FAIL; }

    fprintf(f, "[%4"PRIu32"] t=%"PRId64"ms  %s\n", seq++, uptime_ms, msg);
    fclose(f);

    ESP_LOGI(TAG, "wrote line %"PRIu32" to %s", seq - 1, TEST_FILE);
    return ESP_OK;
}

esp_err_t sdcard_read(char *buf, size_t buf_len)
{
    if (!s_mounted) return ESP_ERR_INVALID_STATE;

    FILE *f = fopen(TEST_FILE, "r");
    if (!f) {
        strncpy(buf, "(TEST.TXT not found – use SD WRITE first)", buf_len - 1);
        buf[buf_len - 1] = '\0';
        return ESP_ERR_NOT_FOUND;
    }

    size_t n = fread(buf, 1, buf_len - 1, f);
    buf[n] = '\0';
    fclose(f);

    ESP_LOGI(TAG, "read %d bytes from %s", (int)n, TEST_FILE);
    return ESP_OK;
}

esp_err_t sdcard_wipe(void)
{
    if (!s_mounted) return ESP_ERR_INVALID_STATE;
    if (remove(TEST_FILE) != 0) {
        ESP_LOGW(TAG, "wipe: file not found");
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "wiped %s", TEST_FILE);
    return ESP_OK;
}

void sdcard_set_enable(bool on)
{
    sd_power(on);
}

bool sdcard_is_mounted(void)
{
    return s_mounted;
}

esp_err_t sdcard_list(char *buf, size_t buf_len)
{
    if (!s_mounted) {
        strncpy(buf, "(not mounted)", buf_len - 1);
        buf[buf_len - 1] = '\0';
        return ESP_ERR_INVALID_STATE;
    }

    DIR *dir = opendir(CFG_SD_MOUNT);
    if (!dir) {
        ESP_LOGE(TAG, "opendir failed");
        return ESP_FAIL;
    }

    buf[0] = '\0';
    size_t remaining = buf_len - 1;
    struct dirent *entry;

    while ((entry = readdir(dir)) != NULL && remaining > 2) {
        if (entry->d_name[0] == '.') continue;   /* skip hidden / . .. */
        size_t name_len = strlen(entry->d_name);
        if (name_len + 1 > remaining) break;
        strncat(buf, entry->d_name, remaining);
        strncat(buf, "\n", remaining - name_len);
        remaining -= name_len + 1;
    }
    closedir(dir);

    if (buf[0] == '\0') strncpy(buf, "(empty)", buf_len - 1);
    return ESP_OK;
}
