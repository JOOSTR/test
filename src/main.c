#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_msc.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/init.h>
extern uint8_t idma_bounce_buffer[4096];

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#include <zephyr/cache.h>

static int early_disk_init(void)
{
    int err = disk_access_init("SD");
    if (err) {
        LOG_ERR("Early SD init failed: %d", err);
        return err;
    }
    LOG_INF("Early SD init OK");

    extern uint8_t idma_bounce_buffer[4096];
    
    /* Fill with known pattern first */
    memset(idma_bounce_buffer, 0xAA, 512);
    
    int ret = disk_access_read("SD", idma_bounce_buffer, 0, 1);
    LOG_INF("disk_access_read returned: %d", ret);
    
    sys_cache_data_invd_range(idma_bounce_buffer, 512);
    
    /* Check if buffer changed at all */
    bool all_aa = true;
    bool all_zero = true;
    for (int i = 0; i < 512; i++) {
        if (idma_bounce_buffer[i] != 0xAA) all_aa = false;
        if (idma_bounce_buffer[i] != 0x00) all_zero = false;
    }
    LOG_INF("Buffer: all_0xAA=%d, all_zero=%d", all_aa, all_zero);
    
    LOG_HEXDUMP_INF(idma_bounce_buffer, 64, "Sector 0 first 64 bytes:");

    return 0;
}

SYS_INIT(early_disk_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

USBD_DEVICE_DEFINE(my_usbd, DEVICE_DT_GET(DT_NODELABEL(usbotg_hs)),
                   0x2fe3, 0x0001);

USBD_DESC_LANG_DEFINE(my_lang);
USBD_DESC_MANUFACTURER_DEFINE(my_mfr, "Joost");
USBD_DESC_PRODUCT_DEFINE(my_product, "SD Card");
USBD_DESC_SERIAL_NUMBER_DEFINE(my_sn);

USBD_CONFIGURATION_DEFINE(my_fs_config, USB_SCD_SELF_POWERED, 200, NULL);
USBD_CONFIGURATION_DEFINE(my_hs_config, USB_SCD_SELF_POWERED, 200, NULL);

USBD_DEFINE_MSC_LUN(sd_lun, "SD", "Joost   ", "SD Card        ", "1.0");

int main(void)
{
    int err;
    int status;
    printk("BOUNCE BUFFER @ %p\n", (void*)idma_bounce_buffer);
    // ... rest of main
    LOG_INF("USB MSC SD starting");

    status = disk_access_status("SD");
    LOG_INF("Disk status in main: %d (0=OK, 1=UNINIT, 2=NOMEDIA)", status);

    usbd_add_descriptor(&my_usbd, &my_lang);
    usbd_add_descriptor(&my_usbd, &my_mfr);
    usbd_add_descriptor(&my_usbd, &my_product);
    usbd_add_descriptor(&my_usbd, &my_sn);

    usbd_add_configuration(&my_usbd, USBD_SPEED_FS, &my_fs_config);
    usbd_add_configuration(&my_usbd, USBD_SPEED_HS, &my_hs_config);

    usbd_register_all_classes(&my_usbd, USBD_SPEED_FS, 1, NULL);
    usbd_register_all_classes(&my_usbd, USBD_SPEED_HS, 1, NULL);

    err = usbd_init(&my_usbd);
    if (err) {
        LOG_ERR("USB init failed: %d", err);
        return err;
    }

    err = usbd_enable(&my_usbd);
    if (err) {
        LOG_ERR("USB enable failed: %d", err);
        return err;
    }

    LOG_INF("USB MSC ready");

    while (1) {
        k_sleep(K_SECONDS(5));
        status = disk_access_status("SD");
        LOG_INF("Disk status: %d", status);
    }

    return 0;
}