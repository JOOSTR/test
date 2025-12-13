c/* bounce_disk.c - Wrapper to handle IDMA buffer requirements */

#include <zephyr/kernel.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(bounce_disk, LOG_LEVEL_INF);

/* 4KB bounce buffer in SRAM1 */
__attribute__((section(".sram1_data")))
__aligned(32)
static uint8_t idma_bounce[4096];

#define SECTOR_SIZE 512
#define BOUNCE_SECTORS (sizeof(idma_bounce) / SECTOR_SIZE)

static bool addr_needs_bounce(const void *addr)
{
    uintptr_t a = (uintptr_t)addr;
    /* SRAM1/2 range on STM32U5 - IDMA can access this */
    if (a >= 0x28000000 && a < 0x28100000) {
        return false;
    }
    /* Everything else needs bounce buffer */
    return true;
}

int bounce_disk_read(const char *name, uint8_t *buf, uint32_t sector, uint32_t count)
{
    int ret;
    
    if (!addr_needs_bounce(buf)) {
        return disk_access_read(name, buf, sector, count);
    }
    
    LOG_DBG("Bounce read: sector %u, count %u", sector, count);
    
    while (count > 0) {
        uint32_t chunk = MIN(count, BOUNCE_SECTORS);
        
        ret = disk_access_read(name, idma_bounce, sector, chunk);
        if (ret != 0) {
            return ret;
        }
        
        memcpy(buf, idma_bounce, chunk * SECTOR_SIZE);
        
        buf += chunk * SECTOR_SIZE;
        sector += chunk;
        count -= chunk;
    }
    
    return 0;
}

int bounce_disk_write(const char *name, const uint8_t *buf, uint32_t sector, uint32_t count)
{
    int ret;
    
    if (!addr_needs_bounce(buf)) {
        return disk_access_write(name, buf, sector, count);
    }
    
    LOG_DBG("Bounce write: sector %u, count %u", sector, count);
    
    while (count > 0) {
        uint32_t chunk = MIN(count, BOUNCE_SECTORS);
        
        memcpy(idma_bounce, buf, chunk * SECTOR_SIZE);
        
        ret = disk_access_write(name, idma_bounce, sector, chunk);
        if (ret != 0) {
            return ret;
        }
        
        buf += chunk * SECTOR_SIZE;
        sector += chunk;
        count -= chunk;
    }
    
    return 0;
}