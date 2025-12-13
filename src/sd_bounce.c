#include <zephyr/kernel.h>
#include <zephyr/storage/disk_access.h>
#include <string.h>

extern uint8_t idma_bounce_buffer[4096];

int sd_bounce_read(const char *disk, uint8_t *dst, uint32_t sector, uint32_t count)
{
    int ret;
    uint32_t sector_size = 512;
    
    while (count > 0) {
        uint32_t chunk = MIN(count, sizeof(idma_bounce_buffer) / sector_size);
        
        ret = disk_access_read(disk, idma_bounce_buffer, sector, chunk);
        if (ret != 0) return ret;
        
        memcpy(dst, idma_bounce_buffer, chunk * sector_size);
        
        dst += chunk * sector_size;
        sector += chunk;
        count -= chunk;
    }
    return 0;
}

int sd_bounce_write(const char *disk, const uint8_t *src, uint32_t sector, uint32_t count)
{
    int ret;
    uint32_t sector_size = 512;
    
    while (count > 0) {
        uint32_t chunk = MIN(count, sizeof(idma_bounce_buffer) / sector_size);
        
        memcpy(idma_bounce_buffer, src, chunk * sector_size);
        
        ret = disk_access_write(disk, idma_bounce_buffer, sector, chunk);
        if (ret != 0) return ret;
        
        src += chunk * sector_size;
        sector += chunk;
        count -= chunk;
    }
    return 0;
}