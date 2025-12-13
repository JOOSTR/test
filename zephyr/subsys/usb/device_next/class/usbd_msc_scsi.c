/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/cache.h>
#include "usbd_msc_scsi.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(usbd_msc, CONFIG_USBD_MSC_LOG_LEVEL);

#define INQUIRY_VERSION_SPC_2	0x04
#define INQUIRY_VERSION_SPC_3	0x05
#define INQUIRY_VERSION_SPC_4	0x06
#define INQUIRY_VERSION_SPC_5	0x07

#define CLAIMED_CONFORMANCE_VERSION INQUIRY_VERSION_SPC_2

#define T10_VENDOR_LENGTH	8
#define T10_PRODUCT_LENGTH	16
#define T10_REVISION_LENGTH	4

#define UNIT_SERIAL_NUMBER "  "

#define SCSI_CMD_STRUCT(opcode) struct scsi_##opcode##_cmd
#define SCSI_CMD_HANDLER(opcode)					\
static int scsi_##opcode(struct scsi_ctx *ctx,				\
			 struct scsi_##opcode##_cmd *cmd,		\
			 uint8_t data_in_buf[static CONFIG_USBD_MSC_SCSI_BUFFER_SIZE])

#define GET_CONTROL_NACA(cmd)		(cmd->control & BIT(2))

enum code_set {
	CODE_SET_BINARY = 0x1,
	CODE_SET_ASCII = 0x2,
	CODE_SET_UTF8 = 0x3,
};

enum scsi_opcode {
	TEST_UNIT_READY = 0x00,
	REQUEST_SENSE = 0x03,
	INQUIRY = 0x12,
	MODE_SENSE_6 = 0x1A,
	START_STOP_UNIT = 0x1B,
	PREVENT_ALLOW_MEDIUM_REMOVAL = 0x1E,
	READ_FORMAT_CAPACITIES = 0x23,
	READ_CAPACITY_10 = 0x25,
	READ_10 = 0x28,
	WRITE_10 = 0x2A,
	MODE_SENSE_10 = 0x5A,
};

SCSI_CMD_STRUCT(TEST_UNIT_READY) {
	uint8_t opcode;
	uint32_t reserved;
	uint8_t control;
} __packed;

#define GET_REQUEST_SENSE_DESC(cmd)	(cmd->desc & BIT(0))

SCSI_CMD_STRUCT(REQUEST_SENSE) {
	uint8_t opcode;
	uint8_t desc;
	uint8_t reserved2;
	uint8_t reserved3;
	uint8_t allocation_length;
	uint8_t control;
} __packed;

#define SENSE_VALID			BIT(7)
#define SENSE_CODE_CURRENT_ERRORS	0x70
#define SENSE_CODE_DEFERRED_ERRORS	0x71

#define SENSE_FILEMARK			BIT(7)
#define SENSE_EOM			BIT(6)
#define SENSE_ILI			BIT(5)
#define SENSE_KEY_MASK			BIT_MASK(4)

#define SENSE_SKSV			BIT(7)

struct scsi_request_sense_response {
	uint8_t valid_code;
	uint8_t obsolete;
	uint8_t filemark_eom_ili_sense_key;
	uint32_t information;
	uint8_t additional_sense_length;
	uint32_t command_specific_information;
	uint16_t additional_sense_with_qualifier;
	uint8_t field_replaceable_unit_code;
	uint8_t sksv;
	uint16_t sense_key_specific;
} __packed;

#define INQUIRY_EVPD		BIT(0)
#define INQUIRY_CMDDT_OBSOLETE	BIT(1)

enum vpd_page_code {
	VPD_SUPPORTED_VPD_PAGES = 0x00,
	VPD_UNIT_SERIAL_NUMBER = 0x80,
	VPD_DEVICE_IDENTIFICATION = 0x83,
};

enum designator_type {
	DESIGNATOR_VENDOR = 0x0,
	DESIGNATOR_T10_VENDOR_ID_BASED = 0x1,
	DESIGNATOR_EUI_64_BASED = 0x2,
	DESIGNATOR_NAA = 0x3,
	DESIGNATOR_RELATIVE_TARGET_PORT_IDENTIFIER = 0x4,
	DESIGNATOR_TARGET_PORT_GROUP = 0x5,
	DESIGNATOR_MD5_LOGICAL_UNIT_IDENTIFIER = 0x6,
	DESIGNATOR_SCSI_NAME_STRING = 0x8,
	DESIGNATOR_PROTOCOL_SPECIFIC_PORT_IDENTIFIER = 0x9,
	DESIGNATOR_UUID_IDENTIFIER = 0xA,
};

SCSI_CMD_STRUCT(INQUIRY) {
	uint8_t opcode;
	uint8_t cmddt_evpd;
	uint8_t page_code;
	uint16_t allocation_length;
	uint8_t control;
} __packed;

struct scsi_inquiry_response {
	uint8_t peripheral;
	uint8_t rmb;
	uint8_t version;
	uint8_t format;
	uint8_t additional_length;
	uint8_t sccs;
	uint8_t encserv;
	uint8_t cmdque;
	char scsi_vendor[T10_VENDOR_LENGTH];
	char product[T10_PRODUCT_LENGTH];
	char revision[T10_REVISION_LENGTH];
} __packed;

#define MODE_SENSE_PAGE_CODE_ALL_PAGES		0x3F

SCSI_CMD_STRUCT(MODE_SENSE_6) {
	uint8_t opcode;
	uint8_t dbd;
	uint8_t page;
	uint8_t subpage;
	uint8_t allocation_length;
	uint8_t control;
} __packed;

struct scsi_mode_sense_6_response {
	uint8_t mode_data_length;
	uint8_t medium_type;
	uint8_t device_specific_parameter;
	uint8_t block_descriptor_length;
} __packed;

#define GET_IMMED(cmd)				(cmd->immed & BIT(0))
#define GET_POWER_CONDITION_MODIFIER(cmd)	(cmd->condition & BIT_MASK(4))
#define GET_POWER_CONDITION(cmd)		((cmd->start & 0xF0) >> 4)
#define GET_NO_FLUSH(cmd)			(cmd->start & BIT(2))
#define GET_LOEJ(cmd)				(cmd->start & BIT(1))
#define GET_START(cmd)				(cmd->start & BIT(0))

enum power_condition {
	POWER_COND_START_VALID = 0x0,
	POWER_COND_ACTIVE = 0x1,
	POWER_COND_IDLE = 0x2,
	POWER_COND_STANDBY = 0x3,
	POWER_COND_LU_CONTROL = 0x7,
	POWER_COND_FORCE_IDLE_0 = 0xA,
	POWER_COND_FORCE_STANDBY_0 = 0xB,
};

SCSI_CMD_STRUCT(START_STOP_UNIT) {
	uint8_t opcode;
	uint8_t immed;
	uint8_t reserved;
	uint8_t condition;
	uint8_t start;
	uint8_t control;
} __packed;

#define GET_PREVENT(cmd)			(cmd->prevent & BIT_MASK(2))

enum prevent_field {
	MEDIUM_REMOVAL_ALLOWED = 0,
	MEDIUM_REMOVAL_SHALL_BE_PREVENTED = 1,
	PREVENT_OBSOLETE_2 = 2,
	PREVENT_OBSOLETE_3 = 3,
};

SCSI_CMD_STRUCT(PREVENT_ALLOW_MEDIUM_REMOVAL) {
	uint8_t opcode;
	uint8_t reserved1;
	uint8_t reserved2;
	uint8_t reserved3;
	uint8_t prevent;
	uint8_t control;
} __packed;

SCSI_CMD_STRUCT(READ_FORMAT_CAPACITIES) {
	uint8_t opcode;
	uint8_t reserved1;
	uint8_t reserved2;
	uint8_t reserved3;
	uint8_t reserved4;
	uint8_t reserved5;
	uint8_t reserved6;
	uint16_t allocation_length;
	uint8_t control;
} __packed;

struct capacity_list_header {
	uint8_t reserved1;
	uint8_t reserved2;
	uint8_t reserved3;
	uint8_t capacity_list_length;
} __packed;

enum descriptor_types {
	UNFORMATTED_OR_BLANK_MEDIA = 1,
	FORMATTED_MEDIA = 2,
	NO_MEDIA_PRESENT_OR_UNKNOWN_CAPACITY = 3,
};

struct current_maximum_capacity_descriptor {
	uint32_t number_of_blocks;
	uint8_t type;
	uint32_t block_length : 24;
} __packed;

struct scsi_read_format_capacities_response {
	struct capacity_list_header header;
	struct current_maximum_capacity_descriptor desc;
} __packed;

SCSI_CMD_STRUCT(READ_CAPACITY_10) {
	uint8_t opcode;
	uint8_t reserved_or_obsolete[8];
	uint8_t control;
} __packed;

struct scsi_read_capacity_10_response {
	uint32_t last_lba;
	uint32_t block_length;
} __packed;

SCSI_CMD_STRUCT(READ_10) {
	uint8_t opcode;
	uint8_t rdprotect;
	uint32_t lba;
	uint8_t group_number;
	uint16_t transfer_length;
	uint8_t control;
} __packed;

SCSI_CMD_STRUCT(WRITE_10) {
	uint8_t opcode;
	uint8_t wrprotect;
	uint32_t lba;
	uint8_t group_number;
	uint16_t transfer_length;
	uint8_t control;
} __packed;

SCSI_CMD_STRUCT(MODE_SENSE_10) {
	uint8_t opcode;
	uint8_t llbaa_dbd;
	uint8_t page;
	uint8_t subpage;
	uint8_t reserved4;
	uint8_t reserved5;
	uint8_t reserved6;
	uint16_t allocation_length;
	uint8_t control;
} __packed;

struct scsi_mode_sense_10_response {
	uint16_t mode_data_length;
	uint8_t medium_type;
	uint8_t device_specific_parameter;
	uint8_t longlba;
	uint8_t reserved5;
	uint16_t block_descriptor_length;
} __packed;

/* Bounce buffer for IDMA compatibility */
extern uint8_t idma_bounce_buffer[4096];

static int update_disk_info(struct scsi_ctx *const ctx)
{
	int status = disk_access_status(ctx->disk);
	LOG_INF("disk_access_status returned: %d", status);

	if (disk_access_ioctl(ctx->disk, DISK_IOCTL_GET_SECTOR_COUNT, &ctx->sector_count) != 0) {
		LOG_ERR("GET_SECTOR_COUNT failed");
		ctx->sector_count = 0;
		status = -EIO;
	}

	if (disk_access_ioctl(ctx->disk, DISK_IOCTL_GET_SECTOR_SIZE, &ctx->sector_size) != 0) {
		LOG_ERR("GET_SECTOR_SIZE failed");
		ctx->sector_size = 0;
		status = -EIO;
	}

	LOG_INF("sector_count=%u, sector_size=%u, buffer_size=%d", 
	        ctx->sector_count, ctx->sector_size, CONFIG_USBD_MSC_SCSI_BUFFER_SIZE);

	if (!ctx->sector_size) {
		LOG_ERR("sector_size is 0");
		status = -EIO;
	} else if ((ctx->sector_size % USBD_MAX_BULK_MPS) &&
		   (USBD_MAX_BULK_MPS % ctx->sector_size)) {
		LOG_ERR("sector_size alignment issue");
		status = -EIO;
	}

	if (ctx->sector_size > CONFIG_USBD_MSC_SCSI_BUFFER_SIZE) {
		LOG_ERR("sector_size > buffer_size!");
		status = -ENOMEM;
	}

	LOG_INF("update_disk_info returning: %d", status);
	return status;
}

static size_t good(struct scsi_ctx *ctx, size_t data_in_bytes)
{
	ctx->status = GOOD;
	ctx->sense_key = NO_SENSE;
	ctx->asc = NO_ADDITIONAL_SENSE_INFORMATION;
	return data_in_bytes;
}

static size_t illegal_request(struct scsi_ctx *ctx, enum scsi_additional_sense_code asc)
{
	ctx->status = CHECK_CONDITION;
	ctx->sense_key = ILLEGAL_REQUEST;
	ctx->asc = asc;
	return 0;
}

static size_t not_ready(struct scsi_ctx *ctx, enum scsi_additional_sense_code asc)
{
	ctx->status = CHECK_CONDITION;
	ctx->sense_key = NOT_READY;
	ctx->asc = asc;
	return 0;
}

static size_t medium_error(struct scsi_ctx *ctx, enum scsi_additional_sense_code asc)
{
	ctx->status = CHECK_CONDITION;
	ctx->sense_key = MEDIUM_ERROR;
	ctx->asc = asc;
	return 0;
}

void scsi_init(struct scsi_ctx *ctx, const char *disk, const char *vendor,
	       const char *product, const char *revision)
{
	memset(ctx, 0, sizeof(struct scsi_ctx));
	ctx->disk = disk;
	ctx->vendor = vendor;
	ctx->product = product;
	ctx->revision = revision;
	scsi_reset(ctx);
}

void scsi_reset(struct scsi_ctx *ctx)
{
	ctx->prevent_removal = false;
	ctx->medium_loaded = true;
}

SCSI_CMD_HANDLER(TEST_UNIT_READY)
{
	if (!ctx->medium_loaded || update_disk_info(ctx) != DISK_STATUS_OK) {
		return not_ready(ctx, MEDIUM_NOT_PRESENT);
	} else {
		return good(ctx, 0);
	}
}

SCSI_CMD_HANDLER(REQUEST_SENSE)
{
	struct scsi_request_sense_response r;
	int length;

	ctx->cmd_is_data_read = true;

	if ((CLAIMED_CONFORMANCE_VERSION >= INQUIRY_VERSION_SPC_4) &&
	    (GET_REQUEST_SENSE_DESC(cmd))) {
		return illegal_request(ctx, INVALID_FIELD_IN_CDB);
	}

	r.valid_code = SENSE_CODE_CURRENT_ERRORS;
	r.obsolete = 0;
	r.filemark_eom_ili_sense_key = ctx->sense_key & SENSE_KEY_MASK;
	r.information = sys_cpu_to_be32(0);
	r.additional_sense_length = sizeof(struct scsi_request_sense_response) - 1 -
		offsetof(struct scsi_request_sense_response, additional_sense_length);
	r.command_specific_information = sys_cpu_to_be32(0);
	r.additional_sense_with_qualifier = sys_cpu_to_be16(ctx->asc);
	r.field_replaceable_unit_code = 0;
	r.sksv = 0;
	r.sense_key_specific = sys_cpu_to_be16(0);

	BUILD_ASSERT(sizeof(r) <= CONFIG_USBD_MSC_SCSI_BUFFER_SIZE);
	length = MIN(cmd->allocation_length, sizeof(r));
	memcpy(data_in_buf, &r, length);

	return good(ctx, length);
}

static int fill_inquiry(struct scsi_ctx *ctx,
			uint8_t buf[static CONFIG_USBD_MSC_SCSI_BUFFER_SIZE])
{
	struct scsi_inquiry_response r;

	memset(&r, 0, sizeof(struct scsi_inquiry_response));

	r.peripheral = 0x00;
	r.rmb = 0x80;
	r.version = CLAIMED_CONFORMANCE_VERSION;
	r.format = 0x02;
	r.additional_length = sizeof(struct scsi_inquiry_response) - 1 -
		offsetof(struct scsi_inquiry_response, additional_length);
	r.sccs = 0x00;
	r.encserv = 0x00;
	r.cmdque = 0x00;

	strncpy(r.scsi_vendor, ctx->vendor, sizeof(r.scsi_vendor));
	strncpy(r.product, ctx->product, sizeof(r.product));
	strncpy(r.revision, ctx->revision, sizeof(r.revision));

	BUILD_ASSERT(sizeof(r) <= CONFIG_USBD_MSC_SCSI_BUFFER_SIZE);
	memcpy(buf, &r, sizeof(r));
	return sizeof(r);
}

static int fill_vpd_page(struct scsi_ctx *ctx, enum vpd_page_code page,
			 uint8_t buf[static CONFIG_USBD_MSC_SCSI_BUFFER_SIZE])
{
	uint16_t offset = 0;
	uint8_t *page_start = &buf[4];

	switch (page) {
	case VPD_SUPPORTED_VPD_PAGES:
		page_start[offset++] = VPD_SUPPORTED_VPD_PAGES;
		page_start[offset++] = VPD_UNIT_SERIAL_NUMBER;
		page_start[offset++] = VPD_DEVICE_IDENTIFICATION;
		break;
	case VPD_DEVICE_IDENTIFICATION:
		page_start[offset++] = CODE_SET_ASCII;
		page_start[offset++] = DESIGNATOR_T10_VENDOR_ID_BASED;
		page_start[offset++] = 0x00;
		page_start[offset++] = T10_VENDOR_LENGTH + sizeof(UNIT_SERIAL_NUMBER) - 1;
		strncpy(&page_start[offset], ctx->vendor, T10_VENDOR_LENGTH);
		offset += T10_VENDOR_LENGTH;
		memcpy(&page_start[offset], UNIT_SERIAL_NUMBER, sizeof(UNIT_SERIAL_NUMBER) - 1);
		offset += sizeof(UNIT_SERIAL_NUMBER) - 1;
		break;
	case VPD_UNIT_SERIAL_NUMBER:
		memcpy(page_start, UNIT_SERIAL_NUMBER, sizeof(UNIT_SERIAL_NUMBER) - 1);
		offset += sizeof(UNIT_SERIAL_NUMBER) - 1;
		break;
	default:
		return -ENOTSUP;
	}

	buf[0] = 0x00;
	buf[1] = page;
	sys_put_be16(offset, &buf[2]);
	return offset + 4;
}

SCSI_CMD_HANDLER(INQUIRY)
{
	int ret;

	ctx->cmd_is_data_read = true;

	if (cmd->cmddt_evpd & INQUIRY_CMDDT_OBSOLETE) {
		ret = -EINVAL;
	} else if (cmd->cmddt_evpd & INQUIRY_EVPD) {
		ret = MIN(sys_be16_to_cpu(cmd->allocation_length),
			  fill_vpd_page(ctx, cmd->page_code, data_in_buf));
	} else if (cmd->page_code != 0) {
		LOG_WRN("Page Code is %d but EVPD set", cmd->page_code);
		ret = -EINVAL;
	} else {
		ret = MIN(sys_be16_to_cpu(cmd->allocation_length),
			  fill_inquiry(ctx, data_in_buf));
	}

	if (ret < 0) {
		return illegal_request(ctx, INVALID_FIELD_IN_CDB);
	}
	return good(ctx, ret);
}

SCSI_CMD_HANDLER(MODE_SENSE_6)
{
	struct scsi_mode_sense_6_response r;
	int length;

	ctx->cmd_is_data_read = true;

	if (cmd->page != MODE_SENSE_PAGE_CODE_ALL_PAGES || cmd->subpage != 0) {
		return illegal_request(ctx, INVALID_FIELD_IN_CDB);
	}

	r.mode_data_length = 3;
	r.medium_type = 0x00;
	r.device_specific_parameter = 0x00;
	r.block_descriptor_length = 0x00;

	BUILD_ASSERT(sizeof(r) <= CONFIG_USBD_MSC_SCSI_BUFFER_SIZE);
	length = MIN(cmd->allocation_length, sizeof(r));
	memcpy(data_in_buf, &r, length);
	return good(ctx, length);
}

SCSI_CMD_HANDLER(START_STOP_UNIT)
{
	bool medium_loaded = ctx->medium_loaded;

	if (GET_POWER_CONDITION(cmd) == POWER_COND_START_VALID) {
		if (GET_LOEJ(cmd)) {
			if (GET_START(cmd)) {
				medium_loaded = true;
			} else {
				medium_loaded = false;
			}
		}
	}

	if (!medium_loaded && ctx->medium_loaded && ctx->prevent_removal) {
		return illegal_request(ctx, MEDIUM_REMOVAL_PREVENTED);
	}

	ctx->medium_loaded = medium_loaded;
	return good(ctx, 0);
}

SCSI_CMD_HANDLER(PREVENT_ALLOW_MEDIUM_REMOVAL)
{
	switch (GET_PREVENT(cmd)) {
	case MEDIUM_REMOVAL_ALLOWED:
		ctx->prevent_removal = false;
		break;
	case MEDIUM_REMOVAL_SHALL_BE_PREVENTED:
		ctx->prevent_removal = true;
		break;
	case PREVENT_OBSOLETE_2:
	case PREVENT_OBSOLETE_3:
		break;
	}

	return good(ctx, 0);
}

SCSI_CMD_HANDLER(READ_FORMAT_CAPACITIES)
{
	struct scsi_read_format_capacities_response r;
	int length;

	ctx->cmd_is_data_read = true;

	memset(&r, 0, sizeof(r));
	r.header.capacity_list_length = sizeof(r) - sizeof(r.header);

	if (update_disk_info(ctx) < 0) {
		r.desc.number_of_blocks = sys_cpu_to_be32(UINT32_MAX);
		r.desc.type = NO_MEDIA_PRESENT_OR_UNKNOWN_CAPACITY;
	} else {
		r.desc.number_of_blocks = sys_cpu_to_be32(ctx->sector_count);
		r.desc.type = FORMATTED_MEDIA;
	}
	r.desc.block_length = sys_cpu_to_be32(ctx->sector_size);

	ctx->cmd_is_data_read = true;

	BUILD_ASSERT(sizeof(r) <= CONFIG_USBD_MSC_SCSI_BUFFER_SIZE);
	length = MIN(sys_be16_to_cpu(cmd->allocation_length), sizeof(r));
	memcpy(data_in_buf, &r, length);
	return good(ctx, length);
}

SCSI_CMD_HANDLER(READ_CAPACITY_10)
{
	struct scsi_read_capacity_10_response r;

	ctx->cmd_is_data_read = true;

	if (!ctx->medium_loaded || update_disk_info(ctx) != DISK_STATUS_OK) {
		return not_ready(ctx, MEDIUM_NOT_PRESENT);
	}

	r.last_lba = sys_cpu_to_be32(ctx->sector_count ? ctx->sector_count - 1 : 0);
	r.block_length = sys_cpu_to_be32(ctx->sector_size);

	ctx->cmd_is_data_read = true;

	BUILD_ASSERT(sizeof(r) <= CONFIG_USBD_MSC_SCSI_BUFFER_SIZE);
	memcpy(data_in_buf, &r, sizeof(r));
	return good(ctx, sizeof(r));
}

static int validate_transfer_length(struct scsi_ctx *ctx, uint32_t lba, uint16_t length)
{
	uint32_t last_lba = lba + length - 1;

	if (lba >= ctx->sector_count) {
		LOG_WRN("LBA %d is out of range", lba);
		return -EINVAL;
	}

	if (length == 0) {
		return 0;
	}

	if ((last_lba >= ctx->sector_count) || (last_lba < lba)) {
		LOG_WRN("%d blocks starting at %d go out of bounds", length, lba);
		return -EINVAL;
	}

	return 0;
}

static size_t fill_read_10(struct scsi_ctx *ctx, uint8_t *buf, size_t length)
{
	uint32_t sectors;
	uint32_t chunk_sectors;
	size_t total_read = 0;

	sectors = MIN(length, ctx->remaining_data) / ctx->sector_size;

	while (sectors > 0) {
		chunk_sectors = MIN(sectors, sizeof(idma_bounce_buffer) / ctx->sector_size);

		int ret = disk_access_read(ctx->disk, idma_bounce_buffer, ctx->lba, chunk_sectors);
		if (ret != 0) {
			LOG_ERR("disk_access_read failed: %d", ret);
			return total_read;
		}

		/* Invalidate cache - DMA wrote to memory, CPU cache is stale */
		sys_cache_data_invd_range(idma_bounce_buffer, chunk_sectors * ctx->sector_size);

		memcpy(buf, idma_bounce_buffer, chunk_sectors * ctx->sector_size);

		if (ctx->lba == 0 && total_read == 0) {
			LOG_INF("MBR: %02x %02x ... %02x %02x",
				buf[0], buf[1], buf[510], buf[511]);
		}

		buf += chunk_sectors * ctx->sector_size;
		ctx->lba += chunk_sectors;
		sectors -= chunk_sectors;
		total_read += chunk_sectors * ctx->sector_size;
	}

	return total_read;
}

SCSI_CMD_HANDLER(READ_10)
{
	uint32_t lba = sys_be32_to_cpu(cmd->lba);
	uint16_t transfer_length = sys_be16_to_cpu(cmd->transfer_length);

	ctx->cmd_is_data_read = true;

	if (!ctx->medium_loaded || update_disk_info(ctx) != DISK_STATUS_OK) {
		return not_ready(ctx, MEDIUM_NOT_PRESENT);
	}

	if (validate_transfer_length(ctx, lba, transfer_length)) {
		return illegal_request(ctx, LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE);
	}

	ctx->read_cb = fill_read_10;
	ctx->lba = lba;
	ctx->remaining_data = ctx->sector_size * transfer_length;

	return good(ctx, 0);
}

static size_t store_write_10(struct scsi_ctx *ctx, const uint8_t *buf, size_t length)
{
	uint32_t sectors;
	uint32_t chunk_sectors;
	size_t total_written = 0;
	uint32_t initial_remaining;

	initial_remaining = ctx->remaining_data;
	sectors = MIN(length, ctx->remaining_data) / ctx->sector_size;

	while (sectors > 0) {
		chunk_sectors = MIN(sectors, sizeof(idma_bounce_buffer) / ctx->sector_size);

		/* Copy to bounce buffer */
		memcpy(idma_bounce_buffer, buf, chunk_sectors * ctx->sector_size);

		/* Flush cache before DMA - CPU wrote to memory, DMA needs to see it */
		sys_cache_data_flush_range(idma_bounce_buffer, chunk_sectors * ctx->sector_size);

		int ret = disk_access_write(ctx->disk, idma_bounce_buffer, ctx->lba, chunk_sectors);
		if (ret != 0) {
			LOG_ERR("disk_access_write failed: %d", ret);
			if (total_written == 0) {
				return medium_error(ctx, WRITE_ERROR);
			}
			break;
		}

		buf += chunk_sectors * ctx->sector_size;
		ctx->lba += chunk_sectors;
		sectors -= chunk_sectors;
		total_written += chunk_sectors * ctx->sector_size;
	}

	/* Flush disk cache if this is the last sector in transfer */
	if (initial_remaining - total_written == 0 || 
	    ctx->remaining_data - total_written == 0) {
		disk_access_ioctl(ctx->disk, DISK_IOCTL_CTRL_SYNC, NULL);
	}

	return total_written;
}

SCSI_CMD_HANDLER(WRITE_10)
{
	uint32_t lba = sys_be32_to_cpu(cmd->lba);
	uint16_t transfer_length = sys_be16_to_cpu(cmd->transfer_length);

	ctx->cmd_is_data_write = true;

	if (!ctx->medium_loaded || update_disk_info(ctx) != DISK_STATUS_OK) {
		return not_ready(ctx, MEDIUM_NOT_PRESENT);
	}

	if (validate_transfer_length(ctx, lba, transfer_length)) {
		return illegal_request(ctx, LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE);
	}

	ctx->write_cb = store_write_10;
	ctx->lba = lba;
	ctx->remaining_data = ctx->sector_size * transfer_length;

	return good(ctx, 0);
}

SCSI_CMD_HANDLER(MODE_SENSE_10)
{
	struct scsi_mode_sense_10_response r;
	int length;

	ctx->cmd_is_data_read = true;

	if (cmd->page != MODE_SENSE_PAGE_CODE_ALL_PAGES || cmd->subpage != 0) {
		return illegal_request(ctx, INVALID_FIELD_IN_CDB);
	}

	r.mode_data_length = sys_cpu_to_be16(6);
	r.medium_type = 0x00;
	r.device_specific_parameter = 0x00;
	r.longlba = 0x00;
	r.reserved5 = 0x00;
	r.block_descriptor_length = sys_cpu_to_be16(0);

	BUILD_ASSERT(sizeof(r) <= CONFIG_USBD_MSC_SCSI_BUFFER_SIZE);
	length = MIN(sys_be16_to_cpu(cmd->allocation_length), sizeof(r));
	memcpy(data_in_buf, &r, length);

	return good(ctx, length);
}

int scsi_usb_boot_cmd_len(const uint8_t *cb, int len)
{
	if (len != 12) {
		return len;
	}

	switch (cb[0]) {
	case TEST_UNIT_READY:	return sizeof(SCSI_CMD_STRUCT(TEST_UNIT_READY));
	case REQUEST_SENSE:	return sizeof(SCSI_CMD_STRUCT(REQUEST_SENSE));
	case INQUIRY:		return sizeof(SCSI_CMD_STRUCT(INQUIRY));
	case READ_CAPACITY_10:	return sizeof(SCSI_CMD_STRUCT(READ_CAPACITY_10));
	case READ_10:		return sizeof(SCSI_CMD_STRUCT(READ_10));
	case WRITE_10:		return sizeof(SCSI_CMD_STRUCT(WRITE_10));
	case MODE_SENSE_10:	return sizeof(SCSI_CMD_STRUCT(MODE_SENSE_10));
	default:		return len;
	}
}

size_t scsi_cmd(struct scsi_ctx *ctx, const uint8_t *cb, int len,
		uint8_t data_in_buf[static CONFIG_USBD_MSC_SCSI_BUFFER_SIZE])
{
	ctx->cmd_is_data_read = false;
	ctx->cmd_is_data_write = false;
	ctx->remaining_data = 0;
	ctx->read_cb = NULL;
	ctx->write_cb = NULL;

#define SCSI_CMD(opcode) do {							\
	if (len == sizeof(SCSI_CMD_STRUCT(opcode)) && cb[0] == opcode) {	\
		LOG_DBG("SCSI " #opcode);					\
		if (GET_CONTROL_NACA(((SCSI_CMD_STRUCT(opcode)*)cb))) {		\
			return illegal_request(ctx, INVALID_FIELD_IN_CDB);	\
		}								\
		return scsi_##opcode(ctx, (SCSI_CMD_STRUCT(opcode)*)cb,		\
				     data_in_buf);				\
	}									\
} while (0)

	SCSI_CMD(TEST_UNIT_READY);
	SCSI_CMD(REQUEST_SENSE);
	SCSI_CMD(INQUIRY);
	SCSI_CMD(MODE_SENSE_6);
	SCSI_CMD(START_STOP_UNIT);
	SCSI_CMD(PREVENT_ALLOW_MEDIUM_REMOVAL);
	SCSI_CMD(READ_FORMAT_CAPACITIES);
	SCSI_CMD(READ_CAPACITY_10);
	SCSI_CMD(READ_10);
	SCSI_CMD(WRITE_10);
	SCSI_CMD(MODE_SENSE_10);

	LOG_ERR("Unknown SCSI opcode 0x%02x", cb[0]);
	return illegal_request(ctx, INVALID_FIELD_IN_CDB);
}

bool scsi_cmd_is_data_read(struct scsi_ctx *ctx)
{
	return ctx->cmd_is_data_read;
}

bool scsi_cmd_is_data_write(struct scsi_ctx *ctx)
{
	return ctx->cmd_is_data_write;
}

size_t scsi_cmd_remaining_data_len(struct scsi_ctx *ctx)
{
	return ctx->remaining_data;
}

size_t scsi_read_data(struct scsi_ctx *ctx, uint8_t *buf, size_t length)
{
	size_t retrieved = 0;

	__ASSERT_NO_MSG(ctx->cmd_is_data_read);

	if ((ctx->remaining_data > 0) && ctx->read_cb) {
		retrieved = ctx->read_cb(ctx, buf, length);
	}
	ctx->remaining_data -= retrieved;
	if (retrieved == 0) {
		ctx->remaining_data = 0;
	}
	return retrieved;
}

size_t scsi_write_data(struct scsi_ctx *ctx, const uint8_t *buf, size_t length)
{
	size_t processed = 0;

	__ASSERT_NO_MSG(ctx->cmd_is_data_write);

	length = MIN(length, ctx->remaining_data);
	if ((length > 0) && ctx->write_cb) {
		processed = ctx->write_cb(ctx, buf, length);
	}
	ctx->remaining_data -= processed;
	if (processed == 0) {
		ctx->remaining_data = 0;
	}
	return processed;
}

enum scsi_status_code scsi_cmd_get_status(struct scsi_ctx *ctx)
{
	return ctx->status;
}