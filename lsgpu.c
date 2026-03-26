#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>

// NVIDIA HEADERS START
#include "nvtypes.h"
#include "smbpbi.h"
// NVIDIA HEADERS END

#define DEF_BUF_SIZE 32
#define DEF_COPY_SZ 4

#define DWORD_SZ 4

#define NVIDIA_FWVERSION_LEN 16

#define VENDOR_ID_LEN 2
#define NVIDIA_VENDOR_ID0 0x10
#define NVIDIA_VENDOR_ID1 0xDE

#define MAX_GPUS 8

#define SMBPBI_ADDR 0x4f
#define SMBPBI_SET_REG 0x5c
#define SMBPBI_GET_REG 0x5d
#define SMBPBI_GET_LEN 5

#define STATUS_GOOD 0x80

// TYPE SIZE LENGTHS
#define SIZE_24I8F 4

struct GpuInfo
{
	uint8_t fwversion[NV_MSGBOX_SYSID_DATA_SIZE_FIRMWARE_VER_V1];
	uint8_t temp[SIZE_24I8F];
	uint8_t power[DWORD_SZ];
	uint8_t board_part_number[NV_MSGBOX_SYSID_DATA_SIZE_BOARD_PART_NUM_V1];
	uint8_t serial_number[NV_MSGBOX_SYSID_DATA_SIZE_SERIAL_NUM_V1];
	uint8_t marketing_name[NV_MSGBOX_SYSID_DATA_SIZE_MARKETING_NAME_V1];
	uint8_t guid[NV_MSGBOX_SYSID_DATA_SIZE_GPU_GUID_V1];
	uint8_t dev_id[NV_MSGBOX_SYSID_DATA_SIZE_DEV_ID_V1];
	uint8_t pcie_speed[NV_MSGBOX_SYSID_DATA_SIZE_PCIE_SPEED_V1];
	uint8_t build_date[NV_MSGBOX_SYSID_DATA_SIZE_BUILD_DATE_V1];
	uint8_t sub_vendor_id[NV_MSGBOX_SYSID_DATA_SIZE_SUB_VENDOR_ID_V1];
	uint8_t sub_id[NV_MSGBOX_SYSID_DATA_SIZE_SUB_ID_V1];
	uint8_t vendor_id[NV_MSGBOX_SYSID_DATA_SIZE_VENDOR_ID_V1];
	uint8_t inforom_version[NV_MSGBOX_SYSID_DATA_SIZE_INFOROM_VER_V1];
	uint8_t pcie_width[NV_MSGBOX_SYSID_DATA_SIZE_PCIE_WIDTH_V1];
	uint8_t tgp_limit[NV_MSGBOX_SYSID_DATA_SIZE_TGP_LIMIT_V1];
	uint8_t fru_part_number[NV_MSGBOX_SYSID_DATA_SIZE_FRU_PART_NUMBER_V1];
	uint8_t gpu_part_number[NV_MSGBOX_SYSID_DATA_SIZE_GPU_PART_NUM_V1];
	uint8_t max_dram_capacity[NV_MSGBOX_SYSID_DATA_SIZE_MAX_DRAM_CAPACITY_V1];
	uint8_t pcie_link_info_0[DWORD_SZ];
	uint8_t pcie_link_info_1[DWORD_SZ];
	uint8_t pcie_link_info_2[DWORD_SZ];
	uint8_t pcie_link_info_3[DWORD_SZ];
	uint8_t pcie_link_info_4[DWORD_SZ];
	uint8_t pcie_link_info_6[DWORD_SZ];
	uint8_t pcie_link_info_8[DWORD_SZ];
	uint8_t pcie_link_info_9[DWORD_SZ];
};

struct GpuInfo gpu_info[MAX_GPUS];

double convert_24i8f_to_double(uint32_t val)
{
	int i = 0;
	double d = 0;
	unsigned char* ptr = (unsigned char*) &val;	
	memcpy(&i, ptr + 1, 3);
	d = (double) ptr[0] / 256;
	
	return i + d;
}

int i2c_rd(int fd, uint8_t reg, uint8_t *value);

int i2c_wr(const char* dev, uint8_t addr, uint8_t reg, uint8_t *value, uint8_t write_len)
{
	int fd = open(dev, O_RDWR);
	if (fd == -1)
	{
		fprintf(stderr, "Failed to open %s\n", dev);
		return -1;
	}

	struct i2c_msg msg;
	struct i2c_rdwr_ioctl_data wr_data;

	uint8_t write_buf[write_len + 2]; // reg + len + data
	write_buf[0] = reg;
	write_buf[1] = write_len;
	memcpy(write_buf + 2, value, write_len);

	msg.addr = addr;
	msg.flags = 0;
	msg.len = write_len + 2;
	msg.buf = write_buf;

	wr_data.msgs = &msg;
	wr_data.nmsgs = 1;

	int ret = ioctl(fd, I2C_RDWR, &wr_data);
	if (ret < 0)
	{
		return -1;
	}
	return 0;
}

int i2c_wr_rd(const char* dev, uint8_t addr, uint8_t reg, uint8_t *value, uint8_t read_len)
{
	int fd = open(dev, O_RDWR);
	if (fd == -1)
	{
		fprintf(stderr, "Failed to open %s\n", dev);
		return -1;
	}

	struct i2c_msg msgs[2];
	struct i2c_rdwr_ioctl_data rdwr_data;
    
	uint8_t reg_buf = reg;
	msgs[0].addr = addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &reg_buf;
    
	if (value != NULL && read_len > 128)
	{
		return -1;	
	}

	uint8_t read_buf[read_len];
	memset(read_buf, 0, sizeof(read_buf));

	msgs[1].addr = addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = read_len;
	msgs[1].buf = read_buf;

	rdwr_data.msgs = msgs;
	rdwr_data.nmsgs = 2;

	int ret = ioctl(fd, I2C_RDWR, &rdwr_data);
	if (ret < 0)
	{
		return -1;
	}

	memcpy(value, read_buf, sizeof(read_buf));

	return 0;
}

// START + addr + STOP
int i2c_quick_command(const char* dev, uint8_t addr)
{
	int fd = open(dev, O_RDWR);
	if (fd == -1)
	{
		fprintf(stderr, "Failed to open %s\n", dev);
		return -1;
	}

	struct i2c_msg msg;
	struct i2c_rdwr_ioctl_data wr_data;

	msg.addr = addr;
	msg.flags = 0;
	msg.len = 0;
	msg.buf = NULL;

	wr_data.msgs = &msg;
	wr_data.nmsgs = 1;

	int ret = ioctl(fd, I2C_RDWR, &wr_data);
	if (ret < 0)
	{
		return -1;
	}

	return 0;
}

int get_smbpbi_data(const char* dev, uint8_t opcode, uint8_t arg1, uint8_t arg2, uint8_t status, uint8_t* buf, int count);

uint64_t get_available_i2c_busses()
{
	uint64_t mask = 0;

	const char* prefix = "i2c-";
	const int prefix_len = strlen(prefix);
	DIR* dir;
	struct dirent* entry;
	dir = opendir("/dev");
	if(dir == NULL) 
	{
		fprintf(stderr, "Failed to open /dev dir\n");
		return 0;
	}

	while ((entry = readdir(dir)) != NULL)
	{
		if (strncmp(entry->d_name, prefix, prefix_len) == 0) 
		{
			int index = 0;
			sscanf(entry->d_name, "i2c-%d", &index);
			mask |= (1ULL << index);
		}			
	}
	
	closedir(dir);
	
	return mask;
}

void print_busses(const char* fmt, uint64_t mask)
{
	for (int i = 0; i < sizeof(mask) * 8; i++) 
	{
		if ((1ULL << i) & mask)
		{
			printf(fmt, i);
		}
	}
}

uint64_t detect_nvidia_device()
{
	uint64_t mask = get_available_i2c_busses();

	uint64_t nvidia_mask = 0;

	// bus shall contains 0x50 and 0x4f devices
	const uint8_t addrs[] = {0x50, 0x4f};
	char bus[32] = {};

	for (int i = 0; i < sizeof(mask) * 8; i++) 
	{
		if (!((1ULL << i) & mask)) 
		{
			continue;
		}
	
		snprintf(bus, 32, "/dev/i2c-%d", i);
	
		if (i2c_quick_command(bus, addrs[0]) != 0)
			continue;
		if (i2c_quick_command(bus, addrs[1]) != 0)
			continue;

		
		uint8_t recv_buf[] = {0x0, 0x0, 0x0, 0x0, 0x0};
		if (get_smbpbi_data(bus, NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA, 
					 NV_MSGBOX_CMD_ARG1_VENDOR_ID_V1, NV_MSGBOX_CMD_ARG2_NULL, 
					 STATUS_GOOD, recv_buf, SMBPBI_GET_LEN) == -1)
			continue;
		if (recv_buf[1] == NVIDIA_VENDOR_ID1 && recv_buf[2] == NVIDIA_VENDOR_ID0) // little endian
			nvidia_mask |= (1ULL << i);

	}
	return nvidia_mask;
}

int get_smbpbi_data(const char* dev, uint8_t opcode, uint8_t arg1, uint8_t arg2, uint8_t status, uint8_t* buf, int count)
{
	uint8_t send_buf[] = {opcode, arg1, arg2, status};
	uint8_t send_size = 4;
	uint8_t recv_buf[] = {0x0, 0x0, 0x0, 0x0, 0x0};
	uint8_t recv_size = count;

	if (i2c_wr(dev, SMBPBI_ADDR, SMBPBI_SET_REG, send_buf, send_size) == -1)
		return -1;

	usleep(400);

	if (i2c_wr_rd(dev, SMBPBI_ADDR, SMBPBI_GET_REG, recv_buf, recv_size) == -1)
		return -1;

	usleep(400);

	memcpy(buf, recv_buf, count);
	return 0;
}

uint8_t* get_ptr_to_field(uint8_t cmd, uint8_t arg1, int gpu_info_index, int field_index);

int smbpbi_cmd(uint64_t mask, uint8_t opcode, uint8_t arg1, uint8_t type_size)
{
	char bus[32] = {};
	uint8_t recv_buf[] = {0x0, 0x0, 0x0, 0x0, 0x0};

	for (int i = 0, k = 0; i < sizeof(mask) * 8; i++) 
	{
		if ((1ULL << i) & mask)
		{
			snprintf(bus, 32, "/dev/i2c-%d", i);
			for (int j = 0, arg2 = 0; j < type_size; j += DEF_COPY_SZ, arg2++) 
			{
				if (get_smbpbi_data(bus, opcode,
						     arg1, arg2,
						     STATUS_GOOD, recv_buf, SMBPBI_GET_LEN) == -1)
					return -1;

				uint8_t* to = get_ptr_to_field(opcode, arg1, k, j);

				uint8_t is_last_transaction = ((j + DEF_COPY_SZ) > type_size);
				if (is_last_transaction)
				{
					memcpy(to, 
					       &recv_buf[1], 
					       type_size - j);
					break;
				}

				memcpy(to, &recv_buf[1], DEF_COPY_SZ);
			}

			k++;
		}
	}	
}

uint8_t* get_ptr_to_field(uint8_t cmd, uint8_t arg1, int gpu_info_index, int field_index)
{
	switch (cmd)
	{
		// SYS_INFO
		case NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA:
			switch (arg1) 
			{
				case NV_MSGBOX_CMD_ARG1_FIRMWARE_VER_V1:
					return &(gpu_info[gpu_info_index].fwversion[field_index]);
					break;

				case NV_MSGBOX_CMD_ARG1_BOARD_PART_NUM_V1:
					return &(gpu_info[gpu_info_index].board_part_number[field_index]);
					break;

				case NV_MSGBOX_CMD_ARG1_SERIAL_NUM_V1:
					return &(gpu_info[gpu_info_index].serial_number[field_index]);
					break;

				case NV_MSGBOX_CMD_ARG1_MARKETING_NAME_V1:
					return &(gpu_info[gpu_info_index].marketing_name[field_index]);
					break;

				case NV_MSGBOX_CMD_ARG1_GPU_GUID_V1:
					return &(gpu_info[gpu_info_index].guid[field_index]);
					break;

				case NV_MSGBOX_CMD_ARG1_DEV_ID_V1:
					return &(gpu_info[gpu_info_index].dev_id[field_index]);
					break;

				case NV_MSGBOX_CMD_ARG1_PCIE_SPEED_V1:
					return &(gpu_info[gpu_info_index].pcie_speed[field_index]);
					break;

				case NV_MSGBOX_CMD_ARG1_BUILD_DATE_V1:
					return &(gpu_info[gpu_info_index].build_date[field_index]);
					break;

				case NV_MSGBOX_CMD_ARG1_SUB_VENDOR_ID_V1:
					return &(gpu_info[gpu_info_index].sub_vendor_id[field_index]);
					break;

				case NV_MSGBOX_CMD_ARG1_SUB_ID_V1:
					return &(gpu_info[gpu_info_index].sub_id[field_index]);
					break;

				case NV_MSGBOX_CMD_ARG1_VENDOR_ID_V1:
					return &(gpu_info[gpu_info_index].vendor_id[field_index]);
					break;

				case NV_MSGBOX_CMD_ARG1_INFOROM_VER_V1:
					return &(gpu_info[gpu_info_index].inforom_version[field_index]);
					break;

				case NV_MSGBOX_CMD_ARG1_PCIE_WIDTH_V1:
					return &(gpu_info[gpu_info_index].pcie_width[field_index]);
					break;

				case NV_MSGBOX_CMD_ARG1_TGP_LIMIT_V1:
					return &(gpu_info[gpu_info_index].tgp_limit[field_index]);
					break;

				case NV_MSGBOX_CMD_ARG1_FRU_PART_NUMBER_V1:
					return &(gpu_info[gpu_info_index].fru_part_number[field_index]);
					break;

				case NV_MSGBOX_CMD_ARG1_GPU_PART_NUM_V1:
					return &(gpu_info[gpu_info_index].gpu_part_number[field_index]);
					break;

				case NV_MSGBOX_CMD_ARG1_MAX_DRAM_CAPACITY_V1:
					return &(gpu_info[gpu_info_index].max_dram_capacity[field_index]);
					break;
			}
			break;

		// SENSORS
		case NV_MSGBOX_CMD_OPCODE_GET_EXT_TEMP:
			return &(gpu_info[gpu_info_index].temp[field_index]);
			break;

		case NV_MSGBOX_CMD_OPCODE_GET_POWER:
			return &(gpu_info[gpu_info_index].power[field_index]);
			break;

		// PCIE_LINK_INFO
		case NV_MSGBOX_CMD_OPCODE_GET_PCIE_LINK_INFO:
			switch (arg1) 
			{
				case NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_0:
					return &(gpu_info[gpu_info_index].pcie_link_info_0[field_index]);
					break;
				case NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_1:
					return &(gpu_info[gpu_info_index].pcie_link_info_1[field_index]);
					break;
				case NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_2:
					return &(gpu_info[gpu_info_index].pcie_link_info_2[field_index]);
					break;
				case NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_3:
					return &(gpu_info[gpu_info_index].pcie_link_info_3[field_index]);
					break;
				case NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_4:
					return &(gpu_info[gpu_info_index].pcie_link_info_4[field_index]);
					break;
				case NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_6:
					return &(gpu_info[gpu_info_index].pcie_link_info_6[field_index]);
					break;
				case NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_8:
					return &(gpu_info[gpu_info_index].pcie_link_info_8[field_index]);
					break;
				case NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_9:
					return &(gpu_info[gpu_info_index].pcie_link_info_9[field_index]);
					break;
			}
			break;
	}
}


void cmd(uint64_t nvidia_mask, uint8_t cmd, uint8_t arg1)
{
	switch (cmd)
	{
		// SYS_INFO
		case NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA:
			switch (arg1)
			{
				case NV_MSGBOX_CMD_ARG1_FIRMWARE_VER_V1: 
					smbpbi_cmd(nvidia_mask, 
						   NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA, 
				   	   	   NV_MSGBOX_CMD_ARG1_FIRMWARE_VER_V1, 
				   	   	   NV_MSGBOX_SYSID_DATA_SIZE_FIRMWARE_VER_V1);
				break;

				case NV_MSGBOX_CMD_ARG1_BOARD_PART_NUM_V1:
					smbpbi_cmd(nvidia_mask, 
				   		   NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA, 
				   		   NV_MSGBOX_CMD_ARG1_BOARD_PART_NUM_V1, 
				   		   NV_MSGBOX_SYSID_DATA_SIZE_BOARD_PART_NUM_V1);
				break;

				case NV_MSGBOX_CMD_ARG1_SERIAL_NUM_V1:
					smbpbi_cmd(nvidia_mask, 
				   		   NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA, 
				   		   NV_MSGBOX_CMD_ARG1_SERIAL_NUM_V1, 
				   		   NV_MSGBOX_SYSID_DATA_SIZE_SERIAL_NUM_V1);
				break;

				case NV_MSGBOX_CMD_ARG1_MARKETING_NAME_V1:
					smbpbi_cmd(nvidia_mask, 
				   		   NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA, 
				   		   NV_MSGBOX_CMD_ARG1_MARKETING_NAME_V1, 
				   		   NV_MSGBOX_SYSID_DATA_SIZE_MARKETING_NAME_V1);
				break;

				case NV_MSGBOX_CMD_ARG1_GPU_GUID_V1:
					smbpbi_cmd(nvidia_mask, 
				   		   NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA, 
				   		   NV_MSGBOX_CMD_ARG1_GPU_GUID_V1, 
				   		   NV_MSGBOX_SYSID_DATA_SIZE_GPU_GUID_V1);
				break;

				case NV_MSGBOX_CMD_ARG1_DEV_ID_V1:
					smbpbi_cmd(nvidia_mask, 
				   		   NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA, 
				   		   NV_MSGBOX_CMD_ARG1_DEV_ID_V1, 
				   		   NV_MSGBOX_SYSID_DATA_SIZE_DEV_ID_V1);
				break;

				case NV_MSGBOX_CMD_ARG1_PCIE_SPEED_V1:
					smbpbi_cmd(nvidia_mask, 
				   		   NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA, 
				   		   NV_MSGBOX_CMD_ARG1_PCIE_SPEED_V1, 
				   		   NV_MSGBOX_SYSID_DATA_SIZE_PCIE_SPEED_V1);
				break;

				case NV_MSGBOX_CMD_ARG1_BUILD_DATE_V1:
					smbpbi_cmd(nvidia_mask, 
				   		   NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA, 
				   		   NV_MSGBOX_CMD_ARG1_BUILD_DATE_V1, 
				   		   NV_MSGBOX_SYSID_DATA_SIZE_BUILD_DATE_V1);
				break;

				case NV_MSGBOX_CMD_ARG1_SUB_VENDOR_ID_V1:
					smbpbi_cmd(nvidia_mask, 
				   		   NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA, 
				   		   NV_MSGBOX_CMD_ARG1_SUB_VENDOR_ID_V1, 
				   		   NV_MSGBOX_SYSID_DATA_SIZE_SUB_VENDOR_ID_V1);
				break;

				case NV_MSGBOX_CMD_ARG1_SUB_ID_V1:
					smbpbi_cmd(nvidia_mask, 
				   		   NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA, 
				   		   NV_MSGBOX_CMD_ARG1_SUB_ID_V1, 
				   		   NV_MSGBOX_SYSID_DATA_SIZE_SUB_ID_V1);
				break;

				case NV_MSGBOX_CMD_ARG1_VENDOR_ID_V1:
					smbpbi_cmd(nvidia_mask, 
				   		   NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA, 
				   		   NV_MSGBOX_CMD_ARG1_VENDOR_ID_V1, 
				   		   NV_MSGBOX_SYSID_DATA_SIZE_VENDOR_ID_V1);
				break;

				case NV_MSGBOX_CMD_ARG1_INFOROM_VER_V1:
					smbpbi_cmd(nvidia_mask, 
				   		   NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA, 
				   		   NV_MSGBOX_CMD_ARG1_INFOROM_VER_V1, 
				   		   NV_MSGBOX_SYSID_DATA_SIZE_INFOROM_VER_V1);
				break;

				case NV_MSGBOX_CMD_ARG1_PCIE_WIDTH_V1:
					smbpbi_cmd(nvidia_mask, 
				   		   NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA, 
				   		   NV_MSGBOX_CMD_ARG1_PCIE_WIDTH_V1, 
				   		   NV_MSGBOX_SYSID_DATA_SIZE_PCIE_WIDTH_V1);
				break;

				case NV_MSGBOX_CMD_ARG1_TGP_LIMIT_V1:
					smbpbi_cmd(nvidia_mask, 
				   		   NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA, 
				   		   NV_MSGBOX_CMD_ARG1_TGP_LIMIT_V1, 
				   		   NV_MSGBOX_SYSID_DATA_SIZE_TGP_LIMIT_V1);
				break;

				case NV_MSGBOX_CMD_ARG1_FRU_PART_NUMBER_V1:
					smbpbi_cmd(nvidia_mask, 
				   		   NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA, 
				   		   NV_MSGBOX_CMD_ARG1_FRU_PART_NUMBER_V1, 
				   		   NV_MSGBOX_SYSID_DATA_SIZE_FRU_PART_NUMBER_V1);
				break;

				case NV_MSGBOX_CMD_ARG1_GPU_PART_NUM_V1:
					smbpbi_cmd(nvidia_mask, 
				   		   NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA, 
				   		   NV_MSGBOX_CMD_ARG1_GPU_PART_NUM_V1, 
				   		   NV_MSGBOX_SYSID_DATA_SIZE_GPU_PART_NUM_V1);
				break;

				case NV_MSGBOX_CMD_ARG1_MAX_DRAM_CAPACITY_V1:
					smbpbi_cmd(nvidia_mask, 
				   		   NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA, 
				   		   NV_MSGBOX_CMD_ARG1_MAX_DRAM_CAPACITY_V1, 
				   		   NV_MSGBOX_SYSID_DATA_SIZE_MAX_DRAM_CAPACITY_V1);
				break;

			}
			break;

		// SENSORS
		case NV_MSGBOX_CMD_OPCODE_GET_EXT_TEMP:
			smbpbi_cmd(nvidia_mask, 
				   NV_MSGBOX_CMD_OPCODE_GET_EXT_TEMP, 
				   NV_MSGBOX_CMD_ARG1_NULL, 
				   SIZE_24I8F);
			break;

		case NV_MSGBOX_CMD_OPCODE_GET_POWER:
			smbpbi_cmd(nvidia_mask, 
				   NV_MSGBOX_CMD_OPCODE_GET_POWER, 
				   NV_MSGBOX_CMD_ARG1_NULL, 
				   DWORD_SZ);
			break;

		// PCIE_LINK_INFO
		case NV_MSGBOX_CMD_OPCODE_GET_PCIE_LINK_INFO:
			switch (arg1)
			{
				case NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_0: 
					smbpbi_cmd(nvidia_mask, 
						   NV_MSGBOX_CMD_OPCODE_GET_PCIE_LINK_INFO, 
				   	   	   NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_0, 
				   	   	   DWORD_SZ);
				break;

				case NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_1:
					smbpbi_cmd(nvidia_mask, 
						   NV_MSGBOX_CMD_OPCODE_GET_PCIE_LINK_INFO, 
				   	   	   NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_1, 
				   	   	   DWORD_SZ);
				break;

				case NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_2:
					smbpbi_cmd(nvidia_mask, 
						   NV_MSGBOX_CMD_OPCODE_GET_PCIE_LINK_INFO, 
				   	   	   NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_2, 
				   	   	   DWORD_SZ);
				break;

				case NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_3:
					smbpbi_cmd(nvidia_mask, 
						   NV_MSGBOX_CMD_OPCODE_GET_PCIE_LINK_INFO, 
				   	   	   NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_3, 
				   	   	   DWORD_SZ);
				break;

				case NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_4:
					smbpbi_cmd(nvidia_mask, 
						   NV_MSGBOX_CMD_OPCODE_GET_PCIE_LINK_INFO, 
				   	   	   NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_4, 
				   	   	   DWORD_SZ);
				break;

				case NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_6:
					smbpbi_cmd(nvidia_mask, 
						   NV_MSGBOX_CMD_OPCODE_GET_PCIE_LINK_INFO, 
				   	   	   NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_6, 
				   	   	   DWORD_SZ);
				break;

				case NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_8:
					smbpbi_cmd(nvidia_mask, 
						   NV_MSGBOX_CMD_OPCODE_GET_PCIE_LINK_INFO, 
				   	   	   NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_8, 
				   	   	   DWORD_SZ);
				break;

				case NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_9:
					smbpbi_cmd(nvidia_mask, 
						   NV_MSGBOX_CMD_OPCODE_GET_PCIE_LINK_INFO, 
				   	   	   NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_9, 
				   	   	   DWORD_SZ);
				break;

			}
			break;
	}
}

double pretty_temp(uint8_t* ptr)
{
	if (!ptr)
		return -1;

	uint32_t val = 0;
	memcpy(&val, ptr, sizeof(uint32_t));

	return convert_24i8f_to_double(val); // Cel
}

int pretty_power(uint8_t* ptr)
{
	if (!ptr)
		return -1;

	uint32_t val = 0;
	memcpy(&val, ptr, DWORD_SZ);

	return val / 1000; // kW to W
}

int pretty_pcie_speed(uint8_t* ptr)
{
	if (!ptr)
		return -1;
	
	int pcie_speed = (int) *ptr;

	return pcie_speed;
}

void print_pcie_link_info(uint8_t arg1, uint32_t raw)
{
        switch (arg1)
        {
                case NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_0: 
		{
			uint8_t link_speed = (raw & 0x7);
			switch (link_speed)
			{
				case NV_MSGBOX_DATA_PCIE_LINK_INFO_PAGE_0_LINK_SPEED_UNKNOWN:
					printf("Link Speed: Unknown\n");
					break;
				case NV_MSGBOX_DATA_PCIE_LINK_INFO_PAGE_0_LINK_SPEED_2500_MTPS:
					printf("Link Speed: 2500 MTPS\n");
					break;
				case NV_MSGBOX_DATA_PCIE_LINK_INFO_PAGE_0_LINK_SPEED_5000_MTPS:
					printf("Link Speed: 5000 MTPS\n");
					break;
				case NV_MSGBOX_DATA_PCIE_LINK_INFO_PAGE_0_LINK_SPEED_8000_MTPS:
					printf("Link Speed: 8000 MTPS\n");
					break;
				case NV_MSGBOX_DATA_PCIE_LINK_INFO_PAGE_0_LINK_SPEED_16000_MTPS:
					printf("Link Speed: 16000 MTPS\n");
					break;
				case NV_MSGBOX_DATA_PCIE_LINK_INFO_PAGE_0_LINK_SPEED_32000_MTPS:
					printf("Link Speed: 32000 MTPS\n");
					break;
			}

			uint8_t link_width = ((raw  >> 4) & 0x7);
			switch (link_width)
			{
				case NV_MSGBOX_DATA_PCIE_LINK_INFO_PAGE_0_LINK_WIDTH_UNKNOWN:
					printf("Link Width: Unknown\n");
					break;
				case NV_MSGBOX_DATA_PCIE_LINK_INFO_PAGE_0_LINK_WIDTH_X1:
					printf("Link Width: X1\n");
					break;
				case NV_MSGBOX_DATA_PCIE_LINK_INFO_PAGE_0_LINK_WIDTH_X2:
					printf("Link Width: X2\n");
					break;
				case NV_MSGBOX_DATA_PCIE_LINK_INFO_PAGE_0_LINK_WIDTH_X4:
					printf("Link Width: X4\n");
					break;
				case NV_MSGBOX_DATA_PCIE_LINK_INFO_PAGE_0_LINK_WIDTH_X8:
					printf("Link Width: X8\n");
					break;
				case NV_MSGBOX_DATA_PCIE_LINK_INFO_PAGE_0_LINK_WIDTH_X16:
					printf("Link Width: X16\n");
					break;
				case NV_MSGBOX_DATA_PCIE_LINK_INFO_PAGE_0_LINK_WIDTH_X32:
					printf("Link Width: X32\n");
					break;
			}

			uint8_t nonfatal_error_count = ((raw >> 8) & 0xFF);
			printf("Nonfatal Error Count: %d\n", nonfatal_error_count);

			uint8_t fatal_error_count = ((raw >> 16) & 0xFF);
			printf("Fatal Error Count: %d\n", fatal_error_count);

			uint8_t unsup_req_count = ((raw >> 24) & 0xFF);
			printf("Unsupported Req Count: %d\n", unsup_req_count);
			
                        break;
		}

                case NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_1:
			printf("PCIe L0 To Recovery count: %u\n", raw);
                        break;

                case NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_2:
		{
			uint16_t replay = (raw & 0x0000FFFF);
			uint16_t naks = ((raw >> 16) & 0x0000FFFF);
			printf("PCIe Replay Count: %u\n", replay);
			printf("PCIe NAK Count: %u\n", naks);
                        break;
		}

                case NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_3:
		{
			uint8_t target_link_speed = (raw & 0x7);
			switch (target_link_speed)
			{
				case NV_MSGBOX_DATA_PCIE_LINK_INFO_PAGE_3_TARGET_LINK_SPEED_UNKNOWN:
					printf("Target Link Speed: Unknown\n");
					break;
				case NV_MSGBOX_DATA_PCIE_LINK_INFO_PAGE_3_TARGET_LINK_SPEED_2500_MTPS:
					printf("Target Link Speed: 2500 MTPS\n");
					break;
				case NV_MSGBOX_DATA_PCIE_LINK_INFO_PAGE_3_TARGET_LINK_SPEED_5000_MTPS:
					printf("Target Link Speed: 5000 MTPS\n");
					break;
				case NV_MSGBOX_DATA_PCIE_LINK_INFO_PAGE_3_TARGET_LINK_SPEED_8000_MTPS:
					printf("Target Link Speed: 8000 MTPS\n");
					break;
				case NV_MSGBOX_DATA_PCIE_LINK_INFO_PAGE_3_TARGET_LINK_SPEED_16000_MTPS:
					printf("Target Link Speed: 16000 MTPS\n");
					break;
				case NV_MSGBOX_DATA_PCIE_LINK_INFO_PAGE_3_TARGET_LINK_SPEED_32000_MTPS:
					printf("Target Link Speed: 32000 MTPS\n");
					break;
			}

                        break;
		}



                case NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_4:
			printf("TX Count: %u\n", raw);
                        break;

                case NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_6:
		{
			uint8_t ltssm_curr_state = (raw & 0x1F);
			const char* msg_prefix = "LTSSM Current State";
			switch (ltssm_curr_state)
			{
				case NV_MSGBOX_DATA_PCIE_LINK_INFO_PAGE_6_LTSSM_STATE_DETECT:
					printf("%s: Detect\n", msg_prefix);
					break;
				case NV_MSGBOX_DATA_PCIE_LINK_INFO_PAGE_6_LTSSM_STATE_POLLING:
					printf("%s: Polling\n", msg_prefix);
					break;
				case NV_MSGBOX_DATA_PCIE_LINK_INFO_PAGE_6_LTSSM_STATE_CONFIGURATION:
					printf("%s: Configuration\n", msg_prefix);
					break;
				case NV_MSGBOX_DATA_PCIE_LINK_INFO_PAGE_6_LTSSM_STATE_RECOVERY:
					printf("%s: Recovery\n", msg_prefix);
					break;
				case NV_MSGBOX_DATA_PCIE_LINK_INFO_PAGE_6_LTSSM_STATE_RECOVERY_EQZN:
					printf("%s: Recovery EQZN\n", msg_prefix);
					break;
				case NV_MSGBOX_DATA_PCIE_LINK_INFO_PAGE_6_LTSSM_STATE_L0:
					printf("%s: L0\n", msg_prefix);
					break;
				case NV_MSGBOX_DATA_PCIE_LINK_INFO_PAGE_6_LTSSM_STATE_L0S:
					printf("%s: L0S\n", msg_prefix);
					break;
				case NV_MSGBOX_DATA_PCIE_LINK_INFO_PAGE_6_LTSSM_STATE_L1_PLL_PD:
					printf("%s: L1 PLL PD\n", msg_prefix);
					break;
				case NV_MSGBOX_DATA_PCIE_LINK_INFO_PAGE_6_LTSSM_STATE_L2:
					printf("%s: L2\n", msg_prefix);
					break;
				case NV_MSGBOX_DATA_PCIE_LINK_INFO_PAGE_6_LTSSM_STATE_L1_CPM:
					printf("%s: L1 CPM\n", msg_prefix);
					break;
				case NV_MSGBOX_DATA_PCIE_LINK_INFO_PAGE_6_LTSSM_STATE_L1_1:
					printf("%s: L1 1\n", msg_prefix);
					break;
				case NV_MSGBOX_DATA_PCIE_LINK_INFO_PAGE_6_LTSSM_STATE_L1_2:
					printf("%s: L1 2\n", msg_prefix);
					break;
				case NV_MSGBOX_DATA_PCIE_LINK_INFO_PAGE_6_LTSSM_STATE_HOT_RESET:
					printf("%s: Hot Reset\n", msg_prefix);
					break;
				case NV_MSGBOX_DATA_PCIE_LINK_INFO_PAGE_6_LTSSM_STATE_LOOPBACK:
					printf("%s: Loopback\n", msg_prefix);
					break;
				case NV_MSGBOX_DATA_PCIE_LINK_INFO_PAGE_6_LTSSM_STATE_DISABLED:
					printf("%s: Disabled\n", msg_prefix);
					break;
				case NV_MSGBOX_DATA_PCIE_LINK_INFO_PAGE_6_LTSSM_STATE_LINK_DOWN:
					printf("%s: Link Down\n", msg_prefix);
					break;
				case NV_MSGBOX_DATA_PCIE_LINK_INFO_PAGE_6_LTSSM_STATE_LINK_READY:
					printf("%s: Link Ready\n", msg_prefix);
					break;
				case NV_MSGBOX_DATA_PCIE_LINK_INFO_PAGE_6_LTSSM_STATE_LANES_IN_SLEEP:
					printf("%s: Lanes In Speed\n", msg_prefix);
					break;
				case NV_MSGBOX_DATA_PCIE_LINK_INFO_PAGE_6_LTSSM_STATE_ILLEGAL:
					printf("%s: Illegal\n", msg_prefix);
					break;
			}

                        break;
		}

                case NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_8:
		{
			uint8_t tx_preset = (raw & 0xF);
			uint8_t tx_use_preset = ((raw >> 4) & 0x1);
			uint8_t tx_fs= ((raw >> 5) & 0x3F);
			uint8_t tx_lf= ((raw >> 11) & 0x3F);

			printf("TX Equalization:\n");
			printf("  Preset: %u\n", tx_preset);
			printf("  Use preset: %s\n", tx_use_preset ? "Yes" : "No");
			printf("  Full Swing (FS): %u\n", tx_fs);
			printf("  Low Freq (LF): %u\n", tx_lf);

                        break;
		}

                case NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_9:
		{
			uint8_t rx_preset = (raw & 0xF);
			uint8_t rx_use_preset = ((raw >> 4) & 0x1);
			uint8_t rx_fs= ((raw >> 5) & 0x3F);
			uint8_t rx_lf= ((raw >> 11) & 0x3F);

			printf("RX Equalization:\n");
			printf("  Preset: %u\n", rx_preset);
			printf("  Use preset: %s\n", rx_use_preset ? "Yes" : "No");
			printf("  Full Swing (FS): %u\n", rx_fs);
			printf("  Low Freq (LF): %u\n", rx_lf);

                        break;
		}
        }
}

void pretty_print(uint64_t mask)
{
	printf("=======================================\n");
	for (int i = 0, k = 0; i < sizeof(mask) * 8; i++) 
	{
		if ((1ULL << i) & mask)
		{
			printf("Find NVIDIA GPU on /dev/i2c-%d\n", i);
			printf("----------------SYS_INFO----------------\n");
			printf("Marketing Name: ");
			for (int j = 0; j < NV_MSGBOX_SYSID_DATA_SIZE_MARKETING_NAME_V1; j++) 
			{
				printf("%c", gpu_info[k].marketing_name[j]);
			}
			printf("\n");

			printf("Serial Number: ");
			for (int j = 0; j < NV_MSGBOX_SYSID_DATA_SIZE_SERIAL_NUM_V1; j++) 
			{
				printf("%c", gpu_info[k].serial_number[j]);
			}
			printf("\n");

			printf("Board Part Number: ");
			for (int j = 0; j < NV_MSGBOX_SYSID_DATA_SIZE_BOARD_PART_NUM_V1; j++) 
			{
				printf("%c", gpu_info[k].board_part_number[j]);
			}
			printf("\n");

			printf("GPU Part Number: ");
			for (int j = 0; j < NV_MSGBOX_SYSID_DATA_SIZE_GPU_PART_NUM_V1; j++) 
			{
				printf("%c", gpu_info[k].gpu_part_number[j]);
			}
			printf("\n");

			printf("VBIOS Version: ");
			for (int j = 0; j < NV_MSGBOX_SYSID_DATA_SIZE_FIRMWARE_VER_V1; j++) 
			{
				printf("%c", gpu_info[k].fwversion[j]);
			}
			printf("\n");

			printf("Build Date: %x.%x.%x%x\n", gpu_info[k].build_date[0], gpu_info[k].build_date[1],
							 gpu_info[k].build_date[3],gpu_info[k].build_date[2]);
			
			printf("GUID: 0x");
			for (int j = 0; j < NV_MSGBOX_SYSID_DATA_SIZE_GPU_GUID_V1; j++) 
			{
				printf("%x", gpu_info[k].guid[j]);
			}
			printf("\n");

			printf("VendorId: 0x%x%x\n", gpu_info[k].vendor_id[1], gpu_info[k].vendor_id[0]);

			printf("DeviceId: 0x%x%x\n", gpu_info[k].dev_id[1], gpu_info[k].dev_id[0]);

			printf("SubvendorId: 0x%x%x\n", gpu_info[k].sub_vendor_id[1], gpu_info[k].sub_vendor_id[0]);

			printf("SubId: 0x%x%x\n", gpu_info[k].sub_id[1], gpu_info[k].sub_id[0]);

			printf("InfoROM Version: ");
			for (int j = 0; j < NV_MSGBOX_SYSID_DATA_SIZE_INFOROM_VER_V1; j++) 
			{
				printf("%c", gpu_info[k].inforom_version[j]);
			}
			printf("\n");

			printf("PCIe Speed: PCIe %d.0\n", pretty_pcie_speed(gpu_info[k].pcie_speed));

			printf("PCIe Width: %d\n", gpu_info[k].pcie_width[0]);
			uint32_t tgp_limit = 0;
			memcpy(&tgp_limit, gpu_info[k].tgp_limit, NV_MSGBOX_SYSID_DATA_SIZE_TGP_LIMIT_V1); // little endian
			printf("TGP (Total Graphics Power) Limit: %d W\n", tgp_limit / 1000); // kW to W

			printf("FRU Part Number: 0x");
			for (int j = NV_MSGBOX_SYSID_DATA_SIZE_FRU_PART_NUMBER_V1 - 1; j >= 0; j--) 
			{
				printf("%x", gpu_info[k].fru_part_number[j]);
			}
			printf("\n");

			uint32_t max_dram_capacity = 0;
			memcpy(&max_dram_capacity, gpu_info[k].max_dram_capacity, NV_MSGBOX_SYSID_DATA_SIZE_MAX_DRAM_CAPACITY_V1); // little endian
			printf("Max DRAM Capacity: %d Mb\n", max_dram_capacity);

			printf("----------------SENSORS-----------------\n");
			printf("GPU Temp: %.1f C\n", pretty_temp(gpu_info[k].temp));

			printf("GPU Power: %d W\n", pretty_power(gpu_info[k].power)); // kW to W*/

			printf("--------------PCIE_LINK_INFO---------------\n");
			
			print_pcie_link_info(NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_0, gpu_info[k].pcie_link_info_0[0]);
			print_pcie_link_info(NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_1, gpu_info[k].pcie_link_info_1[0]);
			print_pcie_link_info(NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_2, gpu_info[k].pcie_link_info_2[0]);
			print_pcie_link_info(NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_3, gpu_info[k].pcie_link_info_3[0]);
			print_pcie_link_info(NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_4, gpu_info[k].pcie_link_info_4[0]);
			print_pcie_link_info(NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_6, gpu_info[k].pcie_link_info_6[0]);
			print_pcie_link_info(NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_8, gpu_info[k].pcie_link_info_8[0]);
			print_pcie_link_info(NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_9, gpu_info[k].pcie_link_info_9[0]);

			k > 0 && printf("=======================================\n");
			k++;
		}
	}
	printf("=======================================\n");
}

int main()
{
	uint64_t nvidia_mask = detect_nvidia_device();

	if (!nvidia_mask)
	{
		printf("NVIDIA devices not found\n");
		return 0;
	} 

	// SYS_INFO
	cmd(nvidia_mask, NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA, NV_MSGBOX_CMD_ARG1_FIRMWARE_VER_V1);
	cmd(nvidia_mask, NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA, NV_MSGBOX_CMD_ARG1_BOARD_PART_NUM_V1);
	cmd(nvidia_mask, NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA, NV_MSGBOX_CMD_ARG1_SERIAL_NUM_V1);
	cmd(nvidia_mask, NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA, NV_MSGBOX_CMD_ARG1_MARKETING_NAME_V1);
	cmd(nvidia_mask, NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA, NV_MSGBOX_CMD_ARG1_GPU_GUID_V1);
	cmd(nvidia_mask, NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA, NV_MSGBOX_CMD_ARG1_DEV_ID_V1);
	cmd(nvidia_mask, NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA, NV_MSGBOX_CMD_ARG1_PCIE_SPEED_V1);
	cmd(nvidia_mask, NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA, NV_MSGBOX_CMD_ARG1_BUILD_DATE_V1);
	cmd(nvidia_mask, NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA, NV_MSGBOX_CMD_ARG1_SUB_VENDOR_ID_V1);
	cmd(nvidia_mask, NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA, NV_MSGBOX_CMD_ARG1_SUB_ID_V1);
	cmd(nvidia_mask, NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA, NV_MSGBOX_CMD_ARG1_VENDOR_ID_V1);
	cmd(nvidia_mask, NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA, NV_MSGBOX_CMD_ARG1_INFOROM_VER_V1);
	cmd(nvidia_mask, NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA, NV_MSGBOX_CMD_ARG1_PCIE_WIDTH_V1);
	cmd(nvidia_mask, NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA, NV_MSGBOX_CMD_ARG1_TGP_LIMIT_V1);
	cmd(nvidia_mask, NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA, NV_MSGBOX_CMD_ARG1_FRU_PART_NUMBER_V1);
	cmd(nvidia_mask, NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA, NV_MSGBOX_CMD_ARG1_GPU_PART_NUM_V1);
	cmd(nvidia_mask, NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA, NV_MSGBOX_CMD_ARG1_MAX_DRAM_CAPACITY_V1);

	// SENSORS
	cmd(nvidia_mask, NV_MSGBOX_CMD_OPCODE_GET_EXT_TEMP, NV_MSGBOX_CMD_ARG1_NULL);
	cmd(nvidia_mask, NV_MSGBOX_CMD_OPCODE_GET_POWER, NV_MSGBOX_CMD_ARG1_NULL);

	// PCIE_LINK_INFO
	cmd(nvidia_mask, NV_MSGBOX_CMD_OPCODE_GET_PCIE_LINK_INFO, NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_0);
	cmd(nvidia_mask, NV_MSGBOX_CMD_OPCODE_GET_PCIE_LINK_INFO, NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_1);
	cmd(nvidia_mask, NV_MSGBOX_CMD_OPCODE_GET_PCIE_LINK_INFO, NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_2);
	cmd(nvidia_mask, NV_MSGBOX_CMD_OPCODE_GET_PCIE_LINK_INFO, NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_3);
	cmd(nvidia_mask, NV_MSGBOX_CMD_OPCODE_GET_PCIE_LINK_INFO, NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_4);
	cmd(nvidia_mask, NV_MSGBOX_CMD_OPCODE_GET_PCIE_LINK_INFO, NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_6);
	cmd(nvidia_mask, NV_MSGBOX_CMD_OPCODE_GET_PCIE_LINK_INFO, NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_8);
	cmd(nvidia_mask, NV_MSGBOX_CMD_OPCODE_GET_PCIE_LINK_INFO, NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_9);

	pretty_print(nvidia_mask);

	return 0;
}
