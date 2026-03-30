#include <stdio.h>
#include <getopt.h>
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

#define DEF_COPY_SZ 4

#define MAX_BUF_SZ 64

#define DWORD_SZ 4

#define NVIDIA_VENDOR_ID0 0x10
#define NVIDIA_VENDOR_ID1 0xDE

#define MAX_GPUS 8

#define SMBPBI_ADDR 0x4f
#define SMBPBI_SET_REG 0x5c
#define SMBPBI_GET_REG 0x5d
#define SMBPBI_GET_LEN 5 // cc + reg size

#define STATUS_GOOD 0x80

struct CmdData
{
	uint8_t opcode;
	uint8_t arg1;
	uint8_t type_size;
	void (*output)(const char*, const char*, int, int, int);
	const char* prefix;
	const char* suffix;
};

uint8_t buffer[MAX_BUF_SZ];

uint8_t* get_buf_ptr(uint8_t clean)
{
	if (clean) 
		memset(buffer, 0, sizeof(buffer));
	return buffer;
}

void usage(const char* progname);
struct CmdData get_cmd_by_id(const char* cmd);
int oneshot_cmd(uint8_t bus_id, struct CmdData cmd_data, int do_pure, int do_raw);

double convert_24i8f_to_double(uint32_t val)
{
	int i = 0;
	double d = 0;
	unsigned char* ptr = (unsigned char*) &val;	
	memcpy(&i, ptr + 1, 3);
	d = (double) ptr[0] / 256;
	
	return i + d;
}

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

	usleep(10000);

	if (i2c_wr_rd(dev, SMBPBI_ADDR, SMBPBI_GET_REG, recv_buf, recv_size) == -1)
		return -1;

	usleep(10000);

	memcpy(buf, recv_buf, count);
	return 0;
}

int cmd(uint8_t bus_id, uint8_t opcode, uint8_t arg1, uint8_t type_size, uint8_t* buf)
{
	if (!buf)
		return -1;

	char bus[32] = {};
	snprintf(bus, 32, "/dev/i2c-%d", bus_id);

	uint8_t recv_buf[] = {0x0, 0x0, 0x0, 0x0, 0x0};

	for (int j = 0, arg2 = 0; j < type_size; j += DEF_COPY_SZ, arg2++)
	{
		if (get_smbpbi_data(bus, opcode, arg1, arg2, STATUS_GOOD, 
				    recv_buf, SMBPBI_GET_LEN) == -1)
		{
			fprintf(stderr, "Failed SMBPBI request\n");
			return -1;
		}

		uint8_t is_last_transaction = ((j + DEF_COPY_SZ) > type_size);
		if (is_last_transaction)
		{
			memcpy(&buf[j], &recv_buf[1], type_size - j);
			break;
		}

		memcpy(&buf[j], &recv_buf[1], DEF_COPY_SZ);
	}

}

void output_char(const char* prefix, const char* suffix, int do_pure, int do_raw, int type_size)
{
	!do_pure && !do_raw && printf("%s", prefix);
	uint8_t* ptr = get_buf_ptr(0);
	for (int j = 0; j < type_size; j++) 
	{
		!do_raw && printf("%c", ptr[j]);
		do_raw && printf("%x ", ptr[j]);
	}
	!do_pure && !do_raw && printf("%s \n", suffix);
}

void output_dec_num(const char* prefix, const char* suffix, int do_pure, int do_raw, int type_size)
{
	uint32_t* val = (uint32_t*) get_buf_ptr(0);
	if (do_raw)
	{
		for (int i = 0; i < type_size; i++) 
		{
			printf("%x ", (uint8_t) val[i]);
		}
		printf("\n");
		return;
	}

	if (do_pure) 
	{
		printf("%u", *val);
		return;
	}

	printf("%s%u%s\n", prefix, *val, suffix);
}

void output_date(const char* prefix, const char* suffix, int do_pure, int do_raw, int type_size)
{
	uint8_t* date = (uint8_t*) get_buf_ptr(0);
	if (do_raw) 
	{
		for (int i = 0; i < type_size; i++) 
		{
			printf("%x ", date[i]);
		}
		printf("\n");
		return;
	}
	
	if (do_pure)
	{
		printf("%x.%x.%x%x\n", date[0], date[1], date[3], date[2]);
		return;
	}

	printf("%s%x.%x.%x%x%s\n", prefix, date[0], date[1], date[3], date[2], suffix);
}

void output_24i8f(const char* prefix, const char* suffix, int do_pure, int do_raw, int type_size)
{
	uint32_t* val = (uint32_t*) get_buf_ptr(0);

	double d = convert_24i8f_to_double(*val);

	if (do_raw)
	{
		for (int i = 0; i < type_size; i++) 
		{
			printf("%x ", (uint8_t) val[i]);
		}
		printf("\n");
		return;
	}

	if (do_pure) 
	{
		printf("%.2f", d);
		return;
	}

	printf("%s%.2f%s\n", prefix, d, suffix);
}

void output_bytes(const char* prefix, const char* suffix, int do_pure, int do_raw, int type_size)
{
	!do_pure && !do_raw && printf("%s", prefix);
	uint8_t* ptr = get_buf_ptr(0);
	!do_raw && printf("0x");
	for (int j = type_size - 1; j >= 0; j--) 
	{
		do_raw && printf("%x ", ptr[j]);
		!do_raw && printf("%x", ptr[j]);
	}
	!do_pure && !do_raw && printf("%s \n", suffix);
}

void output_pcie_link_info0(const char* prefix, const char* suffix, int do_pure, int do_raw, int type_size)
{
	uint32_t* raw = (uint32_t*) get_buf_ptr(0);

	uint8_t link_speed = (*raw & 0x7);
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

	uint8_t link_width = ((*raw  >> 4) & 0x7);
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

	uint8_t nonfatal_error_count = ((*raw >> 8) & 0xFF);
	printf("Nonfatal Error Count: %d\n", nonfatal_error_count);

	uint8_t fatal_error_count = ((*raw >> 16) & 0xFF);
	printf("Fatal Error Count: %d\n", fatal_error_count);

	uint8_t unsup_req_count = ((*raw >> 24) & 0xFF);
	printf("Unsupported Req Count: %d\n", unsup_req_count);
}

void output_pcie_link_info1(const char* prefix, const char* suffix, int do_pure, int do_raw, int type_size)
{
	uint32_t* raw = (uint32_t*) get_buf_ptr(0);
	printf("PCIe L0 To Recovery count: %u\n", *raw);

}

void output_pcie_link_info2(const char* prefix, const char* suffix, int do_pure, int do_raw, int type_size)
{
	uint32_t* raw = (uint32_t*) get_buf_ptr(0);
	uint16_t replay = (*raw & 0xFFFF);
	uint16_t naks = ((*raw >> 16) & 0xFFFF);
	printf("PCIe Replay Count: %u\n", replay);
	printf("PCIe NAK Count: %u\n", naks);

}

void output_pcie_link_info3(const char* prefix, const char* suffix, int do_pure, int do_raw, int type_size)
{
	uint32_t* raw = (uint32_t*) get_buf_ptr(0);
	uint8_t target_link_speed = (*raw & 0x7);
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
}

void output_pcie_link_info4(const char* prefix, const char* suffix, int do_pure, int do_raw, int type_size)
{
	uint32_t* raw = (uint32_t*) get_buf_ptr(0);
	printf("TX Count: %u\n", *raw);
}

void output_pcie_link_info6(const char* prefix, const char* suffix, int do_pure, int do_raw, int type_size)
{
	uint32_t* raw = (uint32_t*) get_buf_ptr(0);
	uint8_t ltssm_curr_state = (*raw & 0x1F);
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
}

void output_pcie_link_info8(const char* prefix, const char* suffix, int do_pure, int do_raw, int type_size)
{
	uint32_t* raw = (uint32_t*) get_buf_ptr(0);
	uint8_t tx_preset = (*raw & 0xF);
	uint8_t tx_use_preset = ((*raw >> 4) & 0x1);
	uint8_t tx_fs= ((*raw >> 5) & 0x3F);
	uint8_t tx_lf= ((*raw >> 11) & 0x3F);

	printf("TX Equalization:\n");
	printf("  Preset: %u\n", tx_preset);
	printf("  Use preset: %s\n", tx_use_preset ? "Yes" : "No");
	printf("  Full Swing (FS): %u\n", tx_fs);
	printf("  Low Freq (LF): %u\n", tx_lf);

}

void output_pcie_link_info9(const char* prefix, const char* suffix, int do_pure, int do_raw, int type_size)
{
	uint32_t* raw = (uint32_t*) get_buf_ptr(0);
	uint8_t rx_preset = (*raw & 0xF);
	uint8_t rx_use_preset = ((*raw >> 4) & 0x1);
	uint8_t rx_fs= ((*raw >> 5) & 0x3F);
	uint8_t rx_lf= ((*raw >> 11) & 0x3F);

	printf("RX Equalization:\n");
	printf("  Preset: %u\n", rx_preset);
	printf("  Use preset: %s\n", rx_use_preset ? "Yes" : "No");
	printf("  Full Swing (FS): %u\n", rx_fs);
	printf("  Low Freq (LF): %u\n", rx_lf);
}

void print_sysinfo(uint8_t bus_id)
{
	printf("----------------SYS_INFO----------------\n");

	struct CmdData cmd_data = get_cmd_by_id("vbios-ver");
	cmd(bus_id, cmd_data.opcode, cmd_data.arg1, cmd_data.type_size, get_buf_ptr(1));
	(cmd_data.output)(cmd_data.prefix, cmd_data.suffix, 0, 0, cmd_data.type_size);

	cmd_data = get_cmd_by_id("board-pn");
	cmd(bus_id, cmd_data.opcode, cmd_data.arg1, cmd_data.type_size, get_buf_ptr(1));
	(cmd_data.output)(cmd_data.prefix, cmd_data.suffix, 0, 0, cmd_data.type_size);

	cmd_data = get_cmd_by_id("sn");
	cmd(bus_id, cmd_data.opcode, cmd_data.arg1, cmd_data.type_size, get_buf_ptr(1));
	(cmd_data.output)(cmd_data.prefix, cmd_data.suffix, 0, 0, cmd_data.type_size);

	cmd_data = get_cmd_by_id("marketing-name");
	cmd(bus_id, cmd_data.opcode, cmd_data.arg1, cmd_data.type_size, get_buf_ptr(1));
	(cmd_data.output)(cmd_data.prefix, cmd_data.suffix, 0, 0, cmd_data.type_size);

	cmd_data = get_cmd_by_id("gpu-pn");
	cmd(bus_id, cmd_data.opcode, cmd_data.arg1, cmd_data.type_size, get_buf_ptr(1));
	(cmd_data.output)(cmd_data.prefix, cmd_data.suffix, 0, 0, cmd_data.type_size);

	cmd_data = get_cmd_by_id("build-date");
	cmd(bus_id, cmd_data.opcode, cmd_data.arg1, cmd_data.type_size, get_buf_ptr(1));
	(cmd_data.output)(cmd_data.prefix, cmd_data.suffix, 0, 0, cmd_data.type_size);

	cmd_data = get_cmd_by_id("vendor-id");
	cmd(bus_id, cmd_data.opcode, cmd_data.arg1, cmd_data.type_size, get_buf_ptr(1));
	(cmd_data.output)(cmd_data.prefix, cmd_data.suffix, 0, 0, cmd_data.type_size);

	cmd_data = get_cmd_by_id("dev-id");
	cmd(bus_id, cmd_data.opcode, cmd_data.arg1, cmd_data.type_size, get_buf_ptr(1));
	(cmd_data.output)(cmd_data.prefix, cmd_data.suffix, 0, 0, cmd_data.type_size);

	cmd_data = get_cmd_by_id("subvendor-id");
	cmd(bus_id, cmd_data.opcode, cmd_data.arg1, cmd_data.type_size, get_buf_ptr(1));
	(cmd_data.output)(cmd_data.prefix, cmd_data.suffix, 0, 0, cmd_data.type_size);

	cmd_data = get_cmd_by_id("sub-id");
	cmd(bus_id, cmd_data.opcode, cmd_data.arg1, cmd_data.type_size, get_buf_ptr(1));
	(cmd_data.output)(cmd_data.prefix, cmd_data.suffix, 0, 0, cmd_data.type_size);

	cmd_data = get_cmd_by_id("gpu-guid");
	cmd(bus_id, cmd_data.opcode, cmd_data.arg1, cmd_data.type_size, get_buf_ptr(1));
	(cmd_data.output)(cmd_data.prefix, cmd_data.suffix, 0, 0, cmd_data.type_size);

	cmd_data = get_cmd_by_id("inforom-ver");
	cmd(bus_id, cmd_data.opcode, cmd_data.arg1, cmd_data.type_size, get_buf_ptr(1));
	(cmd_data.output)(cmd_data.prefix, cmd_data.suffix, 0, 0, cmd_data.type_size);

	cmd_data = get_cmd_by_id("pcie-speed");
	cmd(bus_id, cmd_data.opcode, cmd_data.arg1, cmd_data.type_size, get_buf_ptr(1));
	(cmd_data.output)(cmd_data.prefix, cmd_data.suffix, 0, 0, cmd_data.type_size);

	cmd_data = get_cmd_by_id("pcie-width");
	cmd(bus_id, cmd_data.opcode, cmd_data.arg1, cmd_data.type_size, get_buf_ptr(1));
	(cmd_data.output)(cmd_data.prefix, cmd_data.suffix, 0, 0, cmd_data.type_size);

	cmd_data = get_cmd_by_id("tgp-limit");
	cmd(bus_id, cmd_data.opcode, cmd_data.arg1, cmd_data.type_size, get_buf_ptr(1));
	(cmd_data.output)(cmd_data.prefix, cmd_data.suffix, 0, 0, cmd_data.type_size);

	cmd_data = get_cmd_by_id("fru-pn");
	cmd(bus_id, cmd_data.opcode, cmd_data.arg1, cmd_data.type_size, get_buf_ptr(1));
	(cmd_data.output)(cmd_data.prefix, cmd_data.suffix, 0, 0, cmd_data.type_size);

	cmd_data = get_cmd_by_id("max-dram-capacity");
	cmd(bus_id, cmd_data.opcode, cmd_data.arg1, cmd_data.type_size, get_buf_ptr(1));
	(cmd_data.output)(cmd_data.prefix, cmd_data.suffix, 0, 0, cmd_data.type_size);
}

int print_detect(uint64_t mask, int do_pure)
{
	int count = 0;
	for (int i = 0, k = 0; i < sizeof(uint64_t) * 8; i++)
	{
		if ((1ULL << i) & mask) 
		{
			!do_pure && printf("Detect NVIDIA device on /dev/i2c-%d with index %d\n", i, k);
			count++;
			k++;
		}
	}

	if (!do_pure)
	{
		printf("In total, %d device(s) was detected\n", count);
		exit(0);
	}

	do_pure && printf("%d", count);

	return count;
}

void print_sensors(uint8_t bus_id)
{
	printf("----------------SENSORS-----------------\n");

	struct CmdData cmd_data = get_cmd_by_id("temp");
	cmd(bus_id, cmd_data.opcode, cmd_data.arg1, cmd_data.type_size, get_buf_ptr(1));
	(cmd_data.output)(cmd_data.prefix, cmd_data.suffix, 0, 0, cmd_data.type_size);

	cmd_data = get_cmd_by_id("power");
	cmd(bus_id, cmd_data.opcode, cmd_data.arg1, cmd_data.type_size, get_buf_ptr(1));
	(cmd_data.output)(cmd_data.prefix, cmd_data.suffix, 0, 0, cmd_data.type_size);
}


struct CmdData get_cmd_by_id(const char* cmd)
{
	struct CmdData data = {};

	// SYS_INFO
	if (strcmp(cmd, "vbios-ver") == 0)
	{
		data.opcode = NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA;
		data.arg1 = NV_MSGBOX_CMD_ARG1_FIRMWARE_VER_V1;
		data.type_size = NV_MSGBOX_SYSID_DATA_SIZE_FIRMWARE_VER_V1;
		data.output = output_char;
		data.prefix = "VBIOS Version: ";
		data.suffix = "";
	} 
	else if (strcmp(cmd, "board-pn") == 0) 
	{
		data.opcode = NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA; 
		data.arg1 = NV_MSGBOX_CMD_ARG1_BOARD_PART_NUM_V1;
		data.type_size = NV_MSGBOX_SYSID_DATA_SIZE_BOARD_PART_NUM_V1;
		data.output =  output_char;
		data.prefix = "Board Part Number: ";
		data.suffix = "";
	}
	else if (strcmp(cmd, "sn") == 0) 
	{
		data.opcode = NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA;
		data.arg1 =  NV_MSGBOX_CMD_ARG1_SERIAL_NUM_V1;
		data.type_size = NV_MSGBOX_SYSID_DATA_SIZE_SERIAL_NUM_V1;
		data.output = output_char;
		data.prefix = "Serial Number: ";
		data.suffix = "";
	}
	else if (strcmp(cmd, "marketing-name") == 0) 
	{
		data.opcode = NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA;
		data.arg1 =  NV_MSGBOX_CMD_ARG1_MARKETING_NAME_V1;
		data.type_size = NV_MSGBOX_SYSID_DATA_SIZE_MARKETING_NAME_V1;
		data.output = output_char;
		data.prefix = "Marketing Name: ";
		data.suffix = "";
	}
	else if (strcmp(cmd, "gpu-pn") == 0)
	{
		data.opcode = NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA;
		data.arg1 = NV_MSGBOX_CMD_ARG1_GPU_PART_NUM_V1;
		data.type_size = NV_MSGBOX_SYSID_DATA_SIZE_GPU_PART_NUM_V1;
		data.output = output_char;
		data.prefix = "GPU Part Number: ";
		data.suffix = "";
	}
	else if (strcmp(cmd, "build-date") == 0)
	{
		data.opcode = NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA;
		data.arg1 = NV_MSGBOX_CMD_ARG1_BUILD_DATE_V1;
		data.type_size = NV_MSGBOX_SYSID_DATA_SIZE_BUILD_DATE_V1;
		data.output = output_date;
		data.prefix = "Build Date: ";
		data.suffix = "";
	}
	else if (strcmp(cmd, "vendor-id") == 0)
	{
		data.opcode = NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA;
		data.arg1 = NV_MSGBOX_CMD_ARG1_VENDOR_ID_V1;
		data.type_size = NV_MSGBOX_SYSID_DATA_SIZE_VENDOR_ID_V1;
		data.output = output_bytes;
		data.prefix = "VendorID: ";
		data.suffix = "";
	}
	else if (strcmp(cmd, "dev-id") == 0)
	{
		data.opcode = NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA;
		data.arg1 = NV_MSGBOX_CMD_ARG1_DEV_ID_V1;
		data.type_size = NV_MSGBOX_SYSID_DATA_SIZE_DEV_ID_V1;
		data.output = output_bytes;
		data.prefix = "DeviceID: ";
		data.suffix = "";
	}
	else if (strcmp(cmd, "subvendor-id") == 0)
	{
		data.opcode = NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA;
		data.arg1 = NV_MSGBOX_CMD_ARG1_SUB_VENDOR_ID_V1;
		data.type_size = NV_MSGBOX_SYSID_DATA_SIZE_SUB_VENDOR_ID_V1;
		data.output = output_bytes;
		data.prefix = "SubvendorID: ";
		data.suffix = "";
	}
	else if (strcmp(cmd, "sub-id") == 0)
	{
		data.opcode = NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA;
		data.arg1 = NV_MSGBOX_CMD_ARG1_SUB_ID_V1;
		data.type_size = NV_MSGBOX_SYSID_DATA_SIZE_SUB_ID_V1;
		data.output = output_bytes;
		data.prefix = "SubID: ";
		data.suffix = "";
	}
	else if (strcmp(cmd, "gpu-guid") == 0)
	{
		data.opcode = NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA;
		data.arg1 = NV_MSGBOX_CMD_ARG1_GPU_GUID_V1;
		data.type_size = NV_MSGBOX_SYSID_DATA_SIZE_GPU_GUID_V1;
		data.output = output_bytes;
		data.prefix = "GPU GUID: ";
		data.suffix = "";
	}
	else if (strcmp(cmd, "inforom-ver") == 0)
	{
		data.opcode = NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA;
		data.arg1 = NV_MSGBOX_CMD_ARG1_INFOROM_VER_V1;
		data.type_size = NV_MSGBOX_SYSID_DATA_SIZE_INFOROM_VER_V1;
		data.output = output_char;
		data.prefix = "InfoROM Version: ";
		data.suffix = "";
	}
	else if (strcmp(cmd, "pcie-speed") == 0)
	{
		data.opcode = NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA; 
		data.arg1 = NV_MSGBOX_CMD_ARG1_PCIE_SPEED_V1;
		data.type_size = NV_MSGBOX_SYSID_DATA_SIZE_PCIE_SPEED_V1;
		data.output = output_dec_num;
		data.prefix = "PCIe Speed: ";
		data.suffix = "";
	}
	else if (strcmp(cmd, "pcie-width") == 0)
	{
		data.opcode = NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA;
		data.arg1 = NV_MSGBOX_CMD_ARG1_PCIE_WIDTH_V1;
		data.type_size = NV_MSGBOX_SYSID_DATA_SIZE_PCIE_WIDTH_V1;
		data.output = output_dec_num;
		data.prefix = "PCIe Width: ";
		data.suffix = "";
	}
	else if (strcmp(cmd, "tgp-limit") == 0)
	{
		data.opcode = NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA;
		data.arg1 = NV_MSGBOX_CMD_ARG1_TGP_LIMIT_V1;
		data.type_size = NV_MSGBOX_SYSID_DATA_SIZE_TGP_LIMIT_V1;
		data.output = output_dec_num;
		data.prefix = "TGP Limit: ";
		data.suffix = " mW";
	}
	else if (strcmp(cmd, "fru-pn") == 0)
	{
		data.opcode = NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA;
		data.arg1 = NV_MSGBOX_CMD_ARG1_FRU_PART_NUMBER_V1;
		data.type_size = NV_MSGBOX_SYSID_DATA_SIZE_FRU_PART_NUMBER_V1;
		data.output = output_bytes;
		data.prefix = "FRU Part Number: ";
		data.suffix = "";
	}
	else if (strcmp(cmd, "max-dram-capacity") == 0)
	{
		data.opcode = NV_MSGBOX_CMD_OPCODE_GET_SYS_ID_DATA;
		data.arg1 = NV_MSGBOX_CMD_ARG1_MAX_DRAM_CAPACITY_V1;
		data.type_size = NV_MSGBOX_SYSID_DATA_SIZE_MAX_DRAM_CAPACITY_V1;
		data.output = output_dec_num;
		data.prefix = "Max DRAM Capacity: ";
		data.suffix = " Mb";
	} // SENSORS
	else if (strcmp(cmd, "temp") == 0)
	{
		data.opcode = NV_MSGBOX_CMD_OPCODE_GET_EXT_TEMP;
		data.arg1 = NV_MSGBOX_CMD_ARG1_NULL;
		data.type_size = NV_MSGBOX_SYSID_DATA_SIZE_MAX_DRAM_CAPACITY_V1;
		data.output = output_24i8f;
		data.prefix = "Temperature: ";
		data.suffix = " C";
	}
	else if (strcmp(cmd, "power") == 0)
	{
		data.opcode = NV_MSGBOX_CMD_OPCODE_GET_POWER;
		data.arg1 = NV_MSGBOX_CMD_ARG1_NULL;
		data.type_size = NV_MSGBOX_SYSID_DATA_SIZE_MAX_DRAM_CAPACITY_V1;
		data.output = output_dec_num;
		data.prefix = "Power: ";
		data.suffix = " mW";
	} // PCIE_LINK_INFO
	else if (strcmp(cmd, "pcie-link-info0") == 0)
	{
		data.opcode = NV_MSGBOX_CMD_OPCODE_GET_PCIE_LINK_INFO;
		data.arg1 = NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_0;
		data.type_size = DWORD_SZ;
		data.output = output_pcie_link_info0;
		data.prefix = "";
		data.suffix = "";
	}
	else if (strcmp(cmd, "pcie-link-info1") == 0)
	{
		data.opcode = NV_MSGBOX_CMD_OPCODE_GET_PCIE_LINK_INFO;
		data.arg1 = NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_1;
		data.type_size = DWORD_SZ;
		data.output = output_pcie_link_info1;
		data.prefix = "";
		data.suffix = "";
	}
	else if (strcmp(cmd, "pcie-link-info2") == 0)
	{
		data.opcode = NV_MSGBOX_CMD_OPCODE_GET_PCIE_LINK_INFO;
		data.arg1 = NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_2;
		data.type_size = DWORD_SZ;
		data.output = output_pcie_link_info2;
		data.prefix = "";
		data.suffix = "";
	}
	else if (strcmp(cmd, "pcie-link-info3") == 0)
	{
		data.opcode = NV_MSGBOX_CMD_OPCODE_GET_PCIE_LINK_INFO;
		data.arg1 = NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_3;
		data.type_size = DWORD_SZ;
		data.output = output_pcie_link_info3;
		data.prefix = "";
		data.suffix = "";
	}
	else if (strcmp(cmd, "pcie-link-info4") == 0)
	{
		data.opcode = NV_MSGBOX_CMD_OPCODE_GET_PCIE_LINK_INFO;
		data.arg1 = NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_4;
		data.type_size = DWORD_SZ;
		data.output = output_pcie_link_info4;
		data.prefix = "";
		data.suffix = "";
	}
	else if (strcmp(cmd, "pcie-link-info6") == 0)
	{
		data.opcode = NV_MSGBOX_CMD_OPCODE_GET_PCIE_LINK_INFO;
		data.arg1 = NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_6;
		data.type_size = DWORD_SZ;
		data.output = output_pcie_link_info6;
		data.prefix = "";
		data.suffix = "";
	}
	else if (strcmp(cmd, "pcie-link-info8") == 0)
	{
		data.opcode = NV_MSGBOX_CMD_OPCODE_GET_PCIE_LINK_INFO;
		data.arg1 = NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_8;
		data.type_size = DWORD_SZ;
		data.output = output_pcie_link_info8;
		data.prefix = "";
		data.suffix = "";
	}
	else if (strcmp(cmd, "pcie-link-info9") == 0)
	{
		data.opcode = NV_MSGBOX_CMD_OPCODE_GET_PCIE_LINK_INFO;
		data.arg1 = NV_MSGBOX_CMD_ARG1_GET_PCIE_LINK_INFO_PAGE_9;
		data.type_size = DWORD_SZ;
		data.output = output_pcie_link_info9;
		data.prefix = "";
		data.suffix = "";
	}

	return data;
}

int get_i2c_bus_id(uint64_t mask, int num)
{
	for (int i = 0, k = 0; i < sizeof(mask) * 8; i++) 
	{
		if ((1ULL << i) & mask)
		{
			if (k == num)	
				return i;
			k++;	
		}
	}
	return -1;
}

int main(int argc, char* argv[])
{
	int do_detect = 0;
	int do_pure = 0;
	int do_info = 0;
	int do_sensors = 0;

	int do_cmd = 0;
	char cmd_str[16] = {};
	int do_raw = 0;

	int ord = 0;

	int opt = 0;
	struct option long_options[] = 
	{
		{"detect", no_argument, 0, 'd'},
		{"info", no_argument, 0, 'i'},
		{"sensors", no_argument, 0, 's'},
		{"pure", no_argument, 0, 'p'},
		{"cmd", required_argument, 0, 'c'},
		{"raw", no_argument, 0, 'r'},
		{"ord", required_argument, 0, 'o'},
		{"help", no_argument, 0, 'h'},
		{0, 0, 0, 0,}
	};

	while ((opt = getopt_long(argc, argv, "dispc:ro:", long_options, NULL))  != -1)
	{
		switch (opt) 
		{
			case 'd':
				do_detect = 1;
				break;
			case 'i':
				do_info = 1;
				break;
			case 's':
				do_sensors = 1;
				break;
			case 'p':
				do_pure = 1;
				break;
			case 'c':
				do_cmd = 1;

				if (strlen(optarg) >= sizeof(cmd_str))
					exit(1);

				strcpy(cmd_str, optarg);

				break;
			case 'r':
				do_raw = 1;
				break;
			case 'o':
				ord = strtol(optarg, NULL, 10);	
				break;
			case 'h':
				usage(argv[0]);
				break;
		}
	}

	uint64_t nvidia_mask = 0;
	if (do_detect || do_info || do_sensors || do_cmd)
	{
		nvidia_mask = detect_nvidia_device();
	}

	if (do_detect)
	{
		print_detect(nvidia_mask, do_pure);
		exit(0);
	}

	if (do_info || do_sensors) 
	{	
		int bus = get_i2c_bus_id(nvidia_mask, ord);
		if (bus == -1)
		{
			fprintf(stderr, "Device with index %d do not found\n", ord);
			exit(1);
		}

		if (do_info)
			print_sysinfo(bus);
		if (do_sensors)
			print_sensors(bus);

		exit(0);
	}

	if (do_cmd) 
	{
		int bus_id = get_i2c_bus_id(nvidia_mask, ord);
		if (bus_id == -1)
		{
			fprintf(stderr, "Device with index %d do not found\n", ord);
			exit(1);
		}
	
		struct CmdData cmd_data = get_cmd_by_id(cmd_str);
		if (cmd_data.opcode == 0x0) 
		{
			fprintf(stderr, "Invalid command\n");
			exit(1);
		}

		cmd(bus_id, cmd_data.opcode, cmd_data.arg1, cmd_data.type_size, get_buf_ptr(1));
		(cmd_data.output)(cmd_data.prefix, cmd_data.suffix, do_pure, do_raw, cmd_data.type_size);

		exit(0);
	}

	usage(argv[0]);

	return 0;
}

void usage(const char *progname)
{
	printf("Usage: %s [OPTIONS]\n", progname);
	printf("NVIDIA GPU SMBPBI Monitor Tool\n\n");
	printf("Options:\n");
	printf("  -d, --detect              Detect NVIDIA GPUs and show count\n");
	printf("  -i, --info                Show info about specified GPU\n");
	printf("  -s, --sensors             Show sensors info\n");
	printf("  -p, --pure                Output only numeric value (for scripting)\n");
	printf("  -c, --cmd CMD             Execute one-shot command\n");
	printf("  -r, --raw                 Output in raw fomat\n");
	printf("  -o, --ord N               GPU index (0-based) for command\n");
	printf("  -h, --help                Show this help\n\n");
	printf("Commands for --cmd:\n");
	printf("SysInfo commands\n");
	printf("  vbios-ver                 Read VBIOS version\n");
	printf("  board-pn                  Board part number\n");
	printf("  sn                 	    Serial Number\n");
	printf("  marketing-name            Marketing name\n");
	printf("  gpu-pn                    GPU part number\n");
	printf("  build-date                VBIOS fw build date\n");
	printf("  vendor-id                 VendorID\n");
	printf("  dev-id                    DeviceID\n");
	printf("  subvendor-id              SubvendorId\n");
	printf("  sub-id                    SubID\n");
	printf("  gpu-guid                  GPU GUID\n");
	printf("  inforom-ver               InfoROM version\n");
	printf("  pcie-speed                PCIe speed\n");
	printf("  pcie-width                PCIe width\n");
	printf("  tgp-limit                 TGP limit\n");
	printf("  fru-pn                    FRU part number\n");
	printf("  max-dram-capacity         Max DRAM capacity\n");
	printf("Sensor commands\n");
	printf("  temperature               GPU temperature\n");
	printf("  power         	    GPU power\n");
	printf("PCIe link info commands\n");
	printf("  pcie-link-info0           Link speed/width, error counters\n");
	printf("  pcie-link-info1           L0 to Recovery count\n");
	printf("  pcie-link-info2           Replay/NAK counters\n");
	printf("  pcie-link-info3           Target link speed\n");
	printf("  pcie-link-info4           TX packet count\n");
	printf("  pcie-link-info6           LTSSM current state\n");
	printf("  pcie-link-info8           TX equalization\n");
	printf("  pcie-link-info9           RX equalization\n");
	printf("Examples:\n");
	printf("  %s --detect\n", progname);
	printf("  %s --detect --pure\n", progname);
	printf("  %s --cmd vbios -o 0\n", progname);
	printf("  %s -c vbios -p\n", progname);
	printf("  %s -c vbios -o 0\n", progname);
	printf("  %s -c vbios-ver --raw --pure\n", progname);
	printf("  %s --sensors\n", progname);
	printf("  %s --info -s\n", progname);
}
