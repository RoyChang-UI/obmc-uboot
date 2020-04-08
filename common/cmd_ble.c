#include <common.h>
#include <command.h>
#include <watchdog.h>
#include <asm/io.h>
#include <asm-generic/gpio.h>


/* in fwupdate.c , return SSID and get ath0 MAC address and bluetooth MAC address */
//Roy_temp unsigned int get_ame_board_mac(uint8_t *eth0_hwaddr, uint8_t *ble0_hwaddr);

//#define CONFIG_BLE_DEBUG /* enable this falg to hexdump and print received data step by step */

/* See  'Advertising_Type' (Size: 1 Octet) in BLE Advertising Parameters HCI command :
 * . To transmit '0x00 Connectable undirected advertising (ADV_IND)' packets if 'CONFIG_ADVERTISE_CONNECTABLE_SERVICE' flag is defined 
 * . To transmit '0x02 Scannable undirected advertising (ADV_SCAN_IND)' packets if 'CONFIG_ADVERTISE_CONNECTABLE_SERVICE' flag is not defined 
 */
//#define CONFIG_ADVERTISE_CONNECTABLE_SERVICE

/* the GPIO2 pin which is used to reset bluetooth chip */
#define CONFIG_BLE_RESET_PIN 2

#ifdef CONFIG_BLE_DEBUG
	#define BLE_DEBUG(format, args...) printf("[%s:%d] "format, __FILE__, __LINE__, ##args)
	#define BLE_HEXDUMP(buf,size) ble_hexdump(buf, size)
	#define BLE_HEXDUMP_RECV_DATA(x) ble_hexdump_recv_data(x)
#else
	#define BLE_DEBUG(args...) do{}while(0)
	#define BLE_HEXDUMP(buf,size) do{}while(0)
	#define BLE_HEXDUMP_RECV_DATA(x) do{}while(0)
#endif

#define BD_ADDR_0_OFFSET_IN_BCCMD_SET_BD_ADDR_CMD            24
#define BD_ADDR_1_OFFSET_IN_BCCMD_SET_BD_ADDR_CMD            23
#define BD_ADDR_2_OFFSET_IN_BCCMD_SET_BD_ADDR_CMD            21
#define BD_ADDR_3_OFFSET_IN_BCCMD_SET_BD_ADDR_CMD            17
#define BD_ADDR_4_OFFSET_IN_BCCMD_SET_BD_ADDR_CMD            20
#define BD_ADDR_5_OFFSET_IN_BCCMD_SET_BD_ADDR_CMD            19

#define BLE_UUID_OFFSET_IN_ADVERTISING_DATA                   6
#define BLE_SERIAL_NUMBER_STR_IN_ADVERTISING_DATA            26
#define BLE_SHORTENED_LOCAL_NAME_OFFSET_IN_SCAN_RESPONSE_DATA 3
#define BLE_MAC_SERVICE_OFFSET_IN_SCAN_RESPONSE_DATA         10

// LTU-Wave's Bluetooth UUID
//.uuid_factory_default = "7e9c3518-4ac2-4352-ab07-a3e8a5752324",
//.uuid_user_configured = "b0e402c7-143f-4486-b371-20405c8b14a2",
static const uint8_t LTU_WAVE_UNCONFIGURED_UUID[]= {0x24,0x23,0x75,0xa5,0xe8,0xa3,0x07,0xab,0x52,0x43,0xc2,0x4a,0x18,0x35,0x9c,0x7e};
static const uint8_t LTU_WAVE_CONFIGURED_UUID[]=   {0xa2,0x14,0x8b,0x5c,0x40,0x20,0x71,0xb3,0x86,0x44,0x3f,0x14,0xc7,0x02,0xe4,0xb0};

typedef struct BLE_hci_command {
	const uint8_t opcode[2];
	const uint8_t size;
	uint8_t command[48];
} ble_hci_command_t;

// for example, BLE0=7E:83:C2:9F:95:3A
uint8_t ble_eth_mac[6] = { 0x7E, 0x83, 0xC2, 0x9F, 0x95, 0x3A };
// MAC0 (ath0) address in EEPROM, target baord's MAC Address, which will be set to the uuid 0x252a service data 'Serial Number String' of advertising data
uint8_t ath0_eth_mac[6] = { 0x74, 0x83, 0xC2, 0x9F, 0x95, 0x3A };
/* Reliable packet sequence number - used to assign seq to each rel pkt. */
static uint8_t txseq = 0;
static uint8_t rxseq_txack = 0; /* rxseq == txack. */

const u8 byte_rev_table[256] = {
	0x00, 0x80, 0x40, 0xc0, 0x20, 0xa0, 0x60, 0xe0,
	0x10, 0x90, 0x50, 0xd0, 0x30, 0xb0, 0x70, 0xf0,
	0x08, 0x88, 0x48, 0xc8, 0x28, 0xa8, 0x68, 0xe8,
	0x18, 0x98, 0x58, 0xd8, 0x38, 0xb8, 0x78, 0xf8,
	0x04, 0x84, 0x44, 0xc4, 0x24, 0xa4, 0x64, 0xe4,
	0x14, 0x94, 0x54, 0xd4, 0x34, 0xb4, 0x74, 0xf4,
	0x0c, 0x8c, 0x4c, 0xcc, 0x2c, 0xac, 0x6c, 0xec,
	0x1c, 0x9c, 0x5c, 0xdc, 0x3c, 0xbc, 0x7c, 0xfc,
	0x02, 0x82, 0x42, 0xc2, 0x22, 0xa2, 0x62, 0xe2,
	0x12, 0x92, 0x52, 0xd2, 0x32, 0xb2, 0x72, 0xf2,
	0x0a, 0x8a, 0x4a, 0xca, 0x2a, 0xaa, 0x6a, 0xea,
	0x1a, 0x9a, 0x5a, 0xda, 0x3a, 0xba, 0x7a, 0xfa,
	0x06, 0x86, 0x46, 0xc6, 0x26, 0xa6, 0x66, 0xe6,
	0x16, 0x96, 0x56, 0xd6, 0x36, 0xb6, 0x76, 0xf6,
	0x0e, 0x8e, 0x4e, 0xce, 0x2e, 0xae, 0x6e, 0xee,
	0x1e, 0x9e, 0x5e, 0xde, 0x3e, 0xbe, 0x7e, 0xfe,
	0x01, 0x81, 0x41, 0xc1, 0x21, 0xa1, 0x61, 0xe1,
	0x11, 0x91, 0x51, 0xd1, 0x31, 0xb1, 0x71, 0xf1,
	0x09, 0x89, 0x49, 0xc9, 0x29, 0xa9, 0x69, 0xe9,
	0x19, 0x99, 0x59, 0xd9, 0x39, 0xb9, 0x79, 0xf9,
	0x05, 0x85, 0x45, 0xc5, 0x25, 0xa5, 0x65, 0xe5,
	0x15, 0x95, 0x55, 0xd5, 0x35, 0xb5, 0x75, 0xf5,
	0x0d, 0x8d, 0x4d, 0xcd, 0x2d, 0xad, 0x6d, 0xed,
	0x1d, 0x9d, 0x5d, 0xdd, 0x3d, 0xbd, 0x7d, 0xfd,
	0x03, 0x83, 0x43, 0xc3, 0x23, 0xa3, 0x63, 0xe3,
	0x13, 0x93, 0x53, 0xd3, 0x33, 0xb3, 0x73, 0xf3,
	0x0b, 0x8b, 0x4b, 0xcb, 0x2b, 0xab, 0x6b, 0xeb,
	0x1b, 0x9b, 0x5b, 0xdb, 0x3b, 0xbb, 0x7b, 0xfb,
	0x07, 0x87, 0x47, 0xc7, 0x27, 0xa7, 0x67, 0xe7,
	0x17, 0x97, 0x57, 0xd7, 0x37, 0xb7, 0x77, 0xf7,
	0x0f, 0x8f, 0x4f, 0xcf, 0x2f, 0xaf, 0x6f, 0xef,
	0x1f, 0x9f, 0x5f, 0xdf, 0x3f, 0xbf, 0x7f, 0xff,
};

#ifdef CONFIG_BLE_DEBUG
/**
 *
 * Print on stdout a nice hexa dump of a buffer
 * @param buf buffer
 * @param size buffer size
 */
static void ble_hexdump(const unsigned char *buf, size_t size)
{
	int len, i, j, c;
	
	printf("==== starting from 0x%x,buf size %d(0x%x) ====\n", (unsigned int)buf,size,size);
	for(i=0;i<size;i+=16) {
		len = size - i;
		if (len > 16)
			len = 16;
		printf("%08x ", i);
		for(j=0;j<16;j++) {
			if (j < len)
				printf(" %02x", buf[i+j]);
			else
				printf("   ");
		}
		printf(" ");
		for(j=0;j<len;j++) {
			c = buf[i+j];
			if (c < ' ' || c > '~')
				c = '.';
			printf("%c", c);
		}
		printf("\n");
	}
}

uint8_t print_buf[2048];
static void ble_hexdump_recv_data(int maxcount)
{
	int cnt=0;
	uint8_t *p = &(print_buf[0]);

	if(maxcount > sizeof(print_buf)) {
		BLE_DEBUG("the count of characters to be printed exceed maximum limit %d\n", sizeof(print_buf));
		return;
	}

	while(cnt < maxcount)
	{
		p[cnt]=(uint8_t) ble_serial_getc();
		cnt++;
	}

	BLE_DEBUG("hexdump received data:\n ");
	BLE_HEXDUMP(&(print_buf[0]), maxcount);
}
#endif

static uint8_t bitrev8(uint8_t byte)
{
	return byte_rev_table[byte];
}

static uint16_t bitrev16(uint16_t x)
{
	return (bitrev8(x & 0xff) << 8) | bitrev8(x >> 8);
}

/* ---- BCSP CRC calculation ---- */
/* Table for calculating CRC for polynomial 0x1021, LSB processed first,
initial value 0xffff, bits shifted in reverse order. */
static const uint16_t crc_table[] = {
	0x0000, 0x1081, 0x2102, 0x3183,
	0x4204, 0x5285, 0x6306, 0x7387,
	0x8408, 0x9489, 0xa50a, 0xb58b,
	0xc60c, 0xd68d, 0xe70e, 0xf78f
};
/* Initialise the crc calculator */
#define BCSP_CRC_INIT(x) x = 0xffff
/*
   Update crc with next data byte
   Implementation note
        The data byte is treated as two nibbles.  The crc is generated
        in reverse, i.e., bits are fed into the register from the top.
*/
static void bcsp_crc_update(uint16_t *crc, uint8_t d)
{
	uint16_t reg = *crc;
	reg = (reg >> 4) ^ crc_table[(reg ^ d) & 0x000f];
	reg = (reg >> 4) ^ crc_table[(reg ^ (d >> 4)) & 0x000f];
	*crc = reg;
}

static uint8_t * bcsp_slip_msgdelim(uint8_t *data, int *len)
{
	const char pkt_delim = 0xc0;
	memcpy(data, &pkt_delim, 1);
	*len+=1;
	return (data+1);
}

static uint8_t * bcsp_slip_one_byte(uint8_t *data, uint8_t c, int *len)
{
	const char esc_c0[2] = { 0xdb, 0xdc };
	const char esc_db[2] = { 0xdb, 0xdd };
	switch (c) {
	case 0xc0:
		memcpy(data, &esc_c0, 2);
		*len+=2;
		return (data+2);
		break;
	case 0xdb:
		memcpy(data, &esc_db, 2);
		*len+=2;
		return (data+2);
		break;
	default:
		memcpy(data, &c, 1);
		*len+=1;
		return (data+1);
	}
}



static void ble_gpio_reset(void)
{
	printf("[%s] Rest Bluetooth device with GPIO\n", __func__);

//	gpio_direction_output(CONFIG_BLE_RESET_PIN, 0);
	mdelay(100);
//	gpio_set_value(CONFIG_BLE_RESET_PIN, 1);
//	gpio_free(CONFIG_BLE_RESET_PIN);
	mdelay(200);
}

static void ble_bcsp_initialize(void)
{
	printf("[%s] BCSP Initialization\n", __func__);
	uint8_t bcsp_sync_pkt[10] = {0xc0,0x00,0x41,0x00,0xbe,0xda,0xdc,0xed,0xed,0xc0};
	uint8_t bcsp_conf_pkt[10] = {0xc0,0x00,0x41,0x00,0xbe,0xad,0xef,0xac,0xed,0xc0};
	uint8_t bcsp_sync_resp_pkt[10] = {0xc0,0x00,0x41,0x00,0xbe,0xac,0xaf,0xef,0xee,0xc0};
	uint8_t bcsp_conf_resp_pkt[10] = {0xc0,0x00,0x41,0x00,0xbe,0xde,0xad,0xd0,0xd0,0xc0};
	int i, j;

	// reset global seq number and ack number
	txseq = 0;
	rxseq_txack = 0; /* rxseq == txack. */

	// kick watchdog , LTU got around 8 seconds for hardware watchdog timeout
	WATCHDOG_RESET();
	
	BLE_DEBUG("### BCSP link establishment\n");
	for(j=0; j<20; j++) // CSR8811 got auto uart baud rate detection
	{
		//BLE_DEBUG("Start to send sync packet\n");
		mdelay(5);
		for(i=0; i<sizeof(bcsp_sync_pkt); i++) {
			ble_serial_putc(bcsp_sync_pkt[i]);
		}
	}

	/* sync packet received mostly : bcspsync[4] = {0xda, 0xdc, 0xed, 0xed};
	   few sync responses received :  bcspsyncresp[4] = {0xac,0xaf,0xef,0xee};
	*/

	// let's wait for 160 characters output, 
	// 160 bytes * 11 (1 start bit + 8 data bits + 1 even parity bit + 1 stop bit)= 1760 bits
	// if 921600 bps is used, 1760 / 921600 bps = 0.001909722 = 1.9 ms
	// if 115200 bps is used, 1760 / 115200 bps = 0.0153      = 15.3 ms
	mdelay(15); // for 115200 bps, passed
	BLE_HEXDUMP_RECV_DATA(400);

	BLE_DEBUG("### Start to send sync resp packet\n");
	for(i=0; i<sizeof(bcsp_sync_resp_pkt); i++) {
		ble_serial_putc(bcsp_sync_resp_pkt[i]);
	}


	/* conf packets received mostly : bcspconf[4] = {0xad,0xef,0xac,0xed};
	   very very few sync packets received : bcspsync[4] = {0xda, 0xdc, 0xed, 0xed};
	 */
	mdelay(15); // for 115200 bps
	BLE_HEXDUMP_RECV_DATA(400);

	BLE_DEBUG("### State is curious\n");
	/* State = curious */

	BLE_DEBUG("### Start to send conf packet\n");
	for(i=0; i<sizeof(bcsp_conf_pkt); i++) {
		ble_serial_putc(bcsp_conf_pkt[i]);
	}

	mdelay(15); // for 115200 bps
	/* conf packets received mostly : bcspconf[4] = {0xad,0xef,0xac,0xed}; */
	/* one conf resp packets received : bcspconfresp[4] = {0xde,0xad,0xd0,0xd0}; */
	BLE_HEXDUMP_RECV_DATA(400);

	BLE_DEBUG("### Start to send conf response packet\n");
	for(i=0; i<sizeof(bcsp_conf_resp_pkt); i++) {
		ble_serial_putc(bcsp_conf_resp_pkt[i]);
	}

	mdelay(15); // for 115200 bps
	/* continue to send the following characters 
	   0xc0 0xdb 0xdc 0x65 0x00 0xda 0x0f 0x04 0x00 0x01 0x00 0x00 0x11 0xda 0xc0 
	   0xc0 0xdb 0xdc 0x65 0x00 0xda 0x0f 0x04 0x00 0x01 0x00 0x00 0x11 0xda 0xc0 
	   0xc0 0xdb 0xdc 0x65 0x00 0xda 0x0f 0x04 0x00 0x01 0x00 0x00 0x11 0xda 0xc0 
	   ....
	*/
	BLE_HEXDUMP_RECV_DATA(400);

	/* State = garrulous */
	BLE_DEBUG("### State is garrulous\n");
	printf("[%s] BCSP Initialization done.\n", __func__);
}

static void ble_bcsp_send_hci_packet(ble_hci_command_t *cmd, int use_crc, int print_recv_bytes)
{
	// reserved for about double of BLE_hci_command ~= (2+1+48)*2 bytes because of slip bytes
	uint8_t bcsp_data[128]; 
	uint8_t hdr[4];
	uint16_t BCSP_CRC_INIT(bcsp_txmsg_crc);
	uint8_t chan = 5;   /* BCSP cmd/evt channel */
	int rel = 1;        /* reliable channel */
	int i;
	int len=(sizeof(cmd->opcode) + sizeof(cmd->size) + cmd->size);
	int bcsp_len=0;
	uint8_t *p=&(bcsp_data[0]);
	uint8_t *data=(uint8_t *)(cmd);

	p = bcsp_slip_msgdelim(p, &bcsp_len);

	hdr[0] = rxseq_txack << 3;
	rxseq_txack = (rxseq_txack+1) & 0x07;
	if (rel) {
		hdr[0] |= 0x80 + txseq;
		txseq = (txseq + 1) & 0x07;
	}
	if (use_crc)
		hdr[0] |= 0x40;
	hdr[1] = ((len << 4) & 0xff) | chan;
	hdr[2] = len >> 4;
	hdr[3] = ~(hdr[0] + hdr[1] + hdr[2]);

	/* Put BCSP header */
	for (i = 0; i < 4; i++) {
		p = bcsp_slip_one_byte(p, hdr[i], &bcsp_len);
		if (use_crc)
			bcsp_crc_update(&bcsp_txmsg_crc, hdr[i]);
	}

	/* Put payload */
	for (i = 0; i < len; i++) {
		p = bcsp_slip_one_byte(p, data[i], &bcsp_len);
		if (use_crc)
			bcsp_crc_update(&bcsp_txmsg_crc, data[i]);
	}

	/* Put CRC */
	if (use_crc) {
		bcsp_txmsg_crc = bitrev16(bcsp_txmsg_crc);
		p = bcsp_slip_one_byte(p, (uint8_t) ((bcsp_txmsg_crc >> 8) & 0x00ff), &bcsp_len);
		p = bcsp_slip_one_byte(p, (uint8_t) (bcsp_txmsg_crc & 0x00ff), &bcsp_len);
	}
	p = bcsp_slip_msgdelim(p, &bcsp_len);

	BLE_DEBUG("Start to send hci commands packet: BCSP packet len=%d\n", bcsp_len);
	BLE_HEXDUMP(&bcsp_data[0],bcsp_len);

	for(i=0;i<bcsp_len;i++)
		ble_serial_putc(bcsp_data[i]);

	if(print_recv_bytes) {
		BLE_HEXDUMP_RECV_DATA(400);
	}
}

static void ble_bccmd_set_mac_addr(void)
{
	printf("[%s] BCCMD Set MAC Address\n", __func__);
	// send bcsp packet with verdor-specific HCI commands like the following :
	// bccmd -t bcsp -b 115200 -d /dev/ttyAMA1 psset -r 0x01 0x9F 0x00 0x8E 0x95 0xC2 0x00 0x83 0x7E

	//transport_read CSR_VARID_PS_SIZE=0x00003006
	ble_hci_command_t read_ps_size_cmd = {
		.opcode= {0x00,0xfc},
		.size = 0x13, 
		.command = {0xc2,0x00,0x00,0x09,0x00,0x00,0x00,0x06,0x30,0x00,0x00,0x01,0x00,0x08,0x00,0x00,0x00,0x00,0x00}
	};
	/* Write newM BD addr: MAC[0]=command[24], MAC[1]=command[23], MAC[2]=command[21], MAC[3]=command[17], MAC[4]=command[20], MAC[5]=command[19]*/
	// transport_write CSR_VARID_PS=0x00007003 , e.g. : write new BD MAC address BLE0=7E:83:C2:9F:95:3A , its data look like :
	//                  0    1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18   19   20   21   22   23   24
	// ===========================================================================================================================================
	// 0x00,0xfc,0x19,0xc2,0x02,0x00,0x0c,0x00,0x01,0x00,0x03,0x70,0x00,0x00,0x01,0x00,0x04,0x00,0x08,0x00,0x9f,0x00,0x3a,0x95,0xc2,0x00,0x83,0x7e
	//                                                                                                                                        ---- MAC[0]=0x7E
	//                                                                                                                                   ---- MAC[1]=0x83
	//                                                                                                                         ---- MAC[2]=0xC2
	//                                                                                                     ---- MAC[3]=0x9F
	//                                                                                                                    ---- MAC[4]=0x95
	//                                                                                                               ---- MAC[5]=0x3A
	ble_hci_command_t write_ps_cmd = {
		.opcode= {0x0,0xfc}, 
		.size = 0x19,
		.command = { 0xc2,0x02,0x00,0x0c,0x00,0x01,0x00,0x03,0x70,0x00,0x00,0x01,0x00,0x04,0x00,0x08, \
		             0x00,0x9f,0x00,0x3a,0x95,0xc2,0x00,0x83,0x7e}
		// original Jame's code 
		//.command = {0xc2,0x02,0x00,0x0c,0x00,0x01,0x00,0x03,0x70,0x00,0x00,0x01,0x00,0x04,0x00,0x08,0x00,0x33,0x00,0x55,0x44,0x22,0x00,0x11,0x05}
	};
	// transport_write CSR_VARID_WARM_RESET=0x00004002 , do warm reset
	ble_hci_command_t write_warm_reset_cmd = {
		.opcode= {0x0,0xfc},
		.size = 0x13,
		.command = { 0xc2,0x02,0x00,0x09,0x00,0x02,0x00,0x02,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00, \
		             0x00,0x00,0x00}
		// original Jame's code
		//{.opcode= {0x0,0xfc}, .size = 0x13, .command = {0xc2,0x2,0x0,0x9,0x0,0x2,0x0,0x2,0x40,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0}},
	};

	BLE_DEBUG("### Send 'transport_read CSR_VARID_PS_SIZE'\n");
	ble_bcsp_send_hci_packet(&read_ps_size_cmd, 1, 1);

	// based on 'csr_bcsp.c' dumped log, we receive one more reliable packet, 
	// add one to 'rxseq_txack' to avoid adding uart receiving code
	rxseq_txack = (rxseq_txack+1) & 0x07;

	write_ps_cmd.command[BD_ADDR_0_OFFSET_IN_BCCMD_SET_BD_ADDR_CMD] = ble_eth_mac[0];
	write_ps_cmd.command[BD_ADDR_1_OFFSET_IN_BCCMD_SET_BD_ADDR_CMD] = ble_eth_mac[1];
	write_ps_cmd.command[BD_ADDR_2_OFFSET_IN_BCCMD_SET_BD_ADDR_CMD] = ble_eth_mac[2];
	write_ps_cmd.command[BD_ADDR_3_OFFSET_IN_BCCMD_SET_BD_ADDR_CMD] = ble_eth_mac[3];
	write_ps_cmd.command[BD_ADDR_4_OFFSET_IN_BCCMD_SET_BD_ADDR_CMD] = ble_eth_mac[4];
	write_ps_cmd.command[BD_ADDR_5_OFFSET_IN_BCCMD_SET_BD_ADDR_CMD] = ble_eth_mac[5];
	BLE_DEBUG("### Send 'transport_write CSR_VARID_PS'\n");
	ble_bcsp_send_hci_packet(&write_ps_cmd, 1, 1);

	BLE_DEBUG("### Send 'transport_write CSR_VARID_WARM_RESET'\n");
	ble_bcsp_send_hci_packet(&write_warm_reset_cmd, 1, 0);
	printf("[%s] BCCMD Set MAC Address done.\n", __func__);
}

static void ble_csr8811_start_adv(void)
{
	printf("[%s] Start Advertising\n", __func__);
	char buf[32];
	/* sequences for program 'ubnt-ble-http-transport' to issue advertising packets  are :
	 *  OGF=8 OCF=8  OPCODE=0x2008 LE_Set_Advertising_Data
	 *  OGF=8 OCF=9  OPCODE=0x2009 LE_Set_Scan_Responce_Data
	 *  OGF=8 OCF=6  OPCODE=0x2006 LE_Set_Advertising_Parameters
	 *  OGF=8 OCF=10 OPCODE=0x200a LE_Set_Advertise_Enable
	 */
	/*LE_Set_Advertising_Data OGF=8 OCF=8 OPCODE=0x2008 */
	ble_hci_command_t adv_data_cmd = {
		.opcode= {0x8,0x20},		
		// original James's coomand
		// .size = 0x20,
		// .command = {0x1d,0x2,0x1,0x6,0x11,0x6,0x10,0xba,0x32,0x1a,0xe3,0x3b,0x47,0x39,0xa8,0x2d,0x7e,0x87,0x5c,0x8d,0xed,0x8e,0x6,0x8,0x55,0x46,0x4c,0x48,0x44,0x0,0x0,0x0}
		//
		// dumped from LTU-Wave board :
		// [   26.190964] enqueue:00000000: 08 20 20 1c 11 06 e4 9d 56 91 65 58 77 b3 ca 45  .  .....V.eXw..E
		// [   26.190975] enqueue:00000010: 66 53 82 9c 4b a3 09 16 2a 25 74 83 c2 9f 95 3a  fS..K...*%t....:
		// [   26.190983] enqueue:00000020: 00 00 76                                         ..v
		//
		// add flag '02 01 06'  to the dumped command from LTU-Wave board
		//           -------- AD Type 0x01 means flags, flag value = 0x06 = 0000_0110
		//                    Bit 0 – Indicates LE Limited Discoverable Mode
		//                    Bit 1 – Indicates LE General Discoverable Mode
		//                    Bit 2 – Indicates whether BR/EDR is supported. This is used if your iBeacon is Dual Mode device
		//                    Bit 3 – Indicates whether LE and BR/EDR Controller operates simultaneously
		//                    Bit 4 – Indicates whether LE and BR/EDR Host operates simultaneously
		.size = 0x20,
		.command = { 0x1f,0x02,0x01,0x02,0x11,0x06,0xe4,0x9d,0x56,0x91,0x65,0x58,0x77,0xb3,0xca,0x45, \
		             0x66,0x53,0x82,0x9c,0x4b,0xa3,0x09,0x16,0x2a,0x25,0x74,0x83,0xc2,0x9f,0x95,0x3a, \
		            }
	};
	/*LE_Set_Scan_Responce_Data OGF=8 OCF=9 OPCODE=0x2009 */
	ble_hci_command_t scan_resp_data_cmd = {
		.opcode= {0x09,0x20},
		//
		// original from Jame's command
		// .size = 0x20,
		// .command = {0x1e,0x4,0x16,0x21,0x20,0x0,0x9,0x16,0x2a,0x25,0x18,0xe8,0x29,0xb5,0x12,0xd1,0x6,0x16,0x2b,0x20,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0}
		//
		// dumped from LTU-Wave baord
		// [   26.199484] enqueue:00000000: 09 20 20 11 10 08 4c 54 55 2d 57 61 76 65 2d 39  .  ...LTU-Wave-9
		// [   26.199521] enqueue:00000010: 46 39 35 33 41 00 00 00 00 00 00 00 00 00 00 00  F953A...........
		// [   26.199530] enqueue:00000020: 00 00 00                                         ...
		.size = 0x20,
		.command = {                0x11,0x10,0x08,0x4c,0x54,0x55,0x2d,0x57,0x61,0x76,0x65,0x2d,0x39, \
		             0x46,0x39,0x35,0x33,0x41,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, \
		             0x00,0x00,0x00}
	};
	/*LE_Set_Advertising_Parameters OGF=8 OCF=6 OPCODE=0x2006 */
	ble_hci_command_t adv_param_cmd = {
		.opcode= {0x6,0x20},
		.size = 0xf,
		// dumped from LTU-Wave baord
		// [   26.207876] enqueue:00000000: 06 20 0f a0 00 a0 00 00 00 00 00 00 00 00 00 00  . ..............
		// [   26.207886] enqueue:00000010: 07 00                                            ..
		#ifdef CONFIG_ADVERTISE_CONNECTABLE_SERVICE
			/* Advertising_Type: Size: 1 Octet
			   Value  Parameter       Description
			   0x00   Connectable     undirected advertising (ADV_IND) ( default )
			   0x01   Connectable     directed advertising (ADV_DIRECT_IND)
			   0x02   Scannable       undirected advertising (ADV_SCAN_IND)
			   0x03   Non connectable undirected advertising (ADV_NONCONN_IND)
			   0x04 – 0xFF Reserved for future use
			 */
			.command = {0xa0,0x00,0xa0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x00}
			//                                ^
			//                                |
			//                                +-- Advertising_Type
		#else
			.command = {0xa0,0x00,0xa0,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x00}
		#endif
	};
	/*LE_Set_Advertise_Enable OGF=8 OCF=10 OPCODE=0x200A */
	ble_hci_command_t adv_enable_cmd = {
		.opcode= {0xa,0x20},
		.size = 0x1,
		.command = {0x1}
	};

	BLE_DEBUG("### Send 'LE_Set_Advertising_Data'\n");
	// After sync with other teams, it is decided that, in U-Boot, let BLE transmit advertising packets in non-connectable mode with additional information 
	// via scan responses (ADV_SCAN_IND), and if 'configured UUID' or 'factory UUID' can not be determined in U-Boot phase, use 'configued UUID' as default
	// UUID in U-Boot. And mobile app can use advertising type to determine it is U-Boot phase (non-connectable) or Linux phase (connectable).
	if(0) { 
		memcpy(&(adv_data_cmd.command[BLE_UUID_OFFSET_IN_ADVERTISING_DATA]), LTU_WAVE_UNCONFIGURED_UUID, sizeof(LTU_WAVE_UNCONFIGURED_UUID));
	} else {
		memcpy(&(adv_data_cmd.command[BLE_UUID_OFFSET_IN_ADVERTISING_DATA]), LTU_WAVE_CONFIGURED_UUID, sizeof(LTU_WAVE_CONFIGURED_UUID));
	}
	// set 'Serial Number String'(0x2a25 service UUID) to ath0 MAC address(MAC0 address in EEPROM)
	memcpy(&(adv_data_cmd.command[BLE_SERIAL_NUMBER_STR_IN_ADVERTISING_DATA]), ath0_eth_mac, sizeof(ath0_eth_mac));
	ble_bcsp_send_hci_packet(&adv_data_cmd, 1, 1);

	BLE_DEBUG("### Send 'LE_Set_Scan_Responce_Data'\n");
	sprintf(buf, "LTU-Wave-%02X%02X%02X", ble_eth_mac[3],ble_eth_mac[4],ble_eth_mac[5]);
	memcpy(&(scan_resp_data_cmd.command[BLE_SHORTENED_LOCAL_NAME_OFFSET_IN_SCAN_RESPONSE_DATA]), buf, 15);
	ble_bcsp_send_hci_packet(&scan_resp_data_cmd, 1, 1);

	BLE_DEBUG("### Send 'LE_Set_Advertising_Parameters'\n");
	ble_bcsp_send_hci_packet(&adv_param_cmd, 1, 1);

	BLE_DEBUG("### Send 'LE_Set_Advertise_Enable'\n");
	ble_bcsp_send_hci_packet(&adv_enable_cmd, 1, 1);
	printf("[%s] Start Advertising done.\n", __func__);
}

static void ble_bccmd_set_26MHz(void)
{
	printf("[%s] Set ANA Frequecy\n", __func__);
	// send bcsp packet with verdor-specific HCI commands to set 25MHz similar to the following command :
	// bccmd -t bcsp -b 115200 -d /dev/ttyAMA1 psload -r /lib/firmware/CSR8811/pb-207-csr8x11-rev4-115200.psr
	//
	// 25MHz freq setting in firmware file
	// &01fe = 6590
	// &03c3 = 0001
	// &2579 = 0000
	// &0246 = 0000
	// &0229 = 0000
	// &0217 = 0000
	// &02fc = 0000
	//
	// Loading PSKEY_ANA_FREQ ... csr_write_bcsp: varid=0x00007003, length=8
	ble_hci_command_t any_freq_cmd = {
		.opcode= {0x00,0xfc},
		.size = 0x13, 
		.command = {0xc2,0x02,0x00,0x09,0x00,0x00,0x00,0x03,0x70,0x00,0x00,0xfe,0x01,0x01,0x00,0x08,0x00,0x90,0x65}
	};
	// Loading PSKEY_DEEP_SLEEP_USE_EXTERNAL_CLOCK ... csr_write_bcsp: varid=0x00007003, length=8
	ble_hci_command_t use_ext_clock_cmd = {
		.opcode= {0x00,0xfc},
		.size = 0x13,
		.command = {0xc2,0x02,0x00,0x09,0x00,0x01,0x00,0x03,0x70,0x00,0x00,0xc3,0x03,0x01,0x00,0x08,0x00,0x01,0x00}
	};
	// Loading 0x2579 ... csr_write_bcsp: varid=0x00007003, length=8
	ble_hci_command_t set_2579_cmd = {
		.opcode= {0x00,0xfc},
		.size = 0x13,
		.command = {0xc2,0x02,0x00,0x09,0x00,0x02,0x00,0x03,0x70,0x00,0x00,0x79,0x25,0x01,0x00,0x08,0x00,0x00,0x00}
	};
	// Loading PSKEY_CLOCK_REQUEST_ENABLE ... csr_write_bcsp: varid=0x00007003, length=8
	ble_hci_command_t clock_request_enable_cmd = {
		.opcode= {0x00,0xfc},
		.size = 0x13,
		.command = {0xc2,0x02,0x00,0x09,0x00,0x03,0x00,0x03,0x70,0x00,0x00,0x46,0x02,0x01,0x00,0x08,0x00,0x00,0x00}
	};
	// Loading PSKEY_DEEP_SLEEP_STATE ... csr_write_bcsp: varid=0x00007003, length=8
	ble_hci_command_t deep_sleep_state_cmd = {
		.opcode= {0x00,0xfc},
		.size = 0x13,
		.command = {0xc2,0x02,0x00,0x09,0x00,0x04,0x00,0x03,0x70,0x00,0x00,0x29,0x02,0x01,0x00,0x08,0x00,0x00,0x00}
	};
	// Loading PSKEY_TX_OFFSET_HALF_MHZ ... csr_write_bcsp: varid=0x00007003, length=8
	ble_hci_command_t tx_offset_half_mhz_cmd = {
		.opcode= {0x00,0xfc},
		.size = 0x13,
		.command = {0xc2,0x02,0x00,0x09,0x00,0x05,0x00,0x03,0x70,0x00,0x00,0x17,0x02,0x01,0x00,0x08,0x00,0x00,0x00}
	};
	// Loading 0x02fc ... csr_write_bcsp: varid=0x00007003, length=8
	ble_hci_command_t set_02fc_cmd = {
		.opcode= {0x00,0xfc},
		.size = 0x13,
		.command = {0xc2,0x02,0x00,0x09,0x00,0x06,0x00,0x03,0x70,0x00,0x00,0xfc,0x02,0x01,0x00,0x08,0x00,0x00,0x00}
	};
	// warm_reset command , csr_write_bcsp: varid=0x00004002, length=0
	ble_hci_command_t warm_reset_cmd = {
		.opcode= {0x00,0xfc},
		.size = 0x13,
		.command = {0xc2,0x02,0x00,0x09,0x00,0x07,0x00,0x02,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}
	};

	BLE_DEBUG("### Send 'csr_write_bcsp PSKEY_ANA_FREQ'\n");
	ble_bcsp_send_hci_packet(&any_freq_cmd, 1, 1);

	BLE_DEBUG("### Send 'csr_write_bcsp PSKEY_DEEP_SLEEP_USE_EXTERNAL_CLOCK'\n");
	ble_bcsp_send_hci_packet(&use_ext_clock_cmd, 1, 1);

	BLE_DEBUG("### Send 'csr_write_bcsp 0x2579'\n");
	ble_bcsp_send_hci_packet(&set_2579_cmd, 1, 1);

	BLE_DEBUG("### Send 'csr_write_bcsp PSKEY_CLOCK_REQUEST_ENABLE'\n");
	ble_bcsp_send_hci_packet(&clock_request_enable_cmd, 1, 1);

	BLE_DEBUG("### Send 'csr_write_bcsp PSKEY_DEEP_SLEEP_STATE'\n");
	ble_bcsp_send_hci_packet(&deep_sleep_state_cmd, 1, 1);

	BLE_DEBUG("### Send 'csr_write_bcsp PSKEY_TX_OFFSET_HALF_MHZ'\n");
	ble_bcsp_send_hci_packet(&tx_offset_half_mhz_cmd, 1, 1);

	BLE_DEBUG("### Send 'csr_write_bcsp 0x02fc'\n");
	ble_bcsp_send_hci_packet(&set_02fc_cmd, 1, 1);

	BLE_DEBUG("### Send 'csr_write_bcsp CSR_VARID_WARM_RESET'\n");
	ble_bcsp_send_hci_packet(&warm_reset_cmd, 1, 0);
	printf("[%s] Set ANA Frequecy done.\n", __func__);
}


static int do_ble(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	unsigned int ssid = 0x5678;

	//ssid = get_ame_board_mac(ath0_eth_mac,ble_eth_mac);
	BLE_DEBUG("SSID = 0x%04x\n", ssid);
	BLE_DEBUG("ath0_eth_mac=%02X%02X%02X%02X%02X%02X\n",ath0_eth_mac[0],ath0_eth_mac[1],ath0_eth_mac[2],ath0_eth_mac[3],ath0_eth_mac[4],ath0_eth_mac[5]);
	BLE_DEBUG("ble_eth_mac=%02X%02X%02X%02X%02X%02X\n",ble_eth_mac[0],ble_eth_mac[1],ble_eth_mac[2],ble_eth_mac[3],ble_eth_mac[4],ble_eth_mac[5]);

	#ifdef CONFIG_ADVERTISE_CONNECTABLE_SERVICE 
		printf("Transmit bluetooth 'ADV_IND' advertising packets\n");
	#else
		printf("Transmit bluetooth 'ADV_SCAN_IND' advertising packets\n");
	#endif

	ble_serial_init();
	ble_serial_setbrg();

	ble_gpio_reset();
	ble_bcsp_initialize();
	ble_bccmd_set_26MHz();

	mdelay(100);
	ble_bcsp_initialize();
	ble_bccmd_set_mac_addr();

	mdelay(100);
	ble_bcsp_initialize();
	ble_csr8811_start_adv();

	return 0;
}

U_BOOT_CMD(
	ble,	CONFIG_SYS_MAXARGS,	1,	do_ble,
	"Transmit bluetooth advertising packets",
	""
);
