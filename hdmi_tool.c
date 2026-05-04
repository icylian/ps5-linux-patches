/*
 * hdmi_tool — userspace driver for the PS5 HDMI converter (MN864739).
 *
 * Sends arbitrary i2c sequences to the chip via /dev/icc using the
 * ICC_QUERY ioctl that we add in our spcie.c patch. Mirrors the i2c_*
 * helper functions from drivers/ps5/hdmi.c so we can replay or modify
 * any sequence at runtime without rebuilding the kernel.
 *
 *   gcc -O2 -Wall -o hdmi_tool hdmi_tool.c
 *   sudo ./hdmi_tool init                          # sceHdmiInitVideoConfig
 *   sudo ./hdmi_tool set_video 1920 1080 60        # sceHdmiSetVideoConfig
 *   sudo ./hdmi_tool set_audio                     # sceHdmiSetAudioConfig
 *   sudo ./hdmi_tool video_mute 0                  # unmute video
 *   sudo ./hdmi_tool audio_mute 0                  # unmute audio
 *   sudo ./hdmi_tool disable_encode                # internal helper
 *   sudo ./hdmi_tool init_isr                      # internal helper
 *   sudo ./hdmi_tool init_dp                       # internal helper
 *   sudo ./hdmi_tool raw <hex bytes...>            # send any byte stream
 *   sudo ./hdmi_tool i2c_write <reg16> <val>       # one-off i2c write
 *   sudo ./hdmi_tool i2c_mask  <reg16> <val> <msk> # one-off i2c mask
 *
 * All commands print the reply as hex when verbose mode is on (-v flag).
 */

#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

/* === ICC framing (must match drivers/ps5/spcie.c) ============== */
#define ICC_MSG_MIN_SIZE   0x20
#define ICC_MSG_MAX_SIZE   0x7f0

struct icc_msg {
	uint8_t  magic;        /* kernel sets 0x42 */
	uint8_t  service_id;
	uint16_t msg_type;
	uint16_t unk_04;       /* kernel sets 3 */
	uint16_t id;           /* kernel sets */
	uint16_t length;
	uint16_t checksum;     /* kernel sets */
	uint8_t  data[];
} __attribute__((packed));

struct icc_xfer {
	uint8_t query[ICC_MSG_MAX_SIZE];
	uint8_t reply[ICC_MSG_MAX_SIZE];
};

#define ICC_IOC_MAGIC  'I'
#define ICC_QUERY      _IOWR(ICC_IOC_MAGIC, 2, struct icc_xfer)

#define ICC_SERVICE_ID_GENERAL  0x02
#define ICC_SERVICE_ID_HDMI     0x10

/* === The "i2c over ICC" framing inside the HDMI service ======== */
struct icc_i2c_msg {
	uint8_t  code;
	uint16_t length;
	uint8_t  count;
	uint8_t  data[];
} __attribute__((packed));

struct i2c_cmd_hdr {
	uint8_t major;
	uint8_t length;
	uint8_t minor;
	uint8_t count;
	uint8_t data[];
} __attribute__((packed));

/* sub-command bodies (mirror hdmi.c) */
struct i2c_cmd_2_1 { uint8_t length; uint8_t reg_high; uint8_t reg_low; uint8_t data[]; } __attribute__((packed));
struct i2c_cmd_write { uint8_t length; uint8_t reg_high; uint8_t reg_low; uint8_t data[]; } __attribute__((packed));
struct i2c_cmd_mask { uint8_t length; uint8_t reg_high; uint8_t reg_low; uint8_t value; uint8_t mask[]; } __attribute__((packed));
struct i2c_cmd_delay { uint8_t length; uint8_t time_low; uint8_t time_high; uint8_t unk_03; } __attribute__((packed));
struct i2c_cmd_waitset { uint8_t length; uint8_t reg_high; uint8_t reg_low; uint8_t value; } __attribute__((packed));
struct i2c_cmd_waitclear { uint8_t length; uint8_t reg_high; uint8_t reg_low; uint8_t value; } __attribute__((packed));
struct i2c_cmd_3_5 { uint8_t length; uint8_t reg_high; uint8_t reg_low; uint8_t value; } __attribute__((packed));
struct i2c_cmd_5_3 { uint8_t value; } __attribute__((packed));
struct i2c_cmd_5_4 { uint8_t value; } __attribute__((packed));
struct i2c_cmd_5_5 { uint8_t value; } __attribute__((packed));

struct i2c_block { uint8_t length; uint16_t reg; uint8_t data[32]; };

/* === Builder context ========================================= */
struct i2c_ctx {
	int   fd;
	int   verbose;
	struct icc_xfer xfer;
	uint8_t *cur;                  /* points into xfer.query past the icc_msg header */
	struct icc_i2c_msg *hdr;       /* the icc_i2c_msg sitting at xfer.query[12] */
	struct i2c_block blocks[128];
	int   block_index;             /* -1 = not in block, >=0 = inside block */
};

static struct i2c_ctx C;

/* === Open / send ============================================ */
static void die(const char *fmt, ...) {
	va_list ap; va_start(ap, fmt);
	vfprintf(stderr, fmt, ap); va_end(ap); fputc('\n', stderr); exit(1);
}

static void ctx_open(void) {
	C.fd = open("/dev/icc", O_RDWR);
	if (C.fd < 0) die("open /dev/icc: %s", strerror(errno));
}

static void hexdump(const char *tag, const uint8_t *p, size_t n) {
	fprintf(stderr, "%s [%zu bytes]:", tag, n);
	for (size_t i = 0; i < n; i++) fprintf(stderr, " %02x", p[i]);
	fputc('\n', stderr);
}

static int icc_send(void) {
	if (C.verbose) {
		struct icc_msg *m = (struct icc_msg *)C.xfer.query;
		hexdump("TX", C.xfer.query, m->length);
	}
	int ret = ioctl(C.fd, ICC_QUERY, &C.xfer);
	if (ret) {
		fprintf(stderr, "ioctl ICC_QUERY: ret=%d %s\n", ret, strerror(errno));
		return ret;
	}
	if (C.verbose) {
		struct icc_msg *r = (struct icc_msg *)C.xfer.reply;
		hexdump("RX", C.xfer.reply, r->length > 0 ? r->length : ICC_MSG_MIN_SIZE);
	}
	return 0;
}

/* === Mirror of hdmi.c i2c_* helpers ========================== */
static void i2c_init(uint8_t code) {
	memset(&C.xfer, 0, sizeof(C.xfer));
	struct icc_msg *m = (struct icc_msg *)C.xfer.query;
	m->service_id = ICC_SERVICE_ID_HDMI;
	m->msg_type = 0;
	C.hdr = (struct icc_i2c_msg *)(C.xfer.query + sizeof(struct icc_msg));
	C.hdr->code = code;
	C.hdr->length = 0;
	C.hdr->count = 0;
	C.cur = (uint8_t *)C.hdr + sizeof(struct icc_i2c_msg);
	C.block_index = -1;
}

static int i2c_exec(void) {
	struct icc_msg *m = (struct icc_msg *)C.xfer.query;
	C.hdr->length = (uint16_t)((C.cur - (uint8_t *)C.hdr));
	m->length = sizeof(struct icc_msg) + C.hdr->length;
	if (m->length < ICC_MSG_MIN_SIZE) m->length = ICC_MSG_MIN_SIZE;
	return icc_send();
}

static void i2c_write_block(struct i2c_block *block, size_t count) {
	struct i2c_cmd_hdr *h = (struct i2c_cmd_hdr *)C.cur;
	h->major = 2; h->minor = 2; h->count = (uint8_t)count;
	h->length = sizeof(*h);
	C.cur += sizeof(*h);
	for (size_t i = 0; i < count; i++) {
		struct i2c_cmd_write *cmd = (struct i2c_cmd_write *)C.cur;
		cmd->length = block[i].length;
		cmd->reg_low  = block[i].reg & 0xff;
		cmd->reg_high = block[i].reg >> 8;
		memcpy(cmd->data, block[i].data, block[i].length);
		C.cur += sizeof(*cmd) + cmd->length;
		h->length += sizeof(*cmd) + cmd->length;
	}
	C.hdr->count++;
}

static void i2c_begin_block(void) { C.block_index = 0; }
static void i2c_end_block(void) {
	if (C.block_index > 0) i2c_write_block(C.blocks, C.block_index);
	C.block_index = -1;
}

static void i2c_write_data(uint16_t reg, const uint8_t *value, size_t count) {
	int new_block = (C.block_index == -1);
	if (new_block) i2c_begin_block();
	struct i2c_block *b = &C.blocks[C.block_index++];
	b->reg = reg; b->length = (uint8_t)count;
	memcpy(b->data, value, count);
	if (new_block) i2c_end_block();
}

static void i2c_write(uint16_t reg, uint8_t value) {
	i2c_write_data(reg, &value, 1);
}

static void i2c_cmd_2_1_(uint16_t reg, uint8_t v) {
	struct i2c_cmd_hdr *h = (struct i2c_cmd_hdr *)C.cur;
	h->major = 2; h->minor = 1; h->count = 1; h->length = sizeof(*h);
	C.cur += sizeof(*h);
	struct i2c_cmd_2_1 *cmd = (struct i2c_cmd_2_1 *)C.cur;
	cmd->length = 1; cmd->reg_low = reg & 0xff; cmd->reg_high = reg >> 8; cmd->data[0] = v;
	C.cur += sizeof(*cmd) + cmd->length; h->length += sizeof(*cmd) + cmd->length;
	C.hdr->count++;
}

static void i2c_delay_(uint16_t time) {
	struct i2c_cmd_hdr *h = (struct i2c_cmd_hdr *)C.cur;
	h->major = 3; h->minor = 1; h->count = 1; h->length = sizeof(*h);
	C.cur += sizeof(*h);
	struct i2c_cmd_delay *cmd = (struct i2c_cmd_delay *)C.cur;
	cmd->length = 0; cmd->time_low = time & 0xff; cmd->time_high = time >> 8; cmd->unk_03 = 0;
	C.cur += sizeof(*cmd) + cmd->length; h->length += sizeof(*cmd) + cmd->length;
	C.hdr->count++;
}

static void i2c_mask(uint16_t reg, uint8_t value, uint8_t mask) {
	struct i2c_cmd_hdr *h = (struct i2c_cmd_hdr *)C.cur;
	h->major = 2; h->minor = 3; h->count = 1; h->length = sizeof(*h);
	C.cur += sizeof(*h);
	struct i2c_cmd_mask *cmd = (struct i2c_cmd_mask *)C.cur;
	cmd->length = 1; cmd->reg_low = reg & 0xff; cmd->reg_high = reg >> 8;
	cmd->value = value; cmd->mask[0] = mask;
	C.cur += sizeof(*cmd) + cmd->length; h->length += sizeof(*cmd) + cmd->length;
	C.hdr->count++;
}

static void i2c_waitset(uint16_t reg, uint8_t value) {
	struct i2c_cmd_hdr *h = (struct i2c_cmd_hdr *)C.cur;
	h->major = 3; h->minor = 2; h->count = 1; h->length = sizeof(*h);
	C.cur += sizeof(*h);
	struct i2c_cmd_waitset *cmd = (struct i2c_cmd_waitset *)C.cur;
	cmd->length = 0; cmd->reg_low = reg & 0xff; cmd->reg_high = reg >> 8; cmd->value = value;
	C.cur += sizeof(*cmd) + cmd->length; h->length += sizeof(*cmd) + cmd->length;
	C.hdr->count++;
}

static void i2c_waitclear(uint16_t reg, uint8_t value) {
	struct i2c_cmd_hdr *h = (struct i2c_cmd_hdr *)C.cur;
	h->major = 3; h->minor = 3; h->count = 1; h->length = sizeof(*h);
	C.cur += sizeof(*h);
	struct i2c_cmd_waitclear *cmd = (struct i2c_cmd_waitclear *)C.cur;
	cmd->length = 0; cmd->reg_low = reg & 0xff; cmd->reg_high = reg >> 8; cmd->value = value;
	C.cur += sizeof(*cmd) + cmd->length; h->length += sizeof(*cmd) + cmd->length;
	C.hdr->count++;
}

static void i2c_cmd_3_5_(uint16_t reg, uint8_t value) {
	struct i2c_cmd_hdr *h = (struct i2c_cmd_hdr *)C.cur;
	h->major = 3; h->minor = 5; h->count = 1; h->length = sizeof(*h);
	C.cur += sizeof(*h);
	struct i2c_cmd_3_5 *cmd = (struct i2c_cmd_3_5 *)C.cur;
	cmd->length = 0; cmd->reg_low = reg & 0xff; cmd->reg_high = reg >> 8; cmd->value = value;
	C.cur += sizeof(*cmd) + cmd->length; h->length += sizeof(*cmd) + cmd->length;
	C.hdr->count++;
}

static void i2c_cmd_5_3_(uint8_t v) {
	struct i2c_cmd_hdr *h = (struct i2c_cmd_hdr *)C.cur;
	h->major = 5; h->minor = 3; h->count = 1; h->length = sizeof(*h);
	C.cur += sizeof(*h);
	struct i2c_cmd_5_3 *cmd = (struct i2c_cmd_5_3 *)C.cur;
	cmd->value = v;
	C.cur += sizeof(*cmd); h->length += sizeof(*cmd);
	C.hdr->count++;
}

static void i2c_cmd_5_4_(uint8_t v) {
	struct i2c_cmd_hdr *h = (struct i2c_cmd_hdr *)C.cur;
	h->major = 5; h->minor = 4; h->count = 1; h->length = sizeof(*h);
	C.cur += sizeof(*h);
	struct i2c_cmd_5_4 *cmd = (struct i2c_cmd_5_4 *)C.cur;
	cmd->value = v;
	C.cur += sizeof(*cmd); h->length += sizeof(*cmd);
	C.hdr->count++;
}

static void i2c_cmd_5_5_(uint8_t v) {
	struct i2c_cmd_hdr *h = (struct i2c_cmd_hdr *)C.cur;
	h->major = 5; h->minor = 5; h->count = 1; h->length = sizeof(*h);
	C.cur += sizeof(*h);
	struct i2c_cmd_5_5 *cmd = (struct i2c_cmd_5_5 *)C.cur;
	cmd->value = v;
	C.cur += sizeof(*cmd); h->length += sizeof(*cmd);
	C.hdr->count++;
}

/* === High-level sequences (Flava3 only — that's the user's chip) === */

static void seq_stopHdcpHw(void) {
	memset(&C.xfer, 0, sizeof(C.xfer));
	struct icc_msg *m = (struct icc_msg *)C.xfer.query;
	m->service_id = ICC_SERVICE_ID_GENERAL;
	m->msg_type = 0x1f;
	m->length = ICC_MSG_MIN_SIZE;
	icc_send();
}

static void seq_disable_encode(void) {
	i2c_init(4);
	i2c_mask(0x705f, 0x80, 0x80);
	i2c_cmd_5_3_(0x00);
	i2c_mask(0x7021, 0x00, 0xf0);
	i2c_cmd_2_1_(0x7a88, 0xff);
	i2c_waitclear(0x7a84, 0x01);
	i2c_cmd_2_1_(0x740a, 0xff);
	i2c_waitclear(0x740a, 0xff);
	i2c_mask(0x7419, 0x05, 0x07);
	i2c_mask(0x7a8b, 0x00, 0x07);
	i2c_delay_(100);
	i2c_exec();
}

static void seq_init_isr(uint8_t tmds_polarity, uint8_t tmds_ch_swap) {
	i2c_init(0);
	i2c_begin_block();
	i2c_write(0x705f, 0x12);
	i2c_write(0x6004, 0x80);
	i2c_write(0x6020, 0x00);
	i2c_write(0x7007, 0xff);
	i2c_write(0x100c, 0x01);
	i2c_write(0x6008, 0xc0);
	i2c_write(0x6207, 0x00);
	i2c_write(0x621b, 0x00);
	i2c_write_data(0x6080, (uint8_t[]){0xff,0xff,0xff,0xff,0xff,0xff}, 6);
	i2c_write_data(0x6090, (uint8_t[]){0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff}, 8);
	i2c_write_data(0x10e7, (uint8_t[]){0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff}, 8);
	i2c_write_data(0x10e9, (uint8_t[]){0xff,0xff}, 2);
	i2c_write_data(0x10f0, (uint8_t[]){0xff,0x07,0x00,0x0f,0x00,0x56,0x00,0x00,0xd0,0x00}, 10);
	i2c_write(0x7018, (tmds_polarity == 1) << 1);
	i2c_end_block();
	if (tmds_ch_swap == 1) i2c_mask(0x701a, 0xb1, 0xff);
	i2c_exec();
}

static void seq_init_dp(uint8_t dp_polarity, uint8_t dp_ch_swap) {
	i2c_init(0);
	i2c_begin_block();
	i2c_write(0x6a03, 0x04);
	i2c_write(0x60a2, 0xf1);
	i2c_write(0x60bf, 0x03);
	i2c_write(0x60c0, 0xef);
	i2c_write(0x60c3, 0x4d);
	i2c_write(0x60c7, 0x85);
	i2c_write(0x60bf, 0x04);
	i2c_write(0x60c7, 0x84);
	if (dp_polarity != 0xff) i2c_write(0x600e, dp_polarity << 4);
	if (dp_ch_swap  != 0xff) i2c_write(0x600b, dp_ch_swap);
	i2c_write_data(0x6220, (uint8_t[]){0x40,0x00,0x80,0x00,0x00,0x01,0x02}, 7);
	i2c_write_data(0x6028, (uint8_t[]){0x01,0x02}, 2);
	i2c_write_data(0x6058, (uint8_t[]){0x01,0x02,0x03,0x04}, 4);
	i2c_write(0x62af, 0x01);
	i2c_write(0x6207, 0x00);
	i2c_write(0x621b, 0x00);
	i2c_write(0x60e0, 0x1e);
	i2c_write(0x603c, 0x04);
	i2c_write(0x100e, 0x10);
	i2c_end_block();
	i2c_delay_(2);
	i2c_write(0x6005, 0x01);
	i2c_delay_(2);
	i2c_write(0x6008, 0x00);
	i2c_exec();
}

static void seq_init_video_config(uint8_t tmds_pol, uint8_t tmds_swap, uint8_t dp_pol, uint8_t dp_swap) {
	seq_stopHdcpHw();
	seq_disable_encode();
	seq_init_isr(tmds_pol, tmds_swap);
	seq_init_dp(dp_pol, dp_swap);
}

static void seq_config_param_pre(void) {
	i2c_init(1);
	i2c_begin_block();
	i2c_write_data(0x6224, (uint8_t[]){0x00,0x01}, 2);
	i2c_write_data(0x1047, (uint8_t[]){0x00}, 1);
	i2c_write_data(0x1050, (uint8_t[]){0x00,0x00,0x00,0x00}, 4);
	i2c_write(0x7215, 0x00);
	i2c_write(0x7077, 0x00);
	i2c_write(0x7079, 0x80);
	i2c_end_block();
	i2c_exec();
}

static void seq_config_link_training(void) {
	i2c_init(1);
	i2c_begin_block();
	i2c_write_data(0x600c, (uint8_t[]){0x01,0x00}, 2);
	i2c_write_data(0x6c00, (uint8_t[]){0x1e,0x84,0x00}, 3);
	i2c_end_block();
	i2c_mask(0x6005, 0x01, 0x01);
	i2c_delay_(2);
	i2c_mask(0x6006, 0x04, 0x04);
	i2c_delay_(2);
	i2c_write(0x6a03, 0x47);
	i2c_delay_(10);
	i2c_waitset(0x60f8, 0xff);
	i2c_waitset(0x60f9, 0x01);
	i2c_write(0x6a01, 0x4d);
	i2c_waitset(0x60f9, 0x1a);
	i2c_waitset(0x6083, 0x02);
	i2c_exec();
}

static void seq_basic_video_config(int w, int h, int hz, uint8_t tmds_pol, uint8_t tmds_swap) {
	(void)w; (void)hz;
	int hispeed = (h == 2160) || (h == 1440 && hz >= 100);
	i2c_init(1);
	i2c_mask(0x7021, 0x00, 0xf0);
	i2c_delay_(500);
	i2c_begin_block();
	i2c_write(0x100c, 0x01);
	i2c_write_data(0x68a0, (uint8_t[]){0x96,0x04}, 2);
	i2c_write(0x7005, 0x80);
	i2c_write(0x7019, 0x00);
	i2c_write(0x100c, 0x01);
	i2c_write(0x7005, 0x80);
	i2c_write(0x7009, 0x00);
	i2c_write(0x7040, 0x42);
	i2c_write(0x7225, 0x20 | 0x08);
	if (h == 1080) {
		i2c_write_data(0x7227, (uint8_t[]){0x10,0x00}, 2);
		i2c_write_data(0x7070, (uint8_t[]){0x10,0x10,0x00,0x00,0x00,0x00}, 6);
	} else if (h == 1440) {
		i2c_write_data(0x7227, (uint8_t[]){0x04,0x00}, 2);
		i2c_write_data(0x7070, (uint8_t[]){0x00,0x00,0x00,0x00,0x00,0xfb}, 6);
	} else if (h == 2160) {
		i2c_write_data(0x7227, (uint8_t[]){0x06,0x00}, 2);
		i2c_write_data(0x7070, (uint8_t[]){0x00,0x00,0x00,0x00,0x00,0x00}, 6);
	}
	i2c_write(0x70c0, 0xdc);
	i2c_write(0x621b, 0x00);
	i2c_write(0x629a, 0x00);
	i2c_write_data(0x70c4, (uint8_t[]){0x08,0x08}, 2);
	i2c_write(0x70c2, 0x00);
	i2c_write(0x70fe, 0x02);
	i2c_write(0x70c3, 0x00);
	i2c_write(0x7018, (tmds_pol == 1) << 1);
	if (tmds_swap == 1) {
		i2c_end_block();
		i2c_mask(0x701a, 0xb1, 0xff);
		i2c_begin_block();
	}
	i2c_write(0x10e7, 0xff);
	i2c_write(0x7202, 0x20);
	if (hispeed) {
		i2c_write(0x7072, 0x01);
		i2c_write(0x7074, 0x07);
		i2c_write(0x7206, 0x80);
		i2c_write(0x7203, 0x60);
		i2c_write(0x7011, 0xff);
	} else {
		i2c_write(0x7203, 0x60);
		i2c_write(0x7011, 0xd5);
	}
	i2c_end_block();
	i2c_waitclear(0x7011, 0xff);
	i2c_exec();
}

static void seq_phy_1st(void) {
	i2c_init(0);
	i2c_begin_block();
	i2c_write(0x7022, 0x03);
	i2c_write(0x7030, 0x40); i2c_write_data(0x7024, (uint8_t[]){0x04,0x54}, 2);
	i2c_write(0x7030, 0x41); i2c_write_data(0x7024, (uint8_t[]){0x04,0x54}, 2);
	i2c_write(0x7030, 0x42); i2c_write_data(0x7024, (uint8_t[]){0x04,0x34}, 2);
	i2c_write(0x7030, 0x43); i2c_write_data(0x7024, (uint8_t[]){0x04,0x34}, 2);
	i2c_write(0x7030, 0x04); i2c_write_data(0x7024, (uint8_t[]){0x04,0x71}, 2);
	i2c_write(0x7030, 0x14); i2c_write(0x7025, 0x71);
	i2c_write(0x7030, 0x24); i2c_write(0x7025, 0x70);
	i2c_write(0x7030, 0x34); i2c_write(0x7025, 0x77);
	i2c_write(0x7030, 0x05); i2c_write_data(0x7024, (uint8_t[]){0x04,0x71}, 2);
	i2c_write(0x7030, 0x15); i2c_write(0x7025, 0x71);
	i2c_write(0x7030, 0x25); i2c_write(0x7025, 0x70);
	i2c_write(0x7030, 0x35); i2c_write(0x7025, 0x77);
	i2c_write(0x7030, 0x06); i2c_write_data(0x7024, (uint8_t[]){0x04,0x71}, 2);
	i2c_write(0x7030, 0x16); i2c_write(0x7025, 0x70);
	i2c_write(0x7030, 0x26); i2c_write(0x7025, 0x70);
	i2c_write(0x7030, 0x36); i2c_write(0x7025, 0x77);
	i2c_end_block();
	i2c_exec();
}

static void seq_phy_2nd(void) {
	i2c_init(0);
	i2c_begin_block();
	i2c_write(0x7030, 0x47); i2c_write(0x7024, 0x04); i2c_write(0x7025, 0x37); i2c_write(0x7027, 0x17);
	i2c_write(0x7030, 0x37); i2c_write(0x7025, 0x71); i2c_write(0x7027, 0x14);
	i2c_write(0x7030, 0x48); i2c_write(0x7024, 0x04); i2c_write(0x7025, 0x37); i2c_write(0x7027, 0x17);
	i2c_write(0x7030, 0x38); i2c_write(0x7025, 0x77); i2c_write(0x7027, 0x14);
	i2c_write(0x7030, 0x49); i2c_write(0x7024, 0x01);
	i2c_write(0x7030, 0x19); i2c_write(0x7026, 0xcc);
	i2c_write(0x7030, 0x29); i2c_write(0x7026, 0xdd);
	i2c_write(0x7030, 0x39); i2c_write(0x7026, 0xef);
	i2c_write(0x7030, 0x4a); i2c_write(0x7024, 0x01);
	i2c_write(0x7030, 0x1a); i2c_write(0x7026, 0xcc);
	i2c_write(0x7030, 0x2a); i2c_write(0x7026, 0xdd);
	i2c_write(0x7030, 0x3a); i2c_write(0x7026, 0xef);
	i2c_write(0x7030, 0x4b); i2c_write(0x7024, 0x01);
	i2c_write(0x7030, 0x1b); i2c_write(0x7026, 0xcc);
	i2c_write(0x7030, 0x2b); i2c_write(0x7026, 0xdd);
	i2c_write(0x7030, 0x3b); i2c_write(0x7026, 0xef);
	i2c_end_block();
	i2c_exec();
}

static void seq_mask_7203(void) {
	i2c_init(1);
	i2c_mask(0x7203, 0x00, 0x80);
	i2c_exec();
}

static void seq_video_additional(int w, int h, int hz) {
	(void)w; (void)hz;
	int hispeed = (h == 2160) || (h == 1440 && hz >= 100);
	i2c_init(1);
	i2c_waitset(0x10e7, 0x80);
	if (h == 2160) i2c_mask(0x7227, 97, 0xff);
	i2c_mask(0x7226, 1 << 7, 0x80);
	i2c_mask(0x7228, 3 << 4, 0x30);
	i2c_write(0x7204, 0x40);
	i2c_waitclear(0x7204, 0x40);
	i2c_delay_(10);
	i2c_begin_block();
	if (hispeed) i2c_write(0x7019, 0x01); else i2c_write(0x7019, 0x00);
	i2c_write(0x7419, 0x05);
	i2c_write(0x740a, 0xff);
	i2c_end_block();
	i2c_waitclear(0x740a, 0xff);
	i2c_begin_block();
	i2c_write(0x7404, 0x00);
	i2c_write(0x7a88, 0xff);
	i2c_end_block();
	i2c_waitclear(0x7a84, 0x01);
	i2c_begin_block();
	i2c_write(0x7a8b, 0x05);
	if (hispeed) {
		i2c_write(0x7c00, 0x03);
		i2c_write_data(0x7a80, (uint8_t[]){0xa8,0x20,0x00,0x80}, 4);
	}
	i2c_end_block();
	if (hispeed) i2c_waitclear(0x7a84, 0x01);
	i2c_begin_block();
	i2c_write(0x7021, 0xff);
	i2c_write(0x700a, 0x02);
	i2c_write(0x705f, 0x80);
	i2c_end_block();
	i2c_delay_(500);
	i2c_write(0x7a8b, 0x00);
	i2c_cmd_5_3_(0x01);
	i2c_exec();
}

static void seq_set_video_config(int w, int h, int hz, uint8_t tmds_pol, uint8_t tmds_swap) {
	seq_config_param_pre();
	seq_config_link_training();
	seq_basic_video_config(w, h, hz, tmds_pol, tmds_swap);
	seq_phy_1st();
	seq_phy_2nd();
	seq_mask_7203();
	seq_video_additional(w, h, hz);
}

static void seq_audio_basic(void) {
	i2c_init(2);
	i2c_begin_block();
	i2c_write(0x62a0, 0x06); i2c_write(0x62a7, 0x13); i2c_write(0x62ac, 0x82);
	i2c_write(0x62cb, 0x02); i2c_write(0x62cb, 0x03); i2c_write(0x62cb, 0x00);
	i2c_write(0x70ad, 0x00); i2c_write(0x70af, 0x07); i2c_write(0x70a9, 0x5e);
	i2c_end_block();
	i2c_mask(0x70af, 0x06, 0x06);
	i2c_mask(0x70b3, 0x02, 0x0f);
	i2c_mask(0x70ae, 0x80, 0xe0);
	i2c_mask(0x70ae, 0x05, 0x07);
	i2c_mask(0x70ac, 0x01, 0x21);
	i2c_mask(0x70ab, 0x81, 0x89);
	i2c_mask(0x70a9, 0x08, 0x08);
	i2c_exec();
}

static void seq_audio_additional(void) {
	i2c_init(2);
	i2c_mask(0x70b0, 0x00, 0xff); i2c_mask(0x70b1, 0x79, 0xff);
	i2c_mask(0x70b2, 0x00, 0xff); i2c_mask(0x70b3, 0x02, 0xff);
	i2c_mask(0x70b4, 0x0b, 0x0f); i2c_mask(0x70b5, 0x00, 0xff);
	i2c_mask(0x70b6, 0x00, 0xff);
	i2c_begin_block();
	i2c_write(0x10e7, 0xff); i2c_write(0x7011, 0xa2);
	i2c_end_block();
	i2c_waitset(0x10e7, 0xa2);
	i2c_mask(0x7267, 0x0b, 0xff);
	i2c_write(0x7204, 0x10);
	i2c_waitclear(0x7204, 0x10);
	i2c_write(0x10e7, 0xff);
	i2c_mask(0x7203, 0x10, 0x10);
	i2c_delay_(30);
	i2c_write(0x70a8, 0xc0);
	i2c_exec();
}

static void seq_set_audio_config(void) {
	seq_audio_basic();
	seq_audio_additional();
}

static void seq_video_mute(int mute) {
	i2c_init(4);
	i2c_write(0x705f, (mute == 1) << 7);
	i2c_exec();
}

static void seq_audio_mute(int mute) {
	i2c_init(4);
	i2c_mask(0x70a8, (mute == 1) << 2, 0x04);
	i2c_exec();
}

/* === CLI dispatch =========================================== */

static void usage(const char *prog) {
	fprintf(stderr,
"Usage: %s [-v] <command> [args]\n"
"  -v                       verbose hex dump of TX/RX\n"
"\n"
"High-level commands:\n"
"  init [tmds_pol tmds_swap dp_pol dp_swap]\n"
"                          full sceHdmiInitVideoConfig (default 0 0 0xff 0xff)\n"
"  set_video <w> <h> <hz> [tmds_pol tmds_swap]\n"
"  set_audio                sceHdmiSetAudioConfig\n"
"  video_mute <0|1>\n"
"  audio_mute <0|1>\n"
"\n"
"Init sub-steps (run individually):\n"
"  stop_hdcp\n"
"  disable_encode\n"
"  init_isr  [tmds_pol tmds_swap]\n"
"  init_dp   [dp_pol dp_swap]\n"
"\n"
"SetVideoConfig sub-steps:\n"
"  config_param_pre\n"
"  config_link_training\n"
"  basic_video_config <w> <h> <hz> [tmds_pol tmds_swap]\n"
"  phy_1st\n"
"  phy_2nd\n"
"  mask_7203\n"
"  video_additional <w> <h> <hz>\n"
"\n"
"Atomic i2c ops (build & send single ICC msg):\n"
"  i2c_write    <reg16> <val8>             (code=1)\n"
"  i2c_mask     <reg16> <val8> <mask8>     (code=1)\n"
"  i2c_waitset  <reg16> <val8>             (code=1)\n"
"  i2c_waitclear<reg16> <val8>             (code=1)\n"
"  cmd_5_3      <val8>                     (code=4)\n"
"\n"
"Raw:\n"
"  raw <service_id> <msg_type> <hex bytes>   build & send raw ICC msg\n"
, prog);
}

static unsigned long argl(const char *s) {
	char *e; unsigned long v = strtoul(s, &e, 0);
	if (*e) die("bad number: %s", s);
	return v;
}

int main(int argc, char **argv) {
	int ai = 1;
	if (ai < argc && strcmp(argv[ai], "-v") == 0) { C.verbose = 1; ai++; }
	if (ai >= argc) { usage(argv[0]); return 1; }
	ctx_open();

	const char *cmd = argv[ai++];
	int rest = argc - ai;
	char **a = argv + ai;

	if (!strcmp(cmd, "init")) {
		uint8_t tp=0, ts=0, dp=0xff, ds=0xff;
		if (rest >= 4) { tp=argl(a[0]); ts=argl(a[1]); dp=argl(a[2]); ds=argl(a[3]); }
		seq_init_video_config(tp, ts, dp, ds);
	} else if (!strcmp(cmd, "set_video")) {
		if (rest < 3) { usage(argv[0]); return 1; }
		int w=argl(a[0]), h=argl(a[1]), hz=argl(a[2]);
		uint8_t tp=0, ts=0;
		if (rest >= 5) { tp=argl(a[3]); ts=argl(a[4]); }
		seq_set_video_config(w, h, hz, tp, ts);
	} else if (!strcmp(cmd, "set_audio")) {
		seq_set_audio_config();
	} else if (!strcmp(cmd, "video_mute")) {
		if (rest < 1) { usage(argv[0]); return 1; }
		seq_video_mute(argl(a[0]));
	} else if (!strcmp(cmd, "audio_mute")) {
		if (rest < 1) { usage(argv[0]); return 1; }
		seq_audio_mute(argl(a[0]));
	} else if (!strcmp(cmd, "stop_hdcp")) {
		seq_stopHdcpHw();
	} else if (!strcmp(cmd, "disable_encode")) {
		seq_disable_encode();
	} else if (!strcmp(cmd, "init_isr")) {
		uint8_t tp=0, ts=0;
		if (rest >= 2) { tp=argl(a[0]); ts=argl(a[1]); }
		seq_init_isr(tp, ts);
	} else if (!strcmp(cmd, "init_dp")) {
		uint8_t dp=0xff, ds=0xff;
		if (rest >= 2) { dp=argl(a[0]); ds=argl(a[1]); }
		seq_init_dp(dp, ds);
	} else if (!strcmp(cmd, "config_param_pre")) {
		seq_config_param_pre();
	} else if (!strcmp(cmd, "config_link_training")) {
		seq_config_link_training();
	} else if (!strcmp(cmd, "basic_video_config")) {
		if (rest < 3) { usage(argv[0]); return 1; }
		int w=argl(a[0]), h=argl(a[1]), hz=argl(a[2]);
		uint8_t tp=0, ts=0;
		if (rest >= 5) { tp=argl(a[3]); ts=argl(a[4]); }
		seq_basic_video_config(w, h, hz, tp, ts);
	} else if (!strcmp(cmd, "phy_1st")) {
		seq_phy_1st();
	} else if (!strcmp(cmd, "phy_2nd")) {
		seq_phy_2nd();
	} else if (!strcmp(cmd, "mask_7203")) {
		seq_mask_7203();
	} else if (!strcmp(cmd, "video_additional")) {
		if (rest < 3) { usage(argv[0]); return 1; }
		seq_video_additional(argl(a[0]), argl(a[1]), argl(a[2]));
	} else if (!strcmp(cmd, "i2c_write")) {
		if (rest < 2) { usage(argv[0]); return 1; }
		i2c_init(1); i2c_write(argl(a[0]), argl(a[1])); i2c_exec();
	} else if (!strcmp(cmd, "i2c_mask")) {
		if (rest < 3) { usage(argv[0]); return 1; }
		i2c_init(1); i2c_mask(argl(a[0]), argl(a[1]), argl(a[2])); i2c_exec();
	} else if (!strcmp(cmd, "i2c_waitset")) {
		if (rest < 2) { usage(argv[0]); return 1; }
		i2c_init(1); i2c_waitset(argl(a[0]), argl(a[1])); i2c_exec();
	} else if (!strcmp(cmd, "i2c_waitclear")) {
		if (rest < 2) { usage(argv[0]); return 1; }
		i2c_init(1); i2c_waitclear(argl(a[0]), argl(a[1])); i2c_exec();
	} else if (!strcmp(cmd, "cmd_5_3")) {
		if (rest < 1) { usage(argv[0]); return 1; }
		i2c_init(4); i2c_cmd_5_3_(argl(a[0])); i2c_exec();
	} else if (!strcmp(cmd, "raw")) {
		if (rest < 2) { usage(argv[0]); return 1; }
		memset(&C.xfer, 0, sizeof(C.xfer));
		struct icc_msg *m = (struct icc_msg *)C.xfer.query;
		m->service_id = argl(a[0]);
		m->msg_type = argl(a[1]);
		size_t n = 0;
		for (int i = 2; i < rest && n < ICC_MSG_MAX_SIZE - sizeof(struct icc_msg); i++)
			m->data[n++] = argl(a[i]);
		m->length = sizeof(struct icc_msg) + n;
		if (m->length < ICC_MSG_MIN_SIZE) m->length = ICC_MSG_MIN_SIZE;
		icc_send();
	} else {
		usage(argv[0]); return 1;
	}

	close(C.fd);
	return 0;
}
