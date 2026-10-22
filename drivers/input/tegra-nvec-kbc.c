// SPDX-License-Identifier: GPL-2.0+
/*
 * Keyboard driver for the NVIDIA embedded controller (NVEC) found in the
 * Toshiba AC100 / Dynabook AZ (Compal paz00), Tegra20.
 *
 * The EC is the I2C bus master. The Tegra I2C3 controller runs in slave
 * mode and is polled (no interrupts in U-Boot). To send a request we pull
 * the "request" GPIO low; the EC then does a block read from us and later
 * writes the response back. Key events arrive as unsolicited writes.
 *
 * Protocol and register handling follow the Linux staging/nvec driver.
 * Based on the 2013 U-Boot driver by Andrey Danin <andreydanin@mail.ru>.
 */

#include <dm.h>
#include <input.h>
#include <keyboard.h>
#include <log.h>
#include <stdio_dev.h>
#include <time.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/arch/clock.h>
#include <asm/arch/funcmux.h>
#include <linux/delay.h>
#include <linux/input.h>

/* not in U-Boot's linux/input.h; values as in Linux */
#ifndef KEY_MENU
#define KEY_MENU	139
#endif
#ifndef KEY_SEARCH
#define KEY_SEARCH	217
#endif
#ifndef KEY_CANCEL
#define KEY_CANCEL	223
#endif

#include "tegra-nvec-keytable.h"

/* Tegra I2C controller, slave-mode registers */
#define I2C_CNFG			0x00
#define I2C_CNFG_PACKET_MODE_EN		BIT(10)
#define I2C_CNFG_NEW_MASTER_SFM		BIT(11)
#define I2C_CNFG_DEBOUNCE_CNT_SHIFT	12

#define I2C_SL_CNFG			0x20
#define I2C_SL_NEWSL			BIT(2)

#define I2C_SL_RCVD			0x24
#define I2C_SL_STATUS			0x28
#define I2C_SL_IRQ			BIT(3)
#define END_TRANS			BIT(4)
#define RCVD				BIT(2)
#define RNW				BIT(1)

#define I2C_SL_ADDR1			0x2c
#define I2C_SL_ADDR2			0x30
#define I2C_SL_DELAY_COUNT		0x3c

/* NVEC message types / commands */
#define NVEC_KEYBOARD			0
#define NVEC_SLEEP			4
#define SLEEP_GLOBAL_EVENTS		0
#define NVEC_KBD			5
#define KBD_CNFG_WAKE			3
#define KBD_CNFG_WAKE_KEY_REPORTING	4
#define KBD_SET_LEDS			0xed
#define KBD_ENABLE			0xf4
#define NVEC_PS2			6
#define NVEC_CNTL			7
#define CNTL_NOOP			0x02
#define CNTL_GET_FIRMWARE_VERSION	0x15
#define PS2_SEND_CMD			1
#define PS2_RESET			0xff

#define NVEC_MSG_SIZE			34
#define NVEC_MAX_PRESSED		8

enum {
	NVEC_2BYTES,
	NVEC_3BYTES,
	NVEC_VAR_SIZE,
};

enum {
	KBC_REPEAT_RATE_MS	= 30,
	KBC_REPEAT_DELAY_MS	= 240,
	KBC_STUCK_KEY_MS	= 2000,	/* drop held modifiers after this long */
	NVEC_PING_MS		= 3000,	/* ping the EC after this much silence */
	NVEC_RELEASE_MS		= 300,	/* ...or this much, with a key still down */
	NVEC_REQ_TIMEOUT_MS	= 1000,	/* EC normally answers within 500ms */
	NVEC_REQ_ATTEMPTS	= 3,
};

/* result of handling one slave-controller event */
enum nvec_ev {
	NVEC_EV_NONE,		/* nothing pending */
	NVEC_EV_PROGRESS,	/* a byte moved */
	NVEC_EV_RX_DONE,	/* a message from the EC is in rx[] */
	NVEC_EV_TX_DONE,	/* the EC has read our request */
};

struct nvec_priv {
	void __iomem *base;
	struct gpio_desc request_gpio;
	uint i2c_addr;			/* 8-bit address, 0x8a on paz00 */
	uint clk_hz;
	bool dead;			/* EC never answered; stop trying */

	int state;			/* same numbering as Linux */
	u8 rx[NVEC_MSG_SIZE];
	int rx_pos;
	u8 tx[NVEC_MSG_SIZE];		/* tx[0] is the length byte */
	int tx_size;
	int tx_pos;

	int pressed[NVEC_MAX_PRESSED];	/* held modifier keys */
	int npressed;
	int strikes[16];		/* complete keystrokes not yet delivered */
	int nstrikes;
	ulong last_key_ms;		/* when the EC last reported a key */
	ulong last_mod_ms;		/* when a modifier last changed */
	ulong last_rx_ms;		/* when the EC last said anything */
	uint wedged;			/* polls that made no progress */
	uint n_reinit, n_pingfail;
	uint n_press, n_release;
	u32 outstanding[8];		/* raw codes pressed, release not seen */
	uint test_delay_ms;		/* nvecdbg: stall after each message */
	ulong last_poll_ms, max_gap_ms;	/* how long U-Boot goes without polling */
	uint gaps[4];			/* polls late by >5, >20, >50, >200 ms */
	u32 bad_status[4];		/* last unexpected status words */
	int bad_state[4];
	struct input_config *input;

	/* diagnostics, shown by the 'nvecdbg' command */
	uint n_rx, n_key, n_bad;
	u8 last_rx[NVEC_MSG_SIZE];
	int last_rx_len;
	bool trace;
#define NVEC_LOG 24
	u8 log[NVEC_LOG][8];
	u8 log_len[NVEC_LOG];
	uint log_n;
};

static struct nvec_priv *nvec_dbg_priv;

static void nvec_writel(u32 val, void __iomem *reg)
{
	writel(val, reg);
	readl(reg);	/* Tegra20: read back to make sure the write landed */
}

static void nvec_request(struct nvec_priv *priv, bool assert)
{
	/* DT says ACTIVE_HIGH; the EC wants the line low to see a request */
	dm_gpio_set_value(&priv->request_gpio, assert ? 0 : 1);
}

static bool nvec_is_modifier(int keycode)
{
	switch (keycode) {
	case KEY_LEFTSHIFT: case KEY_RIGHTSHIFT:
	case KEY_LEFTCTRL: case KEY_RIGHTCTRL:
	case KEY_LEFTALT: case KEY_RIGHTALT:
	case KEY_LEFTMETA: case KEY_FN:
		return true;
	}
	return false;
}

/*
 * Releases can get lost (the EC does not wait forever while U-Boot is busy),
 * and a key the input layer believes is held repeats for ever. So only
 * modifiers are tracked as held; every other key-down is delivered as one
 * complete keystroke. If the EC repeats key-downs while a key is held, that
 * gives auto-repeat for free.
 */
static void nvec_key_event(struct nvec_priv *priv, int keycode, bool down)
{
	int i;

	if (!keycode)
		return;
	if (!nvec_is_modifier(keycode)) {
		if (priv->npressed &&
		    get_timer(priv->last_mod_ms) > KBC_STUCK_KEY_MS)
			priv->npressed = 0;	/* a modifier release was lost */
		if (down && priv->nstrikes < ARRAY_SIZE(priv->strikes))
			priv->strikes[priv->nstrikes++] = keycode;
		return;
	}
	priv->last_mod_ms = get_timer(0);
	for (i = 0; i < priv->npressed; i++)
		if (priv->pressed[i] == keycode)
			break;
	if (down) {
		if (i == priv->npressed && priv->npressed < NVEC_MAX_PRESSED)
			priv->pressed[priv->npressed++] = keycode;
	} else if (i < priv->npressed) {
		priv->npressed--;
		for (; i < priv->npressed; i++)
			priv->pressed[i] = priv->pressed[i + 1];
	}
}

static void nvec_rx_completed(struct nvec_priv *priv)
{
	const u8 *msg = priv->rx;
	int size, code;

	priv->n_rx++;
	priv->last_rx_ms = get_timer(0);
	priv->last_rx_len = priv->rx_pos;
	memcpy(priv->last_rx, priv->rx, priv->rx_pos);
	priv->log_len[priv->log_n % NVEC_LOG] = min(priv->rx_pos, 8);
	memcpy(priv->log[priv->log_n % NVEC_LOG], priv->rx, min(priv->rx_pos, 8));
	priv->log_n++;
	if (priv->trace) {
		int i;

		printf("nvec rx[%d]:", priv->rx_pos);
		for (i = 0; i < priv->rx_pos; i++)
			printf(" %02x", priv->rx[i]);
		printf("\n");
	}

	/* only events (bit 7) of type keyboard interest us */
	if (!(msg[0] & 0x80) || (msg[0] & 0x0f) != NVEC_KEYBOARD)
		return;

	size = (msg[0] & (3 << 5)) >> 5;
	if (size == NVEC_VAR_SIZE)
		return;
	if (size == NVEC_3BYTES)
		msg++;

	code = msg[1] & 0x7f;
	priv->n_key++;
	priv->last_key_ms = get_timer(0);
	if (msg[1] & 0x80) {
		priv->n_release++;
		priv->outstanding[(size << 2) | (code >> 5)] &= ~BIT(code & 31);
	} else {
		priv->n_press++;
		priv->outstanding[(size << 2) | (code >> 5)] |= BIT(code & 31);
	}
	if (priv->trace)
		printf("nvec key: raw %02x -> keycode %d %s\n", msg[1],
		       code_tabs[size][code], msg[1] & 0x80 ? "up" : "down");
	nvec_key_event(priv, code_tabs[size][code], !(msg[1] & 0x80));
}

/*
 * Handle one pending slave-controller event. Mirrors nvec_interrupt() in
 * the Linux driver, minus the message queues.
 */
static void nvec_bad(struct nvec_priv *priv, u32 status)
{
	priv->bad_status[priv->n_bad % 4] = status;
	priv->bad_state[priv->n_bad % 4] = priv->state;
	priv->n_bad++;
	if (priv->trace)
		printf("{bad st%d %02x}", priv->state, status);
}

static enum nvec_ev nvec_step(struct nvec_priv *priv)
{
	u32 status = readl(priv->base + I2C_SL_STATUS);
	u32 received = 0;
	u8 to_send = 0xff;
	enum nvec_ev ev = NVEC_EV_PROGRESS;

	if (!(status & I2C_SL_IRQ))
		return NVEC_EV_NONE;
	if (priv->trace)
		printf("[st%d %02lx]", priv->state, (ulong)status);

	/* the EC is writing to us: fetch the byte */
	if (!(status & RNW)) {
		received = readl(priv->base + I2C_SL_RCVD);
		if (status & RCVD)
			nvec_writel(0, priv->base + I2C_SL_RCVD);
	}

	if (status == (I2C_SL_IRQ | RCVD))
		priv->state = 0;

	switch (priv->state) {
	case 0:		/* expect a transfer start */
		if (status != (I2C_SL_IRQ | RCVD))
			nvec_bad(priv, status);
		break;
	case 1:		/* command byte */
		if (status != I2C_SL_IRQ) {
			nvec_bad(priv, status);
			priv->state = 0;
		} else {
			priv->rx[0] = received;
			priv->rx_pos = 1;
			priv->state = 2;
		}
		break;
	case 2:		/* first byte after the command */
		if (status == (I2C_SL_IRQ | RNW | RCVD)) {
			/* EC restarted as reader: it wants our request */
			if (priv->rx[0] != 0x01) {
				debug("nvec: read without read command\n");
				priv->state = 0;
				break;
			}
			priv->state = 3;
			if (!priv->tx_size) {
				/* nothing queued: answer with a no-op */
				priv->tx[0] = 2;
				priv->tx[1] = NVEC_CNTL;
				priv->tx[2] = CNTL_NOOP;
				priv->tx_size = 3;
			}
			to_send = priv->tx[0];
			priv->tx_pos = 1;
			udelay(33);	/* AP20 bug: delay the ACK */
		} else if (status == I2C_SL_IRQ) {
			priv->rx[1] = received;
			priv->rx_pos = 2;
			priv->state = 4;
		} else {
			nvec_bad(priv, status);
			priv->state = 0;
		}
		break;
	case 3:		/* EC reads our request */
		if (status & END_TRANS) {
			if (priv->tx_pos == priv->tx_size) {
				priv->state = 0;
				priv->tx_size = 0;
				ev = NVEC_EV_TX_DONE;
			} else {
				debug("nvec: premature END_TRANS, resending\n");
				priv->tx_pos = 0;
				nvec_request(priv, true);
			}
		} else if (!(status & RNW) || (status & RCVD)) {
			nvec_bad(priv, status);
			priv->state = 0;
		} else if (priv->tx_pos < priv->tx_size) {
			to_send = priv->tx[priv->tx_pos++];
		} else {
			debug("nvec: tx underflow\n");
			priv->state = 0;
		}
		break;
	case 4:		/* EC writes a message to us */
		if ((status & (END_TRANS | RNW)) == END_TRANS) {
			nvec_rx_completed(priv);
			priv->state = 0;
			ev = NVEC_EV_RX_DONE;
			if (priv->test_delay_ms)
				mdelay(priv->test_delay_ms);
		} else if (status & (RNW | RCVD)) {
			nvec_bad(priv, status);
			priv->state = 0;
		} else if (priv->rx_pos < NVEC_MSG_SIZE) {
			priv->rx[priv->rx_pos++] = received;
		} else {
			debug("nvec: rx overflow\n");
		}
		break;
	default:
		priv->state = 0;
	}

	/* a new transfer starts: check it is addressed to us */
	if ((status & (RCVD | RNW)) == RCVD) {
		if (received != priv->i2c_addr)
			debug("nvec: got address %x, expected %x\n", received,
			      priv->i2c_addr);
		priv->state = 1;
	}

	/* EC wants a byte, and this is not the end of the transfer */
	if ((status & (RNW | END_TRANS)) == RNW)
		nvec_writel(to_send, priv->base + I2C_SL_RCVD);

	/* first byte of our request went out: drop the request line */
	if (status == (I2C_SL_IRQ | RNW | RCVD))
		nvec_request(priv, false);

	return ev;
}

/* Run the state machine until @want happens or @timeout_ms passes */
static int nvec_wait(struct nvec_priv *priv, enum nvec_ev want, uint timeout_ms)
{
	ulong start = get_timer(0);

	while (get_timer(start) < timeout_ms) {
		enum nvec_ev ev = nvec_step(priv);

		if (ev == want)
			return 0;
		if (ev == NVEC_EV_NONE)
			udelay(50);
	}

	return -ETIMEDOUT;
}

/* Send a command to the EC and wait until the EC has read it */
static int nvec_write(struct nvec_priv *priv, const u8 *data, int size)
{
	int try;

	if (priv->dead || size + 1 > NVEC_MSG_SIZE)
		return -EINVAL;

	priv->tx[0] = size;
	memcpy(&priv->tx[1], data, size);
	priv->tx_size = size + 1;

	for (try = 0; try < NVEC_REQ_ATTEMPTS; try++) {
		priv->tx_pos = 0;
		priv->state = 0;
		nvec_request(priv, true);
		if (!nvec_wait(priv, NVEC_EV_TX_DONE, NVEC_REQ_TIMEOUT_MS)) {
			/* give the EC a moment to write its response back */
			nvec_wait(priv, NVEC_EV_RX_DONE, 200);
			return 0;
		}
		nvec_request(priv, false);
		debug("nvec: request %02x/%02x: no answer (try %d)\n",
		      data[0], data[1], try);
	}

	return -ETIMEDOUT;
}

static void nvec_init_i2c_slave(struct nvec_priv *priv)
{
	u32 val;

	clock_start_periph_pll(PERIPH_ID_I2C3, CLOCK_ID_PERIPH,
			       priv->clk_hz * 8);
	funcmux_select(PERIPH_ID_I2C3, FUNCMUX_DEFAULT);
	reset_periph(PERIPH_ID_I2C3, 2);

	val = I2C_CNFG_NEW_MASTER_SFM | I2C_CNFG_PACKET_MODE_EN |
	      (0x2 << I2C_CNFG_DEBOUNCE_CNT_SHIFT);
	nvec_writel(val, priv->base + I2C_CNFG);
	nvec_writel(I2C_SL_NEWSL, priv->base + I2C_SL_CNFG);
	nvec_writel(0x1e, priv->base + I2C_SL_DELAY_COUNT);
	nvec_writel(priv->i2c_addr >> 1, priv->base + I2C_SL_ADDR1);
	nvec_writel(0, priv->base + I2C_SL_ADDR2);
}

static void nvec_watchdog(struct nvec_priv *priv);

/*
 * Called by input_tstc()/input_getc(): drain pending EC events, then hand
 * the set of currently held keys to the input layer, which turns it into
 * press/release/repeat.
 */
static int nvec_kbd_check(struct input_config *input)
{
	struct nvec_priv *priv = dev_get_priv(input->dev);
	int i;

	if (priv->dead)
		return 0;

	if (priv->last_poll_ms) {
		ulong gap = get_timer(priv->last_poll_ms);

		if (gap > priv->max_gap_ms)
			priv->max_gap_ms = gap;
		if (gap > 5)
			priv->gaps[0]++;
		if (gap > 20)
			priv->gaps[1]++;
		if (gap > 50)
			priv->gaps[2]++;
		if (gap > 200)
			priv->gaps[3]++;
	}
	priv->last_poll_ms = get_timer(0);

	/* bounded: never stall the console on a chatty EC */
	for (i = 0; i < 64; i++)
		if (nvec_step(priv) == NVEC_EV_NONE)
			break;

	/* still "busy" after 64 steps, poll after poll: the controller is wedged */
	if (i == 64) {
		if (++priv->wedged > 200) {
			printf("nvec: I2C slave wedged, resetting\n");
			nvec_init_i2c_slave(priv);
			priv->state = 0;
			priv->wedged = 0;
		}
	} else {
		priv->wedged = 0;
	}

	/* a lost modifier release must not stick for the whole session */
	if (priv->npressed &&
	    get_timer(priv->last_key_ms) > KBC_STUCK_KEY_MS)
		priv->npressed = 0;

	{
		ulong quiet = get_timer(priv->last_rx_ms);
		bool held = priv->outstanding[0] | priv->outstanding[1] |
			    priv->outstanding[2] | priv->outstanding[3] |
			    priv->outstanding[4] | priv->outstanding[5] |
			    priv->outstanding[6] | priv->outstanding[7];

		if (quiet > NVEC_PING_MS || (held && quiet > NVEC_RELEASE_MS))
			nvec_watchdog(priv);
	}
	priv->last_poll_ms = get_timer(0);

	if (!priv->nstrikes) {
		input_send_keycodes(input, priv->pressed, priv->npressed);
		return 1;
	}

	for (i = 0; i < priv->nstrikes; i++) {
		int keys[NVEC_MAX_PRESSED + 1];

		memcpy(keys, priv->pressed, priv->npressed * sizeof(int));
		keys[priv->npressed] = priv->strikes[i];
		input_send_keycodes(input, keys, priv->npressed + 1);
		input_send_keycodes(input, priv->pressed, priv->npressed);
	}
	priv->nstrikes = 0;

	return 1;
}

static int nvec_kbd_start(struct udevice *dev)
{
	return 0;
}

/* Bring the EC link up: init the slave controller, enable key events */
static int nvec_enable(struct nvec_priv *priv)
{
	/*
	 * Order matters to this EC: on the AC100, key events only started
	 * flowing after the mouse reset plus a (second) keyboard enable, so we
	 * send what Linux sends and what the old U-Boot driver sent, in that
	 * order.
	 */
	static const u8 global_events[] = { NVEC_SLEEP, SLEEP_GLOBAL_EVENTS, 1 };
	static const u8 cnfg_wake[] = { NVEC_KBD, KBD_CNFG_WAKE, 1, 1 };
	static const u8 cnfg_wake_rep[] = { NVEC_KBD, KBD_CNFG_WAKE_KEY_REPORTING, 1 };
	static const u8 mouse_reset[] = { NVEC_PS2, PS2_SEND_CMD, PS2_RESET, 3 };
	static const u8 enable_kbd[] = { NVEC_KBD, KBD_ENABLE };
	static const u8 clear_leds[] = { NVEC_KBD, KBD_SET_LEDS, 0 };

	nvec_request(priv, false);
	udelay(100);
	nvec_init_i2c_slave(priv);
	priv->state = 0;
	priv->npressed = 0;
	priv->nstrikes = 0;

	if (nvec_write(priv, enable_kbd, sizeof(enable_kbd)))
		return -EIO;
	nvec_write(priv, global_events, sizeof(global_events));
	nvec_write(priv, cnfg_wake, sizeof(cnfg_wake));
	nvec_write(priv, cnfg_wake_rep, sizeof(cnfg_wake_rep));
	nvec_write(priv, mouse_reset, sizeof(mouse_reset));
	nvec_write(priv, enable_kbd, sizeof(enable_kbd));
	nvec_write(priv, clear_leds, sizeof(clear_leds));
	priv->last_rx_ms = get_timer(0);

	return 0;
}

static void nvec_dump_log(struct nvec_priv *priv)
{
	uint i, j, first = priv->log_n > NVEC_LOG ? priv->log_n - NVEC_LOG : 0;

	printf("EC messages so far (%u):\n", priv->log_n);
	for (i = first; i < priv->log_n; i++) {
		printf("  #%u [%d]:", i, priv->log_len[i % NVEC_LOG]);
		for (j = 0; j < priv->log_len[i % NVEC_LOG]; j++)
			printf(" %02x", priv->log[i % NVEC_LOG][j]);
		printf("\n");
	}
}

/* Ask the EC something harmless. Returns 0 if it answered. */
static int nvec_ping(struct nvec_priv *priv, uint timeout_ms)
{
	static const u8 fw_version[] = { NVEC_CNTL, CNTL_GET_FIRMWARE_VERSION };

	priv->tx[0] = sizeof(fw_version);
	memcpy(&priv->tx[1], fw_version, sizeof(fw_version));
	priv->tx_size = sizeof(fw_version) + 1;
	priv->tx_pos = 0;
	priv->state = 0;
	nvec_request(priv, true);
	if (!nvec_wait(priv, NVEC_EV_TX_DONE, timeout_ms)) {
		nvec_wait(priv, NVEC_EV_RX_DONE, 300);
		return 0;
	}
	nvec_request(priv, false);
	return -ETIMEDOUT;
}

/*
 * The EC has been quiet: either nothing happened, or a transfer from it
 * failed while we were not listening. In the latter case the link stays
 * dead until our controller is reset, so check, and recover if needed.
 */
static void nvec_watchdog(struct nvec_priv *priv)
{
	memset(priv->outstanding, 0, sizeof(priv->outstanding));
	priv->last_rx_ms = get_timer(0);	/* one attempt per interval */

	if (!nvec_ping(priv, 300))
		return;				/* alive, just quiet */

	priv->n_pingfail++;
	priv->n_reinit++;
	if (nvec_enable(priv))
		printf("nvec: EC link lost, re-init failed, will retry\n");
	else
		printf("[nvec link reset #%u; longest poll gap since last: %lu ms; >20ms:%u >50ms:%u >200ms:%u]\n",
		       priv->n_reinit, priv->max_gap_ms, priv->gaps[1],
		       priv->gaps[2], priv->gaps[3]);
	priv->max_gap_ms = 0;
	priv->last_rx_ms = get_timer(0);
}

static int nvec_kbd_probe(struct udevice *dev)
{
	struct nvec_priv *priv = dev_get_priv(dev);
	struct keyboard_priv *uc_priv = dev_get_uclass_priv(dev);
	struct stdio_dev *sdev = &uc_priv->sdev;
	struct input_config *input = &uc_priv->input;
	int ret;

	nvec_dbg_priv = priv;

	priv->base = dev_read_addr_ptr(dev);
	if (!priv->base)
		return -EINVAL;
	priv->i2c_addr = dev_read_u32_default(dev, "slave-addr", 0x8a);
	priv->clk_hz = dev_read_u32_default(dev, "clock-frequency", 80000);

	ret = gpio_request_by_name(dev, "request-gpios", 0,
				   &priv->request_gpio, GPIOD_IS_OUT);
	if (ret) {
		printf("nvec: no request-gpios (%d)\n", ret);
		return ret;
	}
	if (nvec_enable(priv)) {
		printf("nvec: EC does not answer, internal keyboard disabled\n");
		priv->dead = true;
	} else {
		printf("nvec: internal keyboard enabled\n");
	}

	input_set_delays(input, KBC_REPEAT_DELAY_MS, KBC_REPEAT_RATE_MS);
	input_add_tables(input, false);
	priv->input = input;
	input->dev = dev;
	input->read_keys = nvec_kbd_check;
	strcpy(sdev->name, "nvec-kbc");

	return input_stdio_register(sdev);
}

/*
 * Before the OS starts: stop acting as an I2C slave. Otherwise an EC event
 * sent during the kernel's first seconds finds a slave that never reads the
 * byte, and the bus hangs until the kernel driver resets the controller.
 * Leave the bus as the kernel driver expects to find it: no slave at all.
 */
static int nvec_kbd_remove(struct udevice *dev)
{
	struct nvec_priv *priv = dev_get_priv(dev);

	nvec_request(priv, false);
	nvec_writel(0, priv->base + I2C_SL_CNFG);	/* slave off */
	reset_periph(PERIPH_ID_I2C3, 2);
	priv->dead = true;

	return 0;
}

static const struct keyboard_ops nvec_kbd_ops = {
	.start	= nvec_kbd_start,
};

static const struct udevice_id nvec_kbd_ids[] = {
	{ .compatible = "nvidia,nvec" },
	{ }
};

U_BOOT_DRIVER(tegra_nvec_kbd) = {
	.name		= "tegra_nvec_kbd",
	.id		= UCLASS_KEYBOARD,
	.of_match	= nvec_kbd_ids,
	.probe		= nvec_kbd_probe,
	.remove		= nvec_kbd_remove,
	.ops		= &nvec_kbd_ops,
	.priv_auto	= sizeof(struct nvec_priv),
	.flags		= DM_FLAG_OS_PREPARE,
};

#ifdef CONFIG_CMD_NVECDBG
#include <command.h>
#include <console.h>
#include <env.h>

static void nvec_stage(struct nvec_priv *priv, const char *name, uint delay,
		       uint secs)
{
	ulong start = get_timer(0);
	uint p0 = priv->n_press, r0 = priv->n_release, rx0 = priv->n_rx;
	uint bad0 = priv->n_bad;
	int alive;

	printf("== %s (%u ms stall after each message): type for %u s\n",
	       name, delay, secs);
	priv->test_delay_ms = delay;
	while (get_timer(start) < secs * 1000) {
		if (nvec_step(priv) == NVEC_EV_NONE)
			udelay(100);
	}
	priv->test_delay_ms = 0;
	alive = !nvec_ping(priv, 1000);
	printf("   %u msgs, %u presses, %u releases, %u bad, link %s\n",
	       priv->n_rx - rx0, priv->n_press - p0, priv->n_release - r0,
	       priv->n_bad - bad0, alive ? "OK" : "LOST");
	if (!alive) {
		priv->n_reinit++;
		printf("   re-init: %s\n", nvec_enable(priv) ? "FAILED" : "ok");
	}
	priv->nstrikes = 0;
	memset(priv->outstanding, 0, sizeof(priv->outstanding));
	priv->last_rx_ms = get_timer(0);
}

static int do_nvecdbg(struct cmd_tbl *cmdtp, int flag, int argc,
		      char *const argv[])
{
	struct nvec_priv *priv = nvec_dbg_priv;
	uint secs = argc > 1 ? dectoul(argv[1], NULL) : 6;
	int i;

	if (!priv) {
		printf("nvec: driver never probed\n");
		return 1;
	}
	printf("nvec: rx=%u keys=%u bad=%u reinits=%u pingfail=%u\n",
	       priv->n_rx, priv->n_key, priv->n_bad, priv->n_reinit,
	       priv->n_pingfail);
	printf("poll gaps: max %lu ms; >5ms:%u >20ms:%u >50ms:%u >200ms:%u\n",
	       priv->max_gap_ms, priv->gaps[0], priv->gaps[1], priv->gaps[2],
	       priv->gaps[3]);
	for (i = 0; i < 4 && i < priv->n_bad; i++)
		printf("  bad: state %d status %02x\n", priv->bad_state[i],
		       priv->bad_status[i]);
	priv->max_gap_ms = 0;

	printf("Type steadily (e.g. asdfasdf) during every stage.\n");
	nvec_stage(priv, "A", 0, secs);
	nvec_stage(priv, "B", 10, secs);
	nvec_stage(priv, "C", 25, secs);
	nvec_stage(priv, "D", 60, secs);
	nvec_stage(priv, "E", 150, secs);
	nvec_dump_log(priv);

	return 0;
}

U_BOOT_CMD(nvecdbg, 2, 0, do_nvecdbg, "trace the NVEC embedded controller",
	   "[seconds per stage]");
#endif
