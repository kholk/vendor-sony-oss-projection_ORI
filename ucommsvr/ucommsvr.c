/*
 * Micro Communicator for Projection uC
 * a High-Speed Serial communications server
 *
 * Copyright (C) 2018 AngeloGioacchino Del Regno <kholk11@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

//#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <dlfcn.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <termios.h>

#include <private/android_filesystem_config.h>
#include <utils/Log.h>

#include "ucomm_private.h"
#include "ucomm_ext.h"

#define LOG_TAG			"MicroComm"

#define MICRO_UART		"/dev/ttyHS1"

#define SERPARM_BAUDRATE	B115200		/* 115200bps */
#define SERPARM_IFLAGS		(IGNPAR)	/* Parity enable */

				/* 8n1, RX Enabled */
#define SERPARM_CFLAGS		(CS8 | CREAD)
#define SERPARM_OFLAGS		(0)
#define SERPARM_LFLAGS		(0)

#define UNUSED __attribute__((unused))

/* Serial port fd */
static int serport = -1;
static uint8_t cached_light_val = 100;

/* MicroComm Server */
static int sock;
static int clientsock;
static struct sockaddr_un server_addr;
static pthread_t ucommsvr_thread;
static bool ucthread_run = true;

static int sendcmd(int fd, uint8_t cmd[], int cmd_sz,
		const uint8_t reply[], int reply_len)
{
	int i, rc = -1, retry = 0, sz_ans = 32;
	char buf[40];

	do {
		write(fd, cmd, cmd_sz);
		sleep(1);

		ioctl(fd, FIONREAD, &sz_ans);
		if (sz_ans <= 0)
			continue;

		if (sz_ans > 40)
			sz_ans = 40;

		rc = read(fd, buf, sz_ans);
		if (buf[0] == 0x0b &&
		    buf[1] == 0x0e) {
			/* If VT SO received, check reply */
			for (i = 0; i < reply_len; i++) {
				if (buf[i+2] != reply[i])
					rc = -1;
			}

			/* If no reply to check, just go on */
			if (!reply_len)
				rc = 0;
		}
		else
			ALOGE("INVALID RX DATA: 0x%x 0x%x",
				buf[0], buf[1]);
		if (rc == 0)
			return 0;
		retry++;
	} while (retry < 10);

	return -1;
}

#if 0
static int sendcmd(int fd, uint8_t cmd[], int cmd_sz)
{
	int rc, sz_ans = 32;
	char buf[40];

	do {
		write(fd, cmd, cmd_sz);
		sleep(1);

		ioctl(fd, FIONREAD, &sz_ans);
		if (sz_ans <= 0)
			continue;

		if (sz_ans > 40)
			sz_ans = 40;

		rc = read(fd, buf, sz_ans);
		if (buf[0] == 0x0b &&
		    buf[1] == 0x0e)
			return 0;
		else
			ALOGE("INVALID RX DATA: 0x%x 0x%x",
				buf[0], buf[1]);
	} while (1);

	return -1;
}
#endif

uint8_t *__concat_cmd(const uint8_t head[],
			const uint8_t cmd[],
			int head_len, int cmd_len, int *full_sz)
{
	int footer_len = sizeof(std_footer) / sizeof(std_footer[0]);
	uint8_t *full_cmd = NULL;

	*full_sz = (head_len + cmd_len + footer_len) * sizeof(uint8_t);

	full_cmd = malloc(*full_sz);

	if (full_cmd == NULL) {
		ALOGE("Memory exhausted. Cannot allocate.\n");
		return NULL;
	}

	memcpy(full_cmd, head, head_len);
	memcpy(full_cmd + head_len, cmd, cmd_len);
	memcpy(full_cmd + head_len + cmd_len, std_footer, footer_len);

	return full_cmd;
}

int send_concat_cmd(int fd, const uint8_t head[],
			const uint8_t cmd[],
			int head_len, int cmd_len)
{
	int footer_len = sizeof(std_footer) / sizeof(std_footer[0]);
	int full_sz = 0;
	uint8_t *full_cmd = __concat_cmd(head, cmd,
					head_len, cmd_len, &full_sz);
	if (full_cmd == NULL)
		return -1;

	return sendcmd(fd, full_cmd, full_sz, cmd_reply_nul, 0);
}

int send_init_sequence(int fd)
{
	int rc;
	int head_len = sizeof(std_header) / sizeof(std_header[0]);
	int cmd_len;

	cmd_len = sizeof(cmd_init_hello) / sizeof(cmd_init_hello[0]);
	rc = send_concat_cmd(fd, std_header, cmd_init_hello,
				head_len, cmd_len);
	if (rc)
		goto end;

	cmd_len = sizeof(cmd_init_unk1) / sizeof(cmd_init_unk1[0]);
	rc = send_concat_cmd(fd, std_header, cmd_init_unk1,
				head_len, cmd_len);
	if (rc)
		goto end;

	cmd_len = sizeof(cmd_fan_on) / sizeof(cmd_fan_on[0]);
	rc = send_concat_cmd(fd, std_header, cmd_fan_on,
				head_len, cmd_len);
	if (rc)
		goto end;

	cmd_len = sizeof(cmd_init_unk3) / sizeof(cmd_init_unk3[0]);
	rc = send_concat_cmd(fd, std_header, cmd_init_unk3,
				head_len, cmd_len);
	if (rc)
		goto end;

	cmd_len = sizeof(cmd_init_led) / sizeof(cmd_init_led[0]);
	rc = send_concat_cmd(fd, std_header, cmd_init_led,
				head_len, cmd_len);
end:
	if (rc)
		ALOGE("ERROR: Cannot send init sequence\n");

	return rc;
}

int send_set_brightness(int fd, int brightness)
{
	int rc;
	int head_len = sizeof(std_header) / sizeof(std_header[0]);
	int full_sz;
	int cmd_len;
	uint8_t *full_cmd = NULL;
	uint8_t conv_br = (uint8_t)((((brightness + 1) / 17) * 4) + 40);
	uint8_t control;

	cmd_len = sizeof(cmd_light_lvl) / sizeof(cmd_light_lvl[0]);
	full_cmd = __concat_cmd(std_header, cmd_light_lvl,
				head_len, cmd_len, &full_sz);
	if (full_cmd == NULL)
		return -1;

	/* Paranoid sanity check:
	 * min 40, max 100 for a total of 60 steps */
	if (conv_br > 100)
		conv_br = 100;
	else if (conv_br < 40)
		conv_br = 40;

	full_cmd[head_len + 5] = conv_br;
	full_cmd[head_len + cmd_len - 1] = conv_br + 45;

	rc = sendcmd(fd, full_cmd, full_sz,
			cmd_reply_light_ok, cmd_reply_len);
	if (rc == 0)
		cached_light_val = brightness;

	return rc;
}

int send_power_sequence(int fd, bool poweron)
{
	int rc;
	int head_len = sizeof(std_header) / sizeof(std_header[0]);
	int cmd_len;

	if (poweron) {
		cmd_len = sizeof(cmd_ir_sensor_on) /
				sizeof(cmd_ir_sensor_on[0]);
		rc = send_concat_cmd(fd, std_header, cmd_ir_sensor_on,
					head_len, cmd_len);
		if (rc)
			goto end;

		/* ToDo: When AF implemented, call/restore here */

		rc = send_set_brightness(fd, cached_light_val);
		if (rc)
			goto end;
	} else {
		cmd_len = sizeof(cmd_light_off) /
				sizeof(cmd_light_off[0]);
		rc = send_concat_cmd(fd, std_header, cmd_light_off,
					head_len, cmd_len);
		if (rc)
			goto end;

		cmd_len = sizeof(cmd_ir_sensor_off) /
				sizeof(cmd_ir_sensor_off[0]);
		rc = send_concat_cmd(fd, std_header, cmd_ir_sensor_off,
					head_len, cmd_len);
		if (rc)
			goto end;
	}
end:
	if (rc)
		ALOGE("ERROR: Send power%s sequence FAIL\n",
			poweron ? "on" : "off");

	return rc;
}

int send_set_focus(int fd, int focal)
{
	return -1;
}

int send_set_keystone(int fd, int ksval)
{
	return -1;
}

/*
 * ucomm_dispatch - Recognizes the requested operation and calls
 *		    the appropriate function to dispatch the
 *		    command(s) to the uC.
 *
 * \return Returns success(0) or negative errno.
 */
static int ucomm_dispatch(struct micro_communicator_params *params)
{
	int rc;
	int val = params->value;

	switch (params->operation) {
	case OP_INITIALIZE:
		rc = send_init_sequence(serport);
		break;
	case OP_POWER:
		rc = send_power_sequence(serport, val ? true : false);
		break;
	case OP_BRIGHTNESS:
		rc = send_set_brightness(serport, val);
		break;
	case OP_FOCUS:
		rc = send_set_focus(serport, val);
		break;
	case OP_KEYSTONE:
		rc = send_set_keystone(serport, val);
		break;
	default:
		ALOGE("Invalid operation requested.");
		rc = -1;
	}

	return rc;
}

static void *ucommsvr_looper(void *unusedvar UNUSED)
{
	int ret;
	int32_t microcomm_reply = -EINVAL;
	uint8_t retry;
	socklen_t clientlen = sizeof(struct sockaddr_un);
	struct sockaddr_un client_addr;
	struct micro_communicator_params extparams;

reloop:
	ALOGI("MicroComm Server is waiting for connection...");
	if (clientsock)
		close(clientsock);
	retry = 0;
	while (((clientsock = accept(sock, (struct sockaddr*)&client_addr,
		&clientlen)) > 0) && (ucthread_run == true))
	{
		ret = recv(clientsock, &extparams,
			sizeof(struct micro_communicator_params), 0);
		if (!ret) {
			ALOGE("Cannot receive data from client");
			goto reloop;
		}

		if (ret != sizeof(struct micro_communicator_params)) {
			ALOGE("Received data size mismatch!!");
			goto reloop;
		} else ret = 0;

		microcomm_reply = ucomm_dispatch(&extparams);

retry_send:
		retry++;
		ret = send(clientsock, &microcomm_reply,
			sizeof(microcomm_reply), 0);
		if (ret == -1) {
			microcomm_reply = -EINVAL;
			if (retry < 50)
				goto retry_send;
			ALOGE("ERROR: Cannot send reply!!!");
			goto reloop;
		} else retry = 0;

		if (clientsock)
			close(clientsock);
	}

	ALOGI("MicroComm Server terminated.");
	pthread_exit((void*)((int)0));
}

static int manage_ucommsvr(bool start)
{
	int ret;
	struct stat st = {0};

	if (start == false) {
		ucthread_run = false;
		if (clientsock) {
			shutdown(clientsock, SHUT_RDWR);
			close(clientsock);
		}
		if (sock) {
			shutdown(sock, SHUT_RDWR);
			close(sock);
		}

		return 0;
	}

	ucthread_run = true;

	/* Create folder, if doesn't exist */
	if (stat(UCOMMSERVER_DIR, &st) == -1) {
		mkdir(UCOMMSERVER_DIR, 0773);
	}

	/* Get socket in the UNIX domain */
	sock = socket(PF_UNIX, SOCK_SEQPACKET, 0);
	if (sock < 0) {
		ALOGE("Could not create the socket");
		return -EPROTO;
	}

	/* Create address */
	memset(&server_addr, 0, sizeof(struct sockaddr_un));
	server_addr.sun_family = AF_UNIX;
	strcpy(server_addr.sun_path, UCOMMSERVER_SOCKET);

	/* Free the existing socket file, if any */
	unlink(UCOMMSERVER_SOCKET);

	/* Bind the address to the socket */
	ret = bind(sock, (struct sockaddr*)&server_addr,
			sizeof(struct sockaddr_un));
	if (ret != 0) {
		ALOGE("Cannot bind socket");
		return -EINVAL;
	}

	/* Set socket permissions */
	chown(server_addr.sun_path, AID_ROOT, AID_SYSTEM);
	chmod(server_addr.sun_path, 0666);

	/* Listen on this socket */
	ret = listen(sock, UCOMMSERVER_MAXCONN);
	if (ret != 0) {
		ALOGE("Cannot listen on socket");
		return ret;
	}

	ret = pthread_create(&ucommsvr_thread, NULL, ucommsvr_looper, NULL);
	if (ret != 0) {
		ALOGE("Cannot create MicroComm Server thread");
		return -ENXIO;
	}

	return 0;
}

int main(void)
{
	struct termios tty;
	int rc;

	ALOGI("Initializing MicroComm Server...");

	serport = open(MICRO_UART, (O_RDWR | O_NOCTTY | O_SYNC));

	if (serport < 0) {
		ALOGE("Error: cannot open the serial port.");
		return -1;
	}

	memset(&tty, 0, sizeof tty);

	/* Get current port attributes */
	if (tcgetattr(serport, &tty) != 0) {
		ALOGE("Error: cannot get port attributes\n");
		rc = -1;
		goto err;
	}

	/* Set flags */
	tty.c_cflag = SERPARM_CFLAGS;
	tty.c_iflag = SERPARM_IFLAGS;
	tty.c_oflag = 0;
	tty.c_lflag = 0;

	/* Set communication speed */
        cfsetospeed(&tty, SERPARM_BAUDRATE);
        cfsetispeed(&tty, SERPARM_BAUDRATE);

	/* Flush the comms */
	tcflush(serport, TCIFLUSH);

	/* Send configuration to kernel */
	if (tcsetattr(serport, TCSANOW, &tty) != 0) {
		ALOGE("Error: cannot set port attributes\n");
		rc = -1;
		goto err;
	}
start:
	/* Serial port opened and configured successfully. Start! */
	rc = manage_ucommsvr(true);
	if (rc == 0) {
		ALOGI("MicroComm Server started");
	} else {
		ALOGE("Could not start MicroComm Server");
		goto err;
	}

	pthread_join(ucommsvr_thread, (int**)&rc);
	if (rc == 0)
		goto start;

	return rc;
err:
	close(serport);
	ALOGE("MicroComm Server initialization FAILED.");

	return rc;
}
