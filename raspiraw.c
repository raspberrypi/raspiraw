/*
Copyright (c) 2015, Raspberry Pi Foundation
Copyright (c) 2015, Dave Stevenson
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#define VERSION_STRING "0.0.3"

#define _GNU_SOURCE
#include <ctype.h>
#include <fcntl.h>
#include <libgen.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#define I2C_SLAVE_FORCE 0x0706
#include "bcm_host.h"
#include "interface/vcos/vcos.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/util/mmal_connection.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"

#include "interface/vcsm/user-vcsm.h"

#include "RaspiCLI.h"

#include <sys/ioctl.h>

#include "raw_header.h"

#define DEFAULT_I2C_DEVICE 10
#define ALT_I2C_DEVICE	   0

#define I2C_DEVICE_NAME_LEN 13 // "/dev/i2c-XXX"+NULL

struct brcm_raw_header *brcm_header = NULL;

enum bayer_order
{
	// Carefully ordered so that an hflip is ^1,
	// and a vflip is ^2.
	BAYER_ORDER_BGGR,
	BAYER_ORDER_GBRG,
	BAYER_ORDER_GRBG,
	BAYER_ORDER_RGGB
};

struct sensor_regs
{
	uint16_t reg;
	uint16_t data;
};

struct mode_def
{
	struct sensor_regs *regs;
	int num_regs;
	int width;
	int height;
	MMAL_FOURCC_T encoding;
	enum bayer_order order;
	int native_bit_depth;
	uint8_t image_id;
	uint8_t data_lanes;
	int min_vts;
	int line_time_ns;
	uint32_t timing[5];
	uint32_t term[2];
	int black_level;

	int binning; /* Binning or skipping factor */
};

struct raspiraw_crop
{
	int hinc;
	int vinc;
	int width;
	int height;
	int left;
	int top;
};

struct sensor_def
{
	char *name;
	struct sensor_regs *common_init;
	int num_common_init;
	struct mode_def *modes;
	int num_modes;
	struct sensor_regs *stop;
	int num_stop_regs;

	uint8_t i2c_addr;   // Device I2C slave address
	int i2c_addressing; // Length of register address values
	int i2c_data_size;  // Length of register data to write

	//  Detecting the device
	int i2c_ident_length;	  // Length of I2C ID register
	uint16_t i2c_ident_reg;	  // ID register address
	uint16_t i2c_ident_value; // ID register value

	// Flip configuration
	uint16_t vflip_reg;		   // Register for VFlip
	int vflip_reg_bit;		   // Bit in that register for VFlip
	uint16_t hflip_reg;		   // Register for HFlip
	int hflip_reg_bit;		   // Bit in that register for HFlip
	int flips_dont_change_bayer_order; // Some sensors do not change the
					   // Bayer order by adjusting X/Y
					   // starts to compensate.

	uint16_t exposure_reg;
	int exposure_reg_num_bits;

	uint16_t vts_reg;
	int vts_reg_num_bits;

	uint16_t gain_reg;
	int gain_reg_num_bits;

	int (*set_crop)(const struct sensor_def *, struct mode_def *, const struct raspiraw_crop *cfg);
};

// The process first loads the cleaned up dump of the registers
// than updates the known registers to the proper values
// based on: http://www.seeedstudio.com/wiki/images/3/3c/Ov5647_full.pdf
enum operation
{
	EQUAL, // Set bit to value
	SET,   // Set bit
	CLEAR, // Clear bit
	XOR    // Xor bit
};

void modReg(struct mode_def *mode, uint16_t reg, int startBit, int endBit, int value, enum operation op);

#define NUM_ELEMENTS(a) (sizeof(a) / sizeof(a[0]))

#include "adv7282m_modes.h"
#include "imx219_modes.h"
#include "imx477_modes.h"
#include "ov5647_modes.h"

const struct sensor_def *sensors[] = { &ov5647, &imx219, &adv7282, &imx477, NULL };

enum
{
	CommandHelp,
	CommandMode,
	CommandHFlip,
	CommandVFlip,
	CommandExposure,
	CommandGain,
	CommandOutput,
	CommandWriteHeader,
	CommandTimeout,
	CommandSaveRate,
	CommandBitDepth,
	CommandCameraNum,
	CommandExposureus,
	CommandI2cBus,
	CommandAwbGains,
	CommandRegs,
	CommandHinc,
	CommandVinc,
	CommandFps,
	CommandWidth,
	CommandHeight,
	CommandLeft,
	CommandTop,
	CommandVts,
	CommandLine,
	CommandWriteHeader0,
	CommandWriteHeaderG,
	CommandWriteTimestamps,
	CommandWriteEmpty,
	CommandDecodeMetadata,
	CommandAwb,
	CommandNoPreview,
	CommandProcessing,
	CommandPreview,
	CommandFullScreen,
	CommandOpacity,
	CommandProcessingYUV,
	CommandOutputYUV,
};

static COMMAND_LIST cmdline_commands[] = {
	// clang-format off
	{ CommandHelp,            "-help",           "?",    "This help information", 0 },
	{ CommandMode,            "-mode",           "md",   "Set sensor mode <mode>", 1 },
	{ CommandHFlip,           "-hflip",          "hf",   "Set horizontal flip", 0 },
	{ CommandVFlip,           "-vflip",          "vf",   "Set vertical flip", 0 },
	{ CommandExposure,        "-ss",             "e",    "Set the sensor exposure time (not calibrated units)", 0 },
	{ CommandGain,            "-gain",           "g",    "Set the sensor gain code (not calibrated units)", 0 },
	{ CommandOutput,          "-output",         "o",    "Set the output filename", 0 },
	{ CommandWriteHeader,     "-header",         "hd",   "Write the BRCM header to the output file", 0 },
	{ CommandTimeout,         "-timeout",        "t",    "Time (in ms) before shutting down (if not specified, set to 5s)", 1 },
	{ CommandSaveRate,        "-saverate",       "sr",   "Save every Nth frame", 1 },
	{ CommandBitDepth,        "-bitdepth",       "b",    "Set output raw bit depth (8, 10, 12 or 16, if not specified, set to sensor native)", 1 },
	{ CommandCameraNum,       "-cameranum",      "c",    "Set camera number to use (0=CAM0, 1=CAM1).", 1 },
	{ CommandExposureus,      "-expus",          "eus",  "Set the sensor exposure time in micro seconds.", -1 },
	{ CommandI2cBus,          "-i2c",            "y",    "Set the I2C bus to use.", -1 },
	{ CommandAwbGains,        "-awbgains",       "awbg", "Set the AWB gains to use.", 1 },
	{ CommandRegs,            "-regs",           "r",    "Change (current mode) regs", 0 },
	{ CommandHinc,            "-hinc",           "hi",   "Set horizontal odd/even inc reg", -1 },
	{ CommandVinc,            "-vinc",           "vi",   "Set vertical odd/even inc reg", -1 },
	{ CommandFps,             "-fps",            "f",    "Set framerate regs", -1 },
	{ CommandWidth,           "-width",          "w",    "Set current mode width", -1 },
	{ CommandHeight,          "-height",         "h",    "Set current mode height", -1 },
	{ CommandLeft,            "-left",           "lt",   "Set current mode left", -1 },
	{ CommandTop,             "-top",            "tp",   "Set current mode top", -1 },
	{ CommandWriteHeader0,    "-header0",        "hd0",  "Sets filename to write the BRCM header to", 0 },
	{ CommandWriteHeaderG,    "-headerg",        "hdg",  "Sets filename to write the .pgm header to", 0 },
	{ CommandWriteTimestamps, "-tstamps",        "ts",   "Sets filename to write timestamps to", 0 },
	{ CommandWriteEmpty,      "-empty",          "emp",  "Write empty output files", 0 },
	{ CommandDecodeMetadata,  "-metadata",       "m",    "Decode register metadata", 0 },
	{ CommandAwb,             "-awb",            "awb",  "Use a simple grey-world AWB algorithm", 0 },
	{ CommandNoPreview,       "-nopreview",      "n",    "Do not send the stream to the display", 0 },
	{ CommandProcessing,      "-processing",     "P",    "Pass images into an image processing function", 0 },
	{ CommandPreview,         "-preview",        "p",    "Preview window settings <'x,y,w,h'>", 1 },
	{ CommandFullScreen,      "-fullscreen",     "fs",   "Fullscreen preview mode", 0 },
	{ CommandOpacity,         "-opacity",        "op",   "Preview window opacity (0-255)", 1 },
	{ CommandProcessingYUV,   "-processing_yuv", "PY",   "Pass processed YUV images into an image processing function", 0 },
	{ CommandOutputYUV,       "-output_yuv", "oY", "Set the output filename for YUV data", 0 },
	// clang-format on
};

static int cmdline_commands_size = sizeof(cmdline_commands) / sizeof(cmdline_commands[0]);

typedef struct pts_node
{
	int idx;
	int64_t pts;
	struct pts_node *nxt;
} * PTS_NODE_T;

#define DEFAULT_PREVIEW_LAYER 3

typedef struct raspiraw_params
{
	struct raspiraw_crop crop;
	int mode;
	int hflip;
	int vflip;
	int exposure;
	int gain;
	char *output;
	int capture;
	int write_header;
	int timeout;
	int saverate;
	int bit_depth;
	int camera_num;
	int exposure_us;
	int i2c_bus;
	double awb_gains_r;
	double awb_gains_b;
	char *regs;
	double fps;
	char *write_header0;
	char *write_headerg;
	char *write_timestamps;
	int write_empty;
	PTS_NODE_T ptsa;
	PTS_NODE_T ptso;
	int decodemetadata;
	int awb;
	int no_preview;
	int processing;
	int fullscreen; // 0 is use previewRect, non-zero to use full screen
	int opacity;	// Opacity of window - 0 = transparent, 255 = opaque
	MMAL_RECT_T
	preview_window; // Destination rectangle for the preview window.
	int capture_yuv;
	char *output_yuv;
	int processing_yuv;
} RASPIRAW_PARAMS_T;

typedef struct
{
	RASPIRAW_PARAMS_T *cfg;

	MMAL_POOL_T *rawcam_pool;
	MMAL_PORT_T *rawcam_output;

	MMAL_POOL_T *isp_ip_pool;
	MMAL_PORT_T *isp_ip;

	MMAL_QUEUE_T *awb_queue;
	int awb_thread_quit;
	MMAL_PARAMETER_AWB_GAINS_T wb_gains;

	MMAL_QUEUE_T *processing_queue;
	int processing_thread_quit;
} RASPIRAW_CALLBACK_T;

typedef struct
{
	RASPIRAW_PARAMS_T *cfg;

	MMAL_POOL_T *isp_op_pool;
	MMAL_PORT_T *isp_output;

	MMAL_POOL_T *vr_ip_pool;
	MMAL_PORT_T *vr_ip;

	MMAL_QUEUE_T *processing_yuv_queue;
	int processing_yuv_thread_quit;
} RASPIRAW_ISP_CALLBACK_T;

void update_regs(const struct sensor_def *sensor, struct mode_def *mode, int hflip, int vflip, int exposure, int gain);

static int i2c_rd(int fd, uint8_t i2c_addr, uint16_t reg, uint8_t *values, uint32_t n, const struct sensor_def *sensor)
{
	int err;
	uint8_t buf[2] = { reg >> 8, reg & 0xff };
	struct i2c_rdwr_ioctl_data msgset;
	struct i2c_msg msgs[2] = {
		{
		    .addr = i2c_addr,
		    .flags = 0,
		    .len = 2,
		    .buf = buf,
		},
		{
		    .addr = i2c_addr,
		    .flags = I2C_M_RD,
		    .len = n,
		    .buf = values,
		},
	};

	if (sensor->i2c_addressing == 1)
	{
		msgs[0].len = 1;
	}
	msgset.msgs = msgs;
	msgset.nmsgs = 2;

	err = ioctl(fd, I2C_RDWR, &msgset);
	// vcos_log_error("Read i2c addr %02X, reg %04X (len %d), value %02X,
	// err %d", i2c_addr, msgs[0].buf[0], msgs[0].len, values[0], err);
	if (err != (int)msgset.nmsgs)
		return -1;

	return 0;
}

const struct sensor_def *probe_sensor(int fd)
{
	const struct sensor_def **sensor_list = &sensors[0];
	const struct sensor_def *sensor = NULL;

	while (*sensor_list != NULL)
	{
		uint16_t reg = 0;
		sensor = *sensor_list;
		vcos_log_error("Probing sensor %s on addr %02X", sensor->name, sensor->i2c_addr);
		if (sensor->i2c_ident_length <= 2)
		{
			if (!i2c_rd(fd, sensor->i2c_addr, sensor->i2c_ident_reg, (uint8_t *)&reg,
				    sensor->i2c_ident_length, sensor))
			{
				if (reg == sensor->i2c_ident_value)
				{
					vcos_log_error("Found sensor %s at address %02X", sensor->name,
						       sensor->i2c_addr);
					break;
				}
			}
		}
		sensor_list++;
		sensor = NULL;
	}
	return sensor;
}

void send_regs(int fd, const struct sensor_def *sensor, const struct sensor_regs *regs, int num_regs)
{
	int i;
	for (i = 0; i < num_regs; i++)
	{
		if (regs[i].reg == 0xFFFF)
		{
			if (ioctl(fd, I2C_SLAVE_FORCE, regs[i].data) < 0)
			{
				vcos_log_error("Failed to set I2C address to %02X", regs[i].data);
			}
		}
		else if (regs[i].reg == 0xFFFE)
		{
			vcos_sleep(regs[i].data);
		}
		else
		{
			if (sensor->i2c_addressing == 1)
			{
				unsigned char msg[3] = { regs[i].reg, regs[i].data & 0xFF };
				int len = 2;

				if (sensor->i2c_data_size == 2)
				{
					msg[1] = (regs[i].data >> 8) & 0xFF;
					msg[2] = regs[i].data & 0xFF;
					len = 3;
				}
				if (write(fd, msg, len) != len)
				{
					vcos_log_error("Failed to write register index %d "
						       "(%02X val %02X)",
						       i, regs[i].reg, regs[i].data);
				}
			}
			else
			{
				unsigned char msg[4] = { regs[i].reg >> 8, regs[i].reg, regs[i].data };
				int len = 3;

				if (sensor->i2c_data_size == 2)
				{
					msg[2] = regs[i].data >> 8;
					msg[3] = regs[i].data;
					len = 4;
				}
				if (write(fd, msg, len) != len)
				{
					vcos_log_error("Failed to write register index %d", i);
				}
			}
		}
	}
}

void start_camera_streaming(const struct sensor_def *sensor, struct mode_def *mode, int fd)
{
	if (ioctl(fd, I2C_SLAVE_FORCE, sensor->i2c_addr) < 0)
	{
		vcos_log_error("Failed to set I2C address");
		return;
	}
	if (sensor->common_init)
		send_regs(fd, sensor, sensor->common_init, sensor->num_common_init);
	send_regs(fd, sensor, mode->regs, mode->num_regs);
	vcos_log_error("Now streaming...");
}

void stop_camera_streaming(const struct sensor_def *sensor, int fd)
{
	if (ioctl(fd, I2C_SLAVE_FORCE, sensor->i2c_addr) < 0)
	{
		vcos_log_error("Failed to set I2C address");
		return;
	}
	send_regs(fd, sensor, sensor->stop, sensor->num_stop_regs);
}

/**
 * Allocates and generates a filename based on the
 * user-supplied pattern and the frame number.
 * On successful return, finalName and tempName point to malloc()ed strings
 * which must be freed externally.  (On failure, returns nulls that
 * don't need free()ing.)
 *
 * @param finalName pointer receives an
 * @param pattern sprintf pattern with %d to be replaced by frame
 * @param frame for timelapse, the frame number
 * @return Returns a MMAL_STATUS_T giving result of operation
 */

MMAL_STATUS_T create_filenames(char **finalName, char *pattern, int frame)
{
	*finalName = NULL;
	if (0 > asprintf(finalName, pattern, frame))
	{
		return MMAL_ENOMEM; // It may be some other error, but it is not
				    // worth getting it right
	}
	return MMAL_SUCCESS;
}

void decodemetadataline(uint8_t *data, int bpp)
{
	int c = 1;
	uint8_t tag, dta;
	uint16_t reg = -1;

	if (data[0] == 0x0a)
	{

		while (data[c] != 0x07)
		{
			tag = data[c++];
			if (bpp == 10 && (c % 5) == 4)
				c++;
			if (bpp == 12 && (c % 3) == 2)
				c++;
			dta = data[c++];

			if (tag == 0xaa)
				reg = (reg & 0x00ff) | (dta << 8);
			else if (tag == 0xa5)
				reg = (reg & 0xff00) | dta;
			else if (tag == 0x5a)
				vcos_log_error("Register 0x%04x = 0x%02x", reg++, dta);
			else if (tag == 0x55)
				vcos_log_error("Skip     0x%04x", reg++);
			else
				vcos_log_error("Metadata decode failed %x %x %x", reg, tag, dta);
		}
	}
	else
		vcos_log_error("Doesn't looks like register set %x!=0x0a", data[0]);
}

int encoding_to_bpp(uint32_t encoding)
{
	switch (encoding)
	{
	case MMAL_ENCODING_BAYER_SBGGR10P:
	case MMAL_ENCODING_BAYER_SGBRG10P:
	case MMAL_ENCODING_BAYER_SGRBG10P:
	case MMAL_ENCODING_BAYER_SRGGB10P:
		return 10;
	case MMAL_ENCODING_BAYER_SBGGR12P:
	case MMAL_ENCODING_BAYER_SGBRG12P:
	case MMAL_ENCODING_BAYER_SGRBG12P:
	case MMAL_ENCODING_BAYER_SRGGB12P:
		return 12;
	default:
		return 8;
	};
}

static void buffers_to_rawcam(RASPIRAW_CALLBACK_T *dev)
{
	MMAL_BUFFER_HEADER_T *buffer;

	while ((buffer = mmal_queue_get(dev->rawcam_pool->queue)) != NULL)
	{
		mmal_port_send_buffer(dev->rawcam_output, buffer);
		// vcos_log_error("Buffer %p to rawcam\n", buffer);
	}
}

static void callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
	static int count = 0;
	RASPIRAW_CALLBACK_T *dev = (RASPIRAW_CALLBACK_T *)port->userdata;
	RASPIRAW_PARAMS_T *cfg = (RASPIRAW_PARAMS_T *)dev->cfg;
	MMAL_STATUS_T status;

	// vcos_log_error("Buffer %p returned, filled %d, timestamp %llu, flags %04X", buffer, buffer->length,
	// buffer->pts, buffer->flags);
	if (cfg->capture)
	{

		if (!(buffer->flags & MMAL_BUFFER_HEADER_FLAG_CODECSIDEINFO) && (((count++) % cfg->saverate) == 0))
		{
			// Save every Nth frame
			// SD card access is too slow to do much more.
			FILE *file;
			char *filename = NULL;
			if (create_filenames(&filename, cfg->output, count) == MMAL_SUCCESS)
			{
				file = fopen(filename, "wb");
				if (file)
				{
					if (cfg->ptso) // make sure previous
						       // malloc() was
						       // successful
					{
						cfg->ptso->idx = count;
						cfg->ptso->pts = buffer->pts;
						cfg->ptso->nxt = malloc(sizeof(*cfg->ptso->nxt));
						cfg->ptso = cfg->ptso->nxt;
					}
					if (!cfg->write_empty)
					{
						if (cfg->write_header)
							fwrite(brcm_header, BRCM_RAW_HEADER_LENGTH, 1, file);
						fwrite(buffer->user_data, buffer->length, 1, file);
					}
					fclose(file);
				}
				free(filename);
			}
		}
	}

	if (cfg->decodemetadata && (buffer->flags & MMAL_BUFFER_HEADER_FLAG_CODECSIDEINFO))
	{
		int bpp = encoding_to_bpp(port->format->encoding);
		int pitch = mmal_encoding_width_to_stride(port->format->encoding, port->format->es->video.width);

		vcos_log_error("First metadata line");
		decodemetadataline(buffer->user_data, bpp);
		vcos_log_error("Second metadata line");
		decodemetadataline(buffer->user_data + pitch, bpp);
	}

	/* Pass the buffers off to any other MMAL sinks. */
	if (buffer->length && !(buffer->flags & MMAL_BUFFER_HEADER_FLAG_CODECSIDEINFO))
	{
		if (dev->isp_ip)
		{
			MMAL_BUFFER_HEADER_T *out = mmal_queue_get(dev->isp_ip_pool->queue);
			if (out)
			{
				// vcos_log_error("replicate buffer %p for isp", buffer);
				mmal_buffer_header_replicate(out, buffer);
				out->data = buffer->data;
				out->alloc_size = buffer->alloc_size;
				status = mmal_port_send_buffer(dev->isp_ip, out);
				if (status != MMAL_SUCCESS)
					vcos_log_error("Failed to send buffer "
						       "%p to isp - %d",
						       buffer, status);
			}
		}

		/* Pass to the AWB thread */
		if (dev->awb_queue)
		{
			/* Relying on the AWB thread to release this
			 * buffer`refcount */
			mmal_buffer_header_acquire(buffer);
			mmal_queue_put(dev->awb_queue, buffer);
			// vcos_log_error("send buffer %p to awb", buffer);
		}

		/* Pass to the processing thread */
		if (dev->processing_queue)
		{
			/* Relying on the processing thread to release this
			 * buffer`refcount */
			mmal_buffer_header_acquire(buffer);
			mmal_queue_put(dev->processing_queue, buffer);
			// vcos_log_error("send buffer %p to awb", buffer);
		}
	}

	mmal_buffer_header_release(buffer);

	buffers_to_rawcam(dev);
}

static void buffers_to_isp_op(RASPIRAW_ISP_CALLBACK_T *dev)
{
	MMAL_BUFFER_HEADER_T *buffer;

	while ((buffer = mmal_queue_get(dev->isp_op_pool->queue)) != NULL)
	{
		mmal_port_send_buffer(dev->isp_output, buffer);
		// vcos_log_error("Buffer %p to isp op\n", buffer);
	}
}

static void yuv_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
	static int count = 0;
	RASPIRAW_ISP_CALLBACK_T *yuv_cb = (RASPIRAW_ISP_CALLBACK_T *)port->userdata;
	RASPIRAW_PARAMS_T *cfg = (RASPIRAW_PARAMS_T *)yuv_cb->cfg;
	MMAL_STATUS_T status;

	// vcos_log_error("Buffer %p returned, filled %d, timestamp %llu, flags
	// %04X", buffer, buffer->length, buffer->pts, buffer->flags);
	if (cfg->capture_yuv)
	{

		if (!(buffer->flags & MMAL_BUFFER_HEADER_FLAG_CODECSIDEINFO) && (((count++) % cfg->saverate) == 0))
		{
			// Save every Nth frame
			// SD card access is too slow to do much more.
			FILE *file;
			char *filename = NULL;
			if (create_filenames(&filename, cfg->output_yuv, count) == MMAL_SUCCESS)
			{
				file = fopen(filename, "wb");
				if (file)
				{
					if (!cfg->write_empty)
					{
						fwrite(buffer->user_data, buffer->length, 1, file);
					}
					fclose(file);
				}
				free(filename);
			}
		}
	}

	/* Pass the buffers off to any other MMAL sinks. */
	if (buffer->length && !(buffer->flags & MMAL_BUFFER_HEADER_FLAG_CODECSIDEINFO))
	{
		if (yuv_cb->vr_ip)
		{
			MMAL_BUFFER_HEADER_T *out = mmal_queue_get(yuv_cb->vr_ip_pool->queue);
			if (out)
			{
				// vcos_log_error("replicate buffer %p for vr",
				// buffer);
				mmal_buffer_header_replicate(out, buffer);
				out->data = buffer->data;
				out->alloc_size = buffer->alloc_size;
				status = mmal_port_send_buffer(yuv_cb->vr_ip, out);
				if (status != MMAL_SUCCESS)
					vcos_log_error("Failed to send buffer %p to video "
						       "render - %d",
						       buffer, status);
			}
		}

		/* Pass to the processing thread */
		if (yuv_cb->processing_yuv_queue)
		{
			/* Relying on the processing thread to release this
			 * buffer's refcount */
			mmal_buffer_header_acquire(buffer);
			mmal_queue_put(yuv_cb->processing_yuv_queue, buffer);
			// vcos_log_error("send buffer %p to yuv processing",
			// buffer);
		}
	}

	mmal_buffer_header_release(buffer);

	buffers_to_isp_op(yuv_cb);
}

uint32_t order_and_bit_depth_to_encoding(enum bayer_order order, int bit_depth)
{
	// BAYER_ORDER_BGGR,
	// BAYER_ORDER_GBRG,
	// BAYER_ORDER_GRBG,
	// BAYER_ORDER_RGGB
	const uint32_t depth8[] = { MMAL_ENCODING_BAYER_SBGGR8, MMAL_ENCODING_BAYER_SGBRG8, MMAL_ENCODING_BAYER_SGRBG8,
				    MMAL_ENCODING_BAYER_SRGGB8 };
	const uint32_t depth10[] = { MMAL_ENCODING_BAYER_SBGGR10P, MMAL_ENCODING_BAYER_SGBRG10P,
				     MMAL_ENCODING_BAYER_SGRBG10P, MMAL_ENCODING_BAYER_SRGGB10P };
	const uint32_t depth12[] = {
		MMAL_ENCODING_BAYER_SBGGR12P,
		MMAL_ENCODING_BAYER_SGBRG12P,
		MMAL_ENCODING_BAYER_SGRBG12P,
		MMAL_ENCODING_BAYER_SRGGB12P,
	};
	const uint32_t depth16[] = {
		MMAL_ENCODING_BAYER_SBGGR16,
		MMAL_ENCODING_BAYER_SGBRG16,
		MMAL_ENCODING_BAYER_SGRBG16,
		MMAL_ENCODING_BAYER_SRGGB16,
	};
	if (order < 0 || order > 3)
	{
		vcos_log_error("order out of range - %d", order);
		return 0;
	}

	switch (bit_depth)
	{
	case 8:
		return depth8[order];
	case 10:
		return depth10[order];
	case 12:
		return depth12[order];
	case 16:
		return depth16[order];
	}
	vcos_log_error("%d not one of the handled bit depths", bit_depth);
	return 0;
}

/**
 * Parse the incoming command line and put resulting parameters in to the state
 *
 * @param argc Number of arguments in command line
 * @param argv Array of pointers to strings from command line
 * @param state Pointer to state structure to assign any discovered parameters
 * to
 * @return non-0 if failed for some reason, 0 otherwise
 */
static int parse_cmdline(int argc, char **argv, RASPIRAW_PARAMS_T *cfg)
{
	// Parse the command line arguments.
	// We are looking for --<something> or -<abbreviation of something>

	int valid = 1;
	int i;

	for (i = 1; i < argc && valid; i++)
	{
		int command_id, num_parameters, len;

		if (!argv[i])
			continue;

		if (argv[i][0] != '-')
		{
			valid = 0;
			continue;
		}

		// Assume parameter is valid until proven otherwise
		valid = 1;

		command_id =
		    raspicli_get_command_id(cmdline_commands, cmdline_commands_size, &argv[i][1], &num_parameters);

		// If we found a command but are missing a parameter, continue
		// (and we will drop out of the loop)
		if (command_id != -1 && num_parameters > 0 && (i + 1 >= argc))
			continue;

		//  We are now dealing with a command line option
		switch (command_id)
		{
		case CommandHelp:
			raspicli_display_help(cmdline_commands, cmdline_commands_size);
			// exit straight away if help requested
			return -1;

		case CommandMode:
			if (sscanf(argv[i + 1], "%d", &cfg->mode) != 1)
				valid = 0;
			else
				i++;
			break;

		case CommandHFlip:
			cfg->hflip = 1;
			break;

		case CommandVFlip:
			cfg->vflip = 1;
			break;

		case CommandExposure:
			if (sscanf(argv[i + 1], "%d", &cfg->exposure) != 1)
				valid = 0;
			else
				i++;
			break;

		case CommandGain:
			if (sscanf(argv[i + 1], "%d", &cfg->gain) != 1)
				valid = 0;
			else
				i++;
			break;

		case CommandOutput: // output filename
		{
			len = strlen(argv[i + 1]);
			if (len)
			{
				// We use sprintf to append the frame number for
				// timelapse mode Ensure that any %<char> is
				// either %% or %d.
				const char *percent = argv[i + 1];
				while (valid && *percent && (percent = strchr(percent, '%')) != NULL)
				{
					int digits = 0;
					percent++;
					while (isdigit(*percent))
					{
						percent++;
						digits++;
					}
					if (!((*percent == '%' && !digits) || *percent == 'd'))
					{
						valid = 0;
						fprintf(stderr, "Filename contains %% "
								"characters, but not "
								"%%d or %%%% - sorry, "
								"will fail\n");
					}
					percent++;
				}
				cfg->output = malloc(len + 10); // leave enough space for any timelapse
								// generated changes to filename
				if (cfg->output)
				{
					strncpy(cfg->output, argv[i + 1], len + 1);
					i++;
					cfg->capture = 1;
				}
				else
				{
					fprintf(stderr, "internal error - "
							"allocation fail\n");
					valid = 0;
				}
			}
			else
			{
				valid = 0;
			}
			break;
		}

		case CommandWriteHeader:
			cfg->write_header = 1;
			break;

		case CommandTimeout: // Time to run for in milliseconds
			if (sscanf(argv[i + 1], "%u", &cfg->timeout) == 1)
			{
				i++;
			}
			else
				valid = 0;
			break;

		case CommandSaveRate:
			if (sscanf(argv[i + 1], "%u", &cfg->saverate) == 1)
			{
				i++;
			}
			else
				valid = 0;
			break;

		case CommandBitDepth:
			if (sscanf(argv[i + 1], "%u", &cfg->bit_depth) == 1)
			{
				i++;
			}
			else
				valid = 0;
			break;

		case CommandCameraNum:
			if (sscanf(argv[i + 1], "%u", &cfg->camera_num) == 1)
			{
				i++;
				if (cfg->camera_num != 0 && cfg->camera_num != 1)
				{
					fprintf(stderr,
						"Invalid camera number "
						"specified (%d)."
						" It should be 0 or 1.\n",
						cfg->camera_num);
					valid = 0;
				}
			}
			else
				valid = 0;
			break;

		case CommandExposureus:
			if (sscanf(argv[i + 1], "%d", &cfg->exposure_us) != 1)
				valid = 0;
			else
				i++;
			break;

		case CommandI2cBus:
			if (sscanf(argv[i + 1], "%d", &cfg->i2c_bus) != 1)
				valid = 0;
			else
				i++;
			break;

		case CommandAwbGains:
		{
			double r, b;
			int args;

			args = sscanf(argv[i + 1], "%lf,%lf", &r, &b);

			if (args != 2 || r > 8.0 || b > 8.0)
				valid = 0;

			cfg->awb_gains_r = r;
			cfg->awb_gains_b = b;

			i++;
			break;
		}

		case CommandRegs: // register changes
		{
			len = strlen(argv[i + 1]);
			cfg->regs = malloc(len + 1);
			vcos_assert(cfg->regs);
			strncpy(cfg->regs, argv[i + 1], len + 1);
			i++;
			break;
		}

		case CommandHinc:
			if (strlen(argv[i + 1]) != 2 || sscanf(argv[i + 1], "%x", &cfg->crop.hinc) != 1)
				valid = 0;
			else
				i++;
			break;

		case CommandVinc:
			if (strlen(argv[i + 1]) != 2 || sscanf(argv[i + 1], "%x", &cfg->crop.vinc) != 1)
				valid = 0;
			else
				i++;
			break;

		case CommandFps:
			if (sscanf(argv[i + 1], "%lf", &cfg->fps) != 1)
				valid = 0;
			else
				i++;
			break;

		case CommandWidth:
			if (sscanf(argv[i + 1], "%d", &cfg->crop.width) != 1)
				valid = 0;
			else
				i++;
			break;

		case CommandHeight:
			if (sscanf(argv[i + 1], "%d", &cfg->crop.height) != 1)
				valid = 0;
			else
				i++;
			break;

		case CommandLeft:
			if (sscanf(argv[i + 1], "%d", &cfg->crop.left) != 1)
				valid = 0;
			else
				i++;
			break;

		case CommandTop:
			if (sscanf(argv[i + 1], "%d", &cfg->crop.top) != 1)
				valid = 0;
			else
				i++;
			break;

		case CommandWriteHeader0:
			len = strlen(argv[i + 1]);
			cfg->write_header0 = malloc(len + 1);
			vcos_assert(cfg->write_header0);
			strncpy(cfg->write_header0, argv[i + 1], len + 1);
			i++;
			break;

		case CommandWriteHeaderG:
			len = strlen(argv[i + 1]);
			cfg->write_headerg = malloc(len + 1);
			vcos_assert(cfg->write_headerg);
			strncpy(cfg->write_headerg, argv[i + 1], len + 1);
			i++;
			break;

		case CommandWriteTimestamps:
			len = strlen(argv[i + 1]);
			cfg->write_timestamps = malloc(len + 1);
			vcos_assert(cfg->write_timestamps);
			strncpy(cfg->write_timestamps, argv[i + 1], len + 1);
			i++;
			cfg->ptsa = malloc(sizeof(*cfg->ptsa));
			cfg->ptso = cfg->ptsa;
			break;

		case CommandWriteEmpty:
			cfg->write_empty = 1;
			break;

		case CommandDecodeMetadata:
			cfg->decodemetadata = 1;
			break;

		case CommandAwb:
			vcos_log_error("Let's do AWB");
			cfg->awb = 1;
			break;

		case CommandNoPreview:
			cfg->no_preview = 1;
			break;

		case CommandProcessing:
			cfg->processing = 1;
			break;
		case CommandPreview: // Preview window
		{
			int tmp;

			tmp = sscanf(argv[i + 1], "%d,%d,%d,%d", &cfg->preview_window.x, &cfg->preview_window.y,
				     &cfg->preview_window.width, &cfg->preview_window.height);

			// Failed to get any window parameters, so revert to
			// full screen
			if (tmp != 4)
				cfg->fullscreen = 1;
			else
				cfg->fullscreen = 0;

			i++;

			break;
		}

		case CommandFullScreen: // Want full screen preview mode
					// (overrides display rect)
			cfg->fullscreen = 1;

			i++;
			break;

		case CommandOpacity: // Define preview window opacity
			if (sscanf(argv[i + 1], "%u", &cfg->opacity) != 1)
				cfg->opacity = 255;
			else
				i++;
			break;

		case CommandProcessingYUV:
			cfg->processing_yuv = 1;
			break;

		case CommandOutputYUV: // output filename
		{
			len = strlen(argv[i + 1]);
			if (len)
			{
				// We use sprintf to append the frame number for
				// timelapse mode Ensure that any %<char> is
				// either %% or %d.
				const char *percent = argv[i + 1];
				while (valid && *percent && (percent = strchr(percent, '%')) != NULL)
				{
					int digits = 0;
					percent++;
					while (isdigit(*percent))
					{
						percent++;
						digits++;
					}
					if (!((*percent == '%' && !digits) || *percent == 'd'))
					{
						valid = 0;
						fprintf(stderr, "Filename contains %% "
								"characters, but not "
								"%%d or %%%% - sorry, "
								"will fail\n");
					}
					percent++;
				}
				cfg->output_yuv = malloc(len + 10); // leave enough space for any timelapse
								    // generated changes to filename
				if (cfg->output_yuv)
				{
					strncpy(cfg->output_yuv, argv[i + 1], len + 1);
					i++;
					cfg->capture_yuv = 1;
				}
				else
				{
					fprintf(stderr, "internal error - "
							"allocation fail\n");
					valid = 0;
				}
			}
			else
			{
				valid = 0;
			}
			break;
		}

		default:
			valid = 0;
			break;
		}
	}

	if (!valid)
	{
		fprintf(stderr, "Invalid command line option (%s)\n", argv[i - 1]);
		return 1;
	}

	return 0;
}

uint32_t get_pixel(int x, int y, uint32_t encoding, int stride, uint8_t *data)
{
	uint32_t val;
	const uint8_t raw10_masks[4] = { 0x03, 0x0c, 0x30, 0xc0 };
	const uint8_t raw10_shifts[4] = { 0, 2, 4, 6 };
	const uint8_t raw12_masks[2] = { 0x0f, 0xf0 };
	const uint8_t raw12_shifts[2] = { 0, 4 };

	switch (encoding)
	{
	case MMAL_ENCODING_BAYER_SBGGR8:
	case MMAL_ENCODING_BAYER_SGBRG8:
	case MMAL_ENCODING_BAYER_SGRBG8:
	case MMAL_ENCODING_BAYER_SRGGB8:
		val = data[x + (y * stride)];
		break;

	case MMAL_ENCODING_BAYER_SBGGR10P:
	case MMAL_ENCODING_BAYER_SGBRG10P:
	case MMAL_ENCODING_BAYER_SGRBG10P:
	case MMAL_ENCODING_BAYER_SRGGB10P:
		val = data[x + (x / 4) + (y * stride)] << 2;
		val |= (data[x + (x / 4) + 4 + (y * stride)] & raw10_masks[x & 3]) >> raw10_shifts[x & 3];
		break;

	case MMAL_ENCODING_BAYER_SBGGR12P:
	case MMAL_ENCODING_BAYER_SGBRG12P:
	case MMAL_ENCODING_BAYER_SGRBG12P:
	case MMAL_ENCODING_BAYER_SRGGB12P:
		val = data[x + (x / 2) + (y * stride)] << 4;
		val |= (data[x + (x / 2) + 2 + (y * stride)] & raw12_masks[x & 1]) >> raw12_shifts[x & 1];
		break;

	case MMAL_ENCODING_BAYER_SBGGR16:
	case MMAL_ENCODING_BAYER_SGBRG16:
	case MMAL_ENCODING_BAYER_SGRBG16:
	case MMAL_ENCODING_BAYER_SRGGB16:
	{
		/* CHECK ME: are the MSB and LSB the right way around? */
		uint16_t *data16 = (uint16_t *)data;
		val = data16[x + (y * stride / 2)];
		break;
	}
	}
	return val;
}

enum channels
{
	RED,
	GREEN1,
	GREEN2,
	BLUE
};

uint64_t get_channel(enum channels chan, uint32_t encoding, uint64_t *sums)
{
	const int bggr[4] = { BLUE, GREEN1, GREEN2, RED };
	const int rggb[4] = { RED, GREEN1, GREEN2, BLUE };
	const int gbrg[4] = { GREEN1, BLUE, RED, GREEN2 };
	const int grbg[4] = { GREEN1, RED, BLUE, GREEN2 };

	switch (encoding)
	{
	case MMAL_ENCODING_BAYER_SBGGR8:
	case MMAL_ENCODING_BAYER_SBGGR10P:
	case MMAL_ENCODING_BAYER_SBGGR12P:
	case MMAL_ENCODING_BAYER_SBGGR16:
		return sums[bggr[chan]];

	case MMAL_ENCODING_BAYER_SRGGB8:
	case MMAL_ENCODING_BAYER_SRGGB10P:
	case MMAL_ENCODING_BAYER_SRGGB12P:
	case MMAL_ENCODING_BAYER_SRGGB16:
		return sums[rggb[chan]];

	case MMAL_ENCODING_BAYER_SGBRG8:
	case MMAL_ENCODING_BAYER_SGBRG10P:
	case MMAL_ENCODING_BAYER_SGBRG12P:
	case MMAL_ENCODING_BAYER_SGBRG16:
		return sums[gbrg[chan]];

	case MMAL_ENCODING_BAYER_SGRBG8:
	case MMAL_ENCODING_BAYER_SGRBG10P:
	case MMAL_ENCODING_BAYER_SGRBG12P:
	case MMAL_ENCODING_BAYER_SGRBG16:
		return sums[grbg[chan]];

	default:
		return 0;
	}
}

static void run_awb_calcs(RASPIRAW_CALLBACK_T *dev, MMAL_BUFFER_HEADER_T *buffer)
{
	int x, y, count = 0;
	uint64_t sums[4] = { 0 }; // Sums for each of the 4 channels
	int r_ave, g_ave, b_ave;
	uint32_t encoding = dev->rawcam_output->format->encoding;
	int stride = mmal_encoding_width_to_stride(encoding, dev->rawcam_output->format->es->video.width);

	if (!dev->wb_gains.hdr.id)
	{
		dev->wb_gains.hdr.id = MMAL_PARAMETER_CUSTOM_AWB_GAINS;
		dev->wb_gains.hdr.size = sizeof(dev->wb_gains);
		dev->wb_gains.r_gain.num = 256;
		dev->wb_gains.r_gain.den = 256;
		dev->wb_gains.b_gain.num = 256;
		dev->wb_gains.b_gain.den = 256;
	}

	for (x = 8; x < dev->rawcam_output->format->es->video.crop.width; x += 16)
	{
		for (y = 8; y < dev->rawcam_output->format->es->video.crop.height; y += 16)
		{
			sums[0] += get_pixel(x, y, encoding, stride, buffer->user_data);
			sums[1] += get_pixel(x + 1, y, encoding, stride, buffer->user_data);
			sums[2] += get_pixel(x, y + 1, encoding, stride, buffer->user_data);
			sums[3] += get_pixel(x + 1, y + 1, encoding, stride, buffer->user_data);
			count++;
		}
	}

	r_ave = get_channel(RED, encoding, sums) / vcos_max(count, 1);
	g_ave = (get_channel(GREEN1, encoding, sums) + get_channel(GREEN2, encoding, sums)) / vcos_max(count * 2, 1);
	b_ave = get_channel(BLUE, encoding, sums) / vcos_max(count, 1);

	/* FIXME: Do a moving average filter for smoother changes in values */
	dev->wb_gains.r_gain.num = ((g_ave << 8) / (r_ave + 1));
	dev->wb_gains.b_gain.num = ((g_ave << 8) / (b_ave + 1)); // +1 to protect against div by 0*/

	if (dev->isp_ip)
		mmal_port_parameter_set(dev->isp_ip, &dev->wb_gains.hdr);
}

static void *awb_thread_task(void *arg)
{
	RASPIRAW_CALLBACK_T *dev = (RASPIRAW_CALLBACK_T *)arg;
	MMAL_BUFFER_HEADER_T *buffer;

	while (!dev->awb_thread_quit)
	{
		// Being lazy and using a timed wait instead of setting up a
		// mechanism for skipping this when destroying the thread
		buffer = mmal_queue_timedwait(dev->awb_queue, 1000);
		if (!buffer)
			continue;
		if (!mmal_queue_length(dev->awb_queue))
		{
			/* If more buffers in the queue, loop so we're working
			 * on the latest one
			 */
			run_awb_calcs(dev, buffer);
		}

		mmal_buffer_header_release(buffer);
		buffers_to_rawcam(dev);
	}

	return NULL;
}

static void *processing_thread_task(void *arg)
{
	RASPIRAW_CALLBACK_T *dev = (RASPIRAW_CALLBACK_T *)arg;
	MMAL_BUFFER_HEADER_T *buffer;

	while (!dev->processing_thread_quit)
	{
		// Being lazy and using a timed wait instead of setting up a
		// mechanism for skipping this when destroying the thread
		buffer = mmal_queue_timedwait(dev->processing_queue, 1000);
		if (!buffer)
			continue;
		if (!mmal_queue_length(dev->processing_queue))
		{
			/* If more buffers in the queue, loop so we're working
			 * on the latest one
			 */
			// DO SOME FORM OF PROCESSING ON THE RAW BAYER DATA HERE
			// buffer->user_data points to the data, with
			// buffer->length being the length of the data.
		}

		mmal_buffer_header_release(buffer);
		buffers_to_rawcam(dev);
	}

	return NULL;
}

static void isp_ip_cb(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
	RASPIRAW_CALLBACK_T *dev = (RASPIRAW_CALLBACK_T *)port->userdata;

	mmal_buffer_header_release(buffer);
	buffers_to_rawcam(dev);
}

static void *processing_yuv_thread_task(void *arg)
{
	RASPIRAW_ISP_CALLBACK_T *yuv_cb = (RASPIRAW_ISP_CALLBACK_T *)arg;
	MMAL_BUFFER_HEADER_T *buffer;

	while (!yuv_cb->processing_yuv_thread_quit)
	{
		// Being lazy and using a timed wait instead of setting up a
		// mechanism for skipping this when destroying the thread
		buffer = mmal_queue_timedwait(yuv_cb->processing_yuv_queue, 1000);
		if (!buffer)
			continue;
		if (!mmal_queue_length(yuv_cb->processing_yuv_queue))
		{
			/* If more buffers in the queue, loop so we're working
			 * on the latest one
			 */
			// DO SOME FORM OF PROCESSING ON THE YUV DATA HERE
			// buffer->user_data points to the data, with
			// buffer->length being the length of the data.
		}

		mmal_buffer_header_release(buffer);
		buffers_to_isp_op(yuv_cb);
	}

	return NULL;
}

static void vr_ip_cb(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
	RASPIRAW_ISP_CALLBACK_T *dev = (RASPIRAW_ISP_CALLBACK_T *)port->userdata;

	mmal_buffer_header_release(buffer);
	buffers_to_isp_op(dev);
}

int main(int argc, char **argv)
{
	RASPIRAW_PARAMS_T cfg = { 0 };
	RASPIRAW_CALLBACK_T dev = { .cfg = &cfg, .rawcam_pool = NULL, .rawcam_output = NULL };
	RASPIRAW_ISP_CALLBACK_T yuv_cb = {
		.cfg = &cfg,
	};
	uint32_t encoding;
	const struct sensor_def *sensor;
	struct mode_def *sensor_mode = NULL;
	VCOS_THREAD_T awb_thread;
	VCOS_THREAD_T processing_thread;
	VCOS_THREAD_T processing_yuv_thread;
	char i2c_device_name[I2C_DEVICE_NAME_LEN];
	int i2c_fd;

	// Initialise any non-zero config values.
	cfg.exposure = -1;
	cfg.gain = -1;
	cfg.timeout = 5000;
	cfg.saverate = 20;
	cfg.bit_depth = -1;
	cfg.camera_num = -1;
	cfg.exposure_us = -1;
	cfg.i2c_bus = -1;
	cfg.crop.hinc = -1;
	cfg.crop.vinc = -1;
	cfg.fps = -1;
	cfg.crop.width = -1;
	cfg.crop.height = -1;
	cfg.crop.left = -1;
	cfg.crop.top = -1;
	cfg.opacity = 255;
	cfg.fullscreen = 1;

	bcm_host_init();
	vcos_log_register("RaspiRaw", VCOS_LOG_CATEGORY);

	if (argc == 1)
	{
		fprintf(stdout, "\n%s Camera App %s\n\n", basename(argv[0]), VERSION_STRING);

		raspicli_display_help(cmdline_commands, cmdline_commands_size);
		exit(-1);
	}

	// Parse the command line and put options in to our status structure
	if (parse_cmdline(argc, argv, &cfg))
	{
		exit(-1);
	}

	if (cfg.i2c_bus == -1)
	{
		snprintf(i2c_device_name, sizeof(i2c_device_name), "/dev/i2c-%d", DEFAULT_I2C_DEVICE);
		i2c_fd = open(i2c_device_name, O_RDWR);
		if (!i2c_fd)
		{
			snprintf(i2c_device_name, sizeof(i2c_device_name), "/dev/i2c-%d", ALT_I2C_DEVICE);
			i2c_fd = open(i2c_device_name, O_RDWR);
		}
	}
	else
	{
		snprintf(i2c_device_name, sizeof(i2c_device_name), "/dev/i2c-%d", cfg.i2c_bus);
		i2c_fd = open(i2c_device_name, O_RDWR);
	}

	if (!i2c_fd)
	{
		printf("Failed to open I2C device %s\n", i2c_device_name);
		return -1;
	}

	printf("Using I2C device %s\n", i2c_device_name);

	sensor = probe_sensor(i2c_fd);
	if (!sensor)
	{
		vcos_log_error("No sensor found. Aborting");
		return -1;
	}

	if (cfg.mode >= 0 && cfg.mode < sensor->num_modes)
	{
		sensor_mode = &sensor->modes[cfg.mode];
	}

	if (!sensor_mode)
	{
		vcos_log_error("Invalid mode %d - aborting", cfg.mode);
		return -2;
	}

	if (cfg.regs)
	{
		int r, b;
		char *p, *q;

		p = strtok(cfg.regs, ";");
		while (p)
		{
			vcos_assert(strlen(p) > 6);
			vcos_assert(p[4] == ',');
			vcos_assert(strlen(p) % 2);
			p[4] = '\0';
			q = p + 5;
			sscanf(p, "%4x", &r);
			while (*q)
			{
				vcos_assert(isxdigit(q[0]));
				vcos_assert(isxdigit(q[1]));

				sscanf(q, "%2x", &b);
				vcos_log_error("%04x: %02x", r, b);

				modReg(sensor_mode, r, 0, 7, b, EQUAL);

				++r;
				q += 2;
			}
			p = strtok(NULL, ";");
		}
	}

	if (cfg.crop.hinc >= 0 || cfg.crop.vinc >= 0 || cfg.crop.width > 0 || cfg.crop.width > 0 || cfg.crop.top >= 0 ||
	    cfg.crop.left >= 0)
	{
		if (sensor->set_crop)
		{
			if (sensor->set_crop(sensor, sensor_mode, &cfg.crop))
			{
				vcos_log_error("Failed setting manual crops. Aborting");
				return -1;
			}
		}
		else
		{
			vcos_log_error("This sensor does not currently support "
				       "manual cropping settings. Aborting");
			return -1;
		}
	}

	if (cfg.fps > 0)
	{
		int n = 1000000000 / (sensor_mode->line_time_ns * cfg.fps);
		modReg(sensor_mode, sensor->vts_reg + 0, 0, 7, n >> 8, EQUAL);
		modReg(sensor_mode, sensor->vts_reg + 1, 0, 7, n & 0xFF, EQUAL);
	}

	if (cfg.bit_depth == -1)
	{
		cfg.bit_depth = sensor_mode->native_bit_depth;
	}

	if (cfg.write_headerg && (cfg.bit_depth != sensor_mode->native_bit_depth))
	{
		// needs change after fix for
		// https://github.com/6by9/raspiraw/issues/2
		vcos_log_error("--headerG supported for native bit depth only");
		exit(-1);
	}

	if (cfg.exposure_us != -1)
	{
		cfg.exposure = ((int64_t)cfg.exposure_us * 1000) / sensor_mode->line_time_ns;
		vcos_log_error("Setting exposure to %d from time %dus", cfg.exposure, cfg.exposure_us);
	}

	update_regs(sensor, sensor_mode, cfg.hflip, cfg.vflip, cfg.exposure, cfg.gain);
	if (sensor_mode->encoding == 0)
		encoding = order_and_bit_depth_to_encoding(sensor_mode->order, cfg.bit_depth);
	else
		encoding = sensor_mode->encoding;
	if (!encoding)
	{
		vcos_log_error("Failed to map bitdepth %d and order %d into encoding\n", cfg.bit_depth,
			       sensor_mode->order);
		return -3;
	}
	vcos_log_error("Encoding %08X", encoding);

	if (cfg.awb)
	{
		VCOS_STATUS_T vcos_status;
		printf("Setup awb thread\n");
		vcos_status = vcos_thread_create(&awb_thread, "awb-thread", NULL, awb_thread_task, &dev);
		if (vcos_status != VCOS_SUCCESS)
		{
			printf("Failed to create awb thread\n");
			return -4;
		}
		dev.awb_queue = mmal_queue_create();
		if (!dev.awb_queue)
		{
			printf("Failed to create awb queue\n");
			return -4;
		}
	}
	else
		printf("No AWB\n");

	if (cfg.processing)
	{
		VCOS_STATUS_T vcos_status;
		printf("Setup processing thread\n");
		vcos_status =
		    vcos_thread_create(&processing_thread, "processing-thread", NULL, processing_thread_task, &dev);
		if (vcos_status != VCOS_SUCCESS)
		{
			printf("Failed to create processing thread\n");
			return -4;
		}
		dev.processing_queue = mmal_queue_create();
		if (!dev.processing_queue)
		{
			printf("Failed to create processing queue\n");
			return -4;
		}
	}

	MMAL_COMPONENT_T *rawcam = NULL, *isp = NULL, *render = NULL;
	MMAL_STATUS_T status;
	MMAL_PORT_T *output = NULL;
	MMAL_POOL_T *pool = NULL, *yuv_pool = NULL;
	MMAL_PARAMETER_CAMERA_RX_CONFIG_T rx_cfg;
	MMAL_PARAMETER_CAMERA_RX_TIMING_T rx_timing;
	unsigned int i;

	bcm_host_init();
	vcos_log_register("RaspiRaw", VCOS_LOG_CATEGORY);

	status = mmal_component_create("vc.ril.rawcam", &rawcam);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to create rawcam");
		return -1;
	}
	status = mmal_port_parameter_set_boolean(rawcam->output[0], MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to set zero copy");
		goto component_disable;
	}

	status = mmal_component_create("vc.ril.isp", &isp);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to create isp");
		goto component_destroy;
	}
	status = mmal_port_parameter_set_boolean(isp->input[0], MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to set zero copy");
		goto component_disable;
	}
	status = mmal_port_parameter_set_boolean(isp->output[0], MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to set zero copy");
		goto component_disable;
	}

	status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_RENDERER, &render);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to create render");
		goto component_destroy;
	}
	status = mmal_port_parameter_set_boolean(render->input[0], MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to set zero copy");
		goto component_disable;
	}

	output = rawcam->output[0];

	rx_cfg.hdr.id = MMAL_PARAMETER_CAMERA_RX_CONFIG;
	rx_cfg.hdr.size = sizeof(rx_cfg);
	status = mmal_port_parameter_get(output, &rx_cfg.hdr);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to get cfg");
		goto component_destroy;
	}
	if (sensor_mode->encoding || cfg.bit_depth == sensor_mode->native_bit_depth)
	{
		rx_cfg.unpack = MMAL_CAMERA_RX_CONFIG_UNPACK_NONE;
		rx_cfg.pack = MMAL_CAMERA_RX_CONFIG_PACK_NONE;
	}
	else
	{
		switch (sensor_mode->native_bit_depth)
		{
		case 8:
			rx_cfg.unpack = MMAL_CAMERA_RX_CONFIG_UNPACK_8;
			break;
		case 10:
			rx_cfg.unpack = MMAL_CAMERA_RX_CONFIG_UNPACK_10;
			break;
		case 12:
			rx_cfg.unpack = MMAL_CAMERA_RX_CONFIG_UNPACK_12;
			break;
		case 14:
			rx_cfg.unpack = MMAL_CAMERA_RX_CONFIG_UNPACK_16;
			break;
		case 16:
			rx_cfg.unpack = MMAL_CAMERA_RX_CONFIG_UNPACK_16;
			break;
		default:
			vcos_log_error("Unknown native bit depth %d", sensor_mode->native_bit_depth);
			rx_cfg.unpack = MMAL_CAMERA_RX_CONFIG_UNPACK_NONE;
			break;
		}
		switch (cfg.bit_depth)
		{
		case 8:
			rx_cfg.pack = MMAL_CAMERA_RX_CONFIG_PACK_8;
			break;
		case 10:
			rx_cfg.pack = MMAL_CAMERA_RX_CONFIG_PACK_RAW10;
			break;
		case 12:
			rx_cfg.pack = MMAL_CAMERA_RX_CONFIG_PACK_RAW12;
			break;
		case 14:
			rx_cfg.pack = MMAL_CAMERA_RX_CONFIG_PACK_14;
			break;
		case 16:
			rx_cfg.pack = MMAL_CAMERA_RX_CONFIG_PACK_16;
			break;
		default:
			vcos_log_error("Unknown output bit depth %d", cfg.bit_depth);
			rx_cfg.pack = MMAL_CAMERA_RX_CONFIG_PACK_NONE;
			break;
		}
	}
	vcos_log_error("Set pack to %d, unpack to %d", rx_cfg.unpack, rx_cfg.pack);
	if (sensor_mode->data_lanes)
		rx_cfg.data_lanes = sensor_mode->data_lanes;
	if (sensor_mode->image_id)
		rx_cfg.image_id = sensor_mode->image_id;
	status = mmal_port_parameter_set(output, &rx_cfg.hdr);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to set cfg");
		goto component_destroy;
	}

	rx_timing.hdr.id = MMAL_PARAMETER_CAMERA_RX_TIMING;
	rx_timing.hdr.size = sizeof(rx_timing);
	status = mmal_port_parameter_get(output, &rx_timing.hdr);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to get timing");
		goto component_destroy;
	}
	if (sensor_mode->timing[0])
		rx_timing.timing1 = sensor_mode->timing[0];
	if (sensor_mode->timing[1])
		rx_timing.timing2 = sensor_mode->timing[1];
	if (sensor_mode->timing[2])
		rx_timing.timing3 = sensor_mode->timing[2];
	if (sensor_mode->timing[3])
		rx_timing.timing4 = sensor_mode->timing[3];
	if (sensor_mode->timing[4])
		rx_timing.timing5 = sensor_mode->timing[4];
	if (sensor_mode->term[0])
		rx_timing.term1 = sensor_mode->term[0];
	if (sensor_mode->term[1])
		rx_timing.term2 = sensor_mode->term[1];
	vcos_log_error("Timing %u/%u, %u/%u/%u, %u/%u", rx_timing.timing1, rx_timing.timing2, rx_timing.timing3,
		       rx_timing.timing4, rx_timing.timing5, rx_timing.term1, rx_timing.term2);
	status = mmal_port_parameter_set(output, &rx_timing.hdr);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to set timing");
		goto component_destroy;
	}

	if (cfg.camera_num != -1)
	{
		vcos_log_error("Set camera_num to %d", cfg.camera_num);
		status = mmal_port_parameter_set_int32(output, MMAL_PARAMETER_CAMERA_NUM, cfg.camera_num);
		if (status != MMAL_SUCCESS)
		{
			vcos_log_error("Failed to set camera_num");
			goto component_destroy;
		}
	}

	status = mmal_component_enable(rawcam);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to enable rawcam");
		goto component_destroy;
	}
	status = mmal_component_enable(isp);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to enable isp");
		goto component_destroy;
	}
	status = mmal_component_enable(render);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to enable render");
		goto component_destroy;
	}

	output->format->es->video.crop.width = sensor_mode->width;
	output->format->es->video.crop.height = sensor_mode->height;
	output->format->es->video.width = VCOS_ALIGN_UP(sensor_mode->width, 16);
	output->format->es->video.height = VCOS_ALIGN_UP(sensor_mode->height, 16);
	output->format->encoding = encoding;

	status = mmal_port_format_commit(output);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed port_format_commit");
		goto component_disable;
	}

	output->buffer_size = output->buffer_size_recommended;
	output->buffer_num = output->buffer_num_recommended;

	if (cfg.capture)
	{
		if (cfg.write_header || cfg.write_header0)
		{
			brcm_header = (struct brcm_raw_header *)malloc(BRCM_RAW_HEADER_LENGTH);
			if (brcm_header)
			{
				memset(brcm_header, 0, BRCM_RAW_HEADER_LENGTH);
				brcm_header->id = BRCM_ID_SIG;
				brcm_header->version = HEADER_VERSION;
				brcm_header->mode.width = sensor_mode->width;
				brcm_header->mode.height = sensor_mode->height;
				// FIXME: Ought to check that the sensor is
				// producing Bayer rather than just assuming.
				brcm_header->mode.format = VC_IMAGE_BAYER;
				switch (sensor_mode->order)
				{
				case BAYER_ORDER_BGGR:
					brcm_header->mode.bayer_order = VC_IMAGE_BAYER_BGGR;
					break;
				case BAYER_ORDER_GBRG:
					brcm_header->mode.bayer_order = VC_IMAGE_BAYER_GBRG;
					break;
				case BAYER_ORDER_GRBG:
					brcm_header->mode.bayer_order = VC_IMAGE_BAYER_GRBG;
					break;
				case BAYER_ORDER_RGGB:
					brcm_header->mode.bayer_order = VC_IMAGE_BAYER_RGGB;
					break;
				}
				switch (cfg.bit_depth)
				{
				case 8:
					brcm_header->mode.bayer_format = VC_IMAGE_BAYER_RAW8;
					break;
				case 10:
					brcm_header->mode.bayer_format = VC_IMAGE_BAYER_RAW10;
					break;
				case 12:
					brcm_header->mode.bayer_format = VC_IMAGE_BAYER_RAW12;
					break;
				case 14:
					brcm_header->mode.bayer_format = VC_IMAGE_BAYER_RAW14;
					break;
				case 16:
					brcm_header->mode.bayer_format = VC_IMAGE_BAYER_RAW16;
					break;
				}
				if (cfg.write_header0)
				{
					// Save bcrm_header into one file only
					FILE *file;
					file = fopen(cfg.write_header0, "wb");
					if (file)
					{
						fwrite(brcm_header, BRCM_RAW_HEADER_LENGTH, 1, file);
						fclose(file);
					}
				}
			}
		}
		else if (cfg.write_headerg)
		{
			// Save pgm_header into one file only
			FILE *file;
			file = fopen(cfg.write_headerg, "wb");
			if (file)
			{
				fprintf(file, "P5\n%d %d\n255\n", sensor_mode->width, sensor_mode->height);
				fclose(file);
			}
		}
	}

	MMAL_PORT_T *port;
	port = isp->input[0];
	status = mmal_format_full_copy(port->format, output->format);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to copy port format");
		goto pool_destroy;
	}
	status = mmal_port_format_commit(port);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to commit port format on isp input");
		goto pool_destroy;
	}

	port->buffer_num = output->buffer_num;

	if (port->buffer_size != output->buffer_size)
	{
		vcos_log_error("rawcam output and isp input are different "
			       "sizes - this could end badly");
		vcos_log_error("rawcam %u, isp %u", port->buffer_size, output->buffer_size);
	}

	port = isp->output[0];
	port->format->es->video.crop.width = sensor_mode->width;
	port->format->es->video.crop.height = sensor_mode->height;
	while (port->format->es->video.crop.width > 1920)
	{
		// Display can only go up to a certain resolution before
		// underflowing
		port->format->es->video.crop.width /= 2;
		port->format->es->video.crop.height /= 2;
	}
	port->format->es->video.width = VCOS_ALIGN_UP(port->format->es->video.crop.width, 32);
	port->format->es->video.height = VCOS_ALIGN_UP(port->format->es->video.crop.height, 16);
	port->format->encoding = MMAL_ENCODING_I420;
	port->buffer_num = 6; // Go for 6 output buffers to give some slack
	status = mmal_port_format_commit(port);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to commit port format on isp output");
		goto pool_destroy;
	}

	if (sensor_mode->black_level)
	{
		status =
		    mmal_port_parameter_set_uint32(isp->input[0], MMAL_PARAMETER_BLACK_LEVEL, sensor_mode->black_level);
		if (status != MMAL_SUCCESS)
		{
			vcos_log_error("Failed to set black level - try "
				       "updating firmware");
		}
	}

	if (cfg.awb_gains_r && cfg.awb_gains_b)
	{
		MMAL_PARAMETER_AWB_GAINS_T param = { { MMAL_PARAMETER_CUSTOM_AWB_GAINS, sizeof(param) },
						     { 0, 0 },
						     { 0, 0 } };

		param.r_gain.num = (unsigned int)(cfg.awb_gains_r * 65536);
		param.b_gain.num = (unsigned int)(cfg.awb_gains_b * 65536);
		param.r_gain.den = param.b_gain.den = 65536;
		status = mmal_port_parameter_set(isp->input[0], &param.hdr);
		if (status != MMAL_SUCCESS)
		{
			vcos_log_error("Failed to set white balance");
		}
	}

	/* Set up the rawcam output callback */

	// Create buffer headers for rawcam to isp/awb/raw processing
	// Rawcam output.
	// Need to manually create the pool so that we have control over mem
	// handles, not letting MMAL core handle the magic.
	vcos_log_error("Create pool of %d buffers of size %d", output->buffer_num, output->buffer_size);
	pool = mmal_port_pool_create(output, output->buffer_num, 0);
	if (!pool)
	{
		vcos_log_error("Failed to create pool");
		goto component_disable;
	}
	for (i = 0; i < output->buffer_num; i++)
	{
		MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(pool->queue);
		if (!buffer)
			vcos_log_error("Where did my buffer go?");
		else
		{
			unsigned int vcsm_handle =
			    vcsm_malloc_cache(output->buffer_size, VCSM_CACHE_TYPE_HOST, "mmal_vc_port buffer");
			unsigned int vc_handle = vcsm_vc_hdl_from_hdl(vcsm_handle);
			uint8_t *mem = (uint8_t *)vcsm_lock(vcsm_handle);
			if (!mem || !vc_handle)
			{
				LOG_ERROR("could not allocate %i bytes of "
					  "shared memory (handle %x)",
					  (int)output->buffer_size, vcsm_handle);
				if (mem)
					vcsm_unlock_hdl(vcsm_handle);
				if (vcsm_handle)
					vcsm_free(vcsm_handle);
			}
			else
			{
				buffer->data = (void *)vc_handle;
				buffer->alloc_size = output->buffer_size;
				buffer->user_data = mem;
			}
		}
		mmal_buffer_header_release(buffer);
	}
	dev.rawcam_output = rawcam->output[0];
	dev.rawcam_pool = pool;

	// ISP input
	vcos_log_error("Create pool of %d buffers of size %d", isp->input[0]->buffer_num, isp->input[0]->buffer_size);
	dev.isp_ip_pool = mmal_port_pool_create(isp->input[0], isp->input[0]->buffer_num, 0);
	if (!dev.isp_ip_pool)
	{
		vcos_log_error("Failed to create isp_ip_pool");
		goto component_disable;
	}
	dev.isp_ip = isp->input[0];

	isp->input[0]->userdata = (struct MMAL_PORT_USERDATA_T *)&dev;
	status = mmal_port_enable(isp->input[0], isp_ip_cb);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to enable isp ip port");
		goto pool_destroy;
	}

	// Set up YUV/RGB processing outputs
	yuv_pool = mmal_port_pool_create(isp->output[0], isp->output[0]->buffer_num, 0);
	if (!yuv_pool)
	{
		vcos_log_error("Failed to create yuv pool");
		goto component_disable;
	}
	for (i = 0; i < isp->output[0]->buffer_num; i++)
	{
		MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(yuv_pool->queue);
		if (!buffer)
			vcos_log_error("Where did my buffer go?");
		else
		{
			unsigned int vcsm_handle =
			    vcsm_malloc_cache(isp->output[0]->buffer_size, VCSM_CACHE_TYPE_HOST, "mmal_vc_port buffer");
			unsigned int vc_handle = vcsm_vc_hdl_from_hdl(vcsm_handle);
			uint8_t *mem = (uint8_t *)vcsm_lock(vcsm_handle);
			if (!mem || !vc_handle)
			{
				LOG_ERROR("could not allocate %i bytes of "
					  "shared memory (handle %x)",
					  (int)isp->output[0]->buffer_size, vcsm_handle);
				if (mem)
					vcsm_unlock_hdl(vcsm_handle);
				if (vcsm_handle)
					vcsm_free(vcsm_handle);
			}
			else
			{
				buffer->data = (void *)vc_handle;
				buffer->alloc_size = isp->output[0]->buffer_size;
				buffer->user_data = mem;
			}
		}
		mmal_buffer_header_release(buffer);
	}
	isp->output[0]->userdata = (struct MMAL_PORT_USERDATA_T *)&yuv_cb;
	status = mmal_port_enable(isp->output[0], yuv_callback);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to enable isp op port");
		goto pool_destroy;
	}
	yuv_cb.isp_output = isp->output[0];
	yuv_cb.isp_op_pool = yuv_pool;

	if (!cfg.no_preview)
	{
		MMAL_DISPLAYREGION_T param;
		param.hdr.id = MMAL_PARAMETER_DISPLAYREGION;
		param.hdr.size = sizeof(MMAL_DISPLAYREGION_T);

		param.set = MMAL_DISPLAY_SET_LAYER;
		param.layer = DEFAULT_PREVIEW_LAYER;

		param.set |= MMAL_DISPLAY_SET_ALPHA;
		param.alpha = cfg.opacity;

		if (cfg.fullscreen)
		{
			param.set |= MMAL_DISPLAY_SET_FULLSCREEN;
			param.fullscreen = 1;
		}
		else
		{
			param.set |= (MMAL_DISPLAY_SET_DEST_RECT | MMAL_DISPLAY_SET_FULLSCREEN);
			param.fullscreen = 0;
			param.dest_rect = cfg.preview_window;
		}

		status = mmal_port_parameter_set(render->input[0], &param.hdr);

		if (status != MMAL_SUCCESS && status != MMAL_ENOSYS)
		{
			vcos_log_error("unable to set preview port parameters (%u)", status);
		}

		status = mmal_format_full_copy(render->input[0]->format, isp->output[0]->format);
		if (status != MMAL_SUCCESS)
		{
			vcos_log_error("Failed to copy port format - isp to render");
			goto pool_destroy;
		}
		status = mmal_port_format_commit(render->input[0]);
		if (status != MMAL_SUCCESS)
		{
			vcos_log_error("Failed to commit port format on render input");
			goto pool_destroy;
		}

		render->input[0]->buffer_num = isp->output[0]->buffer_num;

		yuv_cb.vr_ip_pool = mmal_port_pool_create(render->input[0], render->input[0]->buffer_num, 0);
		if (!yuv_cb.vr_ip_pool)
		{
			vcos_log_error("Failed to create vr_ip_pool");
			goto component_disable;
		}
		yuv_cb.vr_ip = render->input[0];

		render->input[0]->userdata = (struct MMAL_PORT_USERDATA_T *)&yuv_cb;

		status = mmal_port_enable(render->input[0], vr_ip_cb);
		if (status != MMAL_SUCCESS)
		{
			vcos_log_error("Failed to enable vr ip port");
			goto pool_destroy;
		}
	}
	if (cfg.processing_yuv)
	{
		VCOS_STATUS_T vcos_status;
		printf("Setup processing thread\n");
		vcos_status = vcos_thread_create(&processing_yuv_thread, "processing-yuv-thread", NULL,
						 processing_yuv_thread_task, &yuv_cb);
		if (vcos_status != VCOS_SUCCESS)
		{
			printf("Failed to create processing yuv thread\n");
			return -4;
		}
		yuv_cb.processing_yuv_queue = mmal_queue_create();
		if (!yuv_cb.processing_yuv_queue)
		{
			printf("Failed to create processing yuv queue\n");
			return -4;
		}
	}

	output->userdata = (struct MMAL_PORT_USERDATA_T *)&dev;
	status = mmal_port_enable(output, callback);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to enable port");
		goto pool_destroy;
	}

	buffers_to_rawcam(&dev);
	buffers_to_isp_op(&yuv_cb);

	start_camera_streaming(sensor, sensor_mode, i2c_fd);

	vcos_sleep(cfg.timeout);

	stop_camera_streaming(sensor, i2c_fd);

port_disable:
	if (cfg.capture)
	{
		status = mmal_port_disable(output);
		if (status != MMAL_SUCCESS)
		{
			vcos_log_error("Failed to disable port");
			return -1;
		}
	}
pool_destroy:
	if (pool)
		mmal_port_pool_destroy(output, pool);
component_disable:
	if (brcm_header)
		free(brcm_header);
	status = mmal_component_disable(render);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to disable render");
	}
	status = mmal_component_disable(isp);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to disable isp");
	}
	status = mmal_component_disable(rawcam);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to disable rawcam");
	}
component_destroy:
	if (rawcam)
		mmal_component_destroy(rawcam);
	if (isp)
		mmal_component_destroy(isp);
	if (render)
		mmal_component_destroy(render);

	if (cfg.awb)
	{
		dev.awb_thread_quit = 1;
		vcos_thread_join(&awb_thread, NULL);
	}

	if (cfg.processing)
	{
		dev.processing_thread_quit = 1;
		vcos_thread_join(&processing_thread, NULL);
	}

	if (cfg.write_timestamps)
	{
		// Save timestamps
		FILE *file;
		file = fopen(cfg.write_timestamps, "wb");
		if (file)
		{
			int64_t old = 0;
			PTS_NODE_T aux;
			for (aux = cfg.ptsa; aux != cfg.ptso; aux = aux->nxt)
			{
				if (aux == cfg.ptsa)
				{
					fprintf(file, ",%d,%lld\n", aux->idx, aux->pts);
				}
				else
				{
					fprintf(file, "%lld,%d,%lld\n", aux->pts - old, aux->idx, aux->pts);
				}
				old = aux->pts;
			}
			fclose(file);
		}

		while (cfg.ptsa != cfg.ptso)
		{
			PTS_NODE_T aux = cfg.ptsa->nxt;
			free(cfg.ptsa);
			cfg.ptsa = aux;
		}
		free(cfg.ptso);
	}
	close(i2c_fd);

	return 0;
}

void modRegBit(struct mode_def *mode, uint16_t reg, int bit, int value, enum operation op)
{
	int i = 0;
	uint16_t val;
	while (i < mode->num_regs && mode->regs[i].reg != reg)
		i++;
	if (i == mode->num_regs)
	{
		vcos_log_error("Reg: %04X not found!\n", reg);
		return;
	}
	val = mode->regs[i].data;

	switch (op)
	{
	case EQUAL:
		val = val & ~(1 << bit);
		val = val | (value << bit);
		break;
	case SET:
		val = val | (1 << bit);
		break;
	case CLEAR:
		val = val & ~(1 << bit);
		break;
	case XOR:
		val = val ^ (value << bit);
		break;
	}
	mode->regs[i].data = val;
}

void modReg(struct mode_def *mode, uint16_t reg, int startBit, int endBit, int value, enum operation op)
{
	int i;
	for (i = startBit; i <= endBit; i++)
	{
		modRegBit(mode, reg, i, value >> i & 1, op);
	}
}

void update_regs(const struct sensor_def *sensor, struct mode_def *mode, int hflip, int vflip, int exposure, int gain)
{
	if (sensor->vflip_reg)
	{
		modRegBit(mode, sensor->vflip_reg, sensor->vflip_reg_bit, vflip, XOR);
		if (vflip && !sensor->flips_dont_change_bayer_order)
			mode->order ^= 2;
	}

	if (sensor->hflip_reg)
	{
		modRegBit(mode, sensor->hflip_reg, sensor->hflip_reg_bit, hflip, XOR);
		if (hflip && !sensor->flips_dont_change_bayer_order)
			mode->order ^= 1;
	}

	if (sensor->exposure_reg && exposure != -1)
	{
		if (exposure < 0 || exposure >= (1 << sensor->exposure_reg_num_bits))
		{
			vcos_log_error("Invalid exposure:%d, exposure range is 0 to %u!\n", exposure,
				       (1 << sensor->exposure_reg_num_bits) - 1);
		}
		else
		{
			uint8_t val;
			int i, j = VCOS_ALIGN_DOWN(sensor->exposure_reg_num_bits - 1, 8);
			int num_regs = (sensor->exposure_reg_num_bits + 7) >> 3;
			for (i = 0; i < num_regs; i++, j -= 8)
			{
				val = (exposure >> j) & 0xFF;
				modReg(mode, sensor->exposure_reg + i, 0, 7, val, EQUAL);
				vcos_log_error("Set exposure %04X to %02X", sensor->exposure_reg + i, val);
			}
		}
	}
	if (sensor->vts_reg && exposure != -1 && exposure >= mode->min_vts)
	{
		if (exposure < 0 || exposure >= (1 << sensor->vts_reg_num_bits))
		{
			vcos_log_error("Invalid exposure:%d, vts range is 0 to %u!\n", exposure,
				       (1 << sensor->vts_reg_num_bits) - 1);
		}
		else
		{
			uint8_t val;
			int i, j = VCOS_ALIGN_DOWN(sensor->vts_reg_num_bits - 1, 8);
			int num_regs = (sensor->vts_reg_num_bits + 7) >> 3;

			for (i = 0; i < num_regs; i++, j -= 8)
			{
				val = (exposure >> j) & 0xFF;
				modReg(mode, sensor->vts_reg + i, 0, 7, val, EQUAL);
				vcos_log_error("Set vts %04X to %02X", sensor->vts_reg + i, val);
			}
		}
	}
	if (sensor->gain_reg && gain != -1)
	{
		if (gain < 0 || gain >= (1 << sensor->gain_reg_num_bits))
		{
			vcos_log_error("Invalid gain:%d, gain range is 0 to %u\n", gain,
				       (1 << sensor->gain_reg_num_bits) - 1);
		}
		else
		{
			uint8_t val;
			int i, j = VCOS_ALIGN_DOWN(sensor->gain_reg_num_bits - 1, 8);
			int num_regs = (sensor->gain_reg_num_bits + 7) >> 3;

			for (i = 0; i < num_regs; i++, j -= 8)
			{
				val = (gain >> j) & 0xFF;
				modReg(mode, sensor->gain_reg + i, 0, 7, val, EQUAL);
				vcos_log_error("Set gain %04X to %02X", sensor->gain_reg + i, val);
			}
		}
	}
}
