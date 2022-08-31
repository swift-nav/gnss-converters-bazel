/*
 * Copyright (C) 2022 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

/* This is a stand-alone tool that takes truncated Swift Binary Protocol (SBP)
 * binary on stdin and writes Swift Binary Protocol (SBP) struct on stdout. The
 * truncated binary contains [msg_type|sender_id|payload]. */

#include <getopt.h>
#include <libsbp/edc.h>
#include <libsbp/sbp.h>
#include <sbp2sbp/internal/sbp2sbp_encoder_main.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <swiftnav/common.h>

static sbp_write_fn_t sbp2sbp_enc_writefn;
static sbp_state_t in_state;
static sbp_state_t out_state;
static uint8_t sbp_frame_buf[SBP_MAX_FRAME_LEN] = {0};
static uint16_t sbp_frame_buf_len = 0;
static uint16_t sbp_frame_buf_read = 0;

struct sbp_frame_data {
  sbp_msg_type_t msg_type;
  uint16_t sender_id;
  sbp_msg_t msg;
};

static inline void reset_sbp_frame_buf() {
  memset(sbp_frame_buf, 0, ARRAY_SIZE(sbp_frame_buf));
  sbp_frame_buf_len = 0;
  sbp_frame_buf_read = 0;
}

/// Decode binary data [msg_type|sender_id|payload] into struct
/// sbp_frame_data
static bool sbp_struct_decode(struct sbp_frame_data *sbp_frame,
                              int32_t (*read)(uint8_t *buff_in,
                                              uint32_t buff_in_len,
                                              void *context)) {
  uint8_t buff[4096] = {0};

  // Decode msg type
  int32_t decode_len = read(buff, sizeof(sbp_frame->msg_type), NULL);
  if (decode_len != sizeof(sbp_frame->msg_type)) {
    return false;
  }
  sbp_frame->msg_type = buff[0] | (buff[1] << 8U);

  // Decode sender_id
  decode_len = read(buff, sizeof(sbp_frame->sender_id), NULL);
  if (decode_len != sizeof(sbp_frame->sender_id)) {
    return false;
  }
  sbp_frame->sender_id = buff[0] | (buff[1] << 8U);

  // Decode msg content
  memset(buff, 0, sizeof(sbp_frame->msg_type));
  decode_len = read(buff, sizeof(sbp_msg_t), NULL);
  if (decode_len != sizeof(sbp_msg_t)) {
    return false;
  }
  memcpy(&sbp_frame->msg, buff, sizeof(sbp_msg_t));

  return true;
}

/// Read & write function on temporary buffer "sbp_frame_buf"
static int32_t write_frame(uint8_t *frame, uint32_t frame_len, void *context) {
  (void)context;

  if ((ARRAY_SIZE(sbp_frame_buf) - sbp_frame_buf_len) < frame_len) {
    return 0;
  }
  memcpy(sbp_frame_buf + sbp_frame_buf_len, frame, frame_len);
  sbp_frame_buf_len += frame_len;
  return frame_len;
}

static int32_t read_frame(uint8_t *frame, uint32_t frame_len, void *context) {
  (void)context;
  if ((sbp_frame_buf_len - sbp_frame_buf_read) == 0) {
    return -1;  // Reach end of buffer
  }

  if (sbp_frame_buf_len < (sbp_frame_buf_read + frame_len)) {
    return -1;
  }

  memcpy(frame, sbp_frame_buf + sbp_frame_buf_read, frame_len);
  sbp_frame_buf_read += frame_len;
  return frame_len;
}

/// Write the SBP packet to STDOUT.
static void sbp_write(uint16_t sender_id,
                      sbp_msg_type_t msg_type,
                      const sbp_msg_t *msg,
                      void *context) {
  (void)context;
  sbp_message_send(&out_state, msg_type, sender_id, msg, sbp2sbp_enc_writefn);
  fprintf(stderr, "Finish writing msg type: %d\n", msg_type);
}

static void help(char *arg, const char *additional_opts_help) {
  fprintf(stderr, "Usage: %s [options]%s\n", arg, additional_opts_help);
  fprintf(stderr, "  -h this message\n");
}

int sbp2sbp_encoder_main(int argc,
                         char **argv,
                         const char *additional_opts_help,
                         readfn_ptr readfn,
                         readfn_eof_ptr readfn_eof,
                         writefn_ptr writefn,
                         void *context) {
  sbp2sbp_enc_writefn = writefn;

  // In state
  sbp_state_init(&in_state);
  sbp_state_set_io_context(&in_state, context);

  // Out state
  sbp_state_init(&out_state);
  sbp_state_set_io_context(&out_state, context);

  sbp_msg_callbacks_node_t out_node;
  sbp_all_message_callback_register(&out_state, sbp_write, NULL, &out_node);

  int opt = -1;
  while ((opt = getopt(argc, argv, "h")) != -1) {
    switch (opt) {
      case 'h':
        help(argv[0], additional_opts_help);
        return 0;
      default:
        break;
    }
  }

  struct sbp_frame_data sbp_data;
  do {
    reset_sbp_frame_buf();

    if (!sbp_struct_decode(&sbp_data, readfn)) {
      continue;
    }

    if (SBP_OK != sbp_message_send(&in_state,
                                   sbp_data.msg_type,
                                   sbp_data.sender_id,
                                   &sbp_data.msg,
                                   write_frame)) {
      continue;
    }

    int8_t ret = SBP_OK;
    do {
      ret = sbp_process(&out_state, read_frame);
    } while ((ret >= 0));
  } while (readfn_eof(context) == 0);

  return 0;
}
