#include "can_protocol.h"

static uint16_t read_u16_le(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static int16_t read_i16_le(const uint8_t *p) { return (int16_t)read_u16_le(p); }

static void write_u16_le(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFFu);
  p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static void write_i16_le(uint8_t *p, int16_t v) { write_u16_le(p, (uint16_t)v); }

bool can_pack_cmd(const can_cmd_t *cmd, uint8_t out_data[8]) {
  if (!cmd || !out_data) {
    return false;
  }
  write_u16_le(&out_data[0], cmd->seq);
  write_i16_le(&out_data[2], cmd->yaw_ref_cdeg);
  write_i16_le(&out_data[4], cmd->pitch_ref_cdeg);
  write_i16_le(&out_data[6], cmd->roll_ref_cdeg);
  return true;
}

bool can_unpack_cmd(const uint8_t data[8], uint8_t dlc, can_cmd_t *cmd_out) {
  if (!data || !cmd_out || dlc != 8) {
    return false;
  }
  cmd_out->seq = read_u16_le(&data[0]);
  cmd_out->yaw_ref_cdeg = read_i16_le(&data[2]);
  cmd_out->pitch_ref_cdeg = read_i16_le(&data[4]);
  cmd_out->roll_ref_cdeg = read_i16_le(&data[6]);
  return true;
}

bool can_pack_status(const can_status_t *st, uint8_t out_data[8]) {
  if (!st || !out_data) {
    return false;
  }
  write_u16_le(&out_data[0], st->seq_echo);
  write_u16_le(&out_data[2], st->pulse_us);
  write_i16_le(&out_data[4], st->cur_pos_cdeg);
  out_data[6] = st->fault;
  out_data[7] = st->vbat_01v;
  return true;
}

bool can_unpack_status(const uint8_t data[8], uint8_t dlc, can_status_t *st_out) {
  if (!data || !st_out || dlc != 8) {
    return false;
  }
  st_out->seq_echo = read_u16_le(&data[0]);
  st_out->pulse_us = read_u16_le(&data[2]);
  st_out->cur_pos_cdeg = read_i16_le(&data[4]);
  st_out->fault = data[6];
  st_out->vbat_01v = data[7];
  return true;
}
