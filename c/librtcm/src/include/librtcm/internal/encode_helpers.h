#ifndef LIBRTCM_INTERNAL_ENCODE_HELPERS_H
#define LIBRTCM_INTERNAL_ENCODE_HELPERS_H

#define BITSTREAM_ENCODE_BOOL(bitstream, field, n_bits) \
  do {                                                  \
    BITSTREAM_ENCODE_U32(bitstream, field, n_bits);     \
  } while (false)

#define BITSTREAM_ENCODE_U64(bitstream, field, n_bits)                         \
  do {                                                                         \
    u64 decode_u64_temp = field;                                               \
    char __encode_assert_1[(sizeof(field) <= sizeof(u64)) ? 1 : -1];           \
    char /* NOLINTNEXTLINE(clang-analyzer-core.VLASize) */                     \
        __encode_assert_2[((size_t)(n_bits) <= (sizeof(field) * 8)) ? 1 : -1]; \
    (void)__encode_assert_1;                                                   \
    (void)__encode_assert_2;                                                   \
    if (!swiftnav_out_bitstream_setbitul(                                      \
            bitstream, &decode_u64_temp, 0, n_bits)) {                         \
      return RC_INVALID_MESSAGE;                                               \
    }                                                                          \
    swiftnav_out_bitstream_remove(bitstream, n_bits);                          \
  } while (false)

#define BITSTREAM_ENCODE_S64(bitstream, field, n_bits)                         \
  do {                                                                         \
    s64 decode_s64_temp = field;                                               \
    char __encode_assert_1[(sizeof(field) == sizeof(s64)) ? 1 : -1];           \
    char                                                                       \
        __encode_assert_2[((size_t)(n_bits) <= (sizeof(field) * 8)) ? 1 : -1]; \
    (void)__encode_assert_1;                                                   \
    (void)__encode_assert_2;                                                   \
    if (!swiftnav_out_bitstream_setbitsl(                                      \
            bitstream, &decode_s64_temp, 0, n_bits)) {                         \
      return RC_INVALID_MESSAGE;                                               \
    }                                                                          \
    swiftnav_out_bitstream_remove(bitstream, n_bits);                          \
  } while (false)

#define BITSTREAM_ENCODE_U32(bitstream, field, n_bits)                         \
  do {                                                                         \
    u32 decode_u32_temp = field;                                               \
    char __encode_assert_1[(sizeof(field) == sizeof(u32)) ? 1 : -1];           \
    char /* NOLINTNEXTLINE(clang-analyzer-core.VLASize) */                     \
        __encode_assert_2[((size_t)(n_bits) <= (sizeof(field) * 8)) ? 1 : -1]; \
    (void)__encode_assert_1;                                                   \
    (void)__encode_assert_2;                                                   \
    if (!swiftnav_out_bitstream_setbitu(                                       \
            bitstream, &decode_u32_temp, 0, n_bits)) {                         \
      return RC_INVALID_MESSAGE;                                               \
    }                                                                          \
    swiftnav_out_bitstream_remove(bitstream, n_bits);                          \
  } while (false)

#define BITSTREAM_ENCODE_S32(bitstream, field, n_bits)                         \
  do {                                                                         \
    s32 decode_s32_temp = field;                                               \
    char __encode_assert_1[(sizeof(field) == sizeof(s32)) ? 1 : -1];           \
    char                                                                       \
        __encode_assert_2[((size_t)(n_bits) <= (sizeof(field) * 8)) ? 1 : -1]; \
    (void)__encode_assert_1;                                                   \
    (void)__encode_assert_2;                                                   \
    if (!swiftnav_out_bitstream_setbits(                                       \
            bitstream, &decode_s32_temp, 0, n_bits)) {                         \
      return RC_INVALID_MESSAGE;                                               \
    }                                                                          \
    swiftnav_out_bitstream_remove(bitstream, n_bits);                          \
  } while (false)

#define BITSTREAM_ENCODE_U16(bitstream, field, n_bits)                         \
  do {                                                                         \
    u32 decode_u16_temp = field;                                               \
    char __encode_assert_1[(sizeof(field) == sizeof(u16)) ? 1 : -1];           \
    char                                                                       \
        __encode_assert_2[((size_t)(n_bits) <= (sizeof(field) * 8)) ? 1 : -1]; \
    (void)__encode_assert_1;                                                   \
    (void)__encode_assert_2;                                                   \
    if (!swiftnav_out_bitstream_setbitu(                                       \
            bitstream, &decode_u16_temp, 0, n_bits)) {                         \
      return RC_INVALID_MESSAGE;                                               \
    }                                                                          \
    swiftnav_out_bitstream_remove(bitstream, n_bits);                          \
  } while (false)

#define BITSTREAM_ENCODE_S16(bitstream, field, n_bits)                         \
  do {                                                                         \
    s32 decode_s16_temp = field;                                               \
    char __encode_assert_1[(sizeof(field) == sizeof(s16)) ? 1 : -1];           \
    char                                                                       \
        __encode_assert_2[((size_t)(n_bits) <= (sizeof(field) * 8)) ? 1 : -1]; \
    (void)__encode_assert_1;                                                   \
    (void)__encode_assert_2;                                                   \
    if (!swiftnav_out_bitstream_setbits(                                       \
            bitstream, &decode_s16_temp, 0, n_bits)) {                         \
      return RC_INVALID_MESSAGE;                                               \
    }                                                                          \
    swiftnav_out_bitstream_remove(bitstream, n_bits);                          \
  } while (false)

#define BITSTREAM_ENCODE_U8(bitstream, field, n_bits)                          \
  do {                                                                         \
    u32 decode_u8_temp = field;                                                \
    char __encode_assert_1[(sizeof(field) == sizeof(u8)) ? 1 : -1];            \
    char                                                                       \
        __encode_assert_2[((size_t)(n_bits) <= (sizeof(field) * 8)) ? 1 : -1]; \
    (void)__encode_assert_1;                                                   \
    (void)__encode_assert_2;                                                   \
    if (!swiftnav_out_bitstream_setbitu(                                       \
            bitstream, &decode_u8_temp, 0, n_bits)) {                          \
      return RC_INVALID_MESSAGE;                                               \
    }                                                                          \
    swiftnav_out_bitstream_remove(bitstream, n_bits);                          \
  } while (false)

#define BITSTREAM_ENCODE_S8(bitstream, field, n_bits)                          \
  do {                                                                         \
    s32 decode_s8_temp = field;                                                \
    char __encode_assert_1[(sizeof(field) == sizeof(s8)) ? 1 : -1];            \
    char                                                                       \
        __encode_assert_2[((size_t)(n_bits) <= (sizeof(field) * 8)) ? 1 : -1]; \
    (void)__encode_assert_1;                                                   \
    (void)__encode_assert_2;                                                   \
    if (!swiftnav_out_bitstream_setbits(                                       \
            bitstream, &decode_s8_temp, 0, n_bits)) {                          \
      return RC_INVALID_MESSAGE;                                               \
    }                                                                          \
    swiftnav_out_bitstream_remove(bitstream, n_bits);                          \
  } while (false)

/* macros for reading rcv/ant descriptor strings */
#define SET_STR(TheBuff, TheIdx, TheLen, TheInput)        \
  do {                                                    \
    for (uint8_t i = 0; i < (TheLen); ++i) {              \
      rtcm_setbitu((TheBuff), (TheIdx), 8, TheInput)[i]); \
      (TheIdx) += 8;                                      \
    }                                                     \
  } while (false);

#endif /* LIBRTCM_INTERNAL_ENCODE_HELPERS_H */
