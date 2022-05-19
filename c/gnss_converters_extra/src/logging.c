#include <gnss-converters-extra/logging.h>
#include <stdarg.h>
#include <swiftnav/logging.h>

#define LOG_BUFFER_LENGTH 1024

static gnss_converters_extra_logging_callback_t logger = NULL;

static void log_normal(int level, const char *format, ...) {
  char buffer[LOG_BUFFER_LENGTH];

  if (logger == NULL) {
    return;
  }

  va_list args;
  va_start(args, format);
  int ret = vsnprintf(buffer, LOG_BUFFER_LENGTH, format, args);
  va_end(args);

  if (ret >= 0) {
    logger(level, buffer);
  }
}

static void log_detail(int level,
                       const char *file_path,
                       const int line_number,
                       const char *format,
                       ...) {
  (void)file_path;
  (void)line_number;

  char buffer[LOG_BUFFER_LENGTH];

  if (logger == NULL) {
    return;
  }

  va_list args;
  va_start(args, format);
  int ret = vsnprintf(buffer, LOG_BUFFER_LENGTH, format, args);
  va_end(args);

  if (ret >= 0) {
    logger(level, buffer);
  }
}

void gnss_converters_extra_setup_logging(
    gnss_converters_extra_logging_callback_t callback) {
  logger = callback;
  logging_set_implementation(log_normal, log_detail);
}
