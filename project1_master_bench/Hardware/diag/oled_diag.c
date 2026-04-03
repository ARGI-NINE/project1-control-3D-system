#include "oled_diag.h"

#include <stdint.h>

#include "perf_diag.h"
#include "OLED.h"

static uint32_t clip4(uint32_t value) {
  if (value > 9999u) {
    return 9999u;
  }
  return value;
}

static uint32_t clip3(uint32_t value) {
  if (value > 999u) {
    return 999u;
  }
  return value;
}

static uint32_t clip2(uint32_t value) {
  if (value > 99u) {
    return 99u;
  }
  return value;
}

static uint32_t us_to_ms_rounded(uint32_t us_value) {
  return (us_value + 500u) / 1000u;
}

void oled_diag_init(void) {
  OLED_Init();
  OLED_Clear();
}

void oled_diag_render(void) {
  perf_snapshot_t s = {0};

  perf_diag_get_snapshot(&s);

  OLED_Clear(); /* full-screen redraw */

  OLED_ShowString(1, 1, "4M");
  OLED_ShowNum(1, 3, clip4(s.max_us_400hz), 4);
  OLED_ShowString(1, 8, "2M");
  OLED_ShowNum(1, 10, clip4(s.max_us_200hz), 4);

  OLED_ShowString(2, 1, "1M");
  OLED_ShowNum(2, 3, clip4(us_to_ms_rounded(s.max_us_1hz)), 4);
  OLED_ShowString(2, 7, "ms");
  OLED_ShowString(2, 10, "OB");
  OLED_ShowNum(2, 12, clip4(s.over_budget_400hz), 4);

  OLED_ShowString(3, 1, "O4");
  OLED_ShowNum(3, 3, clip3(s.overrun_400hz), 3);
  OLED_ShowString(3, 7, "O2");
  OLED_ShowNum(3, 9, clip3(s.overrun_can_200hz), 3);
  OLED_ShowString(3, 13, "O1");
  OLED_ShowNum(3, 15, clip2(s.overrun_log_1hz), 2);

  OLED_ShowString(4, 1, "IR");
  OLED_ShowNum(4, 3, clip4(s.imu_repeat_count), 4);
  OLED_ShowString(4, 8, "IV");
  OLED_ShowNum(4, 10, clip4(s.imu_valid_count), 4);
  OLED_ShowString(4, 14, "R");
  OLED_ShowChar(4, 16, s.imu_repeat_last ? 'Y' : 'N');
}
