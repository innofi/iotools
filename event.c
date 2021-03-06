// SPDX-License-Identifier: Apache-2.0

#include <esp_event.h>
#include <esp_event_base.h>
#include <freertos/portmacro.h>
#include "event.hpp"

ESP_EVENT_DEFINE_BASE(IOTOOLS_EVENT);

esp_event_loop_handle_t loop_handle;
EventGroupHandle_t event_group;

void init_event() {
  esp_event_loop_args_t loop_args = {
      .queue_size = 16,
      .task_name = "event_task",
      .task_priority = uxTaskPriorityGet(NULL),,
      .task_stack_size = 4096,
      .task_core_id = 1,
  };
  esp_event_loop_create(&loop_args, &loop_handle);

  event_group = xEventGroupCreate();
  xEventGroupClearBits(event_group, 0xffffff);

}