#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "driver/spi_master.h"
#include "drv8305.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define BLDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BLDC_MCPWM_PERIOD              500      // 50us, 20KHz
#define BLDC_SPEED_UPDATE_PERIOD_US    200000   // 200ms

#define PWM_H_A 17
#define PWM_H_B 5
#define PWM_H_C 18

#define MISO_PIN 12
#define MOSI_PIN 13
#define SCLK_PIN 14
#define CS_PIN 15
#define DRV8305_EN 2
#define DRV8305_NFAULT 4

#define BLDC_MCPWM_GEN_INDEX_HIGH 0
#define BLDC_MCPWM_GEN_INDEX_LOW  1

static const char TAG[] = "bldc mcpwm";

void init_gpio() {

  drv8305_t dev = {.DRV_EN_GATE_pin = DRV8305_EN,
                   .DRV_N_FAULT_pin = DRV8305_NFAULT,
                   .DRV_MISO_SDO_pin = MISO_PIN,
                   .DRV_MOSI_SDI_pin = MOSI_PIN,
                   .DRV_SCLK_pin = SCLK_PIN,
                   .DRV_SCS_pin = CS_PIN,
                   .spi_host = HSPI_HOST,
                   .max_spi_clockspeed = 1000000};

  // Initialize DRV8305
  ESP_ERROR_CHECK(drv8305_init(&dev));

  drv8305_control_07_reg_t pwm_mode_config;
  drv8305_read_control_07_register(&dev, &pwm_mode_config);
  ESP_LOGI(TAG, "INITIAL VALUE %x", pwm_mode_config.PWM_MODE1);
  ESP_LOGI(TAG, "INITIAL VALUE %x", pwm_mode_config.PWM_MODE2);
  pwm_mode_config.PWM_MODE1 = 0b0;
  pwm_mode_config.PWM_MODE2 = 0b1;
  ESP_ERROR_CHECK(drv8305_write_control_07_register(&dev, pwm_mode_config));

  drv8305_read_control_07_register(&dev, &pwm_mode_config);
  ESP_LOGI(TAG, "FINAL VALUE %x", pwm_mode_config.PWM_MODE1);
  ESP_LOGI(TAG, "FINAL VALUE %x", pwm_mode_config.PWM_MODE2);
}

void mcpwm_config() {
  
  // gpio_config_t drv_en_config = {
  //       .mode = GPIO_MODE_OUTPUT,
  //       .pin_bit_mask = 1ULL << DRV8305_EN,
  //   };
  // ESP_ERROR_CHECK(gpio_config(&drv_en_config));
  // gpio_set_level(DRV8305_EN, 1);


  // mcpwm_timer_handle_t timer = NULL;
  // mcpwm_timer_config_t timer_config = {
  //       .group_id = 0,
  //       .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
  //       .resolution_hz = BLDC_MCPWM_TIMER_RESOLUTION_HZ,
  //       .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
  //       .period_ticks = BLDC_MCPWM_PERIOD,
  //   };

  // ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

  // mcpwm_oper_handle_t operators[3];
  // mcpwm_operator_config_t operator_config = {
  //       .group_id = 0,
  //   };
  // for (int i = 0; i < 3; i++) {
  //       ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operators[i]));
  // }

  // ESP_LOGI(TAG, "Connect operators to the same timer");
  // for (int i = 0; i < 3; i++) {
  //     ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operators[i], timer));
  // }


  // mcpwm_cmpr_handle_t comparators[3];
  // mcpwm_comparator_config_t compare_config = {
  //      .flags.update_cmp_on_tez = true,
  //   };

  // for (int i = 0; i < 3; i++) {
  //       ESP_ERROR_CHECK(mcpwm_new_comparator(operators[i], &compare_config, &comparators[i]));
  //       // set compare value to 0, we will adjust the speed in a period timer callback
  //       ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators[i], 0));
  //   }

  // mcpwm_gen_handle_t generators[3] = {};
  // mcpwm_generator_config_t gen_config = {};
  // const int gen_gpios[3] = {PWM_H_A, PWM_H_B, PWM_H_C};
  // for (int i = 0; i < 3; i++) {
  //       gen_config.gen_gpio_num = gen_gpios[i];
  //       ESP_ERROR_CHECK(mcpwm_new_generator(operators[i], &gen_config, &generators[i]));
        
  //   }

  // ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
  
}

void app_main() {
  
  //mcpwm_config();]
  
  

  // gpio_config_t drv_en_config = {
  //       .mode = GPIO_MODE_OUTPUT,
  //       // .pull_up_en = GPIO_PULLUP_DISABLE,
  //       // .pull_down_en = GPIO_PULLDOWN_DISABLE,
  //       .pin_bit_mask = 1ULL << DRV8305_EN,
  //   };
  // ESP_ERROR_CHECK(gpio_config(&drv_en_config));
  // gpio_set_level(DRV8305_EN, 1);

  init_gpio();

  mcpwm_timer_handle_t timer;
  mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = BLDC_MCPWM_TIMER_RESOLUTION_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = BLDC_MCPWM_PERIOD,
    };



  ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

  ESP_LOGI(TAG, "Create over current fault detector");
  mcpwm_fault_handle_t over_cur_fault = NULL;
  mcpwm_gpio_fault_config_t gpio_fault_config = {
        .gpio_num = DRV8305_EN,
        .group_id = 0,
        .flags.active_level = 0, // low level means fault, refer to DRV8302 datasheet
        .flags.pull_up = true,   // internally pull up
    };
    ESP_ERROR_CHECK(mcpwm_new_gpio_fault(&gpio_fault_config, &over_cur_fault));

  
  

  mcpwm_oper_handle_t operators[3];
  mcpwm_operator_config_t operator_config = {
        .group_id = 0,
    };
  for (int i = 0; i < 3; i++) {
        ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operators[i]));
  }

  ESP_LOGI(TAG, "Connect operators to the same timer");
  for (int i = 0; i < 3; i++) {
      ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operators[i], timer));
  }


  mcpwm_cmpr_handle_t comparators[3];
  mcpwm_comparator_config_t compare_config = {
       .flags.update_cmp_on_tez = true,
       //.flags.update_cmp_on_sync = true,
    };

  for (int i = 0; i < 3; i++) {
        ESP_ERROR_CHECK(mcpwm_new_comparator(operators[i], &compare_config, &comparators[i]));
        // set compare value to 0, we will adjust the speed in a period timer callback
        //ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators[i], 0));
    }

  mcpwm_gen_handle_t generators[3] = {};
  mcpwm_generator_config_t gen_config = {};
  const int gen_gpios[3] = {PWM_H_A, PWM_H_B, PWM_H_C};
  for (int i = 0; i < 3; i++) {
        gen_config.gen_gpio_num = gen_gpios[i];
        ESP_ERROR_CHECK(mcpwm_new_generator(operators[i], &gen_config, &generators[i]));
        
    }

  // gpio_set_level(DRV8305_EN, 1);


  ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer,MCPWM_TIMER_START_NO_STOP));
  
        
  

  // mcpwm_sync_handle_t gpio_sync_source = NULL;
  // mcpwm_gpio_sync_src_config_t gpio_sync_config = {
  //       .group_id = 0,              // GPIO fault should be in the same group of the above timers
  //       .gpio_num = DRV8305_NFAULT,
  //       .flags.pull_down = true,
  //       .flags.active_neg = false,  // by default, a posedge pulse can trigger a sync event
  //   };
  // ESP_ERROR_CHECK(mcpwm_new_gpio_sync_src(&gpio_sync_config, &gpio_sync_source));

  // mcpwm_timer_sync_phase_config_t sync_phase_config = {
  //       .count_value = 0,                      // sync phase: target count value
  //       .direction = MCPWM_TIMER_DIRECTION_UP, // sync phase: count direction
  //       .sync_src = gpio_sync_source,          // sync source
  //   };
  // for (int i = 0; i < 3; i++) {
  //       ESP_ERROR_CHECK(mcpwm_timer_set_phase_on_sync(timer[i], &sync_phase_config));
  //   }
  
  int pwm[3][3] = {{0, 15, 30}, {30, 0, 15}, {15, 30 , 0}}; 

  // PWM values to make it spin
  for (int i = 0; i < 3; i++) {
        // ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer,MCPWM_TIMER_START_STOP_FULL));
        mcpwm_comparator_set_compare_value(comparators[0], pwm[0][i]);
        mcpwm_comparator_set_compare_value(comparators[1], pwm[1][i]);
        mcpwm_comparator_set_compare_value(comparators[2], pwm[2][i]);
        ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_timer_event(generators[0],
                        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH),
                        MCPWM_GEN_TIMER_EVENT_ACTION_END()));
        ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(generators[0],
                        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparators[0], MCPWM_GEN_ACTION_LOW),
                        MCPWM_GEN_COMPARE_EVENT_ACTION_END()));
        ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_timer_event(generators[1],
                        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH),
                        MCPWM_GEN_TIMER_EVENT_ACTION_END()));
        ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(generators[1],
                        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparators[1], MCPWM_GEN_ACTION_LOW),
                        MCPWM_GEN_COMPARE_EVENT_ACTION_END()));
        ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_timer_event(generators[2],
                        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH),
                        MCPWM_GEN_TIMER_EVENT_ACTION_END()));
        ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(generators[2],
                        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparators[2], MCPWM_GEN_ACTION_LOW),
                        MCPWM_GEN_COMPARE_EVENT_ACTION_END()));

        vTaskDelay(50/ portTICK_PERIOD_MS);

        if (i == 2) {
           i = 0;
        }
  }

        // mcpwm_comparator_set_compare_value(comparators[0], 250);
        // mcpwm_comparator_set_compare_value(comparators[1], 250);
        // mcpwm_comparator_set_compare_value(comparators[2], 250);
        // ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_timer_event(generators[0],
        //                 MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW),
        //                 MCPWM_GEN_TIMER_EVENT_ACTION_END()));
        // ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(generators[0],
        //                 MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparators[0], MCPWM_GEN_ACTION_HIGH),
        //                 MCPWM_GEN_COMPARE_EVENT_ACTION_END()));
        // ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_timer_event(generators[1],
        //                 MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW),
        //                 MCPWM_GEN_TIMER_EVENT_ACTION_END()));
        // ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(generators[1],
        //                 MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparators[1], MCPWM_GEN_ACTION_HIGH),
        //                 MCPWM_GEN_COMPARE_EVENT_ACTION_END()));
        // ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_timer_event(generators[2],
        //                 MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW),
        //                 MCPWM_GEN_TIMER_EVENT_ACTION_END()));
        // ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(generators[2],
        //                 MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparators[2], MCPWM_GEN_ACTION_HIGH),
        //                 MCPWM_GEN_COMPARE_EVENT_ACTION_END()));
        

}