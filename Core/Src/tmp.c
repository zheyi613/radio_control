/* 123 */
#include "nrf_payload.h"
#include "main.h"
#define JOYSTICK_GET_CYCLE      10
#define JOYSTICK_RANGE          2048
#define JOYSTICK_ZERO_RANGE     150

uint8_t tick;
uint16_t adc_val[4];
uint8_t joystick_task_trigger = 0;
uint8_t tx_finish;
struct payload pl;
struct ack_payload ack_pl;

struct {
        uint16_t LX;
        uint16_t LY;
        uint16_t RX;
        uint16_t RY;
} joystick_raw;

struct {
        float LX;
        float LY;
        float RX;
        float RY;
        uint8_t L_button;
        uint8_t R_button;
} joystick_data;

struct {
        float roll;
        float pitch;
        float yaw;
        uint16_t motor[4];
        float height;
        float voltage;
        float current;
        uint8_t rec_status;
        uint8_t gps_sv_status;
        uint16_t gps_pAcc;
} uav;

void joystick_task(void) {
        /* Normalize data */
        joystick_data.LX = normalize_joystick(joystick_raw.LX);
        joystick_data.LY = normalize_joystick(joystick_raw.LY);
        joystick_data.RX = normalize_joystick(joystick_raw.RX);
        joystick_data.RY = normalize_joystick(joystick_raw.LY);
        joystick_data.L_button = !HAL_GPIO_ReadPin(SW_L_GPIO_Port, SW_L_Pin);
        joystick_data.R_button = !HAL_GPIO_ReadPin(SW_R_GPIO_Port, SW_R_Pin);
        joystick_task_trigger = 0;
}

float normalize_joystick(uint16_t val)
{
        float tmp = (float)val;

        if ((tmp > (JOYSTICK_RANGE + JOYSTICK_ZERO_RANGE)) ||
            (tmp < (JOYSTICK_RANGE - JOYSTICK_ZERO_RANGE))) {
                tmp = JOYSTICK_RANGE - tmp;
                tmp /= JOYSTICK_RANGE;
        } else {
                tmp = 0;
        }
        return tmp;
}

void radio_task(void) {
        tx_finish = 0;
        nrf24l01p_transmit((uint8_t *)&pl, PAYLOAD_WIDTH);
        
}

void print_msg(void) {

}


uint8_t adc_count = 0;
uint32_t total[4] = {0};

void adc_callback(void) {
        adc_count++;
        
        total[0] += (uint32_t)adc_val[0];
        total[1] += (uint32_t)adc_val[1];
        total[2] += (uint32_t)adc_val[2];
        total[3] += (uint32_t)adc_val[3];

        if (adc_count == JOYSTICK_GET_CYCLE) {
                /* Average 10 samples */
                joystick_data.LX = total[0] / JOYSTICK_GET_CYCLE;
                joystick_data.LY = total[1] / JOYSTICK_GET_CYCLE;
                joystick_data.RX = total[2] / JOYSTICK_GET_CYCLE;
                joystick_data.RY = total[3] / JOYSTICK_GET_CYCLE;
                memset(total, 0, sizeof(total));
                joystick_task_trigger = 1;
        }
}


int main(void)
{
        while (1) {
                if (joystick_task_trigger) {
                        joystick_task();
                }
        }
        
}