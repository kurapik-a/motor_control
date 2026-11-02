/* 编码器移植阶段：主循环读取 QEI0，计算角度/速度并交给 VOFA 上传。 */
#include "board.h"
#include "encoder.h"
#include "motor_vofa.h"

/* 调试器中查看编码器初始化/读取结果；含义见 encoder.h。 */
volatile int encoder_result;

int main(void)
{
    encoder_data_t sample;
    board_init();
    encoder_result = encoder_init();
    if (encoder_result != 0) {
        while (1) { /* 检查 QEI 时钟和 encoder_result。 */ }
    }
    if (motor_vofa_init() != 0) {
        while (1) { /* 使用调试器查看 motor_vofa_io_result。 */ }
    }
    while (1) {
        encoder_result = encoder_read(&sample);
        if (encoder_result == 0) {
            motor_vofa_set_encoder(&sample);
        }
        motor_vofa_poll();
    }
}
