/* 编码器移植阶段：初始化板级和 VOFA，编码器数据由后续移植模块更新。 */
#include "board.h"
#include "motor_vofa.h"

int main(void)
{
    board_init();
    if (motor_vofa_init() != 0) {
        while (1) { /* 使用调试器查看 motor_vofa_io_result。 */ }
    }
    
    while (1) {
        /* 接入编码器后，取得同一次采样的计数、机械角度，再调用：
         * motor_vofa_set_encoder(position_count, angle_deg); */
        motor_vofa_poll();
    }
}
