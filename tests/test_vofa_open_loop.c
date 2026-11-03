/* 真实协议解析+真实应用回调+真实开环状态机，只替换硬件读写。 */
#define main pwm_state_machine_tests
#include "test_pwm_test.c"
#undef main
#include "vofa_hpm_uart.h"
#include "motor_vofa.c"
static const char *rx;
static unsigned tx_count;
static uint8_t tx_bytes[256];
void board_init_uart(UART_Type *ptr) { (void)ptr; }
uint32_t clock_get_frequency(int name) { (void)name; return 24000000; }
int vofa_hpm_uart_init(vofa_hpm_uart_t *port) { return port ? 0 : -1; }
int vofa_hpm_uart_read(void *user, uint8_t *data, size_t capacity)
{
    size_t count=0;
    (void)user;
    while (rx && *rx && count<capacity) data[count++]=(uint8_t)*rx++;
    return (int)count;
}
int vofa_hpm_uart_write(void *user, const uint8_t *data, size_t length)
{
    (void)user;
    CHECK(tx_count+length<=sizeof(tx_bytes));
    memcpy(tx_bytes+tx_count,data,length); tx_count+=(unsigned)length;
    return (int)length;
}
void vofa_hpm_uart_bind(vofa_debug_config_t *config, vofa_hpm_uart_t *port)
{
    config->read=vofa_hpm_uart_read;
    config->write=vofa_hpm_uart_write;
    config->io_user=port;
}
int encoder_to_output_axis(const encoder_data_t *motor, encoder_output_data_t *output)
{
    output->angle_deg=motor->mechanical_angle_deg/MOTOR_SPEED_REDUCTION_RATIO;
    output->speed_rpm=motor->speed_rpm/MOTOR_SPEED_REDUCTION_RATIO;
    output->speed_filtered_rpm=motor->speed_filtered_rpm/MOTOR_SPEED_REDUCTION_RATIO;
    return 0;
}
static int command(const char *text)
{
    rx=text;
    motor_vofa_poll();
    return motor_vofa_context.stats.last_command_result;
}
int main(void)
{
    unsigned i, accepted, rejected;
    CHECK(pwm_state_machine_tests()==0);
    CHECK(motor_vofa_init()==0 && !enabled);
    memset(&input,0,sizeof(input)); fresh();
    CHECK(command("ENABLE,1\r\n")==PWM_TEST_ADC_NOT_READY && !enabled);
    CHECK(motor_vofa_command_result==PWM_TEST_ADC_NOT_READY);
    input.sample_valid=input.current_valid=true; input.offset_state=ADC_OFFSET_READY;
    fresh(); pwm_test_poll(); CHECK(!enabled); /* 被拒绝的命令不能延后自动启动。 */
    CHECK(command("ENABLE,1\r\n")==0 && enabled);
    CHECK(command("ENABLE,1\r")==PWM_TEST_BUSY && enabled);
    CHECK(command("ENABLE,2\r")==-1 && enabled); /* 协议层拒绝非法值。 */
    CHECK(command("SET,SPEED_RPM,1\r")==VOFA_EINVAL && enabled);
    pwm_test_request=1;
    CHECK(command("STOP\r")==0 && !enabled && pwm_test_request==0);
    pwm_test_poll(); CHECK(!enabled);
    fresh(); CHECK(command("ENABLE 1\n")==0 && enabled);
    CHECK(command("ENABLE,0\r")==0 && !enabled);
    fresh(); CHECK(command("ENABLE,1\rSTOP\r")==0 && !enabled);
    fresh(); CHECK(command("ENABLE,1\r")==0 && enabled);
    tick(201,false); CHECK(!enabled && pwm_test_monitor.result==PWM_TEST_ADC_STALE);
    pwm_test_poll(); fresh();
    CHECK(command("ENABLE,1\r")==0 && enabled);
    tick(100000,true); CHECK(!enabled && pwm_test_monitor.result==PWM_TEST_FINISHED);
    input.current_u_ma=MOTOR_PWM_TEST_CURRENT_LIMIT_MA+1; fresh();
    CHECK(command("ENABLE,1\r")==PWM_TEST_OVERCURRENT && !enabled);
    input.current_u_ma=0; fresh();
    accepted=motor_vofa_context.stats.commands_accepted;
    rejected=motor_vofa_context.stats.commands_rejected;
    CHECK(command("PING\r")==0 && !enabled);
    CHECK(motor_vofa_context.stats.commands_accepted==accepted+1 && rejected>=4);
    cycles+=10000; motor_vofa_poll(); motor_vofa_poll();
    CHECK(tx_count>0 && tx_count%16==0);
    for(i=0;i<tx_count;i+=16)
        CHECK(tx_bytes[i+12]==0 && tx_bytes[i+13]==0 && tx_bytes[i+14]==0x80 && tx_bytes[i+15]==0x7f);
    puts("PASS VOFA ENABLE/STOP: real parser, ADC gating, busy/invalid commands, no deferred start, timeout, duration, overcurrent, JustFloat frames");
    return 0;
}
