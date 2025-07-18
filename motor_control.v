`timescale 1ns / 1ps

module pwm_motor(
    input clk,                  // ϵͳʱ�ӣ�50MHz��
    input rst_n,                // ��λ�źţ��͵�ƽ��Ч��
    input enable,               // PWMʹ���ź�
    input [31:0] control_signal, // Ŀ��ռ�ձȿ����ź�
    output reg pwm_out          // PWM����ź�
);

    // PWM���ڲ������붥��ģ�鱣��һ�£�
    parameter PWM_PERIOD = 20000 * 50;  // 20ms = 1_000_000ʱ������
    parameter PWM_MAX_DUTY = 2000 * 50; // 2000��s = 100_000���붥��һ�£�
    parameter RESET_DUTY = 0;           // ��λʱռ�ձ�Ϊ0
    parameter RAMP_STEP = 25;          // ÿ�������仯0.5��s����ƽ����
    
    reg [31:0] counter;         // PWM���ڼ�����
    reg [31:0] current_duty;    // ��ǰʵ��ռ�ձȣ�����б�¿��ƣ�
    reg [31:0] target_duty;     // Ŀ��ռ�ձȣ����������źţ�

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // ��λ��ʼ��
            counter <= 0;
            pwm_out <= 1'b0;
            current_duty <= RESET_DUTY;
            target_duty <= RESET_DUTY;
        end
        else begin
            // ��ʹ��ʱ����Ŀ��ռ�ձȣ����򱣳�Ϊ0
            if (enable) begin
                // ��ȫ����Ŀ��ռ�ձȣ�˫�ر�����
                target_duty <= (control_signal > PWM_MAX_DUTY) ? PWM_MAX_DUTY :
                              (control_signal < (1100 * 50)) ? (1100 * 50) : // ������1100��s
                              control_signal;
            end
            else begin
                target_duty <= 0;  // ��ʹ��ʱĿ��ռ�ձ�Ϊ0
            end
            
            // б�¿��ƣ������ٶ�����
            if (current_duty < target_duty) begin
                current_duty <= (target_duty - current_duty > RAMP_STEP) ? 
                              current_duty + RAMP_STEP : target_duty;
            end
            else if (current_duty > target_duty) begin
                current_duty <= (current_duty - target_duty > RAMP_STEP) ? 
                              current_duty - RAMP_STEP : target_duty;
            end
            
            // ����������
            counter <= (counter < PWM_PERIOD - 1) ? counter + 1 : 0;
            
            // PWM������ɣ���ʹ��ʱ���ʼ��Ϊ0��
            pwm_out <= enable && (counter < current_duty) ? 1'b1 : 1'b0;
        end
    end
endmodule