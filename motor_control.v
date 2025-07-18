`timescale 1ns / 1ps

module pwm_motor(
    input clk,                  // 系统时钟（50MHz）
    input rst_n,                // 复位信号（低电平有效）
    input enable,               // PWM使能信号
    input [31:0] control_signal, // 目标占空比控制信号
    output reg pwm_out          // PWM输出信号
);

    // PWM周期参数（与顶层模块保持一致）
    parameter PWM_PERIOD = 20000 * 50;  // 20ms = 1_000_000时钟周期
    parameter PWM_MAX_DUTY = 2000 * 50; // 2000μs = 100_000（与顶层一致）
    parameter RESET_DUTY = 0;           // 复位时占空比为0
    parameter RAMP_STEP = 25;          // 每周期最大变化0.5μs（更平滑）
    
    reg [31:0] counter;         // PWM周期计数器
    reg [31:0] current_duty;    // 当前实际占空比（用于斜坡控制）
    reg [31:0] target_duty;     // 目标占空比（锁存输入信号）

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // 复位初始化
            counter <= 0;
            pwm_out <= 1'b0;
            current_duty <= RESET_DUTY;
            target_duty <= RESET_DUTY;
        end
        else begin
            // 当使能时锁存目标占空比，否则保持为0
            if (enable) begin
                // 安全锁存目标占空比（双重保护）
                target_duty <= (control_signal > PWM_MAX_DUTY) ? PWM_MAX_DUTY :
                              (control_signal < (1100 * 50)) ? (1100 * 50) : // 不低于1100μs
                              control_signal;
            end
            else begin
                target_duty <= 0;  // 不使能时目标占空比为0
            end
            
            // 斜坡控制：带加速度限制
            if (current_duty < target_duty) begin
                current_duty <= (target_duty - current_duty > RAMP_STEP) ? 
                              current_duty + RAMP_STEP : target_duty;
            end
            else if (current_duty > target_duty) begin
                current_duty <= (current_duty - target_duty > RAMP_STEP) ? 
                              current_duty - RAMP_STEP : target_duty;
            end
            
            // 计数器控制
            counter <= (counter < PWM_PERIOD - 1) ? counter + 1 : 0;
            
            // PWM输出生成（不使能时输出始终为0）
            pwm_out <= enable && (counter < current_duty) ? 1'b1 : 1'b0;
        end
    end
endmodule