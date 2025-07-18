`timescale 1ns / 1ps

module uart_loopback(
    input            sys_clk,     // 外部50MHz时钟
    input            sys_rst_n,   // 外部复位信号，低有效
    input            ext_int_in,  // 新增：外部中断输入信号
    input            ext_int_in2,

    // UART端口
    input            uart_rxd,    // UART接收端口
    output           uart_txd,    // UART发送端口
    
    // PWM输出端口
    output [5:0]     pwm_out,     // 6路PWM电机控制
    //蜂鸣器
    output           bee
);

// 参数定义
parameter CLK_FREQ = 50_000_000;  // 系统时钟频率50MHz
parameter UART_BPS = 115_200;     // 串口波特率115200

// PWM参数
parameter CLK_PER_US = 50;           // 每微秒50个时钟周期
parameter PWM_PERIOD = 20000 * CLK_PER_US;  // 20ms = 1_000_000时钟周期
parameter PWM_MIN = 1000 * CLK_PER_US;      // 1000μs = 50_000 
parameter PWM_MAX = 2000 * CLK_PER_US;      // 2000μs = 100_000
parameter STOP_DUTY = 1000*CLK_PER_US;              // 停止 = 0μs
parameter HOVER_DUTY = 1520 * CLK_PER_US;   // 悬停 = 1520μs
parameter FAST_DUTY = 1650 * CLK_PER_US;    // 高速 = 1850μs
parameter YAW_DUTY_H = 1600 * CLK_PER_US;
parameter YAW_DUTY_L = 1460 * CLK_PER_US;
parameter ARM_DUTY = 1200 * CLK_PER_US;     // 待起飞状态占空比(1200μs)
parameter RAMP_STEP = 25;                  // 每周期最大变化0.5μs

// 新增下降控制参数
parameter DESCEND_RATE = 100_000;    // 下降速率(每10ms调整一次)
parameter DESCEND_STEP = 5;        // 每周期下降步长(0.2μs)
parameter MIN_DUTY = 1100 * CLK_PER_US; // 最低安全占空比(1100μs)

// 修改后的平滑减速/加速参数
parameter SMOOTH_RATE = 2_500_000;   // 0.05s间隔(50MHz时钟)
parameter SMOOTH_STEP = 5 * CLK_PER_US; // 每次变化5μs

// 指令定义
parameter CMD_FORWARD      = 8'h01;
parameter CMD_BACKWARD     = 8'h02;
parameter CMD_LEFT         = 8'h03;
parameter CMD_RIGHT        = 8'h04;
parameter CMD_ALL_STOP     = 8'h00;
parameter CMD_STOP      = 8'h05;
parameter CMD_ARM          = 8'h06;  // 怠速
parameter CMD_HOVER        = 8'hFD;
parameter CMD_SMOOTH_DESCEND = 8'h07;  // 减速下降
parameter CMD_THROTTLE = 8'h08;   // 上升
parameter CMD_LEFT_SHIFT   = 8'h09;    // 左移
parameter CMD_RIGHT_SHIFT  = 8'h10;    // 右移
parameter CMD_INCREASE_DUTY = 8'h11;  // 增加占空比
parameter CMD_DECREASE_DUTY = 8'h12;
parameter COUNTER_ZERO = 8'h0E;
// 电机编号定义（俯视图，顺时针方向）
//      M1（CW）  
// M6（CCW）   M2（CCW）  
//      ↑ 机头方向  
// M5（CCW）   M3（CW）  
//      M4（CCW）  
parameter M1 = 0, M2 = 1, M3 = 2, M4 = 3, M5 = 4, M6 = 5;

// 时间参数
parameter BOOST_TIME = 500_000;       // 启动助推时间(10ms)
parameter ACTION_DURATION = 4*25_000_000; // 2秒
parameter ARM_DELAY = 1_000_000;      // 待起飞状态稳定时间(20ms)

// 内部信号
wire        uart_rx_done;
wire [7:0]  uart_rx_data;

//超时报警
wire        cmd_timeout_flag;
wire [7:0]  cmd_timeout_cmd;

// 电机控制信号
reg [31:0] duty_cycle [0:5];
reg        pwm_update_flag;
reg        pwm_enable;

// 控制状态
reg [31:0] boost_counter [0:5];
reg [31:0] action_timer;
reg        in_action;
reg [31:0] descend_timer;
reg        descending;
reg        first_command_received;
reg        arming;  // 待起飞状态标志
reg [31:0] arm_timer;  // 待起飞计时器
reg        smooth_descending;  // 平滑减速状态标志
reg [31:0] smooth_timer; // 平滑变化计时器
reg        smooth_ascending;    // 平滑加速状态标志
reg [7:0] cmd_counter,cmd_counter2;  // 指令计数器
// 中断相关信号
reg [7:0] current_cmd;  // 当前命令状态
reg [1:0] int_sync;      // 中断同步寄存器
wire int_trigger;        // 中断触发信号

//蜂鸣器
wire bee = ext_int_in || ext_int_in2;
integer i;
/*
// 同步化外部中断信号，防止亚稳态
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        int_sync <= 2'b00;
    end else begin
        int_sync <= {int_sync[0], ext_int_in};
    end
end

// 中断触发条件：高电平且处于可中断状态
assign int_trigger = (int_sync == 2'b11) && 
                   ((current_cmd == CMD_FORWARD) ||
                    (current_cmd == CMD_BACKWARD) ||
                    (current_cmd == CMD_LEFT) ||
                    (current_cmd == CMD_RIGHT) ||
                    (current_cmd == CMD_LEFT_SHIFT) ||
                    (current_cmd == CMD_RIGHT_SHIFT));
*/
// 串口模块实例化
uart_rx #(.CLK_FREQ(CLK_FREQ), .UART_BPS(UART_BPS)) u_uart_rx (
    .clk(sys_clk), .rst_n(sys_rst_n),
    .uart_rxd(uart_rxd),
    .uart_rx_done(uart_rx_done),
    .uart_rx_data(uart_rx_data)
);

uart_tx #(.CLK_FREQ(CLK_FREQ), .UART_BPS(UART_BPS)) u_uart_tx (
    .clk(sys_clk), .rst_n(sys_rst_n),
    .uart_tx_en(uart_rx_done),
    .uart_tx_data(uart_rx_data),
    .uart_txd(uart_txd),
    .uart_tx_busy()
);

external_interrupt u_ext_int (
    .clk(sys_clk),
    .rst_n(sys_rst_n),
    .ext_int_in(ext_int_in),
    .ext_int_in2(ext_int_in2),
    .current_cmd(current_cmd),
    .int_trigger(int_trigger)
);

command_timeout u_cmd_timeout (
    .clk(sys_clk),
    .rst_n(sys_rst_n),
    .cmd_received(uart_rx_done),
    .current_cmd(current_cmd),
    .timeout_cmd(cmd_timeout_cmd),
    .timeout_flag(cmd_timeout_flag)
);

// PWM生成模块
generate
    genvar gi;
    for(gi=0; gi<6; gi=gi+1) begin: pwm_gen
        wire [31:0] safe_duty;
        
        assign safe_duty = (duty_cycle[gi] > PWM_MAX) ? PWM_MAX :
                         (duty_cycle[gi] < PWM_MIN) ? PWM_MIN : 
                         duty_cycle[gi];
            
        pwm_motor #(
            .PWM_PERIOD(PWM_PERIOD),
            .PWM_MAX_DUTY(PWM_MAX),
            .RESET_DUTY(STOP_DUTY),
            .RAMP_STEP(RAMP_STEP)
        ) motor (
            .clk(sys_clk),
            .rst_n(sys_rst_n),
            .enable(pwm_enable),
            .control_signal(safe_duty),
            .pwm_out(pwm_out[gi])
        );
    end
endgenerate

// 指令检测逻辑
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        pwm_update_flag <= 0;
    end 
    else if (uart_rx_done) begin
        case(uart_rx_data)
            CMD_FORWARD, CMD_BACKWARD, CMD_LEFT, 
            CMD_RIGHT, CMD_HOVER, CMD_STOP,
            CMD_ARM, CMD_ALL_STOP, CMD_SMOOTH_DESCEND,
            CMD_THROTTLE, CMD_LEFT_SHIFT,
            CMD_RIGHT_SHIFT,CMD_DECREASE_DUTY,CMD_INCREASE_DUTY,COUNTER_ZERO: begin
                pwm_update_flag <= 1;
            end
            default: pwm_update_flag <= 0;
        endcase
    end
    else begin
        pwm_update_flag <= 0;
    end
end

// 运动控制主逻辑
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        // 复位所有状态
        action_timer <= 0;
        in_action <= 0;
        descend_timer <= 0;
        descending <= 0;
        first_command_received <= 0;
        pwm_enable <= 0;
        arming <= 0;  
        arm_timer <= 0;
        smooth_descending <= 0;
        smooth_ascending <= 0;
        smooth_timer <= 0;
        current_cmd <= CMD_ALL_STOP;
        cmd_counter <= 0;  // 复位计数器
        cmd_counter2 <= 0;
        
        for (i=0; i<6; i=i+1) begin
            duty_cycle[i] <= STOP_DUTY;
            boost_counter[i] <= 0;
        end
    end
    else begin
        // 修改后的首次指令检测（允许所有有效指令触发）
        if (uart_rx_done && !first_command_received && 
            (uart_rx_data >= CMD_FORWARD && uart_rx_data <= CMD_HOVER)) begin
            first_command_received <= 1;
        end
                
        // 中断处理 - 具有最高优先级
        if (int_trigger) begin
            // 执行中断处理 - 进入悬停状态
            in_action <= 0;
            descending <= 0;
            smooth_descending <= 0;
            arming <= 0;
            action_timer <= 0;
            pwm_enable <= 1;
            smooth_ascending <= 1;  // 启动平滑加速到悬停
            smooth_timer <= 0;
            current_cmd <= CMD_HOVER;  // 更新当前状态为悬停
            
            for (i=0; i<6; i=i+1) begin
                boost_counter[i] <= 0;
                // 从当前占空比平滑过渡到悬停占空比
                if (duty_cycle[i] > HOVER_DUTY) begin
                    duty_cycle[i] <= duty_cycle[i] - SMOOTH_STEP;
                end else if (duty_cycle[i] < HOVER_DUTY) begin
                    duty_cycle[i] <= duty_cycle[i] + SMOOTH_STEP;
                end
            end
        end
        else begin
            // 记录当前命令
            if ((pwm_update_flag || cmd_timeout_flag) && first_command_received) begin
                current_cmd <= cmd_timeout_flag ? cmd_timeout_cmd : uart_rx_data;
            end
            
            // 待起飞状态处理
            if (arming) begin
                if (arm_timer >= ARM_DELAY) begin
                    arming <= 0;
                    // 保持ARM_DUTY占空比
                    for (i=0; i<6; i=i+1) begin
                        duty_cycle[i] <= ARM_DUTY;
                    end
                end
                else begin
                    arm_timer <= arm_timer + 1;
                end
            end 

            // 下降控制逻辑
            if (descending) begin
                if (descend_timer >= DESCEND_RATE) begin
                    descend_timer <= 0;
                    for (i=0; i<6; i=i+1) begin
                        if (duty_cycle[i] > ((HOVER_DUTY - 42*CLK_PER_US) + DESCEND_STEP*CLK_PER_US)) begin
                            duty_cycle[i] <= duty_cycle[i] - DESCEND_STEP*CLK_PER_US;
                        end
                        else begin
                            duty_cycle[i] <= HOVER_DUTY - 42*CLK_PER_US;
                        end
                    end
                    
                    if (action_timer >= CLK_FREQ) begin
                        descending <= 0;
                        pwm_enable <= 0;
                        for (i=0; i<6; i=i+1) begin
                            duty_cycle[i] <= STOP_DUTY;
                        end
                    end
                    else begin
                        action_timer <= action_timer + 1;
                    end
                end
                else begin
                    descend_timer <= descend_timer + 1;
                end
            end
            
            // 平滑变化控制逻辑
            if (smooth_timer >= SMOOTH_RATE) begin
                smooth_timer <= 0;
                
                // 平滑减速控制
                if (smooth_descending) begin
                    for (i=0; i<6; i=i+1) begin
                        if (duty_cycle[i] > ARM_DUTY) begin
                            if ((duty_cycle[i] - SMOOTH_STEP) > ARM_DUTY) begin
                                duty_cycle[i] <= duty_cycle[i] - SMOOTH_STEP;
                            end
                            else begin
                                duty_cycle[i] <= ARM_DUTY;
                                smooth_descending <= 0; // 减速完成
                            end
                        end
                        else begin
                            duty_cycle[i] <= ARM_DUTY;
                            smooth_descending <= 0; // 已经低于目标值，停止减速
                        end
                    end
                end
                
                // 平滑加速控制
                if (smooth_ascending) begin
                    for (i=0; i<6; i=i+1) begin
                        if (duty_cycle[i] < HOVER_DUTY) begin
                            if ((duty_cycle[i] + SMOOTH_STEP) < HOVER_DUTY) begin
                                duty_cycle[i] <= duty_cycle[i] + SMOOTH_STEP;
                            end
                            else begin
                                duty_cycle[i] <= HOVER_DUTY;
                                smooth_ascending <= 0; // 加速完成
                            end
                        end
                        else begin
                            duty_cycle[i] <= HOVER_DUTY;
                            smooth_ascending <= 0; // 已经高于目标值，停止加速
                        end
                    end
                end
            end
            else begin
                smooth_timer <= smooth_timer + 1;
            end
            
            // 动作持续时间控制
            if (in_action) begin
                if (action_timer >= ACTION_DURATION) begin
                    in_action <= 0;
                    for (i=0; i<6; i=i+1) begin
                        duty_cycle[i] <= HOVER_DUTY;
                    end
                end
                else begin
                    action_timer <= action_timer + 1;
                end
            end
            
            if (pwm_update_flag && first_command_received) begin
                case(uart_rx_data)
                    CMD_ARM: begin  // 待起飞指令处理
                        in_action <= 0;
                        descending <= 0;
                        smooth_descending <= 0;
                        smooth_ascending <= 0;
                        arming <= 1;  // 进入待起飞状态
                        arm_timer <= 0;
                        pwm_enable <= 1;
                        for (i=0; i<6; i=i+1) begin
                            duty_cycle[i] <= ARM_DUTY;
                            boost_counter[i] <= 0;
                        end
                    end
                    
                    CMD_FORWARD: begin  // 前进：后部电机加速，前部电机减速
                        in_action <= 1;
                        descending <= 0;
                        action_timer <= 0;
                        pwm_enable <= 1;
                        // 后部电机加速（M4,M5,M6）
                        duty_cycle[M4] <= FAST_DUTY;
                        duty_cycle[M5] <= FAST_DUTY;
                        duty_cycle[M6] <= FAST_DUTY;
                        // 前部电机减速（M1,M2,M3）
                        duty_cycle[M1] <= HOVER_DUTY;
                        duty_cycle[M2] <= HOVER_DUTY;
                        duty_cycle[M3] <= HOVER_DUTY;
                        // 助推计数器
                        boost_counter[M4] <= BOOST_TIME;
                        boost_counter[M5] <= BOOST_TIME;
                        boost_counter[M6] <= BOOST_TIME;
                    end
                    
                    CMD_BACKWARD: begin  // 后退：前部电机加速，后部电机减速
                        in_action <= 1;
                        descending <= 0;
                        action_timer <= 0;
                        pwm_enable <= 1;
                        // 前部电机加速（M1,M2,M3）
                        duty_cycle[M1] <= FAST_DUTY;
                        duty_cycle[M2] <= FAST_DUTY;
                        duty_cycle[M3] <= FAST_DUTY;
                        // 后部电机减速（M4,M5,M6）
                        duty_cycle[M4] <= HOVER_DUTY;
                        duty_cycle[M5] <= HOVER_DUTY;
                        duty_cycle[M6] <= HOVER_DUTY;
                        // 助推计数器
                        boost_counter[M1] <= BOOST_TIME;
                        boost_counter[M2] <= BOOST_TIME;
                        boost_counter[M3] <= BOOST_TIME;
                    end
                    
                    CMD_LEFT: begin  // 左转：CCW电机加速，CW电机减速
                        in_action <= 1;
                        descending <= 0;
                        action_timer <= 0;
                        pwm_enable <= 1;
                        // CCW电机加速（M2,M4,M6）
                        duty_cycle[M2] <= FAST_DUTY;
                        duty_cycle[M4] <= FAST_DUTY;
                        duty_cycle[M6] <= FAST_DUTY;
                        // CW电机减速（M1,M3,M5）
                        duty_cycle[M1] <= HOVER_DUTY;
                        duty_cycle[M3] <= HOVER_DUTY;
                        duty_cycle[M5] <= HOVER_DUTY;
                        // 助推计数器（加速电机
                        boost_counter[M2] <= BOOST_TIME;
                        boost_counter[M4] <= BOOST_TIME;
                        boost_counter[M6] <= BOOST_TIME;
                    end
                    
                    CMD_RIGHT: begin  // 右转：CW电机加速，CCW电机减速
                        in_action <= 1;
                        descending <= 0;
                        action_timer <= 0;
                        pwm_enable <= 1;
                        // CW电机加速（M1,M3,M5）
                        duty_cycle[M1] <= FAST_DUTY;
                        duty_cycle[M3] <= FAST_DUTY;
                        duty_cycle[M5] <= FAST_DUTY;
                        // CCW电机减速（M2,M4,M6）
                        duty_cycle[M2] <= HOVER_DUTY;
                        duty_cycle[M4] <= HOVER_DUTY;
                        duty_cycle[M6] <= HOVER_DUTY;
                        // 助推计数器
                        boost_counter[M1] <= BOOST_TIME;
                        boost_counter[M3] <= BOOST_TIME;
                        boost_counter[M5] <= BOOST_TIME;
                    end
                    
                    CMD_LEFT_SHIFT: begin  // 左移：右侧电机加速，左侧电机减速
                        in_action <= 1;
                        descending <= 0;
                        action_timer <= 0;
                        pwm_enable <= 1;
                        // 右侧电机加速（M3,M4）
                        duty_cycle[M3] <= YAW_DUTY_H;
                        duty_cycle[M4] <= YAW_DUTY_H;
                        // 左侧电机减速（M1,M6）
                        duty_cycle[M1] <= YAW_DUTY_L;
                        duty_cycle[M6] <= YAW_DUTY_L;
                        // 中间电机保持悬停（M2,M5）
                        duty_cycle[M2] <= HOVER_DUTY;
                        duty_cycle[M5] <= HOVER_DUTY;
                        // 助推计数器
                        boost_counter[M3] <= BOOST_TIME;
                        boost_counter[M4] <= BOOST_TIME;
                    end
                    
                    CMD_RIGHT_SHIFT: begin  // 右移：左侧电机加速，右侧电机减速
                        in_action <= 1;
                        descending <= 0;
                        action_timer <= 0;
                        pwm_enable <= 1;
                        // 左侧电机加速（M1,M6）
                        duty_cycle[M1] <= YAW_DUTY_H;
                        duty_cycle[M6] <= YAW_DUTY_H;
                        // 右侧电机减速（M3,M4）
                        duty_cycle[M3] <= YAW_DUTY_L;
                        duty_cycle[M4] <= YAW_DUTY_L;
                        // 中间电机保持悬停（M2,M5）
                        duty_cycle[M2] <= HOVER_DUTY;
                        duty_cycle[M5] <= HOVER_DUTY;
                        // 助推计数器
                        boost_counter[M1] <= BOOST_TIME;
                        boost_counter[M6] <= BOOST_TIME;
                    end
                    
                    CMD_HOVER: begin  // 悬停指令
                        in_action <= 0;
                        descending <= 0;
                        smooth_descending <= 0;
                        arming <= 0;
                        action_timer <= 0;
                        pwm_enable <= 1;
                        smooth_ascending <= 1;  // 启动平滑加速
                        smooth_timer <= 0;
                        for (i=0; i<6; i=i+1) begin
                            boost_counter[i] <= 0;
                            // 确保从ARM_DUTY(1200μs)开始加速
                            if (duty_cycle[i] != ARM_DUTY) begin
                                duty_cycle[i] <= ARM_DUTY;
                            end
                        end
                    end
                    
                    CMD_STOP: begin  
                        in_action <= 0;
                        descending <= 1;
                        smooth_descending <= 0;
                        smooth_ascending <= 0;
                        descend_timer <= 0;
                        action_timer <= 0;
                        pwm_enable <= 1;
                        for (i=0; i<6; i=i+1) begin
                            boost_counter[i] <= 0;
                            if (duty_cycle[i] < STOP_DUTY) begin
                                duty_cycle[i] <= STOP_DUTY;
                            end
                        end
                    end
                    
                    CMD_SMOOTH_DESCEND: begin  // 平滑减速指令(下降)
                        in_action <= 0;
                        descending <= 0;
                        smooth_descending <= 1;
                        smooth_ascending <= 0;
                        smooth_timer <= 0;
                        arming <= 0;
                        pwm_enable <= 1;
                        for (i=0; i<6; i=i+1) begin
                            boost_counter[i] <= 0;
                            // 确保从HOVER_DUTY(1520μs)开始减速
                            if (duty_cycle[i] != HOVER_DUTY) begin
                                duty_cycle[i] <= HOVER_DUTY;
                            end
                        end
                    end
                    
                    CMD_THROTTLE: begin  // 上升
                        in_action <= 0;
                        descending <= 0;
                        smooth_descending <= 0;
                        smooth_ascending <= 0;
                        arming <= 0;
                        pwm_enable <= 1;
                        for (i=0; i<6; i=i+1) begin
                            duty_cycle[i] <= FAST_DUTY;
                            boost_counter[i] <= 0;
                        end
                    end
                    
                    CMD_ALL_STOP: begin  // 急停指令
                        in_action <= 0;
                        descending <= 0;
                        smooth_descending <= 0;
                        smooth_ascending <= 0;
                        arming <= 0;
                        pwm_enable <= 0;
                        for (i=0; i<6; i=i+1) begin
                            duty_cycle[i] <= STOP_DUTY;
                            boost_counter[i] <= 0;
                        end
                    end
                    CMD_INCREASE_DUTY:begin
                        cmd_counter <= cmd_counter + 1;  // 计数器加1
                        in_action <= 0;
                        descending <= 0;
                        smooth_descending <= 0;
                        smooth_ascending <= 0;
                        smooth_timer <= 0;
                        arming <= 0;
                        pwm_enable <= 1;
                        for (i=0; i<6; i=i+1) begin
            // 计算增加21μs后的占空比（不超过PWM_MAX）
                            if ((duty_cycle[i] + cmd_counter*21*CLK_PER_US) > PWM_MAX) begin
                                duty_cycle[i] <= PWM_MAX;
                                cmd_counter <= cmd_counter;
                            end
                            else begin
                                duty_cycle[i] <= duty_cycle[i] + cmd_counter*21*CLK_PER_US;
                            end
                        end        
                    end
                    CMD_DECREASE_DUTY:begin
                        cmd_counter2 <= cmd_counter2 + 1;  // 计数器加1
                        in_action <= 0;
                        descending <= 0;
                        smooth_descending <= 0;
                        smooth_ascending <= 0;
                        smooth_timer <= 0;
                        arming <= 0;
                        pwm_enable <= 1;
                       
                        for (i=0; i<6; i=i+1) begin
            // 计算增加21μs后的占空比（不低于PWM_MIN）
                            if ((duty_cycle[i] - cmd_counter2*21*CLK_PER_US) < PWM_MIN) begin
                                duty_cycle[i] <= PWM_MIN;
                                cmd_counter2 <= cmd_counter2;
                            end
                            else begin
                                duty_cycle[i] <= duty_cycle[i] - cmd_counter2*21*CLK_PER_US;
                            end
                        end  
                    end
                    COUNTER_ZERO:begin
                       cmd_counter <= 0;
                       cmd_counter2 <= 0;
                    end
                    default: ;
                endcase
            end
            
            // 助推计数器递减
            for (i=0; i<6; i=i+1) begin
                if (boost_counter[i] > 0) begin
                    boost_counter[i] <= boost_counter[i] - 1;
                    if (boost_counter[i] == 1) begin
                        if (duty_cycle[i] > FAST_DUTY) 
                            duty_cycle[i] <= FAST_DUTY;
                    end
                end
            end
        end
    end
end

endmodule





// 电机编号定义（俯视图，顺时针方向）
//      M1（CW）  
// M6（CCW）   M2（CCW）  
//      ↑ 机头方向  
// M5（CCW）   M3（CW）  
//      M4（CCW）  