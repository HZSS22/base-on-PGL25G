`timescale 1ns / 1ps

module uart_loopback(
    input            sys_clk,     // �ⲿ50MHzʱ��
    input            sys_rst_n,   // �ⲿ��λ�źţ�����Ч
    input            ext_int_in,  // �������ⲿ�ж������ź�
    input            ext_int_in2,

    // UART�˿�
    input            uart_rxd,    // UART���ն˿�
    output           uart_txd,    // UART���Ͷ˿�
    
    // PWM����˿�
    output [5:0]     pwm_out,     // 6·PWM�������
    //������
    output           bee
);

// ��������
parameter CLK_FREQ = 50_000_000;  // ϵͳʱ��Ƶ��50MHz
parameter UART_BPS = 115_200;     // ���ڲ�����115200

// PWM����
parameter CLK_PER_US = 50;           // ÿ΢��50��ʱ������
parameter PWM_PERIOD = 20000 * CLK_PER_US;  // 20ms = 1_000_000ʱ������
parameter PWM_MIN = 1000 * CLK_PER_US;      // 1000��s = 50_000 
parameter PWM_MAX = 2000 * CLK_PER_US;      // 2000��s = 100_000
parameter STOP_DUTY = 1000*CLK_PER_US;              // ֹͣ = 0��s
parameter HOVER_DUTY = 1520 * CLK_PER_US;   // ��ͣ = 1520��s
parameter FAST_DUTY = 1650 * CLK_PER_US;    // ���� = 1850��s
parameter YAW_DUTY_H = 1600 * CLK_PER_US;
parameter YAW_DUTY_L = 1460 * CLK_PER_US;
parameter ARM_DUTY = 1200 * CLK_PER_US;     // �����״̬ռ�ձ�(1200��s)
parameter RAMP_STEP = 25;                  // ÿ�������仯0.5��s

// �����½����Ʋ���
parameter DESCEND_RATE = 100_000;    // �½�����(ÿ10ms����һ��)
parameter DESCEND_STEP = 5;        // ÿ�����½�����(0.2��s)
parameter MIN_DUTY = 1100 * CLK_PER_US; // ��Ͱ�ȫռ�ձ�(1100��s)

// �޸ĺ��ƽ������/���ٲ���
parameter SMOOTH_RATE = 2_500_000;   // 0.05s���(50MHzʱ��)
parameter SMOOTH_STEP = 5 * CLK_PER_US; // ÿ�α仯5��s

// ָ���
parameter CMD_FORWARD      = 8'h01;
parameter CMD_BACKWARD     = 8'h02;
parameter CMD_LEFT         = 8'h03;
parameter CMD_RIGHT        = 8'h04;
parameter CMD_ALL_STOP     = 8'h00;
parameter CMD_STOP      = 8'h05;
parameter CMD_ARM          = 8'h06;  // ����
parameter CMD_HOVER        = 8'hFD;
parameter CMD_SMOOTH_DESCEND = 8'h07;  // �����½�
parameter CMD_THROTTLE = 8'h08;   // ����
parameter CMD_LEFT_SHIFT   = 8'h09;    // ����
parameter CMD_RIGHT_SHIFT  = 8'h10;    // ����
parameter CMD_INCREASE_DUTY = 8'h11;  // ����ռ�ձ�
parameter CMD_DECREASE_DUTY = 8'h12;
parameter COUNTER_ZERO = 8'h0E;
// �����Ŷ��壨����ͼ��˳ʱ�뷽��
//      M1��CW��  
// M6��CCW��   M2��CCW��  
//      �� ��ͷ����  
// M5��CCW��   M3��CW��  
//      M4��CCW��  
parameter M1 = 0, M2 = 1, M3 = 2, M4 = 3, M5 = 4, M6 = 5;

// ʱ�����
parameter BOOST_TIME = 500_000;       // ��������ʱ��(10ms)
parameter ACTION_DURATION = 4*25_000_000; // 2��
parameter ARM_DELAY = 1_000_000;      // �����״̬�ȶ�ʱ��(20ms)

// �ڲ��ź�
wire        uart_rx_done;
wire [7:0]  uart_rx_data;

//��ʱ����
wire        cmd_timeout_flag;
wire [7:0]  cmd_timeout_cmd;

// ��������ź�
reg [31:0] duty_cycle [0:5];
reg        pwm_update_flag;
reg        pwm_enable;

// ����״̬
reg [31:0] boost_counter [0:5];
reg [31:0] action_timer;
reg        in_action;
reg [31:0] descend_timer;
reg        descending;
reg        first_command_received;
reg        arming;  // �����״̬��־
reg [31:0] arm_timer;  // ����ɼ�ʱ��
reg        smooth_descending;  // ƽ������״̬��־
reg [31:0] smooth_timer; // ƽ���仯��ʱ��
reg        smooth_ascending;    // ƽ������״̬��־
reg [7:0] cmd_counter,cmd_counter2;  // ָ�������
// �ж�����ź�
reg [7:0] current_cmd;  // ��ǰ����״̬
reg [1:0] int_sync;      // �ж�ͬ���Ĵ���
wire int_trigger;        // �жϴ����ź�

//������
wire bee = ext_int_in || ext_int_in2;
integer i;
/*
// ͬ�����ⲿ�ж��źţ���ֹ����̬
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        int_sync <= 2'b00;
    end else begin
        int_sync <= {int_sync[0], ext_int_in};
    end
end

// �жϴ����������ߵ�ƽ�Ҵ��ڿ��ж�״̬
assign int_trigger = (int_sync == 2'b11) && 
                   ((current_cmd == CMD_FORWARD) ||
                    (current_cmd == CMD_BACKWARD) ||
                    (current_cmd == CMD_LEFT) ||
                    (current_cmd == CMD_RIGHT) ||
                    (current_cmd == CMD_LEFT_SHIFT) ||
                    (current_cmd == CMD_RIGHT_SHIFT));
*/
// ����ģ��ʵ����
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

// PWM����ģ��
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

// ָ�����߼�
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

// �˶��������߼�
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        // ��λ����״̬
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
        cmd_counter <= 0;  // ��λ������
        cmd_counter2 <= 0;
        
        for (i=0; i<6; i=i+1) begin
            duty_cycle[i] <= STOP_DUTY;
            boost_counter[i] <= 0;
        end
    end
    else begin
        // �޸ĺ���״�ָ���⣨����������Чָ�����
        if (uart_rx_done && !first_command_received && 
            (uart_rx_data >= CMD_FORWARD && uart_rx_data <= CMD_HOVER)) begin
            first_command_received <= 1;
        end
                
        // �жϴ��� - ����������ȼ�
        if (int_trigger) begin
            // ִ���жϴ��� - ������ͣ״̬
            in_action <= 0;
            descending <= 0;
            smooth_descending <= 0;
            arming <= 0;
            action_timer <= 0;
            pwm_enable <= 1;
            smooth_ascending <= 1;  // ����ƽ�����ٵ���ͣ
            smooth_timer <= 0;
            current_cmd <= CMD_HOVER;  // ���µ�ǰ״̬Ϊ��ͣ
            
            for (i=0; i<6; i=i+1) begin
                boost_counter[i] <= 0;
                // �ӵ�ǰռ�ձ�ƽ�����ɵ���ͣռ�ձ�
                if (duty_cycle[i] > HOVER_DUTY) begin
                    duty_cycle[i] <= duty_cycle[i] - SMOOTH_STEP;
                end else if (duty_cycle[i] < HOVER_DUTY) begin
                    duty_cycle[i] <= duty_cycle[i] + SMOOTH_STEP;
                end
            end
        end
        else begin
            // ��¼��ǰ����
            if ((pwm_update_flag || cmd_timeout_flag) && first_command_received) begin
                current_cmd <= cmd_timeout_flag ? cmd_timeout_cmd : uart_rx_data;
            end
            
            // �����״̬����
            if (arming) begin
                if (arm_timer >= ARM_DELAY) begin
                    arming <= 0;
                    // ����ARM_DUTYռ�ձ�
                    for (i=0; i<6; i=i+1) begin
                        duty_cycle[i] <= ARM_DUTY;
                    end
                end
                else begin
                    arm_timer <= arm_timer + 1;
                end
            end 

            // �½������߼�
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
            
            // ƽ���仯�����߼�
            if (smooth_timer >= SMOOTH_RATE) begin
                smooth_timer <= 0;
                
                // ƽ�����ٿ���
                if (smooth_descending) begin
                    for (i=0; i<6; i=i+1) begin
                        if (duty_cycle[i] > ARM_DUTY) begin
                            if ((duty_cycle[i] - SMOOTH_STEP) > ARM_DUTY) begin
                                duty_cycle[i] <= duty_cycle[i] - SMOOTH_STEP;
                            end
                            else begin
                                duty_cycle[i] <= ARM_DUTY;
                                smooth_descending <= 0; // �������
                            end
                        end
                        else begin
                            duty_cycle[i] <= ARM_DUTY;
                            smooth_descending <= 0; // �Ѿ�����Ŀ��ֵ��ֹͣ����
                        end
                    end
                end
                
                // ƽ�����ٿ���
                if (smooth_ascending) begin
                    for (i=0; i<6; i=i+1) begin
                        if (duty_cycle[i] < HOVER_DUTY) begin
                            if ((duty_cycle[i] + SMOOTH_STEP) < HOVER_DUTY) begin
                                duty_cycle[i] <= duty_cycle[i] + SMOOTH_STEP;
                            end
                            else begin
                                duty_cycle[i] <= HOVER_DUTY;
                                smooth_ascending <= 0; // �������
                            end
                        end
                        else begin
                            duty_cycle[i] <= HOVER_DUTY;
                            smooth_ascending <= 0; // �Ѿ�����Ŀ��ֵ��ֹͣ����
                        end
                    end
                end
            end
            else begin
                smooth_timer <= smooth_timer + 1;
            end
            
            // ��������ʱ�����
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
                    CMD_ARM: begin  // �����ָ���
                        in_action <= 0;
                        descending <= 0;
                        smooth_descending <= 0;
                        smooth_ascending <= 0;
                        arming <= 1;  // ��������״̬
                        arm_timer <= 0;
                        pwm_enable <= 1;
                        for (i=0; i<6; i=i+1) begin
                            duty_cycle[i] <= ARM_DUTY;
                            boost_counter[i] <= 0;
                        end
                    end
                    
                    CMD_FORWARD: begin  // ǰ�����󲿵�����٣�ǰ���������
                        in_action <= 1;
                        descending <= 0;
                        action_timer <= 0;
                        pwm_enable <= 1;
                        // �󲿵�����٣�M4,M5,M6��
                        duty_cycle[M4] <= FAST_DUTY;
                        duty_cycle[M5] <= FAST_DUTY;
                        duty_cycle[M6] <= FAST_DUTY;
                        // ǰ��������٣�M1,M2,M3��
                        duty_cycle[M1] <= HOVER_DUTY;
                        duty_cycle[M2] <= HOVER_DUTY;
                        duty_cycle[M3] <= HOVER_DUTY;
                        // ���Ƽ�����
                        boost_counter[M4] <= BOOST_TIME;
                        boost_counter[M5] <= BOOST_TIME;
                        boost_counter[M6] <= BOOST_TIME;
                    end
                    
                    CMD_BACKWARD: begin  // ���ˣ�ǰ��������٣��󲿵������
                        in_action <= 1;
                        descending <= 0;
                        action_timer <= 0;
                        pwm_enable <= 1;
                        // ǰ��������٣�M1,M2,M3��
                        duty_cycle[M1] <= FAST_DUTY;
                        duty_cycle[M2] <= FAST_DUTY;
                        duty_cycle[M3] <= FAST_DUTY;
                        // �󲿵�����٣�M4,M5,M6��
                        duty_cycle[M4] <= HOVER_DUTY;
                        duty_cycle[M5] <= HOVER_DUTY;
                        duty_cycle[M6] <= HOVER_DUTY;
                        // ���Ƽ�����
                        boost_counter[M1] <= BOOST_TIME;
                        boost_counter[M2] <= BOOST_TIME;
                        boost_counter[M3] <= BOOST_TIME;
                    end
                    
                    CMD_LEFT: begin  // ��ת��CCW������٣�CW�������
                        in_action <= 1;
                        descending <= 0;
                        action_timer <= 0;
                        pwm_enable <= 1;
                        // CCW������٣�M2,M4,M6��
                        duty_cycle[M2] <= FAST_DUTY;
                        duty_cycle[M4] <= FAST_DUTY;
                        duty_cycle[M6] <= FAST_DUTY;
                        // CW������٣�M1,M3,M5��
                        duty_cycle[M1] <= HOVER_DUTY;
                        duty_cycle[M3] <= HOVER_DUTY;
                        duty_cycle[M5] <= HOVER_DUTY;
                        // ���Ƽ����������ٵ��
                        boost_counter[M2] <= BOOST_TIME;
                        boost_counter[M4] <= BOOST_TIME;
                        boost_counter[M6] <= BOOST_TIME;
                    end
                    
                    CMD_RIGHT: begin  // ��ת��CW������٣�CCW�������
                        in_action <= 1;
                        descending <= 0;
                        action_timer <= 0;
                        pwm_enable <= 1;
                        // CW������٣�M1,M3,M5��
                        duty_cycle[M1] <= FAST_DUTY;
                        duty_cycle[M3] <= FAST_DUTY;
                        duty_cycle[M5] <= FAST_DUTY;
                        // CCW������٣�M2,M4,M6��
                        duty_cycle[M2] <= HOVER_DUTY;
                        duty_cycle[M4] <= HOVER_DUTY;
                        duty_cycle[M6] <= HOVER_DUTY;
                        // ���Ƽ�����
                        boost_counter[M1] <= BOOST_TIME;
                        boost_counter[M3] <= BOOST_TIME;
                        boost_counter[M5] <= BOOST_TIME;
                    end
                    
                    CMD_LEFT_SHIFT: begin  // ���ƣ��Ҳ������٣����������
                        in_action <= 1;
                        descending <= 0;
                        action_timer <= 0;
                        pwm_enable <= 1;
                        // �Ҳ������٣�M3,M4��
                        duty_cycle[M3] <= YAW_DUTY_H;
                        duty_cycle[M4] <= YAW_DUTY_H;
                        // ��������٣�M1,M6��
                        duty_cycle[M1] <= YAW_DUTY_L;
                        duty_cycle[M6] <= YAW_DUTY_L;
                        // �м���������ͣ��M2,M5��
                        duty_cycle[M2] <= HOVER_DUTY;
                        duty_cycle[M5] <= HOVER_DUTY;
                        // ���Ƽ�����
                        boost_counter[M3] <= BOOST_TIME;
                        boost_counter[M4] <= BOOST_TIME;
                    end
                    
                    CMD_RIGHT_SHIFT: begin  // ���ƣ���������٣��Ҳ�������
                        in_action <= 1;
                        descending <= 0;
                        action_timer <= 0;
                        pwm_enable <= 1;
                        // ��������٣�M1,M6��
                        duty_cycle[M1] <= YAW_DUTY_H;
                        duty_cycle[M6] <= YAW_DUTY_H;
                        // �Ҳ������٣�M3,M4��
                        duty_cycle[M3] <= YAW_DUTY_L;
                        duty_cycle[M4] <= YAW_DUTY_L;
                        // �м���������ͣ��M2,M5��
                        duty_cycle[M2] <= HOVER_DUTY;
                        duty_cycle[M5] <= HOVER_DUTY;
                        // ���Ƽ�����
                        boost_counter[M1] <= BOOST_TIME;
                        boost_counter[M6] <= BOOST_TIME;
                    end
                    
                    CMD_HOVER: begin  // ��ָͣ��
                        in_action <= 0;
                        descending <= 0;
                        smooth_descending <= 0;
                        arming <= 0;
                        action_timer <= 0;
                        pwm_enable <= 1;
                        smooth_ascending <= 1;  // ����ƽ������
                        smooth_timer <= 0;
                        for (i=0; i<6; i=i+1) begin
                            boost_counter[i] <= 0;
                            // ȷ����ARM_DUTY(1200��s)��ʼ����
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
                    
                    CMD_SMOOTH_DESCEND: begin  // ƽ������ָ��(�½�)
                        in_action <= 0;
                        descending <= 0;
                        smooth_descending <= 1;
                        smooth_ascending <= 0;
                        smooth_timer <= 0;
                        arming <= 0;
                        pwm_enable <= 1;
                        for (i=0; i<6; i=i+1) begin
                            boost_counter[i] <= 0;
                            // ȷ����HOVER_DUTY(1520��s)��ʼ����
                            if (duty_cycle[i] != HOVER_DUTY) begin
                                duty_cycle[i] <= HOVER_DUTY;
                            end
                        end
                    end
                    
                    CMD_THROTTLE: begin  // ����
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
                    
                    CMD_ALL_STOP: begin  // ��ָͣ��
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
                        cmd_counter <= cmd_counter + 1;  // ��������1
                        in_action <= 0;
                        descending <= 0;
                        smooth_descending <= 0;
                        smooth_ascending <= 0;
                        smooth_timer <= 0;
                        arming <= 0;
                        pwm_enable <= 1;
                        for (i=0; i<6; i=i+1) begin
            // ��������21��s���ռ�ձȣ�������PWM_MAX��
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
                        cmd_counter2 <= cmd_counter2 + 1;  // ��������1
                        in_action <= 0;
                        descending <= 0;
                        smooth_descending <= 0;
                        smooth_ascending <= 0;
                        smooth_timer <= 0;
                        arming <= 0;
                        pwm_enable <= 1;
                       
                        for (i=0; i<6; i=i+1) begin
            // ��������21��s���ռ�ձȣ�������PWM_MIN��
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
            
            // ���Ƽ������ݼ�
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





// �����Ŷ��壨����ͼ��˳ʱ�뷽��
//      M1��CW��  
// M6��CCW��   M2��CCW��  
//      �� ��ͷ����  
// M5��CCW��   M3��CW��  
//      M4��CCW��  