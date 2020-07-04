// Your code

module CHIP(clk,
            rst_n,
            // For mem_D
            mem_wen_D,
            mem_addr_D,
            mem_wdata_D,
            mem_rdata_D,
            // For mem_I
            mem_addr_I,
            mem_rdata_I);

    input         clk, rst_n ;
    // For mem_D
    output        mem_wen_D  ;
    output [31:0] mem_addr_D ;
    output [31:0] mem_wdata_D;
    input  [31:0] mem_rdata_D;
    // For mem_I
    output [31:0] mem_addr_I ;
    input  [31:0] mem_rdata_I;
    
    //---------------------------------------//
    // Do not modify this part!!!            //
    // Exception: You may change wire to reg //
    reg    [31:0] PC          ;              //
    wire   [31:0] PC_nxt      ;              //
    reg           regWrite    ;              //
    wire   [ 4:0] rs1, rs2, rd;              //
    wire   [31:0] rs1_data    ;              //
    wire   [31:0] rs2_data    ;              //
    reg    [31:0] rd_data     ;              //
    //---------------------------------------//

    // Todo: other wire/reg
    wire and_out, or_out, do_mul, mul_ready, lock; 
    wire [4:0] shamt;
    wire [31:0] Instruction, PC_add4, PC_add_imm, reg1_add_imm, first_mux_out, PC_temp, mul_out;
    wire [63:0] mul_out_temp;
    wire [31:0] ALUin1, ALUin2;
    reg branch, jal, jalr, ALUsrc, memRead, memWrite, zero; 
    reg [1:0]  memToReg, ALUop;
    reg [3:0]  ALU_ctrl;
    reg [31:0]  Immediate, ALU_out;
    //---------------------------------------//
    // Do not modify this part!!!            //
    reg_file reg0(                           //
        .clk(clk),                           //
        .rst_n(rst_n),                       //
        .wen(regWrite),                      //
        .a1(rs1),                            //
        .a2(rs2),                            //
        .aw(rd),                             //
        .d(rd_data),                         //
        .q1(rs1_data),                       //
        .q2(rs2_data));                      //
    //---------------------------------------//
    
    // Todo: any combinational/sequential circuit
    assign Instruction = mem_rdata_I;
    assign mem_addr_I = PC;
    assign rs1 = Instruction[19:15];
    assign rs2 = Instruction[24:20];
    assign rd  = Instruction[11: 7];

    always@(*) begin // immediate generation
        case(Instruction[6:2])
            5'b00100:
                Immediate = { {21{ Instruction[31] }}, Instruction[30:20] }; // I-type
            5'b11001:
                Immediate = {{21{Instruction[31]}},Instruction[30:21],Instruction[20]}; // I-Jalr
            5'b01000:
                Immediate = {{21{Instruction[31]}},Instruction[30:25],Instruction[11:8],Instruction[7]}; // S-type
            5'b11000:
                Immediate = {{20{Instruction[31]}},Instruction[7],Instruction[30:25],Instruction[11:8],1'b0}; // B-type
            5'b11011:
                Immediate = {{12{Instruction[31]}},Instruction[19:12],Instruction[20],Instruction[30:21],1'b0}; // Jal
            5'b00101:
                Immediate = { {13{Instruction[31]}}, Instruction[30:12] }; // auipc
            5'b00000:
                Immediate = { {21{ Instruction[31] }}, Instruction[30:20] }; //lw
            default:
                Immediate = 32'd0;
        endcase
    end

    assign mem_wen_D = memWrite;

    always@(*)begin // Control Unit
        case(Instruction[6:2])
            5'b01100: begin // R-type
                branch   = 0;
                jal      = 0;
                jalr     = 0;
                ALUsrc   = 0;
                memWrite = 0;
                if(Instruction[31:25] == 7'b0000001)
                    regWrite = !lock;
                else
                    regWrite = 1;
                memToReg = 2'b00;
                ALUop    = 2'b10;
            end
            5'b00100:begin // I-type
                branch   = 0;
                jal      = 0;
                jalr     = 0;
                ALUsrc   = 1;
                memWrite = 0;
                regWrite = 1;
                memToReg = 2'b00;
                ALUop    = 2'b10;
            end
            5'b11001:begin // I-Jalr
                branch   = 0;
                jal      = 0;
                jalr     = 1;
                ALUsrc   = 1;
                memWrite = 0;
                regWrite = 1;
                memToReg = 2'b10;
                ALUop    = 2'b00;
            end
            5'b01000:begin // S-type
                branch   = 0;
                jal      = 0;
                jalr     = 0;
                ALUsrc   = 1;
                memWrite = 1;
                regWrite = 0;
                memToReg = 2'b00;
                ALUop    = 2'b00; 
            end
            5'b11000:begin // B-type
                branch   = 1;
                jal      = 0;
                jalr     = 0;
                ALUsrc   = 0;
                memWrite = 0;
                regWrite = 0;
                memToReg = 2'b00;
                ALUop    = 2'b01;
            end
            5'b11011:begin // Jal
                branch   = 0;
                jal      = 1;
                jalr     = 0;
                ALUsrc   = 0;
                memWrite = 0;
                regWrite = 1;
                memToReg = 2'b10;
                ALUop    = 2'b00;
            end
            5'b00101:begin //auipc
                branch   = 0;
                jal      = 0;
                jalr     = 0;
                ALUsrc   = 0;
                memWrite = 0;
                regWrite = 1;
                memToReg = 2'b11;
                ALUop    = 2'b00;
            end
            5'b00000:begin //lw
                branch   = 0;
                jal      = 0;
                jalr     = 0;
                ALUsrc   = 1;
                memWrite = 0;
                regWrite = 1;
                memToReg = 2'b01;
                ALUop    = 2'b00;
            end
            default:begin
                branch   = 0;
                jal      = 0;
                jalr     = 0;
                ALUsrc   = 0;
                memWrite = 0;
                regWrite = 0;
                memToReg = 0;
                ALUop    = 0;
            end
        endcase
    end


    always@(*) begin // ALUctrl
        case(ALUop)
        2'b00:
            ALU_ctrl = 4'b0000; // add
        2'b01:
            ALU_ctrl = 4'b1000; // sub
        2'b10:begin
            if(Instruction[6:2] == 5'b00100) begin //I-type
                if(Instruction[14:12] == 3'b101)
                    ALU_ctrl = { Instruction[30], Instruction[14:12] };
                else
                    ALU_ctrl = { 1'b0, Instruction[14:12] };
            end
            else
                ALU_ctrl = { Instruction[30], Instruction[14:12] };  
        end
        default:
            ALU_ctrl = 4'b0000;
        endcase
    end


    assign  ALUin1 = rs1_data;
    assign  ALUin2 = ALUsrc ? Immediate : rs2_data;
    assign  shamt  = ALUin2[4:0];

    always@(*) begin // ALU
        zero = 0;
        case(ALU_ctrl)
        4'b0000: // add
            ALU_out = ALUin1 + ALUin2;
        4'b1000: begin // sub   
            ALU_out = ALUin1 - ALUin2;
            if( ALU_out == 32'd0 )
                zero = 1;
            else
                zero = 0;
        end
        4'b0010: // slt
            ALU_out = ( $signed( ALUin1 ) < $signed( ALUin2 ) ) ? 32'd1 : 32'd0;
        4'b0001: // SLL
            ALU_out = ALUin1 << shamt;
        4'b0101: // SRL
            ALU_out = ALUin1 >> shamt;
        4'b1101: // SRA
            ALU_out = $signed( ALUin1 ) >>> shamt;
        default:
            ALU_out = 32'd0;
        endcase
    end

    assign mem_wdata_D = rs2_data;
    assign mem_addr_D  = ALU_out;

    always@(*)begin
        case(memToReg)
            2'b00: begin
                if(do_mul)
                    rd_data = mul_out;
                else
                    rd_data = ALU_out;
            end   
            2'b01:
                rd_data = mem_rdata_D;
            2'b10:
                rd_data = PC + 32'd4;
            2'b11:
                rd_data = PC_add_imm;
            default:
                rd_data = 32'd0;
        endcase
    end

//==============  MUL  ====================//
assign do_mul = ( Instruction[31:25] == 7'b0000001 ) & ( Instruction[6:2] == 5'b01100 );
assign mul_out =  mul_out_temp[31:0];
multDiv m0( .clk( clk ), .rst_n( rst_n ), .valid( do_mul ), .ready( mul_ready ), 
            .mode( 1'b0 ), .in_A( rs1_data ), .in_B( rs2_data ), .out( mul_out_temp ) );

//==============  PC address ==============//
    assign PC_add4      = PC + 32'd4;
    assign PC_add_imm   = PC + Immediate;
    assign and_out      = zero & branch;
    assign or_out       = and_out | jal;
    assign reg1_add_imm = rs1_data + Immediate;
    assign first_mux_out = or_out ? PC_add_imm : PC_add4;
    assign PC_temp = jalr ? reg1_add_imm : first_mux_out;

    assign PC_nxt = PC_temp;
    assign lock = ( do_mul & !mul_ready ) ? 1'b1 : 1'b0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            PC <= 32'h00010000; // Do not modify this value!!!
        end
        else if(!lock) begin
            PC <= PC_nxt; 
        end
    end

endmodule

module reg_file(clk, rst_n, wen, a1, a2, aw, d, q1, q2);
   
    parameter BITS = 32;
    parameter word_depth = 32;
    parameter addr_width = 5; // 2^addr_width >= word_depth
    
    input clk, rst_n, wen; // wen: 0:read | 1:write
    input [BITS-1:0] d;
    input [addr_width-1:0] a1, a2, aw;

    output [BITS-1:0] q1, q2;

    reg [BITS-1:0] mem [0:word_depth-1];
    reg [BITS-1:0] mem_nxt [0:word_depth-1];

    integer i;

    assign q1 = mem[a1];
    assign q2 = mem[a2];

    always @(*) begin
        for (i=0; i<word_depth; i=i+1)
            mem_nxt[i] = (wen && (aw == i)) ? d : mem[i];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1) begin
                case(i)
                    32'd2: mem[i] <= 32'hbffffff0;
                    32'd3: mem[i] <= 32'h10008000;
                    default: mem[i] <= 32'h0;
                endcase
            end
        end
        else  begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1)
                mem[i] <= mem_nxt[i];
        end       
    end
endmodule

module multDiv(clk, rst_n, valid, ready, mode, in_A, in_B, out);
    // Todo: your HW3
     // Definition of ports
    input         clk, rst_n;
    input         valid, mode; // mode: 0: multu, 1: divu
    output        ready;
    input  [31:0] in_A, in_B;
    output [63:0] out;

    // Definition of states
    parameter IDLE = 2'b00;
    parameter MULT = 2'b01;
    parameter DIV  = 2'b10;
    parameter OUT  = 2'b11;

    // Todo: Wire and reg
    reg  [ 1:0] state, state_nxt;
    reg  [ 4:0] counter, counter_nxt;
    reg  [63:0] shreg, shreg_nxt;
    reg  [31:0] alu_in, alu_in_nxt;
    reg  [32:0] alu_out;

    // Todo 5: wire assignments
    assign ready = (state == OUT)? 1:0;
    assign out = shreg;
    
    // Combinational always block
    // Todo 1: State machine
    always @(*) begin
        case(state)
            IDLE: begin
                if (valid == 0)
                    state_nxt = IDLE;
                else begin
                    if (mode == 1)
                        state_nxt = DIV;
                    else
                        state_nxt = MULT;
                end
            end
            MULT: begin
                if (counter == 5'd31) begin
                    state_nxt = OUT;
                end
                else
                    state_nxt = MULT;
            end
            DIV : begin
                if (counter == 5'd31) begin
                    state_nxt = OUT;
                end
                else
                    state_nxt = DIV;
            end
            default: state_nxt = IDLE;
        endcase
    end
    // Todo 2: Counter
    always @(*) begin
        case(state)
            MULT: counter_nxt = counter + 5'd1;
            DIV: counter_nxt = counter + 5'd1;
            default: counter_nxt = 5'd0;
        endcase
    end
    
    // ALU input
    always @(*) begin
        case(state)
            IDLE: begin
                if (valid == 1) alu_in_nxt = in_B;
                else       alu_in_nxt = 0;
            end
            OUT : alu_in_nxt = 0;
            default: alu_in_nxt = alu_in;
        endcase
    end

    // Todo 3: ALU output
    always @(*) begin
        case(state)
            MULT: begin
                if (shreg[0] == 1) begin
                    alu_out = alu_in + shreg[63:32];
                end
                else begin
                    alu_out = shreg[63:32];
                end
            end
            DIV: alu_out = shreg[63:32] - alu_in;
            default: alu_out = 33'b0;
        endcase
    end
    
    // Todo 4: Shift register
    always @(*) begin
        case(state)
            IDLE: begin
                if (valid == 1) begin
                    if (mode == 1) shreg_nxt = {31'b0, in_A, 1'b0};
                    else shreg_nxt = {32'b0, in_A};
                end
                else shreg_nxt = 64'b0;
            end
            MULT: begin
                shreg_nxt = {alu_out, shreg[31:1]};
            end
            DIV: begin
                if (counter == 5'd31) begin
                    if (alu_out[31] == 1) shreg_nxt = {1'b0, shreg[62:32], shreg[30:0], 1'b0};
                    else shreg_nxt = {1'b0, alu_out[30:0], shreg[30:0], 1'b1};
                end
                else begin
                    if (alu_out[31] == 1) shreg_nxt = shreg << 1;
                    else shreg_nxt = {alu_out[30:0], shreg[31:0], 1'b1};
                end                    
            end
            default: shreg_nxt = 64'b0;
        endcase
    end

    // Todo: Sequential always block
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            counter <= 0;
        end
        else begin
            state <= state_nxt;
            counter <= counter_nxt;
            shreg <= shreg_nxt;
            alu_in <= alu_in_nxt; 
        end
    end

endmodule