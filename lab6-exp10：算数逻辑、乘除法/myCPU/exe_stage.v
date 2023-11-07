`include "mycpu.h"

module exe_stage(
    input                          clk           ,
    input                          reset         ,
    //allowin
    input                          ms_allowin    ,
    output                         es_allowin    ,
    //from ds
    input                          ds_to_es_valid,
    input  [`DS_TO_ES_BUS_WD -1:0] ds_to_es_bus  ,
    //to ms
    output                         es_to_ms_valid,
    output [`ES_TO_MS_BUS_WD -1:0] es_to_ms_bus  ,
    // data sram interface
    output        data_sram_en   ,
    output [ 3:0] data_sram_wen  ,
    output [31:0] data_sram_addr ,
    output [31:0] data_sram_wdata,
    //forward to id
    output [ 4:0] es_fwd_dest    ,
    output [31:0] es_fwd_res     ,
    output        es_ld
);

reg         es_valid      ;
wire        es_ready_go   ;

reg  [`DS_TO_ES_BUS_WD -1:0] ds_to_es_bus_r;
wire        es_op_need_div;
wire        es_op_need_mul;
wire        es_md_signed;
wire        es_md_high;
wire [11:0] es_alu_op     ;
wire        es_src1_is_pc ;
wire        es_src2_is_imm;
wire        es_gr_we      ;
wire        es_mem_we     ;
wire [ 4:0] es_dest       ;
wire [31:0] es_imm        ;
wire [31:0] es_rj_value   ;
wire [31:0] es_rkd_value  ;
wire [31:0] es_pc         ;

wire        es_res_from_mem;

assign {es_op_need_div    ,  //153:153
        es_op_need_mul    ,  //152:152
        es_md_signed      ,  //151:151
        es_md_high        ,  //150:150
        es_alu_op      ,  //149:138
        es_res_from_mem,  //137:137
        es_src1_is_pc  ,  //136:136
        es_src2_is_imm ,  //135:135
        es_gr_we       ,  //134:134
        es_mem_we      ,  //133:133
        es_dest        ,  //132:128
        es_imm         ,  //127:96
        es_rj_value    ,  //95 :64
        es_rkd_value   ,  //63 :32
        es_pc             //31 :0
       } = ds_to_es_bus_r;

wire [31:0] es_alu_src1    ;
wire [31:0] es_alu_src2    ;
wire [31:0] es_alu_result  ;
wire [63:0] es_mul_result  ;
wire [63:0] es_divs_result ;
wire [63:0] es_divu_result ;
wire [31:0] es_final_result;


assign es_mul_result = $signed({es_md_signed & es_alu_src1[31], es_alu_src1})
                     * $signed({es_md_signed & es_alu_src2[31], es_alu_src2});
//assign es_mul_result   = es_md_signed ? $signed(es_rj_value) * $signed(es_rkd_value)
//                                      : es_rj_value * es_rkd_value;

assign es_final_result = es_op_need_div ? 
                                        es_md_signed ? (es_md_high ? es_divs_result[63:32] : es_divs_result[31:0])
                                                     : (es_md_high ? es_divu_result[63:32] : es_divu_result[31:0])
                       : es_op_need_mul ? (es_md_high ? es_mul_result[63:32] : es_mul_result[31:0])
                       : es_alu_result;

//assign es_res_from_mem = es_load_op;
assign es_to_ms_bus = {es_res_from_mem,  //70:70
                       es_gr_we       ,  //69:69
                       es_dest        ,  //68:64
                       es_final_result  ,//63:32
                       es_pc             //31:0
                      };

assign es_fwd_dest = es_dest & {5{es_valid & es_gr_we}};
assign es_fwd_res  = es_final_result;
assign es_ld       = es_res_from_mem;


wire        divs1_ready  ;
wire        divs2_ready  ;
wire        divu1_ready  ;
wire        divu2_ready  ;
wire        divs_complete;
wire        divu_complete;
wire        divs12_valid ;
wire        divu12_valid ;
reg         divs12_validr;
reg         divu12_validr;

assign divs12_valid = divs12_validr;
assign divu12_valid = divu12_validr;

assign es_ready_go    = !es_op_need_div || (divs_complete || divu_complete);
assign es_allowin     = !es_valid || es_ready_go && ms_allowin;
assign es_to_ms_valid =  es_valid && es_ready_go;
always @(posedge clk) begin
    if (reset) begin
        es_valid <= 1'b0;
    end
    else if (es_allowin) begin
        es_valid <= ds_to_es_valid;
    end
    if (ds_to_es_valid && es_allowin) begin
        ds_to_es_bus_r <= ds_to_es_bus;
    end
end
//div
reg div_en;
always @(posedge clk) begin
    if (es_op_need_div) begin
        div_en <= 1'b0;
    end else begin
        div_en <= 1'b1;
    end
end

always @(posedge clk) begin
    if (reset) begin
        divs12_validr <= 0;
        divu12_validr <= 0;
    end else if (es_op_need_div && es_md_signed && div_en) begin
        divs12_validr <= 1;
    end else if (es_op_need_div &&!es_md_signed && div_en) begin
        divu12_validr <= 1;
    end else if (divs1_ready && divs2_ready) begin
        divs12_validr <= 0;
    end else if (divu1_ready && divu2_ready) begin
        divu12_validr <= 0;
    end
end


assign es_alu_src1 = es_src1_is_pc  ? es_pc[31:0] : es_rj_value;
assign es_alu_src2 = es_src2_is_imm ? es_imm : es_rkd_value;

alu u_alu(
    .alu_op     (es_alu_op    ),
    .alu_src1   (es_alu_src1  ),
    .alu_src2   (es_alu_src2  ),
    .alu_result (es_alu_result)
    );

assign data_sram_en    = (es_res_from_mem || es_mem_we) && es_valid;
assign data_sram_wen   = es_mem_we ? 4'hf : 4'h0;
assign data_sram_addr  = es_final_result;
assign data_sram_wdata = es_rkd_value;

divider_signed div_s(
    .aclk                   (clk),
    .s_axis_divisor_tdata   (es_alu_src2),
    .s_axis_divisor_tready  (divs2_ready),
    .s_axis_divisor_tvalid  (divs12_valid),
    .s_axis_dividend_tdata  (es_alu_src1),
    .s_axis_dividend_tready (divs1_ready),
    .s_axis_dividend_tvalid (divs12_valid),
    .m_axis_dout_tdata      (es_divs_result),
    .m_axis_dout_tvalid     (divs_complete)
    );

divider_unsigned div_u(
    .aclk                   (clk),
    .s_axis_divisor_tdata   (es_alu_src2),
    .s_axis_divisor_tready  (divu2_ready),
    .s_axis_divisor_tvalid  (divu12_valid),
    .s_axis_dividend_tdata  (es_alu_src1),
    .s_axis_dividend_tready (divu1_ready),
    .s_axis_dividend_tvalid (divu12_valid),
    .m_axis_dout_tdata      (es_divu_result),
    .m_axis_dout_tvalid     (divu_complete)
    );

endmodule
