`include "mycpu.h"

module mem_stage(
    input                          clk           ,
    input                          reset         ,
    //allowin
    input                          ws_allowin    ,
    output                         ms_allowin    ,
    //from es
    input                          es_to_ms_valid,
    input  [`ES_TO_MS_BUS_WD -1:0] es_to_ms_bus  ,
    //to ws
    output                         ms_to_ws_valid,
    output [`MS_TO_WS_BUS_WD -1:0] ms_to_ws_bus  ,
    //from data-sram
    input  [31                 :0] data_sram_rdata,

    output [4:0] ms_fwd_dest,
    output [31:0] ms_fwd_res
);

reg         ms_valid;
wire        ms_ready_go;

reg [`ES_TO_MS_BUS_WD -1:0] es_to_ms_bus_r;
wire        ms_ld_signed;
wire        ms_ls_width_h;
wire        ms_ls_width_b;
wire        ms_res_from_mem;
wire        ms_gr_we;
wire [ 4:0] ms_dest;
wire [31:0] ms_alu_result;
wire [31:0] ms_pc;
assign {ms_ld_signed   ,  //73:73
        ms_ls_width_h  ,  //72:72
        ms_ls_width_b  ,  //71:71
        ms_res_from_mem,  //70:70
        ms_gr_we       ,  //69:69
        ms_dest        ,  //68:64
        ms_alu_result  ,  //63:32
        ms_pc             //31:0
       } = es_to_ms_bus_r;

wire [31:0] mem_result;
wire [31:0] ms_final_result;

assign ms_to_ws_bus = {ms_gr_we       ,  //69:69
                       ms_dest        ,  //68:64
                       ms_final_result,  //63:32
                       ms_pc             //31:0
                      };

assign ms_fwd_dest = ms_dest & {5{ms_valid & ms_gr_we}};
assign ms_fwd_res  = ms_final_result;

assign ms_ready_go    = 1'b1;
assign ms_allowin     = !ms_valid || ms_ready_go && ws_allowin;
assign ms_to_ws_valid = ms_valid && ms_ready_go;
always @(posedge clk) begin
    if (reset) begin
        ms_valid <= 1'b0;
    end
    else if (ms_allowin) begin
        ms_valid <= es_to_ms_valid;
    end
    if (es_to_ms_valid && ms_allowin) begin
        es_to_ms_bus_r  <= es_to_ms_bus;
    end
end

wire [1:0] offset;
assign offset = ms_alu_result[1:0];

wire [31:0] mem_shifted_data;
assign mem_shifted_data = data_sram_rdata >> ({3'b0, offset} << 3);

assign mem_result = ms_ls_width_h ?
                                    ms_ld_signed ? {{16{mem_shifted_data[15]}}, mem_shifted_data[15:0]}
                                                 : {16'b0                     , mem_shifted_data[15:0]}
                  : ms_ls_width_b ?                  
                                    ms_ld_signed ? {{24{mem_shifted_data[ 7]}}, mem_shifted_data[ 7:0]}
                                                 : {24'b0                     , mem_shifted_data[ 7:0]}
                  : data_sram_rdata;

assign ms_final_result = ms_res_from_mem ? mem_result
                                         : ms_alu_result;

endmodule
