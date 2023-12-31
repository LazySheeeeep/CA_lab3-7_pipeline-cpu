module remote2local (
    //switch
    input  wire [31:0] remote_switch,
    output wire [ 7:0] local_switch,
    //led
    input  wire [15:0] local_leds,
    output wire [15:0] remote_leds,
    //reset
    input  wire        remote_reset,
    output wire        local_resetn,
    //num
    input  wire [ 7:0] local_num_data,  
    output wire [ 7:0] remote_dpy0,
    output wire [ 7:0] remote_dpy1 
);
    assign local_switch = ~remote_switch[7:0];
    assign local_resetn = ~remote_reset;
    assign remote_leds  = ~local_leds;

    SEG7_LUT segL(.oSEG1(remote_dpy0[7:0]), .iDIG(local_num_data[3:0])); 
    SEG7_LUT segH(.oSEG1(remote_dpy1[7:0]), .iDIG(local_num_data[7:4])); 
endmodule

// number to pin
module SEG7_LUT (   oSEG1,iDIG   );
input   wire[3:0]   iDIG;
output  wire[7:0]   oSEG1;
reg     [6:0]   oSEG;

// oSEG从左到右各位对应 m lt t rt lb b rb
always @(iDIG)
begin
        case(iDIG)
        4'h1: oSEG = 7'b1110110;    // ---t----
        4'h2: oSEG = 7'b0100001;    // |      |
        4'h3: oSEG = 7'b0100100;    // lt    rt
        4'h4: oSEG = 7'b0010110;    // |      |
        4'h5: oSEG = 7'b0001100;    // ---m----
        4'h6: oSEG = 7'b0001000;    // |      |
        4'h7: oSEG = 7'b1100110;    // lb    rb
        4'h8: oSEG = 7'b0000000;    // |      |
        4'h9: oSEG = 7'b0000110;    // ---b----
        4'ha: oSEG = 7'b0000010;
        4'hb: oSEG = 7'b0011000;
        4'hc: oSEG = 7'b1001001;
        4'hd: oSEG = 7'b0110000;
        4'he: oSEG = 7'b0001001;
        4'hf: oSEG = 7'b0001011;
        4'h0: oSEG = 7'b1000000;
        endcase
end

// 1代表数码管亮，0代表暗，因此最后要取反
assign oSEG1 = {~oSEG,1'b0};

endmodule