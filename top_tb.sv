`timescale 1ns / 1ps
module top_tb(
output reg clk,
output reg rst_n,
output reg[3:0] r4_car_cnt,
input r1_yellow ,
r1_red ,
r1_green ,
r2_yellow ,
r2_red ,
r2_green ,
r3_yellow ,
r3_red ,
r3_green ,
r4_yellow ,
r4_red ,
r4_green ,
r4_green_l
);

top road_scenario_inst (.i_clk (clk ) ,
.i_rst_n (rst_n ) ,
.i_r4_car_cnt(r4_car_cnt) ,
.o_r1_yellow (r1_yellow ) ,
.o_r1_red (r1_red ) ,
.o_r1_green (r1_green ) ,
.o_r2_yellow (r2_yellow ) ,
.o_r2_red (r2_red ) ,
.o_r2_green (r2_green ) ,
.o_r3_yellow (r3_yellow ) ,
.o_r3_red (r3_red ) ,
.o_r3_green (r3_green ) ,
.o_r4_yellow (r4_yellow ) ,
.o_r4_red (r4_red ) ,
.o_r4_green (r4_green ) ,
.o_r4_green_l(r4_green_l));

initial begin
clk <= 0;
forever #10ns clk <= ~clk;
end
initial begin
rst_n <= 0;
repeat(1) @(posedge clk);
rst_n <= 1;
end
initial begin 
 r4_car_cnt = 0;
 #520;
 r4_car_cnt = 1;
 end


initial begin
#1000ns;
$finish();
end

endmodule: top_tb