`timescale 1ns / 1ps
module top #
  (parameter FAST_GREEN_SLOW_RED_CLOCKS = 16'd5,
   parameter FAST_RED_SLOW_GREEN_CLOCKS = 16'd10,
   parameter YELLOW_CLOCKS              =   16'd2,
   parameter LEFT_CLOCKS_PER_CAR        =   16'd5,
   parameter TCQ                        =       1)

  (//Clocks and resets input
   input         i_clk        ,
   input         i_rst_n      , 

   //road 4 left car count input
   input  [3:0]  i_r4_car_cnt , 

   //road 1 lights out
   output reg  o_r1_yellow  ,
   output reg  o_r1_red     ,
   output reg  o_r1_green   ,

   //road 2 lights out
   output reg  o_r2_yellow  ,
   output reg  o_r2_red     ,
   output reg  o_r2_green   ,

   //road 3 lights out
   output reg  o_r3_yellow  ,
   output reg  o_r3_red     ,
   output reg  o_r3_green   ,

   //road 4 lights out
   output reg  o_r4_yellow  ,
   output reg  o_r4_red     ,
   output reg  o_r4_green   ,
   output reg  o_r4_green_l);

    typedef enum reg [3:0] {RESET                 , 
                              RD1Y_RD2R_RD3Y_RD4R   , 
                              RD1G_RD2R_RD3G_RD4R   , 
                              RD1Y_RD2Y_RD3Y_RD4Y   , 
                              RD1R_RD2G_RD3R_RD4G   , 
                              RD1R_RD2Y_RD3R_RD4G   , 
                              RD1R_RD2R_RD3R_RD4GG2 ,
                              RD1Y_RD2Y_RD3Y_RD4Y_D } traffic_states; 


   traffic_states curr_state, nxt_state;

   reg[ 3:0] r4_left_cars  ;
   reg[ 3:0] r4_left_cars_d;

   reg[15:0] curr_cnt      ;
   reg[15:0] curr_cnt_d    ;
   reg       trig_cnt      ;  
 
   //current state assignment
   always_ff @ (posedge i_clk) begin
      if(~i_rst_n) begin
         curr_state <= #TCQ RESET ;
      end else begin
         curr_state <= #TCQ nxt_state  ;
      end
   end

   //next state reg
   always_comb begin
      nxt_state = curr_state; //To avoid latch on nxt state signal
      case (curr_state) 
         RESET                 :  nxt_state = RD1Y_RD2R_RD3Y_RD4R                    ;
         RD1Y_RD2R_RD3Y_RD4R   :  nxt_state = trig_cnt       ? RD1G_RD2R_RD3G_RD4R   :
                                                               RD1Y_RD2R_RD3Y_RD4R   ;
         RD1G_RD2R_RD3G_RD4R   :  nxt_state = trig_cnt       ? RD1Y_RD2Y_RD3Y_RD4Y   :
                                                               RD1G_RD2R_RD3G_RD4R   ;
         RD1Y_RD2Y_RD3Y_RD4Y   :  nxt_state = trig_cnt       ? RD1R_RD2G_RD3R_RD4G   :
                                                               RD1Y_RD2Y_RD3Y_RD4Y   ;
         RD1R_RD2G_RD3R_RD4G   :  nxt_state = trig_cnt       ? 
                                              (|r4_left_cars ? RD1R_RD2Y_RD3R_RD4G   :
                                                               RD1Y_RD2Y_RD3Y_RD4Y_D):
                                                               RD1R_RD2G_RD3R_RD4G   ;
         RD1R_RD2Y_RD3R_RD4G   :  nxt_state = trig_cnt       ? RD1R_RD2R_RD3R_RD4GG2 :
                                                               RD1R_RD2Y_RD3R_RD4G   ;
         RD1R_RD2R_RD3R_RD4GG2 :  nxt_state = trig_cnt       ? RD1Y_RD2Y_RD3Y_RD4Y_D :
                                                               RD1R_RD2R_RD3R_RD4GG2 ;
         RD1Y_RD2Y_RD3Y_RD4Y_D :  nxt_state = trig_cnt       ? RD1G_RD2R_RD3G_RD4R   :
                                                               RD1Y_RD2Y_RD3Y_RD4Y_D ;
         default               :  nxt_state = RESET                                  ;
      endcase
   end

   //output signal reg
   always_comb begin
      o_r1_yellow  = 1'b0;
      o_r1_red     = 1'b1;
      o_r1_green   = 1'b0;
      o_r2_yellow  = 1'b0;
      o_r2_red     = 1'b1;
      o_r2_green   = 1'b0;
      o_r3_yellow  = 1'b0;
      o_r3_red     = 1'b1;
      o_r3_green   = 1'b0;
      o_r4_yellow  = 1'b0;
      o_r4_red     = 1'b1;
      o_r4_green   = 1'b0;
      o_r4_green_l = 1'b0;
      case (curr_state)
         RESET                 :   begin
                                      o_r1_yellow  = 1'b0;
                                      o_r1_red     = 1'b1;
                                      o_r1_green   = 1'b0;
                                      o_r2_yellow  = 1'b0;
                                      o_r2_red     = 1'b1;
                                      o_r2_green   = 1'b0;
                                      o_r3_yellow  = 1'b0;
                                      o_r3_red     = 1'b1;
                                      o_r3_green   = 1'b0;
                                      o_r4_yellow  = 1'b0;
                                      o_r4_red     = 1'b1;
                                      o_r4_green   = 1'b0;
                                      o_r4_green_l = 1'b0;
                                   end
         RD1Y_RD2R_RD3Y_RD4R   :   begin
                                      o_r1_yellow  = 1'b1;
                                      o_r1_red     = 1'b0;
                                      o_r1_green   = 1'b0;
                                      o_r2_yellow  = 1'b0;
                                      o_r2_red     = 1'b1;
                                      o_r2_green   = 1'b0;
                                      o_r3_yellow  = 1'b1;
                                      o_r3_red     = 1'b0;
                                      o_r3_green   = 1'b0;
                                      o_r4_yellow  = 1'b0;
                                      o_r4_red     = 1'b1;
                                      o_r4_green   = 1'b0;
                                      o_r4_green_l = 1'b0;
                                   end
         RD1G_RD2R_RD3G_RD4R   :   begin
                                      o_r1_yellow  = 1'b0;
                                      o_r1_red     = 1'b0;
                                      o_r1_green   = 1'b1;
                                      o_r2_yellow  = 1'b0;
                                      o_r2_red     = 1'b1;
                                      o_r2_green   = 1'b0;
                                      o_r3_yellow  = 1'b0;
                                      o_r3_red     = 1'b0;
                                      o_r3_green   = 1'b1;
                                      o_r4_yellow  = 1'b0;
                                      o_r4_red     = 1'b1;
                                      o_r4_green   = 1'b0;
                                      o_r4_green_l = 1'b0;
                                   end
         RD1Y_RD2Y_RD3Y_RD4Y,
         RD1Y_RD2Y_RD3Y_RD4Y_D :   begin
                                      o_r1_yellow  = 1'b1;
                                      o_r1_red     = 1'b0;
                                      o_r1_green   = 1'b0;
                                      o_r2_yellow  = 1'b1;
                                      o_r2_red     = 1'b0;
                                      o_r2_green   = 1'b0;
                                      o_r3_yellow  = 1'b1;
                                      o_r3_red     = 1'b0;
                                      o_r3_green   = 1'b0;
                                      o_r4_yellow  = 1'b1;
                                      o_r4_red     = 1'b0;
                                      o_r4_green   = 1'b0;
                                      o_r4_green_l = 1'b0;
                                   end
         RD1R_RD2G_RD3R_RD4G   :   begin
                                      o_r1_yellow  = 1'b0;
                                      o_r1_red     = 1'b1;
                                      o_r1_green   = 1'b0;
                                      o_r2_yellow  = 1'b0;
                                      o_r2_red     = 1'b0;
                                      o_r2_green   = 1'b1;
                                      o_r3_yellow  = 1'b0;
                                      o_r3_red     = 1'b1;
                                      o_r3_green   = 1'b0;
                                      o_r4_yellow  = 1'b0;
                                      o_r4_red     = 1'b0;
                                      o_r4_green   = 1'b1;
                                      o_r4_green_l = 1'b0;
                                   end
         RD1R_RD2Y_RD3R_RD4G   :   begin
                                      o_r1_yellow  = 1'b0;
                                      o_r1_red     = 1'b1;
                                      o_r1_green   = 1'b0;
                                      o_r2_yellow  = 1'b1;
                                      o_r2_red     = 1'b0;
                                      o_r2_green   = 1'b0;
                                      o_r3_yellow  = 1'b0;
                                      o_r3_red     = 1'b1;
                                      o_r3_green   = 1'b0;
                                      o_r4_yellow  = 1'b0;
                                      o_r4_red     = 1'b0;
                                      o_r4_green   = 1'b1;
                                      o_r4_green_l = 1'b0;
                                   end
         RD1R_RD2R_RD3R_RD4GG2 :   begin
                                      o_r1_yellow  = 1'b0;
                                      o_r1_red     = 1'b1;
                                      o_r1_green   = 1'b0;
                                      o_r2_yellow  = 1'b0;
                                      o_r2_red     = 1'b1;
                                      o_r2_green   = 1'b0;
                                      o_r3_yellow  = 1'b0;
                                      o_r3_red     = 1'b1;
                                      o_r3_green   = 1'b0;
                                      o_r4_yellow  = 1'b0;
                                      o_r4_red     = 1'b0;
                                      o_r4_green   = 1'b1;
                                      o_r4_green_l = 1'b1;
                                   end
         default               :   begin
                                      o_r1_yellow  = 1'b0;
                                      o_r1_red     = 1'b1;
                                      o_r1_green   = 1'b0;
                                      o_r2_yellow  = 1'b0;
                                      o_r2_red     = 1'b1;
                                      o_r2_green   = 1'b0;
                                      o_r3_yellow  = 1'b0;
                                      o_r3_red     = 1'b1;
                                      o_r3_green   = 1'b0;
                                      o_r4_yellow  = 1'b0;
                                      o_r4_red     = 1'b1;
                                      o_r4_green   = 1'b0;
                                      o_r4_green_l = 1'b0;
                                   end
      endcase
   end


  //FSM supporting reg
  assign r4_left_cars          =  curr_state == RD1Y_RD2Y_RD3Y_RD4Y   ? i_r4_car_cnt            : 
                                 (curr_state == RD1R_RD2R_RD3R_RD4GG2 ? (r4_left_cars_d - 4'h1) : 
                                                                         r4_left_cars_d        );
  assign trig_cnt              = (curr_cnt_d == 0)                    ? 1'b1 : 1'b0;

   //counters 
   always_comb begin
      if(trig_cnt) begin
         case (nxt_state) 
            RD1G_RD2R_RD3G_RD4R   :  curr_cnt = FAST_GREEN_SLOW_RED_CLOCKS       - 16'h1 ;
            RD1R_RD2G_RD3R_RD4G   :  curr_cnt = FAST_RED_SLOW_GREEN_CLOCKS       - 16'h1 ;
            RD1R_RD2Y_RD3R_RD4G   :  curr_cnt = YELLOW_CLOCKS                    - 16'h1 ;
            RD1R_RD2R_RD3R_RD4GG2 :  curr_cnt = r4_left_cars*LEFT_CLOCKS_PER_CAR - 16'h1 ;
            RD1Y_RD2R_RD3Y_RD4R   ,
            RD1Y_RD2Y_RD3Y_RD4Y   ,
            RD1Y_RD2Y_RD3Y_RD4Y_D :  curr_cnt = YELLOW_CLOCKS                    - 16'h1 ;
            default               :  curr_cnt = 16'h0                            - 16'h1 ;
         endcase
      end else begin
         curr_cnt = curr_cnt_d - 16'h1;
      end
   end

   //Flop the signals
   always_ff @ (posedge i_clk) begin
      if(~i_rst_n) begin
         curr_cnt_d     <= #TCQ 16'h0 ;
         r4_left_cars_d <= #TCQ  4'h0 ;
      end else begin
         curr_cnt_d     <= #TCQ curr_cnt      ;
         r4_left_cars_d <= #TCQ r4_left_cars  ;
      end
   end

endmodule: top

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