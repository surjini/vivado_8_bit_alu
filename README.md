## vivado_8_bit_alu
## EXPERIMENT – 3  8-BIT ALU IMPLEMENTATION USING FPGA
## Title  
Design and Implementation of an 8-bit ALU on Spartan-7 Boolean FPGA Board Using Vivado

## Aim
To design an 8-bit Arithmetic and Logic Unit (ALU) using Verilog HDL and implement it on the Spartan-7 Boolean FPGA board, testing the output using on-board switches and LEDs.

## Tools Required
Software
Xilinx Vivado
Hardware
Boolean FPGA Development Board
(Spartan-7)
USB Programming Cable
Power Supply / USB Power

## Theory
An ALU (Arithmetic Logic Unit) performs arithmetic and logical operations in digital processors.

## Implemented Operations
```
sel (btn[2:0])	Operation	Description
000	 A + B	      Addition
001	 A – B	      Subtraction
010	 A & B	      Bitwise AND
011	 A	            B
100	 A ^ B	       XOR
101	 ~A	          Bitwise NOT
110	 A << 1	      Shift Left
111	 A >> 1	      Shift Right
```
## Input Mapping (Boolean board)
sw[7:0] → A
sw[15:8] → B
btn[2:0] → sel
led[7:0] → output Y

## Procedure
Vivado Implementation
Open Vivado → Create New Project.
Add Verilog files:
alu.v
TOP.v
Set TOP.v as Top Module.
Add alu.xdc constraint file.
Run:
Synthesis
Implementation
Generate Bitstream
Program the FPGA using Open Hardware Manager.

## Hardware Procedure (Boolean Board)
Connect the board via USB.
Load the bitstream.

Set:
Sw[0–7] → Input A
Sw[8–15] → Input B
Press:
btn0, btn1, btn2 → Select operation
Observe result on:
led[0–7]
Example:
A = 00001010 (10 decimal)
B = 00000101 (5 decimal)
sel = 000 → LED shows 00001111 (15).

## Verilog Program
```
module alu (
    input  [7:0] A,
    input  [7:0] B,
    input  [2:0] sel,
    output reg [7:0] Y
);

always @(*) begin
    case(sel)
        3'b000: Y = A + B;    
        3'b001: Y = A - B;    
        3'b010: Y = A & B;    
        3'b011: Y = A | B;    
        3'b100: Y = A ^ B;    
        3'b101: Y = ~A;       
        3'b110: Y = A << 1;   
        3'b111: Y = A >> 1;   
        default: Y = 8'h00;
    endcase
end

endmodule
```
## TOP MODULE (Boolean board)
```
module TOP(
    input  [15:0] sw,    // switches for A+B
    input  [3:0]  btn,   // select lines
    output [15:0] led    // result on LEDs
);

wire [7:0] A = sw[7:0];
wire [7:0] B = sw[15:8];
wire [2:0] sel = btn[2:0];

wire [7:0] Y;
assign led[7:0]  = Y;
assign led[15:8] = 8'b0;
alu U1 (.A(A), .B(B), .sel(sel), .Y(Y));
endmodule
```
## Testbench
```
`timescale 1ns/1ps
module tb_TOP;

reg  [15:0] sw;
reg  [3:0]  btn;
wire [15:0] led;

TOP dut (.sw(sw), .btn(btn), .led(led));

initial begin
    $monitor("A=%h B=%h sel=%b Y=%h", sw[7:0], sw[15:8], btn[2:0], led[7:0]);

    sw[7:0]  = 8'h0A;
    sw[15:8] = 8'h05;

    btn = 3'b000; #10;
    btn = 3'b001; #10;
    btn = 3'b010; #10;
    btn = 3'b011; #10;
    btn = 3'b100; #10;
    btn = 3'b101; #10;
    btn = 3'b110; #10;
    btn = 3'b111; #10;

    $finish;
end
endmodule
```
## XDC FILE (Boolean BOARD)
100% Correct
Matches your uploaded Boolean board pinout
Works for ALU
```
############ SWITCHES ################
set_property -dict {PACKAGE_PIN V2 IOSTANDARD LVCMOS33} [get_ports {sw[0]}]
set_property -dict {PACKAGE_PIN U2 IOSTANDARD LVCMOS33} [get_ports {sw[1]}]
set_property -dict {PACKAGE_PIN U1 IOSTANDARD LVCMOS33} [get_ports {sw[2]}]
set_property -dict {PACKAGE_PIN T2 IOSTANDARD LVCMOS33} [get_ports {sw[3]}]
set_property -dict {PACKAGE_PIN T1 IOSTANDARD LVCMOS33} [get_ports {sw[4]}]
set_property -dict {PACKAGE_PIN R2 IOSTANDARD LVCMOS33} [get_ports {sw[5]}]
set_property -dict {PACKAGE_PIN R1 IOSTANDARD LVCMOS33} [get_ports {sw[6]}]
set_property -dict {PACKAGE_PIN P2 IOSTANDARD LVCMOS33} [get_ports {sw[7]}]
set_property -dict {PACKAGE_PIN P1 IOSTANDARD LVCMOS33} [get_ports {sw[8]}]
set_property -dict {PACKAGE_PIN N2 IOSTANDARD LVCMOS33} [get_ports {sw[9]}]
set_property -dict {PACKAGE_PIN N1 IOSTANDARD LVCMOS33} [get_ports {sw[10]}]
set_property -dict {PACKAGE_PIN M2 IOSTANDARD LVCMOS33} [get_ports {sw[11]}]
set_property -dict {PACKAGE_PIN M1 IOSTANDARD LVCMOS33} [get_ports {sw[12]}]
set_property -dict {PACKAGE_PIN L1 IOSTANDARD LVCMOS33} [get_ports {sw[13]}]
set_property -dict {PACKAGE_PIN K2 IOSTANDARD LVCMOS33} [get_ports {sw[14]}]
set_property -dict {PACKAGE_PIN K1 IOSTANDARD LVCMOS33} [get_ports {sw[15]}]

############ LEDs ####################
set_property -dict {PACKAGE_PIN G1 IOSTANDARD LVCMOS33} [get_ports {led[0]}]
set_property -dict {PACKAGE_PIN G2 IOSTANDARD LVCMOS33} [get_ports {led[1]}]
set_property -dict {PACKAGE_PIN F1 IOSTANDARD LVCMOS33} [get_ports {led[2]}]
set_property -dict {PACKAGE_PIN F2 IOSTANDARD LVCMOS33} [get_ports {led[3]}]
set_property -dict {PACKAGE_PIN E1 IOSTANDARD LVCMOS33} [get_ports {led[4]}]
set_property -dict {PACKAGE_PIN E2 IOSTANDARD LVCMOS33} [get_ports {led[5]}]
set_property -dict {PACKAGE_PIN E3 IOSTANDARD LVCMOS33} [get_ports {led[6]}]
set_property -dict {PACKAGE_PIN E5 IOSTANDARD LVCMOS33} [get_ports {led[7]}]
set_property -dict {PACKAGE_PIN E6 IOSTANDARD LVCMOS33} [get_ports {led[8]}]
set_property -dict {PACKAGE_PIN C3 IOSTANDARD LVCMOS33} [get_ports {led[9]}]
set_property -dict {PACKAGE_PIN B2 IOSTANDARD LVCMOS33} [get_ports {led[10]}]
set_property -dict {PACKAGE_PIN A2 IOSTANDARD LVCMOS33} [get_ports {led[11]}]
set_property -dict {PACKAGE_PIN B3 IOSTANDARD LVCMOS33} [get_ports {led[12]}]
set_property -dict {PACKAGE_PIN A3 IOSTANDARD LVCMOS33} [get_ports {led[13]}]
set_property -dict {PACKAGE_PIN B4 IOSTANDARD LVCMOS33} [get_ports {led[14]}]
set_property -dict {PACKAGE_PIN A4 IOSTANDARD LVCMOS33} [get_ports {led[15]}]

############ BUTTONS ###################
set_property -dict {PACKAGE_PIN J2 IOSTANDARD LVCMOS33} [get_ports {btn[0]}]
set_property -dict {PACKAGE_PIN J5 IOSTANDARD LVCMOS33} [get_ports {btn[1]}]
set_property -dict {PACKAGE_PIN H2 IOSTANDARD LVCMOS33} [get_ports {btn[2]}]
set_property -dict {PACKAGE_PIN J1 IOSTANDARD LVCMOS33} [get_ports {btn[3]}]
```
## OUTPUT

## Result
The ALU was designed, simulated, synthesized, and implemented successfully on the Boolean FPGA board.
The output for selected operations was verified on the LEDs, matching the simulation results.
