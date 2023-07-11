@echo off
REM ****************************************************************************
REM Vivado (TM) v2020.1 (64-bit)
REM
REM Filename    : simulate.bat
REM Simulator   : Xilinx Vivado Simulator
REM Description : Script for simulating the design by launching the simulator
REM
REM Generated by Vivado on Sat Nov 12 15:35:43 +0800 2022
REM SW Build 2902540 on Wed May 27 19:54:49 MDT 2020
REM
REM Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
REM
REM usage: simulate.bat
REM
REM ****************************************************************************
echo "xsim tb__behav -key {Behavioral:sim_1:Functional:tb_} -tclbatch tb_.tcl -view E:/Semesters/FA22/VE370/Labs/Lab4/Lab4_vvd/tb__behav.wcfg -log simulate.log"
call xsim  tb__behav -key {Behavioral:sim_1:Functional:tb_} -tclbatch tb_.tcl -view E:/Semesters/FA22/VE370/Labs/Lab4/Lab4_vvd/tb__behav.wcfg -log simulate.log
if "%errorlevel%"=="0" goto SUCCESS
if "%errorlevel%"=="1" goto END
:END
exit 1
:SUCCESS
exit 0