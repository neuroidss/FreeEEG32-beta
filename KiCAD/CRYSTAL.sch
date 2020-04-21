EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 9 22
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L fxo-sm7_osc-smd7050:FXO-SM7_OSC-SMD7050_short U8
U 1 1 5CD9CFFA
P 5400 3100
F 0 "U8" H 5400 3397 60  0000 C CNN
F 1 "FXO-SM7_OSC-SMD7050_short" H 5400 3291 60  0000 C CNN
F 2 "Oscillator:Oscillator_SMD_EuroQuartz_XO91-4Pin_7.0x5.0mm_HandSoldering" H 5350 3100 60  0001 C CNN
F 3 "" H 5350 3100 60  0001 C CNN
F 4 "CB3LV-3I-8M1920" H 5400 3100 50  0001 C CNN "MNP"
F 5 "" H 5400 3100 50  0001 C CNN "Manufacturer"
	1    5400 3100
	1    0    0    -1  
$EndComp
Text HLabel 4800 3150 0    50   Input ~ 0
4_Vin
Wire Wire Line
	4800 3150 4900 3150
Text HLabel 4800 3050 0    50   Input ~ 0
1_NC
Wire Wire Line
	4800 3050 4900 3050
Text HLabel 6000 3150 2    50   Input ~ 0
3_OUT
Wire Wire Line
	6000 3150 5900 3150
Text HLabel 6000 3050 2    50   Input ~ 0
2_GND
Wire Wire Line
	6000 3050 5900 3050
Text HLabel 5450 2400 3    50   Input ~ 0
2_GND
Text HLabel 5350 2400 3    50   Input ~ 0
1_NC
Wire Wire Line
	5450 2400 5450 2300
Wire Wire Line
	5350 2400 5350 2300
Text HLabel 5550 2400 3    50   Input ~ 0
3_OUT
Wire Wire Line
	5550 2400 5550 2300
Text HLabel 5250 2400 3    50   Input ~ 0
4_Vin
Wire Wire Line
	5250 2400 5250 2300
$Comp
L Connector_Generic:Conn_01x04 J23
U 1 1 5CF546A2
P 5350 2050
F 0 "J23" V 5316 1762 50  0000 R CNN
F 1 "Conn_01x04" V 5225 1762 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 5350 2050 50  0001 C CNN
F 3 "~" H 5350 2050 50  0001 C CNN
	1    5350 2050
	0    -1   -1   0   
$EndComp
$EndSCHEMATC
