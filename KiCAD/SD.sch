EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 13 22
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
L 473521001:473521001 U13
U 1 1 5CDD43F7
P 5850 3350
F 0 "U13" V 6437 3350 60  0000 C CNN
F 1 "473521001" V 6331 3350 60  0000 C CNN
F 2 "473521001:473521001" H 5650 3350 60  0001 C CNN
F 3 "" H 5650 3350 60  0001 C CNN
F 4 "47352-1001" H 5850 3350 50  0001 C CNN "MNP"
	1    5850 3350
	0    -1   -1   0   
$EndComp
Text HLabel 7450 3000 0    50   Input ~ 0
1_DATA2
Wire Wire Line
	7450 3000 7550 3000
Text HLabel 8150 3000 2    50   Input ~ 0
2_DATA3
Wire Wire Line
	8150 3000 8050 3000
Text HLabel 7450 3100 0    50   Input ~ 0
3_CMD
Wire Wire Line
	7450 3100 7550 3100
Text HLabel 8150 3100 2    50   Input ~ 0
4_VDD
Wire Wire Line
	8150 3100 8050 3100
Text HLabel 8150 3200 2    50   Input ~ 0
6_Vss
Wire Wire Line
	8150 3200 8050 3200
Text HLabel 8150 3300 2    50   Input ~ 0
8_DATA1
Wire Wire Line
	8150 3300 8050 3300
Text HLabel 7450 3400 0    50   Input ~ 0
9_DETECT
Wire Wire Line
	7450 3400 7550 3400
Wire Wire Line
	7450 3300 7550 3300
Wire Wire Line
	7450 3200 7550 3200
Text HLabel 7450 3200 0    50   Input ~ 0
5_CLK
Text HLabel 7450 3300 0    50   Input ~ 0
7_DATA0
Text HLabel 6600 3000 2    50   Input ~ 0
1_DATA2
Wire Wire Line
	6600 3000 6500 3000
Text HLabel 6600 3100 2    50   Input ~ 0
2_DATA3
Wire Wire Line
	6600 3100 6500 3100
Text HLabel 6600 3200 2    50   Input ~ 0
3_CMD
Wire Wire Line
	6600 3200 6500 3200
Text HLabel 6600 3300 2    50   Input ~ 0
4_VDD
Wire Wire Line
	6600 3300 6500 3300
Text HLabel 6600 3500 2    50   Input ~ 0
6_Vss
Wire Wire Line
	6600 3500 6500 3500
Text HLabel 6600 3700 2    50   Input ~ 0
8_DATA1
Wire Wire Line
	6600 3700 6500 3700
Text HLabel 5100 3500 0    50   Input ~ 0
9_DETECT
Wire Wire Line
	5100 3500 5200 3500
Wire Wire Line
	6600 3600 6500 3600
Wire Wire Line
	6600 3400 6500 3400
Text HLabel 6600 3400 2    50   Input ~ 0
5_CLK
Text HLabel 6600 3600 2    50   Input ~ 0
7_DATA0
$Comp
L Connector_Generic:Conn_02x05_Odd_Even J24
U 1 1 5CF51553
P 7750 2350
F 0 "J24" H 7800 2767 50  0000 C CNN
F 1 "Conn_02x05_Odd_Even" H 7800 2676 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x05_P2.54mm_Vertical" H 7750 2350 50  0001 C CNN
F 3 "~" H 7750 2350 50  0001 C CNN
	1    7750 2350
	1    0    0    -1  
$EndComp
$EndSCHEMATC
