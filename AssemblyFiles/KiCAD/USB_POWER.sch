EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 17 22
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
L Connector_Generic:Conn_02x03_Odd_Even J38
U 1 1 5CF566D2
P 4600 3400
AR Path="/5CF566B4/5CF566D2" Ref="J38"  Part="1" 
AR Path="/8F1DEE33/5CF566D2" Ref="J52"  Part="1" 
F 0 "J38" H 4650 3717 50  0000 C CNN
F 1 "Conn_02x03_Odd_Even" H 4650 3626 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x03_P2.54mm_Vertical" H 4600 3400 50  0001 C CNN
F 3 "~" H 4600 3400 50  0001 C CNN
	1    4600 3400
	1    0    0    -1  
$EndComp
$Comp
L Connector:USB_B_Micro J37
U 1 1 5CF56751
P 3150 4150
AR Path="/5CF566B4/5CF56751" Ref="J37"  Part="1" 
AR Path="/8F1DEE33/5CF56751" Ref="J51"  Part="1" 
F 0 "J37" H 3205 4617 50  0000 C CNN
F 1 "USB_B_Micro" H 3205 4526 50  0000 C CNN
F 2 "Connector_USB:USB_Micro-B_Molex-105017-0001" H 3300 4100 50  0001 C CNN
F 3 "~" H 3300 4100 50  0001 C CNN
F 4 "105017-0001" H 3150 4150 50  0001 C CNN "MNP"
F 5 "Molex" H 3150 4150 50  0001 C CNN "Manufacturer"
	1    3150 4150
	1    0    0    -1  
$EndComp
Text HLabel 3550 3950 2    50   Input ~ 0
1_VBUS
Wire Wire Line
	3550 3950 3450 3950
Text HLabel 4300 3650 0    50   Input ~ 0
1_VBUS
Wire Wire Line
	4300 3650 4400 3650
Text HLabel 3550 4150 2    50   Input ~ 0
3_D+
Wire Wire Line
	3550 4150 3450 4150
Text HLabel 3550 4250 2    50   Input ~ 0
2_D-
Wire Wire Line
	3550 4250 3450 4250
Text HLabel 3550 4350 2    50   Input ~ 0
4_ID
Wire Wire Line
	3550 4350 3450 4350
Text HLabel 5000 3650 2    50   Input ~ 0
2_D-
Wire Wire Line
	5000 3650 4900 3650
Text HLabel 5000 3750 2    50   Input ~ 0
4_ID
Wire Wire Line
	5000 3750 4900 3750
Text HLabel 5000 3850 2    50   Input ~ 0
6_Shield
Wire Wire Line
	5000 3850 4900 3850
Text HLabel 4300 3750 0    50   Input ~ 0
3_D+
Wire Wire Line
	4300 3750 4400 3750
Text HLabel 4300 3850 0    50   Input ~ 0
5_GND
Wire Wire Line
	4300 3850 4400 3850
Text HLabel 3050 4650 3    50   Input ~ 0
6_Shield
Wire Wire Line
	3050 4650 3050 4550
Text HLabel 3150 4650 3    50   Input ~ 0
5_GND
Wire Wire Line
	3150 4650 3150 4550
$Comp
L Connector_Generic:Conn_01x02 J63
U 1 1 5EA40A02
P 4550 4450
AR Path="/8F1DEE33/5EA40A02" Ref="J63"  Part="1" 
AR Path="/5CF566B4/5EA40A02" Ref="J61"  Part="1" 
F 0 "J61" H 4630 4442 50  0000 L CNN
F 1 "Conn_01x02" H 4630 4351 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x01_P2.54mm_Horizontal" H 4550 4450 50  0001 C CNN
F 3 "~" H 4550 4450 50  0001 C CNN
F 4 "PREC001DBAN-M71RC" H 4550 4450 50  0001 C CNN "MNP"
	1    4550 4450
	1    0    0    -1  
$EndComp
Text HLabel 4250 4450 0    50   Input ~ 0
1_VBUS
Wire Wire Line
	4250 4450 4350 4450
Text HLabel 4200 5200 0    50   Input ~ 0
2_D-
Wire Wire Line
	4250 4800 4350 4800
Text HLabel 4200 5100 0    50   Input ~ 0
3_D+
Wire Wire Line
	4250 4700 4350 4700
Text HLabel 4250 4550 0    50   Input ~ 0
5_GND
Wire Wire Line
	4250 4550 4350 4550
$Comp
L Connector_Generic:Conn_01x02 J64
U 1 1 5EA40A03
P 4550 4700
AR Path="/8F1DEE33/5EA40A03" Ref="J64"  Part="1" 
AR Path="/5CF566B4/5EA40A03" Ref="J62"  Part="1" 
F 0 "J62" H 4630 4692 50  0000 L CNN
F 1 "Conn_01x02" H 4630 4601 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x01_P2.54mm_Horizontal" H 4550 4700 50  0001 C CNN
F 3 "~" H 4550 4700 50  0001 C CNN
F 4 "PREC001DBAN-M71RC" H 4550 4700 50  0001 C CNN "MNP"
	1    4550 4700
	1    0    0    -1  
$EndComp
Text HLabel 4250 4700 0    50   Input ~ 0
1_VBUS
Text HLabel 4250 4800 0    50   Input ~ 0
5_GND
$EndSCHEMATC
