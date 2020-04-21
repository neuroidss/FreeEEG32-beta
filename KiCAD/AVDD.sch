EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 10 22
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text HLabel 2300 2750 3    50   Input ~ 0
1_IN
Text HLabel 2500 2750 3    50   Input ~ 0
2_GND
Text HLabel 3950 2800 3    50   Input ~ 0
5_OUT
Text HLabel 4150 2800 3    50   Input ~ 0
2_GND
Wire Wire Line
	3950 2800 3950 2700
Wire Wire Line
	4150 2800 4150 2700
$Comp
L Device:C_Small C?
U 1 1 5CDAC673
P 4050 2700
AR Path="/5CDBF57D/5CDAC673" Ref="C?"  Part="1" 
AR Path="/5CE92451/5CDAC673" Ref="C?"  Part="1" 
AR Path="/5CE92453/5CDAC673" Ref="C?"  Part="1" 
AR Path="/5CE92455/5CDAC673" Ref="C?"  Part="1" 
AR Path="/5CDAC673" Ref="C?"  Part="1" 
AR Path="/5CDAC31F/5CDAC673" Ref="C86"  Part="1" 
AR Path="/5CDACBFB/5CDAC673" Ref="C88"  Part="1" 
AR Path="/5D23D721/5CDAC673" Ref="C110"  Part="1" 
F 0 "C110" H 4142 2746 50  0000 L CNN
F 1 "2.2u" H 4142 2655 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4050 2700 50  0001 C CNN
F 3 "~" H 4050 2700 50  0001 C CNN
F 4 "GRM188R71A225KE15D" H 4050 2700 50  0001 C CNN "MNP"
	1    4050 2700
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4450 1900 4550 1900
Text HLabel 4450 1900 0    50   Input ~ 0
5_OUT
Wire Wire Line
	2150 2100 2050 2100
Text HLabel 2150 2100 2    50   Input ~ 0
3_EN
Wire Wire Line
	2150 2000 2050 2000
Text HLabel 2150 2000 2    50   Input ~ 0
2_GND
Wire Wire Line
	2150 1900 2050 1900
Text HLabel 2150 1900 2    50   Input ~ 0
1_IN
Wire Wire Line
	3800 1900 3700 1900
Text HLabel 3800 1900 2    50   Input ~ 0
5_OUT
Wire Wire Line
	2500 2750 2500 2650
Wire Wire Line
	2300 2750 2300 2650
$Comp
L Device:C_Small C?
U 1 1 5CDAC686
P 2400 2650
AR Path="/5CDBF57D/5CDAC686" Ref="C?"  Part="1" 
AR Path="/5CE92451/5CDAC686" Ref="C?"  Part="1" 
AR Path="/5CE92453/5CDAC686" Ref="C?"  Part="1" 
AR Path="/5CE92455/5CDAC686" Ref="C?"  Part="1" 
AR Path="/5CDAC686" Ref="C?"  Part="1" 
AR Path="/5CDAC31F/5CDAC686" Ref="C85"  Part="1" 
AR Path="/5CDACBFB/5CDAC686" Ref="C87"  Part="1" 
AR Path="/5D23D721/5CDAC686" Ref="C109"  Part="1" 
F 0 "C109" H 2492 2696 50  0000 L CNN
F 1 "2.2u" H 2492 2605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2400 2650 50  0001 C CNN
F 3 "~" H 2400 2650 50  0001 C CNN
F 4 "GRM188R71A225KE15D" H 2400 2650 50  0001 C CNN "MNP"
F 5 "" H 2400 2650 50  0001 C CNN "Manufacturer"
	1    2400 2650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2800 2100 2900 2100
Text HLabel 2800 2100 0    50   Input ~ 0
3_EN
Wire Wire Line
	2800 2000 2900 2000
Text HLabel 2800 2000 0    50   Input ~ 0
2_GND
Wire Wire Line
	2800 1900 2900 1900
Text HLabel 2800 1900 0    50   Input ~ 0
1_IN
$Comp
L Connector_Generic:Conn_01x03 J?
U 1 1 5CDAC693
P 1800 2000
AR Path="/5CDAC693" Ref="J?"  Part="1" 
AR Path="/5CDAC31F/5CDAC693" Ref="J17"  Part="1" 
AR Path="/5CDACBFB/5CDAC693" Ref="J19"  Part="1" 
AR Path="/5D23D721/5CDAC693" Ref="J29"  Part="1" 
F 0 "J29" H 1720 1675 50  0000 C CNN
F 1 "Conn_01x03" H 1720 1766 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 1800 2000 50  0001 C CNN
F 3 "~" H 1800 2000 50  0001 C CNN
F 4 "" H 1800 2000 50  0001 C CNN "MNP"
F 5 "Kls, CONNFLY" H 1800 2000 50  0001 C CNN "Manufacturer"
	1    1800 2000
	-1   0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J?
U 1 1 5CDAC69A
P 4800 2000
AR Path="/5CDAC69A" Ref="J?"  Part="1" 
AR Path="/5CDAC31F/5CDAC69A" Ref="J18"  Part="1" 
AR Path="/5CDACBFB/5CDAC69A" Ref="J20"  Part="1" 
AR Path="/5D23D721/5CDAC69A" Ref="J30"  Part="1" 
F 0 "J30" H 4880 2042 50  0000 L CNN
F 1 "Conn_01x03" H 4880 1951 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 4800 2000 50  0001 C CNN
F 3 "~" H 4800 2000 50  0001 C CNN
	1    4800 2000
	1    0    0    -1  
$EndComp
$Comp
L lp5907:LP5907 U?
U 1 1 5CDAC6A1
P 3300 2000
AR Path="/5CDAC6A1" Ref="U?"  Part="1" 
AR Path="/5CDAC31F/5CDAC6A1" Ref="U5"  Part="1" 
AR Path="/5CDACBFB/5CDAC6A1" Ref="U6"  Part="1" 
AR Path="/5D23D721/5CDAC6A1" Ref="U10"  Part="1" 
F 0 "U10" H 3300 2453 60  0000 C CNN
F 1 "LP5907" H 3300 2347 60  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 3300 2241 60  0000 C CNN
F 3 "" H 3300 2000 60  0000 C CNN
F 4 "LP5907QMFX-3.3Q1" H 3300 2000 50  0001 C CNN "MNP"
F 5 "Texas Instruments" H 3300 2000 50  0001 C CNN "Manufacturer"
	1    3300 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 2100 4550 2100
Text HLabel 4450 2100 0    50   Input ~ 0
4_NC
Wire Wire Line
	3800 2100 3700 2100
Text HLabel 3800 2100 2    50   Input ~ 0
4_NC
$EndSCHEMATC
