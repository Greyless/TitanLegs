EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 1 1
Title "Titan Legs PCB Design"
Date ""
Rev "v1.0"
Comp "Society of Robotics and Automation, VJTI"
Comment1 ""
Comment2 "MIT License"
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L power:GND #PWR016
U 1 1 612ACDB6
P 10535 4115
F 0 "#PWR016" H 10535 3865 50  0001 C CNN
F 1 "GND" H 10540 3942 50  0000 C CNN
F 2 "" H 10535 4115 50  0001 C CNN
F 3 "" H 10535 4115 50  0001 C CNN
	1    10535 4115
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR023
U 1 1 61360C33
P 14095 3025
F 0 "#PWR023" H 14095 2775 50  0001 C CNN
F 1 "GND" H 14100 2852 50  0000 C CNN
F 2 "" H 14095 3025 50  0001 C CNN
F 3 "" H 14095 3025 50  0001 C CNN
	1    14095 3025
	1    0    0    -1  
$EndComp
$Comp
L Device:C C19
U 1 1 613C1C94
P 13410 2810
F 0 "C19" H 13240 2815 50  0000 C CNN
F 1 "1uF 6.3V" H 13230 2915 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 13448 2660 50  0001 C CNN
F 3 "~" H 13410 2810 50  0001 C CNN
	1    13410 2810
	-1   0    0    1   
$EndComp
$Comp
L Device:C C21
U 1 1 614B2098
P 15675 2825
F 0 "C21" H 15490 2840 50  0000 C CNN
F 1 "1uF 6.3V" H 15470 2925 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 15713 2675 50  0001 C CNN
F 3 "~" H 15675 2825 50  0001 C CNN
	1    15675 2825
	-1   0    0    1   
$EndComp
$Comp
L Device:C C18
U 1 1 61590AEF
P 13645 1390
F 0 "C18" H 13755 1390 50  0000 L CNN
F 1 "2.2uF 16V" H 13690 1295 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 13683 1240 50  0001 C CNN
F 3 "~" H 13645 1390 50  0001 C CNN
	1    13645 1390
	1    0    0    -1  
$EndComp
$Comp
L Device:C C20
U 1 1 6159B72C
P 14230 1390
F 0 "C20" H 14335 1400 50  0000 L CNN
F 1 "1uF 16V" H 14260 1295 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 14268 1240 50  0001 C CNN
F 3 "~" H 14230 1390 50  0001 C CNN
	1    14230 1390
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C4
U 1 1 61331388
P 3445 1885
F 0 "C4" V 3216 1885 50  0000 C CNN
F 1 "10 uF 50V" V 3307 1885 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3445 1885 50  0001 C CNN
F 3 "~" H 3445 1885 50  0001 C CNN
	1    3445 1885
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C5
U 1 1 61331461
P 3635 1885
F 0 "C5" V 3850 1885 50  0000 C CNN
F 1 "0.1 uF 50V" V 3755 1885 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3635 1885 50  0001 C CNN
F 3 "~" H 3635 1885 50  0001 C CNN
	1    3635 1885
	1    0    0    -1  
$EndComp
Wire Wire Line
	3445 1985 3445 2095
Wire Wire Line
	3445 1785 3445 1685
$Comp
L power:GND #PWR08
U 1 1 613638F2
P 3540 2095
F 0 "#PWR08" H 3540 1845 50  0001 C CNN
F 1 "GND" H 3545 1922 50  0000 C CNN
F 2 "" H 3540 2095 50  0001 C CNN
F 3 "" H 3540 2095 50  0001 C CNN
	1    3540 2095
	1    0    0    -1  
$EndComp
Wire Wire Line
	3635 1685 3635 1785
Wire Wire Line
	3635 1985 3635 2095
Wire Wire Line
	3445 2095 3540 2095
Connection ~ 3540 2095
Wire Wire Line
	3540 2095 3635 2095
$Comp
L RF_Module:ESP32-WROOM-32D U1
U 1 1 613284E1
P 1500 2535
F 0 "U1" H 1055 3880 50  0000 C CNN
F 1 "ESP32-WROOM-32D" V 1500 2545 50  0000 C CNN
F 2 "RF_Module:ESP32-WROOM-32" H 1500 1035 50  0001 C CNN
F 3 "https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32d_esp32-wroom-32u_datasheet_en.pdf" H 1200 2585 50  0001 C CNN
	1    1500 2535
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR03
U 1 1 6132F0D7
P 1500 3935
F 0 "#PWR03" H 1500 3685 50  0001 C CNN
F 1 "GND" H 1505 3762 50  0000 C CNN
F 2 "" H 1500 3935 50  0001 C CNN
F 3 "" H 1500 3935 50  0001 C CNN
	1    1500 3935
	1    0    0    -1  
$EndComp
Text Notes 1180 785  0    157  ~ 31
Microcontroller (ESP32)
$Comp
L Switch:SW_Push SW1
U 1 1 61407B87
P 3425 3595
F 0 "SW1" V 3471 3547 50  0000 R CNN
F 1 "BOOT" V 3380 3547 50  0000 R CNN
F 2 "Tactile_Switch:Tactile_Switch" H 3425 3795 50  0001 C CNN
F 3 "~" H 3425 3795 50  0001 C CNN
	1    3425 3595
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_Small_US R1
U 1 1 6140AE4F
P 3425 3080
F 0 "R1" H 3493 3126 50  0000 L CNN
F 1 "10k" H 3493 3035 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 3425 3080 50  0001 C CNN
F 3 "~" H 3425 3080 50  0001 C CNN
	1    3425 3080
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small_US R4
U 1 1 6140B20D
P 4380 3075
F 0 "R4" H 4448 3121 50  0000 L CNN
F 1 "10k" H 4448 3030 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 4380 3075 50  0001 C CNN
F 3 "~" H 4380 3075 50  0001 C CNN
	1    4380 3075
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C7
U 1 1 6140B773
P 4760 3595
F 0 "C7" V 4975 3595 50  0000 C CNN
F 1 "1uF 50V" V 4880 3595 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4760 3595 50  0001 C CNN
F 3 "~" H 4760 3595 50  0001 C CNN
	1    4760 3595
	1    0    0    -1  
$EndComp
Wire Wire Line
	3425 3180 3425 3270
$Comp
L power:GND #PWR07
U 1 1 61446B2F
P 3425 3795
F 0 "#PWR07" H 3425 3545 50  0001 C CNN
F 1 "GND" H 3430 3622 50  0000 C CNN
F 2 "" H 3425 3795 50  0001 C CNN
F 3 "" H 3425 3795 50  0001 C CNN
	1    3425 3795
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR011
U 1 1 6146B3AD
P 4380 3795
F 0 "#PWR011" H 4380 3545 50  0001 C CNN
F 1 "GND" H 4385 3622 50  0000 C CNN
F 2 "" H 4380 3795 50  0001 C CNN
F 3 "" H 4380 3795 50  0001 C CNN
	1    4380 3795
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW2
U 1 1 6140918C
P 4380 3595
F 0 "SW2" V 4426 3547 50  0000 R CNN
F 1 "RESET" V 4335 3547 50  0000 R CNN
F 2 "Tactile_Switch:Tactile_Switch" H 4380 3795 50  0001 C CNN
F 3 "~" H 4380 3795 50  0001 C CNN
	1    4380 3595
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4760 3495 4760 3395
Wire Wire Line
	4760 3395 4380 3395
Connection ~ 4380 3395
Wire Wire Line
	4380 3795 4760 3795
Wire Wire Line
	4760 3795 4760 3695
Connection ~ 4380 3795
Wire Wire Line
	3425 3270 3745 3270
Connection ~ 3425 3270
Wire Wire Line
	3425 3270 3425 3395
Text GLabel 3745 3270 2    39   Input ~ 0
BOOT
Text GLabel 4700 3275 2    39   Input ~ 0
RESET
Wire Wire Line
	4380 3275 4700 3275
Wire Wire Line
	4380 3175 4380 3275
Connection ~ 4380 3275
Wire Wire Line
	4380 3275 4380 3395
Text GLabel 770  1335 0    39   Input ~ 0
RESET
Wire Wire Line
	900  1335 770  1335
Text GLabel 2275 1335 2    39   Input ~ 0
BOOT
Wire Wire Line
	2100 1335 2275 1335
$Comp
L Connector:Conn_01x04_Female J1
U 1 1 615344D0
P 3405 4165
F 0 "J1" V 3343 3877 50  0000 R CNN
F 1 "Conn_01x04_Female" V 3252 3877 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 3405 4165 50  0001 C CNN
F 3 "~" H 3405 4165 50  0001 C CNN
	1    3405 4165
	0    -1   -1   0   
$EndComp
Text GLabel 2275 1435 2    39   Input ~ 0
TX
Text GLabel 2275 1635 2    39   Input ~ 0
RX
Wire Wire Line
	2100 1635 2275 1635
Text GLabel 3305 4565 3    39   Input ~ 0
RX
Wire Wire Line
	2100 1435 2275 1435
Wire Wire Line
	3305 4365 3305 4565
Text GLabel 3405 4565 3    39   Input ~ 0
TX
Wire Wire Line
	3405 4365 3405 4565
Wire Wire Line
	3505 4365 3505 4565
Wire Wire Line
	3605 4365 3605 4565
Text GLabel 3505 4565 3    39   Input ~ 0
3V3_ISOLATED
$Comp
L power:GND #PWR09
U 1 1 615934C4
P 3605 4565
F 0 "#PWR09" H 3605 4315 50  0001 C CNN
F 1 "GND" V 3605 4375 50  0000 C CNN
F 2 "" H 3605 4565 50  0001 C CNN
F 3 "" H 3605 4565 50  0001 C CNN
	1    3605 4565
	1    0    0    -1  
$EndComp
Text Notes 3795 4850 0    59   ~ 0
3V3_ISOLATED is used to \npower ESP32 when board \nisn't being powered \nby VIN. (while programming)\n
Text GLabel 1500 1135 1    39   Input ~ 0
3V3_IN
Text GLabel 2155 4410 0    39   Input ~ 0
3V3_ISOLATED
Text GLabel 2635 4540 2    39   Input ~ 0
3V3_IN
Text GLabel 2155 4665 0    39   Input ~ 0
3V3_REGULATED
Wire Wire Line
	2155 4410 2300 4410
Wire Wire Line
	2300 4410 2300 4540
Wire Wire Line
	2300 4665 2155 4665
Wire Wire Line
	2635 4540 2300 4540
Connection ~ 2300 4540
Wire Wire Line
	2300 4540 2300 4665
Text Notes 1875 5080 0    39   ~ 0
Don't connect 3V3_ISOLATED \nwhile board is powered by Battery
Text GLabel 3540 1585 1    39   Input ~ 0
3V3_IN
Wire Wire Line
	3445 1685 3540 1685
Wire Wire Line
	3540 1585 3540 1685
Connection ~ 3540 1685
Wire Wire Line
	3540 1685 3635 1685
Text Notes 9445 5490 0    157  ~ 31
Temperature Sensor (DS18S20Z+)
Text Notes 5235 795  0    157  ~ 31
3 Phased Gate MOSFET Switches
Wire Notes Line
	6970 470  6975 470 
Wire Notes Line
	9420 4910 9420 4920
Wire Notes Line
	9425 920  9425 925 
Wire Wire Line
	6450 1270 6510 1270
$Comp
L power:GND #PWR013
U 1 1 6162F8AC
P 6510 1270
F 0 "#PWR013" H 6510 1020 50  0001 C CNN
F 1 "GND" H 6515 1097 50  0000 C CNN
F 2 "" H 6510 1270 50  0001 C CNN
F 3 "" H 6510 1270 50  0001 C CNN
	1    6510 1270
	1    0    0    -1  
$EndComp
Wire Wire Line
	5750 4635 5665 4635
Wire Wire Line
	8430 2975 8430 2865
Wire Wire Line
	7090 2990 7090 2850
Connection ~ 5750 1130
Connection ~ 5850 2965
Connection ~ 5950 1130
Wire Wire Line
	5850 1130 5950 1130
Connection ~ 5850 1130
Wire Wire Line
	5750 1130 5850 1130
Connection ~ 7090 2990
Text Notes 3325 1240 0    59   ~ 0
Decoupling caps, \nkeep close to\npower pins\n
Wire Notes Line width 10
	2970 4060 5155 4060
Wire Notes Line width 10
	2970 2410 5155 2410
Wire Notes Line width 10
	490  4245 2975 4245
Wire Notes Line width 10
	2970 2410 2970 5145
$Comp
L AS5600-ASOT:AS5600-ASOT U2
U 1 1 6180CBD6
P 2205 6345
F 0 "U2" H 2805 6610 50  0000 C CNN
F 1 "AS5600-ASOT" H 2805 6519 50  0000 C CNN
F 2 "AS5600:AS5600" H 3255 6445 50  0001 L CNN
F 3 "http://www.ams.com/eng/content/download/639463/1698857/325295" H 3255 6345 50  0001 L CNN
F 4 "Board Mount Hall Effect / Magnetic Sensors AS5600 Magnetic Sensor 12-Bit" H 3255 6245 50  0001 L CNN "Description"
F 5 "1.75" H 3255 6145 50  0001 L CNN "Height"
F 6 "ams" H 3255 6045 50  0001 L CNN "Manufacturer_Name"
F 7 "AS5600-ASOT" H 3255 5945 50  0001 L CNN "Manufacturer_Part_Number"
F 8 "985-AS5600-ASOT" H 3255 5845 50  0001 L CNN "Mouser Part Number"
F 9 "https://www.mouser.co.uk/ProductDetail/ams/AS5600-ASOT?qs=KTMMzrZdriGJpjhsnAEYBA%3D%3D" H 3255 5745 50  0001 L CNN "Mouser Price/Stock"
F 10 "AS5600-ASOT" H 3255 5645 50  0001 L CNN "Arrow Part Number"
F 11 "https://www.arrow.com/en/products/as5600-asot/ams-ag?region=nac" H 3255 5545 50  0001 L CNN "Arrow Price/Stock"
	1    2205 6345
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C1
U 1 1 61815CDE
P 1330 6550
F 0 "C1" V 1545 6550 50  0000 C CNN
F 1 "100 nF" V 1450 6550 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 1330 6550 50  0001 C CNN
F 3 "~" H 1330 6550 50  0001 C CNN
	1    1330 6550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2205 6645 2135 6645
Wire Wire Line
	2135 6645 2135 6745
$Comp
L power:GND #PWR04
U 1 1 6183198C
P 2135 6745
F 0 "#PWR04" H 2135 6495 50  0001 C CNN
F 1 "GND" V 2135 6555 50  0000 C CNN
F 2 "" H 2135 6745 50  0001 C CNN
F 3 "" H 2135 6745 50  0001 C CNN
	1    2135 6745
	1    0    0    -1  
$EndComp
Wire Wire Line
	2205 6345 2135 6345
Wire Wire Line
	2135 6345 2135 6445
Wire Wire Line
	2135 6445 2205 6445
Wire Wire Line
	2135 6345 2135 6185
Connection ~ 2135 6345
Text GLabel 2135 6185 1    39   Input ~ 0
3V3_IN
Text GLabel 1330 6380 1    39   Input ~ 0
3V3_IN
Wire Wire Line
	1330 6380 1330 6450
Wire Wire Line
	1330 6650 1330 6720
$Comp
L power:GND #PWR01
U 1 1 6186F8DF
P 1330 6720
F 0 "#PWR01" H 1330 6470 50  0001 C CNN
F 1 "GND" V 1330 6530 50  0000 C CNN
F 2 "" H 1330 6720 50  0001 C CNN
F 3 "" H 1330 6720 50  0001 C CNN
	1    1330 6720
	1    0    0    -1  
$EndComp
Text Notes 1190 6795 1    59   ~ 0
Decoupling cap,\nkeep close to\npower pins\n
Text GLabel 3425 2980 1    39   Input ~ 0
3V3_IN
Text GLabel 4380 2975 1    39   Input ~ 0
3V3_IN
Text GLabel 3760 6445 2    39   Input ~ 0
ENC_SDA
Text GLabel 3760 6545 2    39   Input ~ 0
ENC_SCL
Wire Wire Line
	3405 6545 3700 6545
Wire Wire Line
	3760 6445 3585 6445
NoConn ~ 3405 6645
$Comp
L Device:R_Small_US R3
U 1 1 618149BB
P 3700 6125
F 0 "R3" H 3768 6171 50  0000 L CNN
F 1 "4.7k" H 3768 6080 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 3700 6125 50  0001 C CNN
F 3 "~" H 3700 6125 50  0001 C CNN
	1    3700 6125
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small_US R2
U 1 1 61811B61
P 3585 6125
F 0 "R2" H 3660 6080 50  0000 L CNN
F 1 "4.7k" H 3650 6170 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 3585 6125 50  0001 C CNN
F 3 "~" H 3585 6125 50  0001 C CNN
	1    3585 6125
	-1   0    0    1   
$EndComp
Wire Wire Line
	3585 6225 3585 6445
Connection ~ 3585 6445
Wire Wire Line
	3585 6445 3405 6445
Wire Wire Line
	3700 6225 3700 6545
Connection ~ 3700 6545
Wire Wire Line
	3700 6545 3760 6545
Wire Wire Line
	3585 6025 3585 5980
Wire Wire Line
	3585 5980 3645 5980
Wire Wire Line
	3700 5980 3700 6025
Text GLabel 3645 5900 1    39   Input ~ 0
3V3_IN
Wire Wire Line
	3645 5900 3645 5980
Connection ~ 3645 5980
Wire Wire Line
	3645 5980 3700 5980
Wire Wire Line
	2205 6545 1990 6545
Text GLabel 1990 6545 0    39   Input ~ 0
ENC_OUT
Text GLabel 3760 6345 2    39   Input ~ 0
ENC_DIR
Wire Wire Line
	3405 6345 3760 6345
Text Notes 1225 5475 0    157  ~ 31
Magnetic Encoder (AS5600)
Wire Notes Line width 20
	485  7405 5150 7405
Text Notes 4145 6615 0    39   ~ 0
ENC_DIR -> GND, \nthen Clockwise\n\nENC_DIR -> VDD, \nthen Counter Clockwise
Text GLabel 2275 1835 2    39   Input ~ 0
ENC_SDA
Wire Wire Line
	2100 1835 2275 1835
Text GLabel 2275 2535 2    39   Input ~ 0
ENC_SCL
Wire Wire Line
	2100 2535 2275 2535
Text GLabel 2275 2435 2    39   Input ~ 0
ENC_DIR
Text GLabel 2275 2335 2    39   Input ~ 0
ENC_OUT
Wire Wire Line
	2100 2335 2275 2335
Wire Wire Line
	2275 2435 2100 2435
$Comp
L Regulator_Switching:LM2672M-3.3 U3
U 1 1 6131BA8C
P 3075 8680
F 0 "U3" H 3075 9147 50  0000 C CNN
F 1 "LM2672M-3.3" H 3075 9056 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 3125 8330 50  0001 L CIN
F 3 "http://www.ti.com/lit/ds/symlink/lm2672.pdf" H 3075 8680 50  0001 C CNN
	1    3075 8680
	-1   0    0    -1  
$EndComp
Text Notes 1190 7775 0    157  ~ 31
Power Supply (VIN to 3.3v)
$Comp
L power:GND #PWR06
U 1 1 61396B76
P 3075 9290
F 0 "#PWR06" H 3075 9040 50  0001 C CNN
F 1 "GND" V 3075 9100 50  0000 C CNN
F 2 "" H 3075 9290 50  0001 C CNN
F 3 "" H 3075 9290 50  0001 C CNN
	1    3075 9290
	1    0    0    -1  
$EndComp
NoConn ~ 3575 8880
NoConn ~ 3575 8680
NoConn ~ 3575 8780
Wire Wire Line
	3575 8480 3980 8480
Text GLabel 4550 8480 2    39   Input ~ 0
POWER_IN
Wire Wire Line
	3980 8655 3980 8480
$Comp
L power:GND #PWR010
U 1 1 61425A4E
P 3980 9300
F 0 "#PWR010" H 3980 9050 50  0001 C CNN
F 1 "GND" V 3980 9110 50  0000 C CNN
F 2 "" H 3980 9300 50  0001 C CNN
F 3 "" H 3980 9300 50  0001 C CNN
	1    3980 9300
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C3
U 1 1 6143826A
P 2370 8780
F 0 "C3" H 2220 8770 50  0000 C CNN
F 1 "50V 0.01uF " H 2360 8920 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2370 8780 50  0001 C CNN
F 3 "~" H 2370 8780 50  0001 C CNN
	1    2370 8780
	1    0    0    -1  
$EndComp
Text GLabel 1230 8880 0    39   Input ~ 0
3V3_REGULATED
Wire Wire Line
	3980 8955 3980 9300
Wire Wire Line
	3075 9080 3075 9290
$Comp
L power:GND #PWR02
U 1 1 614ACA2E
P 1615 9335
F 0 "#PWR02" H 1615 9085 50  0001 C CNN
F 1 "GND" V 1615 9145 50  0000 C CNN
F 2 "" H 1615 9335 50  0001 C CNN
F 3 "" H 1615 9335 50  0001 C CNN
	1    1615 9335
	1    0    0    -1  
$EndComp
Wire Wire Line
	3980 8480 4550 8480
Connection ~ 3980 8480
Wire Wire Line
	1230 8880 1615 8880
Connection ~ 1615 8880
Wire Wire Line
	1615 8880 1840 8880
Connection ~ 2370 8880
Wire Wire Line
	2370 8880 2575 8880
Wire Wire Line
	2370 8680 2575 8680
$Comp
L power:GND #PWR05
U 1 1 61562ED3
P 2370 9285
F 0 "#PWR05" H 2370 9035 50  0001 C CNN
F 1 "GND" V 2370 9095 50  0000 C CNN
F 2 "" H 2370 9285 50  0001 C CNN
F 3 "" H 2370 9285 50  0001 C CNN
	1    2370 9285
	1    0    0    -1  
$EndComp
Wire Wire Line
	2370 8955 2370 8880
Wire Wire Line
	2370 9255 2370 9285
$Comp
L Device:L L1
U 1 1 61466B3D
P 1990 8880
F 0 "L1" V 2180 8880 50  0000 C CNN
F 1 "47uH" V 2089 8880 50  0000 C CNN
F 2 "8D43_Inductor:8D43" H 1990 8880 50  0001 C CNN
F 3 "~" H 1990 8880 50  0001 C CNN
	1    1990 8880
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2140 8880 2370 8880
Wire Wire Line
	2575 8480 1615 8480
Wire Wire Line
	1615 8480 1615 8880
Text Notes 1585 9995 0    59   ~ 0
Keep Bypass Capacitors\n close to Chip
Text Notes 1490 8395 0    59   ~ 0
Keep Inductor away \nfrom Feedback Route
Text Notes 3615 9825 0    59   ~ 0
SS - Soft Start\nConnect Capacitor for soft start
Text Notes 3640 10265 0    59   ~ 0
Pull ON/~OFF~ High\nTo enable Switching\nNC - Internal Default \nPull Up
$Comp
L Device:D_Schottky D1
U 1 1 613262EF
P 2370 9105
F 0 "D1" V 2324 9184 50  0000 L CNN
F 1 "3A 40V" V 2415 9184 50  0000 L CNN
F 2 "Diode_SMD:D_0603_1608Metric" H 2370 9105 50  0001 C CNN
F 3 "~" H 2370 9105 50  0001 C CNN
	1    2370 9105
	0    1    1    0   
$EndComp
Wire Wire Line
	1615 9270 1615 9335
Wire Notes Line width 20
	5155 505  5155 11210
Wire Notes Line width 20
	485  5150 9380 5150
Wire Notes Line width 20
	9380 490  9380 11195
Text Notes 9610 785  0    157  ~ 31
DRV8305 - Gate Driver
$Comp
L Device:C C24
U 1 1 6135FBF8
P 14095 2875
F 0 "C24" H 13935 2865 50  0000 C CNN
F 1 " 1uF 6.3V" H 13930 2970 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 14133 2725 50  0001 C CNN
F 3 "~" H 14095 2875 50  0001 C CNN
	1    14095 2875
	-1   0    0    1   
$EndComp
Text Notes 12625 3410 0    39   ~ 0
SO1, SO2 and SO3 are current amplifier output pins
Text Notes 12625 3575 0    39   ~ 0
SP1, SP2 and SP3 are positive current sense\ninput pins for amplifers 1,2 and 3 respectively\n
Text Notes 12630 3755 0    39   ~ 0
SN1, SN2 and SN3 are negative current sense \ninput pins for amplifers 1,2 and 3 respectively\n
Text Notes 13370 670  0    39   ~ 0
VCPH - High Side Gate Driver regulator\nVCP_LSD - Low SIde Gate Driver regulator
Text GLabel 11435 1265 1    39   Input ~ 0
DRV_HG_reg
Text GLabel 11535 1265 1    39   Input ~ 0
DRV_LG_reg
Text GLabel 11135 1265 1    39   Input ~ 0
PVDD
Text GLabel 13645 1590 3    39   Input ~ 0
PVDD
Text GLabel 13645 1170 1    39   Input ~ 0
DRV_HG_reg
Text GLabel 14230 1170 1    39   Input ~ 0
DRV_LG_reg
$Comp
L power:GND #PWR021
U 1 1 61596AC8
P 14230 1610
F 0 "#PWR021" H 14230 1360 50  0001 C CNN
F 1 "GND" H 14235 1437 50  0000 C CNN
F 2 "" H 14230 1610 50  0001 C CNN
F 3 "" H 14230 1610 50  0001 C CNN
	1    14230 1610
	1    0    0    -1  
$EndComp
Wire Wire Line
	13645 1170 13645 1240
Wire Wire Line
	13645 1540 13645 1590
Wire Wire Line
	14230 1540 14230 1610
Wire Wire Line
	14230 1240 14230 1170
Text GLabel 11335 1265 1    39   Input ~ 0
CP2H
Text GLabel 11235 1265 1    39   Input ~ 0
CP2L
Text GLabel 14995 1055 1    39   Input ~ 0
CP1H
Text GLabel 14995 1470 3    39   Input ~ 0
CP1L
Text GLabel 10935 1265 1    39   Input ~ 0
CP1H
Text GLabel 11035 1265 1    39   Input ~ 0
CP1L
Wire Wire Line
	11535 1455 11535 1265
Wire Wire Line
	11435 1455 11435 1265
Wire Wire Line
	11335 1455 11335 1265
Wire Wire Line
	11235 1455 11235 1265
Wire Wire Line
	11135 1455 11135 1265
Wire Wire Line
	10435 1455 10435 1265
Wire Wire Line
	10735 1455 10735 1265
Wire Wire Line
	11035 1455 11035 1265
Wire Wire Line
	10935 1455 10935 1265
$Comp
L Device:C C22
U 1 1 6191E71A
P 14995 1260
F 0 "C22" H 15110 1306 50  0000 L CNN
F 1 "0.047uF" H 15110 1215 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 15033 1110 50  0001 C CNN
F 3 "~" H 14995 1260 50  0001 C CNN
	1    14995 1260
	1    0    0    -1  
$EndComp
Wire Wire Line
	14995 1055 14995 1110
Wire Wire Line
	14995 1410 14995 1470
Text GLabel 15525 1050 1    39   Input ~ 0
CP2H
Text GLabel 15525 1465 3    39   Input ~ 0
CP2L
$Comp
L Device:C C23
U 1 1 6198BEA1
P 15525 1255
F 0 "C23" H 15640 1301 50  0000 L CNN
F 1 "0.047uF" H 15640 1210 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 15563 1105 50  0001 C CNN
F 3 "~" H 15525 1255 50  0001 C CNN
	1    15525 1255
	1    0    0    -1  
$EndComp
Wire Wire Line
	15525 1050 15525 1105
Wire Wire Line
	15525 1405 15525 1465
Text Notes 14955 795  0    39   ~ 0
Flying capacitors for \ncharge pumps with rating :\nPVDD for CP1\nPVDD*2 for CP2
$Comp
L power:GND #PWR017
U 1 1 61A45317
P 10735 1265
F 0 "#PWR017" H 10735 1015 50  0001 C CNN
F 1 "GND" H 10740 1092 50  0000 C CNN
F 2 "" H 10735 1265 50  0001 C CNN
F 3 "" H 10735 1265 50  0001 C CNN
	1    10735 1265
	-1   0    0    1   
$EndComp
$Comp
L Device:C C6
U 1 1 61A6629F
P 3980 8805
F 0 "C6" H 4095 8851 50  0000 L CNN
F 1 "100uF 100V" H 4095 8760 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4018 8655 50  0001 C CNN
F 3 "~" H 3980 8805 50  0001 C CNN
	1    3980 8805
	1    0    0    -1  
$EndComp
Text Notes 2225 9695 0    39   ~ 0
do2014ac footprint
$Comp
L Device:C C2
U 1 1 61A8BF16
P 1615 9120
F 0 "C2" H 1730 9166 50  0000 L CNN
F 1 "68uF 10V" H 1730 9075 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 1653 8970 50  0001 C CNN
F 3 "~" H 1615 9120 50  0001 C CNN
	1    1615 9120
	1    0    0    -1  
$EndComp
Wire Wire Line
	1615 8880 1615 8970
$Comp
L Device:R R8
U 1 1 614CBA09
P 15105 2650
F 0 "R8" H 14940 2585 50  0000 L CNN
F 1 "100" H 14905 2700 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 15035 2650 50  0001 C CNN
F 3 "~" H 15105 2650 50  0001 C CNN
	1    15105 2650
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR020
U 1 1 613CEC5C
P 13410 3030
F 0 "#PWR020" H 13410 2780 50  0001 C CNN
F 1 "GND" H 13415 2857 50  0000 C CNN
F 2 "" H 13410 3030 50  0001 C CNN
F 3 "" H 13410 3030 50  0001 C CNN
	1    13410 3030
	1    0    0    -1  
$EndComp
Text GLabel 10635 4245 3    39   Input ~ 0
DRV_AVDD
Text GLabel 10435 1265 1    39   Input ~ 0
DRV_VREG
Text GLabel 15675 2635 1    39   Input ~ 0
DRV_DVDD
Text GLabel 13410 2590 1    39   Input ~ 0
DRV_VREG
Wire Wire Line
	10635 1455 10635 1275
Text GLabel 10635 1275 1    39   Input ~ 0
DRV_DVDD
$Comp
L DRV8305NPHPR:DRV8305NPHPR IC1
U 1 1 61052D6F
P 10135 2255
F 0 "IC1" H 10940 1915 50  0000 L CNN
F 1 "DRV8305NPHPR" H 10720 1670 50  0000 L CNN
F 2 "DRV8305NPHPR:DRV8305NPHPR" H 11685 2855 50  0001 L CNN
F 3 "http://www.mouser.com/ds/2/405/drv8305-768621.pdf" H 11685 2755 50  0001 L CNN
F 4 "Motor / Motion / Ignition Controllers & Drivers Three Phase Gate Driver with Three Integrated Current Shunt Amplifiers 48-HTQFP -40 to 125" H 11685 2655 50  0001 L CNN "Description"
F 5 "1.2" H 11685 2555 50  0001 L CNN "Height"
F 6 "Texas Instruments" H 11685 2455 50  0001 L CNN "Manufacturer_Name"
F 7 "DRV8305NPHPR" H 11685 2355 50  0001 L CNN "Manufacturer_Part_Number"
F 8 "595-DRV8305NPHPR" H 11685 2255 50  0001 L CNN "Mouser Part Number"
F 9 "https://www.mouser.co.uk/ProductDetail/Texas-Instruments/DRV8305NPHPR?qs=kFLL9AfgRSDaehWhXenR2Q%3D%3D" H 11685 2155 50  0001 L CNN "Mouser Price/Stock"
F 10 "DRV8305NPHPR" H 11685 2055 50  0001 L CNN "Arrow Part Number"
F 11 "https://www.arrow.com/en/products/drv8305nphpr/texas-instruments" H 11685 1955 50  0001 L CNN "Arrow Price/Stock"
	1    10135 2255
	1    0    0    -1  
$EndComp
Wire Wire Line
	10535 4055 10535 4115
$Comp
L power:GND #PWR022
U 1 1 615B2683
P 15675 3040
F 0 "#PWR022" H 15675 2790 50  0001 C CNN
F 1 "GND" H 15680 2867 50  0000 C CNN
F 2 "" H 15675 3040 50  0001 C CNN
F 3 "" H 15675 3040 50  0001 C CNN
	1    15675 3040
	1    0    0    -1  
$EndComp
Wire Wire Line
	13410 2590 13410 2660
Wire Wire Line
	13410 2960 13410 3030
Wire Wire Line
	15675 3040 15675 2975
Wire Wire Line
	15675 2635 15675 2675
Text Notes 15500 2265 0    39   ~ 0
DRV_DVDD: 3.3V \ninternal digital\nsupply regulator
Text Notes 12600 3145 0    39   ~ 0
Connect with-> \n3.3V or 5V, 50-mA LDO; \n  connect 1-µF to GND\nReference voltage; \n    LDO disabled
Text GLabel 15105 2895 3    39   Input ~ 0
VDRAIN
Text GLabel 12720 990  1    39   Input ~ 0
PVDD
$Comp
L Device:C C14
U 1 1 6169BF2D
P 12720 1205
F 0 "C14" H 12550 1210 50  0000 C CNN
F 1 "4.7uF" H 12535 1310 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 12758 1055 50  0001 C CNN
F 3 "~" H 12720 1205 50  0001 C CNN
	1    12720 1205
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR018
U 1 1 6169BF33
P 12720 1485
F 0 "#PWR018" H 12720 1235 50  0001 C CNN
F 1 "GND" H 12725 1312 50  0000 C CNN
F 2 "" H 12720 1485 50  0001 C CNN
F 3 "" H 12720 1485 50  0001 C CNN
	1    12720 1485
	1    0    0    -1  
$EndComp
Wire Wire Line
	12720 1355 12720 1485
Wire Wire Line
	12720 990  12720 1055
Wire Wire Line
	10835 1455 10835 1265
Text GLabel 10835 1265 1    39   Input ~ 0
VDRAIN
Text GLabel 15105 2435 1    39   Input ~ 0
PVDD
Wire Wire Line
	15105 2500 15105 2435
Wire Wire Line
	15105 2895 15105 2800
Text GLabel 6935 1130 0    39   Input ~ 0
PVDD
Text GLabel 8275 1155 0    39   Input ~ 0
PVDD
Wire Wire Line
	10635 4245 10635 4055
Text GLabel 14095 2660 1    39   Input ~ 0
DRV_AVDD
Wire Wire Line
	14095 2660 14095 2725
NoConn ~ 10535 1455
Text GLabel 11435 4245 3    39   Input ~ 0
SN1
Text GLabel 6320 6465 3    39   Input ~ 0
SP2
Text GLabel 11235 4245 3    39   Input ~ 0
SN2
Text GLabel 11035 4245 3    39   Input ~ 0
SN3
Text GLabel 7120 6480 3    39   Input ~ 0
SP3
Text GLabel 11535 4245 3    39   Input ~ 0
SP1
Text GLabel 11335 4245 3    39   Input ~ 0
SP2
Text GLabel 11135 4245 3    39   Input ~ 0
SP3
Text GLabel 10735 4245 3    39   Input ~ 0
SO1
Text GLabel 10835 4245 3    39   Input ~ 0
SO2
Text GLabel 10935 4245 3    39   Input ~ 0
SO3
Wire Wire Line
	10835 4245 10835 4055
Wire Wire Line
	10735 4245 10735 4055
Wire Wire Line
	10935 4245 10935 4055
Wire Wire Line
	11035 4245 11035 4055
Wire Wire Line
	11235 4245 11235 4055
Wire Wire Line
	11135 4245 11135 4055
Wire Wire Line
	11335 4245 11335 4055
Wire Wire Line
	11435 4245 11435 4055
Wire Wire Line
	11535 4245 11535 4055
Wire Wire Line
	6320 6345 6320 6465
Wire Wire Line
	7120 6360 7120 6480
Text GLabel 12025 2255 2    39   Input ~ 0
GHA
Text GLabel 12025 2355 2    39   Input ~ 0
SHA
Text GLabel 12025 2455 2    39   Input ~ 0
SLA
Text GLabel 12025 2555 2    39   Input ~ 0
GLA
Wire Wire Line
	12025 2555 11835 2555
Wire Wire Line
	12025 2355 11835 2355
Wire Wire Line
	12025 2455 11835 2455
Wire Wire Line
	12025 2255 11835 2255
Text GLabel 12025 2655 2    39   Input ~ 0
GHB
Text GLabel 12025 2755 2    39   Input ~ 0
SHB
Text GLabel 12025 2855 2    39   Input ~ 0
SLB
Text GLabel 12025 2955 2    39   Input ~ 0
GLB
Wire Wire Line
	12025 2955 11835 2955
Wire Wire Line
	12025 2755 11835 2755
Wire Wire Line
	12025 2855 11835 2855
Wire Wire Line
	12025 2655 11835 2655
Text GLabel 12025 3055 2    39   Input ~ 0
GHC
Text GLabel 12025 3155 2    39   Input ~ 0
SHC
Text GLabel 12025 3255 2    39   Input ~ 0
SLC
Text GLabel 12025 3355 2    39   Input ~ 0
GLC
Wire Wire Line
	12025 3355 11835 3355
Wire Wire Line
	12025 3155 11835 3155
Wire Wire Line
	12025 3255 11835 3255
Wire Wire Line
	12025 3055 11835 3055
Text GLabel 6110 2730 2    39   Input ~ 0
GHA
Text GLabel 5665 2965 0    39   Input ~ 0
SHA
Text GLabel 5665 4635 0    39   Input ~ 0
SLA
Text GLabel 6095 4635 2    39   Input ~ 0
GLA
Wire Wire Line
	5750 1130 5710 1130
Text GLabel 5710 1130 0    39   Input ~ 0
PVDD
Text GLabel 7350 2730 2    39   Input ~ 0
GHB
Text GLabel 6890 2990 0    39   Input ~ 0
SHB
Text GLabel 8690 2755 2    39   Input ~ 0
GHC
Text GLabel 8255 2975 0    39   Input ~ 0
SHC
Text GLabel 9945 3355 0    39   Input ~ 0
DRV_SPI_CLK
Text GLabel 9945 3255 0    39   Input ~ 0
DRV_SPI_MISO
Text GLabel 9945 3155 0    39   Input ~ 0
DRV_SPI_MOSI
Text GLabel 9945 3055 0    39   Input ~ 0
DRV_SPI_nSCS
Wire Wire Line
	9945 3055 10135 3055
Wire Wire Line
	9945 3255 10135 3255
Wire Wire Line
	9945 3155 10135 3155
Wire Wire Line
	9945 3355 10135 3355
Text GLabel 9945 2955 0    39   Input ~ 0
DRV_nFAULT
Text GLabel 9945 2855 0    39   Input ~ 0
DRV_LC
Text GLabel 9945 2755 0    39   Input ~ 0
DRV_HC
Text GLabel 9945 2655 0    39   Input ~ 0
DRV_LB
Wire Wire Line
	9945 2655 10135 2655
Wire Wire Line
	9945 2855 10135 2855
Wire Wire Line
	9945 2755 10135 2755
Wire Wire Line
	9945 2955 10135 2955
Text GLabel 9945 2555 0    39   Input ~ 0
DRV_HB
Text GLabel 9945 2455 0    39   Input ~ 0
DRV_LA
Text GLabel 9945 2355 0    39   Input ~ 0
DRV_HA
Text GLabel 9945 2255 0    39   Input ~ 0
DRV_EN_GATE
Wire Wire Line
	9945 2255 10135 2255
Wire Wire Line
	9945 2455 10135 2455
Wire Wire Line
	9945 2355 10135 2355
Wire Wire Line
	9945 2555 10135 2555
Wire Notes Line width 20
	9395 5145 16085 5145
Wire Notes Line
	16070 5145 16070 5150
Wire Notes Line
	16060 2040 16060 2025
Wire Notes Line width 10
	13305 500  13305 2050
Wire Notes Line
	12380 5145 12375 5145
Wire Notes Line
	15205 5140 15200 5140
Text GLabel 2290 2135 2    39   Input ~ 0
DRV_SPI_CLK
Text GLabel 2290 1935 2    39   Input ~ 0
DRV_SPI_MISO
Text GLabel 2290 2035 2    39   Input ~ 0
DRV_SPI_MOSI
Text GLabel 2290 3335 2    39   Input ~ 0
DRV_SPI_nSCS
Wire Wire Line
	2290 3335 2100 3335
Wire Wire Line
	2290 1935 2100 1935
Wire Wire Line
	2290 2035 2100 2035
Wire Wire Line
	2290 2135 2100 2135
Text GLabel 2290 3535 2    39   Input ~ 0
DRV_nFAULT
Text GLabel 2290 3135 2    39   Input ~ 0
DRV_LC
Text GLabel 2290 3035 2    39   Input ~ 0
DRV_HC
Text GLabel 2290 2935 2    39   Input ~ 0
DRV_LB
Wire Wire Line
	2290 2935 2100 2935
Wire Wire Line
	2290 3135 2100 3135
Wire Wire Line
	2290 3035 2100 3035
Wire Wire Line
	2290 3535 2100 3535
Text GLabel 2290 2835 2    39   Input ~ 0
DRV_HB
Text GLabel 2290 2735 2    39   Input ~ 0
DRV_LA
Text GLabel 2290 2635 2    39   Input ~ 0
DRV_HA
Text GLabel 2290 3235 2    39   Input ~ 0
DRV_EN_GATE
Wire Wire Line
	2290 3235 2100 3235
Wire Wire Line
	2290 2735 2100 2735
Wire Wire Line
	2290 2635 2100 2635
Wire Wire Line
	2290 2835 2100 2835
Text GLabel 14420 3855 1    39   Input ~ 0
DRV_VREG
$Comp
L Device:R R6
U 1 1 623A6BA6
P 14420 4155
F 0 "R6" H 14490 4201 50  0000 L CNN
F 1 "10k" H 14480 4110 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 14350 4155 50  0001 C CNN
F 3 "~" H 14420 4155 50  0001 C CNN
	1    14420 4155
	1    0    0    -1  
$EndComp
Wire Wire Line
	10435 4245 10435 4055
Wire Wire Line
	14420 3855 14420 4005
Text GLabel 10435 4245 3    39   Input ~ 0
PWR_GD
Wire Notes Line width 10
	14320 3285 14320 5155
Wire Notes Line
	13675 5145 13680 5145
Wire Notes Line
	14540 5150 14555 5150
Text Notes 14370 3455 0    39   ~ 0
DRV_nFAULT: Fault\nIndicator and LED
Text Notes 13875 2260 0    39   ~ 0
5-V internal analog \nsupply regulator\nbypass capacitor
Text Notes 14590 2150 0    39   ~ 0
High Side MOSFET Drain
Wire Wire Line
	7120 5940 7120 6060
Text GLabel 7120 5940 1    39   Input ~ 0
SN3
$Comp
L Device:C C17
U 1 1 6132F188
P 7120 6210
F 0 "C17" H 7120 6285 50  0000 L CNN
F 1 "1nF" H 7145 6115 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7158 6060 50  0001 C CNN
F 3 "~" H 7120 6210 50  0001 C CNN
	1    7120 6210
	1    0    0    -1  
$EndComp
$Comp
L Device:C C16
U 1 1 6132E765
P 6320 6195
F 0 "C16" H 6320 6270 50  0000 L CNN
F 1 "1nF" H 6345 6100 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6358 6045 50  0001 C CNN
F 3 "~" H 6320 6195 50  0001 C CNN
	1    6320 6195
	1    0    0    -1  
$EndComp
Wire Notes Line width 10
	12565 2040 16065 2040
Wire Notes Line width 10
	12565 3275 16065 3275
Text Notes 12610 2275 0    39   ~ 0
DRV_VREG:  Internal\namplifier reference \nvoltage and SDO pullup.
$Comp
L CSD18532NQ5B:CSD18532NQ5B U4
U 1 1 62681882
P 5750 2730
F 0 "U4" V 6580 2530 60  0000 L CNN
F 1 "CSD18532NQ5B" H 6255 2305 60  0000 L CNN
F 2 "CSD18532NQ5B:CSD18532NQ5B" H 6550 2970 60  0001 C CNN
F 3 "" H 5750 2730 60  0000 C CNN
	1    5750 2730
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6935 1130 6990 1130
Wire Wire Line
	8255 2975 8330 2975
$Comp
L CSD18532NQ5B:CSD18532NQ5B U6
U 1 1 6271FB30
P 6990 2730
F 0 "U6" V 7820 2530 60  0000 L CNN
F 1 "CSD18532NQ5B" H 7495 2305 60  0000 L CNN
F 2 "CSD18532NQ5B:CSD18532NQ5B" H 7790 2970 60  0001 C CNN
F 3 "" H 6990 2730 60  0000 C CNN
	1    6990 2730
	0    -1   -1   0   
$EndComp
Connection ~ 6990 1130
Wire Wire Line
	6990 1130 7090 1130
Connection ~ 7090 1130
Wire Wire Line
	7090 1130 7190 1130
Connection ~ 7190 1130
Wire Wire Line
	6050 2730 6110 2730
Wire Wire Line
	6990 2730 7090 2730
Wire Wire Line
	7290 2730 7350 2730
Wire Wire Line
	8630 2755 8690 2755
Wire Wire Line
	8330 2755 8430 2755
Wire Wire Line
	8275 1155 8330 1155
Connection ~ 8330 1155
Wire Wire Line
	8330 1155 8430 1155
Connection ~ 8430 1155
Wire Wire Line
	8430 1155 8530 1155
Connection ~ 8530 1155
$Comp
L CSD18532NQ5B:CSD18532NQ5B U8
U 1 1 62706068
P 8330 2755
F 0 "U8" V 9160 2555 60  0000 L CNN
F 1 "CSD18532NQ5B" H 8835 2330 60  0000 L CNN
F 2 "CSD18532NQ5B:CSD18532NQ5B" H 9130 2995 60  0001 C CNN
F 3 "" H 8330 2755 60  0000 C CNN
	1    8330 2755
	0    -1   -1   0   
$EndComp
Connection ~ 8430 2755
Wire Wire Line
	8430 2755 8530 2755
Connection ~ 8430 2975
Wire Wire Line
	8430 2975 8530 2975
Wire Wire Line
	7090 2730 7190 2730
Connection ~ 7090 2730
Wire Wire Line
	5750 2730 5850 2730
Wire Wire Line
	5850 2730 5850 2845
Connection ~ 5850 2730
Wire Wire Line
	5850 2730 5950 2730
$Comp
L CSD18532NQ5B:CSD18532NQ5B U9
U 1 1 628C4F21
P 8330 4575
F 0 "U9" V 9160 4375 60  0000 L CNN
F 1 "CSD18532NQ5B" H 8835 4150 60  0000 L CNN
F 2 "CSD18532NQ5B:CSD18532NQ5B" H 9130 4815 60  0001 C CNN
F 3 "" H 8330 4575 60  0000 C CNN
	1    8330 4575
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8330 2975 8430 2975
Connection ~ 8330 2975
Wire Wire Line
	8530 2975 8630 2975
Connection ~ 8530 2975
Text Notes 4100 8400 0    59   ~ 0
Input Voltage-\n8V to 40V
Text Notes 745  8790 0    59   ~ 0
Output Voltage-\n3.3V 1A
Wire Wire Line
	14420 4470 14420 4365
Text GLabel 14845 3800 1    39   Input ~ 0
DRV_VREG
$Comp
L Device:R R7
U 1 1 614B52B7
P 14845 4450
F 0 "R7" H 14915 4496 50  0000 L CNN
F 1 "330" H 14905 4405 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 14775 4450 50  0001 C CNN
F 3 "~" H 14845 4450 50  0001 C CNN
	1    14845 4450
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D3
U 1 1 614C1AF2
P 14845 4055
F 0 "D3" V 14885 3975 50  0000 R CNN
F 1 "LED" V 14795 3975 50  0000 R CNN
F 2 "Diode_SMD:D_0603_1608Metric" H 14845 4055 50  0001 C CNN
F 3 "~" H 14845 4055 50  0001 C CNN
	1    14845 4055
	0    -1   -1   0   
$EndComp
Text GLabel 14845 4675 3    39   Input ~ 0
DRV_nFAULT
Wire Wire Line
	14845 4675 14845 4600
Wire Wire Line
	14845 3800 14845 3905
Wire Wire Line
	14845 4205 14845 4300
Text Notes 15110 3505 0    39   ~ 0
PWRGD: Power Good DRV_VREG \nand MCU watchdog  fault \nindication and LED
Wire Wire Line
	15255 4615 15255 4400
Text GLabel 15255 4615 3    39   Input ~ 0
PWR_GD
Wire Wire Line
	15255 3910 15255 4065
$Comp
L Device:R R9
U 1 1 623A8C63
P 15255 4215
F 0 "R9" H 15325 4261 50  0000 L CNN
F 1 "10k" H 15315 4170 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 15185 4215 50  0001 C CNN
F 3 "~" H 15255 4215 50  0001 C CNN
	1    15255 4215
	1    0    0    -1  
$EndComp
Text GLabel 15255 3910 1    39   Input ~ 0
DRV_VREG
Wire Notes Line width 10
	15070 3295 15070 5170
Text GLabel 4610 1370 1    39   Input ~ 0
3V3_IN
$Comp
L Device:R R5
U 1 1 6157BAD1
P 4610 1940
F 0 "R5" H 4680 1986 50  0000 L CNN
F 1 "10k" H 4670 1895 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4540 1940 50  0001 C CNN
F 3 "~" H 4610 1940 50  0001 C CNN
	1    4610 1940
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D2
U 1 1 6157BAD7
P 4610 1590
F 0 "D2" V 4649 1473 50  0000 R CNN
F 1 "LED" V 4558 1473 50  0000 R CNN
F 2 "Diode_SMD:D_0603_1608Metric" H 4610 1590 50  0001 C CNN
F 3 "~" H 4610 1590 50  0001 C CNN
	1    4610 1590
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4610 2145 4610 2090
Wire Wire Line
	4610 1370 4610 1440
Wire Wire Line
	4610 1740 4610 1790
$Comp
L power:GND #PWR012
U 1 1 615D1ABD
P 4610 2145
F 0 "#PWR012" H 4610 1895 50  0001 C CNN
F 1 "GND" H 4615 1972 50  0000 C CNN
F 2 "" H 4610 2145 50  0001 C CNN
F 3 "" H 4610 2145 50  0001 C CNN
	1    4610 2145
	1    0    0    -1  
$EndComp
Text Notes 4165 1070 0    59   ~ 0
3.3V Input Power LED\n
Wire Notes Line width 10
	2970 2405 2970 950 
Wire Notes Line width 10
	2970 945  5150 945 
Wire Notes Line width 10
	4130 960  4130 2420
Wire Notes Line
	4135 2405 4130 2405
Wire Notes Line width 10
	12560 495  12560 5120
Wire Notes Line width 10
	14520 3265 14520 2025
Wire Notes Line
	14085 2040 14080 2040
Text Notes 12615 675  0    39   ~ 0
PVDD: Power Bypass \nCapacitor and LED
Text Notes 7340 350  0    236  ~ 47
TitanLegs
$Comp
L Device:C C9
U 1 1 6162F8A6
P 6300 1270
F 0 "C9" V 6250 1325 50  0000 L CNN
F 1 "0.01uF" V 6415 1090 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6338 1120 50  0001 C CNN
F 3 "~" H 6300 1270 50  0001 C CNN
	1    6300 1270
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C8
U 1 1 617B5988
P 6300 1015
F 0 "C8" V 6415 1061 50  0000 L CNN
F 1 "1uF" V 6415 890 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6338 865 50  0001 C CNN
F 3 "~" H 6300 1015 50  0001 C CNN
	1    6300 1015
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6150 1015 6150 1130
Wire Wire Line
	5950 1130 6050 1130
Connection ~ 6050 1130
Wire Wire Line
	6050 1130 6150 1130
Connection ~ 6150 1130
Wire Wire Line
	6150 1130 6150 1270
Wire Wire Line
	6450 1015 6510 1015
Wire Wire Line
	6510 1015 6510 1270
Connection ~ 6510 1270
Wire Wire Line
	7675 1260 7735 1260
$Comp
L power:GND #PWR014
U 1 1 6185706E
P 7735 1260
F 0 "#PWR014" H 7735 1010 50  0001 C CNN
F 1 "GND" H 7740 1087 50  0000 C CNN
F 2 "" H 7735 1260 50  0001 C CNN
F 3 "" H 7735 1260 50  0001 C CNN
	1    7735 1260
	1    0    0    -1  
$EndComp
$Comp
L Device:C C11
U 1 1 61857074
P 7525 1260
F 0 "C11" V 7460 1300 50  0000 L CNN
F 1 "0.01uF" V 7640 1080 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7563 1110 50  0001 C CNN
F 3 "~" H 7525 1260 50  0001 C CNN
	1    7525 1260
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C10
U 1 1 6185707A
P 7525 1005
F 0 "C10" V 7640 1051 50  0000 L CNN
F 1 "1uF" V 7640 880 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7563 855 50  0001 C CNN
F 3 "~" H 7525 1005 50  0001 C CNN
	1    7525 1005
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7675 1005 7735 1005
Wire Wire Line
	7735 1005 7735 1260
Connection ~ 7735 1260
Wire Wire Line
	9020 1280 9080 1280
$Comp
L power:GND #PWR015
U 1 1 61866337
P 9080 1280
F 0 "#PWR015" H 9080 1030 50  0001 C CNN
F 1 "GND" H 9085 1107 50  0000 C CNN
F 2 "" H 9080 1280 50  0001 C CNN
F 3 "" H 9080 1280 50  0001 C CNN
	1    9080 1280
	1    0    0    -1  
$EndComp
$Comp
L Device:C C13
U 1 1 6186633D
P 8870 1280
F 0 "C13" V 8810 1330 50  0000 L CNN
F 1 "0.01uF" V 8985 1100 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 8908 1130 50  0001 C CNN
F 3 "~" H 8870 1280 50  0001 C CNN
	1    8870 1280
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C12
U 1 1 61866343
P 8870 1025
F 0 "C12" V 8985 1071 50  0000 L CNN
F 1 "1uF" V 8985 900 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 8908 875 50  0001 C CNN
F 3 "~" H 8870 1025 50  0001 C CNN
	1    8870 1025
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9020 1025 9080 1025
Wire Wire Line
	9080 1025 9080 1280
Connection ~ 9080 1280
Wire Wire Line
	8720 1025 8720 1155
Connection ~ 8720 1155
Wire Wire Line
	8720 1155 8720 1280
Wire Wire Line
	7375 1005 7375 1130
Connection ~ 7375 1130
Wire Wire Line
	7375 1130 7375 1260
Wire Wire Line
	7190 1130 7290 1130
Connection ~ 7290 1130
Wire Wire Line
	7290 1130 7375 1130
Wire Wire Line
	8530 1155 8630 1155
Connection ~ 8630 1155
Wire Wire Line
	8630 1155 8720 1155
Text GLabel 2290 3435 2    39   Input ~ 0
TEMP_DQ
Wire Wire Line
	2290 3435 2100 3435
NoConn ~ 900  2535
NoConn ~ 900  2635
NoConn ~ 900  2735
NoConn ~ 900  2835
NoConn ~ 900  2935
NoConn ~ 900  3035
NoConn ~ 2100 1735
NoConn ~ 2100 1535
$Comp
L DS18S20Z_:DS18S20Z+ U10
U 1 1 615F141B
P 11590 6505
F 0 "U10" H 11590 6972 50  0000 C CNN
F 1 "DS18S20Z+" H 11590 6881 50  0000 C CNN
F 2 "DS18S20Z_:DS18S20Z_" H 11590 6505 50  0001 L BNN
F 3 "" H 11590 6505 50  0001 L BNN
F 4 "3" H 11590 6505 50  0001 L BNN "PARTREV"
F 5 "IPC7351B" H 11590 6505 50  0001 L BNN "STANDARD"
F 6 "Maxim Integrated" H 11590 6505 50  0001 L BNN "MANUFACTURER"
	1    11590 6505
	1    0    0    -1  
$EndComp
Text GLabel 10700 6505 0    39   Input ~ 0
TEMP_DQ
Wire Wire Line
	10700 6505 10890 6505
Wire Wire Line
	12290 6305 12450 6305
Text GLabel 12450 6305 2    39   Input ~ 0
3V3_IN
$Comp
L power:GND #PWR026
U 1 1 6163356A
P 12365 6915
F 0 "#PWR026" H 12365 6665 50  0001 C CNN
F 1 "GND" V 12365 6725 50  0000 C CNN
F 2 "" H 12365 6915 50  0001 C CNN
F 3 "" H 12365 6915 50  0001 C CNN
	1    12365 6915
	1    0    0    -1  
$EndComp
Wire Wire Line
	12365 6705 12365 6915
Wire Wire Line
	12290 6705 12365 6705
Text Notes 12460 6865 0    59   ~ 0
DQ - 1 Wire Line \nCommunication Interface\n
Wire Notes Line width 20
	5145 9140 9370 9140
Text GLabel 9675 6815 3    39   Input ~ 0
TEMP_DQ
Wire Wire Line
	9675 6815 9675 6625
$Comp
L Device:R R10
U 1 1 616F04DA
P 9675 6475
F 0 "R10" H 9745 6521 50  0000 L CNN
F 1 "4.7k" H 9745 6430 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 9605 6475 50  0001 C CNN
F 3 "~" H 9675 6475 50  0001 C CNN
	1    9675 6475
	1    0    0    -1  
$EndComp
Text GLabel 9675 6245 1    39   Input ~ 0
3V3_IN
Wire Wire Line
	9675 6245 9675 6325
Text Notes 9415 5895 0    50   ~ 0
DQ - External Pull-Up
Wire Notes Line
	9415 5735 10310 5735
Wire Notes Line
	10310 5735 10310 7425
Text Notes 12385 6100 0    59   ~ 0
Connect VDD to GND for\nParasite Power Mode:\nDeriving Power from DQ\nData line
Text GLabel 14420 4470 3    39   Input ~ 0
DRV_nFAULT
$Comp
L Device:C C28
U 1 1 61786B03
P 14580 4575
F 0 "C28" H 14600 4655 50  0000 L CNN
F 1 "0.1uF" H 14590 4475 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 14618 4425 50  0001 C CNN
F 3 "~" H 14580 4575 50  0001 C CNN
	1    14580 4575
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR031
U 1 1 617D0F35
P 14580 4790
F 0 "#PWR031" H 14580 4540 50  0001 C CNN
F 1 "GND" H 14585 4617 50  0000 C CNN
F 2 "" H 14580 4790 50  0001 C CNN
F 3 "" H 14580 4790 50  0001 C CNN
	1    14580 4790
	1    0    0    -1  
$EndComp
Wire Wire Line
	14580 4725 14580 4790
Wire Wire Line
	14580 4425 14580 4365
Wire Wire Line
	14580 4365 14420 4365
Connection ~ 14420 4365
Wire Wire Line
	14420 4365 14420 4305
$Comp
L Device:C C29
U 1 1 61844D5E
P 15415 4565
F 0 "C29" H 15435 4645 50  0000 L CNN
F 1 "0.1uF" H 15425 4465 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 15453 4415 50  0001 C CNN
F 3 "~" H 15415 4565 50  0001 C CNN
	1    15415 4565
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR032
U 1 1 61844D64
P 15415 4745
F 0 "#PWR032" H 15415 4495 50  0001 C CNN
F 1 "GND" H 15420 4572 50  0000 C CNN
F 2 "" H 15415 4745 50  0001 C CNN
F 3 "" H 15415 4745 50  0001 C CNN
	1    15415 4745
	1    0    0    -1  
$EndComp
Wire Wire Line
	15415 4715 15415 4745
Wire Wire Line
	15415 4415 15415 4400
Wire Wire Line
	15415 4400 15255 4400
Connection ~ 15255 4400
Wire Wire Line
	15255 4400 15255 4365
Text Notes 14555 2875 0    39   ~ 0
VDRAIN Should \nHave Low\nImpedance Path\nTo Drain Of The \nHigh-Side FETs
Wire Notes Line width 10
	15345 2050 15345 3290
Wire Notes Line width 10
	14765 475  14765 2045
Text GLabel 13060 920  1    39   Input ~ 0
PVDD
$Comp
L Device:R R12
U 1 1 619F841E
P 13060 1475
F 0 "R12" H 13130 1521 50  0000 L CNN
F 1 "5k" H 13120 1430 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 12990 1475 50  0001 C CNN
F 3 "~" H 13060 1475 50  0001 C CNN
	1    13060 1475
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D4
U 1 1 619F8424
P 13060 1125
F 0 "D4" V 13100 1045 50  0000 R CNN
F 1 "LED" V 13010 1045 50  0000 R CNN
F 2 "Diode_SMD:D_0603_1608Metric" H 13060 1125 50  0001 C CNN
F 3 "~" H 13060 1125 50  0001 C CNN
	1    13060 1125
	0    -1   -1   0   
$EndComp
Wire Wire Line
	13060 920  13060 975 
Wire Wire Line
	13060 1275 13060 1325
$Comp
L power:GND #PWR028
U 1 1 61A5015E
P 13060 1705
F 0 "#PWR028" H 13060 1455 50  0001 C CNN
F 1 "GND" H 13065 1532 50  0000 C CNN
F 2 "" H 13060 1705 50  0001 C CNN
F 3 "" H 13060 1705 50  0001 C CNN
	1    13060 1705
	1    0    0    -1  
$EndComp
Wire Wire Line
	13060 1625 13060 1705
Text GLabel 15760 3865 1    39   Input ~ 0
DRV_VREG
$Comp
L Device:R R15
U 1 1 61ACF9CE
P 15760 4445
F 0 "R15" H 15830 4491 50  0000 L CNN
F 1 "330" H 15820 4400 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 15690 4445 50  0001 C CNN
F 3 "~" H 15760 4445 50  0001 C CNN
	1    15760 4445
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D5
U 1 1 61ACF9D4
P 15760 4110
F 0 "D5" V 15800 4030 50  0000 R CNN
F 1 "LED" V 15710 4030 50  0000 R CNN
F 2 "Diode_SMD:D_0603_1608Metric" H 15760 4110 50  0001 C CNN
F 3 "~" H 15760 4110 50  0001 C CNN
	1    15760 4110
	0    -1   -1   0   
$EndComp
Text GLabel 15760 4685 3    39   Input ~ 0
PWR_GD
Wire Wire Line
	15760 3865 15760 3960
Wire Wire Line
	15760 4260 15760 4295
Wire Wire Line
	15760 4685 15760 4595
Wire Notes Line width 10
	13835 3260 13835 2020
Text GLabel 13535 3990 1    39   Input ~ 0
SO2
$Comp
L Device:C C25
U 1 1 62076155
P 12935 4660
F 0 "C25" H 12945 4735 50  0000 L CNN
F 1 "2.2nF" H 12945 4570 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 12973 4510 50  0001 C CNN
F 3 "~" H 12935 4660 50  0001 C CNN
	1    12935 4660
	1    0    0    -1  
$EndComp
Wire Wire Line
	12935 4810 12935 4860
$Comp
L power:GND #PWR027
U 1 1 6207615D
P 12935 4860
F 0 "#PWR027" H 12935 4610 50  0001 C CNN
F 1 "GND" V 12935 4670 50  0000 C CNN
F 2 "" H 12935 4860 50  0001 C CNN
F 3 "" H 12935 4860 50  0001 C CNN
	1    12935 4860
	1    0    0    -1  
$EndComp
Text GLabel 14090 3990 1    39   Input ~ 0
SO3
NoConn ~ 2100 2235
Wire Wire Line
	13410 2660 13160 2660
Connection ~ 13410 2660
Text Notes 9470 4795 0    39   ~ 8
Operating Voltage for DRV8305 & MOSFETS:\n4.4 to 45 V (PVDD-Drain Voltage)\n
Text Notes 9475 4970 0    39   ~ 8
Peak Gate Driver Current:\n1 to 1.25 A
Text Notes 9505 1470 0    39   ~ 8
DRV_EN_GATE: Enables \ngate driver & current \nshunt amplifiers\n\nDVDD: 3.3V Internal\nVoltage Regulator\n\nAVDD: 5V Internal\nVoltage Regulator
Wire Wire Line
	5750 2965 5850 2965
Wire Wire Line
	5665 2965 5750 2965
Connection ~ 5750 2965
Wire Wire Line
	5950 2965 6050 2965
Wire Wire Line
	5850 2965 5950 2965
Connection ~ 5950 2965
$Comp
L CSD18532NQ5B:CSD18532NQ5B U5
U 1 1 6274500E
P 5750 4565
F 0 "U5" V 6580 4365 60  0000 L CNN
F 1 "CSD18532NQ5B" H 6255 4140 60  0000 L CNN
F 2 "CSD18532NQ5B:CSD18532NQ5B" H 6550 4805 60  0001 C CNN
F 3 "" H 5750 4565 60  0000 C CNN
	1    5750 4565
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5750 4635 5750 4565
Wire Wire Line
	5750 4635 5850 4635
Wire Wire Line
	5950 4635 5950 4565
Connection ~ 5750 4635
Wire Wire Line
	5850 4565 5850 4635
Connection ~ 5850 4635
Wire Wire Line
	5850 4635 5950 4635
Wire Wire Line
	6050 4565 6050 4635
Wire Wire Line
	6050 4635 6095 4635
Wire Wire Line
	6990 4660 6905 4660
Text GLabel 6905 4660 0    39   Input ~ 0
SLB
Text GLabel 7335 4660 2    39   Input ~ 0
GLB
Wire Wire Line
	6990 4660 6990 4590
Wire Wire Line
	6990 4660 7090 4660
Wire Wire Line
	7190 4660 7190 4590
Connection ~ 6990 4660
Wire Wire Line
	7090 4590 7090 4660
Connection ~ 7090 4660
Wire Wire Line
	7090 4660 7190 4660
Wire Wire Line
	7290 4590 7290 4660
Wire Wire Line
	7290 4660 7335 4660
Wire Wire Line
	8330 4645 8245 4645
Text GLabel 8245 4645 0    39   Input ~ 0
SLC
Text GLabel 8675 4645 2    39   Input ~ 0
GLC
Wire Wire Line
	8330 4645 8330 4575
Wire Wire Line
	8330 4645 8430 4645
Wire Wire Line
	8530 4645 8530 4575
Connection ~ 8330 4645
Wire Wire Line
	8430 4575 8430 4645
Connection ~ 8430 4645
Wire Wire Line
	8430 4645 8530 4645
Wire Wire Line
	8630 4575 8630 4645
Wire Wire Line
	8630 4645 8675 4645
Wire Wire Line
	6990 2990 7090 2990
Wire Wire Line
	6890 2990 6990 2990
Connection ~ 6990 2990
Wire Wire Line
	7190 2990 7290 2990
Wire Wire Line
	7090 2990 7190 2990
Connection ~ 7190 2990
$Comp
L CSD18532NQ5B:CSD18532NQ5B U7
U 1 1 62877531
P 6990 4590
F 0 "U7" V 7820 4390 60  0000 L CNN
F 1 "CSD18532NQ5B" H 7495 4165 60  0000 L CNN
F 2 "CSD18532NQ5B:CSD18532NQ5B" H 7790 4830 60  0001 C CNN
F 3 "" H 6990 4590 60  0000 C CNN
	1    6990 4590
	0    -1   -1   0   
$EndComp
Wire Notes Line
	12525 3810 14300 3810
Text GLabel 6320 5925 1    39   Input ~ 0
SN2
Wire Wire Line
	6320 5925 6320 6045
$Comp
L Device:R R13
U 1 1 618F710E
P 13535 4210
F 0 "R13" H 13605 4256 50  0000 L CNN
F 1 "5m" H 13605 4165 50  0000 L CNN
F 2 "" V 13465 4210 50  0001 C CNN
F 3 "~" H 13535 4210 50  0001 C CNN
	1    13535 4210
	1    0    0    -1  
$EndComp
$Comp
L Device:R R14
U 1 1 61909B63
P 14090 4210
F 0 "R14" H 14160 4256 50  0000 L CNN
F 1 "5m" H 14160 4165 50  0000 L CNN
F 2 "" V 14020 4210 50  0001 C CNN
F 3 "~" H 14090 4210 50  0001 C CNN
	1    14090 4210
	1    0    0    -1  
$EndComp
Text GLabel 13430 4430 0    39   Input ~ 0
ISENB
Text GLabel 13995 4430 0    39   Input ~ 0
ISENC
Text GLabel 2290 3635 2    39   Input ~ 0
ISENA
Wire Wire Line
	2290 3635 2100 3635
Text GLabel 710  1535 0    39   Input ~ 0
ISENB
Wire Wire Line
	710  1535 900  1535
Text GLabel 710  1635 0    39   Input ~ 0
ISENC
Wire Wire Line
	710  1635 900  1635
Text Notes 9940 7725 0    157  ~ 31
LiPo Battery Connector
$Comp
L power:GND #PWR019
U 1 1 616C32F7
P 8375 6285
F 0 "#PWR019" H 8375 6035 50  0001 C CNN
F 1 "GND" V 8380 6157 50  0000 R CNN
F 2 "" H 8375 6285 50  0001 C CNN
F 3 "" H 8375 6285 50  0001 C CNN
	1    8375 6285
	1    0    0    -1  
$EndComp
Text GLabel 8145 5980 1    39   Input ~ 0
SP1
Wire Wire Line
	8145 5980 8145 6195
Text GLabel 8375 5980 1    39   Input ~ 0
SP2
Wire Wire Line
	8375 5980 8375 6195
Text GLabel 8610 5980 1    39   Input ~ 0
SP3
Wire Wire Line
	8610 5980 8610 6195
Wire Wire Line
	8610 6195 8375 6195
Connection ~ 8375 6195
Wire Wire Line
	8375 6195 8145 6195
Wire Wire Line
	8375 6285 8375 6195
Text GLabel 940  4500 2    39   Input ~ 0
POWER_IN
Wire Wire Line
	770  4500 940  4500
Text GLabel 770  4500 0    39   Input ~ 0
PVDD
Wire Notes Line width 10
	1485 4250 1485 5145
Text Notes 570  5050 0    39   ~ 0
PVDD i.e input to the Driver\nand the board
Text GLabel 13160 2660 0    39   Input ~ 0
3V3_IN
Wire Notes Line width 20
	9415 7405 13640 7405
Wire Notes Line width 20
	13620 9135 9395 9135
Wire Notes Line width 20
	13645 5180 13645 9160
Wire Notes Line
	13640 9135 13625 9135
Wire Wire Line
	5750 4755 5750 4635
Text GLabel 5750 4755 3    39   Input ~ 0
SN1
Text GLabel 6990 4780 3    39   Input ~ 0
SN2
Wire Wire Line
	6990 4780 6990 4660
Wire Wire Line
	8330 4765 8330 4645
Text GLabel 8330 4765 3    39   Input ~ 0
SN3
Wire Wire Line
	13535 3990 13535 4060
Wire Wire Line
	14090 3990 14090 4060
Wire Wire Line
	12935 3985 12935 4055
Text GLabel 12825 4430 0    39   Input ~ 0
ISENA
$Comp
L Device:R R11
U 1 1 618F5060
P 12935 4205
F 0 "R11" H 13005 4251 50  0000 L CNN
F 1 "5m" H 13005 4160 50  0000 L CNN
F 2 "" V 12865 4205 50  0001 C CNN
F 3 "~" H 12935 4205 50  0001 C CNN
	1    12935 4205
	1    0    0    -1  
$EndComp
Text GLabel 12935 3985 1    39   Input ~ 0
SO1
Wire Wire Line
	12935 4355 12935 4430
$Comp
L power:GND #PWR029
U 1 1 61F2C1DA
P 13535 4865
F 0 "#PWR029" H 13535 4615 50  0001 C CNN
F 1 "GND" V 13535 4675 50  0000 C CNN
F 2 "" H 13535 4865 50  0001 C CNN
F 3 "" H 13535 4865 50  0001 C CNN
	1    13535 4865
	1    0    0    -1  
$EndComp
Wire Wire Line
	13535 4815 13535 4865
$Comp
L Device:C C26
U 1 1 61F02A13
P 13535 4665
F 0 "C26" H 13545 4740 50  0000 L CNN
F 1 "2.2nF" H 13545 4575 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 13573 4515 50  0001 C CNN
F 3 "~" H 13535 4665 50  0001 C CNN
	1    13535 4665
	1    0    0    -1  
$EndComp
Wire Wire Line
	13535 4360 13535 4430
$Comp
L power:GND #PWR030
U 1 1 62087210
P 14090 4870
F 0 "#PWR030" H 14090 4620 50  0001 C CNN
F 1 "GND" V 14090 4680 50  0000 C CNN
F 2 "" H 14090 4870 50  0001 C CNN
F 3 "" H 14090 4870 50  0001 C CNN
	1    14090 4870
	1    0    0    -1  
$EndComp
Wire Wire Line
	14090 4820 14090 4870
$Comp
L Device:C C27
U 1 1 62087208
P 14090 4670
F 0 "C27" H 14100 4745 50  0000 L CNN
F 1 "2.2nF" H 14100 4580 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 14128 4520 50  0001 C CNN
F 3 "~" H 14090 4670 50  0001 C CNN
	1    14090 4670
	1    0    0    -1  
$EndComp
Wire Wire Line
	14090 4360 14090 4430
Wire Wire Line
	12825 4430 12935 4430
Connection ~ 12935 4430
Wire Wire Line
	12935 4430 12935 4510
Wire Wire Line
	13430 4430 13535 4430
Connection ~ 13535 4430
Wire Wire Line
	13535 4430 13535 4515
Wire Wire Line
	13995 4430 14090 4430
Connection ~ 14090 4430
Wire Wire Line
	14090 4430 14090 4520
Text GLabel 5520 6465 3    39   Input ~ 0
SP1
Wire Wire Line
	5520 6345 5520 6465
$Comp
L Device:C C15
U 1 1 6132DCA3
P 5520 6195
F 0 "C15" H 5530 6270 50  0000 L CNN
F 1 "1nF" H 5535 6110 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5558 6045 50  0001 C CNN
F 3 "~" H 5520 6195 50  0001 C CNN
	1    5520 6195
	1    0    0    -1  
$EndComp
Text GLabel 5520 5925 1    39   Input ~ 0
SN1
Wire Wire Line
	5520 5925 5520 6045
Text GLabel 6310 2845 2    39   Input ~ 0
MOT_A
Wire Wire Line
	5850 2845 6310 2845
Connection ~ 5850 2845
Wire Wire Line
	5850 2845 5850 2965
Text GLabel 7550 2850 2    39   Input ~ 0
MOT_B
Wire Wire Line
	7090 2850 7550 2850
Connection ~ 7090 2850
Wire Wire Line
	7090 2850 7090 2730
Text GLabel 8890 2865 2    39   Input ~ 0
MOT_C
Wire Wire Line
	8430 2865 8890 2865
Connection ~ 8430 2865
Wire Wire Line
	8430 2865 8430 2755
$Comp
L Mechanical:MountingHole_Pad H1
U 1 1 616FE4B3
P 6100 8135
F 0 "H1" H 6200 8184 50  0000 L CNN
F 1 "MountingHole_Pad" H 6200 8093 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm" H 6100 8135 50  0001 C CNN
F 3 "~" H 6100 8135 50  0001 C CNN
	1    6100 8135
	1    0    0    -1  
$EndComp
Text GLabel 6100 8410 3    39   Input ~ 0
MOT_A
Wire Wire Line
	6100 8235 6100 8410
$Comp
L Mechanical:MountingHole_Pad H2
U 1 1 6173B6AB
P 6980 8135
F 0 "H2" H 7080 8184 50  0000 L CNN
F 1 "MountingHole_Pad" H 7080 8093 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm" H 6980 8135 50  0001 C CNN
F 3 "~" H 6980 8135 50  0001 C CNN
	1    6980 8135
	1    0    0    -1  
$EndComp
Text GLabel 6980 8410 3    39   Input ~ 0
MOT_B
Wire Wire Line
	6980 8235 6980 8410
$Comp
L Mechanical:MountingHole_Pad H3
U 1 1 6176C92B
P 7845 8135
F 0 "H3" H 7945 8184 50  0000 L CNN
F 1 "MountingHole_Pad" H 7945 8093 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm" H 7845 8135 50  0001 C CNN
F 3 "~" H 7845 8135 50  0001 C CNN
	1    7845 8135
	1    0    0    -1  
$EndComp
Text GLabel 7845 8410 3    39   Input ~ 0
MOT_C
Wire Wire Line
	7845 8235 7845 8410
Text Notes 5865 7720 0    157  ~ 31
Motor Connector Pads
Wire Notes Line width 21
	9375 7405 5155 7405
Wire Notes Line
	5155 7390 5155 7385
Text Notes 6000 5475 0    157  ~ 31
Additional Circuitry
Text Notes 3130 2640 0    118  ~ 24
Boot Reset Switches
Wire Wire Line
	11545 8255 11545 8430
Text GLabel 11545 8430 3    39   Input ~ 0
GND
$Comp
L Mechanical:MountingHole_Pad H5
U 1 1 61833E09
P 11545 8155
F 0 "H5" H 11645 8204 50  0000 L CNN
F 1 "MountingHole_Pad" H 11645 8113 50  0000 L CNN
F 2 "MountingHole:MountingHole_6mm" H 11545 8155 50  0001 C CNN
F 3 "~" H 11545 8155 50  0001 C CNN
	1    11545 8155
	1    0    0    -1  
$EndComp
Wire Wire Line
	10665 8255 10665 8430
Text GLabel 10665 8430 3    39   Input ~ 0
PVDD
$Comp
L Mechanical:MountingHole_Pad H4
U 1 1 61833E01
P 10665 8155
F 0 "H4" H 10765 8204 50  0000 L CNN
F 1 "MountingHole_Pad" H 10765 8113 50  0000 L CNN
F 2 "MountingHole:MountingHole_6mm" H 10665 8155 50  0001 C CNN
F 3 "~" H 10665 8155 50  0001 C CNN
	1    10665 8155
	1    0    0    -1  
$EndComp
$EndSCHEMATC
