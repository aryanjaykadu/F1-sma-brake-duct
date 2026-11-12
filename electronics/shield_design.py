"""Bench rig sensor shield for Arduino UNO R3: parts, footprints and nets (single source for schematic and PCB).

Pin map
  D13 SCK, D12 MISO        MAX31855 thermocouple modules x3 (TC1 spring, TC2 plenum air, TC3 reflector)
  D10 / D8 / D7            chip selects TC1 / TC2 / TC3
  A4 SDA, A5 SCL           AS5600 louver angle sensor (cable), 4.7k pull-ups
  D4 DT, D5 SCK            HX711 load-cell amplifier
  D9                       fan PWM (25 kHz, Timer1) via 1k
  D2                       fan tach (INT0), 10k pull-up
  D6                       heater SSR drive through 2N7000 low-side switch
  D3                       start / stop button to GND (internal pull-up)
  A2                       status LED
  A0                       MPXV7002DP pitot pressure module
12 V input feeds the duct fan and the spring-pack blower (Y-cable on J7: shared PWM, tach from the duct fan)
through a 3 A 1N5822 reverse-polarity diode; the logic runs from USB.
"""

R_FP = "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal"
PARTS = {
    # ref: (lib_id, value, footprint, {pin: net or None for no-connect})
    "A1": ("MCU_Module:Arduino_UNO_R3", "Arduino_UNO_R3", "Module:Arduino_UNO_R3_WithMountingHoles",
           {"1": None, "2": None, "3": None, "4": None, "5": "+5V", "6": "GND", "7": "GND", "8": None,
            "9": "PITOT_OUT", "10": None, "11": "LED_STATUS", "12": None, "13": "SDA", "14": "SCL",
            "15": None, "16": None, "17": "FAN_TACH", "18": "BTN", "19": "HX_DT", "20": "HX_SCK", "21": "SSR_PWM",
            "22": "CS_TC3", "23": "CS_TC2", "24": "FAN_PWM_MCU", "25": "CS_TC1", "26": None, "27": "MISO",
            "28": "SCK", "29": "GND", "30": None, "31": "SDA", "32": "SCL"}),
    "J1": ("Connector_Generic:Conn_01x05", "TC1 MAX31855 (spring)", "Connector_PinSocket_2.54mm:PinSocket_1x05_P2.54mm_Vertical",
           {"1": "+5V", "2": "GND", "3": "SCK", "4": "CS_TC1", "5": "MISO"}),
    "J2": ("Connector_Generic:Conn_01x05", "TC2 MAX31855 (air)", "Connector_PinSocket_2.54mm:PinSocket_1x05_P2.54mm_Vertical",
           {"1": "+5V", "2": "GND", "3": "SCK", "4": "CS_TC2", "5": "MISO"}),
    "J3": ("Connector_Generic:Conn_01x05", "TC3 MAX31855 (heater)", "Connector_PinSocket_2.54mm:PinSocket_1x05_P2.54mm_Vertical",
           {"1": "+5V", "2": "GND", "3": "SCK", "4": "CS_TC3", "5": "MISO"}),
    "J4": ("Connector_Generic:Conn_01x04", "HX711 load cell", "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical",
           {"1": "+5V", "2": "GND", "3": "HX_DT", "4": "HX_SCK"}),
    "J5": ("Connector_Generic:Conn_01x04", "AS5600 angle", "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical",
           {"1": "+5V", "2": "GND", "3": "SDA", "4": "SCL"}),
    "J6": ("Connector_Generic:Conn_01x03", "MPXV7002 pitot", "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical",
           {"1": "+5V", "2": "GND", "3": "PITOT_OUT"}),
    "J7": ("Connector_Generic:Conn_01x04", "Fan 4-pin", "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical",
           {"1": "GND", "2": "+12V", "3": "FAN_TACH", "4": "FAN_PWM"}),
    "J8": ("Connector_Generic:Conn_01x02", "12V in", "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-2_1x02_P5.00mm_Horizontal",
           {"1": "VIN_12V", "2": "GND"}),
    "J9": ("Connector_Generic:Conn_01x02", "Heater SSR input", "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-2_1x02_P5.00mm_Horizontal",
           {"1": "+5V", "2": "SSR_DRV"}),
    "R1": ("Device:R", "4k7", R_FP, {"1": "+5V", "2": "SDA"}),
    "R2": ("Device:R", "4k7", R_FP, {"1": "+5V", "2": "SCL"}),
    "R3": ("Device:R", "10k", R_FP, {"1": "+5V", "2": "FAN_TACH"}),
    "R4": ("Device:R", "1k", R_FP, {"1": "FAN_PWM_MCU", "2": "FAN_PWM"}),
    "R5": ("Device:R", "100", R_FP, {"1": "SSR_PWM", "2": "SSR_GATE"}),
    "R6": ("Device:R", "100k", R_FP, {"1": "SSR_GATE", "2": "GND"}),
    "R7": ("Device:R", "1k", R_FP, {"1": "LED_STATUS", "2": "LED_A"}),
    "C1": ("Device:C", "100n", "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm", {"1": "+5V", "2": "GND"}),
    "C2": ("Device:C", "100n", "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm", {"1": "+5V", "2": "GND"}),
    "C3": ("Device:C_Polarized", "47u", "Capacitor_THT:CP_Radial_D6.3mm_P2.50mm", {"1": "+5V", "2": "GND"}),
    "Q1": ("Transistor_FET:2N7000", "2N7000", "Package_TO_SOT_THT:TO-92_Inline",
           {"1": "GND", "2": "SSR_GATE", "3": "SSR_DRV"}),
    "D1": ("Diode:1N5822", "1N5822", "Diode_THT:D_DO-201AD_P5.08mm_Vertical_AnodeUp", {"1": "+12V", "2": "VIN_12V"}),
    "D2": ("Device:LED", "LED", "LED_THT:LED_D3.0mm", {"1": "GND", "2": "LED_A"}),
    "SW1": ("Switch:SW_Push", "START", "Button_Switch_THT:SW_PUSH_6mm", {"1": "BTN", "2": "GND"}),
}
POWER_NETS = {"+5V": "power:+5V", "GND": "power:GND", "+12V": "power:+12V"}
PWR_FLAG_NETS = ["GND", "+12V"]

