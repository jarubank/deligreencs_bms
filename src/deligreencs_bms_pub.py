#!/usr/bin/env python

import rospy
from sensor_msgs.msg import BatteryState

import serial
import codecs


class BatteryState_Server:
    def __init__(self):
        self.pub_batt_state = rospy.Publisher("/battery_state", BatteryState, queue_size=10)

        if not rospy.has_param("~bms_port"):
            rospy.set_param("~bms_port", "/dev/deligreencs_bms")

        bms_port = rospy.get_param("~bms_port")

        self.ser = serial.Serial(  port=bms_port, baudrate=9600,  timeout=1,  parity=serial.PARITY_NONE, )

         # return [voltage_dec, current_dec, charge_dec, percentage_dec, temperature_dec, power_supply_status_dec, cell_voltage_dec]

        self.voltage_dec = 27.6
        self.current_dec = 0.0
        self.charge_dec = 150000
        self.temperature_dec = 25.0
        self.percentage_dec = 1.0
        self.power_supply_status_dec = 0
        self.cell_voltage_dec = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.counter_read_error = 0

        rate = rospy.Rate(1) # 1hz

        while not rospy.is_shutdown():
            self.update_state()
            rate.sleep()

    def read_bms(self):
         #####################################   data write ################################################    
        # # for soc and  voltage
        self.ser.write(bytearray.fromhex("A5 80 90 08 00 00 00 00 00 00 00 00 BD")) 
        output_90 = self.ser.readline()
        resp_90 = str(codecs.encode(output_90, "hex"))
 

        # # # # # for temperature
        self.ser.write(bytearray.fromhex("A5 80 92 08 00 00 00 00 00 00 00 00 BF")) 
        output_92 = self.ser.readline()
        resp_92 = str(codecs.encode(output_92,"hex"))
        
        # # # # # for capasity
        self.ser.write(bytearray.fromhex("A5 80 93 08 00 00 00 00 00 00 00 00 C0")) 
        output_93 = self.ser.readline()
        resp_93 = str(codecs.encode(output_93, "hex"))
      
        # for call Voltage
        self.ser.write(bytearray.fromhex("A5 40 95 08 00 00 00 00 00 00 00 00 82")) 
        output_95 = self.ser.readline()
        resp_95 = str(codecs.encode(output_95, "hex"))

        if len(resp_95) == 78 and len(resp_90) == 26 and len(resp_92) == 26 and len(resp_93) == 26:

            #####################################   data convert ################################################    

            voltage_hex = resp_90[8] + resp_90[9] + resp_90[10] + resp_90[11]
            self.voltage_dec = int(voltage_hex,16)*0.1  #*0.1 V
            # print "Voltage is :" ,voltage_dec
      
            current_hex = resp_90[16] + resp_90[17] + resp_90[18] + resp_90[19]
            current_int = int(current_hex,16)
            self.current_dec = (current_int -30000 )* -0.1      #3000 = 0.1 A
            # print"current is :" ,current_dec

            charge_hex =  resp_93[16] + resp_93[17] + resp_93[18] + resp_93[19] + resp_93[20] + resp_93[21] + resp_93[22] + resp_93[23]
            self.charge_dec= int(charge_hex, 16)   # unit mAH
            # print "charge is :" ,charge_dec
      
            percentage_hex = resp_90[20] + resp_90[21] + resp_90[22] + resp_90[23]
            self.percentage_dec  = int(percentage_hex, 16)*0.001   # *0.1 %
            # print "percentage is :" ,self.percentage_dec

            temperature_hex = resp_92[8] + resp_92[9]
            temperature_int = int(temperature_hex, 16)
            self.temperature_dec = temperature_int -64 + 24   #offset 40(hex = 64(dec)) = 24 degree celsius
            # print "temperature is :" ,temperature_dec

            power_supply_status_hex =  resp_93[9]
            self.power_supply_status_dec= power_supply_status_hex   # unit mAH
            # print "power_supply_status is:" ,power_supply_status_dec

            ##########################    cell voltage convert   #########################

            cell_1_hex =   resp_95[10] + resp_95[11] + resp_95[12] + resp_95[13]
            # print(cell_1_hex)
            cell_1= int(cell_1_hex, 16)*0.001   # unit mAH
            # print "cell_1 is :" ,cell_1
            cell_2_hex =  resp_95[14] + resp_95[15] + resp_95[16] + resp_95[17]
            # print(cell_2_hex)
            cell_2= int(cell_2_hex, 16)*0.001   # unit mAH
            # print "cell_2 is :" ,cell_2
            cell_3_hex =  resp_95[18] + resp_95[19] + resp_95[20] + resp_95[21]
            # print(cell_3_hex)
            cell_3= int(cell_3_hex, 16)*0.001   # unit mAH
            # print "cell_3 is :" ,cell_3
            cell_4_hex =  resp_95[36] + resp_95[37] + resp_95[38] + resp_95[39]
            # print(cell_4_hex)
            cell_4= int(cell_4_hex, 16)*0.001   # unit mAH
            # print "cell_4 is :" ,cell_4
            cell_5_hex =  resp_95[40] + resp_95[41] + resp_95[42] + resp_95[43]
            # print(cell_5_hex)
            cell_5= int(cell_5_hex, 16)*0.001   # unit mAH
            # print "cell_5 is :" ,cell_5
            cell_6_hex =  resp_95[44] + resp_95[45] + resp_95[46] + resp_95[47]
            # print(cell_6_hex)
            cell_6= int(cell_6_hex, 16)*0.001   # unit mAH
            # print "cell_6 is :" ,cell_6
            cell_7_hex =  resp_95[62] + resp_95[63] + resp_95[64] + resp_95[65]
            # print(cell_7_hex)
            cell_7= int(cell_7_hex, 16)*0.001   # unit mAH
            # print "cell_7 is :" ,cell_7
            cell_8_hex =  resp_95[66] + resp_95[67] + resp_95[68] + resp_95[69]
            # print(cell_8_hex)
            cell_8= int(cell_8_hex, 16)*0.001   # unit mAH
            # print "cell_8 is :" ,cell_8

            self.cell_voltage_dec = ( cell_1, cell_2, cell_3, cell_4, cell_5, cell_6, cell_7, cell_8 )
            # print "array cell voltage " , cell_voltage_dec

            self.counter_read_error = 0

        else:
            self.counter_read_error += 1
        
        if self.counter_read_error >= 10:
            rospy.err_once("[BMS] : cannot read data from Deligreen Smart BMS")

        return [self.voltage_dec, self.current_dec, self.charge_dec, self.percentage_dec, self.temperature_dec, self.power_supply_status_dec, self.cell_voltage_dec]
        


    
        
    def update_state(self):

        voltage_dec_, current_dec_, charge_dec_, percentage_dec_, temperature_dec_, power_supply_status_dec_, cell_voltage_dec_ = self.read_bms()

        battery_msg = BatteryState()

        # Power supply status constants
        # uint8 POWER_SUPPLY_STATUS_UNKNOWN = 0
        # uint8 POWER_SUPPLY_STATUS_CHARGING = 1
        # uint8 POWER_SUPPLY_STATUS_DISCHARGING = 2
        # uint8 POWER_SUPPLY_STATUS_NOT_CHARGING = 3
        # uint8 POWER_SUPPLY_STATUS_FULL = 4

        # Power supply health constants
        # uint8 POWER_SUPPLY_HEALTH_UNKNOWN = 0
        # uint8 POWER_SUPPLY_HEALTH_GOOD = 1
        # uint8 POWER_SUPPLY_HEALTH_OVERHEAT = 2
        # uint8 POWER_SUPPLY_HEALTH_DEAD = 3
        # uint8 POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4
        # uint8 POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5
        # uint8 POWER_SUPPLY_HEALTH_COLD = 6
        # uint8 POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7
        # uint8 POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = 8

        # Power supply technology (chemistry) constants
        # uint8 POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0
        # uint8 POWER_SUPPLY_TECHNOLOGY_NIMH = 1
        # uint8 POWER_SUPPLY_TECHNOLOGY_LION = 2
        # uint8 POWER_SUPPLY_TECHNOLOGY_LIPO = 3
        # uint8 POWER_SUPPLY_TECHNOLOGY_LIFE = 4
        # uint8 POWER_SUPPLY_TECHNOLOGY_NICD = 5
        # uint8 POWER_SUPPLY_TECHNOLOGY_LIMN = 6

        # Populate battery parameters.
        battery_msg.voltage = voltage_dec_                                                           # Voltage in Volts (Mandatory)
        battery_msg.current = current_dec_                                                          # Negative when discharging (A)  (If unmeasured NaN)
        battery_msg.charge  = charge_dec_                                                           # Current charge in Ah  (If unmeasured NaN)
        battery_msg.capacity = 150                                                          # Capacity in Ah (last full capacity)  (If unmeasured NaN)
        battery_msg.design_capacity = 150                                                   # Capacity in Ah (design capacity)  (If unmeasured NaN)
        battery_msg.percentage = percentage_dec_                                                      # Charge percentage on 0 to 1 range  (If unmeasured NaN)
        battery_msg.power_supply_status = int(power_supply_status_dec_)                                                 # The charging status as reported. Values defined above
        battery_msg.power_supply_health = 0                                                 # The battery health metric. Values defined above
        battery_msg.power_supply_technology = battery_msg.POWER_SUPPLY_TECHNOLOGY_LIFE      # The battery chemistry. Values defined above
        battery_msg.present = True                                                          # True if the battery is present
        battery_msg.cell_voltage = cell_voltage_dec_

        self.pub_batt_state.publish(battery_msg)

if __name__== '__main__':
    rospy.init_node('BMS_server')
    rospy.loginfo("Publishing data from BMS..")
    BatteryState_Server()
    rospy.spin()
