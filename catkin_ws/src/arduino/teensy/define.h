//define header for AUV Teensy code

#define MOTOR_PIN_SW_BO 0 //T100_1 : bow_sway
#define MOTOR_PIN_SW_ST 1 //T100_2 : stern_sway
#define MOTOR_PIN_HE_BO 2 //T100_3 : port_bow_heave
#define MOTOR_PIN_HE_ST 3 //T100_4 : starboard_bow_heave
#define MOTOR_PIN_HE_PS 4 //T100_5 : port_stern_heave
#define MOTOR_PIN_HE_SS 5 //T100_6 : starboard_stern_heave

#define MOTOR_T100_RST_VALUE 1500

//Seabotix MOTOR
#define MOTOR_EN_PIN_SU_ST 6
#define MOTOR_EN_PIN_SU_PO 7
//
#define MOTOR_PIN_SU_ST 9 //SeabotimotorCommandValue_1 : port_surge
#define MOTOR_PIN_SU_PO 10 //SeabotimotorCommandValue_2 : starboard_surge
//
#define MOTOR_SEABOTIX_RST_VALUE 512
#define MOTOR_SEABOTICX_DEADBAND 25
//
#define STATUS_PIN_FAULT 8
#define STATUS_PIN_OTW 11


#define PWM_FREQUENCY 93750 //One of the Ideal Frequency of teensy3.1

//SOLENOID, one used pin for (Pin_33)
#define SOLENOID_PIN_D_1 12
#define SOLENOID_PIN_D_2 24
#define SOLENOID_PIN_G_1 25
#define SOLENOID_PIN_G_2 26
#define SOLENOID_PIN_T_1 27
#define SOLENOID_PIN_T_2 28

//ANALOG
#define VOLTAGE_PIN_1 A0
#define VOLTAGE_PIN_2 A1

//TIME INTERVAL(unit microsecond)
#define SOLENOID_TIMEOUT 100
#define MOTOR_TIMEOUT 500          //amount of no signal required to start to reset motors
#define TEMPERATURE_INTERVAL 1000   //amount of delay between each temperatures read
#define VOLTAGE_INTERVAL 500       //amount of delay between each voltages read
#define DEPTH_INTERVAL 20          //amount of delay between each depth read
#define MOTOR_STATUS_INTERVAL 500


//THRESHOLD for motor control
#define THRESHOLD_MOTOR 50
const double VOLT_RATIO = (3.3*30.9 * 24.12) / (3.9 * 1024.0 * 23.46); //(teensy voltage * total resistance / (single resisitance * mamotorCommandValue bit))


