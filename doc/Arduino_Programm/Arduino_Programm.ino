#include <DynamixelSDK.h>
#include <IMU.h>

#define ADDR_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_OPERATING_MODE         11
#define ADDR_GOAL_TORQUE            102
#define ADDR_PRESENT_CURRENT        126
#define ADDR_VELOCITY_LIMIT         44
#define ADDR_GOAL_POSITION          116
#define ADDR_PRESENT_POSITION       132
#define ADDR_PRESENT_VELOCITY       128


// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                          1                   // Dynamixel ID: 1
#define BAUDRATE                        4500000
#define DEVICENAME                      "COM3"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0
#define CURRENT_MODE                    0       
#define VELOCITY_LIMIT                  1023
#define POSITION_MODE                   3
#define DXL_POS_THRESHOLD               0

#define SAMPLE_TIME                     3000
#define GRES                            2000.0/32768.0

#define K_X                             -8.6953
#define K_X_DOT                         -2.0932
#define K_X_PHI_DOT                     0 
#define K_Y                             -8.6953
#define K_Y_DOT                         -2.0932
#define K_Y_PHI_DOT                     0 


#define K1                              125*0.8 
#define K2                              132*0.8 
#define K3                              207*0.8 


#define ALPHA                           PI/4
#define BETA                            0
#define COS_ALPHA                       cos(ALPHA)       
#define SIN_ALPHA                       sin(ALPHA)
#define SIN_BETA                        sin(BETA)
#define COS_BETA                        cos(BETA)
#define SQRT3                           1.73205080757

//#define FILTER_ON
//#define FILTER_OFF
//#define REAL_TORQUES
//#define UNITS_COMPUTED
//#define PRESENT_UNITS
//#define VELOCITY
//#define GOAL_POS
//#define PHI_DOT
//#define EVALV_FILTER_ON_OFF
//#define UNITS_COMPUTED_VS_MEASURED
//#define VIRTUAL_TORQUES_VS_STATES



cIMU imu;
HardwareTimer Timer(TIMER_CH1);
dynamixel::PacketHandler *pHandler;
dynamixel::PortHandler *poHandler;

//Für Zeitmessung global definierte Variable
float time_duration = 0;
float time_counter = 0;



void setup()
{
    Serial.begin(115200);
    while (!Serial) {;}
    Serial.println("Init started");

    //Initialisiere IMU mit Update Frequenz von 200Hz
    imu.begin();
    
    //Initialisere Motoren mit den richtigen Modes
    motors_init(); 

    //Warten 3 Sekunden bis Timer gestartet wird
    delay(3000);

    //Initialisiere und starte Hardware-Timer für die Reglerberechnung
    Timer.stop();
    Timer.setPeriod(SAMPLE_TIME);
    Timer.attachInterrupt(executeController);
    Timer.start();

    Serial.println("Init finished");
}

void loop()
{
}

void executeController(void)
{
    //Aktualisieren der Sensordaten
    imu.update(); 
    
    // Starten Einlesen Daten
    readIMU();
}



void motors_init()
{
     dynamixel::PortHandler *portHandler =  dynamixel::PortHandler::getPortHandler(DEVICENAME);
     dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

     pHandler = packetHandler;
     poHandler = portHandler;

    int index = 0;
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
   

    uint8_t dxl_error = 0;                          // Dynamixel error
    int32_t dxl_present_position = 0;               // Present position

    // Öffnen Port
    if (portHandler->openPort())
    {
      Serial.print("Succeeded to open the port!\n");
    }
    else
    {
      Serial.print("Failed to open the port!\n");
      return;
    }

    // Baudrate setzen
    if (portHandler->setBaudRate(BAUDRATE))
    {
      Serial.print("Succeeded to change the baudrate!\n");
    }
    else
    {
      Serial.print("Failed to change the baudrate!\n");
      return;
    }

    // Ausschalten Motoren
    setTorque(TORQUE_DISABLE);
    
    // Festlegung: Motor auf Drehmomentregelung
    setMode(CURRENT_MODE);

 
    //Motoren einschalten
    setTorque(TORQUE_ENABLE);

    Serial.print("Dynamixel has been successfully connected \n");
}


//Funktion, die beim Interrupt aufgerufen wird und ausgeführt wird
//Sensordaten einlesen
void readIMU()
{
    // Anlegen von Arrays für Vergangenheitswerte
    static float* theta_x = new float[3];
    static float* theta_y = new float[3];
    static float* theta_x_dot = new float[3];
    static float* theta_y_dot = new float[3];
    
    float filter_theta_x = 0; 
    float filter_theta_y = 0; 
    float filter_theta_x_dot = 0; 
    float filter_theta_y_dot = 0; 

  
    //Aktuellen Wert einlesen mit festen Offset und umgewandelt in rad
    theta_x[0] = convert2radiand(imu.rpy[1]+2.6);//+0.04;
    theta_y[0] = convert2radiand(imu.rpy[0]-0.80);//-0.02;
    theta_x_dot[0] = convert2radiand(imu.gyroData[1]*GRES);
    theta_y_dot[0] = convert2radiand(imu.gyroData[0]*GRES);


    
    //Auslesen Sensorwerte ohne Filter
    #ifdef FILTER_OFF
          Serial.print(theta_x[0]*180/PI); Serial.print("\t");Serial.print(theta_y[0]*180/PI); Serial.print("\t");Serial.print(theta_x_dot[0]*180/PI); Serial.print("\t");Serial.print(theta_y_dot[0]*180/PI); Serial.print("\n");
    #endif

    //Filter mit 3 Vergangenheitswerte anlegen
    for(int i=0; i<3; i++)
    {
        filter_theta_x += theta_x[i];
        filter_theta_y += theta_y[i];
        filter_theta_x_dot += theta_x_dot[i];
        filter_theta_y_dot += theta_y_dot[i];
    }

    
    filter_theta_x = filter_theta_x/3;
    filter_theta_y = filter_theta_y/3;
    filter_theta_x_dot = filter_theta_x_dot/3;
    filter_theta_y_dot = filter_theta_y_dot/3;

    float theta[2] = {filter_theta_x, filter_theta_y};
    float theta_dot[2] = {filter_theta_x_dot, filter_theta_y_dot};

    //Shiften der Werte
    theta_x[2] = theta_x[1];
    theta_x[1] = theta_x[0];

    theta_y[2] = theta_y[1];
    theta_y[1] = theta_y[0];

    theta_x_dot[2] = theta_x_dot[1];
    theta_x_dot[1] = theta_x_dot[0];

    theta_y_dot[2] = theta_y_dot[1];
    theta_y_dot[1] = theta_y_dot[0];

    //Funktion zur Berechnng der Drehmomente
    controller(filter_theta_x,filter_theta_y,filter_theta_x_dot,filter_theta_y_dot);


    //Ausgabe der gefilterten Werte
    #ifdef FILTER_ON
        Serial.print(millis()/1000); Serial.print("\t");Serial.print(filter_theta_x*180/PI); Serial.print("\t");Serial.print(filter_theta_y*180/PI); Serial.print("\t");Serial.print(filter_theta_x_dot*180/PI); Serial.print("\t");  Serial.print(filter_theta_y_dot*180/PI); Serial.print("\n");
    #endif
}


//Ausführung des Regler 
void controller(float theta_x, float theta_y, float theta_x_dot, float theta_y_dot)
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    
    //Berechung virtuelle Drehmomente Tx,Ty
    static float* virtual_torques = new float[3];
    static float* real_torques = new float[3];  
    
    // Drehmoment in der yz Ebene --> T_x
    virtual_torques[0] = (theta_x * K_X  + theta_x_dot * K_X_DOT )*-1;
    
    //Drehmoment in der xz Ebene --> T_y
    virtual_torques[1] = (theta_y * K_Y + theta_y_dot * K_Y_DOT )*-1;

    //Drehmoment in der xy Ebene --> T_z
    virtual_torques[2] = 0.0;

    #ifdef VIRTUAL_TORQUES_VS_STATES
        Serial.print(time_counter); Serial.print("\t");Serial.print(theta_x*180/PI); Serial.print("\t");Serial.print(theta_y*180/PI); Serial.print("\t");Serial.print(theta_x_dot*180/PI); Serial.print("\t");  Serial.print(theta_y_dot*180/PI); Serial.print("\t");Serial.print(virtual_torques[0]); Serial.print("\t");Serial.print(virtual_torques[1]); Serial.print("\t");Serial.print(virtual_torques[2]); Serial.print("\n");
    #endif


    //Berechnung der realen Drehmomente
    //Drehmoment T1 
    real_torques[0] = 0.333333333 * (virtual_torques[2] + (2 / COS_ALPHA) * (virtual_torques[0] * COS_BETA - virtual_torques[1] * SIN_BETA));
    
    //Drehmoment T2
    real_torques[1] = 0.333333333 * (virtual_torques[2] + (1 / COS_ALPHA) * (SIN_BETA * (-virtual_torques[0] * SQRT3 + virtual_torques[1]) - COS_BETA * (virtual_torques[0] + SQRT3 * virtual_torques[1])));

    //Drehmoment T3
    real_torques[2] = 0.333333333 * (virtual_torques[2] + (1 / COS_ALPHA) * (SIN_BETA * (virtual_torques[0] * SQRT3 + virtual_torques[1]) + COS_BETA * (-virtual_torques[0] + SQRT3 * virtual_torques[1])));

    //Ausgabe reale Drehmomente
    #ifdef REAL_TORQUES
        Serial.print(theta_x*180/PI); Serial.print("\t");Serial.print(theta_y*180/PI); Serial.print("\t");Serial.print(theta_x_dot*180/PI); Serial.print("\t");  Serial.print(theta_y_dot*180/PI); Serial.print("\t");Serial.print(real_torques[0]); Serial.print("\t");Serial.print(real_torques[1]); Serial.print("\t");Serial.print(real_torques[2]); Serial.print("\n");
    #endif

    //Berechnung der entsprechenden Units
    int* current_units = compute2currentunits(real_torques);

    //Ausgabe der berechneteten Units
    #ifdef UNITS_COMPUTED
        Serial.print(current_units[0]); Serial.print("\t");Serial.print(current_units[1]); Serial.print("\t");Serial.print(current_units[2]); Serial.print("\n");
    #endif

    //Auslesen der aktuellen Units in den Motoren
//    int* present_units =  new int[3]; 
//
//    for(int i=1; i<=3; i++)
//    {
//      present_units[i-1] = readMotorData(i,2,ADDR_PRESENT_CURRENT);
//      if (present_units[i-1] > 1000){
//          present_units[i-1] = present_units[i-1] - 65536;
//      }
//       
//    }

//    //Ausgabe akutellen Units
//    #ifdef PRESENT_UNITS
//        Serial.print(present_units[0]); Serial.print("\t");Serial.print(present_units[1]); Serial.print("\t");Serial.print(present_units[2]); Serial.print("\n");
//    #endif

    
    

    //Drehmoment Offset
//    for (int i = 0; i < 3; i++)
//    {
//        if (i==1) {
//            if (current_units[i] > 0){
//                current_units[i] += 4;//18;
//            }
//            else {
//                current_units[i] -= 4;//18;
//            }
//        }
//    
//        else if (i==2) {
//            if (current_units[i] > 0){
//                current_units[i] += 6;//25;
//            }
//            else {
//                current_units[i] -= 6;//25;
//            }
//        }
//        else if (i==3) {
//            if (current_units[i] > 0){
//                current_units[i] += 4;//23;
//            }
//            else {
//                current_units[i] -= 4;//23;
//            }
//        }
//    }

// Drehmomentfilter ausgeschalten
//    static int* units_T1 = new int[3];
//    static int* units_T2 = new int[3]; 
//    static int* units_T3 = new int[3]; 
//
//    int* filtered_units = new int[3];
//    int filtered_T1 =0;
//    int filtered_T2 =0;
//    int filtered_T3 =0;
//    
//
//    units_T1[0] = current_units[0]; 
//    units_T2[0] = current_units[1];
//    units_T3[0] = current_units[2]; 
//
//    for(int i=1; i<=3; i++){
//      filtered_T1 =+ units_T1[i-1];
//      filtered_T2 =+ units_T2[i-1];
//      filtered_T3 =+ units_T3[i-1];   
//    }
//
//
//    filtered_units[0] = filtered_T1/3;
//    filtered_units[1] = filtered_T2/3;
//    filtered_units[2] = filtered_T3/3;
//
//    //Shiften der Vergangenheitswerte
//    units_T1[2] = units_T1[1];
//    units_T1[1] = units_T1[0];
//
//    units_T2[2] = units_T2[1];
//    units_T2[1] = units_T2[0];
//
//    units_T3[2] = units_T3[1];
//    units_T3[1] = units_T3[0];
    
    #ifdef UNITS_COMPUTED_VS_MEASURED
        Serial.print(time_counter); Serial.print("\t");Serial.print(filtered_units[0]); Serial.print("\t");Serial.print(filtered_units[1]); Serial.print("\t");Serial.print(filtered_units[2]); Serial.print("\t");Serial.print(present_units[0]); Serial.print("\t");Serial.print(present_units[1]); Serial.print("\t");Serial.print(present_units[2]); Serial.print("\n");
    #endif
   
    //Drehmomente/Units auf die Motoren geben
    //Warten, bis IMU Initialisierungsvorgang abgeschlossen
    if(millis()>30000){
      for(int i=1; i<=3; i++)
      {
          dxl_comm_result = writeMotorData(i, 2, ADDR_GOAL_TORQUE,current_units[i-1]);
      }
    }
}


//Umrechung zwischen realen Torques und Units
int *compute2currentunits(float real_torques_arr[]){
    static int* ret_arr = new int[3];

    ret_arr[0] = (int)(K1 * real_torques_arr[0]);
    ret_arr[1] = (int)(K2 * real_torques_arr[1]);
    ret_arr[2] = (int)(K3 * real_torques_arr[2]);

    return ret_arr;
}


//Umrechung Degree in Radiand
float convert2radiand(float val_deg){
    return (val_deg * PI) / 180;
}

//Torque der Motoren ein oder ausschalten
void setTorque(int mode){
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  uint8_t dxl_error = 0; 
  for(int i=1; i<=3; i++){
    dxl_comm_result = pHandler->write1ByteTxRx(poHandler, i, ADDR_TORQUE_ENABLE, mode, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      pHandler->printTxRxResult(dxl_comm_result);
      return;
     }
     else if (dxl_error != 0)
     {
       pHandler->printRxPacketError(dxl_error);
       return;
      }
    }
}

//Einstellen welcher Mode bei den Motoren eingestellt werden soll
void setMode(int mode){
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;
  
    for(int i=1; i<=3; i++)
    {
        dxl_comm_result = pHandler->write1ByteTxRx(poHandler, i, ADDR_OPERATING_MODE, mode, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            pHandler->printTxRxResult(dxl_comm_result);
            return;
        }
        else if (dxl_error != 0)
        {
            pHandler->printRxPacketError(dxl_error);
            return;
        }
    }
}

//Methode zum Auslesen von bestimmten Motordaten wie Geschwindigkeit....  
int readMotorData(uint8_t id, uint8_t length, uint16_t address){
    // turn on motors first to read out values of the motors!
    uint8_t  data1 = 0;
    uint16_t data2 = 0;
    uint32_t data3 = 0;
    uint8_t  dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    switch(length)
    {
        case 1:
              dxl_comm_result = pHandler->read1ByteTxRx(poHandler, id, address, (uint8_t*)&data1, &dxl_error);
              return data1;//return (int) data1;
              break;
        case 2:
              dxl_comm_result = pHandler->read2ByteTxRx(poHandler, id, address, (uint16_t*)&data2, &dxl_error);
              return data2;//return (int) data2;
              break;
        case 4:
              dxl_comm_result = pHandler->read4ByteTxRx(poHandler, id, address, (uint32_t*)&data3, &dxl_error);
              return data3;//return (int) data3;
              break;
    }
  
    if(dxl_comm_result != COMM_SUCCESS)
    {
        pHandler->printTxRxResult(dxl_comm_result);
    }
    else if(dxl_error != 0)
    {
        pHandler->printRxPacketError(dxl_error);
    }
  
    return dxl_comm_result; // this is just zero if no error is there!
}

//Methode zum Schreiben von bestimmten Motordaten wie Drehmoment
int writeMotorData(uint8_t id, uint8_t length, uint16_t address, int data)
{
    // turn on motors first to read out values of the motors!
    uint8_t  data1 = 0;
    uint16_t data2 = 0;
    uint32_t data3 = 0;
    uint8_t  dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
  
    switch(length)
    {
        case 1:
              dxl_comm_result = pHandler->write1ByteTxRx(poHandler, id, address, data, &dxl_error);
              break;
        case 2:
              dxl_comm_result = pHandler->write2ByteTxRx(poHandler, id, address, data, &dxl_error);
              break;
        case 4:
              dxl_comm_result = pHandler->write4ByteTxRx(poHandler, id, address, data, &dxl_error);
              break;
    }
  
    if(dxl_comm_result != COMM_SUCCESS)
    {
        pHandler->printTxRxResult(dxl_comm_result);
    }
    else if(dxl_error != 0)
    {
        pHandler->printRxPacketError(dxl_error);
    }
  
    return dxl_comm_result; // this is just zero if no error is there!
}



