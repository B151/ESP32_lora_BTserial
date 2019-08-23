// LoRa receiver sending packets to serial port
// ESP32 BT communication 
// Added checksum calculation for ZINOO protocol
// Added $$ at the beginning of packet 
// LED on 16 blink if valid data
#include <string.h>
#include <ctype.h>
#include <SPI.h>
#include <BluetoothSerial.h> 
BluetoothSerial ESP_BT; //Object for Bluetooth

/*---------------------------------------------------*\
|                                                     |
|                                                     |
|               ESP32 21 - RFM DIO0                    |
|               ESP32 3 - RFM DIO5                    |
|               ESP32 5 - RFM NSS                     |
|               ESP32 23 - RFM MOSI                   |
|               ESP32 19 - RFM MISO                   |
|               ESP32 18 - RFM CLK                    |
|                                                     |
\*---------------------------------------------------*/


// RFM98
int data_LED = 16;
bool led_status;
int _slaveSelectPin = 5; 
String content = "";
char character;
int dio0 = 21;
// int dio5 = 9;
byte currentMode = 0x81;
unsigned long UpdateClientAt=0;
double Frequency=434.448;
int ImplicitOrExplicit;
int ErrorCoding;
int Bandwidth;
int SpreadingFactor;
int LowDataRateOptimize;
uint16_t CRC;

#define REG_FIFO                    0x00
#define REG_FIFO_ADDR_PTR           0x0D 
#define REG_FIFO_TX_BASE_AD         0x0E
#define REG_FIFO_RX_BASE_AD         0x0F
#define REG_RX_NB_BYTES             0x13
#define REG_OPMODE                  0x01
#define REG_FIFO_RX_CURRENT_ADDR    0x10
#define REG_IRQ_FLAGS               0x12
#define REG_PACKET_SNR              0x19
#define REG_PACKET_RSSI             0x1A
#define REG_RSSI_CURRENT            0x1B
#define REG_DIO_MAPPING_1           0x40
#define REG_DIO_MAPPING_2           0x41
#define REG_MODEM_CONFIG            0x1D
#define REG_MODEM_CONFIG2           0x1E
#define REG_MODEM_CONFIG3           0x26
#define REG_PAYLOAD_LENGTH          0x22
#define REG_IRQ_FLAGS_MASK          0x11
#define REG_HOP_PERIOD              0x24
#define REG_FREQ_ERROR              0x28
#define REG_DETECT_OPT              0x31
#define REG_DETECTION_THRESHOLD     0x37

// MODES
// MODES
#define RF96_MODE_RX_CONTINUOUS     0x85
#define RF96_MODE_SLEEP             0x80
#define RF96_MODE_STANDBY           0x81

#define PAYLOAD_LENGTH              80

// Modem Config 1
#define EXPLICIT_MODE               0x00
#define IMPLICIT_MODE               0x01

#define ERROR_CODING_4_5            0x02
#define ERROR_CODING_4_6            0x04
#define ERROR_CODING_4_7            0x06
#define ERROR_CODING_4_8            0x08

#define BANDWIDTH_7K8               0x00
#define BANDWIDTH_10K4              0x10
#define BANDWIDTH_15K6              0x20
#define BANDWIDTH_20K8              0x30
#define BANDWIDTH_31K25             0x40
#define BANDWIDTH_41K7              0x50
#define BANDWIDTH_62K5              0x60
#define BANDWIDTH_125K              0x70
#define BANDWIDTH_250K              0x80
#define BANDWIDTH_500K              0x90

// Modem Config 2

#define SPREADING_6                 0x60
#define SPREADING_7                 0x70
#define SPREADING_8                 0x80
#define SPREADING_9                 0x90
#define SPREADING_10                0xA0
#define SPREADING_11                0xB0
#define SPREADING_12                0xC0

#define CRC_OFF                     0x00
#define CRC_ON                      0x04


// POWER AMPLIFIER CONFIG
#define REG_PA_CONFIG               0x09
#define PA_MAX_BOOST                0x8F
#define PA_LOW_BOOST                0x81
#define PA_MED_BOOST                0x8A
#define PA_MAX_UK                   0x88
#define PA_OFF_BOOST                0x00
#define RFO_MIN                     0x00

// LOW NOISE AMPLIFIER
#define REG_LNA                     0x0C
#define LNA_MAX_GAIN                0x23  // 0010 0011
#define LNA_OFF_GAIN                0x00


char Hex[] = "0123456789ABCDEF";

void SetParametersFromLoRaMode(int LoRaMode)
{
  LowDataRateOptimize = 0;
  
  if (LoRaMode == 7)//zinoo
  {
    ImplicitOrExplicit = EXPLICIT_MODE;
    ErrorCoding = ERROR_CODING_4_5;
    Bandwidth = BANDWIDTH_125K;
    SpreadingFactor = SPREADING_10;
  }
  else if (LoRaMode == 6)
  {
    ImplicitOrExplicit = IMPLICIT_MODE;
    ErrorCoding = ERROR_CODING_4_5;
    Bandwidth = BANDWIDTH_41K7;
    SpreadingFactor = SPREADING_6;
  }
  else if (LoRaMode == 5)
  {
    ImplicitOrExplicit = EXPLICIT_MODE;
    ErrorCoding = ERROR_CODING_4_8;
    Bandwidth = BANDWIDTH_41K7;
    SpreadingFactor = SPREADING_11;
  }
  else if (LoRaMode == 4)
  {
    ImplicitOrExplicit = IMPLICIT_MODE;
    ErrorCoding = ERROR_CODING_4_5;
    Bandwidth = BANDWIDTH_250K;
    SpreadingFactor = SPREADING_6;
  }
  else if (LoRaMode == 3)
  {
    ImplicitOrExplicit = EXPLICIT_MODE;
    ErrorCoding = ERROR_CODING_4_6;
    Bandwidth = BANDWIDTH_250K;
    SpreadingFactor = SPREADING_7;
  }
  else if (LoRaMode == 2)
  {
    ImplicitOrExplicit = EXPLICIT_MODE;
    ErrorCoding = ERROR_CODING_4_8;
    Bandwidth = BANDWIDTH_62K5;
    SpreadingFactor = SPREADING_8;

  }
  else if (LoRaMode == 1)
  {
    ImplicitOrExplicit = IMPLICIT_MODE;
    ErrorCoding = ERROR_CODING_4_5;
    Bandwidth = BANDWIDTH_20K8;
    SpreadingFactor = SPREADING_6;

  }
 
  else if (LoRaMode == 0)
  {
   ImplicitOrExplicit = EXPLICIT_MODE;
    ErrorCoding = ERROR_CODING_4_8;
    Bandwidth = BANDWIDTH_20K8;
    SpreadingFactor = SPREADING_11;
    LowDataRateOptimize = 0x08;
  
    
  }
  
  
}

// initialize the library with the numbers of the interface pins

void setup()
{
  pinMode(data_LED, OUTPUT);
Serial.begin(57600); //   57600);
   ESP_BT.begin("ESP32_LoRa"); //Name of your Bluetooth Signal
  Serial.println("Bluetooth Device is Ready to Pair");

  Serial.println("");
  Serial.println("LoRa USB Receiver V1.2");
  Serial.println("");

  SetParametersFromLoRaMode(1);

  setupRFM98();
}

void loop()
{
 led_status=LOW; 
 digitalWrite (data_LED, led_status);
  CheckPC();
  
  CheckRx();
  
  UpdateClient();
}

void UpdateClient(void)
{
  if (millis() >= UpdateClientAt)
  {
    int CurrentRSSI;

    if (Frequency > 525)
    {
      CurrentRSSI = readRegister(REG_RSSI_CURRENT) - 157;
    }
    else
    {
      CurrentRSSI = readRegister(REG_RSSI_CURRENT) - 164;
    }
    
    ESP_BT.print("CurrentRSSI=");
    ESP_BT.println(CurrentRSSI);
    //Serial.print("CurrentRSSI=");
    //Serial.println(CurrentRSSI);
    
    UpdateClientAt = millis() + 1000;
  }
}

double FrequencyReference(void)
{
  switch (Bandwidth)
  {
    case  BANDWIDTH_7K8:  return 7800;
    case  BANDWIDTH_10K4:   return 10400; 
    case  BANDWIDTH_15K6:   return 15600; 
    case  BANDWIDTH_20K8:   return 20800; 
    case  BANDWIDTH_31K25:  return 31250; 
    case  BANDWIDTH_41K7:   return 41700; 
    case  BANDWIDTH_62K5:   return 62500; 
    case  BANDWIDTH_125K:   return 125000; 
    case  BANDWIDTH_250K:   return 250000; 
    case  BANDWIDTH_500K:   return 500000; 
  }
}


double FrequencyError(void)
{
  int32_t Temp;
  double T;
  
  Temp = (int32_t)readRegister(REG_FREQ_ERROR) & 7;
  Temp <<= 8L;
  Temp += (int32_t)readRegister(REG_FREQ_ERROR+1);
  Temp <<= 8L;
  Temp += (int32_t)readRegister(REG_FREQ_ERROR+2);
  
  if (readRegister(REG_FREQ_ERROR) & 8)
  {
    Temp = Temp - 524288;
  }

  T = (double)Temp;
  T *=  (16777216.0 / 32000000.0);
  T *= (FrequencyReference() / 500000.0);

  return -T;
} 

int receiveMessage(unsigned char *message)
{
  int i, Bytes, x, currentAddr;

  Bytes = 0;

  x = readRegister(REG_IRQ_FLAGS);
  // printf("Message status = %02Xh\n", x);
  
  // clear the rxDone flag
  // writeRegister(REG_IRQ_FLAGS, 0x40); 
  writeRegister(REG_IRQ_FLAGS, 0xFF); 
   
  // check for payload crc issues (0x20 is the bit we are looking for
  if((x & 0x20) == 0x20)
  {
    // printf("CRC Failure %02Xh!!\n", x);
    // reset the crc flags
    writeRegister(REG_IRQ_FLAGS, 0x20); 
  }
  else
  {
    currentAddr = readRegister(REG_FIFO_RX_CURRENT_ADDR);
    Bytes = readRegister(REG_RX_NB_BYTES);
    // printf ("%d bytes in packet\n", Bytes);

    // printf("RSSI = %d\n", readRegister(REG_RSSI) - 137);
	
    writeRegister(REG_FIFO_ADDR_PTR, currentAddr);   
    // now loop over the fifo getting the data
    for(i = 0; i < Bytes; i++)
    {
      message[i] = (unsigned char)readRegister(REG_FIFO);
    }
    message[Bytes] = '\0';

    // writeRegister(REG_FIFO_ADDR_PTR, 0);  // currentAddr);   
  } 
  
  return Bytes;
}

void ReplyOK(void)
{
  ESP_BT.println('*');
  //Serial.println('*');
}

void ReplyBad(void)
{
  ESP_BT.println('?');
  //Serial.println('?');
}

void SetFrequency(char *Line)
{
  double Freq;

  Freq = atof(Line);

  if (Frequency > 0)
  {
    ReplyOK();

    Frequency = Freq;
    
    ESP_BT.print("Frequency=");
    ESP_BT.println(Frequency);
  //  Serial.print("Frequency=");
   // Serial.println(Frequency);

    startReceiving();
  }
  else
  {
    ReplyBad();
  }
}

void SetMode(char *Line)
{
  int Mode;

  Mode = atoi(Line);

  if ((Mode >= 0) && (Mode <= 7))
  {
    ReplyOK();

    SetParametersFromLoRaMode(Mode);
    
    ESP_BT.print("Mode=");
    ESP_BT.println(Mode);
  // Serial.print("Mode=");
   // Serial.println(Mode);
    startReceiving();
  }
  else
  {
    ReplyBad();
  }
}

void SetBandwidth(char *Line)
{
  ESP_BT.print("BW len="); ESP_BT.print(strlen(Line)); ESP_BT.print(" <"); ESP_BT.print(Line); ESP_BT.println(">");
  /*Serial.print("BW len="); Serial.print(strlen(Line)); Serial.print(" <"); Serial.print(Line); Serial.println(">");*/
  if (strcmp(Line, "7K8") == 0)
  {
    ReplyOK();
    Bandwidth = BANDWIDTH_7K8;
    startReceiving();
  }
  else if (strcmp(Line, "10K4") == 0)
  {
    ReplyOK();
    Bandwidth = BANDWIDTH_10K4;
    startReceiving();
  }
  else if (strcmp(Line, "15K6") == 0)
  {
    ReplyOK();
    Bandwidth = BANDWIDTH_15K6;
    startReceiving();
  }
  else if (strcmp(Line, "20K8") == 0)
  {
    ReplyOK();
    Bandwidth = BANDWIDTH_20K8;
    startReceiving();
  }
  else if (strcmp(Line, "31K25") == 0)
  {
    ReplyOK();
    Bandwidth = BANDWIDTH_31K25;
    startReceiving();
  }
  else if (strcmp(Line, "41K7") == 0)
  {
    ReplyOK();
    Bandwidth = BANDWIDTH_41K7;
    startReceiving();
  }
  else if (strcmp(Line, "62K5") == 0)
  {
    ReplyOK();
    Bandwidth = BANDWIDTH_62K5;
    startReceiving();
  }
  else if (strcmp(Line, "125K") == 0)
  {
    ReplyOK();
    Bandwidth = BANDWIDTH_125K;
    startReceiving();
  }
  else if (strcmp(Line, "250K") == 0)
  {
    ReplyOK();
    Bandwidth = BANDWIDTH_250K;
    startReceiving();
  }
  else if (strcmp(Line, "500K") == 0)
  {
    ReplyOK();
    Bandwidth = BANDWIDTH_500K;
    startReceiving();
  }
  else
  {
    ReplyBad();
  }
}

void SetErrorCoding(char *Line)
{
  int Coding;

  Coding = atoi(Line);

  if ((Coding >= 5) && (Coding <= 8))
  {
    ReplyOK();
    ErrorCoding = (Coding-4) << 1;
    startReceiving();
  }
  else
  {
    ReplyBad();
  }
}

void SetSpreadingFactor(char *Line)
{
  int Spread;

  Spread = atoi(Line);

  if ((Spread >= 6) && (Spread <= 12))
  {
    ReplyOK();
    SpreadingFactor = Spread << 4;
    startReceiving();
  }
  else
  {
    ReplyBad();
  }
}

void SetImplicit(char *Line)
{
  int Implicit;

  Implicit = atoi(Line);

  ReplyOK();
  
  ImplicitOrExplicit = Implicit ? IMPLICIT_MODE : EXPLICIT_MODE;
  startReceiving();
}

void SetLowOpt(char *Line)
{
  int LowOpt;

  LowOpt = atoi(Line);

  ReplyOK();
  
  ImplicitOrExplicit = LowOpt ? 0x08 : 0;
  
  startReceiving();
}

void ProcessCommand(char *Line)
{
  char Command;

  Command = Line[1];
  Line += 2;
       
  if (Command == 'F')
  {
    SetFrequency(Line);
  }
  else if (Command == 'M')
  {
    SetMode(Line);
  }
  else if (Command == 'B')
  {
    SetBandwidth(Line);
  }
  else if (Command == 'E')
  {
    SetErrorCoding(Line);
  }
  else if (Command == 'S')
  {
    SetSpreadingFactor(Line);
  }
  else if (Command == 'I')
  {
    SetImplicit(Line);
  }
  else if (Command == 'L')
  {
    SetLowOpt(Line);
  }
  else
  {
    ReplyBad();
  }
}

void CheckPC()
{
  static char Line[32];
  static int Length=0;
  char Character;

  while (ESP_BT.available())
  { 
    Character = ESP_BT.read();
   //Character = Serial.read();
    
    if (Character == '~')
    {
      Line[0] = Character;
      Length = 1;
    }
    else if (Length >= sizeof(Line))
    {
      Length = 0;
    }
    else if (Length > 0)
    {
      if (Character == '\r')
      {
        Line[Length] = '\0';
        ProcessCommand(Line);
        Length = 0;
      }
      else
      {
        Line[Length++] = Character;
      }
    }
  }
}

void CheckRx()
{
  if (digitalRead(dio0))
  {
    unsigned char Message[256];
    int Bytes, SNR, RSSI, i;
    long Altitude;
    
    Bytes = receiveMessage(Message);
    
    ESP_BT.print("FreqErr="); ESP_BT.println(FrequencyError()/1000.0);
   // Serial.print("FreqErr="); Serial.println(FrequencyError()/1000.0);

    SNR = readRegister(REG_PACKET_SNR);
    SNR /= 4;
    
    if (Frequency > 525)
    {
      RSSI = readRegister(REG_PACKET_RSSI) - 157;
    }
    else
    {
      RSSI = readRegister(REG_PACKET_RSSI) - 164;
    }
    
    if (SNR < 0)
    {
      RSSI += SNR;
    }
    
    ESP_BT.print("PacketRSSI="); ESP_BT.println(RSSI);
    ESP_BT.print("PacketSNR="); ESP_BT.println(SNR);
   // Serial.print("PacketRSSI="); Serial.println(RSSI);
   // Serial.print("PacketSNR="); Serial.println(SNR);
    

    // ESP_BT.print("Packet size = "); ESP_BT.println(Bytes);

    // Telemetry='$$LORA1,108,20:30:39,51.95027,-2.54445,00141,0,0,11*9B74
    //CRC = 0xffff;           // Seed
    if (Message[0] == '$')
    //if (Message[0] == 0xFF)
    {
      ESP_BT.print("Message=");
      ESP_BT.println((char *)Message);
      //Serial.print("Message=");
      //Serial.println((char *)Message);
    }
    else if (Message[0] == '%')
    {
      char *ptr, *ptr2;

      Message[0] = '$';
      
      ptr = (char *)Message;
      do
      {
        if ((ptr2 = strchr(ptr, '\n')) != NULL)
        {
          *ptr2 = '\0';
          ESP_BT.print("Message=");
          ESP_BT.println(ptr);
         // Serial.print("Message=");
         // Serial.println(ptr);
          ptr = ptr2 + 1;
        }
      } while (ptr2 != NULL);
    }
    else
    {
      led_status=HIGH;
      digitalWrite (data_LED, led_status);
      ESP_BT.print("Message=$$");
      
      //Serial.print("Hex=");
      CRC = 0xFFFF;
      for (i=5; i<Bytes; i++)
      {
        if (Message[i] < 0x10)
        {
          ESP_BT.print("0");
         // Serial.print("0");
        } 
        ESP_BT.print(char(Message[i]));

        
        //----CRC
        //CRC = 0xffff;           // Seed
  //xPolynomial = 0x1021;
   int j;

    CRC = CRC ^ ((uint16_t)Message[i] << 8);;
    for (j=0; j<8; j++)
    {
      if (CRC & 0x8000)
        CRC = (CRC << 1) ^ 0x1021;
      else
        CRC <<= 1;
    }
  
   

}
        
       //Serial.print(Message[i], HEX);
     
       char chksum_str[8];
        sprintf(chksum_str, "*%04X\n", CRC);
       //ESP_BT.print(',') ;
        ESP_BT.print(chksum_str) ;
     // ESP_BT.println();
     // Serial.println();
    }
  }
}


/////////////////////////////////////
//    Method:   Change the mode
//////////////////////////////////////
void setMode(byte newMode)
{
  if(newMode == currentMode)
    return;  
  
  switch (newMode) 
  {
    case RF96_MODE_RX_CONTINUOUS:
      writeRegister(REG_PA_CONFIG, PA_OFF_BOOST);  // TURN PA OFF FOR RECIEVE??
      writeRegister(REG_LNA, LNA_MAX_GAIN);  // LNA_MAX_GAIN);  // MAX GAIN FOR RECIEVE
      writeRegister(REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
      
      break;
    case RF96_MODE_SLEEP:
      writeRegister(REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
    case RF96_MODE_STANDBY:
      writeRegister(REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
    default: return;
  } 
  
  if(newMode != RF96_MODE_SLEEP){
    //while(digitalRead(dio5) == 0)
    //{
      // 
    //} 
    delay(10);
  }
   
  return;
}


/////////////////////////////////////
//    Method:   Read Register
//////////////////////////////////////

byte readRegister(byte addr)
{
  select();
  SPI.transfer(addr & 0x7F);
  byte regval = SPI.transfer(0);
  unselect();
  return regval;
}

/////////////////////////////////////
//    Method:   Write Register
//////////////////////////////////////

void writeRegister(byte addr, byte value)
{
  select();
  SPI.transfer(addr | 0x80); // OR address with 10000000 to indicate write enable;
  SPI.transfer(value);
  unselect();
}

/////////////////////////////////////
//    Method:   Select Transceiver
//////////////////////////////////////
void select() 
{
  digitalWrite(_slaveSelectPin, LOW);
}

/////////////////////////////////////
//    Method:   UNSelect Transceiver
//////////////////////////////////////
void unselect() 
{
  digitalWrite(_slaveSelectPin, HIGH);
}

void SetLoRaFrequency()
{
  unsigned long FrequencyValue;
  double Temp;
  
  Temp = Frequency * 7110656 / 434;
  FrequencyValue = (unsigned long)(Temp);

  ESP_BT.print("FrequencyValue is ");
  ESP_BT.println(FrequencyValue);
 // Serial.print("FrequencyValue is ");
  //Serial.println(FrequencyValue);

  writeRegister(0x06, (FrequencyValue >> 16) & 0xFF);    // Set frequency
  writeRegister(0x07, (FrequencyValue >> 8) & 0xFF);
  writeRegister(0x08, FrequencyValue & 0xFF);
}

void SetLoRaParameters()
{
  writeRegister(REG_MODEM_CONFIG, ImplicitOrExplicit | ErrorCoding | Bandwidth);
  writeRegister(REG_MODEM_CONFIG2, SpreadingFactor | CRC_ON);
  writeRegister(REG_MODEM_CONFIG3, 0x04 | LowDataRateOptimize);                  // 0x04: AGC sets LNA gain
  writeRegister(REG_DETECT_OPT, (readRegister(REG_DETECT_OPT) & 0xF8) | ((SpreadingFactor == SPREADING_6) ? 0x05 : 0x03));  // 0x05 For SF6; 0x03 otherwise
  writeRegister(REG_DETECTION_THRESHOLD, (SpreadingFactor == SPREADING_6) ? 0x0C : 0x0A);    // 0x0C for SF6, 0x0A otherwise

}

void startReceiving()
{
  setMode(RF96_MODE_SLEEP);
  writeRegister(REG_OPMODE,0x80);  
  setMode(RF96_MODE_SLEEP);

  SetLoRaFrequency();
  
  SetLoRaParameters();
  
  writeRegister(REG_PAYLOAD_LENGTH, 255);
  writeRegister(REG_RX_NB_BYTES, 255);
  
  writeRegister(REG_FIFO_RX_BASE_AD, 0);
  writeRegister(REG_FIFO_ADDR_PTR, 0);
  
  // Setup Receive Continous Mode
  setMode(RF96_MODE_RX_CONTINUOUS);
}

void setupRFM98(void)
{
  // initialize the pins
  pinMode( _slaveSelectPin, OUTPUT);
  pinMode(dio0, INPUT);
  // pinMode(dio5, INPUT);

  SPI.begin();
  
  startReceiving();
  
  ESP_BT.println("Setup Complete");
 // Serial.println("Setup Complete");
}
