#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <Wire.h>
//#include <SPI.h>
#include <errno.h>
#include <limits.h>
#include "LTC268x.h"
//#include "PCA9557.h"


#define MAX_MSG_LENGTH 1024
#define LEN(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))
#define ETHERNET
#define DELAY 10

/////digital pin on the VME Bus, is one is used please write it down on a comment here//////
#define D00 23
#define D01 22
#define D02 21
#define D03 24
#define D04 1
#define D05 14
#define D06 4
#define D07 15
#define D13 3
#define D14 5
#define D15 20



/* Base address for the PCA9557. This base address is modified by the three
 * least significant bits set by the DIP switches on each board. */
#define BASE_ADDR 0x18

/* Note: The DIP switches are backwards from what you'd think, i.e. if the
 * switches are in position 0,0,1, then you need to talk to address 0b100. I've
 * ordered the list here so they behave as expected, i.e. if the switches are
 * in position 0,0,1, then you should use bus[1]. */
#define MAXDACS 8

/* Which module boards are present. For the final setup this should be all
 * 1's. However when testing a small number of boards, we have no way to test
 * which boards are connected since the Wire library resets the whole Teensy if
 * it fails to communicate, so we need to manually keep track of which boards
 * are present. */
int rv=0;
int boardsactive[MAXDACS] = {
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0
};

 ltc268x *DAC[MAXDACS];

 ltc268x_device_id dacid = LTC2688;
 uint16_t PWDdacSett = 0x0000;
 uint16_t ditherToggleEN = false;
 bool ditherMode[16];
 ltc268x_voltage_range DACRange[16];
 ltc268x_dither_phase ditherPhase[16];
 ltc268x_dither_period ditherPeriod[16];
 ltc268x_clk_input clkIn[16];
 ltc268x_a_b_register ABReg[16];

  bool debug = false;
  char cmd[MAX_MSG_LENGTH];
  char err[MAX_MSG_LENGTH];
  char msg[MAX_MSG_LENGTH];
  int k = 0;

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = { 0xFA, 0xAA, 0xAA, 0xAA, 0xAD, 0xAC  };

unsigned int localPort = 8880;      // local port to listen on

// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  // buffer to hold incoming packet,

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;


int mystrtoi(const char *str, int *value)
{
    long lvalue;
    char *endptr;
    errno = 0;
    lvalue = strtol(str,&endptr,0);
    if (errno)
        return -1;
    if (endptr == str)
        return -1;
    if (lvalue > INT_MAX || lvalue < INT_MIN)
        return -1;
    *value = (int) lvalue;
    return 0;
}
double mystrtod(const char *nptr, double *value)
{
    char *endptr;
    errno = 0;
    *value = strtod(nptr,&endptr);

    if (endptr == nptr) {
        sprintf(err, "error converting '%s' to a double", nptr);
        return -1;
    } else if (errno != 0) {
        sprintf(err, "error converting '%s' to a double", nptr);
        return -1;
    }

    return 0;
}
int strtobool(const char *str, bool *value)
{
    if (!strcasecmp(str,"on") || !strcasecmp(str,"1"))
        *value = true;
    else if (!strcasecmp(str,"off") || !strcasecmp(str,"0"))
        *value = false;
    else
        return -1;
    return 0;
}

int reset(){
    
    //int rv = 0;
    uint8_t channel = 0;
    int ret;
    for (int i = 0; i < MAXDACS; i++) {
        if (!boardsactive[i]) {
            sprintf(err, "Skipping board %i because it's not active\n", i);
            Serial.print(err);
            continue;
        }

      for(int j=0; j<16; j++){
         ditherMode[j] = false;
         DACRange[j] = LTC268X_VOLTAGE_RANGE_M10V_10V;
         ditherPhase[j] = LTC268X_DITH_PHASE_0;
         ditherPeriod[j]= LTC268X_DITH_PERIOD_4;
         clkIn[j]= LTC268X_SOFT_TGL;
         ABReg[j]=LTC268X_SELECT_A_REG;
      }
      DAC[i] = new ltc268x(dacid, 
                          PWDdacSett,
                          ditherToggleEN,
                          ditherMode,
                          DACRange,
                          ditherPhase,
                          ditherPeriod,
                          clkIn,
                          ABReg,
                          BASE_ADDR+i,
                          &Wire,
                          i);

   
  }
  
  return 0;
}
int setDAC(int board, int ch, double dac_voltage){
  if (debug){
    Serial.print("setDAC ");
    Serial.print(board);Serial.print("\t");
    Serial.print(ch);Serial.print("\t");
    Serial.println(dac_voltage);
  }

  DAC[board]->set_voltage(ch, dac_voltage);

  return 0;
}


int do_command(char *cmd, float *value){
    int ntok = 0;
    char *tokens[10];
    char *tok;
    int channel, board ;
    double voltage;
    bool ison;
    
    if (cmd[strlen(cmd)-1] == '\n')
        cmd[strlen(cmd)-1] = '\0';

    if (debug) {
        sprintf(msg, "received command: %s\n", cmd);
        Serial.print(msg);
    }

    tok = strtok(cmd, " ");
    while (tok != NULL && ntok < (int) LEN(tokens)) {
        tokens[ntok++] = tok;
        tok = strtok(NULL, " ");
        //sprintf(msg, "tok:: %s\t", tok);
        //Serial.print(msg);
    }
    if (!strcmp(tokens[0], "SetDAC")) {
        if (ntok != 4) {
            sprintf(err, "SetDAC command expects 3 arguments: SetDac [board] [channel] [voltage]");
            return -1;
        }else if (mystrtoi(tokens[1],&board)) {
            sprintf(err, "expected argument 1 to be integer but got '%s'", tokens[1]);
            return -1;
        }else if (mystrtoi(tokens[2],&channel)) {
            sprintf(err, "expected argument 1 to be integer but got '%s'", tokens[1]);
            return -1;
        }else if (mystrtod(tokens[3],&voltage)) {
            sprintf(err, "expected argument 2 to be double but got '%s'", tokens[2]);
            return -1;
        }else  if ((voltage < -10) || (voltage > 10)) {
            sprintf(err, "voltage must be between -10V to 10V");
            return -1;
        }else if(setDAC(board, channel, voltage))return -1;
    
    }else if (!strcmp(tokens[0], "debug")) {
          if (ntok != 2) {
              sprintf(err, "debug command expects 1 argument: debug [on/off]");
              return -1;
          }
  
          if (strtobool(tokens[1],&ison)) {
              sprintf(err, "expected argument 1 to be yes/no but got '%s'", tokens[1]);
              return -1;
          }
  
          debug = ison;
    }else if (!strcmp(tokens[0], "help")) {
        sprintf(err,"SetDac [board] [channel] [voltage]\n");
                   
        return -1;
    }else {
        sprintf(err, "unknown command '%s'", tokens[0]);
        //sprintf(err, "error : message '%s'", cmd);
        return -1;
    }
 
    return 0;
}


void format_message(int rv, float value)
{
    //Serial.print("return from the func on f m: ");
    //Serial.println(rv);
    if (rv < 0) {
        sprintf(msg, "-%s\n", err);
    } else if (rv == 1) {
        sprintf(msg, ":%i\n", (int) value);
    } else if (rv == 2) {
        sprintf(msg, ",%.18f\n", value);
    } else {
        sprintf(msg, "+ok\n");
    }
}
void scanI2C(){
  byte error;
  int address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 24; address < 33; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
      for (int i = 0; i < MAXDACS; i++){
            //Serial.println(BASE_ADDR+i, HEX);
            if(address==BASE_ADDR+i){
                  boardsactive[i]=1;
                  //Serial.println(i);Serial.println(address);
                  }
      }
      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}

void setup()
{
  Serial.begin(9600);
  if (debug)Serial.println("init");
  Wire.begin();
  scanI2C();
//pinMode(18, INPUT_PULLUP);
//pinMode(19, INPUT_PULLUP);

#ifdef ETHERNET
    // start the Ethernet
    Ethernet.begin(mac);
  
    // Check for Ethernet hardware present
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
        Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
        while (true) {
            delay(1); // do nothing, no point running without Ethernet hardware
        }
    }
    if (debug)Serial.println("there is ethernet hardware");
    if (Ethernet.linkStatus() == LinkOFF) {
        Serial.println("Ethernet cable is not connected.");
    }else if (debug)Serial.println("cable connected");
  if (debug)Serial.println("start udp");
    // start UDP
    Udp.begin(localPort);
#endif

  
  pinMode(D01,OUTPUT);
  pinMode(D02,OUTPUT);
  pinMode(D03,OUTPUT);
  pinMode(D04,OUTPUT);
  pinMode(D05,OUTPUT);
  pinMode(D06,OUTPUT);
  pinMode(D07,OUTPUT);
  pinMode(D13,OUTPUT);
  pinMode(D14,OUTPUT);
  pinMode(D15,OUTPUT);

  reset();
  int board = 0;
 

}

void loop()
{
    float temp = 0;
  
    while (Serial.available() > 0) {
        if (k >= (int) LEN(cmd) - 1) {
            Serial.print("Error: too many characters in command!\n");
            k = 0;
        }
        cmd[k++] = Serial.read();
        if (cmd[k-1] == '\n') {
            cmd[k-1] = '\0';
            temp = 0;
            int rv = do_command(cmd, &temp);
            format_message(rv,temp);
            Serial.print(msg);
            k = 0;
        }
    }

#ifdef ETHERNET
    // if there's data available, read a packet
    int packetSize = Udp.parsePacket();
    //if (debug)Serial.print("read from LAN:  ");
    if (debug && packetSize)Serial.println(packetSize);
    if (packetSize) {
        if (packetBuffer[packetSize-1] == '\n')
            packetBuffer[packetSize-1] = '\0';

        packetBuffer[packetSize] = '\0';

        if (debug) {
            Serial.print("Received packet of size ");
            Serial.println(packetSize);
            Serial.print("From ");
            IPAddress remote = Udp.remoteIP();
            for (int i=0; i < 4; i++) {
                Serial.print(remote[i], DEC);
                if (i < 3) {
                    Serial.print(".");
                }
            }

            Serial.print(", port ");
            Serial.println(Udp.remotePort());
        }

        // read the packet into packetBufffer
        Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
        //Serial.println("Contents:");
        //Serial.println(packetBuffer);

        temp = 0;
        rv = do_command(packetBuffer, &temp);
        format_message(rv,temp);
        if(!temp){   
          Serial.print(msg);
        }

        // send a reply to the IP address and port that sent us the packet we received
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write(msg);
        Udp.endPacket();
    }
#endif
}
