//***********************************************************************************************************************************************
//*******Code to use an ADF5355 as a signal generator from 52 to 13600 MHz based on that of DD7LP Christian Petersen for the ADF4351, (See 
//*******credits below). The code has been ported to run on a Maple Mini 32 bit ARM Cortex board. This was necessary to take advantage **********
//*******of the frequency resolution capabilities of the ADF5355. The display is an I2C OLED type. Brian Flynn GM8BJF. 1 November 2018 **********
//***********************************************************************************************************************************************
//***********************************************************************************************************************************************
// ****** Modifikation der Routinen vorgesehen für ADF4351 DDS bearbeitet von DD7LP Christian Petersen www.darc-husum.de im Februar    2017 *****
// *******Wesentliche Programmteile, Funktionen und Änderungen programmiert von DJ7OO Klaus Hirschelmann http://www.kh-gps.de/ im Februar 2017 **
//  ******Programmroutinen für das Ansprechen des ADF4351 aus einer Software von OE6OCG  Richard Posch 8302 Nestelbach 8452. Austria ************
//  ******Modifikation bezüglich des Kanalrasters. Diese wurden neu programmiert von OM Peter, DF6YL                     im Juli 2017************
//  ******Danke an OM Peter für diese Arbeit:                                 Anschrift Peter Beutel Verdistr. 2 59227 Ahlen         ************
// Achtung, kommerzielle Verwertung diese Software ist nicht gestattet bedarf der schriftlichen Zustimmmmung der Autoren, bzw Programmierer *****
//***********************************************************************************************************************************************

#include <SPI.h>
#include <RotaryEncoder.h>   //***************//
#include <Wire.h>            // I2C library //
#include <Adafruit_SSD1306.h>  // device driver for 128x64 SPI
#include <Adafruit_GFX.h>               //************** https://github.com/adafruit/Adafruit-GFX-Library

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1   // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

unsigned long currentTime;
unsigned long loopTime;
const int pin_A = 0;  // pin 0
const int pin_B = 1;  // pin 1
unsigned char encoder_A;
unsigned char encoder_B;
unsigned char encoder_A_prev = 0;

unsigned long loopTime2;
const int pin_A2 = 8;  // pin 8
const int pin_B2 = 9;  // pin 9
unsigned char encoder_A2;
unsigned char encoder_B2;
unsigned char encoder_A2_prev = 0;

boolean mrk1, mrk1_old, mrk2, mrk2_old;

int press = 0;
int cnt_step = 6;
int cnt_step_old;
int cnt_fix = 4;
int cnt_fix_old;
int cnt_pwr = 1;
int cnt_pwr_old;
int mdbm = 0;

const int slaveSelectPin = 7;  // SPI-SS bzw. enable ADF4350  wurde von mir von pin 3 auf pin 10 geändert

long Freq = 500000000;  // Start-up frequency = 5000 MHz
long Freq_Old;
long refin =  10000000;      // Referenzquarz = 100.000 Mhz - Enter exact value to calibrate (divide by 10)
long ChanStep = 10000;      // Initial channel step = 100.0 Khz  
unsigned long Reg[13];      // ADF5355 Reg's
int pegelZ = 0;            // Zähler konstante für Ausgangspegel-Zähler
int outPegel = mdbm;       // Ausgangspegel beim start = -4 dBm

///////////////////////// Subroutine: Set Frequency ADF5355 ///////////////////////////
void SetFreq(long Frequ)     // Freq hier lokal oben global
{
  
  Serial.begin(9600);    // USB to PC for Debug only
  Serial.println(" ");
  Serial.print("Frequuency = ");
  Serial.println(Frequ);


  //Initialisation sequence
  ConvertFreq(Reg);
  WriteADF2(12);
  delayMicroseconds(10);
  WriteADF2(11);
  delayMicroseconds(10);
  WriteADF2(10);
  delayMicroseconds(10);
  WriteADF2(9);
  delayMicroseconds(10);
  WriteADF2(8);
  delayMicroseconds(10);
  WriteADF2(7);
  delayMicroseconds(10);
  WriteADF2(6);
  delayMicroseconds(10);
  WriteADF2(5);
  delayMicroseconds(10);
  WriteADF2(4);
  delayMicroseconds(10);
  WriteADF2(3);
  delayMicroseconds(10);
  WriteADF2(2);
  delayMicroseconds(10);
  WriteADF2(1);
  delayMicroseconds(200);
  WriteADF2(0);  
}

////////////////////////// Part-Subroutine ADF5355 ////////////////////////////
void WriteADF2(int idx)
{ // make 4 byte from integer for SPI-Transfer
  byte buf[4];
  for (int i = 0; i < 4; i++)
    buf[i] = (byte)(Reg[idx] >> (i * 8));
  WriteADF(buf[3], buf[2], buf[1], buf[0]);
}

/////////////////////////// Subroutine ADF5355 ////////////////////////////
int WriteADF(byte a1, byte a2, byte a3, byte a4) {
  // write over SPI to ADF5355
  digitalWrite(slaveSelectPin, LOW);
  SPI.beginTransaction(SPISettings(100000000, MSBFIRST, SPI_MODE0));   //  Speed (e.g.10000000=10MHz), BitOrder (MSBFIRST), Mode (SPI_MODE0 - SPI_MODE3
  delayMicroseconds(1);
  SPI.transfer(a1);
  SPI.transfer(a2);
  SPI.transfer(a3);
  SPI.transfer(a4);
  Toggle();
}

///////////////////////////// Sub-Subroutine ADF5355 ////////////////////////////
int Toggle() {
  digitalWrite(slaveSelectPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(slaveSelectPin, LOW);
}

////////////////////////////// Sub-Subroutine ADF5355 //////////////////////////
void ConvertFreq(unsigned long R[])
////  Declare variables for registers/////
{
  // PLL-Reg-R0         =  32bit
  //   Registerselect        4bit
   //  int N_Int = 92;       // 16bit
  int Prescal = 0;         // 1bit geht nicht ??? it does not work
  int Autocal = 1;          //1 bit
   //  reserved           // 10bit

  // PLL-Reg-R1         =  32bit
  //   Registerselect        4bit
  //   int FRAC1 = 10;       // 24 bit
  //   reserved              // 4bit

  // PLL-Reg-R2         =  32bit
  //    Registerselect        4bit
      int M_Mod2 = 16383;            // 14 bit
  //    int Frac2 = 0;            // 14 bit
 

  // PLL-Reg-R3         =  32bit - FIXED !
  // Registerselect        4bit
  //Fixed value to be written = 0x3 =3
 

  // PLL-Reg-R4         =  32bit
  // Registerselect        4bit
    int U1_CountRes = 0;     // 1bit
    int U2_Cp3state = 0;     // 1bit
    int U3_PwrDown = 0;      // 1bit
    int U4_PDpola = 1;       // 1bit
    int U5_MuxLog = 1;          // 1bit
    int U6_RefMode = 1;          // 1bit 
  //  int U5_LPD = 0;          // 1bit
  //  int U6_LPF = 1;          // 1bit 1=Integer, 0=Frac not spported yet
    int CP_ChgPump = 9;      // 4bit
    int D1_DoublBuf = 0;     // 1bit
    int R_Counter = 1;       // 10bit 
    int RD1_Rdiv2 = 0;       // 1bit 
    int RD2refdoubl = 0;     // 1bit
    int M_Muxout = 6;        // 3bit
  // reserved              // 2bit
  
 

  // PLL-Reg-R5         =  32bit
  // Registerselect        // 4bit
  // Phase Select: Not of partcular interst in Amatuer radio applications. Leave at a string of zeros.
  
  // PLL-Reg-R6         =  32bit
  // Registerselect        // 4bit
  //Variable value to be written!!!
  int D_out_PWR = mdbm;      // 2bit  OutPwr 0-3 3= +5dBm   Power out 1
  int D_RF_ena = 1;            // 1bit  OutPwr 1=on           0 = off  Outport Null freischalten
  int Reserved  = 0;                 // 3bit
  int D_RFoutB = 1;         // 1bit  aux OutSel
  int D_MTLD = 0;              // 1bit
  int CPBleed = 126;   // 8bit
  int D_RfDivSel = 3;      // 3bit 3=70cm 4=2m    lokale Variable
  int D_FeedBack = 1;       // 1bit
  // reserved              // 7bit

  // PLL-Reg-R7         =  32bit
  // Registerselect        // 4bit
  //Fixed value to be written = 0x120000E7 = 301990119 (dec)

  // PLL-Reg-R8         =  32bit
  // Registerselect        // 4bit
  //Fixed value to be written = 0x102D0428 = 271385640 (dec)

  // PLL-Reg-R9         =  32bit
  // Registerselect        // 4bit
  //Fixed value to be written = 0x5047CC9 = 84180169 (dec)

  // PLL-Reg-R10         =  32bit
  // Registerselect        // 4bit
  //Fixed value to be written = 0xC0067A = 12584570 9dec)

  // PLL-Reg-R11         =  32bit
  // Registerselect        // 4bit
  //Fixed value to be written = 0x61300B = 6369291 (dec)

  // PLL-Reg-R12         =  32bit
  // Registerselect        // 4bit
  //Fixed value to be written = 0x1041C = 66588 (dec)

  // Referenz Freg Calc






  // int F4_BandSel = 10.0 * B_BandSelClk / PFDFreq;
  double RFout = Freq;       // VCO-Frequenz  144200000  Freq ist global, RFout ist lokal

  // calc bandselect und RF-div
  float outdiv = 1;
  if (RFout >= 680000000) {
    outdiv = 0.5;
    D_RfDivSel = 0;
    D_RFoutB = 0;
    D_RF_ena = 0;
  }
  if (RFout < 680000000) {
    outdiv = 1;
    D_RfDivSel = 0;
    D_RFoutB = 1;
    D_RF_ena = 1;
  }
  if (RFout < 340000000) {
    outdiv = 2;
    D_RfDivSel = 1;
    D_RFoutB = 1;
    D_RF_ena = 1;
  }
  if (RFout < 170000000) {
    outdiv = 4;
    D_RfDivSel = 2;
    D_RFoutB = 1;
    D_RF_ena = 1;
  }
  if (RFout < 85000000) {
    outdiv = 8;
    D_RfDivSel = 3;
    D_RFoutB = 1;
    D_RF_ena = 1;
  }
  if (RFout < 42500000) {
    outdiv = 16;
    D_RfDivSel = 4;
    D_RFoutB = 1;
    D_RF_ena = 1;
  }
  if (RFout < 21250000) {
    outdiv = 32;
    D_RfDivSel = 5;
    D_RFoutB = 1;
    D_RF_ena = 1;
  }
  if (RFout < 10625000) {
    outdiv = 64;
    D_RfDivSel = 6;
    D_RFoutB = 1;
    D_RF_ena = 1;
  }
  
/////////////////////////////////////////////////////////////////////////////
//////////////////////// N and Frac1 and Frac2 calculations /////////////////
//////////////////////// Done using double precision 64 bit /////////////////
//////////////////////// Results agree exactly with AD demo /////////////////
/////////////////////////////////////////////////////////////////////////////

   double PFDFreq = refin * ((1.0 + RD2refdoubl) / (R_Counter * (1.0 + RD1_Rdiv2))); //Phase detector frequency

   double N = ((RFout) * outdiv) / PFDFreq;   // Calculate N
  
   int N_Int = N;   // N= 50 for 5 GHz   // Turn N into integer

   double F_Frac1x = (N - N_Int) * pow(2, 24);   // Calculate Frac1 (N remainder * 2^24)
   
   int F_FracN = F_Frac1x;  // turn Frac1 into an integer
   
   double F_Frac2x = ((F_Frac1x - F_FracN)) * pow(2, 14);  // Claculate Frac2 (F_FracN remainder * 2^14)
   
   int F_Frac1 =   F_Frac1x;  // turn Frac1 into integer
   int F_Frac2 =   F_Frac2x;  // turn Frac2 into integer
   

   
////////////////// Set 32 bit register values R0 to R12 ///////////////////////////
   
    R[0] = (unsigned long)(0 + N_Int * pow(2, 4) + Prescal * pow(2, 20) + Autocal * pow(2,21)); // R0 für Startfrequenz ok
  
    R[1]=(unsigned long)(1 + F_Frac1 * pow(2, 4));
    
    R[2] = (unsigned long)(2 + M_Mod2 * pow(2, 4) + F_Frac2 * pow(2, 18)); // 
  
    R[3] = (unsigned long)(0x3);  //Fixed value (Phase control not needed)
  
    R[4] = (unsigned long)(4 + U1_CountRes * pow(2, 4) + U2_Cp3state * pow(2, 5) + U3_PwrDown * pow(2, 6) + U4_PDpola * pow(2, 7) + U5_MuxLog * pow(2, 8) + U6_RefMode * pow(2, 9) + CP_ChgPump * pow(2, 10) + D1_DoublBuf * pow(2, 14) + R_Counter * pow(2, 15) + RD1_Rdiv2 * pow(2, 25) + RD2refdoubl * pow(2, 26) + M_Muxout * pow(2, 27));
   
    R[5] = (unsigned long) (0x800025); // Fixed (Reserved)

    R[6] = (unsigned long)(6 + D_out_PWR * pow(2, 4) + D_RF_ena * pow(2, 6) + Reserved * pow(2, 7) + D_RFoutB * pow(2, 10) + D_MTLD * pow(2, 11) + Reserved * pow(2, 12) + CPBleed * pow(2, 13) +  D_RfDivSel * pow(2, 21) + D_FeedBack * pow(2, 24) +10 * pow(2, 25));
  
    R[7] = (unsigned long) (0x120000E7);
  R[8] = (unsigned long) (0x102D0428);
  R[9] = (unsigned long) (0x2A29FCC9);
  R[10] = (unsigned long) (0xC0043A);
  R[11] = (unsigned long) (0x61300B);
  R[12] = (unsigned long) (0x1041C);

  /////////////// Serial print values to terminal for diagnostics - Comment out in normal operation ///////////////
 
    Serial.print("Inhalt R0 ");
    Serial.println(R[0], HEX);
    Serial.print("Inhalt R1 ");
    Serial.println(R[1], HEX);
    Serial.print("Inhalt R2 ");
    Serial.println(R[2], HEX);
    Serial.print("Inhalt R3 ");
    /*
    SerialUSB.println(R[3], HEX);
    SerialUSB.print("Inhalt R4 ");
    SerialUSB.println(R[4], HEX);
    SerialUSB.print("Inhalt R5 ");
    SerialUSB.println(R[5], HEX);
    SerialUSB.print("Inhalt R6 ");
    SerialUSB.println(R[6], HEX);
    SerialUSB.print("Inhalt R7 ");
    SerialUSB.println(R[7], HEX);
    SerialUSB.print("Inhalt R8 ");
    SerialUSB.println(R[8], HEX);
    SerialUSB.print("Inhalt R9 ");
    SerialUSB.println(R[9], HEX);
    SerialUSB.print("Inhalt R10 ");
    SerialUSB.println(R[10], HEX);
    SerialUSB.print("Inhalt R11 ");
    SerialUSB.println(R[11], HEX);
    SerialUSB.print("Inhalt R12 ");
    SerialUSB.println(R[12], HEX);

    SerialUSB.println(" ");
    SerialUSB.print("PFDFreq = ");
    SerialUSB.println(PFDFreq,  8);
    SerialUSB.print("RFout = ");
    SerialUSB.println(RFout,  10);
    SerialUSB.print("N = ");
    SerialUSB.println(N,  20);
    SerialUSB.print("N_Int = ");
    SerialUSB.println(N_Int);
//    SerialUSB.print("x = ");
//    SerialUSB.println(x, 20);
//    SerialUSB.print("F_Fracx = ");
//    SerialUSB.println(F_Fracx, 20);
    SerialUSB.print("N - N_Int = ");
    SerialUSB.println(N - N_Int, 20);
    SerialUSB.print("F_Frac1 = ");
    SerialUSB.println(F_Frac1, DEC);
    SerialUSB.print("F_Frac2 = ");
    SerialUSB.println(F_Frac2, DEC);
    SerialUSB.println(" ");
    
    SerialUSB.print("PDFFreq = ");
    SerialUSB.println(PFDFreq);
 */   

}

//////////////////////////////////////////////////////////////////////////////
//                                      Setup                               //
//////////////////////////////////////////////////////////////////////////////
void setup() {

  // ******************Screen mask static text*****************
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // enable internal HV supply for display and set I2C address
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.drawFastHLine(0,  14, 128, WHITE);
  display.drawFastHLine(0,  34, 128, WHITE);
  display.display();
  display.fillRect(85,0,43,12,BLACK); 
  display.setTextColor(WHITE); 
  display.setTextSize(1);
  display.setCursor(85, 0);
  display.print("GM8BJF");       
  display.display();

  SPI.begin();  // Start SPI for ADF5355
  pinMode (slaveSelectPin, OUTPUT);
  digitalWrite(slaveSelectPin, LOW);
  
 
  pinMode(pin_A, INPUT_PULLUP);
  pinMode(pin_B, INPUT_PULLUP);
  pinMode(pin_A2, INPUT_PULLUP);
  pinMode(pin_B2, INPUT_PULLUP);

  pinMode(2, INPUT_PULLUP);     // fix channel select
  pinMode(10, INPUT_PULLUP);     // power select
  pinMode(12, INPUT_PULLUP);    // lock/unlock
  pinMode(14, INPUT_PULLUP);    // intref/extref via #A0  *************************Pin für int/ext Referenz *******************
  pinMode(16, INPUT_PULLUP);    // intref/extref via #A7 !!!! NB DIFFERENT FOR DUE !!!!

  currentTime = millis();
  loopTime = currentTime;
  loopTime2 = currentTime;  
}

// *********************** Subroutine: update Display  **************************
void updateDisplay() {

 // I2C display update routine.
  if (mdbm == 0) {
    delayMicroseconds(100);
    display.fillRect(8,40,128,12,BLACK);
    delayMicroseconds(100);
    display.setTextColor(WHITE);
    delayMicroseconds(100);
    display.setTextSize(1);
    delayMicroseconds(100);
    display.setCursor(8, 40);
    delayMicroseconds(25);
    display.print("Power out = -4 dBm");
    delayMicroseconds(25000);

  }
  else if (mdbm == 1) {
    display.fillRect(8,40,128,12,BLACK);
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(8, 40);
    display.print("Power out = -1 dBm");

  }
  else if (mdbm == 2) {
    display.fillRect(8,40,128,12,BLACK);
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(8, 40);
    display.print("Power out = +2 dBm");

  }
  else if (mdbm == 3) {
    display.fillRect(8,40,128,12,BLACK);
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(8, 40);
    display.print("Power out = +5 dBm");
  }
  display.display();

  //*********************Diplaying tuning step size *************************
  
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(8 , 53);
  display.print("Step = ");
  float ChanStep2 = ChanStep;
  if (ChanStep2 < 100)
  { ChanStep2 = ChanStep2 / 0.1;
    display.fillRect(40,53,100,16,BLACK);
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(60 , 53);
    display.print(ChanStep2, 0);
    display.print(" Hz");
    display.display();

  }

   else if (ChanStep2 < 100000)
  { ChanStep2 = ChanStep2 / 100;
    display.fillRect(40,53,100,16,BLACK);
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(60 , 53);
    display.print(ChanStep2, 0);
    display.print(" KHz");
    display.display();

  }
  else
  { ChanStep2 = ChanStep2 / 100000;
    display.fillRect(40,53,100,16,BLACK);
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(60 , 53);
    display.print(ChanStep2, 0);
    display.print(" MHz");
  }

  //**********************Displaying frequency***************************
  
  double Freq2;
  Freq2 = Freq;
  Freq2 = Freq2 / 100000;
  /// For diagnostics//
  /*
    SerialUSB.print("Freq = ");
    SerialUSB.print(Freq);
    SerialUSB.println(" ");
    SerialUSB.print("Freq2 = ");
    SerialUSB.print(Freq2, 6);
    SerialUSB.println(" MHz");
  */  
   
  if (Freq2 < 1000) {
    
   display.fillRect(0,15,128,16,BLACK);
   
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(20 , 20);
    
    display.print(Freq2, 6 );
    display.print(" MHz");
    //////////////////////// Serial print for testing only ///////////////////////////////
    /*
    SerialUSB.print("Frequency = ");
    SerialUSB.print(Freq2, 5);
    SerialUSB.println(" MHz");
    */
  }
  
  else
  {
    display.fillRect(0,15,128,16,BLACK);
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(20 , 20);
    display.print(Freq2, 6);
    display.print(" MHz");
  }
 
 
}

////////////////////////////////////////////////////////////////////////
//                      MAIN PROGRAM LOOP                        //
////////////////////////////////////////////////////////////////////////
void loop() 
  {
  rotary_enc2();
  if (cnt_step != cnt_step_old) {
    updateDisplay();
    delayMicroseconds(250);
    updateDisplay();   // display updating needed a delay added
    cnt_step_old = cnt_step;
  }

  fixfrq_select();
  if (cnt_fix != cnt_fix_old) {
    updateDisplay();
    cnt_fix_old = cnt_fix;

  }

  pwr_select();
  if (cnt_pwr != cnt_pwr_old) {
    updateDisplay();
    cnt_pwr_old = cnt_pwr;
    SetFreq(Freq);
  }
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);


  if (digitalRead(12) == HIGH)   // select lock/unlock
  {
    mrk1 = 1;
  } else {
    mrk1 = 0;
  }
  //*************************
  
  if (mrk1 != mrk1_old) {

    if (digitalRead(12) == HIGH)
    {
      display.fillRect(0,0,80,12,BLACK);
      display.setTextColor(WHITE);
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.print("Locked");
      display.display();
    }
    else
    {
      display.fillRect(0,0,80,12,BLACK);
      display.setTextColor(WHITE);
      display.setTextSize(0.5);
      display.setCursor(0, 0);
      display.print("Unlocked");
      display.display();
    }
  }

  mrk1_old = mrk1;

  rotary_enc();
  if (Freq != Freq_Old) {
    updateDisplay();   // display updating needed a delay added
    SetFreq(Freq);
    
    Freq_Old = Freq;
  }
//delayMicroseconds(25000);
}
  
//*************************** End of main loop  ********************************************

/////////////////////////////// Subroutine: Frequency select /////////////////////////////////
void rotary_enc()
{
  currentTime = millis();
  if (currentTime >= (loopTime + 2)) {
    encoder_A = digitalRead(pin_A);
    encoder_B = digitalRead(pin_B);
    if ((!encoder_A) && (encoder_A_prev)) {
      if (encoder_B) {
        Freq = Freq + ChanStep;
        if (Freq > 1360000000) {
          Freq = 5400000;
        }
      }
      else {
        Freq = Freq - ChanStep;
        if (Freq < 5400000) {
          Freq = 1360000000;
        }
      }
    }
    encoder_A_prev = encoder_A;     // Store value of A for next time
    loopTime = currentTime;         // Updates loopTime
  }
}

/////////////////////////////// Subroutine: Step select ////////////////////////////////
void rotary_enc2()
{
  currentTime = millis();
  if (currentTime >= (loopTime2 + 2)) {
    encoder_A2 = digitalRead(pin_A2);
    encoder_B2 = digitalRead(pin_B2);
    if ((!encoder_A2) && (encoder_A2_prev)) {
      if (encoder_B2) {
        cnt_step = cnt_step + 1;
        if (cnt_step > 9) {
          cnt_step = 1;
        }
      }
      else {
        cnt_step = cnt_step - 1;
        if (cnt_step < 1) {
          cnt_step = 9;
        }
      }
    }
    encoder_A2_prev = encoder_A2;     // Store value of A for next time
    loopTime2 = currentTime;         // Updates loopTime
  }
  // Serial.println(cnt_step);
  if (cnt_step == 0) {
    ChanStep = 0.1;  // 1 Hz
  }
  else if (cnt_step == 1) {
    ChanStep = 1;  // 10 Hz
  }
  else if (cnt_step == 2) {
    ChanStep = 10;  // 100 Hz
  }
  
  else if (cnt_step == 3) {
    ChanStep = 100;  // 1 KHz
  }
  else if (cnt_step == 4) {
    ChanStep = 1000;  // 10 KHz
  }
  else if (cnt_step == 5) {
    ChanStep = 10000;  // 100KHz
  }
  else if (cnt_step == 6) {
    ChanStep = 100000;  // 1 MHz
  }
  else if (cnt_step == 7) {
    ChanStep = 1000000;  // 10 MHz
  }
  else if (cnt_step == 8) {
    ChanStep = 10000000;  // 100 MHz
  }
   else if (cnt_step == 9) {
    ChanStep = 100000000;  // 1000 MHz 
  }
  
}

/////////////////////////// Subroutine: Fixfrequencyselect ////////////////////////////
void fixfrq_select()
{
  press = digitalRead(2);
  if (press == LOW)
  {
    
    if (cnt_fix == 0) {
      Freq = 5200000;  // 52.0 MHz
    }
    else if (cnt_fix == 1) {
      Freq = 7010000;  // 70.1 MHz
    }
    else if (cnt_fix == 2) {
      Freq = 14420000;  // 144.2 MHz  
    }
    else if (cnt_fix == 3) {
      Freq = 43290000;  // 432.9 MHz  
    }
    else if (cnt_fix == 4) {
      Freq = 129690000;  // 1296.9 MHz
    }
    else if (cnt_fix == 5) {
      Freq = 232090000;  // 2320.9 MHz
    }
    else if (cnt_fix == 6) {
      Freq = 345610000;  // 3456.1 MHz 
    }
    else if (cnt_fix == 7) {
      Freq = 576050000;  // 5760.5 MHz 
    }
    else if (cnt_fix == 8) {
      Freq = 1036810000;  // 10368.1  MHz 
    }
    
    cnt_fix = cnt_fix + 1;
    if (cnt_fix == 9) {
      cnt_fix = 0 ;
    }
    delay(300);
  }
}

////////////////////////////////// Subroutine: Power select ///////////////////////////////
void pwr_select()
{
  press = digitalRead(10);
  if (press == LOW)
  {
    if (cnt_pwr == 0) {
      mdbm = 0;  // -4dBm
    }
    else if (cnt_pwr == 1) {
      mdbm = 1;  // -1dBm
    }
    else if (cnt_pwr == 2) {
      mdbm = 2;  // +2dBm
    }
    else if (cnt_pwr == 3) {
      mdbm = 3;  // +5dBm
    }
    cnt_pwr = cnt_pwr + 1;
    if (cnt_pwr == 4) {
      cnt_pwr = 0 ;
    }
    delay(300);
  }
}
//////////////////////////////////////////////////////////////////////////////////////
