   
    /******************************** Scharre Soft™ *******************************

    Copyright 2022 Willem E.J. Hoekert.

    Redistribution and use in source and binary forms, with or without 
    modification, are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, 
    this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice, 
    this list of conditions and the following disclaimer in the documentation 
    and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors 
    may be used to endorse or promote products derived from this software without 
    specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER AND CONTRIBUTORS "AS IS" 
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
    FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
    SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  
 
    (c) 2022 Willem Hoekert
    Scharre Soft™ 
    Amsterdam, The Netherlands
    ScharreSoft@outlook.com

    ******************************************************************************/


#include "PM909_config.h" 
#include <Wire.h>
#include<LiquidCrystal_I2C.h>
#include "Tlc5940_AVR0.h"                            //   Edit Tlc_AVR0_config.h:     NUM_TLCS 2  
                                                     //   Edit Tlc_AVR0_config.h:     GRAYSCALE_RESOLUTION 511

LiquidCrystal_I2C lcd(0x27,16,2);                    //   I2C address depends on the chip of the I2C interface: 0x27(PCF8574(T)) or 0x3F(PCF8574A(T))

long address;                                        //   used for writing to the EEPROM
int disk1, disk2;

#define BUTTONS 16
#define MaxSteps 16
#define RESOLUTION 1023

#define INSTRUMENT 1                                 //  Menu items
#define SHUFFLE 2
#define FLAM 3
#define ACCENT_HI 4
#define ACCENT_LO 5
#define MUTE_VOICE 6
#define PATTERN_PLAY 7
#define WRITE_TRACK 8
#define LOAD 9
#define SAVE 10   
#define LASTSTEP 11  
#define CLEAR_TRACK 12
#define COPY_PATTERN 13                                                                 
#define TRACK_PLAY 14
#define DELETE_PATTERN 15
#define INSERT_PATTERN 16                                                                 
#define MIDI_IN 17
#define MIDI_OUT 18
#define EDIT_TRACK 19
#define RANDOM_TRACK 20
#define IMPROVISE 21
#define SHIFT_PATTERN 22
#define ERASE_EEPROM 23
#define SHOW_BPM 24
#define TEMPO 25
#define CLOCK_SOURCE 26
#define TAP_MODE 27

#define Open_hihat 7                                 // Required when using a TR-909 or TR-808 hihat module. 
#define Closed_hihat 8                               // Required when using a TR-909 or TR-808 hihat module. 

struct INSTR {
  uint16_t NOTE;                                     // the 16 bits in  NOTE   represent the Notes for the 16 steps.     '1': there is a note, '0': there is no note
  uint16_t ACCENT;                                   // the 16 bits in  ACCENT represent the Accents for the 16 steps.   '1': there is accent, '0': there is no accent
  uint16_t FLAMZ;                                    // the 16 bits in  FLAMZ  represent the Flam for the 16 steps.      '1': there is flam,   '0': there is no flam
};

struct PATTERN {
  INSTR Voice[MaxVoices];                            // 1 PATTERN consistes of 11 voices (MaxVoices = 11). PATTERN holds all the information (notes, accents, flam) for all voices in one pattern of 16 steps
};

PATTERN DRUM_PATTERN[MaxPatterns];                   // Create an array of PATTERNs 

byte NoteBit, AccentBit, FlamBit;

byte midi_start = 0xfa;                              // variables used for MIDI
byte midi_stop = 0xfc;
byte midi_clock = 0xf8;
byte midi_continue = 0xfb;
int play_flag = 0;
byte data;
byte MIDI_Clock_Counter=0;
byte noteON = 144;                                   // 144 = 10010000 in binary, note on command
byte noteOFF = 128;                                  // 128 = 10000000 in binary, note off command

const byte STEP_pin = A0;
const byte ENTER_pin = A1;
const byte ESCAPE_pin = A2;
const byte STARTSTOP_pin = A3;
const byte Eight_Buttons_pin = A6;
const byte RESET_pin = A7;
const byte SELECT_pin = 6;
const byte CLOCK_OUT_pin = 7;


byte getalA = 0B00000000;                            // Used for the shift registers
byte getalB = 0B00000000;
const byte clockPin = 4;                             // Pin connected to SH_CP of 74HC595     ( Shift registers )
const byte dataPin  = 5;                             // Pin connected to DS    of 74HC595     ( Shift registers )
const byte latchPin = 8;                             // Pin connected to ST_CP of 74HC595     ( Shift registers )
              
unsigned long debounceDelay=200,                     // variables used for debouncing the switches
              TapDebounce=5,
              lastDebounceTime_STEP_switch=0,
              lastDebounceTime_OPTION_button=0,
              lastDebounceTime_ENTER_switch=0,
              lastDebounceTime_ESCAPE_switch=0,
              lastDebounceTime_STARTSTOP_switch=0,
              WaitUntil=0,
              Potmeter_Debounce=0;

volatile bool PlayNotes=false;                       // used in the ISR

unsigned long PostponeUntil[MaxVoices], PlayAgainAt[MaxVoices], MeasureTime, lastmillis;                           //  variables used for Flam and Shuffle
byte Mute[MaxVoices], Flam[MaxVoices], Shuffle[MaxVoices]; bool ShuffleHasBeenPlayed[MaxVoices], FlamHasBeenPlayed[MaxVoices];      //  variables used for Flam and Shuffle

byte Accent_HI[MaxVoices], Accent_LO[MaxVoices];

bool UP_Pressed=false, 
     DOWN_Pressed=false, 
     ENTER_Pressed=false, 
     ESCAPE_Pressed=false, 
     RUN=true,
     Select_instru=false,
     Add_flam=false,
     Int_clock_mode=true,
     Read_Tempo_Pot=false,
     RESET=false,
     PotmeterChanged=false,
     Tap_Mode=false,
     Mute_Voice_Mode=false,
     VoiceChanged=false,
     refresh_it=false;

byte StepCounter;  
byte STEP, LAST_STEP=16, PatternNumber, Menu_Item=INSTRUMENT;
byte VoiceNumber;
byte Rhythm_Track[MaxComposition]; 
byte UsedPatterns; 
byte CurrentTrackNumber=0;

volatile int TimerInterruptFactor=0;
volatile byte Sub_StepCounter;
int BPM=120, new_BPM;
int BPM_Factor;

unsigned long trigger_OFF, temp_millis;

long value,oldvalue;

byte Instru_Number; 
int lastStepCounter;   // TapMode

// -------------------------------------------------------------------------------------------------------------------------------------------------------------------


void CheckTempoPotmeter() {

  value=analogRead(STARTSTOP_pin);
  
  if (abs(value-oldvalue)>0)
    new_BPM = 80 + (value-575)*120/431;
  oldvalue=value;

  if (new_BPM>0) {                                         // if Start/Stop is pressed, BPM is < 0, ignore that   
      if ((abs(new_BPM-BPM)>1) && (new_BPM % 2) == 0) {
      PotmeterChanged=true;
      Potmeter_Debounce = millis();
    }  
 
    if (millis()-Potmeter_Debounce>200) PotmeterChanged=false;                                                     

    if (PotmeterChanged) {   
      BPM = 80 + (value-575)*120/431;
      BPM_Factor = (240000)/BPM;   TCB2.CCMP = BPM_Factor;
      if (Menu_Item==TEMPO) {
      lcd.setCursor(0,1);  lcd.print(BPM); lcd.print(F(" BPM  "));  }
      MeasureTime = 4*60000/BPM; 
      PotmeterChanged=false;
    }  
  } 
}


void setup() {
byte i,j,k;
  
  pinMode(ENTER_pin,INPUT_PULLUP);   
  pinMode(ESCAPE_pin,INPUT_PULLUP); 
  pinMode(STARTSTOP_pin,INPUT);
  pinMode(SELECT_pin,INPUT_PULLUP);
  pinMode(RESET_pin,INPUT_PULLUP);
  pinMode(CLOCK_OUT_pin,OUTPUT);
  
  pinMode(latchPin, OUTPUT);          // Shift registers       used for the trigger pins
  pinMode(clockPin, OUTPUT);          // Shift registers
  pinMode(dataPin, OUTPUT);           // Shift registers

  if (Large_EEPROM) { disk1=0x50; disk2=0x54; }  // for EEPROMS of 1025 Kbit, two addresses are required : disk1 and disk2
  else              { disk1=0x54; disk2=0x50; }  // for smaller EEPROMS, only one address is required : disk1 

  lcd.begin();                        // initialize LCD screen and show start-up screen
  lcd.print(F("  ScharTronic   "));    
  lcd.setCursor(0,1);
  lcd.print(F(" PM-909  v4.0.2 "));
  delay(1500);

  Menu_Item = INSTRUMENT; refresh_it=true;
 
  Wire.begin();    address = 0;       // initialize EEPROM 
  Tlc_init();                         // Initialize TLC5940 and set all channels off
  Tlc_update();

  Serial1.begin(31250);               // MIDI

  PatternNumber=1;
  VoiceNumber=1;
  StepCounter=1;
  UsedPatterns=1;
  
  for (i=0; i<MaxVoices; i++){ 
      Mute[i]=0;
      Shuffle[i]=0; Flam[i]=0; 
      Accent_LO[i]=5; Accent_HI[i]=10; 
      FlamHasBeenPlayed[i]=true;
      ShuffleHasBeenPlayed[i]=true;
  }
  
  for (i=0; i<MaxPatterns; i++)        
    for (j=0; j<MaxVoices; j++)        
      for (k=1; k<=MaxSteps; k++) {
        WriteNote  (i, j, k, 0);
        WriteAccent(i, j, k, 0);
        WriteFlam  (i, j, k, 0); 
      }

  for (i=0; i<MaxComposition; i++) Rhythm_Track[i]=0;
  
  set_Pattern_LEDs();  
  UP_Pressed = false;
  DOWN_Pressed = false;
  ENTER_Pressed = false; 
  ESCAPE_Pressed=false;
  RUN=true;
  LAST_STEP=MaxSteps;

  TCB2.CTRLB = TCB_CNTMODE_INT_gc;         // Periodic Interrupt              // Setup timer interrup for the internal clock
  TCB2.CCMP = 2000;                        // Value to compare with
  TCB2.CTRLA = TCB_CLKSEL_CLKDIV1_gc;      // div1 prescale
  TCB2.CTRLA |= TCB_ENABLE_bm;             // enable timer
  TCB2.INTCTRL = TCB_CAPT_bm;              // Enable the interrupt

  attachInterrupt(digitalPinToInterrupt(2), isr, CHANGE);                     // Setup hardware interrupt for the external clock (LFO)
  detachInterrupt(digitalPinToInterrupt(2));

//  LoadFileNr(3);                                                            // Un-comment this line if you want the sequencer to load and play a file on start up.
}


void print_instrument(byte Z){
  if (Menu_Item==INSTRUMENT) {
    if (Mute_Voice_Mode) lcd.print(F("m>")); 
    else if (Tap_Mode) lcd.print(F("t>")); 
    else lcd.print(F("=>")); 
  }
  else lcd.setCursor(2,0);
  strcpy_P(buffer, (char *)pgm_read_word(&(string_table[Z])));                               // Instrument (voice) names are stored in a string table in FLASH memory.
  lcd.print(buffer);
}


void Send_MIDI_Note(byte Command, byte Note, byte Velocy) {
  Serial1.write(Command);   //send note on command 
  Serial1.write(Note);      //send pitch data
  Serial1.write(Velocy);    //send velocity data   
}


void General_MIDI_Drum_Map(byte Command, byte _instr, byte Velocy) {                            // choose the MIDI note according to the General MIDI PERCUSSION Key Map
  switch (_instr) {
     case 0:  Send_MIDI_Note(Command, 36, Velocy);  break;    //   C1   bass drum     (2)
     case 1:  Send_MIDI_Note(Command, 38, Velocy);  break;    //   D1   snare drum    (3)
     case 2:  Send_MIDI_Note(Command, 45, Velocy);  break;    //   A1   low tom       (6)
     case 3:  Send_MIDI_Note(Command, 47, Velocy);  break;    //   B1   mid tom       (8)
     case 4:  Send_MIDI_Note(Command, 50, Velocy);  break;    //   D2   hi tom        (10)
     case 5:  Send_MIDI_Note(Command, 33, Velocy);  break;    //   A0   rim shot      (1)        / Rim shot is not in the GM PERCUSSION Key Map. I use note 33 instead
     case 6:  Send_MIDI_Note(Command, 39, Velocy);  break;    //   D#1  clap          (4)
     case 7:  Send_MIDI_Note(Command, 46, Velocy);  break;    //   A#1  open hihat    (7)
     case 8:  Send_MIDI_Note(Command, 42, Velocy);  break;    //   F#1  closed hihat  (5)
     case 9:  Send_MIDI_Note(Command, 51, Velocy);  break;    //   D#2  ride cymbal   (11)
     case 10: Send_MIDI_Note(Command, 49, Velocy);  break;    //   C#2  crash cymbal  (9)

     case 11: Send_MIDI_Note(Command, 56, Velocy);  break;    //   G#2  cow bell      (13)
     case 12: Send_MIDI_Note(Command, 75, Velocy);  break;    //   D#4  clave         (16)
     case 13: Send_MIDI_Note(Command, 54, Velocy);  break;    //   F#2  tambourine    (12)
     case 14: Send_MIDI_Note(Command, 73, Velocy);  break;    //   C#4  guiro         (15)
     case 15: Send_MIDI_Note(Command, 70, Velocy);  break;    //   A#3  maracas       (14) 
  }
}


void MIDI_Out() {                                                                
  byte stap, MIDInote;  bool finish=false, NootGespeeld; 
  byte MIDIvelocity, trackstep=0;
  unsigned long Noot_Off[MaxVoices];    byte instr;      

  stap=1; PatternNumber=Rhythm_Track[trackstep]; PlayNotes=false;   bool DoYouWantToExit=false;
  for (int i=0; i<MaxVoices; i++){ 
      FlamHasBeenPlayed[i]=true;
      ShuffleHasBeenPlayed[i]=true;
  }

  if (PatternNumber==0) {                                                                    // check if a track has been written. Only a written track can be sent
    lcd.setCursor(0,1); lcd.print(F("No track defined")); 
    while ((!ESCAPE_Pressed) && (!ENTER_Pressed)) {
      Check_For_ENTER_Press();
      Check_For_ESCAPE_Press();
      if ((ESCAPE_Pressed) || (ENTER_Pressed)) { 
        finish=true; PatternNumber=1; 
        Menu_Item=INSTRUMENT;
        Refresh_LCD();
      }
    }  
  }

  if (!finish) {
    lcd.setCursor(0,0); lcd.print(F("Sending MIDI    ")); 
    lcd.setCursor(0,1); lcd.print(F("Step:1   Ptrn:"));  lcd.print(PatternNumber);
  }
    
  while (!finish) {

    Check_Start_Stop_Button();
    Check_For_ESCAPE_Press();
    if (ESCAPE_Pressed) {
      if (DoYouWantToExit==true) { 
        DoYouWantToExit = false; 
        lcd.setCursor(8,0); lcd.print(F("MIDI    "));
      }
      else {
        DoYouWantToExit = true; 
        lcd.setCursor(8,0); lcd.print(F(" Exit?  "));
      }    
      ESCAPE_Pressed = false;
    }
    
    Check_For_ENTER_Press(); 
    if (ENTER_Pressed) {  
      if (DoYouWantToExit) {
        finish = true;
        Menu_Item=INSTRUMENT;
        Refresh_LCD();
      }
      ENTER_Pressed=false;
    }

    temp_millis = millis();                                                                  // --------- Shuffle: are there postponed notes that have to be played right now?
    for (instr=0; instr<MaxVoices; instr++) {
      if (temp_millis > PostponeUntil[instr])                                                     
        if (!ShuffleHasBeenPlayed[instr]) {
          AccentBit = ReadAccent(PatternNumber-1,instr, stap);
          
          if (AccentBit==0) MIDIvelocity = 12*Accent_LO[instr];   // zacht  
          if (AccentBit==1) MIDIvelocity = 12*Accent_HI[instr];   // hard
          General_MIDI_Drum_Map(noteON, instr, MIDIvelocity);    
          Noot_Off[instr] = temp_millis + 20;
          FlamBit = ReadFlam(PatternNumber-1, instr, stap);
          if ((FlamBit==1)  && (Flam[instr]))  {                                             // there is flam!
            PlayAgainAt[instr] = temp_millis + Flam[instr]*4 + 6;                     
            if (Flam[instr]==10) PlayAgainAt[instr] = temp_millis + MeasureTime/8;
            FlamHasBeenPlayed[instr]=0;
          }
          ShuffleHasBeenPlayed[instr]=true;
        }
    }  

    temp_millis = millis();                                                  
    for (instr=0; instr<MaxVoices; instr++) {                                                // ------------- Flam: are there notes that have to be played again?
    if (temp_millis > PlayAgainAt[instr])                                               
      if (!FlamHasBeenPlayed[instr]){
        AccentBit = ReadAccent(PatternNumber-1,instr, stap);
        if (AccentBit==0) MIDIvelocity = 12*Accent_LO[instr];   // zacht  
        if (AccentBit==1) MIDIvelocity = 12*Accent_HI[instr];   // hard
        General_MIDI_Drum_Map(noteON, instr, MIDIvelocity); 
        Noot_Off[instr] = temp_millis + 20;
        FlamHasBeenPlayed[instr]=true; 
      }
    } 

    if (PlayNotes) {
      PlayNotes=false;  
      for(instr=0; instr<MaxVoices; instr++) {

        AccentBit = ReadAccent(PatternNumber-1,instr, stap);
        if (AccentBit==0) MIDIvelocity = 12*Accent_LO[instr];   // Low accent  
        if (AccentBit==1) MIDIvelocity = 12*Accent_HI[instr];   // High accent

        NoteBit   = ReadNote(PatternNumber-1,instr,stap);
        FlamBit   = ReadFlam(PatternNumber-1,instr,stap);
      
        if (NoteBit==1)                                                                      // is there a note?
          if ((Shuffle[instr]) && ((stap % 2) == 0))  {                                      // is there shuffle and is it an even note?
            PostponeUntil[instr] = temp_millis + Shuffle[instr]*MeasureTime/400;             // if so, postpone note.                       
            if (Shuffle[instr]==10) PostponeUntil[instr] = temp_millis + MeasureTime/32;                         
            ShuffleHasBeenPlayed[instr]=false;                          
          }
          else {  
            General_MIDI_Drum_Map(noteON, instr, MIDIvelocity);                              // is there a note and no shuffle? then play it immediately.
            Noot_Off[instr] = temp_millis + 20;
            
            if ((FlamBit==1) && (Flam[instr]))  {                                            // the note has flam, prepare a second note to be played later on          
              PlayAgainAt[instr] = temp_millis + Flam[instr]*4 + 6;
              if (Flam[instr]==10) PlayAgainAt[instr] = temp_millis + MeasureTime/32;        // if flam = 10, then in between two notes (32th)              
              FlamHasBeenPlayed[instr]=false; 
            }
          }  
      }
      
      stap++; if (stap>LAST_STEP)    { 
        stap=1; 
        trackstep++; if (Rhythm_Track[trackstep]==0) { trackstep=0; lcd.setCursor(5,1); lcd.print(F("   "));} 
        if (trackstep>MaxComposition) finish=true;                                           //  never?
        PatternNumber = Rhythm_Track[trackstep];
        lcd.setCursor(5,1); lcd.print(trackstep+1);  
        lcd.setCursor(14,1); lcd.print(Rhythm_Track[trackstep]); lcd.print(F(" "));       
      }
    }

    for (instr=0; instr<MaxVoices; instr++) {
      if (temp_millis>Noot_Off[instr]) {   
        General_MIDI_Drum_Map(noteOFF, instr, 0);                                            // send MIDI noteOff command
      }
    }    
  }
}


void MIDI_In() {
  byte incomingByte;  bool finish=false;
  
  byte state;
  int noteDown = LOW;
  byte note; 
  byte channel = 0;                                                               // set MIDI channel 1

  bool DoYouWantToExit=false;
  lcd.setCursor(0,0); lcd.print(F("Receiving MIDI        "));
  for (byte i=16; i<32; i++) Tlc_set(i,0);   Tlc_update();                        // All LEDs off
  
  while (!finish) {
   
    Check_For_ESCAPE_Press();
    if (ESCAPE_Pressed) {
      if (DoYouWantToExit==true) { 
        DoYouWantToExit = false; 
        lcd.setCursor(9,1); lcd.print(F("     "));
      }
      else {
        DoYouWantToExit = true; 
        lcd.setCursor(9,1); lcd.print(F("Exit?  "));
      }    
      ESCAPE_Pressed = false;
    }
    
    Check_For_ENTER_Press(); 
    if (ENTER_Pressed) {  
      if (DoYouWantToExit) {
        finish = true;
        Menu_Item=INSTRUMENT;
        Refresh_LCD();
      }
      ENTER_Pressed=false;
    }
 
    if (Serial1.available() > 0) {
     
      incomingByte = Serial1.read();                 // read the incoming byte:
      switch (state){
        case 0:
                                                     // look for as status-byte, our channel, note on
          if (incomingByte== (144 | channel)){       // un comment and comment the line below - to read only one channel
    //    if ((incomingByte & 0xf0) == 0x90){        // this reads all channels
          noteDown = HIGH;
          state=1;
        }
                                                     // look for as status-byte, our channel, note off
          if (incomingByte== (128 | channel)){       // un comment and comment the line below - to read only one channel 
    //    if ((incomingByte & 0xf0) == 0x80){        // this reads all channels
          noteDown = LOW;
          state=1;
        }
        
        case 1:
          if(incomingByte < 128) {                   // get the note to play or stop
            note=incomingByte;
            state=2;
          }
          else {
            state = 0;                               // reset state machine as this should be a note number
          }
          break;
       
        case 2:
          if((incomingByte < 128) && (incomingByte>0)) {    // get the velocity
            if (noteDown) {
              if (note==36) { Tlc_set(0,255-incomingByte*2);    SetPin(1,HIGH);   Tlc_set(16,LED_HI); }           //  C1   bass drum      (2)
              if (note==38) { Tlc_set(1,255-incomingByte*2);    SetPin(2,HIGH);   Tlc_set(17,LED_HI); }           //  D1   snare drum     (3)   
              if (note==45) { Tlc_set(2,255-incomingByte*2);    SetPin(3,HIGH);   Tlc_set(18,LED_HI); }           //  A1   low tom        (6)
              if (note==47) { Tlc_set(3,255-incomingByte*2);    SetPin(4,HIGH);   Tlc_set(19,LED_HI); }           //  B1   mid tom        (8)
              if (note==50) { Tlc_set(4,255-incomingByte*2);    SetPin(5,HIGH);   Tlc_set(20,LED_HI); }           //  D2   hi tom         (10)
              if (note==33) { Tlc_set(5,255-incomingByte*2);    SetPin(6,HIGH);   Tlc_set(21,LED_HI); }           //  A0   rim shot       (1)
              if (note==39) { Tlc_set(6,255-incomingByte*2);    SetPin(7,HIGH);   Tlc_set(22,LED_HI); }           //  D#1  clap           (4)          
              if (note==46) {                                                     Tlc_set(23,LED_HI); 
                              #if ((!TR_909_HIHAT) && (!TR_808_HIHAT)) 
                                { Tlc_set(7,255-incomingByte*2);    SetPin(8,HIGH);   }                           //  A#1  open hihat     (7) 
                              #endif  
                                
                              #if (TR_909_HIHAT) 
                                Tlc_set(Closed_hihat,255-incomingByte*2);
                                SetPin(Open_hihat+1,LOW);         // HIHAT SELECT = LOW for an Open Hihat sound
                                SetPin(Closed_hihat+1,HIGH);      // Closed hihat pin is HIGH for OH_Trigger 
                              #endif
                            }  
              if (note==42) {                                                     Tlc_set(24,LED_HI);
                              #if ((!TR_909_HIHAT) && (!TR_808_HIHAT))
                                Tlc_set(8,255-incomingByte*2);    SetPin(9,HIGH);                                 //  F#1  closed hihat   (5)
                              #endif 

                              #if (TR_909_HIHAT)
                                Tlc_set(Closed_hihat,255-incomingByte*2);
                                SetPin(Open_hihat+1,HIGH);        // HIHAT SELECT = HIGH for a Closed Hihat sound  
                                SetPin(Closed_hihat+1,HIGH);      // Closed hihat pin is HIGH for CH_Trigger 
                              #endif  
                              
                              #if (TR_808_HIHAT) 
                                Tlc_set(Open_hihat,255-incomingByte*2); SetPin(9,HIGH);
                              #endif                                
                            }
              if (note==51) { Tlc_set(9,255-incomingByte*2);    SetPin(10,HIGH);   Tlc_set(25,LED_HI); }          //  D#2  ride cymbal    (11)
              if (note==49) { Tlc_set(10,255-incomingByte*2);   SetPin(11,HIGH);   Tlc_set(26,LED_HI); }          //  C#2  crash cymbal   (9)

              if (note==56) { Tlc_set(11,255-incomingByte*2);   SetPin(12,HIGH);   Tlc_set(27,LED_HI); }          //  G#2  cow bell       (13)
              if (note==75) { Tlc_set(12,255-incomingByte*2);   SetPin(13,HIGH);   Tlc_set(28,LED_HI); }          //  D#4  clave          (16) 
              if (note==54) { Tlc_set(13,255-incomingByte*2);   SetPin(14,HIGH);   Tlc_set(29,LED_HI); }          //  F#2  tambourine     (12)
              if (note==73) { Tlc_set(14,255-incomingByte*2);   SetPin(15,HIGH);   Tlc_set(30,LED_HI); }          //  C#4  guiro          (15)            
              if (note==70) { Tlc_set(15,255-incomingByte*2);   SetPin(16,HIGH);   Tlc_set(31,LED_HI); }          //  A#3  maracas        (14)   

              Tlc_update();   WritePins(); 
            }  
          } 
          state = 0;  // reset state machine to start            
      }
    } 

    if (millis()>trigger_OFF) {
      #if (!TR_909_HIHAT)
        for(byte i=0; i<MaxVoices; i++) { 
          SetPin(i+1,LOW);                                   // normal PMDS operation: set all triggers to 0 after 5 ms
          Tlc_set(16+i,0);
        }
      #endif  

      #if (TR_909_HIHAT)
        for(byte i=0; i<MaxVoices; i++) {
          if (i!=Open_hihat) SetPin(i+1,LOW);                // PM909 operation: all triggers to 0 except: OH trigger
          Tlc_set(16+i,0);
        }  
      #endif 
  
      WritePins();   
    }
  }
  set_Pattern_LEDs(); 
}


void Do_PlayTrack() {

  bool finish=false;   byte TrackStep=0;  bool DoYouWantToExit=false;   

  if (Rhythm_Track[0]==0) {                                                                  // check if a track has been written. Only a written track can be sent
    lcd.setCursor(0,0); lcd.print(F("              "));    
    lcd.setCursor(0,1); lcd.print(F("No track defined")); 
    while ((!ESCAPE_Pressed) && (!ENTER_Pressed)) {
      Check_For_ENTER_Press();
      Check_For_ESCAPE_Press();
      if ((ESCAPE_Pressed) || (ENTER_Pressed)) { 
        finish=true; 
        Menu_Item=INSTRUMENT;
        Refresh_LCD();
      }
    }  
  } else { lcd.setCursor(14,1); lcd.print(Rhythm_Track[0]);  } 

  if (!finish) {
    ENTER_Pressed = false; ESCAPE_Pressed = false;
    PatternNumber=Rhythm_Track[0];   
    lcd.clear(); lcd.print(F("Playing track"));
    lcd.setCursor(0,1); lcd.print(F("Step:1   Ptrn:")); lcd.print(PatternNumber);
  }

  while (!finish ) {

    Check_For_ENTER_Press(); 
    if (ENTER_Pressed) {  
      if (DoYouWantToExit) {
        finish = true;
        Menu_Item=INSTRUMENT;
        Refresh_LCD();
      }
      if (!RUN) {
        TrackStep=0;  PatternNumber=1;
       lcd.setCursor(0,1); lcd.print(F("Step:1   Ptrn:")); lcd.print(Rhythm_Track[0]); lcd.print(F(" "));
      }
      ENTER_Pressed=false;
    }

    Check_For_ESCAPE_Press();
    if (ESCAPE_Pressed){
      if (DoYouWantToExit) {
        DoYouWantToExit = false; 
        lcd.setCursor(9,1); lcd.print(F("Ptrn:"));
      }
      else {
        DoYouWantToExit = true; 
        lcd.setCursor(9,1); lcd.print(F("Exit?  "));
      }  
      ESCAPE_Pressed = false;
    }

    if ( (millis() - lastDebounceTime_STARTSTOP_switch) > debounceDelay) { 
      if (digitalRead(STARTSTOP_pin)==LOW) {
        lastDebounceTime_STARTSTOP_switch = millis();
        if (RUN==true) { 
          RUN=false; 
          Serial1.write(midi_stop);
        }  
        else {
          StepCounter=1;  set_Pattern_LEDs();  RUN=true;   
          lastmillis=millis();                                                               // to correct MeasureTime   
          Serial1.write(midi_start);
          play_flag = 1;
          TimerInterruptFactor=0;
          PlayNotes=true;                
        }        
      }
    }

    if (Mute_Voice_Mode) Check_Voice_Mute_Mode();
    Check_For_Shuffle();
    Check_For_Flam();
    Check_MIDI_Clock_In();
    Check_Reset();    
    
    if(Read_Tempo_Pot) CheckTempoPotmeter();

    if (PlayNotes) {
      
      if (StepCounter==1) set_Pattern_LEDs();
      
      if (!RESET) {
        Set_Velocities();
        Walking_LEDs();
        Set_Trigger_Pins();    
        if (RUN) StepCounter++;   
       
        if (StepCounter>LAST_STEP) {                       // Measure_Finished();
          StepCounter=1;
          MeasureTime=millis()-lastmillis;                 
          lastmillis=millis();      
           
          TrackStep++;   
          if ((Rhythm_Track[TrackStep]==0) || (TrackStep > MaxComposition-1)) {
            TrackStep=0; lcd.setCursor(5,1); lcd.print(F("   ")); 
          }
          
          PatternNumber = Rhythm_Track[TrackStep]; // set_Pattern_LEDs();
        
          lcd.setCursor(5,1); lcd.print(TrackStep+1);  
          if (!DoYouWantToExit) { lcd.setCursor(14,1); lcd.print(Rhythm_Track[TrackStep]); lcd.print(F(" "));  } 
        }
        
        PlayNotes = false;  
      }
      
      if (RESET) {
        Reset_Sequencer(); 
        TrackStep=0; 
        PatternNumber = Rhythm_Track[TrackStep];   
        lcd.setCursor(5,1); lcd.print(F("1 ")); 
        lcd.setCursor(14,1); lcd.print(Rhythm_Track[0]); lcd.print(F(" "));
      }
    }
    if (millis()>trigger_OFF) Set_Triggers_To_Zero();      
  }
}


void Select_Clock_Source() {                                                                    
  bool  finish=false; byte _button;                                                          // clock source can be Internal Clock (timer interrupt), or External Clock (hardware interrupt or MIDI clock signal).
  
  lcd.setCursor(0,1); 
  if  (Int_clock_mode) lcd.print(F("Internal Clock  "));  
  if (!Int_clock_mode) lcd.print(F("LFO / MIDI      "));
  
  ENTER_Pressed = false;  delay(700);
  while (!finish) {
    if ( (millis() - lastDebounceTime_OPTION_button) > debounceDelay) {
      UP_Pressed = false;
      DOWN_Pressed = false;
      _button = readAnalog_MODE_Button();
      lastDebounceTime_OPTION_button = millis();
      switch (_button) {
        case 1:      
        case 2: switch (Int_clock_mode) {
                  case true: lcd.setCursor(0,1); lcd.print(F("LFO / MIDI      ")); Int_clock_mode=false;  break;
                  case false: lcd.setCursor(0,1); lcd.print(F("Internal Clock  ")); Int_clock_mode=true;  break;
                }
                break;           
      }
      Check_For_ENTER_Press();  Check_For_ESCAPE_Press();

      if (ESCAPE_Pressed){
        finish = true;
        Menu_Item=INSTRUMENT;
        Refresh_LCD(); 
      }
        
      if (ENTER_Pressed) {
        finish = true;
        Menu_Item=INSTRUMENT;
        Refresh_LCD(); 
        if (!Int_clock_mode) {          
           TCB2.INTCTRL = 0b00000000;                               // disable Timer interrupt
           attachInterrupt(digitalPinToInterrupt(2), isr, CHANGE);  // enable Hardware interrupt
        }   
        if (Int_clock_mode) {           
           TCB2.INTCTRL = 0b00000001;                               // enable Timer interrupt
           detachInterrupt(digitalPinToInterrupt(2));               // disable Hardware interrupt
           MeasureTime = 4*60000/BPM;
        }  
      }  
    } 
  }
}


void Delete_Pattern() {
bool finish=false;
ENTER_Pressed=false; ESCAPE_Pressed=false;  lcd.setCursor(7,0); lcd.print(F("        "));
lcd.setCursor(0,1); lcd.print(F("pattern "));  lcd.print(PatternNumber); lcd.print(F("?"));

  while (!finish) {  
   
    Check_For_ESCAPE_Press();
    if (ESCAPE_Pressed) {
      finish=true;   
      ESCAPE_Pressed = false;
    }
    
    Check_For_ENTER_Press(); 
    if (ENTER_Pressed) {
      for (byte i=PatternNumber-1; i<MaxPatterns; i++) DRUM_PATTERN[i] = DRUM_PATTERN[i+1];    
      if (UsedPatterns>1) UsedPatterns--;  finish=true;
      ENTER_Pressed=false;
    }
  }
  Menu_Item=INSTRUMENT;
  Refresh_LCD();
}  


void Insert_Pattern() {
bool finish=false;
ENTER_Pressed=false; ESCAPE_Pressed=false;
lcd.setCursor(0,1); lcd.print(F("between "));  lcd.print(PatternNumber-1); lcd.print(F(" and ")); lcd.print(PatternNumber);  lcd.print(F("?"));

  while (!finish) {  
   
    Check_For_ESCAPE_Press();
    if (ESCAPE_Pressed) {
      finish=true;   
      ESCAPE_Pressed = false;
    }
    
    Check_For_ENTER_Press();
    if (ENTER_Pressed) {
      for (byte i=MaxPatterns-1; i>PatternNumber-1; i--)  
        DRUM_PATTERN[i] = DRUM_PATTERN[i-1] ;                                //  shift patterns 

      for (byte j=0; j<MaxVoices; j++) {    
        DRUM_PATTERN[PatternNumber-1].Voice[j].NOTE   = 0;
        DRUM_PATTERN[PatternNumber-1].Voice[j].ACCENT = 0;
        DRUM_PATTERN[PatternNumber-1].Voice[j].FLAMZ  = 0;      
      }  
      if (UsedPatterns<MaxPatterns) UsedPatterns++;  finish=true;
      ENTER_Pressed=false;
    }
  }
  Menu_Item=INSTRUMENT;
  Refresh_LCD();  
} 


void Erase_EEPROM(){
  bool finish=false;  ENTER_Pressed=false;
  lcd.setCursor(0,1); lcd.print(F("Erase all data?")); delay(1000);
  
  while (!finish) {  
    
    Check_For_ESCAPE_Press();
    if (ESCAPE_Pressed) {
      finish=true;
      Menu_Item=INSTRUMENT;
      Refresh_LCD();
      ESCAPE_Pressed = false;      
    }
    
    Check_For_ENTER_Press(); 

    if (ENTER_Pressed) {

//       writeEEPROM(3, 0);                                                 // This line only erases drum track 4
//       writeEEPROM(7, 16);                                                // This line only un-erases drum track 8
      
//      for (byte i=1; i<=64; i++) {                                                 
//        address = i-1;  writeEEPROM(address, 0); }                          //  These two lines erase all 64 drum tracks

//      for (byte i=1; i<=64; i++) {                                                 
//        address = i-1;  writeEEPROM(address, 16); }                       //  These two lines un-erase all 64 drum tracks        
  
     lcd.setCursor(0,1); lcd.print(F("EEPROM Erased! "));
     delay(1000);
        Menu_Item=INSTRUMENT;
        Refresh_LCD();
        finish=true;
        ENTER_Pressed=false;
    } 
  }
}


byte Get_Two_Digits(byte _X) {
  byte getal=0;   byte tmpnummer;   ENTER_Pressed=false;   ESCAPE_Pressed=false;
  byte firstDigit=255, secondDigit=255;

  while((!ENTER_Pressed) && (!ESCAPE_Pressed)) {
    
    if ( (millis() - lastDebounceTime_STEP_switch) > debounceDelay) {                
      tmpnummer = readAnalogButton();
      lastDebounceTime_STEP_switch = millis();
      if ( ((firstDigit!=255) && (secondDigit!=255) && (tmpnummer>0)) || ((firstDigit>9) && (tmpnummer>0)) ) {          
        firstDigit=255; secondDigit=255;                          
        lcd.setCursor(_X,1);  lcd.print(F("  "));                   // erase number and enter a new one
      }
      if (tmpnummer>0) {
        if ((firstDigit>0) && (firstDigit<10))
          if (tmpnummer<11) { 
            if (tmpnummer==10) secondDigit=0; else secondDigit = tmpnummer;
            lcd.setCursor(_X+1,1);  lcd.print(secondDigit); delay(500);
          }            
        if (firstDigit==255) { firstDigit = tmpnummer;   lcd.setCursor(_X,1);  lcd.print(firstDigit);  lcd.print(F(" ")); delay(500); } 
      }  
    }
  
    Check_For_ENTER_Press(); 
    if (ENTER_Pressed) {  
      if (firstDigit!=255)  {
        String A="", B="", C="";
        A = String(firstDigit);
        if (secondDigit!=255) { B = String(secondDigit);  C = String(A + B); }
        else { C = String(A); }
        getal = C.toInt(); 
      }
      delay(300);        // Wait some time because enter button was just pressed. If not, next command could be executed
    }
    Check_For_ESCAPE_Press();
    if (ESCAPE_Pressed) getal=0;
  }
  return getal;
}


void Copy_Pattern() {
  bool finish=false, nextstep=false; byte number; 
   
  lcd.clear(); 
  lcd.setCursor(0,0); lcd.print(F("Copy pattern ")); lcd.print(PatternNumber); 
  lcd.setCursor(0,1); lcd.print(F("Destination: "));
  delay(1000);    // Wait some time because enter button was just pressed. If not, next command could be executed 


  while(!finish) {

    number = Get_Two_Digits(12);

    if (ENTER_Pressed) {
      lcd.setCursor(12,1);                            
      if ((number > MaxPatterns) || (number==0)) { 
        lcd.print(F("XX")); delay(1000);
        lcd.setCursor(12,1); lcd.print(F("  "));       // remove 'XX' if it was there
      }
      else {
        lcd.print(F("  "));
        nextstep=true;
      }                       
      ENTER_Pressed=false;
    }

    if (ESCAPE_Pressed) {
      finish=true;
      ESCAPE_Pressed = false;
      lcd.setCursor(0,1); lcd.print(F("Nothing copied  ")); delay(1000);
    }

    if (nextstep) {
      for (byte i=0; i<MaxVoices; i++) {
        DRUM_PATTERN[number-1].Voice[i].NOTE   = DRUM_PATTERN[PatternNumber-1].Voice[i].NOTE;
        DRUM_PATTERN[number-1].Voice[i].ACCENT = DRUM_PATTERN[PatternNumber-1].Voice[i].ACCENT;
        DRUM_PATTERN[number-1].Voice[i].FLAMZ  = DRUM_PATTERN[PatternNumber-1].Voice[i].FLAMZ;
      }
      if (number-1>UsedPatterns) UsedPatterns=number-1;

      lcd.setCursor(0,1); lcd.print(F("Pattern copied  ")); delay(1000);
      finish=true;
      lastmillis-millis();  
    }
  }
  Menu_Item=INSTRUMENT;
  Refresh_LCD();
}


void Shift_Pattern() {
  bool finish=false;   byte _button=0;
  lcd.setCursor(0,1); lcd.print(F("Press Up / Down"));   bool DoYouWantToExit=false;
  
  while (!finish) {
    
    Check_For_ESCAPE_Press();
    if (ESCAPE_Pressed) {
      if (DoYouWantToExit==true) { 
        DoYouWantToExit = false; 
        lcd.setCursor(0,1); lcd.print(F("Press Up / Down "));
      }
      else {
        DoYouWantToExit = true; 
        lcd.setCursor(0,1); lcd.print(F("         Exit? "));
      }    
      ESCAPE_Pressed = false;
    }
    
    Check_For_ENTER_Press(); 
    if (ENTER_Pressed) {  
      if (DoYouWantToExit) {
        finish = true;
        Menu_Item=INSTRUMENT;
        Refresh_LCD();
      }
      ENTER_Pressed=false;
    }

    if ( (millis() - lastDebounceTime_OPTION_button) > debounceDelay) {
      UP_Pressed = false;
      DOWN_Pressed = false;
      _button = readAnalog_MODE_Button();
      lastDebounceTime_OPTION_button = millis();

      switch (_button) {
        case 1: /*UP_Pressed*/    
                  for (byte j=0; j<MaxVoices; j++) {
                    DRUM_PATTERN[PatternNumber-1].Voice[j].NOTE =   DRUM_PATTERN[PatternNumber-1].Voice[j].NOTE >> 1 |   (DRUM_PATTERN[PatternNumber-1].Voice[j].NOTE << (15));        // Bit Rotate right 1 bit
                    DRUM_PATTERN[PatternNumber-1].Voice[j].ACCENT = DRUM_PATTERN[PatternNumber-1].Voice[j].ACCENT >> 1 | (DRUM_PATTERN[PatternNumber-1].Voice[j].ACCENT << (15));
                    DRUM_PATTERN[PatternNumber-1].Voice[j].FLAMZ =  DRUM_PATTERN[PatternNumber-1].Voice[j].FLAMZ >> 1 |  (DRUM_PATTERN[PatternNumber-1].Voice[j].FLAMZ << (15));
                  }
                  set_Pattern_LEDs();  
                break;       
        case 2: /*DOWN_Pressed */ 
                  for (byte j=0; j<MaxVoices; j++) {
                    DRUM_PATTERN[PatternNumber-1].Voice[j].NOTE =   DRUM_PATTERN[PatternNumber-1].Voice[j].NOTE << 1 |   (DRUM_PATTERN[PatternNumber-1].Voice[j].NOTE >> (15));         // Bit Rotate left 1 bit
                    DRUM_PATTERN[PatternNumber-1].Voice[j].ACCENT = DRUM_PATTERN[PatternNumber-1].Voice[j].ACCENT << 1 | (DRUM_PATTERN[PatternNumber-1].Voice[j].ACCENT >> (15));
                    DRUM_PATTERN[PatternNumber-1].Voice[j].FLAMZ =  DRUM_PATTERN[PatternNumber-1].Voice[j].FLAMZ << 1 |  (DRUM_PATTERN[PatternNumber-1].Voice[j].FLAMZ >> (15));
                  }  
                  set_Pattern_LEDs();       
                break;
      }   
      if ((_button>0) && (_button<3) && (DoYouWantToExit)) {
        DoYouWantToExit = false; lcd.setCursor(9,1); lcd.print(F("Next:")); 
      }
    }
  }
}


void Improvise() {
  byte number;  bool nextstep = false, finish = false;
  lcd.setCursor(0,1); lcd.print(F("Enter (1-32): "));
  delay(1000);    // Wait some time because enter button was just pressed. If not, next command could be executed
  
  while (!finish) {

    number = Get_Two_Digits(13);
    
    if (ENTER_Pressed) {  
      lcd.setCursor(13,1);
      if ((number > 32) || (number < 2)) { 
        lcd.print(F("XX")); delay(1000);
        lcd.setCursor(13,1); lcd.print(F("  ")); 
      }
      else {
        lcd.print(F("  "));
        nextstep=true; finish=true;
      }
      ENTER_Pressed=false;
    }

    if (ESCAPE_Pressed) {
      finish=true;
      ESCAPE_Pressed = false;
    }
  }

  if (nextstep) {
    for (byte j=0; j<MaxVoices; j++)
      for (byte k=1; k<=MaxSteps; k++) {
        if (random(number)==1) { 
          WriteNote(PatternNumber-1,j,k,1);
          if (random(number)==1) WriteAccent(PatternNumber-1,j,k,1); else WriteAccent(PatternNumber-1,j,k,0);   
          if (random(number)==1) WriteFlam(PatternNumber-1,j,k,1);   else WriteFlam(PatternNumber-1,j,k,0);  
          if (random(number)==1) Flam[j]=random(11) + 1; else Flam[j]=0; 
        }
      } 
  } 
  Menu_Item=INSTRUMENT;
  Refresh_LCD();  
}


void RandomRhythm() {
  byte number;  bool nextstep = false, finish = false;  
  lcd.setCursor(0,1); lcd.print(F("Enter (1-32): "));
  delay(1000);    // Wait some time because enter button was just pressed. If not, next command could be executed
  
  while (!finish) {

    number = Get_Two_Digits(13);
    
    if (ENTER_Pressed) {  
      lcd.setCursor(13,1);
      if ((number > 32) || (number < 2)) { 
        lcd.print(F("XX"));  delay(1000);
        lcd.setCursor(13,1); lcd.print(F("  ")); 
      }
      else {
        lcd.print(F("  "));
        nextstep=true; finish=true;
      }  
      ENTER_Pressed=false;
    }

    if (ESCAPE_Pressed) {
      finish=true;
      ESCAPE_Pressed = false;
    }
  }

  if (nextstep) {
    lcd.setCursor(0,1); lcd.print(F("Please wait... "));
    for (byte i=PatternNumber-1; i<MaxPatterns; i++)
      for (byte j=0; j<MaxVoices; j++)
        for (byte k=1; k<=MaxSteps; k++) {
          if (random(number)==1) { 
            WriteNote(i,j,k,1);
            if (random(number)==1) WriteAccent(i,j,k,1); else WriteAccent(i,j,k,0);   
            if (random(number)==1) { WriteFlam(i,j,k,1); Flam[j]=random(11); }  else WriteFlam(i,j,k,0);  
          }
          else { WriteNote(i,j,k,0); WriteAccent(i,j,k,0); WriteFlam(i,j,k,0); }
        }
        UsedPatterns = MaxPatterns;
  } 
  Menu_Item=INSTRUMENT;
  Refresh_LCD();   
}


byte readAnalog_MODE_Button() {                                        // Check if one of the eight function keys is pressed:
  byte BUTTONZ = 8 ; int REZOLUTION=1023;                              // UP, DOWN, TRACK, Ptrn Down, PATTERN, Ptrn Up, Voice, MODE
  float avg = REZOLUTION / float(BUTTONZ);
  int val = analogRead(Eight_Buttons_pin);
  if (val > (BUTTONZ - 0.5) * avg)    {  return 0;  }
  for (byte i = 0; i < BUTTONZ; i++) {
    if (val < round((i + 0.5) * avg)) {  return i + 1;   } }
  return 0;
}


void Do_PatternPlay() {
  byte tmpnummer;    bool finish=false;  byte NewPattern=0; 
  byte number, _button;
  bool DoYouWantToExit=false;   byte firstDigit=255, secondDigit=255;

  lcd.setCursor(0,1); lcd.print(F("Ptrn:  ")); lcd.setCursor(5,1); lcd.print(PatternNumber);
  lcd.setCursor(9,1); lcd.print(F("Next:  ")); lcd.setCursor(14,1);
  
  while (!finish ) {

    if ( (millis() - lastDebounceTime_OPTION_button) > debounceDelay) {
      
      _button = readAnalog_MODE_Button();
      lastDebounceTime_OPTION_button = millis();

      switch (_button) {
        case 4: /*Pattern DOWN_Pressed */ 
                if (PatternNumber-1 > 0) PatternNumber--;  break;        
        case 6: /*Pattern UP_Pressed*/    
                if (PatternNumber < UsedPatterns) PatternNumber++; break;       
      }   

      if ((_button==4) || (_button==6)) {
        lcd.setCursor(14,1); lcd.print(F("  "));
        lcd.setCursor(5,1); lcd.print(PatternNumber);  lcd.print(F(" ")); 
        firstDigit=255; secondDigit=255;  
      }
      
      if ((_button>0) && (_button<3) && (DoYouWantToExit)) {
        DoYouWantToExit = false; lcd.setCursor(9,1); lcd.print(F("Next:")); 
      }
    }
    
    if ( (millis() - lastDebounceTime_STEP_switch) > debounceDelay) {                
      tmpnummer = readAnalogButton();
      lastDebounceTime_STEP_switch = millis();
      if (tmpnummer>0) {
         if (DoYouWantToExit) { DoYouWantToExit = false;   lcd.setCursor(9,1); lcd.print(F("Next:")); }
         if ((firstDigit > 0) && (firstDigit<10))
           if (tmpnummer<11) { 
             if (tmpnummer==10) secondDigit = 0; else secondDigit = tmpnummer;
             lcd.setCursor(15,1);  lcd.print(secondDigit);
         }            
         if (firstDigit==255) { firstDigit = tmpnummer;   lcd.setCursor(14,1);  lcd.print(firstDigit);  lcd.print(F(" ")); } 
      }
    }

    Check_For_ENTER_Press(); 
    if (ENTER_Pressed) {  
      if (firstDigit!=255)  {
        String A="", B="", C="";
        A = String(firstDigit);
        if (secondDigit!=255) { B = String(secondDigit);  C = String(A + B); }
        else { C = String(A); }
        number = C.toInt(); 
        lcd.setCursor(14,1);
        if (number > UsedPatterns) { 
          lcd.print(F("XX")); 
        }
        else {
           lcd.print(F("  "));
           NewPattern = number;
        }  
        firstDigit=255; secondDigit=255; 
      }  
      if (DoYouWantToExit) {
        finish = true;
        Menu_Item=INSTRUMENT;
        refresh_it=true;                // Refresh_LCD();
      }
      ENTER_Pressed=false;
    }

    Check_For_ESCAPE_Press(); 
    if (ESCAPE_Pressed){ 
      if (DoYouWantToExit) { 
        DoYouWantToExit = false; 
        lcd.setCursor(9,1); lcd.print(F("Next:  "));
      }
      else if (!DoYouWantToExit) {
        DoYouWantToExit = true; 
        lcd.setCursor(9,1); lcd.print(F("Exit?  "));
      }    
      ESCAPE_Pressed = false;
    } 
    
    if ( (millis() - lastDebounceTime_STARTSTOP_switch) > debounceDelay) {    
      if (digitalRead(STARTSTOP_pin)==LOW) {
        lastDebounceTime_STARTSTOP_switch = millis();

        if (RUN==true) { 
          RUN=false; 
          Serial1.write(midi_stop);
          lcd.setCursor(9,1); lcd.print(F("Next:")); 
          DoYouWantToExit = false;
          firstDigit=255; secondDigit=255;
        } 
        else {
          StepCounter=1;  set_Pattern_LEDs();
          RUN=true;   
          lastmillis=millis();
          Serial1.write(midi_start);
          play_flag = 1;
          TimerInterruptFactor=0;
          PlayNotes=true;
          lcd.setCursor(9,1); lcd.print(F("Next:")); 
          DoYouWantToExit = false;         
        }        
      }
    }  
 
    Check_For_Shuffle();
    Check_For_Flam();
    Check_MIDI_Clock_In();
    Check_Reset();
    if(Read_Tempo_Pot) CheckTempoPotmeter();

    if (PlayNotes) { 
      if (!RESET)  {
        Set_Velocities();
        Walking_LEDs();
        Set_Trigger_Pins();    
        if (RUN) StepCounter++;   
      
        if (StepCounter>LAST_STEP)  {       
          Pattern_Finished();
          if (NewPattern) { 
            PatternNumber = NewPattern;  NewPattern = 0;
            lcd.setCursor(5,1); lcd.print(PatternNumber); lcd.print(F(" "));
            set_Pattern_LEDs(); 
          }
        }
        PlayNotes = false;    
      }
      if (RESET) Reset_Sequencer(); 
    }
    
    if (millis()>trigger_OFF) Set_Triggers_To_Zero();      
  }
}


void Write_Track() {
  byte nextnummer=0, stepnummer=1; bool finish=false; //, erase_XX=false;   
  byte number;   bool DoYouWantToExit=false;

  lcd.setCursor(0,1); lcd.print(F("Step:")); lcd.print(stepnummer); lcd.print(F("   Ptrn:"));
  delay(1000);

  while (!finish){
 
    number = Get_Two_Digits(14);
        
    if (ENTER_Pressed) {   

      if (!DoYouWantToExit) {
        lcd.setCursor(14,1);
        if ((number>UsedPatterns) || (number==0)) { 
          lcd.print(F("XX")); delay(1000);        
          lcd.setCursor(14,1); lcd.print(F("  ")); 
        }        
        else {
          Rhythm_Track[stepnummer-1] = number;
          lcd.setCursor(14,1); lcd.print(F("  "));
          lcd.setCursor(14,1); lcd.print(Rhythm_Track[stepnummer-1]);
          if (stepnummer < MaxComposition) stepnummer++;
          lcd.setCursor(5,1); lcd.print(stepnummer);
          lcd.setCursor(14,1); lcd.print(F("  "));
        }  
      }
      if (DoYouWantToExit) {
        finish = true;
        Menu_Item=INSTRUMENT;     
        for (byte i=stepnummer-1; i<MaxComposition; i++) Rhythm_Track[i]=0;
        Refresh_LCD(); 
      }  
    }

    if (ESCAPE_Pressed){      
      if (DoYouWantToExit) { 
        DoYouWantToExit = false; 
        lcd.setCursor(9,1); lcd.print(F("Next:"));
      }
      else if (!DoYouWantToExit) {
        DoYouWantToExit = true; 
        lcd.setCursor(9,1); lcd.print(F("Exit?  "));
      }    
      ESCAPE_Pressed = false;
    }    
  }
}


void Edit_Track() {
  byte nextnummer=0, stepnummer=1; bool finish=false;  
  byte firstDigit=255, secondDigit=255;   byte _button;
  String A="",B="", C="";  bool DoYouWantToExit=false;
  lcd.setCursor(0,1); lcd.print(F("Step:")); lcd.print(stepnummer); lcd.print(F("   Ptrn:")); lcd.print(Rhythm_Track[stepnummer-1]);
  bool Insert_Pressed = false, Delete_Pressed = false;  
  byte number; 
  
  while (!finish){

    if ( (millis() - lastDebounceTime_OPTION_button) > debounceDelay) {
      _button = readAnalog_MODE_Button();
      lastDebounceTime_OPTION_button = millis();

      switch (_button) {
        case 1: /*UP_Pressed*/    
                 if (Rhythm_Track[stepnummer]>0); { 
                    if (stepnummer < MaxComposition) stepnummer++;                 // Cannot be higher than MaxComposition
                    lcd.setCursor(5,1); lcd.print(stepnummer);
                 }  break;       
        case 2: /*DOWN_Pressed */ 
                 if (stepnummer-1 > 0) { 
                    stepnummer--; 
                    lcd.setCursor(5,1); lcd.print(stepnummer); lcd.print(F(" "));
                 }  break;
        case 4:  Delete_Pressed = true;  Insert_Pressed = false; 
                 lcd.setCursor(12,0); lcd.print(F("Del?"));                
                 break;                  
        case 6:  Insert_Pressed = true; Delete_Pressed = false;
                 lcd.setCursor(12,0); lcd.print(F("Ins?"));
                 break;
      }
         
      if ((_button==1) || (_button==2) || (_button==4) || (_button==6)) {
        firstDigit=255; secondDigit=255;  
        lcd.setCursor(14,1); lcd.print(Rhythm_Track[stepnummer-1]);  lcd.print(F(" ")); 
      }
      if ((_button==1) || (_button==2)) {
        if ((Insert_Pressed) || (Delete_Pressed)) {
          Delete_Pressed = false;  Insert_Pressed = false;
          lcd.setCursor(12,0); lcd.print(F("    "));        
        } 
      }   
      if ((_button>0) && (DoYouWantToExit)) {
        DoYouWantToExit = false;  lcd.setCursor(9,1); lcd.print(F("Ptrn:"));  
      }
    }

    if ( (millis() - lastDebounceTime_STEP_switch) > debounceDelay) {                
      byte tmpnummer = readAnalogButton();
      lastDebounceTime_STEP_switch = millis();
      if (tmpnummer>0) {
         if ((firstDigit > 0) && (firstDigit<10))
           if (tmpnummer<11) { 
             if (tmpnummer==10) secondDigit = 0; else secondDigit = tmpnummer;
             lcd.setCursor(15,1);  lcd.print(secondDigit);
         } 
         if (firstDigit==255) { firstDigit = tmpnummer;   lcd.setCursor(14,1);  lcd.print(firstDigit);  lcd.print(F(" "));} 
         Delete_Pressed = false;  Insert_Pressed = false;  lcd.setCursor(12,0); lcd.print(F("    "));
         if (DoYouWantToExit) { DoYouWantToExit = false;  lcd.setCursor(9,1); lcd.print(F("Ptrn:"));  }
      }
    }

    Check_For_ENTER_Press(); 
    if (ENTER_Pressed) {  
      
      if (Insert_Pressed) { 
        for (int i=MaxComposition-1; i>=stepnummer; i--)  Rhythm_Track[i] = Rhythm_Track[i-1];   
        Rhythm_Track[stepnummer-1] = 0;
      }

      if (Delete_Pressed) {
        for (int i=stepnummer-1; i<MaxComposition; i++) Rhythm_Track[i] = Rhythm_Track[i+1];
      }

      if ((Delete_Pressed) || (Insert_Pressed)) {
        lcd.setCursor(12,0); lcd.print(F("    "));  
        lcd.setCursor(14,1); lcd.print(Rhythm_Track[stepnummer-1]);  lcd.print(F(" "));
        Insert_Pressed = false; Delete_Pressed = false;
      }

      if (firstDigit!=255)  {
        A = String(firstDigit);
        if (secondDigit!=255) {B = String(secondDigit);  C = String(A + B); }
        else {C = String(A); }
        number = C.toInt(); 
        lcd.setCursor(14,1);
        if (number>UsedPatterns) { 
          lcd.print(F("XX")); delay(1000);
          lcd.setCursor(14,1); lcd.print(Rhythm_Track[stepnummer-1]); lcd.print(" ");
        }
        else {
          Rhythm_Track[stepnummer-1] = number;
          lcd.setCursor(14,1); lcd.print(F("  "));  delay(300);
          lcd.setCursor(14,1); lcd.print(Rhythm_Track[stepnummer-1]);
        }  
        firstDigit=255; secondDigit=255;
      }
      
      if (DoYouWantToExit) {  
        finish = true;
        Menu_Item=INSTRUMENT;
        Refresh_LCD();
      }
      ENTER_Pressed=false;
    }

    Check_For_ESCAPE_Press();  
    if (ESCAPE_Pressed){
      if ((Insert_Pressed) || (Delete_Pressed)){
        Insert_Pressed=false;
        Delete_Pressed=false;
        lcd.setCursor(12,0); lcd.print(F("    "));
      }
      else if (DoYouWantToExit) { 
        DoYouWantToExit = false; 
        lcd.setCursor(9,1); lcd.print(F("Ptrn:"));  lcd.print(Rhythm_Track[stepnummer-1]);
      }
      else if (!DoYouWantToExit) {
        DoYouWantToExit = true; 
        lcd.setCursor(9,1); lcd.print(F("Exit?  "));
        firstDigit=255; secondDigit=255;
      }
      ESCAPE_Pressed = false;     
    }
  }
}


void WriteNote(byte measure, byte instru, byte Step, byte value) {            // Write the bytes into the NOTE variable
  if (value) bitSet(DRUM_PATTERN[measure].Voice[instru].NOTE,16-Step);   
  else       bitClear(DRUM_PATTERN[measure].Voice[instru].NOTE,16-Step); 
}


void WriteAccent(byte measure, byte instru, byte Step, byte value) {          // Write the bytes into the ACCENT variable
  if (value) bitSet(DRUM_PATTERN[measure].Voice[instru].ACCENT,16-Step);   
  else       bitClear(DRUM_PATTERN[measure].Voice[instru].ACCENT,16-Step);    
}


void WriteFlam(byte measure, byte instru, byte Step, byte value) {            // Write the bytes into the FLAMZ variable
  if (value) bitSet(DRUM_PATTERN[measure].Voice[instru].FLAMZ,16-Step);   
  else       bitClear(DRUM_PATTERN[measure].Voice[instru].FLAMZ,16-Step); 
}


byte ReadNote(byte measure, byte instru, byte Step) {                         // Read the bytes from the NOTE variable
  byte value;
  value = bitRead(DRUM_PATTERN[measure].Voice[instru].NOTE,16-Step);
  return value;
}


byte ReadAccent(byte measure, byte instru, byte Step) {                       // Read the bytes from the ACCENT variable
  byte value;
  value = bitRead(DRUM_PATTERN[measure].Voice[instru].ACCENT,16-Step);
  return value;
}


byte ReadFlam(byte measure, byte instru, byte Step) {                         // Read the bytes from the FLAMZ variable
  byte value;
  value = bitRead(DRUM_PATTERN[measure].Voice[instru].FLAMZ,16-Step);
  return value;
}


void Check_For_ENTER_Press() {
  if ( (millis() - lastDebounceTime_ENTER_switch) > debounceDelay) {          // Check if ENTER button is pressed
    if (digitalRead(ENTER_pin)==LOW) {
      lastDebounceTime_ENTER_switch = millis();
      ENTER_Pressed = true;
    }
  }
}


void Check_For_ESCAPE_Press() {
  if ( (millis() - lastDebounceTime_ESCAPE_switch) > debounceDelay) {         // Check if ESCAPE button is pressed
    if (digitalRead(ESCAPE_pin)==LOW) {
      lastDebounceTime_ESCAPE_switch = millis();
      ESCAPE_Pressed = true;
    }
  }
}


byte readEEPROM(long eeaddress )                   //  Read DrumTrack data from the EEPROM
{
  byte rdata = 0xFF;

  if (eeaddress < 65536) Wire.beginTransmission(disk1);
  else                   Wire.beginTransmission(disk2);

  Wire.write((int)(eeaddress >> 8)); // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();
  
  if (eeaddress < 65536) Wire.requestFrom(disk1,1);
  else                   Wire.requestFrom(disk2,1);

  if (Wire.available()) rdata = Wire.read();  
  return rdata;
}


void  LoadFileNr(byte Z)                                                                                         
{
  CurrentTrackNumber=Z;
  address = Z-1;  LAST_STEP=readEEPROM(address);
  address = 64 + address*((MaxPatterns*MaxVoices*6) + 4 * MaxVoices + MaxComposition + 3 );

  BPM    = (readEEPROM(address + 1) << 8); 
  BPM   |= readEEPROM(address);  address+=2; 
  MeasureTime = 4*60000/BPM;    
  BPM_Factor = (240000)/BPM;  TCB2.CCMP = BPM_Factor;                         // adjust Timer Interrupt
  UsedPatterns = readEEPROM(address); address++;

  for (byte i=0; i<UsedPatterns; i++)
    for (byte j=0; j<MaxVoices; j++) {
      
      DRUM_PATTERN[i].Voice[j].NOTE    = (readEEPROM(address + 1) << 8); 
      DRUM_PATTERN[i].Voice[j].NOTE   |= readEEPROM(address);   address+=2;

      DRUM_PATTERN[i].Voice[j].ACCENT  = (readEEPROM(address + 1) << 8);
      DRUM_PATTERN[i].Voice[j].ACCENT |= readEEPROM(address);   address+=2;

      DRUM_PATTERN[i].Voice[j].FLAMZ   = (readEEPROM(address + 1) << 8);
      DRUM_PATTERN[i].Voice[j].FLAMZ  |= readEEPROM(address);   address+=2;
    } 
    
  for (byte i=UsedPatterns; i<MaxPatterns; i++)
    for (byte j=0; j<MaxVoices; j++) {
      DRUM_PATTERN[i].Voice[j].NOTE    = 0;  address+=2;
      DRUM_PATTERN[i].Voice[j].ACCENT  = 0;  address+=2;
      DRUM_PATTERN[i].Voice[j].FLAMZ   = 0;  address+=2;
    } 
        
    for (byte i=0; i<MaxVoices; i++) {  Shuffle[i] = readEEPROM(address);            address++;   }
    for (byte i=0; i<MaxVoices; i++) {  Flam[i] = readEEPROM(address);               address++;   }  
    for (byte i=0; i<MaxVoices; i++) {  Accent_HI[i] = readEEPROM(address);          address++;   }
    for (byte i=0; i<MaxVoices; i++) {  Accent_LO[i] = readEEPROM(address);          address++;   }   
    for (byte i=0; i<MaxComposition; i++) {  Rhythm_Track[i] = readEEPROM(address);  address++;   }      
}


void Load_Track(){
  bool  finish=false, nextstep=false;  byte number;
  
  lcd.setCursor(0,1); lcd.print(F("No (1-"));  lcd.print(MaxTracks);  lcd.print(F("):"));
  lcd.setCursor(12,0); lcd.print(F("("));   lcd.print(CurrentTrackNumber);   lcd.print(F(")"));
  delay(1000);     // Wait some time because enter button was just pressed. If not, next command could be executed 

  while(!finish) {

    number = Get_Two_Digits(10);
       
    if (ENTER_Pressed) { 
 
      lcd.setCursor(10,1);
      if ((number > MaxTracks) || (number==0)) {                   // number does not exist
        lcd.print(F("XX")); delay(1000);
        lcd.setCursor(10,1); lcd.print(F("  "));             
      }                   
      else {
        lcd.print(F("  "));
        if (readEEPROM(number-1)==0) { 
          lcd.setCursor(0,1); lcd.print(F("File not found"));      // file does not exist, tell so
            
          ENTER_Pressed=false;
          while ((!ENTER_Pressed) && (!ESCAPE_Pressed)) {
            Check_For_ENTER_Press(); Check_For_ESCAPE_Press();                     
            if ((ENTER_Pressed) || (ESCAPE_Pressed)) finish=true; 
          }
        }  else nextstep=true;                                     // file does exist, so it can be opened
      }                 
      ENTER_Pressed=false;
    }

    if (ESCAPE_Pressed) {
      finish=true;
    }

    if (nextstep) {  
      lcd.clear();  lcd.print(F("Load File"));  
      lcd.setCursor(0,1); lcd.print(F("Please Wait..."));
      LoadFileNr(number);   PatternNumber=1;                                    // load the file and start at the beginning of pattern 1
      lastmillis=millis();  Pattern_Finished();                                 // When omitted, flam and/or shuffle will sound strange during one measure 
      finish=true;     
      for (byte i=0; i<MaxVoices; i++) Mute[i]=0;
    }    
  }

  ESCAPE_Pressed=false;  ENTER_Pressed=false;
  Menu_Item=INSTRUMENT;
  Refresh_LCD();
  set_Pattern_LEDs();
}


void writeEEPROM(long eeaddress, byte data )       //  Write DrumTrack data to EEprom
{
  if (eeaddress < 65536) { Wire.beginTransmission(disk1);  eeaddress &= 0xFFFF; }  //Erase the first 16 bits of the long variable  
  else                     Wire.beginTransmission(disk2);
  
  Wire.write((int)(eeaddress >> 8)); // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.write(data);
  Wire.endTransmission();
  delay(5);
}


void SaveFileNr(byte Z)
{
  CurrentTrackNumber=Z;
  address = Z-1;   writeEEPROM(address, LAST_STEP);
  address = 64 + address*((MaxPatterns*MaxVoices*6) + 4 * MaxVoices + MaxComposition + 3 );
    
  writeEEPROM(address, BPM);    
  writeEEPROM(address + 1, BPM >> 8);  address+=2;
  writeEEPROM(address, UsedPatterns);  address++;
  
  for (byte i=0; i<UsedPatterns; i++)  
    for (byte j=0; j<MaxVoices; j++) {
      writeEEPROM(address, DRUM_PATTERN[i].Voice[j].NOTE);    
      writeEEPROM(address + 1, DRUM_PATTERN[i].Voice[j].NOTE >> 8);    address+=2;  

      writeEEPROM(address, DRUM_PATTERN[i].Voice[j].ACCENT);
      writeEEPROM(address + 1, DRUM_PATTERN[i].Voice[j].ACCENT >> 8);  address+=2; 
         
      writeEEPROM(address, DRUM_PATTERN[i].Voice[j].FLAMZ);        
      writeEEPROM(address + 1, DRUM_PATTERN[i].Voice[j].FLAMZ >> 8);   address+=2;  
    }

  for (byte i=UsedPatterns; i<MaxPatterns; i++)  
    for (byte j=0; j<MaxVoices; j++) {
      address+=2;  
      address+=2; 
      address+=2;  
    }

  for (byte i=0; i<MaxVoices; i++) {  writeEEPROM(address, Shuffle[i]);            address++;   } 
  for (byte i=0; i<MaxVoices; i++) {  writeEEPROM(address, Flam[i]);               address++;   }  
  for (byte i=0; i<MaxVoices; i++) {  writeEEPROM(address, Accent_HI[i]);          address++;   }
  for (byte i=0; i<MaxVoices; i++) {  writeEEPROM(address, Accent_LO[i]);          address++;   }   
  for (byte i=0; i<MaxComposition; i++) {  writeEEPROM(address, Rhythm_Track[i]);  address++;   }
}


void Save_Track() {
  byte number;  bool finish=false, nextstep=false; 
  
  lcd.setCursor(0,1); lcd.print(F("No (1-"));   lcd.print(MaxTracks);  lcd.print(F("):"));
  lcd.setCursor(12,0); lcd.print(F("("));   lcd.print(CurrentTrackNumber);   lcd.print(F(")"));
  delay(1000);    // Wait some time because enter button was just pressed. If not, next command could be executed
  
  while(!finish) {
   
    number = Get_Two_Digits(10);    
    
    if (ENTER_Pressed) {   
      lcd.setCursor(10,1);
      if ((number > MaxTracks) || (number==0)) {
        lcd.print(F("XX")); delay(1000);
        lcd.setCursor(10,1); lcd.print(F("  ")); 
      }                                          
      else {
        lcd.print(F("  "));
        if (readEEPROM(number-1)) { 
          lcd.setCursor(0,1); lcd.print(F("Overwrite ")); lcd.print(number); lcd.print(F("?")); delay(1000);
          ENTER_Pressed=false;
          while ((!ENTER_Pressed) && (!ESCAPE_Pressed)) {
            Check_For_ENTER_Press(); Check_For_ESCAPE_Press();                     
            if (ENTER_Pressed) nextstep=true;                                          // if file already exists, ask to overwrite 
          }
        } else nextstep=true;                                                          // file does not exist yet, so save
      }  
      ENTER_Pressed=false;
    }

    if (ESCAPE_Pressed) {
      finish=true;
      lcd.setCursor(0,1); lcd.print(F("Nothing saved  ")); delay(1000);
    }

    if (nextstep) { 
      lcd.clear();  lcd.print(F("Save File"));  lcd.setCursor(0,1); lcd.print(F("Please Wait..."));
      SaveFileNr(number); finish=true;
    }    
  }

  ESCAPE_Pressed=false;  ENTER_Pressed=false;
  Menu_Item=INSTRUMENT;
  Refresh_LCD();
  lastmillis=millis();                       // When omitted, flam and/or shuffle will sound strange during one measure
}


void Clear_Track(){
byte i,j;  

  for (i=0; i<MaxPatterns; i++)
    for (j=0; j<MaxVoices; j++) {
      DRUM_PATTERN[i].Voice[j].NOTE = 0;
      DRUM_PATTERN[i].Voice[j].ACCENT = 0;
      DRUM_PATTERN[i].Voice[j].FLAMZ = 0;  
    }
  for (i=0; i<MaxVoices; i++) {
    Shuffle[i]=0;
    Flam[i]=0;
    Accent_HI[i]=10;
    Accent_LO[i]=5;
    Mute[i]=0;
  }
  for (i=0; i<MaxComposition; i++) Rhythm_Track[i]=0;
  Mute_Voice_Mode=false;
  Tap_Mode=false;
  BPM=120;   MeasureTime = 4*60000/BPM;
  TCB2.CCMP = 2000;                         // set timer interrupt to 120 BPM
  set_Pattern_LEDs();
  LAST_STEP=MaxSteps;
  Menu_Item=INSTRUMENT;
  PatternNumber=1;  UsedPatterns=1;
  Refresh_LCD();
}


void Last_Step(){
  bool finish=false; byte laststep=0, tmpLastStep=0;  
  lcd.setCursor(0,1); lcd.print(F("Enter Step:"));
  
  while ((!finish) && (!ESCAPE_Pressed)){

    while ((!ENTER_Pressed) && (!ESCAPE_Pressed) && (laststep==0)) {
      if ( (millis() - lastDebounceTime_STARTSTOP_switch) > debounceDelay) {          // BORROW STARTSTOP DEBOUNCE VARIABLE      
        tmpLastStep = readAnalogButton();
        lastDebounceTime_STARTSTOP_switch = millis();
        if (tmpLastStep>0) { laststep=tmpLastStep; lcd.print(laststep); } 
      }      
    } 
     
    Check_For_ENTER_Press(); Check_For_ESCAPE_Press();

    if (ENTER_Pressed) {
      if (laststep>0) LAST_STEP = laststep;
      finish=true;
      ENTER_Pressed=false;
      Menu_Item=INSTRUMENT;
      Refresh_LCD();
    }

    if (ESCAPE_Pressed){
      Menu_Item=INSTRUMENT;
      Refresh_LCD();
      ESCAPE_Pressed=false;
      finish=true;
    }
  }
}
  

void set_Pattern_LEDs() {
  if (!Mute_Voice_Mode) {
    for (byte i=1; i<=MaxSteps; i++) {
       NoteBit = ReadNote(PatternNumber-1, VoiceNumber-1, i);    
       AccentBit = ReadAccent(PatternNumber-1, VoiceNumber-1, i); 
       if (NoteBit==1) {
         if (AccentBit==0) Tlc_set(15+i,LED_LO); 
         if (AccentBit==1) Tlc_set(15+i,LED_HI);
       } else Tlc_set(15+i,0);
    }
  }
  Tlc_update();
}


void SetPin(byte PinNummer, byte state) {
  
  #if (!TR_909_HIHAT)
    if (!Mute[PinNummer-1]) {
      switch (PinNummer) {
        case 1:  if (state) bitSet(getalB,7); else bitClear(getalB,7); break;
        case 2:  if (state) bitSet(getalB,6); else bitClear(getalB,6); break;
        case 3:  if (state) bitSet(getalB,5); else bitClear(getalB,5); break;
        case 4:  if (state) bitSet(getalB,4); else bitClear(getalB,4); break;
        case 5:  if (state) bitSet(getalB,3); else bitClear(getalB,3); break;
        case 6:  if (state) bitSet(getalB,2); else bitClear(getalB,2); break;
        case 7:  if (state) bitSet(getalB,1); else bitClear(getalB,1); break;
        case 8:  if (state) bitSet(getalB,0); else bitClear(getalB,0); break;   // OH
        case 9:  if (state) bitSet(getalA,7); else bitClear(getalA,7); break;   // CH
        case 10: if (state) bitSet(getalA,6); else bitClear(getalA,6); break;
        case 11: if (state) bitSet(getalA,5); else bitClear(getalA,5); break;
        case 12: if (state) bitSet(getalA,4); else bitClear(getalA,4); break;
        case 13: if (state) bitSet(getalA,3); else bitClear(getalA,3); break;
        case 14: if (state) bitSet(getalA,2); else bitClear(getalA,2); break;
        case 15: if (state) bitSet(getalA,1); else bitClear(getalA,1); break;
        case 16: if (state) bitSet(getalA,0); else bitClear(getalA,0); break;
      }
    }
  #endif
  
  #if (TR_909_HIHAT)
    if (!Mute[PinNummer-1]) {
      switch (PinNummer) {
        case 1:  if (state) bitSet(getalB,7); else bitClear(getalB,7); break;
        case 2:  if (state) bitSet(getalB,6); else bitClear(getalB,6); break;
        case 3:  if (state) bitSet(getalB,5); else bitClear(getalB,5); break;
        case 4:  if (state) bitSet(getalB,4); else bitClear(getalB,4); break;
        case 5:  if (state) bitSet(getalB,3); else bitClear(getalB,3); break;
        case 6:  if (state) bitSet(getalB,2); else bitClear(getalB,2); break;
        case 7:  if (state) bitSet(getalB,1); else bitClear(getalB,1); break;
        
        case 10: if (state) bitSet(getalA,6); else bitClear(getalA,6); break;
        case 11: if (state) bitSet(getalA,5); else bitClear(getalA,5); break;
        case 12: if (state) bitSet(getalA,4); else bitClear(getalA,4); break;
        case 13: if (state) bitSet(getalA,3); else bitClear(getalA,3); break;
        case 14: if (state) bitSet(getalA,2); else bitClear(getalA,2); break;
        case 15: if (state) bitSet(getalA,1); else bitClear(getalA,1); break;
        case 16: if (state) bitSet(getalA,0); else bitClear(getalA,0); break;
      }
    }

    if (PinNummer==8) if (state) bitSet(getalB,0); else bitClear(getalB,0);   // OH
    if (PinNummer==9) if (state) bitSet(getalA,7); else bitClear(getalA,7);   // CH
     
  #endif
    
}


void WritePins() {
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, LSBFIRST, getalA);  
  shiftOut(dataPin, clockPin, LSBFIRST, getalB); 
  digitalWrite(latchPin, HIGH);
  trigger_OFF=millis()+5; 
}


byte readAnalogButton() {                                                                                  // which of the sixteen buttons is pressed?
  float avg = RESOLUTION / float(BUTTONS);
  int val = analogRead(STEP_pin);
  if (val > (BUTTONS - 0.5) * avg)    {  return 0;  }
  for (byte i = 0; i < BUTTONS; i++) {
    if (val < round((i + 0.5) * avg)) {  return i + 1;   } }
  return 0;
}


void Check_The_Sixteen_Buttons()  {
  
  if ( (millis() - lastDebounceTime_STEP_switch) > debounceDelay) {                                        // check if one of the sixteen buttons are pressed: 
    STEP = readAnalogButton();
    lastDebounceTime_STEP_switch = millis();

    if (STEP>0) {

      if (!Select_instru) {
        if ((Flam[VoiceNumber-1]) && (Add_flam)) {
          NoteBit   = ReadNote(  PatternNumber-1, VoiceNumber-1, STEP);                                    // there is flam and we want to add it
          AccentBit = ReadAccent(PatternNumber-1, VoiceNumber-1, STEP);                                    
               if ((NoteBit==0) && (AccentBit==0)) { WriteNote(  PatternNumber-1,VoiceNumber-1,STEP,1); WriteFlam(PatternNumber-1,VoiceNumber-1,STEP,1);                                                    Tlc_set(16+STEP-1,LED_LO);  }  
          else if ((NoteBit==1) && (AccentBit==0)) { WriteAccent(PatternNumber-1,VoiceNumber-1,STEP,1); WriteFlam(PatternNumber-1,VoiceNumber-1,STEP,1);                                                    Tlc_set(16+STEP-1,LED_HI); }   
          else if ((NoteBit==1) && (AccentBit==1)) { WriteNote(PatternNumber-1,VoiceNumber-1,STEP,0);   WriteAccent(PatternNumber-1,VoiceNumber-1,STEP,0); WriteFlam(PatternNumber-1,VoiceNumber-1,STEP,0); Tlc_set(16+STEP-1,0);   } 
        }
        else { 
          NoteBit   = ReadNote(  PatternNumber-1, VoiceNumber-1, STEP);
          AccentBit = ReadAccent(PatternNumber-1, VoiceNumber-1, STEP);                                    // there is no flam
               if ((NoteBit==0) && (AccentBit==0)) { WriteNote(PatternNumber-1,VoiceNumber-1,STEP,1);                                                     Tlc_set(16+STEP-1,LED_LO);  }   // Low accent
          else if ((NoteBit==1) && (AccentBit==0)) { WriteAccent(PatternNumber-1,VoiceNumber-1,STEP,1);                                                   Tlc_set(16+STEP-1,LED_HI); }    // High accent               
          else if ((NoteBit==1) && (AccentBit==1)) { WriteNote(PatternNumber-1,VoiceNumber-1,STEP,0); WriteAccent(PatternNumber-1,VoiceNumber-1,STEP,0);  Tlc_set(16+STEP-1,0);   }       // No note
          WriteFlam(PatternNumber-1,VoiceNumber-1,STEP,0);  // set flambit to 0
        }
        Tlc_update(); 
      }  

      if (Select_instru) {                                                                                 // if SELECT button was pressed, select a new voice
        Select_instru=false;
        if (STEP<=MaxVoices) {
          VoiceNumber=STEP;
          set_Pattern_LEDs();
          WaitUntil = millis() + 1000;  VoiceChanged=true;                                                 // prevents that a note is written right after changing a voice 
        }
        refresh_it = true;
      }
    }   
  } 
}


void Check_Start_Stop_Button() {
  if ( (millis() - lastDebounceTime_STARTSTOP_switch) > debounceDelay) {                                   //----------------------START / STOP--------------------------
    if (digitalRead(STARTSTOP_pin)==LOW) {
      lastDebounceTime_STARTSTOP_switch = millis();
      
      if (RUN==true) { 
        RUN=false; 
        Serial1.write(midi_stop);
      } 
      else {
        StepCounter=1;  set_Pattern_LEDs();  RUN=true;
        lastmillis=millis();                              // when omitted: MeasureTime will not be correct
        Serial1.write(midi_start);
        play_flag = 1;
        TimerInterruptFactor=0;
        PlayNotes=true;
      }
    }
  }   
}


void Check_Select_Button() {
  if ( (millis() - lastDebounceTime_ENTER_switch) > debounceDelay) {                                   //----------------------SELECT button--------------------------
    if (digitalRead(SELECT_pin)==LOW) {
      lastDebounceTime_ENTER_switch = millis();                                                        // use the lastDebouncetime from the Start/Stop button

      if (Menu_Item==TEMPO) {
        if (Read_Tempo_Pot) Read_Tempo_Pot=false; else Read_Tempo_Pot=true;
      }
      else {
        switch (Menu_Item) {
          case SHUFFLE:
          case FLAM:
          case ACCENT_HI:
          case ACCENT_LO: break;
          default: Menu_Item=INSTRUMENT;  break;                                        
        }
        if (!Mute_Voice_Mode) Select_instru = true;
        Add_flam = false;
      }
      refresh_it = true;
    }
  }   
}


void Print_SelectInstru_or_Instrument() {
  if (Select_instru) 
    { if (Tap_Mode) lcd.print(F("Erase voice:  ")); else lcd.print(F("Select voice:  ")); }
  else print_instrument(VoiceNumber-1);  
}


void Refresh_LCD() {
 
  lcd.clear();  
  switch (Menu_Item) {    
     case INSTRUMENT:     Print_SelectInstru_or_Instrument(); 
                          lcd.setCursor(2,1);  
                          if (Add_flam) lcd.print(F("Add flam    ")); 
                          else { lcd.print(F("Pattern:")); lcd.print(PatternNumber); }                              break; 
                                              
     case TEMPO:          lcd.setCursor(0,0);  lcd.print(F("Tempo")); if (Read_Tempo_Pot) lcd.print(F(" @"));
                          lcd.setCursor(0,1);  lcd.print(BPM); lcd.print(F(" BPM"));                                break;
                      
     case SHUFFLE:        Print_SelectInstru_or_Instrument(); 
                          lcd.setCursor(0,1);  lcd.print(F("=>Shuffle:")); lcd.print(Shuffle[VoiceNumber-1]);       break;
                       
     case FLAM:           Print_SelectInstru_or_Instrument(); 
                          lcd.setCursor(0,1);  lcd.print(F("=>Flam:")); lcd.print(Flam[VoiceNumber-1]);             break;
                         
     case MUTE_VOICE:     Print_SelectInstru_or_Instrument();
                          lcd.setCursor(0,1);  lcd.print(F("=>Mute Mode ")); 
                          if (Mute_Voice_Mode) lcd.print(F("off")); else lcd.print(F("on"));                        break; 
                         
     case ACCENT_HI:      Print_SelectInstru_or_Instrument(); 
                          lcd.setCursor(0,1);  lcd.print(F("=>Accent Hi:")); lcd.print(Accent_HI[VoiceNumber-1]);   break;
                          
     case ACCENT_LO:      Print_SelectInstru_or_Instrument(); 
                          lcd.setCursor(0,1);  lcd.print(F("=>Accent Lo:")); lcd.print(Accent_LO[VoiceNumber-1]);   break;

     case LOAD:           lcd.print(F("Load Track"));        break;
     case SAVE:           lcd.print(F("Save Track"));        break;
     case CLEAR_TRACK:    lcd.print(F("Clear Track"));       break;      
     case TRACK_PLAY:     lcd.print(F("Play Track"));        break;  
     case WRITE_TRACK:    lcd.print(F("Write Track"));       break; 
     case EDIT_TRACK:     lcd.print(F("Edit Track"));        break;
     case RANDOM_TRACK:   lcd.print(F("Random Track"));      break;
     case IMPROVISE:      lcd.print(F("Improvise"));         break;     
     case ERASE_EEPROM:   lcd.print(F("Erase EEPROM"));      break;
     case CLOCK_SOURCE:   lcd.print(F("Clock source"));      break;     
     case SHOW_BPM:       lcd.print(F("BPM Monitor"));       break;
     case COPY_PATTERN:   lcd.print(F("Copy Pattern"));      break;  
     case SHIFT_PATTERN:  lcd.print(F("Shift Pattern"));     break;
     case DELETE_PATTERN: lcd.print(F("Delete Pattern"));    break;  
     case INSERT_PATTERN: lcd.print(F("Insert Pattern"));    break;       
     case PATTERN_PLAY:   lcd.print(F("Pattern Play"));      break;
     case MIDI_IN:        lcd.print(F("MIDI In"));           break;
     case MIDI_OUT:       lcd.print(F("MIDI Out"));          break;
     case TAP_MODE:       lcd.print(F("Tap Mode ")); if (Tap_Mode) lcd.print(F("off")); else lcd.print(F("on"));  break;     
     case LASTSTEP:       lcd.print(F("Last Step"));         break; 
  }
}


void Check_The_Option_Buttons() {
  byte Mode_Button_NR;
  if ( (millis() - lastDebounceTime_OPTION_button) > debounceDelay) {
    Mode_Button_NR = readAnalog_MODE_Button();
    lastDebounceTime_OPTION_button = millis();  byte x; bool PatternNotEmpty=false;
    
    if (Select_instru) {                                    // skip first 3 menu items when SELECT was pressed
      if (Mode_Button_NR==3) Menu_Item=CLEAR_TRACK;
      if (Mode_Button_NR==5) Menu_Item=SHIFT_PATTERN;
      if (Mode_Button_NR==7) Menu_Item=MUTE_VOICE;      
      if (Mode_Button_NR==8) Menu_Item=MIDI_OUT;
    }
    if (Mode_Button_NR>0) { Select_instru=false; Add_flam=false; }

    if ((Mode_Button_NR==7) && (Mute_Voice_Mode) && (Menu_Item==INSTRUMENT)) Menu_Item=FLAM;    
    // if VOICE menu pressed and MUTE_VOICE_MODE: first Menu option is to disable it

    if ((Mode_Button_NR==8) && (Tap_Mode) && (Menu_Item==INSTRUMENT)) Menu_Item=CLOCK_SOURCE;    
    // if MODE menu pressed and TAP_MODE: first Menu option is to disable it
    
    switch (Mode_Button_NR) {
      case 1: UP_Pressed = true;    break;
      case 2: DOWN_Pressed = true;  break;          
      case 3: switch(Menu_Item) {
                case TRACK_PLAY    : Menu_Item=LOAD;             break;
                case LOAD          : Menu_Item=CLEAR_TRACK;      break;
                case CLEAR_TRACK   : Menu_Item=SAVE;             break;
                case SAVE          : Menu_Item=WRITE_TRACK;      break; 
                case WRITE_TRACK   : Menu_Item=EDIT_TRACK;       break;
                case EDIT_TRACK    : Menu_Item=RANDOM_TRACK;     break;   
                case RANDOM_TRACK  : Menu_Item=TRACK_PLAY;       break;                                                 
                default            : Menu_Item=TRACK_PLAY;       break;
              }
              refresh_it=true;                                   break;
              
      case 4: if (PatternNumber>1)  PatternNumber--; 
              set_Pattern_LEDs();   refresh_it=true;             break;  
                     
      case 5: switch(Menu_Item) {
                case COPY_PATTERN   : Menu_Item=PATTERN_PLAY;    break;
                case PATTERN_PLAY   : Menu_Item=SHIFT_PATTERN;   break;
                case SHIFT_PATTERN  : Menu_Item=LASTSTEP;        break;
                case LASTSTEP       : Menu_Item=INSERT_PATTERN;  break;
                case INSERT_PATTERN : Menu_Item=DELETE_PATTERN;  break;
                case DELETE_PATTERN : Menu_Item=IMPROVISE;       break;  
                case IMPROVISE      : Menu_Item=COPY_PATTERN;    break;                               
                default             : Menu_Item=COPY_PATTERN;    break;
              }
              refresh_it=true;                                   break;
              
      case 6: PatternNotEmpty=0; 
              for (x=0; x<MaxVoices; x++) if (DRUM_PATTERN[PatternNumber-1].Voice[x].NOTE) PatternNotEmpty=true;     //  check if the current pattern is empty
              if (PatternNotEmpty) {                                                                                 //  if not empty, go to next pattern
                if (PatternNumber<MaxPatterns) { 
                  PatternNumber++; if (UsedPatterns<PatternNumber) UsedPatterns=PatternNumber; 
                }
                set_Pattern_LEDs();   refresh_it=true;             
              }                                                                                          
              break;      
      case 7: switch(Menu_Item) {
                case SHUFFLE        : Menu_Item=FLAM;            break;
                case FLAM           : Menu_Item=MUTE_VOICE;      break;
                case MUTE_VOICE     : Menu_Item=ACCENT_LO;       break;
                case ACCENT_LO      : Menu_Item=ACCENT_HI;       break;
                case ACCENT_HI      : Menu_Item=SHUFFLE;         break;
                default             : Menu_Item=SHUFFLE;         break;
              }
              refresh_it=true;                                   break; 

      case 8: switch(Menu_Item) {
                case TEMPO          : Menu_Item=MIDI_IN;         break;
                case MIDI_IN        : Menu_Item=MIDI_OUT;        break;
                case MIDI_OUT       : Menu_Item=CLOCK_SOURCE;    break;
                case CLOCK_SOURCE   : Menu_Item=TAP_MODE;        break;  
                case TAP_MODE       : Menu_Item=ERASE_EEPROM;    break;                
                case ERASE_EEPROM   : Menu_Item=TEMPO;           break; 
                default             : Menu_Item=TEMPO;           break;
              }
              refresh_it=true;                                   break;             
    }
  } 
}


void Up_Pressed_or_Down_Pressed() {
  if ((UP_Pressed) || (DOWN_Pressed)) {
    switch (Menu_Item) {
      case INSTRUMENT:    if (UP_Pressed)   if (VoiceNumber<MaxVoices) VoiceNumber++; else VoiceNumber=1;
                          if (DOWN_Pressed) if (VoiceNumber>1) VoiceNumber--; else VoiceNumber=MaxVoices;
                          set_Pattern_LEDs();
                          break;
      case TEMPO        : if (!Read_Tempo_Pot) {
                            if (UP_Pressed)  
                              if (BPM<500) { BPM++;  BPM_Factor = (240000)/BPM;  TCB2.CCMP = BPM_Factor; }   //  adjust the Timer Interrupt if BPM was changed                     
                            if (DOWN_Pressed)  
                              if (BPM>15)  { BPM--;  BPM_Factor = (240000)/BPM;  TCB2.CCMP = BPM_Factor; }   //  adjust the Timer Interrupt if BPM was changed
                            lcd.setCursor(0,1);  lcd.print(BPM); lcd.print(F(" BPM"));  
                            MeasureTime = 4*60000/BPM;
                          }
                          break;                 
      case SHUFFLE      : if (UP_Pressed)   if (Shuffle[VoiceNumber-1]<10) Shuffle[VoiceNumber-1]++; 
                          if (DOWN_Pressed) if (Shuffle[VoiceNumber-1]>0)  Shuffle[VoiceNumber-1]--; 
                          break;                        
      case FLAM         : if (UP_Pressed)   if (Flam[VoiceNumber-1]<10) Flam[VoiceNumber-1]++; 
                          if (DOWN_Pressed) if (Flam[VoiceNumber-1]>0)  Flam[VoiceNumber-1]--; 
                          break; 
      case ACCENT_HI    : if (UP_Pressed)   if (Accent_HI[VoiceNumber-1]<10) Accent_HI[VoiceNumber-1]++; 
                          if (DOWN_Pressed) if (Accent_HI[VoiceNumber-1]>Accent_LO[VoiceNumber-1]+1)  Accent_HI[VoiceNumber-1]--;  //  Accent_HI is always higher than Accent_LO
                          break; 
      case ACCENT_LO    : if (UP_Pressed)   if (Accent_LO[VoiceNumber-1]<Accent_HI[VoiceNumber-1]-1) Accent_LO[VoiceNumber-1]++;   //  Accent_LO is always lower than Accent_HI
                          if (DOWN_Pressed) if (Accent_LO[VoiceNumber-1]>0)  Accent_LO[VoiceNumber-1]--; 
                          break;  
      default           : break;                 
    } 
    UP_Pressed   = false;
    DOWN_Pressed = false;
    refresh_it=true;
  } 
}


void Do_Enter_Pressed() {
  
  ENTER_Pressed = false;  
  if (Menu_Item==LOAD)           { Load_Track(); }
  if (Menu_Item==SAVE)           { Save_Track(); }
  if (Menu_Item==CLEAR_TRACK)    { Clear_Track(); } 
  if (Menu_Item==TRACK_PLAY)     { Do_PlayTrack(); }
  if (Menu_Item==RANDOM_TRACK)   { RandomRhythm(); }
  if (Menu_Item==IMPROVISE)      { Improvise(); }  
  if (Menu_Item==SHIFT_PATTERN)  { Shift_Pattern(); } 
  if (Menu_Item==DELETE_PATTERN) { Delete_Pattern(); }  
  if (Menu_Item==INSERT_PATTERN) { Insert_Pattern(); }   
  if (Menu_Item==ERASE_EEPROM)   { Erase_EEPROM(); } 
  if (Menu_Item==FLAM)           { if (!Flam[VoiceNumber-1]) { lcd.setCursor(0,1); lcd.print(F("Set value first")); }
                                   if (Flam[VoiceNumber-1]) { 
                                     Select_instru=false; Menu_Item=INSTRUMENT; refresh_it=true;
                                     Add_flam=true;
                                   } 
                                 }   
  if (Menu_Item==MIDI_IN)        { MIDI_In(); }
  if (Menu_Item==MIDI_OUT)       { MIDI_Out(); }  
  if (Menu_Item==WRITE_TRACK)    { Write_Track(); }
  if (Menu_Item==EDIT_TRACK)     { Edit_Track(); }
  if (Menu_Item==COPY_PATTERN)   { Copy_Pattern(); }
  if (Menu_Item==CLOCK_SOURCE)   { Select_Clock_Source(); }   
  if (Menu_Item==PATTERN_PLAY)   { Do_PatternPlay(); }  
  if (Menu_Item==TEMPO)          { Menu_Item=INSTRUMENT; refresh_it=true; } 
  if (Menu_Item==TAP_MODE)       { 
                                   if (Tap_Mode) Tap_Mode=false; else Tap_Mode=true;
                                   for (byte i=0; i<MaxVoices; i++) Mute[i]=0;
                                   Mute_Voice_Mode=false;                        // Tap_Mode and Mute_Voice_Mode cannot be on at the same time
                                   Menu_Item=INSTRUMENT; refresh_it=true;
                                 }
  if (Menu_Item==MUTE_VOICE)     { 
                                   if (Mute_Voice_Mode) {
                                     Mute_Voice_Mode=false;
                                     set_Pattern_LEDs();
                                   }
                                   else {
                                     Mute_Voice_Mode=true; 
                                     for (byte i=1; i<=16; i++) Tlc_set(15+i, 0);
                                     for (byte i=1; i<=MaxVoices; i++) if (!Mute[i-1]) Tlc_set(15+i, LED_HI);
                                     Tlc_update();                       
                                   }
                                   Tap_Mode=false;                              // Tap_Mode and Mute_Voice_Mode cannot be on at the same time
                                   Menu_Item=INSTRUMENT; refresh_it=true;
                                 }
  if (Menu_Item==LASTSTEP)       { Last_Step(); }   
}


void Walking_LEDs() { 
  if(!Mute_Voice_Mode) {  
    AccentBit = ReadAccent(PatternNumber-1,VoiceNumber-1,StepCounter);          // set the new LED for the new note
    if (AccentBit==0) Tlc_set(15+StepCounter,LED_HI);                           // bright
    else              Tlc_set(15+StepCounter,LED_LO);                           // dim

    byte tempStepCounter = StepCounter-1;  
    if (tempStepCounter==0) tempStepCounter = LAST_STEP;
    AccentBit = ReadAccent(PatternNumber-1,VoiceNumber-1,tempStepCounter);      // reset the previous LED when going to the next LED
    NoteBit   = ReadNote(PatternNumber-1,VoiceNumber-1,tempStepCounter);
    if (AccentBit==1)    Tlc_set(15+tempStepCounter,LED_HI);                    // bright
    else if (NoteBit==1) Tlc_set(15+tempStepCounter,LED_LO);                    // dim
    else                 Tlc_set(15+tempStepCounter,0);                         // off
  }
  Tlc_update();
}


void Pattern_Finished() { 
  StepCounter=1; 
  lastStepCounter=-1;                                           // for TapMode    
  if (!Int_clock_mode) MeasureTime=millis()-lastmillis;        // Calculate MeasureTime, it is used to calculate Shuffle and Flam interval
  lastmillis=millis();
}


void Set_Triggers_To_Zero() {

  #if (!TR_909_HIHAT)
    for(byte i=0; i<MaxVoices; i++) SetPin(i+1,LOW);           // normal PMDS operation: set all triggers to 0 after 5 ms
  #endif  

  #if (TR_909_HIHAT)
    for(byte i=0; i<MaxVoices; i++) 
    if (i!=Open_hihat) SetPin(i+1,LOW);                        // PM909 operation: all triggers to 0 except: OH trigger
  #endif 
  
  WritePins();   
}


void Check_MIDI_Clock_In() {
  if(Serial1.available() > 0) {
    data = Serial1.read();
    if (data == midi_start) {
      play_flag = 1;  
      if (RUN) PlayNotes = true;
      MIDI_Clock_Counter=0;
      StepCounter=1;
      set_Pattern_LEDs();
      lastmillis=millis();     
    }
    else if (data == midi_continue) {
      play_flag = 1;
    }
    else if (data == midi_stop) {
      play_flag = 0;
    }
    else if ((data == midi_clock) && (play_flag == 1)) {
      MIDI_Clock_Counter++; if (MIDI_Clock_Counter==6) {
        if (RUN) PlayNotes = true;
        MIDI_Clock_Counter=0;
      }  
    }
  } 
}


void Send_MIDI_Clock_Out() {                                         //   Nothing here
}


void Check_Reset() {
 if (digitalRead(RESET_pin)==HIGH){                                  // RESET 
   RESET=true;
 }
}


void Reset_Sequencer() {
  StepCounter=1; 
  lastmillis=millis(); 
  set_Pattern_LEDs();
  RESET=false; 
  Serial1.write(midi_stop);
  Serial1.write(midi_start);
  play_flag = 1;  
}


void Set_Velocities() {                                                                        // set velocities for all instruments (accent)
  
  for (byte instr=0; instr<MaxVoices; instr++) {                                            
    NoteBit   = ReadNote(PatternNumber-1,instr,StepCounter);                                   // check if there is a note,
    if (NoteBit==1) {                                                                          // if so, change velocity
      AccentBit = ReadAccent(PatternNumber-1,instr, StepCounter);
      if (AccentBit==0) Tlc_set(instr,250-25*Accent_LO[instr]);                                // low accent
      if (AccentBit==1) Tlc_set(instr,250-25*Accent_HI[instr]);                                // high accent

      #if (TR_909_HIHAT)
        if (instr==Open_hihat) {                                                
           if (AccentBit==0) Tlc_set(Closed_hihat,250-25*Accent_LO[Open_hihat]);                          
           if (AccentBit==1) Tlc_set(Closed_hihat,250-25*Accent_HI[Open_hihat]);       
        }        
        if (instr==Closed_hihat) {                                                
           if (AccentBit==0) Tlc_set(Closed_hihat,250-25*Accent_LO[Closed_hihat]);                          
           if (AccentBit==1) Tlc_set(Closed_hihat,250-25*Accent_HI[Closed_hihat]);       
        }                                                     
      #endif

      #if (TR_808_HIHAT)                                                                     
        if (instr==Closed_hihat) { 
          if (AccentBit==0) Tlc_set(Open_hihat,250-25*Accent_LO[Closed_hihat]);                // Open hihat & closed hihat share the same velocity pin. (TR-808 hihat module)
          if (AccentBit==1) Tlc_set(Open_hihat,250-25*Accent_HI[Closed_hihat]);                // velocity (CV) of the Closed Hihat pin should be the same as the velocity of the Open Hihat pin
        }      
      #endif
    }  
  } 
}


void Set_Trigger_Pins() {
  for (byte instr=0; instr<MaxVoices; instr++) { 
    NoteBit   = ReadNote(PatternNumber-1,instr,StepCounter);
    FlamBit   = ReadFlam(PatternNumber-1,instr,StepCounter);
    if (NoteBit==1) {                                                                          // is there a note?
      
      if ((Shuffle[instr]) && ((StepCounter % 2) == 0))  {                                     // is there shuffle and is it an even note?
        PostponeUntil[instr] = millis() + Shuffle[instr]*MeasureTime/400;                      // if so, postpone note.                       
        if (Shuffle[instr]==10) PostponeUntil[instr] = millis() + MeasureTime/32;              // if shuffle = 10, then delay half a note (32th)
        ShuffleHasBeenPlayed[instr]=false;                          
      }
      else { 
        
        #if (!TR_909_HIHAT)        
          SetPin(instr+1,HIGH);                                                                // is there a note and no shuffle? then play it immediately.
        #endif  

        #if (TR_909_HIHAT)    
          if (instr==Open_hihat) { 
            SetPin(Open_hihat+1,LOW);                              // HIHAT SELECT = LOW for an Open Hihat sound
            if (!Mute[Open_hihat]) SetPin(Closed_hihat+1,HIGH);    // Closed hihat pin is HIGH for OH_Trigger 
          }   
          else if (instr==Closed_hihat) { 
            SetPin(Open_hihat+1,HIGH);                             // HIHAT SELECT = HIGH for a Closed Hihat sound  
            if (!Mute[Closed_hihat]) SetPin(Closed_hihat+1,HIGH);  // Closed hihat pin is HIGH for CH_Trigger     
          } 
          else SetPin(instr+1,HIGH);                                                                        
        #endif 
        
        if ((FlamBit==1) && (Flam[instr]))  {                                                  // does the note have flam?           
          PlayAgainAt[instr] = millis() + Flam[instr]*4 + 6;                                   // if so, prepare a second note to be played later on
          if (Flam[instr]==10) PlayAgainAt[instr] = millis() + MeasureTime/32;                 // if flam = 10, then in between two notes (32th)               
          FlamHasBeenPlayed[instr]=false; 
        }
      }       
    }    
  } 
  WritePins();   
}


void Check_For_Shuffle() {                                                                     // Shuffle: are there postponed notes that have to be played right now?
  temp_millis = millis();   
  for (byte instr=0; instr<MaxVoices; instr++) {
  if (temp_millis > PostponeUntil[instr])                                                     
    if (!ShuffleHasBeenPlayed[instr]){

      #if (!TR_909_HIHAT)       
        SetPin(instr+1,HIGH); 
      #endif

      #if (TR_909_HIHAT)    
        if (instr==Open_hihat) { 
          SetPin(Open_hihat+1,LOW);                              // HIHAT SELECT = LOW for an Open Hihat sound
          if (!Mute[Open_hihat]) SetPin(Closed_hihat+1,HIGH);    // Closed hihat pin is HIGH for OH_Trigger 
        }   
        else if (instr==Closed_hihat) { 
          SetPin(Open_hihat+1,HIGH);                             // HIHAT SELECT = HIGH for a Closed Hihat sound
          if (!Mute[Closed_hihat]) SetPin(Closed_hihat+1,HIGH);  // Closed hihat pin is HIGH for CH_Trigger
        }
        else SetPin(instr+1,HIGH); 
      #endif

      WritePins();    
      FlamBit = ReadFlam(PatternNumber-1, instr, StepCounter);
      if ((FlamBit==1)  && (Flam[instr]))  {                                                   // there is flam! Prepare a second note that will be played later on
        PlayAgainAt[instr] = millis() + Flam[instr]*4 + 6;                     
        if (Flam[instr]==10) PlayAgainAt[instr] = millis() + MeasureTime/8;
        FlamHasBeenPlayed[instr]=0;
      }
      ShuffleHasBeenPlayed[instr]=true;
    }
  }   
}


void Check_For_Flam() {                                                                        // Flam: are there notes that have to be played again right now?
  temp_millis = millis();  
  for (byte instr=0; instr<MaxVoices; instr++) {
  if (temp_millis > PlayAgainAt[instr])                                               
    if (!FlamHasBeenPlayed[instr]){

      #if (!TR_909_HIHAT)     
        SetPin(instr+1,HIGH); 
      #endif

      #if (TR_909_HIHAT)    
        if (instr==Open_hihat) { 
          SetPin(Open_hihat+1,LOW);                              // HIHAT SELECT = LOW for an Open Hihat sound
          if (!Mute[Open_hihat]) SetPin(Closed_hihat+1,HIGH);    // Closed hihat pin is HIGH for OH_Trigger 
        }   
        else if (instr==Closed_hihat) { 
          SetPin(Open_hihat+1,HIGH);                             // HIHAT SELECT = HIGH for a Closed Hihat sound
          if (!Mute[Closed_hihat]) SetPin(Closed_hihat+1,HIGH);  // Closed hihat pin is HIGH for CH_Trigger
        } 
        else SetPin(instr+1,HIGH); 
      #endif
      
      WritePins();  
      FlamHasBeenPlayed[instr]=true; 
    }
  } 
}


void Check_Sixteen_Buttons_Tap_Mode()  {

byte Instru_Number;
// static byte lastStepCounter;
byte Step = StepCounter + Sub_StepCounter - 1;
  
  if ( (millis() - lastDebounceTime_STEP_switch) > TapDebounce) {           // check if one of the sixteen buttons is pressed: 
    Instru_Number = readAnalogButton();
    lastDebounceTime_STEP_switch = millis();

    if (Instru_Number>0) {

      if (!RUN) {
        if (millis() > PlayAgainAt[Instru_Number-1]) {                      // play sounds when the sequencer does not run
          Tlc_set(Instru_Number,250-25*Accent_HI[Instru_Number]);       
          SetPin(Instru_Number,HIGH);    
          WritePins();  PlayAgainAt[Instru_Number-1] = millis() + 200;
        }
      }

      if (RUN) {
        if (Select_instru) {                                                // Erase notes of a voice when SELECT was pressed
          WriteNote(PatternNumber-1,Instru_Number-1,Step,0);
          WriteAccent(PatternNumber-1,Instru_Number-1,Step,0); 
          WriteFlam(PatternNumber-1,Instru_Number-1,Step,0); 
        }

        if (!Select_instru) { 
        
          if (Step - lastStepCounter > 1) {                                 // disable two notes right after each other
        
            NoteBit   = ReadNote(PatternNumber-1, Instru_Number-1, Step);
        
            if (NoteBit==0) {                                               // no note? Add a note
              WriteNote(PatternNumber-1,Instru_Number-1,Step,1);
              WriteAccent(PatternNumber-1,Instru_Number-1,Step,1);  
              if (Add_flam) WriteFlam(PatternNumber-1,Instru_Number-1,Step,1);
              
              if (Step==StepCounter-1) {                                    // play the sound even if button is pressed after Step
                Tlc_set(Instru_Number,250-25*Accent_HI[Instru_Number]);     // don't play it again if button is pressed before Step  
                SetPin(Instru_Number,HIGH);    
                WritePins();
              }                     
              lastStepCounter = Step;
            }
               
            if (NoteBit==1) {                                               // already a note? erase it
              WriteNote(PatternNumber-1,Instru_Number-1,Step,0);
              WriteAccent(PatternNumber-1,Instru_Number-1,Step,0);
              WriteFlam(PatternNumber-1,Instru_Number-1,Step,0); 
              lastStepCounter = Step;
            } 
          }
        }
      }
    }
  }  
}


void Check_Voice_Mute_Mode() {
 
  if ( (millis() - lastDebounceTime_STEP_switch) > debounceDelay) {
    Instru_Number = readAnalogButton();
    lastDebounceTime_STEP_switch = millis();

    if ((Instru_Number>0) && (Instru_Number<=MaxVoices)) { 

      if (Mute[Instru_Number-1]) {
        Mute[Instru_Number-1]=0;
        Tlc_set(15+Instru_Number,LED_HI); 
        Tlc_update();
      }      
      else {
        Mute[Instru_Number-1]=1;
        Tlc_set(15+Instru_Number,0);
        Tlc_update();
      }    
    }
  }
}                      


void loop() {
  if (Tap_Mode) Check_Sixteen_Buttons_Tap_Mode(); 
  if (Mute_Voice_Mode) Check_Voice_Mute_Mode();
  if ((!Tap_Mode) && (!Mute_Voice_Mode) && (!VoiceChanged)) Check_The_Sixteen_Buttons();   
  if (VoiceChanged) { if (millis()>WaitUntil) VoiceChanged = false; }                      // prevents that a note is written right after changing a voice
  
  Check_Start_Stop_Button();
  if (!Select_instru) Check_Select_Button();
  Check_The_Option_Buttons();                                                              // check if TRACK, PATTERN, MODE, or VOICE button is pressed
  Up_Pressed_or_Down_Pressed();

  Check_For_ENTER_Press();   
  if (ENTER_Pressed) Do_Enter_Pressed();

  Check_For_ESCAPE_Press();
  if (ESCAPE_Pressed) {
    ESCAPE_Pressed=false;
    Select_instru=false;
    Add_flam=false;
    Menu_Item=INSTRUMENT;   
    refresh_it=true;
  }

  Check_For_Shuffle();
  Check_For_Flam();
  Check_MIDI_Clock_In();
  Check_Reset();
  
  if (PlayNotes) {
    if (!RESET) {
      Set_Velocities();
      Walking_LEDs();
      Set_Trigger_Pins();    
      if (RUN) StepCounter++;   if (StepCounter>LAST_STEP) Pattern_Finished();  
      PlayNotes = false; 
    } 
    
    if (Tap_Mode) {
      if ((StepCounter==2) || (StepCounter==6) || (StepCounter==10) || (StepCounter==14)) {
        Tlc_set(6,2); Tlc_update();
        SetPin(6,HIGH);                                        // metronome sound in Tap Mode
        WritePins();
      }
    } 
    
    if (RESET) Reset_Sequencer(); 
    
    if (refresh_it) {
      Set_Triggers_To_Zero();                                  // set triggers pin low, if not: weird sound when pressing buttons 
      Refresh_LCD();  refresh_it=false;                        // refresh the LCD (when needed) right after the notes have been played
    }  
  }

  if (!RUN) 
    if (refresh_it) { Refresh_LCD(); refresh_it=false; }

  if (!Int_clock_mode)
    if (refresh_it) { Refresh_LCD(); refresh_it=false; }
  
  if (millis()>trigger_OFF) Set_Triggers_To_Zero(); 

  if ((Read_Tempo_Pot) && (RUN))  CheckTempoPotmeter(); 
}  


void isr()                                                     // Hardware interrupt (external clock: LFO )
{
  if (digitalRead(2) == HIGH) {
    if (RUN) { 
      PlayNotes=true;
      digitalWrite(CLOCK_OUT_pin,LOW);
      Sub_StepCounter=0; 
    }  
  }
  else {
    digitalWrite(CLOCK_OUT_pin,HIGH);  
    Sub_StepCounter=1;
  }
}


ISR(TCB2_INT_vect)                                             // Timer interrupt  (internal clock)
{
  if (RUN) 
    if (TimerInterruptFactor>999) {
      TimerInterruptFactor=0;
      PlayNotes=true;      
    }
    else TimerInterruptFactor++; 
    
  switch(TimerInterruptFactor) {
    case 0:    Serial1.write(midi_clock) ; digitalWrite(CLOCK_OUT_pin,LOW);   Sub_StepCounter=0; break;            // send 6 midi ticks for each STEP
    case 167:  Serial1.write(midi_clock) ; break;
    case 333:  Serial1.write(midi_clock) ; break;
    case 500:  Serial1.write(midi_clock) ; digitalWrite(CLOCK_OUT_pin,HIGH);  Sub_StepCounter=1; break;
    case 667:  Serial1.write(midi_clock) ; break;
    case 833:  Serial1.write(midi_clock) ; break;
  }  

  TCB2.INTFLAGS = TCB_CAPT_bm;                                 // Clear interrupt flag
}
