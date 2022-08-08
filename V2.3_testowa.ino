#include "BluetoothSerial.h"
#include "ELMduino.h"

BluetoothSerial SerialBT;
#define ELM_PORT   SerialBT
#define LED1 16 // LED
#define LED2 17 // LED
#define LED3 18 // LED
#define LED4 19 // LED
#define LED5 21 // LED
#define LED6 22 // LED
#define LED7 23 // LED
#define LED8 33 // LED
#define LED9 26 // LED
#define LED10 27 // LED

ELM327 vlink; //przypisanie nazwy vlink
byte count = 0; //zmienna pomocnicza iteracji
int mode_number = 1; //zmienna pomocnicza iteracji
uint32_t DPF_LRD = 0;
uint32_t PM_GEN = 0;
uint32_t PM_ACC = 0;
uint32_t DPF_Regen = 0;
float DPF_LRD_calc = 0;
float PM_GEN_calc = 0;
float PM_ACC_calc = 0;
byte DPF_Regen_b = 0;
byte D = 0;
byte C = 0;
byte B = 0;
byte A = 0;
bool S1 = false;
bool S2 = false;
bool S3 = false;
bool S4 = false;
bool S5 = false;
bool S6 = false;
bool S7 = false;
bool S8 = false;
bool S9 = false;
bool S10 = false;
bool S11 = false;
bool S12 = false;

unsigned long previousMillis = 0;
const long interval = 1000;

//MAC address = "10:21:3E:47:FC:D2";
uint8_t address[6]  = {0x10, 0x21, 0x3E, 0x47, 0xFC, 0xD2};

void setup()
{
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  pinMode(LED5, OUTPUT);
  pinMode(LED6, OUTPUT);
  pinMode(LED7, OUTPUT);
  pinMode(LED8, OUTPUT);
  pinMode(LED9, OUTPUT);
  pinMode(LED10, OUTPUT);


  ELM_PORT.setPin("1234"); //kod pin do parowania z vlinkiem
  ELM_PORT.begin("M6",true); //aktywacja BT, true=debug on
  
  if (!ELM_PORT.connect(address))
  {
  while (count < 4) //dioda led miga 4 razy co 1 sekunde gdy nie uda sie polaczyc z interfejsem
  {
      analogWrite(LED10, 50);
      delay(1000);
      analogWrite(LED10, 0);
      delay(1000);
      count++;
  }
  while(1);
  }

  if (!vlink.begin(ELM_PORT, true, 2000))
  {
  while (count < 5) //dioda led miga 5 razy co 1 sekunde gdy nie uda sie polaczyc z interfejsem
  {
      analogWrite(LED10, 50);
      delay(1000);
      analogWrite(LED10, 0);
      delay(1000);
      count++;
  }
  while(1);
  }

  while (count < 2) //dioda led miga szybko 2 razy uda sie polaczyc z interfejsem
  {
      analogWrite(LED10, 50);
      delay(250);
      analogWrite(LED10, 0);
      delay(250);
      count++;
  }

  }

void loop()
{

unsigned long currentMillis = millis();
  
switch (mode_number)
{
  
case 1:
  DPF_LRD = vlink.processPID(34, 1076, 4, 4, 1, 0); //PID odleglosci od ostatniego wypalania, PID 220434; Equation:((B<16)+(C<8)+D)*( 1/640 )
  D = vlink.responseByte_0;
  C = vlink.responseByte_1;
  B = vlink.responseByte_2;
  A = vlink.responseByte_3;

  if (vlink.nb_rx_state == ELM_SUCCESS)
    { 
    DPF_LRD_calc =((256*256*B)+(C*256)+D)/640.00;
    mode_number = 2;
    A=0;
    B=0;
    C=0;
    D=0;
    }
  break;
    

case 2:
  PM_ACC = vlink.processPID(34, 1068, 4, 4, 1, 0); //PID ilosci zgromadzonej sadzy, PID 22042C, Equation: ((A*256)+B)/628 
  D = vlink.responseByte_0;
  C = vlink.responseByte_1;
  B = vlink.responseByte_2;
  A = vlink.responseByte_3;

  if (vlink.nb_rx_state == ELM_SUCCESS)
    { 
    PM_ACC_calc = ((C*256)+D)/655.00;
    mode_number = 3;
    A=0;
    B=0;
    C=0;
    D=0;
    }
  break;
    
    
case 3:
  PM_GEN = vlink.processPID(34, 1069, 4, 4, 1, 0); //PID ilosci teoretycznie zgromadzonej sadzy, PID 22042D, Equation: ((A*256)+B)/628 
  D = vlink.responseByte_0;
  C = vlink.responseByte_1;
  B = vlink.responseByte_2;
  A = vlink.responseByte_3;

  if (vlink.nb_rx_state == ELM_SUCCESS)
    { 
    PM_GEN_calc = ((C*256)+D)/655.00;
    mode_number = 4;
    A=0;
    B=0;
    C=0;
    D=0;
    }
  break;
    
    
case 4:
  DPF_Regen = vlink.processPID(34, 896, 1, 1, 1, 0); //PID aktywnej regeneracji filtra, PID 220380, Equation: A
  DPF_Regen_b = vlink.responseByte_3;
  if (vlink.nb_rx_state == ELM_SUCCESS)
    {
      if (DPF_Regen_b==2)
      {
        analogWrite(LED10, 50);
      }
      mode_number = 5;
    }
  break;
    
    
case 5:
      if (((PM_GEN_calc)<4.2) && ((PM_ACC_calc)>5.7) && DPF_Regen_b == 0)
      {
      if (S12 == false)
        {
        analogWrite(LED1, 50);
        analogWrite(LED2, 50);
        analogWrite(LED3, 50);
        analogWrite(LED4, 50);
        analogWrite(LED5, 50);
        analogWrite(LED6, 50);
        analogWrite(LED7, 50);
        analogWrite(LED8, 0); 
        analogWrite(LED9, 0);
        S1 = false;
        S2 = false;
        S3 = false;
        S4 = false;
        S5 = false;
        S6 = false;
        S7 = false;
        S8 = false;
        S9 = false;
        S10 = false;
        S11 = false;  
        S12 = true;
        }
      if (currentMillis - previousMillis >=500) 
        {
        analogWrite(LED8, 0);  
        analogWrite(LED9, 0);
        }
      if (currentMillis - previousMillis >= 1000) 
        {
        previousMillis = currentMillis;
        analogWrite(LED8, 50);  
        analogWrite(LED9, 50);
        }  
      }

      
      else if ((PM_GEN_calc) <0.3)
        {
	        if (S1 == false)
	        {
          analogWrite(LED1, 0);
          analogWrite(LED2, 0);
          analogWrite(LED3, 0);
          analogWrite(LED4, 0);
          analogWrite(LED5, 0);
          analogWrite(LED6, 0);
          analogWrite(LED7, 0); 
          analogWrite(LED8, 0); 
          analogWrite(LED9, 0);
	        S1 = true;
	        S2 = false;
	        S3 = false;
	        S4 = false;
	        S5 = false;
	        S6 = false;
	        S7 = false;
	        S8 = false;
	        S9 = false;
	        S10 = false;
	        S11 = false;
          S12 = false;
	        }
        }

      else if (((PM_GEN_calc) >=0.3) && ((PM_GEN_calc) < 0.93))
      {
	      if (S2 == false)
	      {
        analogWrite(LED1, 50);
        analogWrite(LED2, 0);
        analogWrite(LED3, 0);
        analogWrite(LED4, 0);
        analogWrite(LED5, 0);
        analogWrite(LED6, 0);
        analogWrite(LED7, 0); 
        analogWrite(LED8, 0); 
        analogWrite(LED9, 0);
	      S1 = false;
	      S2 = true;
	      S3 = false;
	      S4 = false;
	      S5 = false;
	      S6 = false;
	      S7 = false;
	      S8 = false;
	      S9 = false;
	      S10 = false;
	      S11 = false;	
        S12 = false;
	      }
      }

      else if (((PM_GEN_calc) >=0.93) && ((PM_GEN_calc) < 1.56))   
      {
	      if (S3 == false)
	      {
        analogWrite(LED1, 50);
        analogWrite(LED2, 50);
        analogWrite(LED3, 0);
        analogWrite(LED4, 0);
        analogWrite(LED5, 0);
        analogWrite(LED6, 0);
        analogWrite(LED7, 0); 
        analogWrite(LED8, 0); 
        analogWrite(LED9, 0);
	      S1 = false;
	      S2 = false;
	      S3 = true;
	      S4 = false;
	      S5 = false;
	      S6 = false;
	      S7 = false;
	      S8 = false;
	      S9 = false;
	      S10 = false;
	      S11 = false;	
        S12 = false;
	      }
      }
      
      else if (((PM_GEN_calc) >=1.56) && ((PM_GEN_calc) < 2.19))
      {
	      if (S4 == false)
	      {
        analogWrite(LED1, 50);
        analogWrite(LED2, 50);
        analogWrite(LED3, 50);
        analogWrite(LED4, 0);
        analogWrite(LED5, 0);
        analogWrite(LED6, 0);
        analogWrite(LED7, 0); 
        analogWrite(LED8, 0); 
        analogWrite(LED9, 0);
	      S1 = false;
	      S2 = false;
	      S3 = false;
	      S4 = true;
	      S5 = false;
	      S6 = false;
	      S7 = false;
	      S8 = false;
	      S9 = false;
	      S10 = false;
	      S11 = false;	
        S12 = false;
	      }
      }
      
      else if (((PM_GEN_calc) >=2.19) && ((PM_GEN_calc) < 2.82))
      {
	      if (S5 == false)
	      {
        analogWrite(LED1, 50);
        analogWrite(LED2, 50);
        analogWrite(LED3, 50);
        analogWrite(LED4, 50);
        analogWrite(LED5, 0);
        analogWrite(LED6, 0);
        analogWrite(LED7, 0); 
        analogWrite(LED8, 0); 
        analogWrite(LED9, 0);
	      S1 = false;
	      S2 = false;
	      S3 = false;
	      S4 = false;
	      S5 = true;
	      S6 = false;
	      S7 = false;
	      S8 = false;
	      S9 = false;
	      S10 = false;
	      S11 = false;	
        S12 = false;
	      }
      }
      
      else if (((PM_GEN_calc) >=2.82) && ((PM_GEN_calc) < 3.45))
      {
	      if (S6 == false)
	      {
        analogWrite(LED1, 50);
        analogWrite(LED2, 50);
        analogWrite(LED3, 50);
        analogWrite(LED4, 50);
        analogWrite(LED5, 50);
        analogWrite(LED6, 0);
        analogWrite(LED7, 0); 
        analogWrite(LED8, 0); 
        analogWrite(LED9, 0);
	      S1 = false;
	      S2 = false;
	      S3 = false;
	      S4 = false;
	      S5 = false;
	      S6 = true;
	      S7 = false;
	      S8 = false;
	      S9 = false;
	      S10 = false;
	      S11 = false;	
        S12 = false;
	      }
      }
      
      else if (((PM_GEN_calc) >=3.45) && ((PM_GEN_calc) < 4.08))
      {
	      if (S7 == false)
	      {
        analogWrite(LED1, 50);
        analogWrite(LED2, 50);
        analogWrite(LED3, 50);
        analogWrite(LED4, 50);
        analogWrite(LED5, 50);
        analogWrite(LED6, 50);
        analogWrite(LED7, 0); 
        analogWrite(LED8, 0); 
        analogWrite(LED9, 0);
	      S1 = false;
	      S2 = false;
	      S3 = false;
	      S4 = false;
	      S5 = false;
	      S6 = false;
	      S7 = true;
	      S8 = false;
	      S9 = false;
	      S10 = false;
	      S11 = false;	
        S12 = false;
	      }
      }
     
      else if (((PM_GEN_calc) >=4.08) && ((PM_GEN_calc) < 4.71))
      {
	      if (S8 == false)
	      {
        analogWrite(LED1, 50);
        analogWrite(LED2, 50);
        analogWrite(LED3, 50);
        analogWrite(LED4, 50);
        analogWrite(LED5, 50);
        analogWrite(LED6, 50);
        analogWrite(LED7, 50);
        analogWrite(LED8, 0); 
        analogWrite(LED9, 0);
	      S1 = false;
	      S2 = false;
	      S3 = false;
	      S4 = false;
	      S5 = false;
	      S6 = false;
	      S7 = false;
      	S8 = true;
	      S9 = false;
	      S10 = false;
	      S11 = false;	
        S12 = false;
	      }
      }

      else if (((PM_GEN_calc) >=4.71) && ((PM_GEN_calc) < 5.34))
      {
	      if (S9 == false)
	      {
        analogWrite(LED1, 50);
        analogWrite(LED2, 50);
        analogWrite(LED3, 50);
        analogWrite(LED4, 50);
        analogWrite(LED5, 50);
        analogWrite(LED6, 50);
        analogWrite(LED7, 50); 
        analogWrite(LED8, 50);  
        analogWrite(LED9, 0);
	      S1 = false;
	      S2 = false;
	      S3 = false;
	      S4 = false;
	      S5 = false;
	      S6 = false;
	      S7 = false;
	      S8 = false;
	      S9 = true;
	      S10 = false;
	      S11 = false;
        S12 = false;	
	      }
      }

      else if ((PM_GEN_calc) >=5.34)
      {
	      if (S10 == false)
	      {
        analogWrite(LED1, 50);
        analogWrite(LED2, 50);
        analogWrite(LED3, 50);
        analogWrite(LED4, 50);
        analogWrite(LED5, 50);
        analogWrite(LED6, 50);
        analogWrite(LED7, 50); 
        analogWrite(LED8, 50); 
        analogWrite(LED9, 50); 
	      S1 = false;
	      S2 = false;
	      S3 = false;
	      S4 = false;
	      S5 = false;
      	S6 = false;
      	S7 = false;
      	S8 = false;
	      S9 = false;
	      S10 = true;
	      S11 = false;	
        S12 = false;
      	}
      }

      if ((DPF_Regen_b==0) && (DPF_LRD_calc <0.1))
      {
	      if (S11 == false)
	      {
        analogWrite(LED10, 0);
        analogWrite(LED1, 0);
        analogWrite(LED2, 0);
        analogWrite(LED3, 0);
        analogWrite(LED4, 0);
        analogWrite(LED5, 0);
        analogWrite(LED6, 0);
        analogWrite(LED7, 0); 
        analogWrite(LED8, 0); 
        analogWrite(LED9, 0);
      	S1 = false;
	      S2 = false;
	      S3 = false;
	      S4 = false;
	      S5 = false;
	      S6 = false;
      	S7 = false;
      	S8 = false;
      	S9 = false;
	      S10 = false;
	      S11 = true;	
        S12 = false;
       }
      mode_number = 1;
      break;
      }
}
}
