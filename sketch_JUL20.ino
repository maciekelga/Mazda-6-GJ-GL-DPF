#include "BluetoothSerial.h"
#include "ELMduino.h"

BluetoothSerial SerialBT;
#define ELM_PORT   SerialBT
#define DEBUG_PORT Serial
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

  DEBUG_PORT.begin(115200); //ustawienie do komunikacji po usb
  SerialBT.setPin("1234"); //kod pin do parowania z vlinkiem
  ELM_PORT.begin("M6",true); //aktywacja BT, true=debug on
  
  if (!ELM_PORT.connect("V-LINK"))
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

  if (!vlink.begin(ELM_PORT))
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


 // Serial.println("Connected to V-LINK");
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
      A=0;
      B=0;
      C=0;
      D=0;
    }
  break;
    
    
case 5:

      if (((PM_ACC_calc) <0.3) or ((PM_GEN_calc) <0.3))
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
      }

      else if ((((PM_ACC_calc) >=0.3) && ((PM_ACC_calc) < 0.93)) or (((PM_GEN_calc) >=0.3) && ((PM_GEN_calc) < 0.93)))
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
      }

      else if ((((PM_ACC_calc) >= 0.93) && ((PM_ACC_calc) < 1.56)) or (((PM_GEN_calc) >=0.93) && ((PM_GEN_calc) < 1.56)))   
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
      }
      else if ((((PM_ACC_calc) >= 1.56) && ((PM_ACC_calc) < 2.19)) or (((PM_GEN_calc) >=1.56) && ((PM_GEN_calc) < 2.19)))
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
      }
      else if ((((PM_ACC_calc) >= 2.19) && ((PM_ACC_calc) < 2.82)) or (((PM_GEN_calc) >=2.19) && ((PM_GEN_calc) < 2.82)))
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
      }
      else if ((((PM_ACC_calc) >= 2.82) && ((PM_ACC_calc) < 3.45)) or (((PM_GEN_calc) >=2.82) && ((PM_GEN_calc) < 3.45)))
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
      }
      else if ((((PM_ACC_calc) >= 3.45) && ((PM_ACC_calc) < 4.08)) or (((PM_GEN_calc) >=3.45) && ((PM_GEN_calc) < 4.08)))
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
      }
      else if ((((PM_ACC_calc) >= 4.08) && ((PM_ACC_calc) < 4.71)) or (((PM_GEN_calc) >=4.08) && ((PM_GEN_calc) < 4.71)))
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
      }
      else if ((((PM_ACC_calc) >= 4.71) && ((PM_ACC_calc) < 5.34)) or (((PM_GEN_calc) >=4.71) && ((PM_GEN_calc) < 5.34)))
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
      }
      else if (((PM_ACC_calc) >= 5.34) or ((PM_GEN_calc) >=5.34))
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
      }

      if ((DPF_Regen==0) && (DPF_LRD_calc <0.1))
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
      }
      
      mode_number = 1;
      break;
}

}
