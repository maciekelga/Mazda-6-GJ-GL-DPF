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
#define ILL_IN 13 // LED
ELM327 vlink; //przypisanie nazwy vlink

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
bool start_up = false;
bool ill_in_state = false;
bool S20 = false;
bool S21 = false;


byte count = 0; //zmienna pomocnicza iteracji dla migania LED przy inicjalizacji
byte fail_1 = 0; //zmienna pomocnicza licznika zliczajaca ilosc odczytow wykraczajacych poza zakres DPF_LRD
byte fail_2 = 0; //zmienna pomocnicza licznika zliczajaca ilosc odczytow wykraczajacych poza zakres PM_GEN
byte fail_3 = 0; //zmienna pomocnicza licznika zliczajaca ilosc odczytow wykraczajacych poza zakres PM_ACC
byte count_2 = 0; //zmienna pomocnicza licznika zliczajaca ilosc odczytow wykraczajacych poza zakres Regen
byte DPF_Regen_b = 0; //zmienna do odczytu z BT
byte D = 0; //zmienna bitu D
byte C = 0; //zmienna bitu C
byte B = 0; //zmienna bitu B
byte A = 0; //zmienna bitu A
unsigned int PWM_value_g = 0; //zmienna wypelnienia wyjsc LED zielonych
unsigned int PWM_value_orb = 0; //zmienna wypelnienia wyjsc LED pomaranczowych, czerwonych i niebieskiej
int mode_number = 1; //zmienna pomocnicza iteracji petli odczytow

uint32_t DPF_LRD = 0; //zmienna do odczytu z BT
uint32_t PM_GEN = 0; //zmienna do odczytu z BT
uint32_t PM_ACC = 0; //zmienna do odczytu z BT
uint32_t DPF_Regen = 0; //zmienna do odczytu z BT
float DPF_LRD_calc = 0; //zmienna do kalkulacji wartosci parametru
float PM_GEN_calc = 0; //zmienna do kalkulacji wartosci parametru
float PM_ACC_calc = 0; //zmienna do kalkulacji wartosci parametru
float DPF_LRD_calc_act = 0; //zmienna wartosci aktualnej
float PM_GEN_calc_act = 0; //zmienna wartosci aktualnej
float PM_ACC_calc_act = 0; //zmienna wartosci aktualnej
float DPF_LRD_calc_abs = 0; //zmienna wartosci absolutnej roznicy wartosci aktualnej i z chwili poprzedniej
float PM_GEN_calc_abs = 0; //zmienna wartosci absolutnej roznicy wartosci aktualnej i z chwili poprzedniej
float PM_ACC_calc_abs = 0; //zmienna wartosci absolutnej roznicy wartosci aktualnej i z chwili poprzedniej



unsigned long previousMillis = 0; //zmienna do przechowywania wartosci czasu chwili poprzedniej
unsigned long previousMillis2 = 0; //zmienna do przechowywania wartosci czasu chwili poprzedniej
unsigned long currentMillis = 0;

//---------------------------------------------------------------------------------------------------
//MAC address = "10:21:3E:47:FC:D2"; //przykladowy MAC adres, na jego podstawie wypelnic ponizej
uint8_t address[6]  = {0x10, 0x21, 0x3E, 0x47, 0xFC, 0xD2};
//---------------------------------------------------------------------------------------------------

void setup()
{
  ledcAttachPin(LED1, 1);
  ledcSetup(1, 1000, 16);
  ledcAttachPin(LED2, 2);
  ledcSetup(2, 1000, 16);
  ledcAttachPin(LED3, 3);
  ledcSetup(3, 1000, 16);
  ledcAttachPin(LED4, 4);
  ledcSetup(4, 1000, 16);
  ledcAttachPin(LED5, 5);
  ledcSetup(5, 1000, 16);
  ledcAttachPin(LED6, 6);
  ledcSetup(6, 1000, 16);
  ledcAttachPin(LED7, 7);
  ledcSetup(7, 1000, 16);
  ledcAttachPin(LED8, 8);
  ledcSetup(8, 1000, 16);
  ledcAttachPin(LED9, 9);
  ledcSetup(9, 1000, 16);
  ledcAttachPin(LED10, 10);
  ledcSetup(10, 1000, 16);
  pinMode(ILL_IN, INPUT_PULLUP);

  ill_in_state = digitalRead(ILL_IN);
  if (ill_in_state == true)
    {
    PWM_value_g = 18000;
    PWM_value_orb = 45000;
   }
  else
   {
   PWM_value_g = 25;
   PWM_value_orb = 75;
   }

  ELM_PORT.setPin("1234"); //kod pin do parowania z vlinkiem
  ELM_PORT.begin("M6",true); //aktywacja BT, true=debug on
  
  if (!ELM_PORT.connect(address))
  {
  while (count < 4) //dioda led miga 4 razy co 1 sekunde gdy nie uda sie polaczyc z interfejsem
  {
      ledcWrite(10, PWM_value_orb);
      delay(1000);
      ledcWrite(10, 0);
      delay(1000);
      count++;
  }
  while(1);
  }

  if (!vlink.begin(ELM_PORT, true, 2000))
  {
  while (count < 5) //dioda led miga 5 razy co 1 sekunde gdy nie ma komunikacji z interfejsem
  {
      ledcWrite(10, PWM_value_orb);
      delay(1000);
      ledcWrite(10, 0);
      delay(1000);
      count++;
  }
  while(1);
  }

  while (count < 2) //dioda led miga szybko 2 razy uda sie polaczyc z interfejsem
  {
      ledcWrite(10, PWM_value_orb);
      delay(250);
      ledcWrite(10, 0);
      delay(250);
      count++;
  }
  illumination();
  }

void loop()
{


  
switch (mode_number)
{
  
case 1:
  DPF_LRD = vlink.processPID(34, 1076, 4, 4, 1, 0); //PID odleglosci od ostatniego wypalania, PID 220434; Equation:((B<16)+(C<8)+D)*( 1/640 )
  if (vlink.nb_rx_state == ELM_SUCCESS)
    { 
    D = vlink.responseByte_0;
    C = vlink.responseByte_1;
    B = vlink.responseByte_2;
    A = vlink.responseByte_3;
    DPF_LRD_calc =((256*256*B)+(C*256)+D)/640.00;
    mode_number = 2;
    A=0;
    B=0;
    C=0;
    D=0;
    illumination();
    }
  break;
    

case 2:
  PM_ACC = vlink.processPID(34, 1068, 4, 4, 1, 0); //PID ilosci zgromadzonej sadzy, PID 22042C, Equation: ((A*256)+B)/628 
  if (vlink.nb_rx_state == ELM_SUCCESS)
    { 
    D = vlink.responseByte_0;
    C = vlink.responseByte_1;
    B = vlink.responseByte_2;
    A = vlink.responseByte_3;
    PM_ACC_calc = ((C*256)+D)/655.00;
    mode_number = 3;
    A=0;
    B=0;
    C=0;
    D=0;
    illumination();
    }
  break;
    
    
case 3:
  PM_GEN = vlink.processPID(34, 1069, 4, 4, 1, 0); //PID ilosci teoretycznie zgromadzonej sadzy, PID 22042D, Equation: ((A*256)+B)/628 
  if (vlink.nb_rx_state == ELM_SUCCESS)
    { 
    D = vlink.responseByte_0;
    C = vlink.responseByte_1;
    B = vlink.responseByte_2;
    A = vlink.responseByte_3;
    PM_GEN_calc = ((C*256)+D)/655.00;
    mode_number = 4;
    A=0;
    B=0;
    C=0;
    D=0;
    illumination();
    }
  break;
    
    
case 4:
  DPF_Regen = vlink.processPID(34, 896, 1, 1, 1, 0); //PID aktywnej regeneracji filtra, PID 220380, Equation: A
  if (vlink.nb_rx_state == ELM_SUCCESS)
    {
    DPF_Regen_b = vlink.responseByte_3;
      if (DPF_Regen_b==2)
      {
        count_2++;
        if (count_2 >5)
        {
        ledcWrite(10, PWM_value_orb); 
        count_2 = 0;
        }
      }
      else
      {
        count_2 = 0;
      }
    mode_number = 5;
    illumination();    
    }
  break;
    

case 5:
      currentMillis = millis(); //przypisanie wartosci czasu w chwili obecnej


      if (start_up == false) //przypisanie wartosci przy pierwszym odczycie
      {
      DPF_LRD_calc_act = DPF_LRD_calc;
      PM_GEN_calc_act = PM_GEN_calc;
      PM_ACC_calc_act = PM_ACC_calc;
      start_up = true;
      }
      if (currentMillis - previousMillis2 >= 300) //czesc programu ktora jest wykonywana z 300ms opoznieniem po otrzymaniu wszystkich 4 odczytow. W tej czesci kodu nastepuje eliminacja blednych pojedynczych odczytow.
        {
      
          DPF_LRD_calc_abs = DPF_LRD_calc - DPF_LRD_calc_act; //roznica miedzy wartoscia z odczytu a wartoscia z poprzedniego odczytu, podczas napelniania wartosc jest dodatnia, przy wypalaniu wartosc jest ujemna
          DPF_LRD_calc_abs = abs(DPF_LRD_calc_abs); //wartosc bezwzgledna z roznicy
          
             if (DPF_LRD_calc_abs < 5) //jesli roznica jest mniejsza niz 5 nastepuje przypisanie wartosci z pomiaru
             {
             DPF_LRD_calc_act = DPF_LRD_calc;
             fail_1 = 0; //kasowanie licznika blednych odczytow
             }
             else if (DPF_LRD_calc_abs >= 5) //jesli roznica jest wieksza niz 1 sprawdzane sa 4 kolejne probki w celu eliminacji bledow pojedynczych odczytow
             {
             fail_1++;
              if (fail_1 > 3)
              {
              DPF_LRD_calc_act = DPF_LRD_calc; //jesli 4 odczyty z rzedu sa pdoobne wtedy wartosc uznawana jest za prawidlowa
              fail_1 = 0;  
              }
             }
            
          PM_GEN_calc_abs = PM_GEN_calc - PM_GEN_calc_act;
          PM_GEN_calc_abs = abs(PM_GEN_calc_abs);
          
             if (PM_GEN_calc_abs < 0.07)
             {
             PM_GEN_calc_act = PM_GEN_calc;
             fail_2 = 0;
             }
             else if (PM_GEN_calc_abs >= 0.07)
             {
             fail_2++;
              if (fail_2 > 3)
              {
              PM_GEN_calc_act = PM_GEN_calc;
              fail_2 = 0; 
              }
             }


          PM_ACC_calc_abs = PM_ACC_calc - PM_ACC_calc_act;
          PM_ACC_calc_abs = abs(PM_ACC_calc_abs);
          
             if (PM_ACC_calc_abs < 0.1)
             {
             PM_ACC_calc_act = PM_ACC_calc;
             fail_3 = 0;
             }
             else if (PM_ACC_calc_abs >= 0.1  )
             {
             fail_3++;
              if (fail_3 > 3)
              {
              PM_ACC_calc_act = PM_ACC_calc;
              fail_3 = 0;  
              }
             }

          DPF_LRD_calc_abs = 0;
          PM_GEN_calc_abs = 0;
          PM_ACC_calc_abs = 0;
          mode_number = 6;
          previousMillis2 = millis();
        }
  illumination();
  break;
    
case 6:
      if (((PM_GEN_calc_act)<4.2) && ((PM_ACC_calc_act)>5.7) && DPF_Regen_b == 0) // gdy PM_ACC jest wiekszy niz 5.7 przy PM_GEN<4.6 dwie ostatnie diody migaja do momentu rozpoczecia wypalania
      {
      if (S12 == false)
        {
        ledcWrite(1, PWM_value_g);
        ledcWrite(2, PWM_value_g);
        ledcWrite(3, PWM_value_g);
        ledcWrite(4, PWM_value_g);
        ledcWrite(5, PWM_value_orb);
        ledcWrite(6, PWM_value_orb);
        ledcWrite(7, PWM_value_orb);
        ledcWrite(8, 0);
        ledcWrite(9, 0);
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
      if (currentMillis - previousMillis >=1000) 
        {
        ledcWrite(8, 0);
        ledcWrite(9, 0);
        }
      if (currentMillis - previousMillis >= 2000) 
        {
        previousMillis = millis();
        ledcWrite(8, PWM_value_orb);
        ledcWrite(9, PWM_value_orb);
        }  
      }

      
      else if ((PM_GEN_calc_act) <0.3)
        {
	        if (S1 == false) //zabezpieczenie przed ponownym wywolaniem i nadpisaniem wyjsc
	        {
        ledcWrite(1, 0);
        ledcWrite(2, 0);
        ledcWrite(3, 0);
        ledcWrite(4, 0);
        ledcWrite(5, 0);
        ledcWrite(6, 0);
        ledcWrite(7, 0);
        ledcWrite(8, 0);
        ledcWrite(9, 0);
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

      else if (((PM_GEN_calc_act) >=0.3) && ((PM_GEN_calc_act) < 0.93))
      {
	      if (S2 == false)
	      {
        ledcWrite(1, PWM_value_g);
        ledcWrite(2, 0);
        ledcWrite(3, 0);
        ledcWrite(4, 0);
        ledcWrite(5, 0);
        ledcWrite(6, 0);
        ledcWrite(7, 0);
        ledcWrite(8, 0);
        ledcWrite(9, 0);
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

      else if (((PM_GEN_calc_act) >=0.93) && ((PM_GEN_calc_act) < 1.56))   
      {
	      if (S3 == false)
	      {
        ledcWrite(1, PWM_value_g);
        ledcWrite(2, PWM_value_g);
        ledcWrite(3, 0);
        ledcWrite(4, 0);
        ledcWrite(5, 0);
        ledcWrite(6, 0);
        ledcWrite(7, 0);
        ledcWrite(8, 0);
        ledcWrite(9, 0);
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
      
      else if (((PM_GEN_calc_act) >=1.56) && ((PM_GEN_calc_act) < 2.19))
      {
	      if (S4 == false)
	      {
        ledcWrite(1, PWM_value_g);
        ledcWrite(2, PWM_value_g);
        ledcWrite(3, PWM_value_g);
        ledcWrite(4, 0);
        ledcWrite(5, 0);
        ledcWrite(6, 0);
        ledcWrite(7, 0);
        ledcWrite(8, 0);
        ledcWrite(9, 0);
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
      
      else if (((PM_GEN_calc_act) >=2.19) && ((PM_GEN_calc_act) < 2.82))
      {
	      if (S5 == false)
	      {
        ledcWrite(1, PWM_value_g);
        ledcWrite(2, PWM_value_g);
        ledcWrite(3, PWM_value_g);
        ledcWrite(4, PWM_value_g);
        ledcWrite(5, 0);
        ledcWrite(6, 0);
        ledcWrite(7, 0);
        ledcWrite(8, 0);
        ledcWrite(9, 0);
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
      
      else if (((PM_GEN_calc_act) >=2.82) && ((PM_GEN_calc_act) < 3.45))
      {
	      if (S6 == false)
	      {
        ledcWrite(1, PWM_value_g);
        ledcWrite(2, PWM_value_g);
        ledcWrite(3, PWM_value_g);
        ledcWrite(4, PWM_value_g);
        ledcWrite(5, PWM_value_orb);
        ledcWrite(6, 0);
        ledcWrite(7, 0);
        ledcWrite(8, 0);
        ledcWrite(9, 0);
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
      
      else if (((PM_GEN_calc_act) >=3.45) && ((PM_GEN_calc_act) < 4.08))
      {
	      if (S7 == false)
	      {
        ledcWrite(1, PWM_value_g);
        ledcWrite(2, PWM_value_g);
        ledcWrite(3, PWM_value_g);
        ledcWrite(4, PWM_value_g);
        ledcWrite(5, PWM_value_orb);
        ledcWrite(6, PWM_value_orb);
        ledcWrite(7, 0);
        ledcWrite(8, 0);
        ledcWrite(9, 0);
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
     
      else if (((PM_GEN_calc_act) >=4.08) && ((PM_GEN_calc_act) < 4.71))
      {
	      if (S8 == false)
	      {
        ledcWrite(1, PWM_value_g);
        ledcWrite(2, PWM_value_g);
        ledcWrite(3, PWM_value_g);
        ledcWrite(4, PWM_value_g);
        ledcWrite(5, PWM_value_orb);
        ledcWrite(6, PWM_value_orb);
        ledcWrite(7, PWM_value_orb);
        ledcWrite(8, 0);
        ledcWrite(9, 0);
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

      else if (((PM_GEN_calc_act) >=4.71) && ((PM_GEN_calc_act) < 5.34))
      {
	      if (S9 == false)
	      {
        ledcWrite(1, PWM_value_g);
        ledcWrite(2, PWM_value_g);
        ledcWrite(3, PWM_value_g);
        ledcWrite(4, PWM_value_g);
        ledcWrite(5, PWM_value_orb);
        ledcWrite(6, PWM_value_orb);
        ledcWrite(7, PWM_value_orb);
        ledcWrite(8, PWM_value_orb);
        ledcWrite(9, 0);
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

      else if ((PM_GEN_calc_act) >=5.34)
      {
	      if (S10 == false)
	      {
        ledcWrite(1, PWM_value_g);
        ledcWrite(2, PWM_value_g);
        ledcWrite(3, PWM_value_g);
        ledcWrite(4, PWM_value_g);
        ledcWrite(5, PWM_value_orb);
        ledcWrite(6, PWM_value_orb);
        ledcWrite(7, PWM_value_orb);
        ledcWrite(8, PWM_value_orb);
        ledcWrite(9, PWM_value_orb);
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

      if ((DPF_Regen_b==0) && (DPF_LRD_calc_act <2))
      {
	      if (S11 == false)
	      {
        ledcWrite(1, 0);
        ledcWrite(2, 0);
        ledcWrite(3, 0);
        ledcWrite(4, 0);
        ledcWrite(5, 0);
        ledcWrite(6, 0);
        ledcWrite(7, 0);
        ledcWrite(8, 0);
        ledcWrite(9, 0);
        ledcWrite(10, 0);
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
      }
      mode_number = 1;
      break;
}
}

void illumination()
{
    ill_in_state = digitalRead(ILL_IN);
    if (ill_in_state == true)
    {
      PWM_value_g = 18000;
      PWM_value_orb = 45000;
      if (S20 == false)
        {
        mode_number = 6;
        S20 = true;
        S21 = false; 
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
        S12 = false; 
        }
     }
      else
     {
      PWM_value_g = 25;
      PWM_value_orb = 75;
      if (S21 == false)
        {
        mode_number = 6;
        S20 = false;
        S21 = true;  
        
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
        S12 = false;
        }
      }
  
}
