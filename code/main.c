#include "MKL05Z4.h"
#include "ADC.h"
#include "frdm_bsp.h"
#include "lcd1602.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "uart0.h"

#define RED_LED_POS 8     //pin diody czerwonej 
#define BLUE_LED_POS 10   //pin diody niebieskiej

float volt_coeff = ((float)(((float)2.91) / 4095) );			// Współczynnik korekcji wyniku, w stosunku do napięcia referencyjnego przetwornika
uint8_t wynik_ok=0;
uint16_t temp;
float	wynik, wynik_eff;   //wartość z przetwornika AC

char rx_buf[]={0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20};

int	time=0;      //
int j=0;         //
int k=0;         //
int y=0;         //

char macierz[64][7];  /////////////// tablica dwuwymiarowa, 1 indeks to wartosc dziesietna znaku , 2 indeks to ilość elementow

char sign;

char sign_1[]={0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20};                          //tablice do przechowywania danych przy konwersji sygnału na znak

int number;
char znak_1;  //znak końcowy
int num_tab[8]={0,0,0,0,0,0,0,0};
int num_tab_0[8]={0,0,0,0,0,0,0,0};

void signal(void)   //funkcja do określenia czy sygnał jest krótki czy długi, dodatkowo służy do informowania czy trwa dalej nadawanie znaku czy nie 
{
	    if(time>=3)   //jeżeli time większy od 3 to ,,zwracany" jest element '-'
			{
				LCD1602_ClearAll();
				LCD1602_SetCursor(0,0);
				LCD1602_Print("Sygnal: ");
				sign='-';   //przypisanie elementu do zmiennej
				sign_1[y]='-';
				LCD1602_SetCursor(8,0);
		    LCD1602_Print(sign_1);  //wyświetlenie elementu '-'
				
			}
		  else if(time<3 && time>0)  //jeżeli time jest większy od zera i mniejszy od 3 to ,,zwracany" jest element '.'
			{
				LCD1602_ClearAll();
				LCD1602_SetCursor(0,0);
				LCD1602_Print("Sygnal: ");
				sign='.';    //przypisanie elementu do zmiennej
				sign_1[y]='.';
		    LCD1602_Print(sign_1); //wyświetlenie elementu '.'
			}
			else if(time==0 && k>0 && k<=3)  //jeżeli jest spełniony warunek to program informuje że nie odbierany jest żaden sygnał ale na niego czeka (nadawanie konkretnego znaku nadal trwa) 
			{
				LCD1602_SetCursor(0,0);
				LCD1602_Print("                ");
			  LCD1602_SetCursor(0,1);
				LCD1602_Print("Nadawanie w toku");  //wyświetlenie na lcd komunikatu 
				sign = 'C';
			  
			}	
			else if(time==0 && k>3 && k<=4)  //informuje ze sygnal jest dekodowany
			{
				LCD1602_SetCursor(0,1);
				LCD1602_Print("Dekodowanie...  ");  //wyświetlenie na lcd komunikatu 
				sign='D';
			}
			
			else if((time==0 && k>4))  //informuje ze nadawanie się zakończyło
			{
				LCD1602_SetCursor(0,0);
				LCD1602_Print("                ");
				LCD1602_SetCursor(0,1);
				LCD1602_Print("Koniec znaku..  ");  //wyświetlenie na lcd komunikatu 
				sign='Q';
			  sign_1[0]=0x20;
				sign_1[1]=0x20;
				sign_1[2]=0x20;
				sign_1[3]=0x20;
				sign_1[4]=0x20;
				sign_1[5]=0x20;
				sign_1[6]=0x20;
			}
			
}

void LED(void)  //funkcja, która informuje jaki mamy znak w trakcie nadawania sygnału na czujnik 
{
	if(j>0 && j<3)  //sygnał '.' to dioda czerwona  
	{
		PTB->PDOR|=(1<<BLUE_LED_POS);
		PTB->PDOR&=~(1<<RED_LED_POS);
	}
	else if(j>=3)   //sygnał '-' to dioda niebieska 
	{
		PTB->PDOR|=(1<<RED_LED_POS);
		PTB->PDOR&=~(1<<BLUE_LED_POS);
	}
	else if(j==0)    //zgaszone diody
	{
		PTB->PDOR|=(1<<BLUE_LED_POS);
		PTB->PDOR|=(1<<RED_LED_POS);
	}
}

void value(void)   //elementy 0 i 1 są grupowane w tablice, np.00011 
{

	if(sign=='.')
	{
		num_tab_0[y]=0; 
		++y;
	}
	else if(sign=='-')
	{
		num_tab_0[y]=1; 
		++y;
	}
	else if(sign=='C')
	{
		///nic się nie dzieje		
	}
	else if(sign=='D')   //kiedy nastąpi sygnał dekodowania to każdy element przypisywany jest do tablicy która będzie dekodowana w dalszej części kodu
	{
		num_tab[0]=num_tab_0[0];    
		num_tab[1]=num_tab_0[1];
		num_tab[2]=num_tab_0[2];
		num_tab[3]=num_tab_0[3];
		num_tab[4]=num_tab_0[4];
		num_tab[5]=num_tab_0[5];
		num_tab[6]=num_tab_0[6];
		num_tab[7]=num_tab_0[7];
	}
	else if (sign=='Q') //zerowanie tablicy 
	{
		y=0;
	
		num_tab_0[0]=0;
		num_tab_0[1]=0;
		num_tab_0[2]=0;
		num_tab_0[3]=0;
		num_tab_0[4]=0;
		num_tab_0[5]=0;
		num_tab_0[6]=0;
		num_tab_0[7]=0;
	}
}

void display_sign(void) //funkcja służąca do wysyłania znaku do komputera przez UART
{
	UART0->S1 & UART0_S1_TDRE_MASK;
	UART0->D = znak_1;
	znak_1=0x20;
}

void decoded_sign(void)   //funkcja, która dekoduje sygnał
{
	
			number=num_tab[0]*1+num_tab[1]*2+num_tab[2]*4+num_tab[3]*8+num_tab[4]*16+num_tab[5]*32+num_tab[6]*64;  
			                                            ///////////////////////////////obliczenie wartosci znaku do systemu dziesietnego
          
			if(y==0)              
			{
					//continue
			}
			else if(y==1)
			{
				znak_1=macierz[number][0];
				display_sign();
				
			}
			else if(y==2)
			{
				znak_1=macierz[number][1];
				display_sign();
			}
			else if(y==3)
			{
				znak_1=macierz[number][2];
				display_sign();
			}
			else if(y==4)
			{
				znak_1=macierz[number][3];
				display_sign();
			}
			else if(y==5)
			{
				znak_1=macierz[number][4];
				display_sign();
			}
			else if(y==6)
			{
				znak_1=macierz[number][5];
				display_sign();
			}
			
}

void ADC0_IRQHandler()  //handler do przetwornika AC
{	
	
	temp = ADC0->R[0];	// Odczyt danej i skasowanie flagi COCO
	if(!wynik_ok)				// Sprawdź, czy wynik skonsumowany przez pętlę główną
	{
		wynik = temp;			// Wyślij nową daną do pętli głównej
		wynik_eff = wynik*volt_coeff; // Dostosowanie wyniku do zakresu napięciowego
		wynik_ok=1;
		
		if(wynik_eff>=2)  //jeżeli na czujnik pada światło (nadajemy sygnał) to liczymy czas przez jaki to trwało
			{
				++j;
				k=0;   //zerujemy zmienną do liczenia czasu kiedy na czujnik nie padało światło
			}
	  else if(wynik_eff<2) //liczymy czas przez który na czujnik nie padało światło
			{
				++k;        
				time=j;    //przypisujemy liczbę do zmiennej time 
		    j=0;       //zerujemy zmienną do liczenia czasu kiedy na czujnik padało światło
	    }	
	}
	NVIC_EnableIRQ(ADC0_IRQn);
}

int main (void)
{
	uint8_t	kal_error;
  
	macierz[2][1]='A';   // uzupełnienie tablicy kodujacej
  macierz[1][3]='B';
  macierz[5][3]='C';
  macierz[1][2]='D';
  macierz[0][0]='E';
  macierz[4][3]='F';
  macierz[6][2]='W';   
  macierz[0][3]='H';
  macierz[0][1]='I';
  macierz[14][3]='J';
  macierz[5][2]='K';
  macierz[2][3]='L';
  macierz[3][1]='M';
  macierz[1][1]='N';
  macierz[7][2]='O';
  macierz[6][3]='P';
  macierz[11][3]='Q';
  macierz[2][2]='R';
  macierz[0][2]='S';
  macierz[1][0]='T';
  macierz[4][2]='U';
  macierz[8][3]='V';
  macierz[3][2]='G';   
  macierz[9][3]='X';
  macierz[13][3]='Y';
  macierz[3][3]='Z';
  macierz[42][5]='.';
	macierz[51][5]=',';
	macierz[30][5]=0x27;
  macierz[18][5]='"';
	macierz[44][5]='_';
	macierz[7][5]=':';
	macierz[21][5]=';';
	macierz[12][5]='?';
	macierz[53][5]='!';
	macierz[33][5]='-';
	macierz[45][5]=')';
	macierz[22][5]='@';
	macierz[10][4]='+';
	macierz[9][4]='/';
	macierz[13][4]='(';
	macierz[17][4]='=';
	macierz[30][4]='1';
	macierz[28][4]='2';
	macierz[24][4]='3';
	macierz[16][4]='4';
	macierz[0][4]='5';
	macierz[1][4]='6';
	macierz[3][4]='7';
	macierz[7][4]='8';
	macierz[15][4]='9';
	macierz[31][4]='0';
	macierz[15][3]='^';
	macierz[7][3]='%';
	macierz[10][3]=0x5c;
	macierz[12][3]='#';
	
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;  // komendy do zadeklarowania diod led w programie
	                                     //
	PORTB->PCR[8] |= PORT_PCR_MUX(1);    //
	PTB->PDDR |= (1<<RED_LED_POS);       //
	                                     //
	PORTB->PCR[10] |= PORT_PCR_MUX(1);   //
	PTB->PDDR |= (1<<BLUE_LED_POS);      //
	
	LCD1602_Init();		 // Inicjalizacja wyświetlacza LCD
	LCD1602_Backlight(TRUE);
					
	kal_error=ADC_Init();				// Inicjalizacja i kalibracja przetwornika A/C
	
  
	
	if(kal_error)
	{	
		while(1);									// Klaibracja się nie powiodła
	}
	
	ADC0->SC1[0] = ADC_SC1_AIEN_MASK | ADC_SC1_ADCH(12);		// Pierwsze wyzwolenie przetwornika ADC0 w kanale 12 i odblokowanie przerwania
	
	UART0_Init();		// Inicjalizacja portu szeregowego UART0
	
	while(1)   //nieskończona pętla while 
	{
		if(wynik_ok) 
		{
			LED();   
			signal();
			value();
			
		  if(y>6)            //jezeli ciag elementow jest za dlugi
			{
				y=0;
				sign_1[0]=0x20;
				sign_1[1]=0x20;
				sign_1[2]=0x20;
				sign_1[3]=0x20;
				sign_1[4]=0x20;
				sign_1[5]=0x20;
				sign_1[6]=0x20;
				sign_1[7]=0x20;
				LCD1602_SetCursor(0,0);
				LCD1602_Print("Error           ");
			  LCD1602_SetCursor(0,1);
				LCD1602_Print("Reset indeksu   ");
				DELAY(250)
			}
			
			if(sign=='D')     //dekodowanie sygnalu
			{
				decoded_sign();
			}
			
			DELAY(500)
			wynik_ok=0;
	}
}
}
