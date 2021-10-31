#include "stm32f4xx.h"

void Init();

void Deinit();

void LEDOff(char color);

void LEDOn(char color);

void TIM6Enable(int prescaler, int cout, int interrupt);

void TIM1Enable(int prescaler, int cout, int interrupt);

void PWMTim1(int comp);

void PWMTim1Update(int duty);

void GenerateSignal(float* signal, int f, int f_s, int n_s, int sig);

void LEDSEnable();

void ADCEnable();

void GPIOAEnable(int pin, char mode, int interrupt);

void GPIOAOn(int pin);

void GPIOAOff(int pin);

void GPIOBEnable(int pin, char mode, int interrupt);

void GPIOBOn(int pin);

void GPIOBOff(int pin);

void ButtonEnable(int interrupt);

int ButtonPushed();

void CountEnable(int time);

int CountCheck();

void PWM__Start();

void PWM__Init();
