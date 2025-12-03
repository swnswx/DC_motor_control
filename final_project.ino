#define DWT_CTRL      (*((volatile uint32_t *)(0xE0001000UL)))
#define DWT_CYCCNT    (*((volatile uint32_t *)(0xE0001004UL)))

volatile uint32_t pwm_rise_cycle = 0;
volatile uint32_t pwm_last_rise_cycle = 0;
volatile uint32_t pwm_high_cycle = 0;
volatile uint32_t pwm_period_cycle = 0;
volatile uint8_t pwm_first_rise = 0;

void DWT_Init();
void configure_pio_interrupt();

float sim_time=0;
uint32_t start_time, end_time;
uint32_t SampleTimeCycle;
uint32_t position_ref;

float sample_time=0.005;

void setup()
{
  Serial.begin(115200);
  
  DWT_Init();
  configure_pio_interrupt();
  
  SampleTimeCycle = (uint32_t)(sample_time*84000000.0f);
  start_time=DWT_CYCCNT;
  end_time=start_time+SampleTimeCycle;
}

void loop()
{
  while(!((end_time-DWT_CYCCNT)&0x80000000));
  end_time+=SampleTimeCycle;

  uint32_t high, period;

  high=pwm_high_cycle;
  period=pwm_period_cycle;

  if(period>0)
  {
    float duty=(float)high/(float)period;

    if(duty<0.2f) duty=0.2f;
    if(duty>0.8f) duty=0.8f;

    position_ref=100*(duty-0.5)/0.3;

    Serial.println(duty,4);
    
    //char buf[80];
    //sprintf(buf, "high=%lu period=%lu duty=%.3f pos_ref=%.2f\r\n", high, period, duty, position_ref);
    //Serial.print(buf);
  }
}

void DWT_Init()
{
  CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;
  DWT_CYCCNT=0;
  DWT_CTRL|=1;
}

void configure_pio_interrupt()
{
  pmc_enable_periph_clk(ID_PIOC); //PIOC에 clock 공급

  PIOC->PIO_AIMDR=PIO_PC5; //PC5의 interrupt를 default인 both edge detection으로 설정
  PIOC->PIO_IDR=PIO_PC5;   //PC5의 interrupt를 disable 시킨다.
  PIOC->PIO_PER=PIO_PC5;   //PC5의 PIO 기능을 enable 시킨다.
  PIOC->PIO_ODR=PIO_PC5;   //PC5를 input으로 설정한다.

#if 1 //빠른 신호일 때 사용
  PIOC->PIO_SCIFSR=PIO_PC5; //System clock glitch filter가 Tmck/2보다 system clock의 작은 glitch를 걸러낸다.
#endif

#if 0 //느린 신호일 때 사용
  PIOC->PIO_DIFSR=PIO_PC5; //Glitch filter가 duration이 Tdiv_sick/2보다 작은 pulse를 걸러낸다.
  //이걸 사용하면 slow clock 기준으로 처리하기 때문에 interrupt latency가 커진다. 느린 신호에 사용할 것

  PIOC->PIO_SCDR = 0; //이건 slow clock을 사용할 때의 분주비 조정할 때 사용
#endif

  PIOC->PIO_IFER=PIO_PC5; //input filter enable register

  PIOC->PIO_IER=PIO_PC5;  //interrupt enable

  NVIC_DisableIRQ(PIOC_IRQn);
  NVIC_ClearPendingIRQ(PIOC_IRQn);
  NVIC_SetPriority(PIOC_IRQn,1);

  NVIC_EnableIRQ(PIOC_IRQn);
}

void PIOC_Handler(void)
{
  uint32_t status, PC_value, now;
  status = PIOC->PIO_ISR; //Interrupt status를 읽어들인다. 이렇게 읽으면 ISR은 다시 0으로
                          //되기 때문에 ISR을 읽어들인 후 해당 되는 interrupt를 다 수행해 주어야 함.
  
  PC_value = PIOC->PIO_PDSR; //PIOC를 읽어 들이자

  now = DWT_CYCCNT;      //현재 cycle counter 읽기
  
  if(PC_value&PIO_PC5) //PC5=1인경우 즉 rising edge
  {
    if(pwm_first_rise)
    {
      pwm_period_cycle=(uint32_t)((int32_t)(now-pwm_last_rise_cycle));
    }
    else                    //PC5=0인 경우, 즉 falling edge
    {
      pwm_first_rise=1;
    }
    pwm_last_rise_cycle=now;
    pwm_rise_cycle=now;
  }
  else
  {
    pwm_high_cycle=(uint32_t)((int32_t)(now-pwm_rise_cycle));
  }
}
