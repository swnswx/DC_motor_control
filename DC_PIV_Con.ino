#define DWT_CTRL    (*((volatile uint32_t *)(0xE0001000UL)))
#define DWT_CYCCNT  (*((volatile uint32_t *)(0xE0001004UL)))
#define PI 3.141592653589793
#define PPR 64

void DWT_Init();        //DWT를 사용할 수 있도록 초기화하는 함수
void configure_pio_interrupt();
void configure_encoder_counter();
void configure_PWM();

float sample_time = 0.005;      //sampling time. 200Hz
uint32_t MicrosSampleTime;      //micro-second 단위로 환산한 sample time
uint32_t MicrosCycle;           //1 micro-second에 해당하는 instruction cycle 수
uint32_t SampleTimeCycle;       //sample time에 해당하는 instruction cycle 수
uint32_t start_time, end_time;  //주기적 연산의 시작시간과 종료시간. instruction cycle의 수를 단위로 사용
volatile uint32_t duty_us;         //PWM duty
volatile uint32_t rising_edge, falling_edge, high_signal;

float position_ref, position_ref_prev, position_ref_dot;
float position_error, speed_error, speed_error_kv, speed_error_ki, anti_windup=0.0f;
//float u_unsat=0.0f, u_sat=0.0f;

//encoder
float speed;
float position;
int32_t cnt1, cnt2;
int32_t cnt1_p, cnt2_p;
float count_to_radian;
char buffer[128];

class PI_controller
{
public:
  PI_controller();
  ~PI_controller();
  float calc(float position_ref, float position, float position_ref_dot, float speed);
public:
  float kp, ki, kv;
  float integrator_state;
  float aw_gain;
};

PI_controller PIV;  //PIV 객체 생성

void setup() {

  Serial.begin(115200);
  
  MicrosSampleTime=(uint32_t)(sample_time*1e6);   //sample time에 해당하는 micro second 단위의 정수를 계산
  MicrosCycle=SystemCoreClock/1000000;            //due의 systemcoreclock은 84MHz임. 따라서 1us에 해당하는 cycle 수는 84

  SampleTimeCycle=MicrosCycle*MicrosSampleTime;

  DWT_Init();
  configure_pio_interrupt();
  configure_encoder_counter();
  configure_PWM();
  
  start_time=DWT_CYCCNT;                    //cycle counter를 읽어 들여 현재 시간을 기록한다.
  end_time=start_time+SampleTimeCycle;      //주기적 연산이 종료되야 하는 시점이 어디인지를 계산, 이때 단위는 instruction cycle 수이다.

  cnt1_p=0;
  cnt2_p=0;
  count_to_radian=2*PI/PPR;
}

void loop() {
  uint32_t duty;

  cnt1 = TC0->TC_CHANNEL[0].TC_CV;
  speed = count_to_radian*(cnt1-cnt1_p)/sample_time;
  position = count_to_radian*cnt1;

  cnt1_p=cnt1;

  if(high_signal==0)
  {
    duty_us=0;
    position_ref=0.0f;

    position_ref_prev=0.0f;
    position_ref_dot=0.0f;
    
    PIV.integrator_state=0.0f;
    //u_unsat=0.0f;
    //u_sat=0.0f;
  }
  else
  {
    duty_us = high_signal/MicrosCycle;     //us단위의 duty가 나옴

    position_ref = ((float)duty_us-500)/3.0f;  //position_ref 생성 -> -100 ~ 100rad

    position_ref_dot=(position_ref-position_ref_prev)/sample_time;
    position_ref_prev=position_ref;
  }

  float u_sat = PIV.calc(position_ref, position, position_ref_dot, speed);
  
  if(u_sat>=0)
  {
    PIOC->PIO_SODR=PIO_PC3;
  }
  else
  {
    PIOC->PIO_CODR=PIO_PC3;
    u_sat = -u_sat;
  }
  
  duty = (uint32_t)(u_sat/12*2100);  //0~12v를 0~2100으로 매칭하고, -12~0은 dir을 바꿔줌(0 or 1)

  sprintf(buffer, "%f %f\n", position_ref, position);
  Serial.println(buffer);
  //sprintf(buffer, "%f %f %f\n", duty, u_unsat, u_sat);
  //Serial.println(duty);
  PWM->PWM_CH_NUM[0].PWM_CDTYUPD = duty;
  PWM->PWM_SCUC=1;     //UPDULOCK=1. 이 register에는 0번 bit만 의미가 있다.

  //루틴 종료했으므로 while-loop 돌면서 시간 보내면 됨. end_time이 unsigned integer이므로 end_time-DWT_CYCCNT를 수행한 결과의 최상위
  //bit가 1이라는 것은 DWT_CYCCNT로 측정한 현재 시간이 end_time을 넘어섰다는 것을 의미.
  while(!((end_time-DWT_CYCCNT)&0x80000000));
  
  // 이번 연산을 마쳤으므로 이제 다음번 연산이 끝나야 되는 시점이 어디인지를 계산.
  end_time+=SampleTimeCycle;
}

void DWT_Init()
{
  CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;
  DWT_CYCCNT=0;
  DWT_CTRL|=1;
}

void configure_pio_interrupt()
{
  pmc_enable_periph_clk(ID_PIOC);   //PIOC에 clock을 공급.

  PIOC->PIO_AIMDR = PIO_PC5;        //PC5의 interrupt를 default인 both edge detection으로 설정
  PIOC->PIO_IDR   = PIO_PC5;        //PC5의 interrupt를 disable
  PIOC->PIO_PER   = PIO_PC5;        //PC5의 PIO 기능을 enable
  PIOC->PIO_ODR   = PIO_PC5;        //PC5를 input으로 설정

  PIOC->PIO_SCIFSR = PIO_PC5;       //System clock glitch filter가 Tmck/2보다 system clock의 작은 glitch를 걸러냄

  PIOC->PIO_IFER = PIO_PC5;         //input filter enable register

  PIOC->PIO_IER = PIO_PC5;          //interrupt enable

  NVIC_DisableIRQ(PIOC_IRQn);
  NVIC_ClearPendingIRQ(PIOC_IRQn);
  NVIC_SetPriority(PIOC_IRQn,1);
  NVIC_EnableIRQ(PIOC_IRQn);
}

void configure_encoder_counter()
{
  PIOB->PIO_PDR = PIO_PB25|PIO_PB27;      //PB25와 PB27의 PIO 기능을 disable
  PIOB->PIO_ABSR |= (PIO_PB25|PIO_PB27);  //PB25와 PB27을 peripheral option B로 기능 설정
  pmc_enable_periph_clk(ID_TC0);          //TC0에 clock 공급
  TC0->TC_CHANNEL[0].TC_CMR=5;            //XC0를 clock source로 설정하고 mode는 capture mode로 설정

  TC0->TC_BMR = (1<<9)|(1<<8)|(1<<12)|(1<<19)|(30<<20); //x4 method(4체배)

  TC0->TC_CHANNEL[0].TC_CCR = 5;
}

void configure_PWM()
{
  PIOC->PIO_PDR = PIO_PC2;
  PIOC->PIO_ABSR |= PIO_PC2;

  PIOC->PIO_PER |= PIO_PC3;
  PIOC->PIO_OER |= PIO_PC3;
  PIOC->PIO_CODR = PIO_PC3;
  
  pmc_enable_periph_clk(ID_PWM);    //Peripheral PWM에 공급되는 clock을 enable 시킨다.

  PWM->PWM_DIS = 1u<<0;             //PWM channel 0번 disable

  PWM->PWM_CLK&=~0x0F000F00;        //Prea=0, Preb=0, MCK를 사용하겠다는 말
  PWM->PWM_CLK&=~0x00FF0000;        //DIVB=0, CLKB는 turned off된다.
  PWM->PWM_CLK&=~0x000000FF;        //Clear DIVA
  PWM->PWM_CLK|=1u<<0;              //DIVA=1

  PWM->PWM_CH_NUM[0].PWM_CMR&=~0x0000000F;    //CPRE를 모두 0으로 clear
  PWM->PWM_CH_NUM[0].PWM_CMR|=0xB;            //CPRE=0xB=0b1011 (CLKA 선택)
  PWM->PWM_CH_NUM[0].PWM_CMR|=1u<<8;          //CALG=1, center aligned
  PWM->PWM_CH_NUM[0].PWM_CMR&=~(1u<<9);       //CPOL=0

  PWM->PWM_CH_NUM[0].PWM_CMR|=(1u<<16);       //DTE=1, Dead-time generator enable

  PWM->PWM_CH_NUM[0].PWM_DT=0x00030003;       //CH0: DTL=3, DTH=3

  PWM->PWM_CH_NUM[0].PWM_CPRD=2100;

  PWM->PWM_CH_NUM[0].PWM_CDTY=0;

  PWM->PWM_SCM|=0x00000007;
  PWM->PWM_SCM&=~0x00030000;

  PWM->PWM_ENA=1u<<0;
}

void PIOC_Handler(void)
{
  uint32_t status, PC_value;

  status = PIOC->PIO_ISR; //interrupt status를 읽어들인다. 이렇게 읽으면 ISR은 다시 0으로 되기 때문에
                          //ISR을 읽어들인 후 해당되는 interrupt를 다 수행해 주어야 한다.

  PC_value = PIOC->PIO_PDSR;  //PIOC를 읽어 들이자

  if(status&PIO_PC5)
  {
    if(PC_value&PIO_PC5)  //PC5가 1인 경우, 즉 rising edge
    {
      rising_edge=DWT_CYCCNT;
    }
    else                  //PC5가 0인 경우, 즉 falling edge
    {
      falling_edge=DWT_CYCCNT;
      high_signal = falling_edge-rising_edge;
    }
  }
}

PI_controller::PI_controller()
{
  kp = 90;
  ki = 0.8;
  kv = 0.05;

  integrator_state = 0.0;

  aw_gain = 100.0;
}

float PI_controller::calc(float position_ref, float position, float position_ref_dot, float speed)  
{
  float u_sat = 0.0f;
  float position_error = (position_ref - position)*kp;
  float speed_error = position_error - speed + (position_ref_dot*0.0);
  float speed_error_kv = speed_error*kv;
  float speed_error_ki = speed_error*ki;

  float u_unsat = integrator_state+speed_error_kv;

  if(u_unsat>=12.0) u_sat = 12.0;
  else if(u_unsat<=-12.0) u_sat = -12.0f;
  else  u_sat = u_unsat;

  float anti_windup = (u_sat - u_unsat)*aw_gain;
  integrator_state += (speed_error_ki+anti_windup)*sample_time;

  return u_sat;
}

PI_controller::~PI_controller()   //소멸자
{
  
}
