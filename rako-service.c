#include <contiki.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include "rako-service.h"

#define PORTCAT(a,b,c)     a##b##c
#define PIN(port, pin)     PORTCAT(PIN,port,)
#define PINPIN(port, pin)   PORTCAT(PIN,port,pin)
#define PORT(port, pin)    PORTCAT(PORT,port,)
#define PORTPIN(port, pin) PORTCAT(PORT,port,pin)
#define DDR(port, pin)     PORTCAT(DDR,port,)
#define DD(port, pin)      PORTCAT(DD,port,pin)

#define BIT(i)             (1 << (i))
#define BIT_GET(value, i)  ((value) & BIT(i))
#define BIT_SET(value, i)  ((value) |= BIT(i))
#define BIT_CLR(value, i)  ((value) &= ~BIT(i))

#define PIN_GET(x)         BIT_GET(PIN x, PINPIN x)
#define PIN_HIGH(x)        BIT_SET(PORT x, PORTPIN x)
#define PIN_LOW(x)         BIT_CLR(PORT x, PORTPIN x)
#define PIN_OUTPUT(x)      BIT_SET(DDR x, DD x)
#define PIN_INPUT(x)       BIT_CLR(DDR x, DD x)

#define RADIO_TX_ENABLE  (G, 5)
#define RADIO_TX_DATA    (E, 3)
#define RADIO_RX_ENABLE  (E, 7)
#define RADIO_RX_DATA    (E, 6)
#define DEBUGPIN (G, 0)

#define RADIO_ENABLE_NONE  0
#define RADIO_ENABLE_RX    1
#define RADIO_ENABLE_TX    2

#define EDGE_TIME_MIN 450 /* uS */
#define EDGE_TIME_MAX 650 /* uS */

#define RAKO_FLAG_RECV_COMPLETE    (1 << 0)
#define RAKO_FLAG_SEND_COMPLETE    (1 << 1)
#define RAKO_FLAG_SEND_TWO         (1 << 2)

PROCESS(rako_process, "RAKO Process");

process_event_t rako_recv_event;
process_event_t rako_sent_event;
process_event_t rako_cmd_event;


static struct process* g_calling_process = NULL;
static uint8_t g_send_repeat = 4;
static uint16_t g_send_repeat_delay = 1;

static rako_msg_t g_send_msg[2];
static rako_cmd_t g_cmd;
static uint8_t g_send_index;

static volatile uint8_t g_flags = 0;
static volatile rako_msg_t g_recv_msg;
static volatile uint32_t g_msg_data;
static volatile uint8_t g_msg_data_len;
static volatile uint8_t g_msg_data_pos;
static volatile uint8_t g_send_data[16];
static volatile uint8_t g_delay = 0;

static uint32_t micros()
{
  uint8_t status = SREG;
  uint16_t i;

  cli();
  i = TCNT1L;
  i |= (uint16_t)TCNT1H << 8;
  SREG = status;

  return i << 3;  // 8uS per tick
}

ISR(INT6_vect)
{
  static uint32_t last_time = 0;
  static uint8_t last = 0;
  static uint8_t have_start_mark = 0;

  uint32_t current_time = micros();
  uint16_t duration = current_time - last_time;
  int current = PIN_GET(RADIO_RX_DATA);

  if(g_msg_data_len >= 32)
    goto RESTART;

  if(duration < EDGE_TIME_MIN)
    goto RESTART;

  if(last == 0)
  {
    if(duration > EDGE_TIME_MAX)
      goto RESTART;
  }
  else
  {
    if(duration > EDGE_TIME_MIN && duration < EDGE_TIME_MAX)
    {
      g_msg_data = (g_msg_data << 1) | 0;
      ++g_msg_data_len;
    }
    else if(duration > EDGE_TIME_MIN * 2 && duration < EDGE_TIME_MAX * 2)
    {
      g_msg_data = (g_msg_data << 1) | 1;
      ++g_msg_data_len;
    }
    else if(duration > EDGE_TIME_MIN * 4 && duration < EDGE_TIME_MAX * 4)
    {
      ++g_msg_data_len;
      if(have_start_mark != 1)
      {
        g_msg_data_len = 0;
        g_msg_data = 0;
        have_start_mark = 1;
      }
      else
      {
        if(g_msg_data_len >= 28)
        {
          g_recv_msg.raw = g_msg_data << 3;
          g_flags |= RAKO_FLAG_RECV_COMPLETE;
          process_poll(&rako_process);
        }
        goto RESTART;
      }
    }
    else
      goto RESTART;
  }

  last_time = current_time;
  last = current;
  return;

RESTART:
  last_time = current_time;
  last = current;
  g_msg_data_len = 0;
  g_msg_data = 0;
  have_start_mark = 0;
}

static void radio_enable(int radio)
{
  if(radio == RADIO_ENABLE_TX)
  {
    PIN_LOW(RADIO_RX_ENABLE);
    PIN_HIGH(RADIO_TX_ENABLE);
    BIT_CLR(EIMSK, INT6);
  }
  else if(radio == RADIO_ENABLE_RX)
  {
    PIN_LOW(RADIO_TX_ENABLE);
    PIN_HIGH(RADIO_RX_ENABLE);
    BIT_SET(EIMSK, INT6);
  }
  else
  {
    PIN_LOW(RADIO_TX_ENABLE);
    PIN_LOW(RADIO_RX_ENABLE);
    BIT_CLR(EIMSK, INT6);
  }
}

static void send_timer_start()
{
  radio_enable(RADIO_ENABLE_TX);
  TCNT1L = 0;
  TCNT1H = 0;
  BIT_SET(TIFR1, OCF1A);
  BIT_SET(TIMSK1, OCIE1A);
}

static void send_timer_stop()
{
  BIT_CLR(TIMSK1, OCIE1A);
  radio_enable(RADIO_ENABLE_RX);
}

ISR(TIMER1_COMPA_vect)
{
  TCNT1L = 0;
  TCNT1H = 0;

  if(g_delay > 0)
  {
    --g_delay;
    return;
  }

  if(g_msg_data_pos < g_msg_data_len)
  {
    if((g_send_data[g_msg_data_pos / 8] & (1 << (g_msg_data_pos % 8))) != 0)
      PIN_HIGH(RADIO_TX_DATA);
    else
      PIN_LOW(RADIO_TX_DATA);
    ++g_msg_data_pos;
  }
  else
  {
    send_timer_stop();
    g_flags |= RAKO_FLAG_SEND_COMPLETE;
    process_poll(&rako_process);
  }
}

inline int rako_is_send_in_progress()
{
  return PIN_GET(RADIO_TX_ENABLE);
}

static void send_data_reset()
{
  memset((void*)g_send_data, 0, sizeof(g_send_data));
  g_msg_data = 0;
  g_msg_data_len = 0;
  g_msg_data_pos = 0;
}

static void send_data_append(const char* bits)
{
  while(*bits != '\0')
  {
    if(*bits == '1')
      g_send_data[g_msg_data_len / 8] |= 1 << (g_msg_data_len % 8);
    ++g_msg_data_len;
    ++bits;
  }
}

static void rako_send_start(rako_msg_t* msg, int delay)
{
  int i;
  uint32_t data = msg->raw;
  int check = 0;

  // disable both radios to make sure nothing is touching the message buffer
  radio_enable(RADIO_ENABLE_NONE);

  send_data_reset();

  // Preamble and start mark
  send_data_append("10101011110");

  // Message body
  for(i = 0; i < 28; ++i)
  {
    if(data & 0x80000000UL)
    {
      ++check;
      send_data_append("110");
    }
    else
      send_data_append("10");
    data <<= 1;
  }

  // Check bit
  if(check & 1)
    send_data_append("110");
  else
    send_data_append("10");

  // Stop mark
  send_data_append("11110");

  g_delay = delay;
  send_timer_start();
}

void rako_send(rako_msg_t* msg)
{
  g_send_msg[1].raw = msg->raw;
  g_flags &= ~RAKO_FLAG_SEND_TWO;
  g_send_index = 1;
  rako_send_start(&g_send_msg[0], 0);
}

void rako_send_two(rako_msg_t* msg1, rako_msg_t* msg2)
{
  g_send_msg[0].raw = msg1->raw;
  g_send_msg[1].raw = msg2->raw;
  g_flags |= RAKO_FLAG_SEND_TWO;
  g_send_index = 0;
  rako_send_start(&g_send_msg[0], 0);
}

void rako_send_scene(int house, int room, int channel, int scene)
{
  if(scene <= 4)
  {
    rako_msg_t msg = 
    {
      .command.type = RAKO_MSG_TYPE_COMMAND,
      .command.house = house,
      .command.room = room,
      .command.channel = channel,
      .command.command = RAKO_CMD_SCENE1
    };
    rako_send(&msg);
  }
  else
  {
    rako_msg_t cmd = 
    {
      .command.type = RAKO_MSG_TYPE_COMMAND,
      .command.house = house,
      .command.room = room,
      .command.channel = channel,
      .command.command = RAKO_CMD_SCENE_SET
    };
    rako_msg_t data = 
    {
      .data.type = RAKO_MSG_TYPE_DATA,
      .data.house = house,
      .data.data = scene,
      .data.address = 0
    };
    rako_send_two(&cmd, &data);
  }
}

void rako_send_off(int house, int room, int channel)
{
  rako_msg_t msg = 
  {
    .command.type = RAKO_MSG_TYPE_COMMAND,
    .command.house = house,
    .command.room = room,
    .command.channel = channel,
    .command.command = RAKO_CMD_OFF
  };
  rako_send(&msg);
}

void rako_send_raise(int house, int room, int channel)
{
  rako_msg_t msg = 
  {
    .command.type = RAKO_MSG_TYPE_COMMAND,
    .command.house = house,
    .command.room = room,
    .command.channel = channel,
    .command.command = RAKO_CMD_RAISE
  };
  rako_send(&msg);
}

void rako_send_lower(int house, int room, int channel)
{
  rako_msg_t msg = 
  {
    .command.type = RAKO_MSG_TYPE_COMMAND,
    .command.house = house,
    .command.room = room,
    .command.channel = channel,
    .command.command = RAKO_CMD_LOWER
  };
  rako_send(&msg);
}

void rako_send_level(int house, int room, int channel, int level)
{
    rako_msg_t cmd = 
    {
      .command.type = RAKO_MSG_TYPE_COMMAND,
      .command.house = house,
      .command.room = room,
      .command.channel = channel,
      .command.command = RAKO_CMD_LEVEL_SET
    };
    rako_msg_t data = 
    {
      .data.type = RAKO_MSG_TYPE_DATA,
      .data.house = house,
      .data.data = level,
      .data.address = 0
    };
    rako_send_two(&cmd, &data);
}

void rako_send_stop(int house, int room, int channel)
{
  rako_msg_t msg = 
  {
    .command.type = RAKO_MSG_TYPE_COMMAND,
    .command.house = house,
    .command.room = room,
    .command.channel = channel,
    .command.command = RAKO_CMD_STOP
  };
  rako_send(&msg);
}

void rako_send_ident(int house, int room, int channel)
{
  rako_msg_t msg = 
  {
    .command.type = RAKO_MSG_TYPE_COMMAND,
    .command.house = house,
    .command.room = room,
    .command.channel = channel,
    .command.command = RAKO_CMD_IDENT
  };
  rako_send(&msg);
}

void rako_send_store(int house, int room, int channel)
{
  rako_msg_t msg = 
  {
    .command.type = RAKO_MSG_TYPE_COMMAND,
    .command.house = house,
    .command.room = room,
    .command.channel = channel,
    .command.command = RAKO_CMD_STORE
  };
  rako_send(&msg);
}

void rako_init()
{
  // Set timer 0 to clk/64 => 1 tick every 8 uS
  TCCR1A = 0;
  TCCR1B = BIT(CS00) | BIT(CS01);
  // compare after 69 ticks => 552uS
  OCR1AH = 0;
  OCR1AL = 69;
  //disable output compare interrupt (will enable when transmitting)
  BIT_CLR(TIMSK1, OCIE1A);

  EIMSK &= ~BIT(INT6);  // disable int6, will enable when radio_enable(RADIO_ENABLE_RX);
  EICRB &= ~BIT(ISC61); // int 6 triggers on any edge
  EICRB |= BIT(ISC60); // "
  EIFR |= BIT(INTF6);  // clear int6 flag

  PIN_LOW(RADIO_TX_ENABLE);
  PIN_LOW(RADIO_TX_DATA);
  PIN_LOW(RADIO_RX_ENABLE);
  PIN_LOW(RADIO_RX_DATA);
  PIN_OUTPUT(RADIO_TX_ENABLE);
  PIN_OUTPUT(RADIO_TX_DATA);
  PIN_OUTPUT(RADIO_RX_ENABLE);
  PIN_INPUT(RADIO_RX_DATA);

  radio_enable(RADIO_ENABLE_RX);

  g_calling_process = PROCESS_CURRENT();
  rako_recv_event = process_alloc_event();
  rako_sent_event = process_alloc_event();
  rako_cmd_event = process_alloc_event();
  process_start(&rako_process, NULL);
}

static int send_complete()
{
  if((g_send_index & 1) && g_send_index >> 1 >= g_send_repeat - 1)
  {
    process_post(g_calling_process, rako_sent_event, NULL);
    return 0;
  }
  else
  {
    if(g_flags & RAKO_FLAG_SEND_TWO)
      return g_send_index + 1;
    else
      return g_send_index + 2;
  }
}

static void recv_complete()
{
  process_post(g_calling_process, rako_recv_event, (void*)&g_recv_msg);

  if(g_recv_msg.unknown.type == RAKO_MSG_TYPE_COMMAND)
  {
    g_cmd.house = g_recv_msg.command.house;
    g_cmd.room = g_recv_msg.command.room;
    g_cmd.channel = g_recv_msg.command.channel;
    g_cmd.command = g_recv_msg.command.command;

    if(!(g_recv_msg.command.command == RAKO_CMD_LEVEL_SET || 
         g_recv_msg.command.command == RAKO_CMD_SCENE_SET || 
         g_recv_msg.command.command == RAKO_CMD_EEPROM_WRITE))
    {
      g_cmd.data = 0;
      g_cmd.address = 0;

      if(g_recv_msg.command.command >= RAKO_CMD_SCENE1 && g_recv_msg.command.command <= RAKO_CMD_SCENE4)
      {
        g_cmd.command = RAKO_CMD_SCENE_SET;
        g_cmd.data = 1 + (g_recv_msg.command.command - RAKO_CMD_SCENE1);
      }

      process_post(g_calling_process, rako_cmd_event, (void*)&g_cmd);
    }
  }
  else if(g_recv_msg.unknown.type == RAKO_MSG_TYPE_DATA)
  {
    if(g_cmd.house == g_recv_msg.data.house)
    {
      g_cmd.data = g_recv_msg.data.data;
      g_cmd.address = g_recv_msg.data.address;
      process_post(g_calling_process, rako_cmd_event, (void*)&g_cmd);
    }
    else
      g_cmd.house = 0;
  }
  else
    g_cmd.house = 0;
}

PROCESS_THREAD(rako_process, ev, data)
{
  PROCESS_BEGIN();

  for(;;)
  {
    PROCESS_WAIT_EVENT();

    if(ev == PROCESS_EVENT_POLL)
    {
      if(g_flags & RAKO_FLAG_RECV_COMPLETE)
      {
        g_flags &= ~RAKO_FLAG_RECV_COMPLETE;
        recv_complete();
      }
      else if(g_flags & RAKO_FLAG_SEND_COMPLETE)
      {
        g_flags &= ~RAKO_FLAG_SEND_COMPLETE;
        g_send_index = send_complete();
        if(g_send_index != 0)
          rako_send_start(&g_send_msg[g_send_index & 1], 128);
      }
    }
  }

  PROCESS_END();
}

