
typedef enum rako_msg_type
{
  RAKO_MSG_TYPE_COMMAND  = 0,
  RAKO_MSG_TYPE_DATA     = 2

} rako_msg_type_t;

typedef enum rako_cmd_id
{
  RAKO_CMD_OFF          = 0x0,
  RAKO_CMD_RAISE        = 0x1,
  RAKO_CMD_LOWER        = 0x2,
  RAKO_CMD_SCENE1       = 0x3,
  RAKO_CMD_SCENE2       = 0x4,
  RAKO_CMD_SCENE3       = 0x5,
  RAKO_CMD_SCENE4       = 0x6,
  RAKO_CMD_PROGRAM_MODE = 0x7,
  RAKO_CMD_IDENT        = 0x8,
  RAKO_CMD_SCENE_SET    = 0x9,
  RAKO_CMD_LOW_BATTERY  = 0xa,
  RAKO_CMD_EEPROM_WRITE = 0xb,
  RAKO_CMD_LEVEL_SET    = 0xc,
  RAKO_CMD_STORE        = 0xd,
  RAKO_CMD_EXIT         = 0xe,
  RAKO_CMD_STOP         = 0xf,
	
} rako_cmd_id_t;

// NOTE: Field order in the RakoMsg bit fields is reversed due 
//       to the uint32_t type being stored in little-endian order.
typedef union rako_msg rako_msg_t;
union rako_msg
{
  struct
  {
    uint32_t data : 28;
    uint32_t type : 4;

  } unknown;
  
  struct
  {
    uint32_t padding : 3;

    uint32_t check   : 1;
    uint32_t command : 4;
    uint32_t channel : 4;
    uint32_t room    : 8;
    uint32_t house   : 8;
    uint32_t type    : 4;

  } command;

  struct
  {
    uint32_t padding : 3;

    uint32_t check   : 1;
    uint32_t address : 8;
    uint32_t data    : 8;
    uint32_t house   : 8;
    uint32_t type    : 4;

  } data;

  uint32_t raw;

} __attribute__((packed));

typedef struct rako_cmd
{
  uint8_t house;
  uint8_t room;
  uint8_t channel;
  uint8_t command;
  uint8_t data;
  uint8_t address;

} rako_cmd_t;

extern process_event_t rako_recv_event;
extern process_event_t rako_sent_event;
extern process_event_t rako_cmd_event;

void rako_init();
void rako_send(rako_msg_t* msg);
void rako_send_two(rako_msg_t* msg1, rako_msg_t* msg2);

void rako_set_send_repeat(int count, int delay);

void rako_send_scene(int house, int room, int channel, int scene);
void rako_send_off(int house, int room, int channel);
void rako_send_raise(int house, int room, int channel);
void rako_send_lower(int house, int room, int channel);
void rako_send_level(int house, int room, int channel, int level);
void rako_send_stop(int house, int room, int channel);
void rako_send_ident(int house, int room, int channel);
void rako_send_store(int house, int room, int channel);

