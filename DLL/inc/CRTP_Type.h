#ifndef _CRTP_TYPE_H_
#define _CRTP_TYPE_H_
#ifdef __cplusplus
extern "C" {
#endif

#define CRTP_MAX_DATA_SIZE 30
  
typedef enum CRTPPort{
  CRTP_PORT_CONSOLE     = 0x00,
  CRTP_PORT_PARAM       = 0x02,
  CRTP_PORT_COMMANDER   = 0x03,
  CRTP_PORT_LOG         = 0x05,  
  CRTP_PORT_PID         = 0X06,  
  CRTP_PORT_DEBUG       = 0x08,  //for nRF24L01 debug
  CRTP_PORT_PLATFORM    = 0x0D,
  CRTP_PORT_LINK        = 0x0F,
}CRTPPort;

typedef struct _CRTPPacket
{
  uint8_t size;
   __packed union {
     __packed struct {
       __packed union {
        uint8_t header;
         __packed struct {
          uint8_t channel     : 2;
          uint8_t reserved    : 2;
          uint8_t port        : 4;
        };
      };
      uint8_t data[CRTP_MAX_DATA_SIZE];
    };
    uint8_t raw[CRTP_MAX_DATA_SIZE+2];
  };
}__packed CRTPPacket;

#ifdef __cplusplus
}
#endif
#endif
