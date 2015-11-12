//#ifndef ArduinoSerialForward_h
//
//#define ArduinoSerialForward_h
//#include <Arduino.h>
//#include "ArduinoSerialForward.c"
//
//void Serial1_setup(long bps);
//void Serial1_setListenerID(uint16_t id);
//void Serial1_forwardMessage(byte * msg, uint8_t size);
//void Serial1_read();

//#endif
//#ifndef ArduinoSerialForward_h
//
//#define ArduinoSerialForward_h
//#include <Arduino.h>
//#include "ArduinoSerialForward.c"
//
//void Serial3_setup(long bps);
//void Serial3_setListenerID(uint16_t id);
//void Serial3_forwardMessage(byte * msg, uint8_t size);
//void Serial3_read();

//#endif
//#ifndef ArduinoSerialForward_h
//
//#define ArduinoSerialForward_h
//#include <Arduino.h>
//#include "ArduinoSerialForward.c"
//
//void Serial_setup(long bps);
//void Serial_setListenerID(uint16_t id);
//void Serial_forwardMessage(byte * msg, uint8_t size);
//void Serial_read();

//#endif
/*****************************************************************************
 * Headers for type : Gateway
 *****************************************************************************/

// Definition of the instance stuct:
struct Gateway_Instance {
// Variables for the ID of the ports of the instance
uint16_t id_Serial0;
// Pointer to receiver list
struct Msg_Handler ** Serial0_receiver_list_head;
struct Msg_Handler ** Serial0_receiver_list_tail;
// Handler Array
struct Msg_Handler * Serial0_handlers;
uint16_t id_Serial1;
// Pointer to receiver list
struct Msg_Handler ** Serial1_receiver_list_head;
struct Msg_Handler ** Serial1_receiver_list_tail;
// Handler Array
struct Msg_Handler * Serial1_handlers;
uint16_t id_Serial2;
// Pointer to receiver list
struct Msg_Handler ** Serial2_receiver_list_head;
struct Msg_Handler ** Serial2_receiver_list_tail;
// Handler Array
struct Msg_Handler * Serial2_handlers;
uint16_t id_Serial3;
// Pointer to receiver list
struct Msg_Handler ** Serial3_receiver_list_head;
struct Msg_Handler ** Serial3_receiver_list_tail;
// Handler Array
struct Msg_Handler * Serial3_handlers;
// Variables for the current instance state
int Gateway_GatewayChart_State;
// Variables for the properties of the instance

};
// Declaration of prototypes outgoing messages:
void Gateway_GatewayChart_OnEntry(int state, struct Gateway_Instance *_instance);
void Gateway_handle_Serial1_mayIHaveID(struct Gateway_Instance *_instance, uint8_t id);
void Gateway_handle_Serial1_tronGo(struct Gateway_Instance *_instance, uint8_t nbID);
void Gateway_handle_Serial1_loose(struct Gateway_Instance *_instance, uint8_t id);
void Gateway_handle_Serial1_hasID(struct Gateway_Instance *_instance, uint8_t id);
void Gateway_handle_Serial1_iHaveID(struct Gateway_Instance *_instance, uint8_t id);
void Gateway_handle_Serial1_tronReady(struct Gateway_Instance *_instance, uint8_t id);
void Gateway_handle_Serial1_addHead(struct Gateway_Instance *_instance, uint8_t x, uint8_t y, uint8_t id);
void Gateway_handle_Serial3_mayIHaveID(struct Gateway_Instance *_instance, uint8_t id);
void Gateway_handle_Serial3_tronGo(struct Gateway_Instance *_instance, uint8_t nbID);
void Gateway_handle_Serial3_loose(struct Gateway_Instance *_instance, uint8_t id);
void Gateway_handle_Serial3_hasID(struct Gateway_Instance *_instance, uint8_t id);
void Gateway_handle_Serial3_iHaveID(struct Gateway_Instance *_instance, uint8_t id);
void Gateway_handle_Serial3_tronReady(struct Gateway_Instance *_instance, uint8_t id);
void Gateway_handle_Serial3_addHead(struct Gateway_Instance *_instance, uint8_t x, uint8_t y, uint8_t id);
void Gateway_handle_Serial2_mayIHaveID(struct Gateway_Instance *_instance, uint8_t id);
void Gateway_handle_Serial2_tronGo(struct Gateway_Instance *_instance, uint8_t nbID);
void Gateway_handle_Serial2_loose(struct Gateway_Instance *_instance, uint8_t id);
void Gateway_handle_Serial2_hasID(struct Gateway_Instance *_instance, uint8_t id);
void Gateway_handle_Serial2_iHaveID(struct Gateway_Instance *_instance, uint8_t id);
void Gateway_handle_Serial2_tronReady(struct Gateway_Instance *_instance, uint8_t id);
void Gateway_handle_Serial2_addHead(struct Gateway_Instance *_instance, uint8_t x, uint8_t y, uint8_t id);
void Gateway_handle_Serial0_mayIHaveID(struct Gateway_Instance *_instance, uint8_t id);
void Gateway_handle_Serial0_tronGo(struct Gateway_Instance *_instance, uint8_t nbID);
void Gateway_handle_Serial0_loose(struct Gateway_Instance *_instance, uint8_t id);
void Gateway_handle_Serial0_hasID(struct Gateway_Instance *_instance, uint8_t id);
void Gateway_handle_Serial0_iHaveID(struct Gateway_Instance *_instance, uint8_t id);
void Gateway_handle_Serial0_tronReady(struct Gateway_Instance *_instance, uint8_t id);
void Gateway_handle_Serial0_addHead(struct Gateway_Instance *_instance, uint8_t x, uint8_t y, uint8_t id);
// Declaration of callbacks for incoming messages:
void register_Gateway_send_Serial0_addHead_listener(void (*_listener)(struct Gateway_Instance *, uint8_t, uint8_t, uint8_t));
void register_external_Gateway_send_Serial0_addHead_listener(void (*_listener)(struct Gateway_Instance *, uint8_t, uint8_t, uint8_t));
void register_Gateway_send_Serial0_loose_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_external_Gateway_send_Serial0_loose_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_Gateway_send_Serial0_tronReady_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_external_Gateway_send_Serial0_tronReady_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_Gateway_send_Serial0_tronGo_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_external_Gateway_send_Serial0_tronGo_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_Gateway_send_Serial0_hasID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_external_Gateway_send_Serial0_hasID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_Gateway_send_Serial0_iHaveID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_external_Gateway_send_Serial0_iHaveID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_Gateway_send_Serial0_mayIHaveID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_external_Gateway_send_Serial0_mayIHaveID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_Gateway_send_Serial1_addHead_listener(void (*_listener)(struct Gateway_Instance *, uint8_t, uint8_t, uint8_t));
void register_external_Gateway_send_Serial1_addHead_listener(void (*_listener)(struct Gateway_Instance *, uint8_t, uint8_t, uint8_t));
void register_Gateway_send_Serial1_loose_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_external_Gateway_send_Serial1_loose_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_Gateway_send_Serial1_tronReady_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_external_Gateway_send_Serial1_tronReady_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_Gateway_send_Serial1_tronGo_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_external_Gateway_send_Serial1_tronGo_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_Gateway_send_Serial1_hasID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_external_Gateway_send_Serial1_hasID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_Gateway_send_Serial1_iHaveID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_external_Gateway_send_Serial1_iHaveID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_Gateway_send_Serial1_mayIHaveID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_external_Gateway_send_Serial1_mayIHaveID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_Gateway_send_Serial2_addHead_listener(void (*_listener)(struct Gateway_Instance *, uint8_t, uint8_t, uint8_t));
void register_external_Gateway_send_Serial2_addHead_listener(void (*_listener)(struct Gateway_Instance *, uint8_t, uint8_t, uint8_t));
void register_Gateway_send_Serial2_loose_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_external_Gateway_send_Serial2_loose_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_Gateway_send_Serial2_tronReady_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_external_Gateway_send_Serial2_tronReady_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_Gateway_send_Serial2_tronGo_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_external_Gateway_send_Serial2_tronGo_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_Gateway_send_Serial2_hasID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_external_Gateway_send_Serial2_hasID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_Gateway_send_Serial2_iHaveID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_external_Gateway_send_Serial2_iHaveID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_Gateway_send_Serial2_mayIHaveID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_external_Gateway_send_Serial2_mayIHaveID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_Gateway_send_Serial3_addHead_listener(void (*_listener)(struct Gateway_Instance *, uint8_t, uint8_t, uint8_t));
void register_external_Gateway_send_Serial3_addHead_listener(void (*_listener)(struct Gateway_Instance *, uint8_t, uint8_t, uint8_t));
void register_Gateway_send_Serial3_loose_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_external_Gateway_send_Serial3_loose_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_Gateway_send_Serial3_tronReady_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_external_Gateway_send_Serial3_tronReady_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_Gateway_send_Serial3_tronGo_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_external_Gateway_send_Serial3_tronGo_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_Gateway_send_Serial3_hasID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_external_Gateway_send_Serial3_hasID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_Gateway_send_Serial3_iHaveID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_external_Gateway_send_Serial3_iHaveID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_Gateway_send_Serial3_mayIHaveID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));
void register_external_Gateway_send_Serial3_mayIHaveID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t));

// Definition of the states:
#define GATEWAY_GATEWAYCHART_STATE 0
#define GATEWAY_GATEWAYCHART_ACTIVE_STATE 1


/* Adds and instance to the runtime and returns its id */
uint16_t add_instance(void * instance_struct);
/* Returns the instance with id */
void * instance_by_id(uint16_t id);

/* Returns the number of byte currently in the fifo */
int fifo_byte_length();
/* Returns the number of bytes currently available in the fifo */
int fifo_byte_available();
/* Returns true if the fifo is empty */
int fifo_empty();
/* Return true if the fifo is full */
int fifo_full();
/* Enqueue 1 byte in the fifo if there is space
   returns 1 for sucess and 0 if the fifo was full */
int fifo_enqueue(byte b);
/* Enqueue 1 byte in the fifo without checking for available space
   The caller should have checked that there is enough space */
int _fifo_enqueue(byte b);
/* Dequeue 1 byte in the fifo.
   The caller should check that the fifo is not empty */
byte fifo_dequeue();
//#ifndef ArduinoSerialForward_h
//
//#define ArduinoSerialForward_h
//#include <Arduino.h>
//#include "ArduinoSerialForward.c"
//
//void Serial2_setup(long bps);
//void Serial2_setListenerID(uint16_t id);
//void Serial2_forwardMessage(byte * msg, uint8_t size);
//void Serial2_read();

//#endif
//Port message handler structure
typedef struct Msg_Handler {
	int nb_msg;
	uint16_t * msg;
	void ** msg_handler;
	void * instance;
};



/*SOFTWARE_SERIAL*/

#define Serial1_LISTENER_STATE_IDLE 0
#define Serial1_LISTENER_STATE_READING 1
#define Serial1_LISTENER_STATE_ESCAPE 2
#define Serial1_LISTENER_STATE_ERROR 3


#define Serial1_START_BYTE 18
#define Serial1_STOP_BYTE 19
#define Serial1_ESCAPE_BYTE 125

#define Serial1_LIMIT_BYTE_PER_LOOP 10
#define Serial1_MAX_MSG_SIZE 5
#define Serial1_MSG_BUFFER_SIZE 10


byte Serial1_serialBuffer[Serial1_MSG_BUFFER_SIZE];
uint8_t Serial1_serialMsgSize = 0;
byte Serial1_incoming = 0;
uint8_t Serial1_serialListenerState = Serial1_LISTENER_STATE_IDLE;


struct Serial1_instance_type {
    uint16_t listener_id;
    //Connector// Pointer to receiver list
struct Msg_Handler ** Serial1_receiver_list_head;
struct Msg_Handler ** Serial1_receiver_list_tail;
// Handler Array
struct Msg_Handler * Serial1_handlers;

} Serial1_instance;

int fifo_byte_available();
int _fifo_enqueue(byte b);

void Serial1_setup() {
	Serial1.begin(115200);
}

void Serial1_set_listener_id(uint16_t id) {
	Serial1_instance.listener_id = id;
}


void Serial1_forwardMessage(byte * msg, uint8_t size) {
  
  Serial1.write(Serial1_START_BYTE);
  for(uint8_t i = 0; i < size; i++) {
    if((msg[i] == Serial1_START_BYTE) 
		|| (msg[i] == Serial1_STOP_BYTE) 
		|| (msg[i] == Serial1_ESCAPE_BYTE)) {
      Serial1.write(Serial1_ESCAPE_BYTE);
    }
    Serial1.write(msg[i]);
  }
  Serial1.write(Serial1_STOP_BYTE);
}

void Serial1_read() {
  byte limit = 0;
  while ((Serial1.available()) && (limit < Serial1_LIMIT_BYTE_PER_LOOP)) {
   limit++;
    Serial1_incoming = Serial1.read();
    
    switch(Serial1_serialListenerState) {
      case Serial1_LISTENER_STATE_IDLE:
        if(Serial1_incoming == Serial1_START_BYTE) {
          Serial1_serialListenerState = Serial1_LISTENER_STATE_READING;
          Serial1_serialMsgSize = 0;
        }
      break;
      
      case Serial1_LISTENER_STATE_READING:
        if (Serial1_serialMsgSize > Serial1_MAX_MSG_SIZE) {
          Serial1_serialListenerState = Serial1_LISTENER_STATE_ERROR;
        } else {
          if(Serial1_incoming == Serial1_STOP_BYTE) {
            Serial1_serialListenerState = Serial1_LISTENER_STATE_IDLE;
            externalMessageEnqueue(Serial1_serialBuffer, Serial1_serialMsgSize, Serial1_instance.listener_id);
            
          } else if (Serial1_incoming == Serial1_ESCAPE_BYTE) {
            Serial1_serialListenerState = Serial1_LISTENER_STATE_ESCAPE;
          } else {
            Serial1_serialBuffer[Serial1_serialMsgSize] = Serial1_incoming;
            Serial1_serialMsgSize++;
          }
        }
      break;
      
      case Serial1_LISTENER_STATE_ESCAPE:
        if (Serial1_serialMsgSize >= Serial1_MAX_MSG_SIZE) {
          Serial1_serialListenerState = Serial1_LISTENER_STATE_ERROR;
        } else {
          Serial1_serialBuffer[Serial1_serialMsgSize] = Serial1_incoming;
          Serial1_serialMsgSize++;
          Serial1_serialListenerState = Serial1_LISTENER_STATE_READING;
        }
      break;
      
      case Serial1_LISTENER_STATE_ERROR:
        Serial1_serialListenerState = Serial1_LISTENER_STATE_IDLE;
        Serial1_serialMsgSize = 0;
      break;
    }
  }
  
}

/*SOFTWARE_SERIAL*/

#define Serial_LISTENER_STATE_IDLE 0
#define Serial_LISTENER_STATE_READING 1
#define Serial_LISTENER_STATE_ESCAPE 2
#define Serial_LISTENER_STATE_ERROR 3


#define Serial_START_BYTE 18
#define Serial_STOP_BYTE 19
#define Serial_ESCAPE_BYTE 125

#define Serial_LIMIT_BYTE_PER_LOOP 10
#define Serial_MAX_MSG_SIZE 5
#define Serial_MSG_BUFFER_SIZE 10


byte Serial_serialBuffer[Serial_MSG_BUFFER_SIZE];
uint8_t Serial_serialMsgSize = 0;
byte Serial_incoming = 0;
uint8_t Serial_serialListenerState = Serial_LISTENER_STATE_IDLE;


struct Serial_instance_type {
    uint16_t listener_id;
    //Connector// Pointer to receiver list
struct Msg_Handler ** Serial0_receiver_list_head;
struct Msg_Handler ** Serial0_receiver_list_tail;
// Handler Array
struct Msg_Handler * Serial0_handlers;

} Serial_instance;

int fifo_byte_available();
int _fifo_enqueue(byte b);

void Serial_setup() {
	Serial.begin(115200);
}

void Serial_set_listener_id(uint16_t id) {
	Serial_instance.listener_id = id;
}


void Serial_forwardMessage(byte * msg, uint8_t size) {
  
  Serial.write(Serial_START_BYTE);
  for(uint8_t i = 0; i < size; i++) {
    if((msg[i] == Serial_START_BYTE) 
		|| (msg[i] == Serial_STOP_BYTE) 
		|| (msg[i] == Serial_ESCAPE_BYTE)) {
      Serial.write(Serial_ESCAPE_BYTE);
    }
    Serial.write(msg[i]);
  }
  Serial.write(Serial_STOP_BYTE);
}

void Serial_read() {
  byte limit = 0;
  while ((Serial.available()) && (limit < Serial_LIMIT_BYTE_PER_LOOP)) {
   limit++;
    Serial_incoming = Serial.read();
    
    switch(Serial_serialListenerState) {
      case Serial_LISTENER_STATE_IDLE:
        if(Serial_incoming == Serial_START_BYTE) {
          Serial_serialListenerState = Serial_LISTENER_STATE_READING;
          Serial_serialMsgSize = 0;
        }
      break;
      
      case Serial_LISTENER_STATE_READING:
        if (Serial_serialMsgSize > Serial_MAX_MSG_SIZE) {
          Serial_serialListenerState = Serial_LISTENER_STATE_ERROR;
        } else {
          if(Serial_incoming == Serial_STOP_BYTE) {
            Serial_serialListenerState = Serial_LISTENER_STATE_IDLE;
            externalMessageEnqueue(Serial_serialBuffer, Serial_serialMsgSize, Serial_instance.listener_id);
            
          } else if (Serial_incoming == Serial_ESCAPE_BYTE) {
            Serial_serialListenerState = Serial_LISTENER_STATE_ESCAPE;
          } else {
            Serial_serialBuffer[Serial_serialMsgSize] = Serial_incoming;
            Serial_serialMsgSize++;
          }
        }
      break;
      
      case Serial_LISTENER_STATE_ESCAPE:
        if (Serial_serialMsgSize >= Serial_MAX_MSG_SIZE) {
          Serial_serialListenerState = Serial_LISTENER_STATE_ERROR;
        } else {
          Serial_serialBuffer[Serial_serialMsgSize] = Serial_incoming;
          Serial_serialMsgSize++;
          Serial_serialListenerState = Serial_LISTENER_STATE_READING;
        }
      break;
      
      case Serial_LISTENER_STATE_ERROR:
        Serial_serialListenerState = Serial_LISTENER_STATE_IDLE;
        Serial_serialMsgSize = 0;
      break;
    }
  }
  
}
/*****************************************************************************
 * Implementation for type : Gateway
 *****************************************************************************/

// Declaration of prototypes:
void Gateway_GatewayChart_OnExit(int state, struct Gateway_Instance *_instance);
void Gateway_send_Serial0_addHead(struct Gateway_Instance *_instance, uint8_t x, uint8_t y, uint8_t id);
void Gateway_send_Serial0_loose(struct Gateway_Instance *_instance, uint8_t id);
void Gateway_send_Serial0_tronReady(struct Gateway_Instance *_instance, uint8_t id);
void Gateway_send_Serial0_tronGo(struct Gateway_Instance *_instance, uint8_t nbID);
void Gateway_send_Serial0_hasID(struct Gateway_Instance *_instance, uint8_t id);
void Gateway_send_Serial0_iHaveID(struct Gateway_Instance *_instance, uint8_t id);
void Gateway_send_Serial0_mayIHaveID(struct Gateway_Instance *_instance, uint8_t id);
void Gateway_send_Serial1_addHead(struct Gateway_Instance *_instance, uint8_t x, uint8_t y, uint8_t id);
void Gateway_send_Serial1_loose(struct Gateway_Instance *_instance, uint8_t id);
void Gateway_send_Serial1_tronReady(struct Gateway_Instance *_instance, uint8_t id);
void Gateway_send_Serial1_tronGo(struct Gateway_Instance *_instance, uint8_t nbID);
void Gateway_send_Serial1_hasID(struct Gateway_Instance *_instance, uint8_t id);
void Gateway_send_Serial1_iHaveID(struct Gateway_Instance *_instance, uint8_t id);
void Gateway_send_Serial1_mayIHaveID(struct Gateway_Instance *_instance, uint8_t id);
void Gateway_send_Serial2_addHead(struct Gateway_Instance *_instance, uint8_t x, uint8_t y, uint8_t id);
void Gateway_send_Serial2_loose(struct Gateway_Instance *_instance, uint8_t id);
void Gateway_send_Serial2_tronReady(struct Gateway_Instance *_instance, uint8_t id);
void Gateway_send_Serial2_tronGo(struct Gateway_Instance *_instance, uint8_t nbID);
void Gateway_send_Serial2_hasID(struct Gateway_Instance *_instance, uint8_t id);
void Gateway_send_Serial2_iHaveID(struct Gateway_Instance *_instance, uint8_t id);
void Gateway_send_Serial2_mayIHaveID(struct Gateway_Instance *_instance, uint8_t id);
void Gateway_send_Serial3_addHead(struct Gateway_Instance *_instance, uint8_t x, uint8_t y, uint8_t id);
void Gateway_send_Serial3_loose(struct Gateway_Instance *_instance, uint8_t id);
void Gateway_send_Serial3_tronReady(struct Gateway_Instance *_instance, uint8_t id);
void Gateway_send_Serial3_tronGo(struct Gateway_Instance *_instance, uint8_t nbID);
void Gateway_send_Serial3_hasID(struct Gateway_Instance *_instance, uint8_t id);
void Gateway_send_Serial3_iHaveID(struct Gateway_Instance *_instance, uint8_t id);
void Gateway_send_Serial3_mayIHaveID(struct Gateway_Instance *_instance, uint8_t id);
// Declaration of functions:

// On Entry Actions:
void Gateway_GatewayChart_OnEntry(int state, struct Gateway_Instance *_instance) {
switch(state) {
case GATEWAY_GATEWAYCHART_STATE:
_instance->Gateway_GatewayChart_State = GATEWAY_GATEWAYCHART_ACTIVE_STATE;
Gateway_GatewayChart_OnEntry(_instance->Gateway_GatewayChart_State, _instance);
break;
case GATEWAY_GATEWAYCHART_ACTIVE_STATE:
break;
default: break;
}
}

// On Exit Actions:
void Gateway_GatewayChart_OnExit(int state, struct Gateway_Instance *_instance) {
switch(state) {
case GATEWAY_GATEWAYCHART_STATE:
Gateway_GatewayChart_OnExit(_instance->Gateway_GatewayChart_State, _instance);
break;
case GATEWAY_GATEWAYCHART_ACTIVE_STATE:
break;
default: break;
}
}

// Event Handlers for incoming messages:
void Gateway_handle_Serial1_mayIHaveID(struct Gateway_Instance *_instance, uint8_t id) {
uint8_t Gateway_GatewayChart_State_event_consumed = 0;
if (_instance->Gateway_GatewayChart_State == GATEWAY_GATEWAYCHART_ACTIVE_STATE) {
if (Gateway_GatewayChart_State_event_consumed == 0 && 1) {
Gateway_send_Serial0_mayIHaveID(_instance, id);
Gateway_send_Serial2_mayIHaveID(_instance, id);
Gateway_send_Serial3_mayIHaveID(_instance, id);
Gateway_GatewayChart_State_event_consumed = 1;
}
}
}
void Gateway_handle_Serial1_tronGo(struct Gateway_Instance *_instance, uint8_t nbID) {
uint8_t Gateway_GatewayChart_State_event_consumed = 0;
if (_instance->Gateway_GatewayChart_State == GATEWAY_GATEWAYCHART_ACTIVE_STATE) {
if (Gateway_GatewayChart_State_event_consumed == 0 && 1) {
Gateway_send_Serial0_tronGo(_instance, nbID);
Gateway_send_Serial2_tronGo(_instance, nbID);
Gateway_send_Serial3_tronGo(_instance, nbID);
Gateway_GatewayChart_State_event_consumed = 1;
}
}
}
void Gateway_handle_Serial1_loose(struct Gateway_Instance *_instance, uint8_t id) {
uint8_t Gateway_GatewayChart_State_event_consumed = 0;
if (_instance->Gateway_GatewayChart_State == GATEWAY_GATEWAYCHART_ACTIVE_STATE) {
if (Gateway_GatewayChart_State_event_consumed == 0 && 1) {
Gateway_send_Serial0_loose(_instance, id);
Gateway_send_Serial2_loose(_instance, id);
Gateway_send_Serial3_loose(_instance, id);
Gateway_GatewayChart_State_event_consumed = 1;
}
}
}
void Gateway_handle_Serial1_hasID(struct Gateway_Instance *_instance, uint8_t id) {
uint8_t Gateway_GatewayChart_State_event_consumed = 0;
if (_instance->Gateway_GatewayChart_State == GATEWAY_GATEWAYCHART_ACTIVE_STATE) {
if (Gateway_GatewayChart_State_event_consumed == 0 && 1) {
Gateway_send_Serial0_hasID(_instance, id);
Gateway_send_Serial2_hasID(_instance, id);
Gateway_send_Serial3_hasID(_instance, id);
Gateway_GatewayChart_State_event_consumed = 1;
}
}
}
void Gateway_handle_Serial1_iHaveID(struct Gateway_Instance *_instance, uint8_t id) {
uint8_t Gateway_GatewayChart_State_event_consumed = 0;
if (_instance->Gateway_GatewayChart_State == GATEWAY_GATEWAYCHART_ACTIVE_STATE) {
if (Gateway_GatewayChart_State_event_consumed == 0 && 1) {
Gateway_send_Serial0_iHaveID(_instance, id);
Gateway_send_Serial2_iHaveID(_instance, id);
Gateway_send_Serial3_iHaveID(_instance, id);
Gateway_GatewayChart_State_event_consumed = 1;
}
}
}
void Gateway_handle_Serial1_tronReady(struct Gateway_Instance *_instance, uint8_t id) {
uint8_t Gateway_GatewayChart_State_event_consumed = 0;
if (_instance->Gateway_GatewayChart_State == GATEWAY_GATEWAYCHART_ACTIVE_STATE) {
if (Gateway_GatewayChart_State_event_consumed == 0 && 1) {
Gateway_send_Serial0_tronReady(_instance, id);
Gateway_send_Serial2_tronReady(_instance, id);
Gateway_send_Serial3_tronReady(_instance, id);
Gateway_GatewayChart_State_event_consumed = 1;
}
}
}
void Gateway_handle_Serial1_addHead(struct Gateway_Instance *_instance, uint8_t x, uint8_t y, uint8_t id) {
uint8_t Gateway_GatewayChart_State_event_consumed = 0;
if (_instance->Gateway_GatewayChart_State == GATEWAY_GATEWAYCHART_ACTIVE_STATE) {
if (Gateway_GatewayChart_State_event_consumed == 0 && 1) {
Gateway_send_Serial0_addHead(_instance, x, y, id);
Gateway_send_Serial2_addHead(_instance, x, y, id);
Gateway_send_Serial3_addHead(_instance, x, y, id);
Gateway_GatewayChart_State_event_consumed = 1;
}
}
}
void Gateway_handle_Serial3_mayIHaveID(struct Gateway_Instance *_instance, uint8_t id) {
uint8_t Gateway_GatewayChart_State_event_consumed = 0;
if (_instance->Gateway_GatewayChart_State == GATEWAY_GATEWAYCHART_ACTIVE_STATE) {
if (Gateway_GatewayChart_State_event_consumed == 0 && 1) {
Gateway_send_Serial1_mayIHaveID(_instance, id);
Gateway_send_Serial0_mayIHaveID(_instance, id);
Gateway_send_Serial2_mayIHaveID(_instance, id);
Gateway_GatewayChart_State_event_consumed = 1;
}
}
}
void Gateway_handle_Serial3_tronGo(struct Gateway_Instance *_instance, uint8_t nbID) {
uint8_t Gateway_GatewayChart_State_event_consumed = 0;
if (_instance->Gateway_GatewayChart_State == GATEWAY_GATEWAYCHART_ACTIVE_STATE) {
if (Gateway_GatewayChart_State_event_consumed == 0 && 1) {
Gateway_send_Serial1_tronGo(_instance, nbID);
Gateway_send_Serial0_tronGo(_instance, nbID);
Gateway_send_Serial2_tronGo(_instance, nbID);
Gateway_GatewayChart_State_event_consumed = 1;
}
}
}
void Gateway_handle_Serial3_loose(struct Gateway_Instance *_instance, uint8_t id) {
uint8_t Gateway_GatewayChart_State_event_consumed = 0;
if (_instance->Gateway_GatewayChart_State == GATEWAY_GATEWAYCHART_ACTIVE_STATE) {
if (Gateway_GatewayChart_State_event_consumed == 0 && 1) {
Gateway_send_Serial1_loose(_instance, id);
Gateway_send_Serial0_loose(_instance, id);
Gateway_send_Serial2_loose(_instance, id);
Gateway_GatewayChart_State_event_consumed = 1;
}
}
}
void Gateway_handle_Serial3_hasID(struct Gateway_Instance *_instance, uint8_t id) {
uint8_t Gateway_GatewayChart_State_event_consumed = 0;
if (_instance->Gateway_GatewayChart_State == GATEWAY_GATEWAYCHART_ACTIVE_STATE) {
if (Gateway_GatewayChart_State_event_consumed == 0 && 1) {
Gateway_send_Serial1_hasID(_instance, id);
Gateway_send_Serial0_hasID(_instance, id);
Gateway_send_Serial2_hasID(_instance, id);
Gateway_GatewayChart_State_event_consumed = 1;
}
}
}
void Gateway_handle_Serial3_iHaveID(struct Gateway_Instance *_instance, uint8_t id) {
uint8_t Gateway_GatewayChart_State_event_consumed = 0;
if (_instance->Gateway_GatewayChart_State == GATEWAY_GATEWAYCHART_ACTIVE_STATE) {
if (Gateway_GatewayChart_State_event_consumed == 0 && 1) {
Gateway_send_Serial1_iHaveID(_instance, id);
Gateway_send_Serial0_iHaveID(_instance, id);
Gateway_send_Serial2_iHaveID(_instance, id);
Gateway_GatewayChart_State_event_consumed = 1;
}
}
}
void Gateway_handle_Serial3_tronReady(struct Gateway_Instance *_instance, uint8_t id) {
uint8_t Gateway_GatewayChart_State_event_consumed = 0;
if (_instance->Gateway_GatewayChart_State == GATEWAY_GATEWAYCHART_ACTIVE_STATE) {
if (Gateway_GatewayChart_State_event_consumed == 0 && 1) {
Gateway_send_Serial1_tronReady(_instance, id);
Gateway_send_Serial0_tronReady(_instance, id);
Gateway_send_Serial2_tronReady(_instance, id);
Gateway_GatewayChart_State_event_consumed = 1;
}
}
}
void Gateway_handle_Serial3_addHead(struct Gateway_Instance *_instance, uint8_t x, uint8_t y, uint8_t id) {
uint8_t Gateway_GatewayChart_State_event_consumed = 0;
if (_instance->Gateway_GatewayChart_State == GATEWAY_GATEWAYCHART_ACTIVE_STATE) {
if (Gateway_GatewayChart_State_event_consumed == 0 && 1) {
Gateway_send_Serial1_addHead(_instance, x, y, id);
Gateway_send_Serial0_addHead(_instance, x, y, id);
Gateway_send_Serial2_addHead(_instance, x, y, id);
Gateway_GatewayChart_State_event_consumed = 1;
}
}
}
void Gateway_handle_Serial2_mayIHaveID(struct Gateway_Instance *_instance, uint8_t id) {
uint8_t Gateway_GatewayChart_State_event_consumed = 0;
if (_instance->Gateway_GatewayChart_State == GATEWAY_GATEWAYCHART_ACTIVE_STATE) {
if (Gateway_GatewayChart_State_event_consumed == 0 && 1) {
Gateway_send_Serial1_mayIHaveID(_instance, id);
Gateway_send_Serial0_mayIHaveID(_instance, id);
Gateway_send_Serial3_mayIHaveID(_instance, id);
Gateway_GatewayChart_State_event_consumed = 1;
}
}
}
void Gateway_handle_Serial2_tronGo(struct Gateway_Instance *_instance, uint8_t nbID) {
uint8_t Gateway_GatewayChart_State_event_consumed = 0;
if (_instance->Gateway_GatewayChart_State == GATEWAY_GATEWAYCHART_ACTIVE_STATE) {
if (Gateway_GatewayChart_State_event_consumed == 0 && 1) {
Gateway_send_Serial1_tronGo(_instance, nbID);
Gateway_send_Serial0_tronGo(_instance, nbID);
Gateway_send_Serial3_tronGo(_instance, nbID);
Gateway_GatewayChart_State_event_consumed = 1;
}
}
}
void Gateway_handle_Serial2_loose(struct Gateway_Instance *_instance, uint8_t id) {
uint8_t Gateway_GatewayChart_State_event_consumed = 0;
if (_instance->Gateway_GatewayChart_State == GATEWAY_GATEWAYCHART_ACTIVE_STATE) {
if (Gateway_GatewayChart_State_event_consumed == 0 && 1) {
Gateway_send_Serial1_loose(_instance, id);
Gateway_send_Serial0_loose(_instance, id);
Gateway_send_Serial3_loose(_instance, id);
Gateway_GatewayChart_State_event_consumed = 1;
}
}
}
void Gateway_handle_Serial2_hasID(struct Gateway_Instance *_instance, uint8_t id) {
uint8_t Gateway_GatewayChart_State_event_consumed = 0;
if (_instance->Gateway_GatewayChart_State == GATEWAY_GATEWAYCHART_ACTIVE_STATE) {
if (Gateway_GatewayChart_State_event_consumed == 0 && 1) {
Gateway_send_Serial1_hasID(_instance, id);
Gateway_send_Serial0_hasID(_instance, id);
Gateway_send_Serial3_hasID(_instance, id);
Gateway_GatewayChart_State_event_consumed = 1;
}
}
}
void Gateway_handle_Serial2_iHaveID(struct Gateway_Instance *_instance, uint8_t id) {
uint8_t Gateway_GatewayChart_State_event_consumed = 0;
if (_instance->Gateway_GatewayChart_State == GATEWAY_GATEWAYCHART_ACTIVE_STATE) {
if (Gateway_GatewayChart_State_event_consumed == 0 && 1) {
Gateway_send_Serial1_iHaveID(_instance, id);
Gateway_send_Serial0_iHaveID(_instance, id);
Gateway_send_Serial3_iHaveID(_instance, id);
Gateway_GatewayChart_State_event_consumed = 1;
}
}
}
void Gateway_handle_Serial2_tronReady(struct Gateway_Instance *_instance, uint8_t id) {
uint8_t Gateway_GatewayChart_State_event_consumed = 0;
if (_instance->Gateway_GatewayChart_State == GATEWAY_GATEWAYCHART_ACTIVE_STATE) {
if (Gateway_GatewayChart_State_event_consumed == 0 && 1) {
Gateway_send_Serial1_tronReady(_instance, id);
Gateway_send_Serial0_tronReady(_instance, id);
Gateway_send_Serial3_tronReady(_instance, id);
Gateway_GatewayChart_State_event_consumed = 1;
}
}
}
void Gateway_handle_Serial2_addHead(struct Gateway_Instance *_instance, uint8_t x, uint8_t y, uint8_t id) {
uint8_t Gateway_GatewayChart_State_event_consumed = 0;
if (_instance->Gateway_GatewayChart_State == GATEWAY_GATEWAYCHART_ACTIVE_STATE) {
if (Gateway_GatewayChart_State_event_consumed == 0 && 1) {
Gateway_send_Serial1_addHead(_instance, x, y, id);
Gateway_send_Serial0_addHead(_instance, x, y, id);
Gateway_send_Serial3_addHead(_instance, x, y, id);
Gateway_GatewayChart_State_event_consumed = 1;
}
}
}
void Gateway_handle_Serial0_mayIHaveID(struct Gateway_Instance *_instance, uint8_t id) {
uint8_t Gateway_GatewayChart_State_event_consumed = 0;
if (_instance->Gateway_GatewayChart_State == GATEWAY_GATEWAYCHART_ACTIVE_STATE) {
if (Gateway_GatewayChart_State_event_consumed == 0 && 1) {
Gateway_send_Serial1_mayIHaveID(_instance, id);
Gateway_send_Serial2_mayIHaveID(_instance, id);
Gateway_send_Serial3_mayIHaveID(_instance, id);
Gateway_GatewayChart_State_event_consumed = 1;
}
}
}
void Gateway_handle_Serial0_tronGo(struct Gateway_Instance *_instance, uint8_t nbID) {
uint8_t Gateway_GatewayChart_State_event_consumed = 0;
if (_instance->Gateway_GatewayChart_State == GATEWAY_GATEWAYCHART_ACTIVE_STATE) {
if (Gateway_GatewayChart_State_event_consumed == 0 && 1) {
Gateway_send_Serial1_tronGo(_instance, nbID);
Gateway_send_Serial2_tronGo(_instance, nbID);
Gateway_send_Serial3_tronGo(_instance, nbID);
Gateway_GatewayChart_State_event_consumed = 1;
}
}
}
void Gateway_handle_Serial0_loose(struct Gateway_Instance *_instance, uint8_t id) {
uint8_t Gateway_GatewayChart_State_event_consumed = 0;
if (_instance->Gateway_GatewayChart_State == GATEWAY_GATEWAYCHART_ACTIVE_STATE) {
if (Gateway_GatewayChart_State_event_consumed == 0 && 1) {
Gateway_send_Serial1_loose(_instance, id);
Gateway_send_Serial2_loose(_instance, id);
Gateway_send_Serial3_loose(_instance, id);
Gateway_GatewayChart_State_event_consumed = 1;
}
}
}
void Gateway_handle_Serial0_hasID(struct Gateway_Instance *_instance, uint8_t id) {
uint8_t Gateway_GatewayChart_State_event_consumed = 0;
if (_instance->Gateway_GatewayChart_State == GATEWAY_GATEWAYCHART_ACTIVE_STATE) {
if (Gateway_GatewayChart_State_event_consumed == 0 && 1) {
Gateway_send_Serial1_hasID(_instance, id);
Gateway_send_Serial2_hasID(_instance, id);
Gateway_send_Serial3_hasID(_instance, id);
Gateway_GatewayChart_State_event_consumed = 1;
}
}
}
void Gateway_handle_Serial0_iHaveID(struct Gateway_Instance *_instance, uint8_t id) {
uint8_t Gateway_GatewayChart_State_event_consumed = 0;
if (_instance->Gateway_GatewayChart_State == GATEWAY_GATEWAYCHART_ACTIVE_STATE) {
if (Gateway_GatewayChart_State_event_consumed == 0 && 1) {
Gateway_send_Serial1_iHaveID(_instance, id);
Gateway_send_Serial2_iHaveID(_instance, id);
Gateway_send_Serial3_iHaveID(_instance, id);
Gateway_GatewayChart_State_event_consumed = 1;
}
}
}
void Gateway_handle_Serial0_tronReady(struct Gateway_Instance *_instance, uint8_t id) {
uint8_t Gateway_GatewayChart_State_event_consumed = 0;
if (_instance->Gateway_GatewayChart_State == GATEWAY_GATEWAYCHART_ACTIVE_STATE) {
if (Gateway_GatewayChart_State_event_consumed == 0 && 1) {
Gateway_send_Serial1_tronReady(_instance, id);
Gateway_send_Serial2_tronReady(_instance, id);
Gateway_send_Serial3_tronReady(_instance, id);
Gateway_GatewayChart_State_event_consumed = 1;
}
}
}
void Gateway_handle_Serial0_addHead(struct Gateway_Instance *_instance, uint8_t x, uint8_t y, uint8_t id) {
uint8_t Gateway_GatewayChart_State_event_consumed = 0;
if (_instance->Gateway_GatewayChart_State == GATEWAY_GATEWAYCHART_ACTIVE_STATE) {
if (Gateway_GatewayChart_State_event_consumed == 0 && 1) {
Gateway_send_Serial1_addHead(_instance, x, y, id);
Gateway_send_Serial2_addHead(_instance, x, y, id);
Gateway_send_Serial3_addHead(_instance, x, y, id);
Gateway_GatewayChart_State_event_consumed = 1;
}
}
}

// Observers for outgoing messages:
void (*external_Gateway_send_Serial0_addHead_listener)(struct Gateway_Instance *, uint8_t, uint8_t, uint8_t)= 0x0;
void register_external_Gateway_send_Serial0_addHead_listener(void (*_listener)(struct Gateway_Instance *, uint8_t, uint8_t, uint8_t)){
external_Gateway_send_Serial0_addHead_listener = _listener;
}
void (*Gateway_send_Serial0_addHead_listener)(struct Gateway_Instance *, uint8_t, uint8_t, uint8_t)= 0x0;
void register_Gateway_send_Serial0_addHead_listener(void (*_listener)(struct Gateway_Instance *, uint8_t, uint8_t, uint8_t)){
Gateway_send_Serial0_addHead_listener = _listener;
}
void Gateway_send_Serial0_addHead(struct Gateway_Instance *_instance, uint8_t x, uint8_t y, uint8_t id){
if (Gateway_send_Serial0_addHead_listener != 0x0) Gateway_send_Serial0_addHead_listener(_instance, x, y, id);
if (external_Gateway_send_Serial0_addHead_listener != 0x0) external_Gateway_send_Serial0_addHead_listener(_instance, x, y, id);
;
}
void (*external_Gateway_send_Serial0_loose_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_external_Gateway_send_Serial0_loose_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
external_Gateway_send_Serial0_loose_listener = _listener;
}
void (*Gateway_send_Serial0_loose_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_Gateway_send_Serial0_loose_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
Gateway_send_Serial0_loose_listener = _listener;
}
void Gateway_send_Serial0_loose(struct Gateway_Instance *_instance, uint8_t id){
if (Gateway_send_Serial0_loose_listener != 0x0) Gateway_send_Serial0_loose_listener(_instance, id);
if (external_Gateway_send_Serial0_loose_listener != 0x0) external_Gateway_send_Serial0_loose_listener(_instance, id);
;
}
void (*external_Gateway_send_Serial0_tronReady_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_external_Gateway_send_Serial0_tronReady_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
external_Gateway_send_Serial0_tronReady_listener = _listener;
}
void (*Gateway_send_Serial0_tronReady_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_Gateway_send_Serial0_tronReady_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
Gateway_send_Serial0_tronReady_listener = _listener;
}
void Gateway_send_Serial0_tronReady(struct Gateway_Instance *_instance, uint8_t id){
if (Gateway_send_Serial0_tronReady_listener != 0x0) Gateway_send_Serial0_tronReady_listener(_instance, id);
if (external_Gateway_send_Serial0_tronReady_listener != 0x0) external_Gateway_send_Serial0_tronReady_listener(_instance, id);
;
}
void (*external_Gateway_send_Serial0_tronGo_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_external_Gateway_send_Serial0_tronGo_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
external_Gateway_send_Serial0_tronGo_listener = _listener;
}
void (*Gateway_send_Serial0_tronGo_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_Gateway_send_Serial0_tronGo_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
Gateway_send_Serial0_tronGo_listener = _listener;
}
void Gateway_send_Serial0_tronGo(struct Gateway_Instance *_instance, uint8_t nbID){
if (Gateway_send_Serial0_tronGo_listener != 0x0) Gateway_send_Serial0_tronGo_listener(_instance, nbID);
if (external_Gateway_send_Serial0_tronGo_listener != 0x0) external_Gateway_send_Serial0_tronGo_listener(_instance, nbID);
;
}
void (*external_Gateway_send_Serial0_hasID_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_external_Gateway_send_Serial0_hasID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
external_Gateway_send_Serial0_hasID_listener = _listener;
}
void (*Gateway_send_Serial0_hasID_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_Gateway_send_Serial0_hasID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
Gateway_send_Serial0_hasID_listener = _listener;
}
void Gateway_send_Serial0_hasID(struct Gateway_Instance *_instance, uint8_t id){
if (Gateway_send_Serial0_hasID_listener != 0x0) Gateway_send_Serial0_hasID_listener(_instance, id);
if (external_Gateway_send_Serial0_hasID_listener != 0x0) external_Gateway_send_Serial0_hasID_listener(_instance, id);
;
}
void (*external_Gateway_send_Serial0_iHaveID_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_external_Gateway_send_Serial0_iHaveID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
external_Gateway_send_Serial0_iHaveID_listener = _listener;
}
void (*Gateway_send_Serial0_iHaveID_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_Gateway_send_Serial0_iHaveID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
Gateway_send_Serial0_iHaveID_listener = _listener;
}
void Gateway_send_Serial0_iHaveID(struct Gateway_Instance *_instance, uint8_t id){
if (Gateway_send_Serial0_iHaveID_listener != 0x0) Gateway_send_Serial0_iHaveID_listener(_instance, id);
if (external_Gateway_send_Serial0_iHaveID_listener != 0x0) external_Gateway_send_Serial0_iHaveID_listener(_instance, id);
;
}
void (*external_Gateway_send_Serial0_mayIHaveID_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_external_Gateway_send_Serial0_mayIHaveID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
external_Gateway_send_Serial0_mayIHaveID_listener = _listener;
}
void (*Gateway_send_Serial0_mayIHaveID_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_Gateway_send_Serial0_mayIHaveID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
Gateway_send_Serial0_mayIHaveID_listener = _listener;
}
void Gateway_send_Serial0_mayIHaveID(struct Gateway_Instance *_instance, uint8_t id){
if (Gateway_send_Serial0_mayIHaveID_listener != 0x0) Gateway_send_Serial0_mayIHaveID_listener(_instance, id);
if (external_Gateway_send_Serial0_mayIHaveID_listener != 0x0) external_Gateway_send_Serial0_mayIHaveID_listener(_instance, id);
;
}
void (*external_Gateway_send_Serial1_addHead_listener)(struct Gateway_Instance *, uint8_t, uint8_t, uint8_t)= 0x0;
void register_external_Gateway_send_Serial1_addHead_listener(void (*_listener)(struct Gateway_Instance *, uint8_t, uint8_t, uint8_t)){
external_Gateway_send_Serial1_addHead_listener = _listener;
}
void (*Gateway_send_Serial1_addHead_listener)(struct Gateway_Instance *, uint8_t, uint8_t, uint8_t)= 0x0;
void register_Gateway_send_Serial1_addHead_listener(void (*_listener)(struct Gateway_Instance *, uint8_t, uint8_t, uint8_t)){
Gateway_send_Serial1_addHead_listener = _listener;
}
void Gateway_send_Serial1_addHead(struct Gateway_Instance *_instance, uint8_t x, uint8_t y, uint8_t id){
if (Gateway_send_Serial1_addHead_listener != 0x0) Gateway_send_Serial1_addHead_listener(_instance, x, y, id);
if (external_Gateway_send_Serial1_addHead_listener != 0x0) external_Gateway_send_Serial1_addHead_listener(_instance, x, y, id);
;
}
void (*external_Gateway_send_Serial1_loose_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_external_Gateway_send_Serial1_loose_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
external_Gateway_send_Serial1_loose_listener = _listener;
}
void (*Gateway_send_Serial1_loose_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_Gateway_send_Serial1_loose_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
Gateway_send_Serial1_loose_listener = _listener;
}
void Gateway_send_Serial1_loose(struct Gateway_Instance *_instance, uint8_t id){
if (Gateway_send_Serial1_loose_listener != 0x0) Gateway_send_Serial1_loose_listener(_instance, id);
if (external_Gateway_send_Serial1_loose_listener != 0x0) external_Gateway_send_Serial1_loose_listener(_instance, id);
;
}
void (*external_Gateway_send_Serial1_tronReady_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_external_Gateway_send_Serial1_tronReady_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
external_Gateway_send_Serial1_tronReady_listener = _listener;
}
void (*Gateway_send_Serial1_tronReady_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_Gateway_send_Serial1_tronReady_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
Gateway_send_Serial1_tronReady_listener = _listener;
}
void Gateway_send_Serial1_tronReady(struct Gateway_Instance *_instance, uint8_t id){
if (Gateway_send_Serial1_tronReady_listener != 0x0) Gateway_send_Serial1_tronReady_listener(_instance, id);
if (external_Gateway_send_Serial1_tronReady_listener != 0x0) external_Gateway_send_Serial1_tronReady_listener(_instance, id);
;
}
void (*external_Gateway_send_Serial1_tronGo_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_external_Gateway_send_Serial1_tronGo_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
external_Gateway_send_Serial1_tronGo_listener = _listener;
}
void (*Gateway_send_Serial1_tronGo_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_Gateway_send_Serial1_tronGo_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
Gateway_send_Serial1_tronGo_listener = _listener;
}
void Gateway_send_Serial1_tronGo(struct Gateway_Instance *_instance, uint8_t nbID){
if (Gateway_send_Serial1_tronGo_listener != 0x0) Gateway_send_Serial1_tronGo_listener(_instance, nbID);
if (external_Gateway_send_Serial1_tronGo_listener != 0x0) external_Gateway_send_Serial1_tronGo_listener(_instance, nbID);
;
}
void (*external_Gateway_send_Serial1_hasID_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_external_Gateway_send_Serial1_hasID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
external_Gateway_send_Serial1_hasID_listener = _listener;
}
void (*Gateway_send_Serial1_hasID_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_Gateway_send_Serial1_hasID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
Gateway_send_Serial1_hasID_listener = _listener;
}
void Gateway_send_Serial1_hasID(struct Gateway_Instance *_instance, uint8_t id){
if (Gateway_send_Serial1_hasID_listener != 0x0) Gateway_send_Serial1_hasID_listener(_instance, id);
if (external_Gateway_send_Serial1_hasID_listener != 0x0) external_Gateway_send_Serial1_hasID_listener(_instance, id);
;
}
void (*external_Gateway_send_Serial1_iHaveID_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_external_Gateway_send_Serial1_iHaveID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
external_Gateway_send_Serial1_iHaveID_listener = _listener;
}
void (*Gateway_send_Serial1_iHaveID_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_Gateway_send_Serial1_iHaveID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
Gateway_send_Serial1_iHaveID_listener = _listener;
}
void Gateway_send_Serial1_iHaveID(struct Gateway_Instance *_instance, uint8_t id){
if (Gateway_send_Serial1_iHaveID_listener != 0x0) Gateway_send_Serial1_iHaveID_listener(_instance, id);
if (external_Gateway_send_Serial1_iHaveID_listener != 0x0) external_Gateway_send_Serial1_iHaveID_listener(_instance, id);
;
}
void (*external_Gateway_send_Serial1_mayIHaveID_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_external_Gateway_send_Serial1_mayIHaveID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
external_Gateway_send_Serial1_mayIHaveID_listener = _listener;
}
void (*Gateway_send_Serial1_mayIHaveID_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_Gateway_send_Serial1_mayIHaveID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
Gateway_send_Serial1_mayIHaveID_listener = _listener;
}
void Gateway_send_Serial1_mayIHaveID(struct Gateway_Instance *_instance, uint8_t id){
if (Gateway_send_Serial1_mayIHaveID_listener != 0x0) Gateway_send_Serial1_mayIHaveID_listener(_instance, id);
if (external_Gateway_send_Serial1_mayIHaveID_listener != 0x0) external_Gateway_send_Serial1_mayIHaveID_listener(_instance, id);
;
}
void (*external_Gateway_send_Serial2_addHead_listener)(struct Gateway_Instance *, uint8_t, uint8_t, uint8_t)= 0x0;
void register_external_Gateway_send_Serial2_addHead_listener(void (*_listener)(struct Gateway_Instance *, uint8_t, uint8_t, uint8_t)){
external_Gateway_send_Serial2_addHead_listener = _listener;
}
void (*Gateway_send_Serial2_addHead_listener)(struct Gateway_Instance *, uint8_t, uint8_t, uint8_t)= 0x0;
void register_Gateway_send_Serial2_addHead_listener(void (*_listener)(struct Gateway_Instance *, uint8_t, uint8_t, uint8_t)){
Gateway_send_Serial2_addHead_listener = _listener;
}
void Gateway_send_Serial2_addHead(struct Gateway_Instance *_instance, uint8_t x, uint8_t y, uint8_t id){
if (Gateway_send_Serial2_addHead_listener != 0x0) Gateway_send_Serial2_addHead_listener(_instance, x, y, id);
if (external_Gateway_send_Serial2_addHead_listener != 0x0) external_Gateway_send_Serial2_addHead_listener(_instance, x, y, id);
;
}
void (*external_Gateway_send_Serial2_loose_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_external_Gateway_send_Serial2_loose_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
external_Gateway_send_Serial2_loose_listener = _listener;
}
void (*Gateway_send_Serial2_loose_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_Gateway_send_Serial2_loose_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
Gateway_send_Serial2_loose_listener = _listener;
}
void Gateway_send_Serial2_loose(struct Gateway_Instance *_instance, uint8_t id){
if (Gateway_send_Serial2_loose_listener != 0x0) Gateway_send_Serial2_loose_listener(_instance, id);
if (external_Gateway_send_Serial2_loose_listener != 0x0) external_Gateway_send_Serial2_loose_listener(_instance, id);
;
}
void (*external_Gateway_send_Serial2_tronReady_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_external_Gateway_send_Serial2_tronReady_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
external_Gateway_send_Serial2_tronReady_listener = _listener;
}
void (*Gateway_send_Serial2_tronReady_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_Gateway_send_Serial2_tronReady_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
Gateway_send_Serial2_tronReady_listener = _listener;
}
void Gateway_send_Serial2_tronReady(struct Gateway_Instance *_instance, uint8_t id){
if (Gateway_send_Serial2_tronReady_listener != 0x0) Gateway_send_Serial2_tronReady_listener(_instance, id);
if (external_Gateway_send_Serial2_tronReady_listener != 0x0) external_Gateway_send_Serial2_tronReady_listener(_instance, id);
;
}
void (*external_Gateway_send_Serial2_tronGo_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_external_Gateway_send_Serial2_tronGo_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
external_Gateway_send_Serial2_tronGo_listener = _listener;
}
void (*Gateway_send_Serial2_tronGo_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_Gateway_send_Serial2_tronGo_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
Gateway_send_Serial2_tronGo_listener = _listener;
}
void Gateway_send_Serial2_tronGo(struct Gateway_Instance *_instance, uint8_t nbID){
if (Gateway_send_Serial2_tronGo_listener != 0x0) Gateway_send_Serial2_tronGo_listener(_instance, nbID);
if (external_Gateway_send_Serial2_tronGo_listener != 0x0) external_Gateway_send_Serial2_tronGo_listener(_instance, nbID);
;
}
void (*external_Gateway_send_Serial2_hasID_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_external_Gateway_send_Serial2_hasID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
external_Gateway_send_Serial2_hasID_listener = _listener;
}
void (*Gateway_send_Serial2_hasID_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_Gateway_send_Serial2_hasID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
Gateway_send_Serial2_hasID_listener = _listener;
}
void Gateway_send_Serial2_hasID(struct Gateway_Instance *_instance, uint8_t id){
if (Gateway_send_Serial2_hasID_listener != 0x0) Gateway_send_Serial2_hasID_listener(_instance, id);
if (external_Gateway_send_Serial2_hasID_listener != 0x0) external_Gateway_send_Serial2_hasID_listener(_instance, id);
;
}
void (*external_Gateway_send_Serial2_iHaveID_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_external_Gateway_send_Serial2_iHaveID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
external_Gateway_send_Serial2_iHaveID_listener = _listener;
}
void (*Gateway_send_Serial2_iHaveID_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_Gateway_send_Serial2_iHaveID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
Gateway_send_Serial2_iHaveID_listener = _listener;
}
void Gateway_send_Serial2_iHaveID(struct Gateway_Instance *_instance, uint8_t id){
if (Gateway_send_Serial2_iHaveID_listener != 0x0) Gateway_send_Serial2_iHaveID_listener(_instance, id);
if (external_Gateway_send_Serial2_iHaveID_listener != 0x0) external_Gateway_send_Serial2_iHaveID_listener(_instance, id);
;
}
void (*external_Gateway_send_Serial2_mayIHaveID_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_external_Gateway_send_Serial2_mayIHaveID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
external_Gateway_send_Serial2_mayIHaveID_listener = _listener;
}
void (*Gateway_send_Serial2_mayIHaveID_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_Gateway_send_Serial2_mayIHaveID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
Gateway_send_Serial2_mayIHaveID_listener = _listener;
}
void Gateway_send_Serial2_mayIHaveID(struct Gateway_Instance *_instance, uint8_t id){
if (Gateway_send_Serial2_mayIHaveID_listener != 0x0) Gateway_send_Serial2_mayIHaveID_listener(_instance, id);
if (external_Gateway_send_Serial2_mayIHaveID_listener != 0x0) external_Gateway_send_Serial2_mayIHaveID_listener(_instance, id);
;
}
void (*external_Gateway_send_Serial3_addHead_listener)(struct Gateway_Instance *, uint8_t, uint8_t, uint8_t)= 0x0;
void register_external_Gateway_send_Serial3_addHead_listener(void (*_listener)(struct Gateway_Instance *, uint8_t, uint8_t, uint8_t)){
external_Gateway_send_Serial3_addHead_listener = _listener;
}
void (*Gateway_send_Serial3_addHead_listener)(struct Gateway_Instance *, uint8_t, uint8_t, uint8_t)= 0x0;
void register_Gateway_send_Serial3_addHead_listener(void (*_listener)(struct Gateway_Instance *, uint8_t, uint8_t, uint8_t)){
Gateway_send_Serial3_addHead_listener = _listener;
}
void Gateway_send_Serial3_addHead(struct Gateway_Instance *_instance, uint8_t x, uint8_t y, uint8_t id){
if (Gateway_send_Serial3_addHead_listener != 0x0) Gateway_send_Serial3_addHead_listener(_instance, x, y, id);
if (external_Gateway_send_Serial3_addHead_listener != 0x0) external_Gateway_send_Serial3_addHead_listener(_instance, x, y, id);
;
}
void (*external_Gateway_send_Serial3_loose_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_external_Gateway_send_Serial3_loose_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
external_Gateway_send_Serial3_loose_listener = _listener;
}
void (*Gateway_send_Serial3_loose_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_Gateway_send_Serial3_loose_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
Gateway_send_Serial3_loose_listener = _listener;
}
void Gateway_send_Serial3_loose(struct Gateway_Instance *_instance, uint8_t id){
if (Gateway_send_Serial3_loose_listener != 0x0) Gateway_send_Serial3_loose_listener(_instance, id);
if (external_Gateway_send_Serial3_loose_listener != 0x0) external_Gateway_send_Serial3_loose_listener(_instance, id);
;
}
void (*external_Gateway_send_Serial3_tronReady_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_external_Gateway_send_Serial3_tronReady_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
external_Gateway_send_Serial3_tronReady_listener = _listener;
}
void (*Gateway_send_Serial3_tronReady_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_Gateway_send_Serial3_tronReady_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
Gateway_send_Serial3_tronReady_listener = _listener;
}
void Gateway_send_Serial3_tronReady(struct Gateway_Instance *_instance, uint8_t id){
if (Gateway_send_Serial3_tronReady_listener != 0x0) Gateway_send_Serial3_tronReady_listener(_instance, id);
if (external_Gateway_send_Serial3_tronReady_listener != 0x0) external_Gateway_send_Serial3_tronReady_listener(_instance, id);
;
}
void (*external_Gateway_send_Serial3_tronGo_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_external_Gateway_send_Serial3_tronGo_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
external_Gateway_send_Serial3_tronGo_listener = _listener;
}
void (*Gateway_send_Serial3_tronGo_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_Gateway_send_Serial3_tronGo_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
Gateway_send_Serial3_tronGo_listener = _listener;
}
void Gateway_send_Serial3_tronGo(struct Gateway_Instance *_instance, uint8_t nbID){
if (Gateway_send_Serial3_tronGo_listener != 0x0) Gateway_send_Serial3_tronGo_listener(_instance, nbID);
if (external_Gateway_send_Serial3_tronGo_listener != 0x0) external_Gateway_send_Serial3_tronGo_listener(_instance, nbID);
;
}
void (*external_Gateway_send_Serial3_hasID_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_external_Gateway_send_Serial3_hasID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
external_Gateway_send_Serial3_hasID_listener = _listener;
}
void (*Gateway_send_Serial3_hasID_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_Gateway_send_Serial3_hasID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
Gateway_send_Serial3_hasID_listener = _listener;
}
void Gateway_send_Serial3_hasID(struct Gateway_Instance *_instance, uint8_t id){
if (Gateway_send_Serial3_hasID_listener != 0x0) Gateway_send_Serial3_hasID_listener(_instance, id);
if (external_Gateway_send_Serial3_hasID_listener != 0x0) external_Gateway_send_Serial3_hasID_listener(_instance, id);
;
}
void (*external_Gateway_send_Serial3_iHaveID_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_external_Gateway_send_Serial3_iHaveID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
external_Gateway_send_Serial3_iHaveID_listener = _listener;
}
void (*Gateway_send_Serial3_iHaveID_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_Gateway_send_Serial3_iHaveID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
Gateway_send_Serial3_iHaveID_listener = _listener;
}
void Gateway_send_Serial3_iHaveID(struct Gateway_Instance *_instance, uint8_t id){
if (Gateway_send_Serial3_iHaveID_listener != 0x0) Gateway_send_Serial3_iHaveID_listener(_instance, id);
if (external_Gateway_send_Serial3_iHaveID_listener != 0x0) external_Gateway_send_Serial3_iHaveID_listener(_instance, id);
;
}
void (*external_Gateway_send_Serial3_mayIHaveID_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_external_Gateway_send_Serial3_mayIHaveID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
external_Gateway_send_Serial3_mayIHaveID_listener = _listener;
}
void (*Gateway_send_Serial3_mayIHaveID_listener)(struct Gateway_Instance *, uint8_t)= 0x0;
void register_Gateway_send_Serial3_mayIHaveID_listener(void (*_listener)(struct Gateway_Instance *, uint8_t)){
Gateway_send_Serial3_mayIHaveID_listener = _listener;
}
void Gateway_send_Serial3_mayIHaveID(struct Gateway_Instance *_instance, uint8_t id){
if (Gateway_send_Serial3_mayIHaveID_listener != 0x0) Gateway_send_Serial3_mayIHaveID_listener(_instance, id);
if (external_Gateway_send_Serial3_mayIHaveID_listener != 0x0) external_Gateway_send_Serial3_mayIHaveID_listener(_instance, id);
;
}




#define MAX_INSTANCES 8
#define FIFO_SIZE 256

/*********************************
 * Instance IDs and lookup
 *********************************/

void * instances[MAX_INSTANCES];
uint16_t instances_count = 0;

void * instance_by_id(uint16_t id) {
  return instances[id];
}

uint16_t add_instance(void * instance_struct) {
  instances[instances_count] = instance_struct;
  return instances_count++;
}

/******************************************
 * Simple byte FIFO implementation
 ******************************************/

byte fifo[FIFO_SIZE];
int fifo_head = 0;
int fifo_tail = 0;

// Returns the number of byte currently in the fifo
int fifo_byte_length() {
  if (fifo_tail >= fifo_head)
    return fifo_tail - fifo_head;
  return fifo_tail + FIFO_SIZE - fifo_head;
}

// Returns the number of bytes currently available in the fifo
int fifo_byte_available() {
  return FIFO_SIZE - 1 - fifo_byte_length();
}

// Returns true if the fifo is empty
int fifo_empty() {
  return fifo_head == fifo_tail;
}

// Return true if the fifo is full
int fifo_full() {
  return fifo_head == ((fifo_tail + 1) % FIFO_SIZE);
}

// Enqueue 1 byte in the fifo if there is space
// returns 1 for sucess and 0 if the fifo was full
int fifo_enqueue(byte b) {
  int new_tail = (fifo_tail + 1) % FIFO_SIZE;
  if (new_tail == fifo_head) return 0; // the fifo is full
  fifo[fifo_tail] = b;
  fifo_tail = new_tail;
  return 1;
}

// Enqueue 1 byte in the fifo without checking for available space
// The caller should have checked that there is enough space
int _fifo_enqueue(byte b) {
  fifo[fifo_tail] = b;
  fifo_tail = (fifo_tail + 1) % FIFO_SIZE;
}

// Dequeue 1 byte in the fifo.
// The caller should check that the fifo is not empty
byte fifo_dequeue() {
  if (!fifo_empty()) {
    byte result = fifo[fifo_head];
    fifo_head = (fifo_head + 1) % FIFO_SIZE;
    return result;
  }
  return 0;
}


/*SOFTWARE_SERIAL*/

#define Serial2_LISTENER_STATE_IDLE 0
#define Serial2_LISTENER_STATE_READING 1
#define Serial2_LISTENER_STATE_ESCAPE 2
#define Serial2_LISTENER_STATE_ERROR 3


#define Serial2_START_BYTE 18
#define Serial2_STOP_BYTE 19
#define Serial2_ESCAPE_BYTE 125

#define Serial2_LIMIT_BYTE_PER_LOOP 10
#define Serial2_MAX_MSG_SIZE 5
#define Serial2_MSG_BUFFER_SIZE 10


byte Serial2_serialBuffer[Serial2_MSG_BUFFER_SIZE];
uint8_t Serial2_serialMsgSize = 0;
byte Serial2_incoming = 0;
uint8_t Serial2_serialListenerState = Serial2_LISTENER_STATE_IDLE;


struct Serial2_instance_type {
    uint16_t listener_id;
    //Connector// Pointer to receiver list
struct Msg_Handler ** Serial2_receiver_list_head;
struct Msg_Handler ** Serial2_receiver_list_tail;
// Handler Array
struct Msg_Handler * Serial2_handlers;

} Serial2_instance;

int fifo_byte_available();
int _fifo_enqueue(byte b);

void Serial2_setup() {
	Serial2.begin(115200);
}

void Serial2_set_listener_id(uint16_t id) {
	Serial2_instance.listener_id = id;
}


void Serial2_forwardMessage(byte * msg, uint8_t size) {
  
  Serial2.write(Serial2_START_BYTE);
  for(uint8_t i = 0; i < size; i++) {
    if((msg[i] == Serial2_START_BYTE) 
		|| (msg[i] == Serial2_STOP_BYTE) 
		|| (msg[i] == Serial2_ESCAPE_BYTE)) {
      Serial2.write(Serial2_ESCAPE_BYTE);
    }
    Serial2.write(msg[i]);
  }
  Serial2.write(Serial2_STOP_BYTE);
}

void Serial2_read() {
  byte limit = 0;
  while ((Serial2.available()) && (limit < Serial2_LIMIT_BYTE_PER_LOOP)) {
   limit++;
    Serial2_incoming = Serial2.read();
    
    switch(Serial2_serialListenerState) {
      case Serial2_LISTENER_STATE_IDLE:
        if(Serial2_incoming == Serial2_START_BYTE) {
          Serial2_serialListenerState = Serial2_LISTENER_STATE_READING;
          Serial2_serialMsgSize = 0;
        }
      break;
      
      case Serial2_LISTENER_STATE_READING:
        if (Serial2_serialMsgSize > Serial2_MAX_MSG_SIZE) {
          Serial2_serialListenerState = Serial2_LISTENER_STATE_ERROR;
        } else {
          if(Serial2_incoming == Serial2_STOP_BYTE) {
            Serial2_serialListenerState = Serial2_LISTENER_STATE_IDLE;
            externalMessageEnqueue(Serial2_serialBuffer, Serial2_serialMsgSize, Serial2_instance.listener_id);
            
          } else if (Serial2_incoming == Serial2_ESCAPE_BYTE) {
            Serial2_serialListenerState = Serial2_LISTENER_STATE_ESCAPE;
          } else {
            Serial2_serialBuffer[Serial2_serialMsgSize] = Serial2_incoming;
            Serial2_serialMsgSize++;
          }
        }
      break;
      
      case Serial2_LISTENER_STATE_ESCAPE:
        if (Serial2_serialMsgSize >= Serial2_MAX_MSG_SIZE) {
          Serial2_serialListenerState = Serial2_LISTENER_STATE_ERROR;
        } else {
          Serial2_serialBuffer[Serial2_serialMsgSize] = Serial2_incoming;
          Serial2_serialMsgSize++;
          Serial2_serialListenerState = Serial2_LISTENER_STATE_READING;
        }
      break;
      
      case Serial2_LISTENER_STATE_ERROR:
        Serial2_serialListenerState = Serial2_LISTENER_STATE_IDLE;
        Serial2_serialMsgSize = 0;
      break;
    }
  }
  
}

/*SOFTWARE_SERIAL*/

#define Serial3_LISTENER_STATE_IDLE 0
#define Serial3_LISTENER_STATE_READING 1
#define Serial3_LISTENER_STATE_ESCAPE 2
#define Serial3_LISTENER_STATE_ERROR 3


#define Serial3_START_BYTE 18
#define Serial3_STOP_BYTE 19
#define Serial3_ESCAPE_BYTE 125

#define Serial3_LIMIT_BYTE_PER_LOOP 10
#define Serial3_MAX_MSG_SIZE 5
#define Serial3_MSG_BUFFER_SIZE 10


byte Serial3_serialBuffer[Serial3_MSG_BUFFER_SIZE];
uint8_t Serial3_serialMsgSize = 0;
byte Serial3_incoming = 0;
uint8_t Serial3_serialListenerState = Serial3_LISTENER_STATE_IDLE;


struct Serial3_instance_type {
    uint16_t listener_id;
    //Connector// Pointer to receiver list
struct Msg_Handler ** Serial3_receiver_list_head;
struct Msg_Handler ** Serial3_receiver_list_tail;
// Handler Array
struct Msg_Handler * Serial3_handlers;

} Serial3_instance;

int fifo_byte_available();
int _fifo_enqueue(byte b);

void Serial3_setup() {
	Serial3.begin(115200);
}

void Serial3_set_listener_id(uint16_t id) {
	Serial3_instance.listener_id = id;
}


void Serial3_forwardMessage(byte * msg, uint8_t size) {
  
  Serial3.write(Serial3_START_BYTE);
  for(uint8_t i = 0; i < size; i++) {
    if((msg[i] == Serial3_START_BYTE) 
		|| (msg[i] == Serial3_STOP_BYTE) 
		|| (msg[i] == Serial3_ESCAPE_BYTE)) {
      Serial3.write(Serial3_ESCAPE_BYTE);
    }
    Serial3.write(msg[i]);
  }
  Serial3.write(Serial3_STOP_BYTE);
}

void Serial3_read() {
  byte limit = 0;
  while ((Serial3.available()) && (limit < Serial3_LIMIT_BYTE_PER_LOOP)) {
   limit++;
    Serial3_incoming = Serial3.read();
    
    switch(Serial3_serialListenerState) {
      case Serial3_LISTENER_STATE_IDLE:
        if(Serial3_incoming == Serial3_START_BYTE) {
          Serial3_serialListenerState = Serial3_LISTENER_STATE_READING;
          Serial3_serialMsgSize = 0;
        }
      break;
      
      case Serial3_LISTENER_STATE_READING:
        if (Serial3_serialMsgSize > Serial3_MAX_MSG_SIZE) {
          Serial3_serialListenerState = Serial3_LISTENER_STATE_ERROR;
        } else {
          if(Serial3_incoming == Serial3_STOP_BYTE) {
            Serial3_serialListenerState = Serial3_LISTENER_STATE_IDLE;
            externalMessageEnqueue(Serial3_serialBuffer, Serial3_serialMsgSize, Serial3_instance.listener_id);
            
          } else if (Serial3_incoming == Serial3_ESCAPE_BYTE) {
            Serial3_serialListenerState = Serial3_LISTENER_STATE_ESCAPE;
          } else {
            Serial3_serialBuffer[Serial3_serialMsgSize] = Serial3_incoming;
            Serial3_serialMsgSize++;
          }
        }
      break;
      
      case Serial3_LISTENER_STATE_ESCAPE:
        if (Serial3_serialMsgSize >= Serial3_MAX_MSG_SIZE) {
          Serial3_serialListenerState = Serial3_LISTENER_STATE_ERROR;
        } else {
          Serial3_serialBuffer[Serial3_serialMsgSize] = Serial3_incoming;
          Serial3_serialMsgSize++;
          Serial3_serialListenerState = Serial3_LISTENER_STATE_READING;
        }
      break;
      
      case Serial3_LISTENER_STATE_ERROR:
        Serial3_serialListenerState = Serial3_LISTENER_STATE_IDLE;
        Serial3_serialMsgSize = 0;
      break;
    }
  }
  
}



/*****************************************************************************
 * Definitions for configuration : GatewayCfg
 *****************************************************************************/

//Declaration of connexion array
#define NB_MAX_CONNEXION 4
struct Msg_Handler * GatewayCfg_receivers[NB_MAX_CONNEXION];

//Declaration of instance variables
//Instance GatewayCfg_g
struct Gateway_Instance GatewayCfg_g_var;
struct Msg_Handler GatewayCfg_g_Serial0_handlers;
uint16_t GatewayCfg_g_Serial0_msgs[7];
void * GatewayCfg_g_Serial0_handlers_tab[7];

struct Msg_Handler GatewayCfg_g_Serial1_handlers;
uint16_t GatewayCfg_g_Serial1_msgs[7];
void * GatewayCfg_g_Serial1_handlers_tab[7];

struct Msg_Handler GatewayCfg_g_Serial2_handlers;
uint16_t GatewayCfg_g_Serial2_msgs[7];
void * GatewayCfg_g_Serial2_handlers_tab[7];

struct Msg_Handler GatewayCfg_g_Serial3_handlers;
uint16_t GatewayCfg_g_Serial3_msgs[7];
void * GatewayCfg_g_Serial3_handlers_tab[7];




//Dynamic dispatcher for message mayIHaveID
void dispatch_mayIHaveID(uint16_t sender, uint8_t param_id) {
struct executor {
static void executor_dispatch_mayIHaveID(struct Msg_Handler ** head, struct Msg_Handler ** tail, uint8_t param_id) {
struct Msg_Handler ** cur = head;
while (cur != NULL) {
   void (*handler)(void *, uint8_t param_id) = NULL;
   int i;
   for(i = 0; i < (**cur).nb_msg; i++) {
       if((**cur).msg[i] == 52) {
           handler = (void (*) (void *, uint8_t)) (**cur).msg_handler[i];
           break;
       }
   }
   if(handler != NULL) {
       handler((**cur).instance, param_id);
}
   if(cur == tail){
       cur = NULL;}
   else {
   cur++;}
}
}
};
if (sender == GatewayCfg_g_var.id_Serial1) {
executor::executor_dispatch_mayIHaveID(GatewayCfg_g_var.Serial1_receiver_list_head, GatewayCfg_g_var.Serial1_receiver_list_tail, param_id);}
if (sender == GatewayCfg_g_var.id_Serial3) {
executor::executor_dispatch_mayIHaveID(GatewayCfg_g_var.Serial3_receiver_list_head, GatewayCfg_g_var.Serial3_receiver_list_tail, param_id);}
if (sender == GatewayCfg_g_var.id_Serial2) {
executor::executor_dispatch_mayIHaveID(GatewayCfg_g_var.Serial2_receiver_list_head, GatewayCfg_g_var.Serial2_receiver_list_tail, param_id);}
if (sender == GatewayCfg_g_var.id_Serial0) {
executor::executor_dispatch_mayIHaveID(GatewayCfg_g_var.Serial0_receiver_list_head, GatewayCfg_g_var.Serial0_receiver_list_tail, param_id);}
if (sender == Serial_instance.listener_id) {
executor::executor_dispatch_mayIHaveID(Serial_instance.Serial0_receiver_list_head,Serial_instance.Serial0_receiver_list_tail, param_id);}
if (sender == Serial2_instance.listener_id) {
executor::executor_dispatch_mayIHaveID(Serial2_instance.Serial2_receiver_list_head,Serial2_instance.Serial2_receiver_list_tail, param_id);}
if (sender == Serial1_instance.listener_id) {
executor::executor_dispatch_mayIHaveID(Serial1_instance.Serial1_receiver_list_head,Serial1_instance.Serial1_receiver_list_tail, param_id);}
if (sender == Serial3_instance.listener_id) {
executor::executor_dispatch_mayIHaveID(Serial3_instance.Serial3_receiver_list_head,Serial3_instance.Serial3_receiver_list_tail, param_id);}
}

//Dynamic dispatcher for message tronGo
void dispatch_tronGo(uint16_t sender, uint8_t param_nbID) {
struct executor {
static void executor_dispatch_tronGo(struct Msg_Handler ** head, struct Msg_Handler ** tail, uint8_t param_nbID) {
struct Msg_Handler ** cur = head;
while (cur != NULL) {
   void (*handler)(void *, uint8_t param_nbID) = NULL;
   int i;
   for(i = 0; i < (**cur).nb_msg; i++) {
       if((**cur).msg[i] == 54) {
           handler = (void (*) (void *, uint8_t)) (**cur).msg_handler[i];
           break;
       }
   }
   if(handler != NULL) {
       handler((**cur).instance, param_nbID);
}
   if(cur == tail){
       cur = NULL;}
   else {
   cur++;}
}
}
};
if (sender == GatewayCfg_g_var.id_Serial3) {
executor::executor_dispatch_tronGo(GatewayCfg_g_var.Serial3_receiver_list_head, GatewayCfg_g_var.Serial3_receiver_list_tail, param_nbID);}
if (sender == GatewayCfg_g_var.id_Serial2) {
executor::executor_dispatch_tronGo(GatewayCfg_g_var.Serial2_receiver_list_head, GatewayCfg_g_var.Serial2_receiver_list_tail, param_nbID);}
if (sender == GatewayCfg_g_var.id_Serial0) {
executor::executor_dispatch_tronGo(GatewayCfg_g_var.Serial0_receiver_list_head, GatewayCfg_g_var.Serial0_receiver_list_tail, param_nbID);}
if (sender == GatewayCfg_g_var.id_Serial1) {
executor::executor_dispatch_tronGo(GatewayCfg_g_var.Serial1_receiver_list_head, GatewayCfg_g_var.Serial1_receiver_list_tail, param_nbID);}
if (sender == Serial_instance.listener_id) {
executor::executor_dispatch_tronGo(Serial_instance.Serial0_receiver_list_head,Serial_instance.Serial0_receiver_list_tail, param_nbID);}
if (sender == Serial2_instance.listener_id) {
executor::executor_dispatch_tronGo(Serial2_instance.Serial2_receiver_list_head,Serial2_instance.Serial2_receiver_list_tail, param_nbID);}
if (sender == Serial1_instance.listener_id) {
executor::executor_dispatch_tronGo(Serial1_instance.Serial1_receiver_list_head,Serial1_instance.Serial1_receiver_list_tail, param_nbID);}
if (sender == Serial3_instance.listener_id) {
executor::executor_dispatch_tronGo(Serial3_instance.Serial3_receiver_list_head,Serial3_instance.Serial3_receiver_list_tail, param_nbID);}
}

//Dynamic dispatcher for message hasID
void dispatch_hasID(uint16_t sender, uint8_t param_id) {
struct executor {
static void executor_dispatch_hasID(struct Msg_Handler ** head, struct Msg_Handler ** tail, uint8_t param_id) {
struct Msg_Handler ** cur = head;
while (cur != NULL) {
   void (*handler)(void *, uint8_t param_id) = NULL;
   int i;
   for(i = 0; i < (**cur).nb_msg; i++) {
       if((**cur).msg[i] == 50) {
           handler = (void (*) (void *, uint8_t)) (**cur).msg_handler[i];
           break;
       }
   }
   if(handler != NULL) {
       handler((**cur).instance, param_id);
}
   if(cur == tail){
       cur = NULL;}
   else {
   cur++;}
}
}
};
if (sender == GatewayCfg_g_var.id_Serial1) {
executor::executor_dispatch_hasID(GatewayCfg_g_var.Serial1_receiver_list_head, GatewayCfg_g_var.Serial1_receiver_list_tail, param_id);}
if (sender == GatewayCfg_g_var.id_Serial0) {
executor::executor_dispatch_hasID(GatewayCfg_g_var.Serial0_receiver_list_head, GatewayCfg_g_var.Serial0_receiver_list_tail, param_id);}
if (sender == GatewayCfg_g_var.id_Serial2) {
executor::executor_dispatch_hasID(GatewayCfg_g_var.Serial2_receiver_list_head, GatewayCfg_g_var.Serial2_receiver_list_tail, param_id);}
if (sender == GatewayCfg_g_var.id_Serial3) {
executor::executor_dispatch_hasID(GatewayCfg_g_var.Serial3_receiver_list_head, GatewayCfg_g_var.Serial3_receiver_list_tail, param_id);}
if (sender == Serial_instance.listener_id) {
executor::executor_dispatch_hasID(Serial_instance.Serial0_receiver_list_head,Serial_instance.Serial0_receiver_list_tail, param_id);}
if (sender == Serial2_instance.listener_id) {
executor::executor_dispatch_hasID(Serial2_instance.Serial2_receiver_list_head,Serial2_instance.Serial2_receiver_list_tail, param_id);}
if (sender == Serial1_instance.listener_id) {
executor::executor_dispatch_hasID(Serial1_instance.Serial1_receiver_list_head,Serial1_instance.Serial1_receiver_list_tail, param_id);}
if (sender == Serial3_instance.listener_id) {
executor::executor_dispatch_hasID(Serial3_instance.Serial3_receiver_list_head,Serial3_instance.Serial3_receiver_list_tail, param_id);}
}

//Dynamic dispatcher for message iHaveID
void dispatch_iHaveID(uint16_t sender, uint8_t param_id) {
struct executor {
static void executor_dispatch_iHaveID(struct Msg_Handler ** head, struct Msg_Handler ** tail, uint8_t param_id) {
struct Msg_Handler ** cur = head;
while (cur != NULL) {
   void (*handler)(void *, uint8_t param_id) = NULL;
   int i;
   for(i = 0; i < (**cur).nb_msg; i++) {
       if((**cur).msg[i] == 51) {
           handler = (void (*) (void *, uint8_t)) (**cur).msg_handler[i];
           break;
       }
   }
   if(handler != NULL) {
       handler((**cur).instance, param_id);
}
   if(cur == tail){
       cur = NULL;}
   else {
   cur++;}
}
}
};
if (sender == GatewayCfg_g_var.id_Serial1) {
executor::executor_dispatch_iHaveID(GatewayCfg_g_var.Serial1_receiver_list_head, GatewayCfg_g_var.Serial1_receiver_list_tail, param_id);}
if (sender == GatewayCfg_g_var.id_Serial3) {
executor::executor_dispatch_iHaveID(GatewayCfg_g_var.Serial3_receiver_list_head, GatewayCfg_g_var.Serial3_receiver_list_tail, param_id);}
if (sender == GatewayCfg_g_var.id_Serial2) {
executor::executor_dispatch_iHaveID(GatewayCfg_g_var.Serial2_receiver_list_head, GatewayCfg_g_var.Serial2_receiver_list_tail, param_id);}
if (sender == GatewayCfg_g_var.id_Serial0) {
executor::executor_dispatch_iHaveID(GatewayCfg_g_var.Serial0_receiver_list_head, GatewayCfg_g_var.Serial0_receiver_list_tail, param_id);}
if (sender == Serial_instance.listener_id) {
executor::executor_dispatch_iHaveID(Serial_instance.Serial0_receiver_list_head,Serial_instance.Serial0_receiver_list_tail, param_id);}
if (sender == Serial2_instance.listener_id) {
executor::executor_dispatch_iHaveID(Serial2_instance.Serial2_receiver_list_head,Serial2_instance.Serial2_receiver_list_tail, param_id);}
if (sender == Serial1_instance.listener_id) {
executor::executor_dispatch_iHaveID(Serial1_instance.Serial1_receiver_list_head,Serial1_instance.Serial1_receiver_list_tail, param_id);}
if (sender == Serial3_instance.listener_id) {
executor::executor_dispatch_iHaveID(Serial3_instance.Serial3_receiver_list_head,Serial3_instance.Serial3_receiver_list_tail, param_id);}
}

//Dynamic dispatcher for message loose
void dispatch_loose(uint16_t sender, uint8_t param_id) {
struct executor {
static void executor_dispatch_loose(struct Msg_Handler ** head, struct Msg_Handler ** tail, uint8_t param_id) {
struct Msg_Handler ** cur = head;
while (cur != NULL) {
   void (*handler)(void *, uint8_t param_id) = NULL;
   int i;
   for(i = 0; i < (**cur).nb_msg; i++) {
       if((**cur).msg[i] == 56) {
           handler = (void (*) (void *, uint8_t)) (**cur).msg_handler[i];
           break;
       }
   }
   if(handler != NULL) {
       handler((**cur).instance, param_id);
}
   if(cur == tail){
       cur = NULL;}
   else {
   cur++;}
}
}
};
if (sender == GatewayCfg_g_var.id_Serial3) {
executor::executor_dispatch_loose(GatewayCfg_g_var.Serial3_receiver_list_head, GatewayCfg_g_var.Serial3_receiver_list_tail, param_id);}
if (sender == GatewayCfg_g_var.id_Serial2) {
executor::executor_dispatch_loose(GatewayCfg_g_var.Serial2_receiver_list_head, GatewayCfg_g_var.Serial2_receiver_list_tail, param_id);}
if (sender == GatewayCfg_g_var.id_Serial0) {
executor::executor_dispatch_loose(GatewayCfg_g_var.Serial0_receiver_list_head, GatewayCfg_g_var.Serial0_receiver_list_tail, param_id);}
if (sender == GatewayCfg_g_var.id_Serial1) {
executor::executor_dispatch_loose(GatewayCfg_g_var.Serial1_receiver_list_head, GatewayCfg_g_var.Serial1_receiver_list_tail, param_id);}
if (sender == Serial_instance.listener_id) {
executor::executor_dispatch_loose(Serial_instance.Serial0_receiver_list_head,Serial_instance.Serial0_receiver_list_tail, param_id);}
if (sender == Serial2_instance.listener_id) {
executor::executor_dispatch_loose(Serial2_instance.Serial2_receiver_list_head,Serial2_instance.Serial2_receiver_list_tail, param_id);}
if (sender == Serial1_instance.listener_id) {
executor::executor_dispatch_loose(Serial1_instance.Serial1_receiver_list_head,Serial1_instance.Serial1_receiver_list_tail, param_id);}
if (sender == Serial3_instance.listener_id) {
executor::executor_dispatch_loose(Serial3_instance.Serial3_receiver_list_head,Serial3_instance.Serial3_receiver_list_tail, param_id);}
}

//Dynamic dispatcher for message tronReady
void dispatch_tronReady(uint16_t sender, uint8_t param_id) {
struct executor {
static void executor_dispatch_tronReady(struct Msg_Handler ** head, struct Msg_Handler ** tail, uint8_t param_id) {
struct Msg_Handler ** cur = head;
while (cur != NULL) {
   void (*handler)(void *, uint8_t param_id) = NULL;
   int i;
   for(i = 0; i < (**cur).nb_msg; i++) {
       if((**cur).msg[i] == 53) {
           handler = (void (*) (void *, uint8_t)) (**cur).msg_handler[i];
           break;
       }
   }
   if(handler != NULL) {
       handler((**cur).instance, param_id);
}
   if(cur == tail){
       cur = NULL;}
   else {
   cur++;}
}
}
};
if (sender == GatewayCfg_g_var.id_Serial2) {
executor::executor_dispatch_tronReady(GatewayCfg_g_var.Serial2_receiver_list_head, GatewayCfg_g_var.Serial2_receiver_list_tail, param_id);}
if (sender == GatewayCfg_g_var.id_Serial3) {
executor::executor_dispatch_tronReady(GatewayCfg_g_var.Serial3_receiver_list_head, GatewayCfg_g_var.Serial3_receiver_list_tail, param_id);}
if (sender == GatewayCfg_g_var.id_Serial0) {
executor::executor_dispatch_tronReady(GatewayCfg_g_var.Serial0_receiver_list_head, GatewayCfg_g_var.Serial0_receiver_list_tail, param_id);}
if (sender == GatewayCfg_g_var.id_Serial1) {
executor::executor_dispatch_tronReady(GatewayCfg_g_var.Serial1_receiver_list_head, GatewayCfg_g_var.Serial1_receiver_list_tail, param_id);}
if (sender == Serial_instance.listener_id) {
executor::executor_dispatch_tronReady(Serial_instance.Serial0_receiver_list_head,Serial_instance.Serial0_receiver_list_tail, param_id);}
if (sender == Serial2_instance.listener_id) {
executor::executor_dispatch_tronReady(Serial2_instance.Serial2_receiver_list_head,Serial2_instance.Serial2_receiver_list_tail, param_id);}
if (sender == Serial1_instance.listener_id) {
executor::executor_dispatch_tronReady(Serial1_instance.Serial1_receiver_list_head,Serial1_instance.Serial1_receiver_list_tail, param_id);}
if (sender == Serial3_instance.listener_id) {
executor::executor_dispatch_tronReady(Serial3_instance.Serial3_receiver_list_head,Serial3_instance.Serial3_receiver_list_tail, param_id);}
}

//Dynamic dispatcher for message addHead
void dispatch_addHead(uint16_t sender, uint8_t param_x, uint8_t param_y, uint8_t param_id) {
struct executor {
static void executor_dispatch_addHead(struct Msg_Handler ** head, struct Msg_Handler ** tail, uint8_t param_x, uint8_t param_y, uint8_t param_id) {
struct Msg_Handler ** cur = head;
while (cur != NULL) {
   void (*handler)(void *, uint8_t param_x, uint8_t param_y, uint8_t param_id) = NULL;
   int i;
   for(i = 0; i < (**cur).nb_msg; i++) {
       if((**cur).msg[i] == 55) {
           handler = (void (*) (void *, uint8_t, uint8_t, uint8_t)) (**cur).msg_handler[i];
           break;
       }
   }
   if(handler != NULL) {
       handler((**cur).instance, param_x, param_y, param_id);
}
   if(cur == tail){
       cur = NULL;}
   else {
   cur++;}
}
}
};
if (sender == GatewayCfg_g_var.id_Serial2) {
executor::executor_dispatch_addHead(GatewayCfg_g_var.Serial2_receiver_list_head, GatewayCfg_g_var.Serial2_receiver_list_tail, param_x, param_y, param_id);}
if (sender == GatewayCfg_g_var.id_Serial3) {
executor::executor_dispatch_addHead(GatewayCfg_g_var.Serial3_receiver_list_head, GatewayCfg_g_var.Serial3_receiver_list_tail, param_x, param_y, param_id);}
if (sender == GatewayCfg_g_var.id_Serial0) {
executor::executor_dispatch_addHead(GatewayCfg_g_var.Serial0_receiver_list_head, GatewayCfg_g_var.Serial0_receiver_list_tail, param_x, param_y, param_id);}
if (sender == GatewayCfg_g_var.id_Serial1) {
executor::executor_dispatch_addHead(GatewayCfg_g_var.Serial1_receiver_list_head, GatewayCfg_g_var.Serial1_receiver_list_tail, param_x, param_y, param_id);}
if (sender == Serial_instance.listener_id) {
executor::executor_dispatch_addHead(Serial_instance.Serial0_receiver_list_head,Serial_instance.Serial0_receiver_list_tail, param_x, param_y, param_id);}
if (sender == Serial2_instance.listener_id) {
executor::executor_dispatch_addHead(Serial2_instance.Serial2_receiver_list_head,Serial2_instance.Serial2_receiver_list_tail, param_x, param_y, param_id);}
if (sender == Serial1_instance.listener_id) {
executor::executor_dispatch_addHead(Serial1_instance.Serial1_receiver_list_head,Serial1_instance.Serial1_receiver_list_tail, param_x, param_y, param_id);}
if (sender == Serial3_instance.listener_id) {
executor::executor_dispatch_addHead(Serial3_instance.Serial3_receiver_list_head,Serial3_instance.Serial3_receiver_list_tail, param_x, param_y, param_id);}
}

void processMessageQueue() {
if (fifo_empty()) return; // return if there is nothing to do

byte mbuf[5];
uint8_t mbufi = 0;

// Read the code of the next port/message in the queue
uint16_t code = fifo_dequeue() << 8;

code += fifo_dequeue();

// Switch to call the appropriate handler
switch(code) {
case 52:
while (mbufi < 3) mbuf[mbufi++] = fifo_dequeue();
union u_mayIHaveID_id_t {
uint8_t p;
byte bytebuffer[1];
} u_mayIHaveID_id;
u_mayIHaveID_id.bytebuffer[0] = mbuf[2];
dispatch_mayIHaveID((mbuf[0] << 8) + mbuf[1] /* instance port*/,
 u_mayIHaveID_id.p /* id */ );
break;
case 54:
while (mbufi < 3) mbuf[mbufi++] = fifo_dequeue();
union u_tronGo_nbID_t {
uint8_t p;
byte bytebuffer[1];
} u_tronGo_nbID;
u_tronGo_nbID.bytebuffer[0] = mbuf[2];
dispatch_tronGo((mbuf[0] << 8) + mbuf[1] /* instance port*/,
 u_tronGo_nbID.p /* nbID */ );
break;
case 56:
while (mbufi < 3) mbuf[mbufi++] = fifo_dequeue();
union u_loose_id_t {
uint8_t p;
byte bytebuffer[1];
} u_loose_id;
u_loose_id.bytebuffer[0] = mbuf[2];
dispatch_loose((mbuf[0] << 8) + mbuf[1] /* instance port*/,
 u_loose_id.p /* id */ );
break;
case 50:
while (mbufi < 3) mbuf[mbufi++] = fifo_dequeue();
union u_hasID_id_t {
uint8_t p;
byte bytebuffer[1];
} u_hasID_id;
u_hasID_id.bytebuffer[0] = mbuf[2];
dispatch_hasID((mbuf[0] << 8) + mbuf[1] /* instance port*/,
 u_hasID_id.p /* id */ );
break;
case 51:
while (mbufi < 3) mbuf[mbufi++] = fifo_dequeue();
union u_iHaveID_id_t {
uint8_t p;
byte bytebuffer[1];
} u_iHaveID_id;
u_iHaveID_id.bytebuffer[0] = mbuf[2];
dispatch_iHaveID((mbuf[0] << 8) + mbuf[1] /* instance port*/,
 u_iHaveID_id.p /* id */ );
break;
case 53:
while (mbufi < 3) mbuf[mbufi++] = fifo_dequeue();
union u_tronReady_id_t {
uint8_t p;
byte bytebuffer[1];
} u_tronReady_id;
u_tronReady_id.bytebuffer[0] = mbuf[2];
dispatch_tronReady((mbuf[0] << 8) + mbuf[1] /* instance port*/,
 u_tronReady_id.p /* id */ );
break;
case 55:
while (mbufi < 5) mbuf[mbufi++] = fifo_dequeue();
union u_addHead_x_t {
uint8_t p;
byte bytebuffer[1];
} u_addHead_x;
u_addHead_x.bytebuffer[0] = mbuf[2];
union u_addHead_y_t {
uint8_t p;
byte bytebuffer[1];
} u_addHead_y;
u_addHead_y.bytebuffer[0] = mbuf[3];
union u_addHead_id_t {
uint8_t p;
byte bytebuffer[1];
} u_addHead_id;
u_addHead_id.bytebuffer[0] = mbuf[4];
dispatch_addHead((mbuf[0] << 8) + mbuf[1] /* instance port*/,
 u_addHead_x.p /* x */ ,
 u_addHead_y.p /* y */ ,
 u_addHead_id.p /* id */ );
break;
}
}

// Forwarding of messages Serial::Gateway::Serial0::addHead
void forward_Serial_Gateway_send_Serial0_addHead(struct Gateway_Instance *_instance, uint8_t x, uint8_t y, uint8_t id){
byte forward_buf[5];
forward_buf[0] = (55 >> 8) & 0xFF;
forward_buf[1] =  55 & 0xFF;


// parameter x
union u_x_t {
uint8_t p;
byte bytebuffer[1];
} u_x;
u_x.p = x;
forward_buf[2] =  (u_x.bytebuffer[0] & 0xFF);

// parameter y
union u_y_t {
uint8_t p;
byte bytebuffer[1];
} u_y;
u_y.p = y;
forward_buf[3] =  (u_y.bytebuffer[0] & 0xFF);

// parameter id
union u_id_t {
uint8_t p;
byte bytebuffer[1];
} u_id;
u_id.p = id;
forward_buf[4] =  (u_id.bytebuffer[0] & 0xFF);

//Forwarding with specified function 
Serial_forwardMessage(forward_buf, 5);
}

// Forwarding of messages Serial::Gateway::Serial0::loose
void forward_Serial_Gateway_send_Serial0_loose(struct Gateway_Instance *_instance, uint8_t id){
byte forward_buf[3];
forward_buf[0] = (56 >> 8) & 0xFF;
forward_buf[1] =  56 & 0xFF;


// parameter id
union u_id_t {
uint8_t p;
byte bytebuffer[1];
} u_id;
u_id.p = id;
forward_buf[2] =  (u_id.bytebuffer[0] & 0xFF);

//Forwarding with specified function 
Serial_forwardMessage(forward_buf, 3);
}

// Forwarding of messages Serial::Gateway::Serial0::tronReady
void forward_Serial_Gateway_send_Serial0_tronReady(struct Gateway_Instance *_instance, uint8_t id){
byte forward_buf[3];
forward_buf[0] = (53 >> 8) & 0xFF;
forward_buf[1] =  53 & 0xFF;


// parameter id
union u_id_t {
uint8_t p;
byte bytebuffer[1];
} u_id;
u_id.p = id;
forward_buf[2] =  (u_id.bytebuffer[0] & 0xFF);

//Forwarding with specified function 
Serial_forwardMessage(forward_buf, 3);
}

// Forwarding of messages Serial::Gateway::Serial0::tronGo
void forward_Serial_Gateway_send_Serial0_tronGo(struct Gateway_Instance *_instance, uint8_t nbID){
byte forward_buf[3];
forward_buf[0] = (54 >> 8) & 0xFF;
forward_buf[1] =  54 & 0xFF;


// parameter nbID
union u_nbID_t {
uint8_t p;
byte bytebuffer[1];
} u_nbID;
u_nbID.p = nbID;
forward_buf[2] =  (u_nbID.bytebuffer[0] & 0xFF);

//Forwarding with specified function 
Serial_forwardMessage(forward_buf, 3);
}

// Forwarding of messages Serial::Gateway::Serial0::hasID
void forward_Serial_Gateway_send_Serial0_hasID(struct Gateway_Instance *_instance, uint8_t id){
byte forward_buf[3];
forward_buf[0] = (50 >> 8) & 0xFF;
forward_buf[1] =  50 & 0xFF;


// parameter id
union u_id_t {
uint8_t p;
byte bytebuffer[1];
} u_id;
u_id.p = id;
forward_buf[2] =  (u_id.bytebuffer[0] & 0xFF);

//Forwarding with specified function 
Serial_forwardMessage(forward_buf, 3);
}

// Forwarding of messages Serial::Gateway::Serial0::iHaveID
void forward_Serial_Gateway_send_Serial0_iHaveID(struct Gateway_Instance *_instance, uint8_t id){
byte forward_buf[3];
forward_buf[0] = (51 >> 8) & 0xFF;
forward_buf[1] =  51 & 0xFF;


// parameter id
union u_id_t {
uint8_t p;
byte bytebuffer[1];
} u_id;
u_id.p = id;
forward_buf[2] =  (u_id.bytebuffer[0] & 0xFF);

//Forwarding with specified function 
Serial_forwardMessage(forward_buf, 3);
}

// Forwarding of messages Serial::Gateway::Serial0::mayIHaveID
void forward_Serial_Gateway_send_Serial0_mayIHaveID(struct Gateway_Instance *_instance, uint8_t id){
byte forward_buf[3];
forward_buf[0] = (52 >> 8) & 0xFF;
forward_buf[1] =  52 & 0xFF;


// parameter id
union u_id_t {
uint8_t p;
byte bytebuffer[1];
} u_id;
u_id.p = id;
forward_buf[2] =  (u_id.bytebuffer[0] & 0xFF);

//Forwarding with specified function 
Serial_forwardMessage(forward_buf, 3);
}

// Forwarding of messages Serial1::Gateway::Serial1::addHead
void forward_Serial1_Gateway_send_Serial1_addHead(struct Gateway_Instance *_instance, uint8_t x, uint8_t y, uint8_t id){
byte forward_buf[5];
forward_buf[0] = (55 >> 8) & 0xFF;
forward_buf[1] =  55 & 0xFF;


// parameter x
union u_x_t {
uint8_t p;
byte bytebuffer[1];
} u_x;
u_x.p = x;
forward_buf[2] =  (u_x.bytebuffer[0] & 0xFF);

// parameter y
union u_y_t {
uint8_t p;
byte bytebuffer[1];
} u_y;
u_y.p = y;
forward_buf[3] =  (u_y.bytebuffer[0] & 0xFF);

// parameter id
union u_id_t {
uint8_t p;
byte bytebuffer[1];
} u_id;
u_id.p = id;
forward_buf[4] =  (u_id.bytebuffer[0] & 0xFF);

//Forwarding with specified function 
Serial1_forwardMessage(forward_buf, 5);
}

// Forwarding of messages Serial1::Gateway::Serial1::loose
void forward_Serial1_Gateway_send_Serial1_loose(struct Gateway_Instance *_instance, uint8_t id){
byte forward_buf[3];
forward_buf[0] = (56 >> 8) & 0xFF;
forward_buf[1] =  56 & 0xFF;


// parameter id
union u_id_t {
uint8_t p;
byte bytebuffer[1];
} u_id;
u_id.p = id;
forward_buf[2] =  (u_id.bytebuffer[0] & 0xFF);

//Forwarding with specified function 
Serial1_forwardMessage(forward_buf, 3);
}

// Forwarding of messages Serial1::Gateway::Serial1::tronReady
void forward_Serial1_Gateway_send_Serial1_tronReady(struct Gateway_Instance *_instance, uint8_t id){
byte forward_buf[3];
forward_buf[0] = (53 >> 8) & 0xFF;
forward_buf[1] =  53 & 0xFF;


// parameter id
union u_id_t {
uint8_t p;
byte bytebuffer[1];
} u_id;
u_id.p = id;
forward_buf[2] =  (u_id.bytebuffer[0] & 0xFF);

//Forwarding with specified function 
Serial1_forwardMessage(forward_buf, 3);
}

// Forwarding of messages Serial1::Gateway::Serial1::tronGo
void forward_Serial1_Gateway_send_Serial1_tronGo(struct Gateway_Instance *_instance, uint8_t nbID){
byte forward_buf[3];
forward_buf[0] = (54 >> 8) & 0xFF;
forward_buf[1] =  54 & 0xFF;


// parameter nbID
union u_nbID_t {
uint8_t p;
byte bytebuffer[1];
} u_nbID;
u_nbID.p = nbID;
forward_buf[2] =  (u_nbID.bytebuffer[0] & 0xFF);

//Forwarding with specified function 
Serial1_forwardMessage(forward_buf, 3);
}

// Forwarding of messages Serial1::Gateway::Serial1::hasID
void forward_Serial1_Gateway_send_Serial1_hasID(struct Gateway_Instance *_instance, uint8_t id){
byte forward_buf[3];
forward_buf[0] = (50 >> 8) & 0xFF;
forward_buf[1] =  50 & 0xFF;


// parameter id
union u_id_t {
uint8_t p;
byte bytebuffer[1];
} u_id;
u_id.p = id;
forward_buf[2] =  (u_id.bytebuffer[0] & 0xFF);

//Forwarding with specified function 
Serial1_forwardMessage(forward_buf, 3);
}

// Forwarding of messages Serial1::Gateway::Serial1::iHaveID
void forward_Serial1_Gateway_send_Serial1_iHaveID(struct Gateway_Instance *_instance, uint8_t id){
byte forward_buf[3];
forward_buf[0] = (51 >> 8) & 0xFF;
forward_buf[1] =  51 & 0xFF;


// parameter id
union u_id_t {
uint8_t p;
byte bytebuffer[1];
} u_id;
u_id.p = id;
forward_buf[2] =  (u_id.bytebuffer[0] & 0xFF);

//Forwarding with specified function 
Serial1_forwardMessage(forward_buf, 3);
}

// Forwarding of messages Serial1::Gateway::Serial1::mayIHaveID
void forward_Serial1_Gateway_send_Serial1_mayIHaveID(struct Gateway_Instance *_instance, uint8_t id){
byte forward_buf[3];
forward_buf[0] = (52 >> 8) & 0xFF;
forward_buf[1] =  52 & 0xFF;


// parameter id
union u_id_t {
uint8_t p;
byte bytebuffer[1];
} u_id;
u_id.p = id;
forward_buf[2] =  (u_id.bytebuffer[0] & 0xFF);

//Forwarding with specified function 
Serial1_forwardMessage(forward_buf, 3);
}

// Forwarding of messages Serial2::Gateway::Serial2::addHead
void forward_Serial2_Gateway_send_Serial2_addHead(struct Gateway_Instance *_instance, uint8_t x, uint8_t y, uint8_t id){
byte forward_buf[5];
forward_buf[0] = (55 >> 8) & 0xFF;
forward_buf[1] =  55 & 0xFF;


// parameter x
union u_x_t {
uint8_t p;
byte bytebuffer[1];
} u_x;
u_x.p = x;
forward_buf[2] =  (u_x.bytebuffer[0] & 0xFF);

// parameter y
union u_y_t {
uint8_t p;
byte bytebuffer[1];
} u_y;
u_y.p = y;
forward_buf[3] =  (u_y.bytebuffer[0] & 0xFF);

// parameter id
union u_id_t {
uint8_t p;
byte bytebuffer[1];
} u_id;
u_id.p = id;
forward_buf[4] =  (u_id.bytebuffer[0] & 0xFF);

//Forwarding with specified function 
Serial2_forwardMessage(forward_buf, 5);
}

// Forwarding of messages Serial2::Gateway::Serial2::loose
void forward_Serial2_Gateway_send_Serial2_loose(struct Gateway_Instance *_instance, uint8_t id){
byte forward_buf[3];
forward_buf[0] = (56 >> 8) & 0xFF;
forward_buf[1] =  56 & 0xFF;


// parameter id
union u_id_t {
uint8_t p;
byte bytebuffer[1];
} u_id;
u_id.p = id;
forward_buf[2] =  (u_id.bytebuffer[0] & 0xFF);

//Forwarding with specified function 
Serial2_forwardMessage(forward_buf, 3);
}

// Forwarding of messages Serial2::Gateway::Serial2::tronReady
void forward_Serial2_Gateway_send_Serial2_tronReady(struct Gateway_Instance *_instance, uint8_t id){
byte forward_buf[3];
forward_buf[0] = (53 >> 8) & 0xFF;
forward_buf[1] =  53 & 0xFF;


// parameter id
union u_id_t {
uint8_t p;
byte bytebuffer[1];
} u_id;
u_id.p = id;
forward_buf[2] =  (u_id.bytebuffer[0] & 0xFF);

//Forwarding with specified function 
Serial2_forwardMessage(forward_buf, 3);
}

// Forwarding of messages Serial2::Gateway::Serial2::tronGo
void forward_Serial2_Gateway_send_Serial2_tronGo(struct Gateway_Instance *_instance, uint8_t nbID){
byte forward_buf[3];
forward_buf[0] = (54 >> 8) & 0xFF;
forward_buf[1] =  54 & 0xFF;


// parameter nbID
union u_nbID_t {
uint8_t p;
byte bytebuffer[1];
} u_nbID;
u_nbID.p = nbID;
forward_buf[2] =  (u_nbID.bytebuffer[0] & 0xFF);

//Forwarding with specified function 
Serial2_forwardMessage(forward_buf, 3);
}

// Forwarding of messages Serial2::Gateway::Serial2::hasID
void forward_Serial2_Gateway_send_Serial2_hasID(struct Gateway_Instance *_instance, uint8_t id){
byte forward_buf[3];
forward_buf[0] = (50 >> 8) & 0xFF;
forward_buf[1] =  50 & 0xFF;


// parameter id
union u_id_t {
uint8_t p;
byte bytebuffer[1];
} u_id;
u_id.p = id;
forward_buf[2] =  (u_id.bytebuffer[0] & 0xFF);

//Forwarding with specified function 
Serial2_forwardMessage(forward_buf, 3);
}

// Forwarding of messages Serial2::Gateway::Serial2::iHaveID
void forward_Serial2_Gateway_send_Serial2_iHaveID(struct Gateway_Instance *_instance, uint8_t id){
byte forward_buf[3];
forward_buf[0] = (51 >> 8) & 0xFF;
forward_buf[1] =  51 & 0xFF;


// parameter id
union u_id_t {
uint8_t p;
byte bytebuffer[1];
} u_id;
u_id.p = id;
forward_buf[2] =  (u_id.bytebuffer[0] & 0xFF);

//Forwarding with specified function 
Serial2_forwardMessage(forward_buf, 3);
}

// Forwarding of messages Serial2::Gateway::Serial2::mayIHaveID
void forward_Serial2_Gateway_send_Serial2_mayIHaveID(struct Gateway_Instance *_instance, uint8_t id){
byte forward_buf[3];
forward_buf[0] = (52 >> 8) & 0xFF;
forward_buf[1] =  52 & 0xFF;


// parameter id
union u_id_t {
uint8_t p;
byte bytebuffer[1];
} u_id;
u_id.p = id;
forward_buf[2] =  (u_id.bytebuffer[0] & 0xFF);

//Forwarding with specified function 
Serial2_forwardMessage(forward_buf, 3);
}

// Forwarding of messages Serial3::Gateway::Serial3::addHead
void forward_Serial3_Gateway_send_Serial3_addHead(struct Gateway_Instance *_instance, uint8_t x, uint8_t y, uint8_t id){
byte forward_buf[5];
forward_buf[0] = (55 >> 8) & 0xFF;
forward_buf[1] =  55 & 0xFF;


// parameter x
union u_x_t {
uint8_t p;
byte bytebuffer[1];
} u_x;
u_x.p = x;
forward_buf[2] =  (u_x.bytebuffer[0] & 0xFF);

// parameter y
union u_y_t {
uint8_t p;
byte bytebuffer[1];
} u_y;
u_y.p = y;
forward_buf[3] =  (u_y.bytebuffer[0] & 0xFF);

// parameter id
union u_id_t {
uint8_t p;
byte bytebuffer[1];
} u_id;
u_id.p = id;
forward_buf[4] =  (u_id.bytebuffer[0] & 0xFF);

//Forwarding with specified function 
Serial3_forwardMessage(forward_buf, 5);
}

// Forwarding of messages Serial3::Gateway::Serial3::loose
void forward_Serial3_Gateway_send_Serial3_loose(struct Gateway_Instance *_instance, uint8_t id){
byte forward_buf[3];
forward_buf[0] = (56 >> 8) & 0xFF;
forward_buf[1] =  56 & 0xFF;


// parameter id
union u_id_t {
uint8_t p;
byte bytebuffer[1];
} u_id;
u_id.p = id;
forward_buf[2] =  (u_id.bytebuffer[0] & 0xFF);

//Forwarding with specified function 
Serial3_forwardMessage(forward_buf, 3);
}

// Forwarding of messages Serial3::Gateway::Serial3::tronReady
void forward_Serial3_Gateway_send_Serial3_tronReady(struct Gateway_Instance *_instance, uint8_t id){
byte forward_buf[3];
forward_buf[0] = (53 >> 8) & 0xFF;
forward_buf[1] =  53 & 0xFF;


// parameter id
union u_id_t {
uint8_t p;
byte bytebuffer[1];
} u_id;
u_id.p = id;
forward_buf[2] =  (u_id.bytebuffer[0] & 0xFF);

//Forwarding with specified function 
Serial3_forwardMessage(forward_buf, 3);
}

// Forwarding of messages Serial3::Gateway::Serial3::tronGo
void forward_Serial3_Gateway_send_Serial3_tronGo(struct Gateway_Instance *_instance, uint8_t nbID){
byte forward_buf[3];
forward_buf[0] = (54 >> 8) & 0xFF;
forward_buf[1] =  54 & 0xFF;


// parameter nbID
union u_nbID_t {
uint8_t p;
byte bytebuffer[1];
} u_nbID;
u_nbID.p = nbID;
forward_buf[2] =  (u_nbID.bytebuffer[0] & 0xFF);

//Forwarding with specified function 
Serial3_forwardMessage(forward_buf, 3);
}

// Forwarding of messages Serial3::Gateway::Serial3::hasID
void forward_Serial3_Gateway_send_Serial3_hasID(struct Gateway_Instance *_instance, uint8_t id){
byte forward_buf[3];
forward_buf[0] = (50 >> 8) & 0xFF;
forward_buf[1] =  50 & 0xFF;


// parameter id
union u_id_t {
uint8_t p;
byte bytebuffer[1];
} u_id;
u_id.p = id;
forward_buf[2] =  (u_id.bytebuffer[0] & 0xFF);

//Forwarding with specified function 
Serial3_forwardMessage(forward_buf, 3);
}

// Forwarding of messages Serial3::Gateway::Serial3::iHaveID
void forward_Serial3_Gateway_send_Serial3_iHaveID(struct Gateway_Instance *_instance, uint8_t id){
byte forward_buf[3];
forward_buf[0] = (51 >> 8) & 0xFF;
forward_buf[1] =  51 & 0xFF;


// parameter id
union u_id_t {
uint8_t p;
byte bytebuffer[1];
} u_id;
u_id.p = id;
forward_buf[2] =  (u_id.bytebuffer[0] & 0xFF);

//Forwarding with specified function 
Serial3_forwardMessage(forward_buf, 3);
}

// Forwarding of messages Serial3::Gateway::Serial3::mayIHaveID
void forward_Serial3_Gateway_send_Serial3_mayIHaveID(struct Gateway_Instance *_instance, uint8_t id){
byte forward_buf[3];
forward_buf[0] = (52 >> 8) & 0xFF;
forward_buf[1] =  52 & 0xFF;


// parameter id
union u_id_t {
uint8_t p;
byte bytebuffer[1];
} u_id;
u_id.p = id;
forward_buf[2] =  (u_id.bytebuffer[0] & 0xFF);

//Forwarding with specified function 
Serial3_forwardMessage(forward_buf, 3);
}

void forward_Gateway_send_Serial1_mayIHaveID(struct Gateway_Instance *_instance, uint8_t id){
if(_instance->id_Serial1 == GatewayCfg_g_var.id_Serial1) {
forward_Serial1_Gateway_send_Serial1_mayIHaveID(_instance, id);
}
}
void forward_Gateway_send_Serial3_mayIHaveID(struct Gateway_Instance *_instance, uint8_t id){
if(_instance->id_Serial3 == GatewayCfg_g_var.id_Serial3) {
forward_Serial3_Gateway_send_Serial3_mayIHaveID(_instance, id);
}
}
void forward_Gateway_send_Serial2_mayIHaveID(struct Gateway_Instance *_instance, uint8_t id){
if(_instance->id_Serial2 == GatewayCfg_g_var.id_Serial2) {
forward_Serial2_Gateway_send_Serial2_mayIHaveID(_instance, id);
}
}
void forward_Gateway_send_Serial0_mayIHaveID(struct Gateway_Instance *_instance, uint8_t id){
if(_instance->id_Serial0 == GatewayCfg_g_var.id_Serial0) {
forward_Serial_Gateway_send_Serial0_mayIHaveID(_instance, id);
}
}
void forward_Gateway_send_Serial1_tronGo(struct Gateway_Instance *_instance, uint8_t nbID){
if(_instance->id_Serial1 == GatewayCfg_g_var.id_Serial1) {
forward_Serial1_Gateway_send_Serial1_tronGo(_instance, nbID);
}
}
void forward_Gateway_send_Serial3_tronGo(struct Gateway_Instance *_instance, uint8_t nbID){
if(_instance->id_Serial3 == GatewayCfg_g_var.id_Serial3) {
forward_Serial3_Gateway_send_Serial3_tronGo(_instance, nbID);
}
}
void forward_Gateway_send_Serial2_tronGo(struct Gateway_Instance *_instance, uint8_t nbID){
if(_instance->id_Serial2 == GatewayCfg_g_var.id_Serial2) {
forward_Serial2_Gateway_send_Serial2_tronGo(_instance, nbID);
}
}
void forward_Gateway_send_Serial0_tronGo(struct Gateway_Instance *_instance, uint8_t nbID){
if(_instance->id_Serial0 == GatewayCfg_g_var.id_Serial0) {
forward_Serial_Gateway_send_Serial0_tronGo(_instance, nbID);
}
}
void forward_Gateway_send_Serial1_loose(struct Gateway_Instance *_instance, uint8_t id){
if(_instance->id_Serial1 == GatewayCfg_g_var.id_Serial1) {
forward_Serial1_Gateway_send_Serial1_loose(_instance, id);
}
}
void forward_Gateway_send_Serial3_loose(struct Gateway_Instance *_instance, uint8_t id){
if(_instance->id_Serial3 == GatewayCfg_g_var.id_Serial3) {
forward_Serial3_Gateway_send_Serial3_loose(_instance, id);
}
}
void forward_Gateway_send_Serial2_loose(struct Gateway_Instance *_instance, uint8_t id){
if(_instance->id_Serial2 == GatewayCfg_g_var.id_Serial2) {
forward_Serial2_Gateway_send_Serial2_loose(_instance, id);
}
}
void forward_Gateway_send_Serial0_loose(struct Gateway_Instance *_instance, uint8_t id){
if(_instance->id_Serial0 == GatewayCfg_g_var.id_Serial0) {
forward_Serial_Gateway_send_Serial0_loose(_instance, id);
}
}
void forward_Gateway_send_Serial1_hasID(struct Gateway_Instance *_instance, uint8_t id){
if(_instance->id_Serial1 == GatewayCfg_g_var.id_Serial1) {
forward_Serial1_Gateway_send_Serial1_hasID(_instance, id);
}
}
void forward_Gateway_send_Serial3_hasID(struct Gateway_Instance *_instance, uint8_t id){
if(_instance->id_Serial3 == GatewayCfg_g_var.id_Serial3) {
forward_Serial3_Gateway_send_Serial3_hasID(_instance, id);
}
}
void forward_Gateway_send_Serial2_hasID(struct Gateway_Instance *_instance, uint8_t id){
if(_instance->id_Serial2 == GatewayCfg_g_var.id_Serial2) {
forward_Serial2_Gateway_send_Serial2_hasID(_instance, id);
}
}
void forward_Gateway_send_Serial0_hasID(struct Gateway_Instance *_instance, uint8_t id){
if(_instance->id_Serial0 == GatewayCfg_g_var.id_Serial0) {
forward_Serial_Gateway_send_Serial0_hasID(_instance, id);
}
}
void forward_Gateway_send_Serial1_iHaveID(struct Gateway_Instance *_instance, uint8_t id){
if(_instance->id_Serial1 == GatewayCfg_g_var.id_Serial1) {
forward_Serial1_Gateway_send_Serial1_iHaveID(_instance, id);
}
}
void forward_Gateway_send_Serial3_iHaveID(struct Gateway_Instance *_instance, uint8_t id){
if(_instance->id_Serial3 == GatewayCfg_g_var.id_Serial3) {
forward_Serial3_Gateway_send_Serial3_iHaveID(_instance, id);
}
}
void forward_Gateway_send_Serial2_iHaveID(struct Gateway_Instance *_instance, uint8_t id){
if(_instance->id_Serial2 == GatewayCfg_g_var.id_Serial2) {
forward_Serial2_Gateway_send_Serial2_iHaveID(_instance, id);
}
}
void forward_Gateway_send_Serial0_iHaveID(struct Gateway_Instance *_instance, uint8_t id){
if(_instance->id_Serial0 == GatewayCfg_g_var.id_Serial0) {
forward_Serial_Gateway_send_Serial0_iHaveID(_instance, id);
}
}
void forward_Gateway_send_Serial1_tronReady(struct Gateway_Instance *_instance, uint8_t id){
if(_instance->id_Serial1 == GatewayCfg_g_var.id_Serial1) {
forward_Serial1_Gateway_send_Serial1_tronReady(_instance, id);
}
}
void forward_Gateway_send_Serial3_tronReady(struct Gateway_Instance *_instance, uint8_t id){
if(_instance->id_Serial3 == GatewayCfg_g_var.id_Serial3) {
forward_Serial3_Gateway_send_Serial3_tronReady(_instance, id);
}
}
void forward_Gateway_send_Serial2_tronReady(struct Gateway_Instance *_instance, uint8_t id){
if(_instance->id_Serial2 == GatewayCfg_g_var.id_Serial2) {
forward_Serial2_Gateway_send_Serial2_tronReady(_instance, id);
}
}
void forward_Gateway_send_Serial0_tronReady(struct Gateway_Instance *_instance, uint8_t id){
if(_instance->id_Serial0 == GatewayCfg_g_var.id_Serial0) {
forward_Serial_Gateway_send_Serial0_tronReady(_instance, id);
}
}
void forward_Gateway_send_Serial1_addHead(struct Gateway_Instance *_instance, uint8_t x, uint8_t y, uint8_t id){
if(_instance->id_Serial1 == GatewayCfg_g_var.id_Serial1) {
forward_Serial1_Gateway_send_Serial1_addHead(_instance, x, y, id);
}
}
void forward_Gateway_send_Serial3_addHead(struct Gateway_Instance *_instance, uint8_t x, uint8_t y, uint8_t id){
if(_instance->id_Serial3 == GatewayCfg_g_var.id_Serial3) {
forward_Serial3_Gateway_send_Serial3_addHead(_instance, x, y, id);
}
}
void forward_Gateway_send_Serial2_addHead(struct Gateway_Instance *_instance, uint8_t x, uint8_t y, uint8_t id){
if(_instance->id_Serial2 == GatewayCfg_g_var.id_Serial2) {
forward_Serial2_Gateway_send_Serial2_addHead(_instance, x, y, id);
}
}
void forward_Gateway_send_Serial0_addHead(struct Gateway_Instance *_instance, uint8_t x, uint8_t y, uint8_t id){
if(_instance->id_Serial0 == GatewayCfg_g_var.id_Serial0) {
forward_Serial_Gateway_send_Serial0_addHead(_instance, x, y, id);
}
}

//external Message enqueue
void externalMessageEnqueue(uint8_t * msg, uint8_t msgSize, uint16_t listener_id) {
if ((msgSize >= 2) && (msg != NULL)) {
uint8_t msgSizeOK = 0;
switch(msg[0] * 256 + msg[1]) {
case 52:
if(msgSize == 3) {
msgSizeOK = 1;}
break;
case 54:
if(msgSize == 3) {
msgSizeOK = 1;}
break;
case 56:
if(msgSize == 3) {
msgSizeOK = 1;}
break;
case 50:
if(msgSize == 3) {
msgSizeOK = 1;}
break;
case 51:
if(msgSize == 3) {
msgSizeOK = 1;}
break;
case 53:
if(msgSize == 3) {
msgSizeOK = 1;}
break;
case 55:
if(msgSize == 5) {
msgSizeOK = 1;}
break;
}

if(msgSizeOK == 1) {
if ( fifo_byte_available() > (msgSize + 2) ) {
	uint8_t i;
	for (i = 0; i < 2; i++) {
		_fifo_enqueue(msg[i]);
	}
	_fifo_enqueue((listener_id >> 8) & 0xFF);
	_fifo_enqueue(listener_id & 0xFF);
	for (i = 2; i < msgSize; i++) {
		_fifo_enqueue(msg[i]);
	}
}
}
}
}

void initialize_configuration_GatewayCfg() {
// Initialize connectors
register_external_Gateway_send_Serial0_addHead_listener(forward_Gateway_send_Serial0_addHead);
register_external_Gateway_send_Serial0_loose_listener(forward_Gateway_send_Serial0_loose);
register_external_Gateway_send_Serial0_tronReady_listener(forward_Gateway_send_Serial0_tronReady);
register_external_Gateway_send_Serial0_tronGo_listener(forward_Gateway_send_Serial0_tronGo);
register_external_Gateway_send_Serial0_hasID_listener(forward_Gateway_send_Serial0_hasID);
register_external_Gateway_send_Serial0_iHaveID_listener(forward_Gateway_send_Serial0_iHaveID);
register_external_Gateway_send_Serial0_mayIHaveID_listener(forward_Gateway_send_Serial0_mayIHaveID);
register_external_Gateway_send_Serial1_addHead_listener(forward_Gateway_send_Serial1_addHead);
register_external_Gateway_send_Serial1_loose_listener(forward_Gateway_send_Serial1_loose);
register_external_Gateway_send_Serial1_tronReady_listener(forward_Gateway_send_Serial1_tronReady);
register_external_Gateway_send_Serial1_tronGo_listener(forward_Gateway_send_Serial1_tronGo);
register_external_Gateway_send_Serial1_hasID_listener(forward_Gateway_send_Serial1_hasID);
register_external_Gateway_send_Serial1_iHaveID_listener(forward_Gateway_send_Serial1_iHaveID);
register_external_Gateway_send_Serial1_mayIHaveID_listener(forward_Gateway_send_Serial1_mayIHaveID);
register_external_Gateway_send_Serial2_addHead_listener(forward_Gateway_send_Serial2_addHead);
register_external_Gateway_send_Serial2_loose_listener(forward_Gateway_send_Serial2_loose);
register_external_Gateway_send_Serial2_tronReady_listener(forward_Gateway_send_Serial2_tronReady);
register_external_Gateway_send_Serial2_tronGo_listener(forward_Gateway_send_Serial2_tronGo);
register_external_Gateway_send_Serial2_hasID_listener(forward_Gateway_send_Serial2_hasID);
register_external_Gateway_send_Serial2_iHaveID_listener(forward_Gateway_send_Serial2_iHaveID);
register_external_Gateway_send_Serial2_mayIHaveID_listener(forward_Gateway_send_Serial2_mayIHaveID);
register_external_Gateway_send_Serial3_addHead_listener(forward_Gateway_send_Serial3_addHead);
register_external_Gateway_send_Serial3_loose_listener(forward_Gateway_send_Serial3_loose);
register_external_Gateway_send_Serial3_tronReady_listener(forward_Gateway_send_Serial3_tronReady);
register_external_Gateway_send_Serial3_tronGo_listener(forward_Gateway_send_Serial3_tronGo);
register_external_Gateway_send_Serial3_hasID_listener(forward_Gateway_send_Serial3_hasID);
register_external_Gateway_send_Serial3_iHaveID_listener(forward_Gateway_send_Serial3_iHaveID);
register_external_Gateway_send_Serial3_mayIHaveID_listener(forward_Gateway_send_Serial3_mayIHaveID);

// Init the ID, state variables and properties for instance GatewayCfg_g
GatewayCfg_g_var.id_Serial0 = add_instance( (void*) &GatewayCfg_g_var);
GatewayCfg_g_Serial0_msgs[0] = 55;
GatewayCfg_g_Serial0_handlers_tab[0] = (void*) &Gateway_handle_Serial0_addHead;
GatewayCfg_g_Serial0_msgs[1] = 56;
GatewayCfg_g_Serial0_handlers_tab[1] = (void*) &Gateway_handle_Serial0_loose;
GatewayCfg_g_Serial0_msgs[2] = 53;
GatewayCfg_g_Serial0_handlers_tab[2] = (void*) &Gateway_handle_Serial0_tronReady;
GatewayCfg_g_Serial0_msgs[3] = 54;
GatewayCfg_g_Serial0_handlers_tab[3] = (void*) &Gateway_handle_Serial0_tronGo;
GatewayCfg_g_Serial0_msgs[4] = 50;
GatewayCfg_g_Serial0_handlers_tab[4] = (void*) &Gateway_handle_Serial0_hasID;
GatewayCfg_g_Serial0_msgs[5] = 51;
GatewayCfg_g_Serial0_handlers_tab[5] = (void*) &Gateway_handle_Serial0_iHaveID;
GatewayCfg_g_Serial0_msgs[6] = 52;
GatewayCfg_g_Serial0_handlers_tab[6] = (void*) &Gateway_handle_Serial0_mayIHaveID;
GatewayCfg_g_Serial0_handlers.nb_msg = 7;
GatewayCfg_g_Serial0_handlers.msg = (uint16_t *) &GatewayCfg_g_Serial0_msgs;
GatewayCfg_g_Serial0_handlers.msg_handler = (void **) &GatewayCfg_g_Serial0_handlers_tab;
GatewayCfg_g_Serial0_handlers.instance = &GatewayCfg_g_var;
GatewayCfg_g_var.Serial0_handlers = &GatewayCfg_g_Serial0_handlers;
GatewayCfg_g_var.Serial0_receiver_list_head = NULL;
GatewayCfg_g_var.Serial0_receiver_list_tail = &GatewayCfg_receivers[0];
GatewayCfg_g_var.id_Serial1 = add_instance( (void*) &GatewayCfg_g_var);
GatewayCfg_g_Serial1_msgs[0] = 55;
GatewayCfg_g_Serial1_handlers_tab[0] = (void*) &Gateway_handle_Serial1_addHead;
GatewayCfg_g_Serial1_msgs[1] = 56;
GatewayCfg_g_Serial1_handlers_tab[1] = (void*) &Gateway_handle_Serial1_loose;
GatewayCfg_g_Serial1_msgs[2] = 53;
GatewayCfg_g_Serial1_handlers_tab[2] = (void*) &Gateway_handle_Serial1_tronReady;
GatewayCfg_g_Serial1_msgs[3] = 54;
GatewayCfg_g_Serial1_handlers_tab[3] = (void*) &Gateway_handle_Serial1_tronGo;
GatewayCfg_g_Serial1_msgs[4] = 50;
GatewayCfg_g_Serial1_handlers_tab[4] = (void*) &Gateway_handle_Serial1_hasID;
GatewayCfg_g_Serial1_msgs[5] = 51;
GatewayCfg_g_Serial1_handlers_tab[5] = (void*) &Gateway_handle_Serial1_iHaveID;
GatewayCfg_g_Serial1_msgs[6] = 52;
GatewayCfg_g_Serial1_handlers_tab[6] = (void*) &Gateway_handle_Serial1_mayIHaveID;
GatewayCfg_g_Serial1_handlers.nb_msg = 7;
GatewayCfg_g_Serial1_handlers.msg = (uint16_t *) &GatewayCfg_g_Serial1_msgs;
GatewayCfg_g_Serial1_handlers.msg_handler = (void **) &GatewayCfg_g_Serial1_handlers_tab;
GatewayCfg_g_Serial1_handlers.instance = &GatewayCfg_g_var;
GatewayCfg_g_var.Serial1_handlers = &GatewayCfg_g_Serial1_handlers;
GatewayCfg_g_var.Serial1_receiver_list_head = NULL;
GatewayCfg_g_var.Serial1_receiver_list_tail = &GatewayCfg_receivers[0];
GatewayCfg_g_var.id_Serial2 = add_instance( (void*) &GatewayCfg_g_var);
GatewayCfg_g_Serial2_msgs[0] = 55;
GatewayCfg_g_Serial2_handlers_tab[0] = (void*) &Gateway_handle_Serial2_addHead;
GatewayCfg_g_Serial2_msgs[1] = 56;
GatewayCfg_g_Serial2_handlers_tab[1] = (void*) &Gateway_handle_Serial2_loose;
GatewayCfg_g_Serial2_msgs[2] = 53;
GatewayCfg_g_Serial2_handlers_tab[2] = (void*) &Gateway_handle_Serial2_tronReady;
GatewayCfg_g_Serial2_msgs[3] = 54;
GatewayCfg_g_Serial2_handlers_tab[3] = (void*) &Gateway_handle_Serial2_tronGo;
GatewayCfg_g_Serial2_msgs[4] = 50;
GatewayCfg_g_Serial2_handlers_tab[4] = (void*) &Gateway_handle_Serial2_hasID;
GatewayCfg_g_Serial2_msgs[5] = 51;
GatewayCfg_g_Serial2_handlers_tab[5] = (void*) &Gateway_handle_Serial2_iHaveID;
GatewayCfg_g_Serial2_msgs[6] = 52;
GatewayCfg_g_Serial2_handlers_tab[6] = (void*) &Gateway_handle_Serial2_mayIHaveID;
GatewayCfg_g_Serial2_handlers.nb_msg = 7;
GatewayCfg_g_Serial2_handlers.msg = (uint16_t *) &GatewayCfg_g_Serial2_msgs;
GatewayCfg_g_Serial2_handlers.msg_handler = (void **) &GatewayCfg_g_Serial2_handlers_tab;
GatewayCfg_g_Serial2_handlers.instance = &GatewayCfg_g_var;
GatewayCfg_g_var.Serial2_handlers = &GatewayCfg_g_Serial2_handlers;
GatewayCfg_g_var.Serial2_receiver_list_head = NULL;
GatewayCfg_g_var.Serial2_receiver_list_tail = &GatewayCfg_receivers[0];
GatewayCfg_g_var.id_Serial3 = add_instance( (void*) &GatewayCfg_g_var);
GatewayCfg_g_Serial3_msgs[0] = 55;
GatewayCfg_g_Serial3_handlers_tab[0] = (void*) &Gateway_handle_Serial3_addHead;
GatewayCfg_g_Serial3_msgs[1] = 56;
GatewayCfg_g_Serial3_handlers_tab[1] = (void*) &Gateway_handle_Serial3_loose;
GatewayCfg_g_Serial3_msgs[2] = 53;
GatewayCfg_g_Serial3_handlers_tab[2] = (void*) &Gateway_handle_Serial3_tronReady;
GatewayCfg_g_Serial3_msgs[3] = 54;
GatewayCfg_g_Serial3_handlers_tab[3] = (void*) &Gateway_handle_Serial3_tronGo;
GatewayCfg_g_Serial3_msgs[4] = 50;
GatewayCfg_g_Serial3_handlers_tab[4] = (void*) &Gateway_handle_Serial3_hasID;
GatewayCfg_g_Serial3_msgs[5] = 51;
GatewayCfg_g_Serial3_handlers_tab[5] = (void*) &Gateway_handle_Serial3_iHaveID;
GatewayCfg_g_Serial3_msgs[6] = 52;
GatewayCfg_g_Serial3_handlers_tab[6] = (void*) &Gateway_handle_Serial3_mayIHaveID;
GatewayCfg_g_Serial3_handlers.nb_msg = 7;
GatewayCfg_g_Serial3_handlers.msg = (uint16_t *) &GatewayCfg_g_Serial3_msgs;
GatewayCfg_g_Serial3_handlers.msg_handler = (void **) &GatewayCfg_g_Serial3_handlers_tab;
GatewayCfg_g_Serial3_handlers.instance = &GatewayCfg_g_var;
GatewayCfg_g_var.Serial3_handlers = &GatewayCfg_g_Serial3_handlers;
GatewayCfg_g_var.Serial3_receiver_list_head = NULL;
GatewayCfg_g_var.Serial3_receiver_list_tail = &GatewayCfg_receivers[0];
GatewayCfg_g_var.Gateway_GatewayChart_State = GATEWAY_GATEWAYCHART_ACTIVE_STATE;

// Init the ID, state variables and properties for external connector Serial
Serial_instance.listener_id = add_instance( (void*) &Serial_instance);
GatewayCfg_receivers[0] = &GatewayCfg_g_Serial0_handlers;
Serial_instance.Serial0_receiver_list_head = &GatewayCfg_receivers[0];
Serial_instance.Serial0_receiver_list_tail = &GatewayCfg_receivers[0];
// Init the ID, state variables and properties for external connector Serial1
Serial1_instance.listener_id = add_instance( (void*) &Serial1_instance);
GatewayCfg_receivers[1] = &GatewayCfg_g_Serial1_handlers;
Serial1_instance.Serial1_receiver_list_head = &GatewayCfg_receivers[1];
Serial1_instance.Serial1_receiver_list_tail = &GatewayCfg_receivers[1];
// Init the ID, state variables and properties for external connector Serial2
Serial2_instance.listener_id = add_instance( (void*) &Serial2_instance);
GatewayCfg_receivers[2] = &GatewayCfg_g_Serial2_handlers;
Serial2_instance.Serial2_receiver_list_head = &GatewayCfg_receivers[2];
Serial2_instance.Serial2_receiver_list_tail = &GatewayCfg_receivers[2];
// Init the ID, state variables and properties for external connector Serial3
Serial3_instance.listener_id = add_instance( (void*) &Serial3_instance);
GatewayCfg_receivers[3] = &GatewayCfg_g_Serial3_handlers;
Serial3_instance.Serial3_receiver_list_head = &GatewayCfg_receivers[3];
Serial3_instance.Serial3_receiver_list_tail = &GatewayCfg_receivers[3];

// Network Initilization 
//Serial:
Serial_setup();
//Serial1:
Serial1_setup();
//Serial2:
Serial2_setup();
//Serial3:
Serial3_setup();


// End Network Initilization 

Gateway_GatewayChart_OnEntry(GATEWAY_GATEWAYCHART_STATE, &GatewayCfg_g_var);
}




void setup() {
initialize_configuration_GatewayCfg();

}

void loop() {

// Network Listener
Serial_read();
Serial1_read();
Serial2_read();
Serial3_read();

    processMessageQueue();
}
