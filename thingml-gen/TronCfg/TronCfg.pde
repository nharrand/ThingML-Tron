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
 * Headers for type : Adafruit_1_8pLCDShieldShield
 *****************************************************************************/


// BEGIN: Code from the c_header annotation Adafruit_1_8pLCDShieldShield

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <SPI.h>
#define TFT_CS 10
#define TFT_RST 9
#define TFT_DC 8
#define TFT_SCLK 13
#define TFT_MOSI 11
// END: Code from the c_header annotation Adafruit_1_8pLCDShieldShield

// Definition of the instance stuct:
struct Adafruit_1_8pLCDShieldShield_Instance {
// Variables for the ID of the ports of the instance
uint16_t id_button;
// Pointer to receiver list
struct Msg_Handler ** button_receiver_list_head;
struct Msg_Handler ** button_receiver_list_tail;
// Handler Array
struct Msg_Handler * button_handlers;
uint16_t id_lcd;
// Pointer to receiver list
struct Msg_Handler ** lcd_receiver_list_head;
struct Msg_Handler ** lcd_receiver_list_tail;
// Handler Array
struct Msg_Handler * lcd_handlers;
uint16_t id_arduino;
// Pointer to receiver list
struct Msg_Handler ** arduino_receiver_list_head;
struct Msg_Handler ** arduino_receiver_list_tail;
// Handler Array
struct Msg_Handler * arduino_handlers;
// Variables for the current instance state
int Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State;
// Variables for the properties of the instance
uint8_t Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_bpin__var;
uint8_t Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_Idle_bstate__var;

};
// Declaration of prototypes outgoing messages:
void Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_OnEntry(int state, struct Adafruit_1_8pLCDShieldShield_Instance *_instance);
void Adafruit_1_8pLCDShieldShield_handle_button_button_state(struct Adafruit_1_8pLCDShieldShield_Instance *_instance);
void Adafruit_1_8pLCDShieldShield_handle_arduino_ready(struct Adafruit_1_8pLCDShieldShield_Instance *_instance);
void Adafruit_1_8pLCDShieldShield_handle_arduino_100ms_interrupt(struct Adafruit_1_8pLCDShieldShield_Instance *_instance);
void Adafruit_1_8pLCDShieldShield_handle_lcd_print_dec(struct Adafruit_1_8pLCDShieldShield_Instance *_instance, double num);
void Adafruit_1_8pLCDShieldShield_handle_lcd_drawRect(struct Adafruit_1_8pLCDShieldShield_Instance *_instance, uint16_t x, uint16_t y, uint16_t w, uint16_t l, uint16_t col);
void Adafruit_1_8pLCDShieldShield_handle_lcd_set_cursor(struct Adafruit_1_8pLCDShieldShield_Instance *_instance, uint8_t c, uint8_t l);
void Adafruit_1_8pLCDShieldShield_handle_lcd_fillRect(struct Adafruit_1_8pLCDShieldShield_Instance *_instance, uint16_t x, uint16_t y, uint16_t w, uint16_t l, uint16_t col);
void Adafruit_1_8pLCDShieldShield_handle_lcd_print_num(struct Adafruit_1_8pLCDShieldShield_Instance *_instance, int16_t num);
void Adafruit_1_8pLCDShieldShield_handle_lcd_clear(struct Adafruit_1_8pLCDShieldShield_Instance *_instance);
void Adafruit_1_8pLCDShieldShield_handle_lcd_print_str(struct Adafruit_1_8pLCDShieldShield_Instance *_instance, char * msg);
// Declaration of callbacks for incoming messages:
void register_Adafruit_1_8pLCDShieldShield_send_button_button_state_response_listener(void (*_listener)(struct Adafruit_1_8pLCDShieldShield_Instance *, uint8_t));
void register_external_Adafruit_1_8pLCDShieldShield_send_button_button_state_response_listener(void (*_listener)(struct Adafruit_1_8pLCDShieldShield_Instance *, uint8_t));
void register_Adafruit_1_8pLCDShieldShield_send_button_button_state_change_listener(void (*_listener)(struct Adafruit_1_8pLCDShieldShield_Instance *, uint8_t));
void register_external_Adafruit_1_8pLCDShieldShield_send_button_button_state_change_listener(void (*_listener)(struct Adafruit_1_8pLCDShieldShield_Instance *, uint8_t));
void register_Adafruit_1_8pLCDShieldShield_send_lcd_LCDready_listener(void (*_listener)(struct Adafruit_1_8pLCDShieldShield_Instance *));
void register_external_Adafruit_1_8pLCDShieldShield_send_lcd_LCDready_listener(void (*_listener)(struct Adafruit_1_8pLCDShieldShield_Instance *));
void register_Adafruit_1_8pLCDShieldShield_send_arduino_timer_start_listener(void (*_listener)(struct Adafruit_1_8pLCDShieldShield_Instance *, uint8_t, int16_t));
void register_external_Adafruit_1_8pLCDShieldShield_send_arduino_timer_start_listener(void (*_listener)(struct Adafruit_1_8pLCDShieldShield_Instance *, uint8_t, int16_t));
void register_Adafruit_1_8pLCDShieldShield_send_arduino_timer_cancel_listener(void (*_listener)(struct Adafruit_1_8pLCDShieldShield_Instance *, uint8_t));
void register_external_Adafruit_1_8pLCDShieldShield_send_arduino_timer_cancel_listener(void (*_listener)(struct Adafruit_1_8pLCDShieldShield_Instance *, uint8_t));

// Definition of the states:
#define ADAFRUIT_1_8PLCDSHIELDSHIELD_RGBLCDSHIELDSM_STATE 0
#define ADAFRUIT_1_8PLCDSHIELDSHIELD_RGBLCDSHIELDSM_EMPTY_STATE 1
#define ADAFRUIT_1_8PLCDSHIELDSHIELD_RGBLCDSHIELDSM_SETUP_STATE 2
#define ADAFRUIT_1_8PLCDSHIELDSHIELD_RGBLCDSHIELDSM_IDLE_STATE 3


/*****************************************************************************
 * Headers for type : Tron
 *****************************************************************************/


// BEGIN: Code from the c_header annotation Tron

#define _SNAKE_TAB_SIZE 273 // not 2184 / 8 
#include <SoftwareSerial.h> 
// END: Code from the c_header annotation Tron

// Definition of the instance stuct:
struct Tron_Instance {
// Variables for the ID of the ports of the instance
uint16_t id_TronPort;
// Pointer to receiver list
struct Msg_Handler ** TronPort_receiver_list_head;
struct Msg_Handler ** TronPort_receiver_list_tail;
// Handler Array
struct Msg_Handler * TronPort_handlers;
uint16_t id_button;
// Pointer to receiver list
struct Msg_Handler ** button_receiver_list_head;
struct Msg_Handler ** button_receiver_list_tail;
// Handler Array
struct Msg_Handler * button_handlers;
uint16_t id_lcd;
// Pointer to receiver list
struct Msg_Handler ** lcd_receiver_list_head;
struct Msg_Handler ** lcd_receiver_list_tail;
// Handler Array
struct Msg_Handler * lcd_handlers;
uint16_t id_arduino;
// Pointer to receiver list
struct Msg_Handler ** arduino_receiver_list_head;
struct Msg_Handler ** arduino_receiver_list_tail;
// Handler Array
struct Msg_Handler * arduino_handlers;
// Variables for the current instance state
int Tron_TronStateChart_State;
// Variables for the properties of the instance
uint8_t Tron_myID__var;
uint8_t Tron_nbID__var;
uint8_t Tron_curID__var;
uint8_t Tron_nbReady__var;
uint8_t Tron_headX__var;
uint8_t Tron_headY__var;
uint8_t Tron_tailX__var;
uint8_t Tron_tailY__var;
uint16_t Tron_headIndex__var;
uint8_t Tron_headIndexQuarter__var;
uint8_t Tron_lost__var;
uint8_t Tron_won__var;
uint8_t Tron_timer__var;
int16_t Tron_speed__var;
uint8_t Tron_direction__var;
uint8_t Tron_dirBuff__var;
uint16_t Tron_color__var[4];
uint8_t Tron_hasLost__var[4];
uint8_t Tron_isReady__var[4];

};
// Declaration of prototypes outgoing messages:
void Tron_TronStateChart_OnEntry(int state, struct Tron_Instance *_instance);
void Tron_handle_TronPort_tronReady(struct Tron_Instance *_instance, uint8_t id);
void Tron_handle_TronPort_iHaveID(struct Tron_Instance *_instance, uint8_t id);
void Tron_handle_TronPort_hasID(struct Tron_Instance *_instance, uint8_t id);
void Tron_handle_TronPort_mayIHaveID(struct Tron_Instance *_instance, uint8_t id);
void Tron_handle_TronPort_loose(struct Tron_Instance *_instance, uint8_t id);
void Tron_handle_TronPort_tronGo(struct Tron_Instance *_instance, uint8_t nbID);
void Tron_handle_TronPort_addHead(struct Tron_Instance *_instance, uint8_t x, uint8_t y, uint8_t id);
void Tron_handle_lcd_LCDready(struct Tron_Instance *_instance);
void Tron_handle_arduino_100ms_interrupt(struct Tron_Instance *_instance);
void Tron_handle_arduino_timeout(struct Tron_Instance *_instance, uint8_t id);
void Tron_handle_button_button_state_change(struct Tron_Instance *_instance, uint8_t bstate);
// Declaration of callbacks for incoming messages:
void register_Tron_send_TronPort_addHead_listener(void (*_listener)(struct Tron_Instance *, uint8_t, uint8_t, uint8_t));
void register_external_Tron_send_TronPort_addHead_listener(void (*_listener)(struct Tron_Instance *, uint8_t, uint8_t, uint8_t));
void register_Tron_send_TronPort_loose_listener(void (*_listener)(struct Tron_Instance *, uint8_t));
void register_external_Tron_send_TronPort_loose_listener(void (*_listener)(struct Tron_Instance *, uint8_t));
void register_Tron_send_TronPort_tronReady_listener(void (*_listener)(struct Tron_Instance *, uint8_t));
void register_external_Tron_send_TronPort_tronReady_listener(void (*_listener)(struct Tron_Instance *, uint8_t));
void register_Tron_send_TronPort_tronGo_listener(void (*_listener)(struct Tron_Instance *, uint8_t));
void register_external_Tron_send_TronPort_tronGo_listener(void (*_listener)(struct Tron_Instance *, uint8_t));
void register_Tron_send_TronPort_hasID_listener(void (*_listener)(struct Tron_Instance *, uint8_t));
void register_external_Tron_send_TronPort_hasID_listener(void (*_listener)(struct Tron_Instance *, uint8_t));
void register_Tron_send_TronPort_iHaveID_listener(void (*_listener)(struct Tron_Instance *, uint8_t));
void register_external_Tron_send_TronPort_iHaveID_listener(void (*_listener)(struct Tron_Instance *, uint8_t));
void register_Tron_send_TronPort_mayIHaveID_listener(void (*_listener)(struct Tron_Instance *, uint8_t));
void register_external_Tron_send_TronPort_mayIHaveID_listener(void (*_listener)(struct Tron_Instance *, uint8_t));
void register_Tron_send_button_button_state_listener(void (*_listener)(struct Tron_Instance *));
void register_external_Tron_send_button_button_state_listener(void (*_listener)(struct Tron_Instance *));
void register_Tron_send_lcd_print_num_listener(void (*_listener)(struct Tron_Instance *, int16_t));
void register_external_Tron_send_lcd_print_num_listener(void (*_listener)(struct Tron_Instance *, int16_t));
void register_Tron_send_lcd_print_dec_listener(void (*_listener)(struct Tron_Instance *, double));
void register_external_Tron_send_lcd_print_dec_listener(void (*_listener)(struct Tron_Instance *, double));
void register_Tron_send_lcd_print_str_listener(void (*_listener)(struct Tron_Instance *, char *));
void register_external_Tron_send_lcd_print_str_listener(void (*_listener)(struct Tron_Instance *, char *));
void register_Tron_send_lcd_clear_listener(void (*_listener)(struct Tron_Instance *));
void register_external_Tron_send_lcd_clear_listener(void (*_listener)(struct Tron_Instance *));
void register_Tron_send_lcd_set_cursor_listener(void (*_listener)(struct Tron_Instance *, uint8_t, uint8_t));
void register_external_Tron_send_lcd_set_cursor_listener(void (*_listener)(struct Tron_Instance *, uint8_t, uint8_t));
void register_Tron_send_lcd_set_bgcolor_listener(void (*_listener)(struct Tron_Instance *, uint8_t));
void register_external_Tron_send_lcd_set_bgcolor_listener(void (*_listener)(struct Tron_Instance *, uint8_t));
void register_Tron_send_lcd_fillRect_listener(void (*_listener)(struct Tron_Instance *, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t));
void register_external_Tron_send_lcd_fillRect_listener(void (*_listener)(struct Tron_Instance *, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t));
void register_Tron_send_lcd_drawRect_listener(void (*_listener)(struct Tron_Instance *, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t));
void register_external_Tron_send_lcd_drawRect_listener(void (*_listener)(struct Tron_Instance *, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t));
void register_Tron_send_arduino_timer_start_listener(void (*_listener)(struct Tron_Instance *, uint8_t, int16_t));
void register_external_Tron_send_arduino_timer_start_listener(void (*_listener)(struct Tron_Instance *, uint8_t, int16_t));
void register_Tron_send_arduino_timer_cancel_listener(void (*_listener)(struct Tron_Instance *, uint8_t));
void register_external_Tron_send_arduino_timer_cancel_listener(void (*_listener)(struct Tron_Instance *, uint8_t));

// Definition of the states:
#define TRON_TRONSTATECHART_STATE 0
#define TRON_TRONSTATECHART_INIT_STATE 1
#define TRON_TRONSTATECHART_DISCOVERY_STATE 2
#define TRON_TRONSTATECHART_IDOPTION_STATE 3
#define TRON_TRONSTATECHART_RANDOMWAIT_STATE 4
#define TRON_TRONSTATECHART_RENDEZVOUS_STATE 5
#define TRON_TRONSTATECHART_READY_STATE 6
#define TRON_TRONSTATECHART_GAME_STATE 7
#define TRON_TRONSTATECHART_DEFEAT_STATE 8
#define TRON_TRONSTATECHART_VICTORY_STATE 9


/*****************************************************************************
 * Headers for type : ArduinoScheduler
 *****************************************************************************/


// BEGIN: Code from the c_header annotation ArduinoScheduler

#define NB_SOFT_TIMERS 4 // for 4 different timers, 0 to 3. change here to get more or less timers.

// END: Code from the c_header annotation ArduinoScheduler

// Definition of the instance stuct:
struct ArduinoScheduler_Instance {
// Variables for the ID of the ports of the instance
uint16_t id_arduino;
// Pointer to receiver list
struct Msg_Handler ** arduino_receiver_list_head;
struct Msg_Handler ** arduino_receiver_list_tail;
// Handler Array
struct Msg_Handler * arduino_handlers;
uint16_t id_polling;
// Handler Array
struct Msg_Handler * polling_handlers;
// Variables for the current instance state
int ArduinoScheduler_ArduinoSchedulerStateChart_State;
// Variables for the properties of the instance
uint8_t ArduinoScheduler_interrupt_counter__var;
long ArduinoScheduler_ArduinoSchedulerStateChart_timers__var[NB_SOFT_TIMERS];
long ArduinoScheduler_ArduinoSchedulerStateChart_prev_1sec__var;

};
// Declaration of prototypes outgoing messages:
void ArduinoScheduler_ArduinoSchedulerStateChart_OnEntry(int state, struct ArduinoScheduler_Instance *_instance);
void ArduinoScheduler_handle_polling_poll(struct ArduinoScheduler_Instance *_instance);
void ArduinoScheduler_handle_polling_setup(struct ArduinoScheduler_Instance *_instance);
void ArduinoScheduler_handle_arduino_timer_cancel(struct ArduinoScheduler_Instance *_instance, uint8_t id);
void ArduinoScheduler_handle_arduino_timer_start(struct ArduinoScheduler_Instance *_instance, uint8_t id, int16_t time);
// Declaration of callbacks for incoming messages:
void register_ArduinoScheduler_send_arduino_ready_listener(void (*_listener)(struct ArduinoScheduler_Instance *));
void register_external_ArduinoScheduler_send_arduino_ready_listener(void (*_listener)(struct ArduinoScheduler_Instance *));
void register_ArduinoScheduler_send_arduino_4ms_interrupt_listener(void (*_listener)(struct ArduinoScheduler_Instance *));
void register_external_ArduinoScheduler_send_arduino_4ms_interrupt_listener(void (*_listener)(struct ArduinoScheduler_Instance *));
void register_ArduinoScheduler_send_arduino_100ms_interrupt_listener(void (*_listener)(struct ArduinoScheduler_Instance *));
void register_external_ArduinoScheduler_send_arduino_100ms_interrupt_listener(void (*_listener)(struct ArduinoScheduler_Instance *));
void register_ArduinoScheduler_send_arduino_1s_poll_listener(void (*_listener)(struct ArduinoScheduler_Instance *));
void register_external_ArduinoScheduler_send_arduino_1s_poll_listener(void (*_listener)(struct ArduinoScheduler_Instance *));
void register_ArduinoScheduler_send_arduino_timeout_listener(void (*_listener)(struct ArduinoScheduler_Instance *, uint8_t));
void register_external_ArduinoScheduler_send_arduino_timeout_listener(void (*_listener)(struct ArduinoScheduler_Instance *, uint8_t));

// Definition of the states:
#define ARDUINOSCHEDULER_ARDUINOSCHEDULERSTATECHART_STATE 0
#define ARDUINOSCHEDULER_ARDUINOSCHEDULERSTATECHART_ACTIVE_STATE 1


//Port message handler structure
typedef struct Msg_Handler {
	int nb_msg;
	uint16_t * msg;
	void ** msg_handler;
	void * instance;
};


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

#define MAX_INSTANCES 10
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

/*****************************************************************************
 * Implementation for type : Tron
 *****************************************************************************/


// BEGIN: Code from the c_global annotation Tron
volatile uint8_t tab[_SNAKE_TAB_SIZE];
// END: Code from the c_global annotation Tron

// Declaration of prototypes:
void Tron_TronStateChart_OnExit(int state, struct Tron_Instance *_instance);
void Tron_send_TronPort_addHead(struct Tron_Instance *_instance, uint8_t x, uint8_t y, uint8_t id);
void Tron_send_TronPort_loose(struct Tron_Instance *_instance, uint8_t id);
void Tron_send_TronPort_tronReady(struct Tron_Instance *_instance, uint8_t id);
void Tron_send_TronPort_tronGo(struct Tron_Instance *_instance, uint8_t nbID);
void Tron_send_TronPort_hasID(struct Tron_Instance *_instance, uint8_t id);
void Tron_send_TronPort_iHaveID(struct Tron_Instance *_instance, uint8_t id);
void Tron_send_TronPort_mayIHaveID(struct Tron_Instance *_instance, uint8_t id);
void f_Tron_displayTab(struct Tron_Instance *_instance);
uint8_t f_Tron_hasWon(struct Tron_Instance *_instance);
void f_Tron_addHeadDir(struct Tron_Instance *_instance, uint8_t x, uint8_t y, uint8_t dir);
uint8_t f_Tron_outOfBound(struct Tron_Instance *_instance, uint8_t x, uint8_t y);
uint8_t f_Tron_isInSnake(struct Tron_Instance *_instance, uint8_t x, uint8_t y);
uint8_t f_Tron_shallWe(struct Tron_Instance *_instance);
// Declaration of functions:
// Definition of function displayTab
void f_Tron_displayTab(struct Tron_Instance *_instance) {

}
// Definition of function hasWon
uint8_t f_Tron_hasWon(struct Tron_Instance *_instance) {

		bool won = true;
		for(uint8_t i = 0; i < _instance->Tron_nbID__var; i++) {
			if(i != _instance->Tron_myID__var) {won &= _instance->Tron_hasLost__var[i]
;}
		}
return won;
}
// Definition of function addHeadDir
void f_Tron_addHeadDir(struct Tron_Instance *_instance, uint8_t x, uint8_t y, uint8_t dir) {
Tron_send_TronPort_addHead(_instance, x, y, _instance->Tron_myID__var);
Tron_send_lcd_drawRect(_instance, 2+3*x, 2+3*y, 2, 2, _instance->Tron_color__var[_instance->Tron_myID__var]
);
uint8_t move = dir;
		uint8_t hI = _instance->Tron_headIndex__var;
		uint8_t hIQ = _instance->Tron_headIndexQuarter__var;
		
		if (hIQ == 0) {
			tab[hI] = B00000000;
		}
		
		
		tab[hI] |= (move << (2 * hIQ));
		
		hIQ = (hIQ + 1) % 4;
		if (hIQ == 0) {
			hI++;
		}
_instance->Tron_headX__var = x;
_instance->Tron_headY__var = y;
_instance->Tron_headIndex__var = hI;
_instance->Tron_headIndexQuarter__var = hIQ;
}
// Definition of function outOfBound
uint8_t f_Tron_outOfBound(struct Tron_Instance *_instance, uint8_t x, uint8_t y) {
return ((x > 41) || (y > 51));
}
// Definition of function isInSnake
uint8_t f_Tron_isInSnake(struct Tron_Instance *_instance, uint8_t x, uint8_t y) {
bool found = false;
		uint8_t curx = _instance->Tron_tailX__var;
		uint8_t cury = _instance->Tron_tailY__var;
		uint8_t targetx = x;
		uint8_t targety = y;
		uint16_t headIndex = _instance->Tron_headIndex__var;
		uint8_t  headIndexQuarter = _instance->Tron_headIndexQuarter__var;
		uint8_t curtab = 0;
		
		for(uint16_t i = 0; i < headIndex; i++) {
			for(uint8_t q = 0; q < 4; q++) {
				curtab = tab[i];
				
				
				//delayMicroseconds(1);
				
				if ((targetx == curx) && (targety == cury)) {
					return true;
				}
				switch((curtab >> (2 * q)) & B00000011) {
					case B00000000:
						cury--;
					break;
					case B00000001:
						curx++;
					break;
					case B00000010:
						cury++;
					break;
					case B00000011:
						curx--;
					break;
				}
				
				
			}
		}
		
		for(uint8_t q = 0; q < headIndexQuarter; q++) {
				
				
				//delayMicroseconds(1);
				
				if ((targetx == curx) && (targety == cury)) {
					return true;
				}
				curtab = tab[headIndex];
				
				switch((curtab >> (2 * q)) & B00000011) {
					case B00000000:
						cury--;
					break;
					case B00000001:
						curx++;
					break;
					case B00000010:
						cury++;
					break;
					case B00000011:
						curx--;
					break;
				}
				
				
			}
return 0;
}
// Definition of function shallWe
uint8_t f_Tron_shallWe(struct Tron_Instance *_instance) {
if( !((_instance->Tron_myID__var == 0))) {
return 0;

}
;uint8_t res = 1;
;uint8_t mytmp = 0;
while(mytmp < _instance->Tron_nbID__var) {
res = res && _instance->Tron_isReady__var[mytmp]
;
mytmp = mytmp + 1;

}
return res;
}

// On Entry Actions:
void Tron_TronStateChart_OnEntry(int state, struct Tron_Instance *_instance) {
switch(state) {
case TRON_TRONSTATECHART_STATE:
_instance->Tron_TronStateChart_State = TRON_TRONSTATECHART_INIT_STATE;
Tron_TronStateChart_OnEntry(_instance->Tron_TronStateChart_State, _instance);
break;
case TRON_TRONSTATECHART_INIT_STATE:
_instance->Tron_color__var[0] = 0x001F;
_instance->Tron_color__var[1] = 0xF800;
_instance->Tron_color__var[2] = 0x3FE0;
_instance->Tron_color__var[3] = 0x0FFF;
_instance->Tron_hasLost__var[0] = 0;
_instance->Tron_hasLost__var[1] = 0;
_instance->Tron_hasLost__var[2] = 0;
_instance->Tron_hasLost__var[3] = 0;
_instance->Tron_isReady__var[0] = 0;
_instance->Tron_isReady__var[1] = 0;
_instance->Tron_isReady__var[2] = 0;
_instance->Tron_isReady__var[3] = 0;
break;
case TRON_TRONSTATECHART_DISCOVERY_STATE:
Tron_send_lcd_clear(_instance);
Tron_send_lcd_drawRect(_instance, 1, 1, 127, 157, 0xFFFF);
Tron_send_lcd_set_cursor(_instance, 16, 68);
Tron_send_lcd_print_str(_instance, "Welcome to Tron!");
Tron_send_lcd_set_cursor(_instance, 16, 78);
Tron_send_lcd_print_str(_instance, "Discovery phase.");
Tron_send_TronPort_hasID(_instance, _instance->Tron_curID__var);
Tron_send_arduino_timer_start(_instance, 0, 300);
break;
case TRON_TRONSTATECHART_IDOPTION_STATE:
Tron_send_TronPort_mayIHaveID(_instance, _instance->Tron_myID__var);
Tron_send_arduino_timer_start(_instance, 0, 300);
Tron_send_lcd_drawRect(_instance, 1, 1, 127, 157, _instance->Tron_color__var[_instance->Tron_myID__var]
);
break;
case TRON_TRONSTATECHART_RANDOMWAIT_STATE:
Tron_send_arduino_timer_start(_instance, 0, random(0, 1200));
break;
case TRON_TRONSTATECHART_RENDEZVOUS_STATE:
Tron_send_lcd_clear(_instance);
Tron_send_lcd_drawRect(_instance, 1, 1, 127, 157, _instance->Tron_color__var[_instance->Tron_myID__var]
);
Tron_send_lcd_set_cursor(_instance, 12, 68);
Tron_send_lcd_print_str(_instance, "Press the joystick");
Tron_send_lcd_set_cursor(_instance, 12, 78);
Tron_send_lcd_print_str(_instance, "when you are ready");
if(_instance->Tron_myID__var == 0) {
_instance->Tron_headX__var = 10;
_instance->Tron_headY__var = 10;
_instance->Tron_tailX__var = 10;
_instance->Tron_tailY__var = 10;

}
if(_instance->Tron_myID__var == 1) {
_instance->Tron_headX__var = 30;
_instance->Tron_headY__var = 40;
_instance->Tron_tailX__var = 30;
_instance->Tron_tailY__var = 40;

}
if(_instance->Tron_myID__var == 2) {
_instance->Tron_headX__var = 10;
_instance->Tron_headY__var = 40;
_instance->Tron_tailX__var = 10;
_instance->Tron_tailY__var = 40;

}
if(_instance->Tron_myID__var == 3) {
_instance->Tron_headX__var = 30;
_instance->Tron_headY__var = 10;
_instance->Tron_tailX__var = 30;
_instance->Tron_tailY__var = 10;

}
break;
case TRON_TRONSTATECHART_READY_STATE:
Tron_send_lcd_clear(_instance);
Tron_send_lcd_drawRect(_instance, 1, 1, 127, 157, _instance->Tron_color__var[_instance->Tron_myID__var]
);
Tron_send_lcd_set_cursor(_instance, 12, 78);
Tron_send_lcd_print_str(_instance, "Waiting for others");
_instance->Tron_isReady__var[_instance->Tron_myID__var] = 1;
Tron_send_TronPort_tronReady(_instance, _instance->Tron_myID__var);
break;
case TRON_TRONSTATECHART_GAME_STATE:
_instance->Tron_nbReady__var = 0;
Tron_send_lcd_drawRect(_instance, 2+3*_instance->Tron_headX__var, 2+3*_instance->Tron_headY__var, 2, 2, _instance->Tron_color__var[_instance->Tron_myID__var]
);
Tron_send_TronPort_addHead(_instance, _instance->Tron_headX__var, _instance->Tron_headY__var, _instance->Tron_myID__var);
_instance->Tron_nbReady__var = 0;
Tron_send_arduino_timer_start(_instance, _instance->Tron_timer__var, _instance->Tron_speed__var);
break;
case TRON_TRONSTATECHART_DEFEAT_STATE:
Tron_send_lcd_clear(_instance);
Tron_send_lcd_drawRect(_instance, 1, 1, 127, 157, _instance->Tron_color__var[_instance->Tron_myID__var]
);
Tron_send_lcd_set_cursor(_instance, 45, 78);
Tron_send_lcd_print_str(_instance, "Defeat!");
break;
case TRON_TRONSTATECHART_VICTORY_STATE:
f_Tron_displayTab(_instance);
Tron_send_lcd_clear(_instance);
Tron_send_lcd_drawRect(_instance, 1, 1, 127, 157, _instance->Tron_color__var[_instance->Tron_myID__var]
);
Tron_send_lcd_set_cursor(_instance, 44, 78);
Tron_send_lcd_print_str(_instance, "Victory!");
break;
default: break;
}
}

// On Exit Actions:
void Tron_TronStateChart_OnExit(int state, struct Tron_Instance *_instance) {
switch(state) {
case TRON_TRONSTATECHART_STATE:
Tron_TronStateChart_OnExit(_instance->Tron_TronStateChart_State, _instance);
break;
case TRON_TRONSTATECHART_INIT_STATE:
break;
case TRON_TRONSTATECHART_DISCOVERY_STATE:
break;
case TRON_TRONSTATECHART_IDOPTION_STATE:
break;
case TRON_TRONSTATECHART_RANDOMWAIT_STATE:
break;
case TRON_TRONSTATECHART_RENDEZVOUS_STATE:
break;
case TRON_TRONSTATECHART_READY_STATE:
Tron_send_lcd_clear(_instance);
Tron_send_lcd_drawRect(_instance, 1, 1, 127, 157, _instance->Tron_color__var[_instance->Tron_myID__var]
);
break;
case TRON_TRONSTATECHART_GAME_STATE:
break;
case TRON_TRONSTATECHART_DEFEAT_STATE:
break;
case TRON_TRONSTATECHART_VICTORY_STATE:
break;
default: break;
}
}

// Event Handlers for incoming messages:
void Tron_handle_TronPort_tronReady(struct Tron_Instance *_instance, uint8_t id) {
uint8_t Tron_TronStateChart_State_event_consumed = 0;
if (_instance->Tron_TronStateChart_State == TRON_TRONSTATECHART_RENDEZVOUS_STATE) {
if (Tron_TronStateChart_State_event_consumed == 0 && 1) {
_instance->Tron_isReady__var[id] = 1;
Tron_TronStateChart_State_event_consumed = 1;
}
}
else if (_instance->Tron_TronStateChart_State == TRON_TRONSTATECHART_READY_STATE) {
if (Tron_TronStateChart_State_event_consumed == 0 && 1) {
_instance->Tron_isReady__var[id] = 1;
Tron_TronStateChart_State_event_consumed = 1;
}
}
}
void Tron_handle_TronPort_iHaveID(struct Tron_Instance *_instance, uint8_t id) {
uint8_t Tron_TronStateChart_State_event_consumed = 0;
if (_instance->Tron_TronStateChart_State == TRON_TRONSTATECHART_DISCOVERY_STATE) {
if (Tron_TronStateChart_State_event_consumed == 0 && (id == _instance->Tron_curID__var)) {
Tron_send_arduino_timer_cancel(_instance, 0);
Tron_send_arduino_timer_start(_instance, 0, 300);
Tron_send_lcd_drawRect(_instance, 2+3*_instance->Tron_curID__var, 2, 2, 2, _instance->Tron_color__var[_instance->Tron_curID__var]
);
_instance->Tron_curID__var = _instance->Tron_curID__var + 1;
Tron_TronStateChart_State_event_consumed = 1;
}
}
else if (_instance->Tron_TronStateChart_State == TRON_TRONSTATECHART_IDOPTION_STATE) {
if (Tron_TronStateChart_State_event_consumed == 0 && (id == _instance->Tron_curID__var)) {
Tron_TronStateChart_OnExit(TRON_TRONSTATECHART_IDOPTION_STATE, _instance);
_instance->Tron_TronStateChart_State = TRON_TRONSTATECHART_DISCOVERY_STATE;
Tron_send_arduino_timer_cancel(_instance, 0);
Tron_TronStateChart_OnEntry(TRON_TRONSTATECHART_DISCOVERY_STATE, _instance);
Tron_TronStateChart_State_event_consumed = 1;
}
}
}
void Tron_handle_TronPort_hasID(struct Tron_Instance *_instance, uint8_t id) {
uint8_t Tron_TronStateChart_State_event_consumed = 0;
if (_instance->Tron_TronStateChart_State == TRON_TRONSTATECHART_RENDEZVOUS_STATE) {
if (Tron_TronStateChart_State_event_consumed == 0 && (id == _instance->Tron_myID__var)) {
Tron_send_TronPort_iHaveID(_instance, _instance->Tron_myID__var);
Tron_TronStateChart_State_event_consumed = 1;
}
}
else if (_instance->Tron_TronStateChart_State == TRON_TRONSTATECHART_READY_STATE) {
if (Tron_TronStateChart_State_event_consumed == 0 && (id == _instance->Tron_myID__var)) {
Tron_send_TronPort_iHaveID(_instance, _instance->Tron_myID__var);
Tron_TronStateChart_State_event_consumed = 1;
}
}
}
void Tron_handle_TronPort_mayIHaveID(struct Tron_Instance *_instance, uint8_t id) {
uint8_t Tron_TronStateChart_State_event_consumed = 0;
if (_instance->Tron_TronStateChart_State == TRON_TRONSTATECHART_IDOPTION_STATE) {
if (Tron_TronStateChart_State_event_consumed == 0 && (id == _instance->Tron_curID__var)) {
Tron_TronStateChart_OnExit(TRON_TRONSTATECHART_IDOPTION_STATE, _instance);
_instance->Tron_TronStateChart_State = TRON_TRONSTATECHART_RANDOMWAIT_STATE;
Tron_send_arduino_timer_cancel(_instance, 0);
Tron_TronStateChart_OnEntry(TRON_TRONSTATECHART_RANDOMWAIT_STATE, _instance);
Tron_TronStateChart_State_event_consumed = 1;
}
}
else if (_instance->Tron_TronStateChart_State == TRON_TRONSTATECHART_RENDEZVOUS_STATE) {
if (Tron_TronStateChart_State_event_consumed == 0 && 1) {
if(id == _instance->Tron_myID__var) {
Tron_send_TronPort_iHaveID(_instance, _instance->Tron_myID__var);

}
if(id > (_instance->Tron_nbID__var - 1)) {
_instance->Tron_nbID__var = id + 1;

}
Tron_TronStateChart_State_event_consumed = 1;
}
}
else if (_instance->Tron_TronStateChart_State == TRON_TRONSTATECHART_READY_STATE) {
if (Tron_TronStateChart_State_event_consumed == 0 && 1) {
if(id == _instance->Tron_myID__var) {
Tron_send_TronPort_iHaveID(_instance, _instance->Tron_myID__var);

}
if(id > (_instance->Tron_nbID__var - 1)) {
_instance->Tron_nbID__var = id + 1;

}
Tron_TronStateChart_State_event_consumed = 1;
}
}
}
void Tron_handle_TronPort_loose(struct Tron_Instance *_instance, uint8_t id) {
uint8_t Tron_TronStateChart_State_event_consumed = 0;
if (_instance->Tron_TronStateChart_State == TRON_TRONSTATECHART_GAME_STATE) {
if (Tron_TronStateChart_State_event_consumed == 0 && (id == _instance->Tron_myID__var)) {
_instance->Tron_lost__var = 1;
Tron_TronStateChart_State_event_consumed = 1;
}
else if (Tron_TronStateChart_State_event_consumed == 0 &&  !((id == _instance->Tron_myID__var))) {
_instance->Tron_hasLost__var[id] = 1;
_instance->Tron_won__var = f_Tron_hasWon(_instance);
Tron_TronStateChart_State_event_consumed = 1;
}
}
}
void Tron_handle_TronPort_tronGo(struct Tron_Instance *_instance, uint8_t nbID) {
uint8_t Tron_TronStateChart_State_event_consumed = 0;
if (_instance->Tron_TronStateChart_State == TRON_TRONSTATECHART_READY_STATE) {
if (Tron_TronStateChart_State_event_consumed == 0 && 1) {
Tron_TronStateChart_OnExit(TRON_TRONSTATECHART_READY_STATE, _instance);
_instance->Tron_TronStateChart_State = TRON_TRONSTATECHART_GAME_STATE;
_instance->Tron_nbID__var = nbID;
Tron_TronStateChart_OnEntry(TRON_TRONSTATECHART_GAME_STATE, _instance);
Tron_TronStateChart_State_event_consumed = 1;
}
}
}
void Tron_handle_TronPort_addHead(struct Tron_Instance *_instance, uint8_t x, uint8_t y, uint8_t id) {
uint8_t Tron_TronStateChart_State_event_consumed = 0;
if (_instance->Tron_TronStateChart_State == TRON_TRONSTATECHART_GAME_STATE) {
if (Tron_TronStateChart_State_event_consumed == 0 && 1) {
Tron_send_lcd_drawRect(_instance, 2+3*x, 2+3*y, 2, 2, _instance->Tron_color__var[id]
);
if(f_Tron_isInSnake(_instance, x, y)) {
Tron_send_TronPort_loose(_instance, id);
_instance->Tron_hasLost__var[id] = 1;
_instance->Tron_won__var = f_Tron_hasWon(_instance);

}
Tron_TronStateChart_State_event_consumed = 1;
}
}
else if (_instance->Tron_TronStateChart_State == TRON_TRONSTATECHART_DEFEAT_STATE) {
if (Tron_TronStateChart_State_event_consumed == 0 && 1) {
if(f_Tron_isInSnake(_instance, x, y)) {
Tron_send_TronPort_loose(_instance, id);

}
Tron_TronStateChart_State_event_consumed = 1;
}
}
}
void Tron_handle_lcd_LCDready(struct Tron_Instance *_instance) {
uint8_t Tron_TronStateChart_State_event_consumed = 0;
if (_instance->Tron_TronStateChart_State == TRON_TRONSTATECHART_INIT_STATE) {
if (Tron_TronStateChart_State_event_consumed == 0 && 1) {
Tron_TronStateChart_OnExit(TRON_TRONSTATECHART_INIT_STATE, _instance);
_instance->Tron_TronStateChart_State = TRON_TRONSTATECHART_DISCOVERY_STATE;
Tron_TronStateChart_OnEntry(TRON_TRONSTATECHART_DISCOVERY_STATE, _instance);
Tron_TronStateChart_State_event_consumed = 1;
}
}
}
void Tron_handle_arduino_100ms_interrupt(struct Tron_Instance *_instance) {
uint8_t Tron_TronStateChart_State_event_consumed = 0;
if (_instance->Tron_TronStateChart_State == TRON_TRONSTATECHART_READY_STATE) {
if (Tron_TronStateChart_State_event_consumed == 0 && f_Tron_shallWe(_instance)) {
Tron_TronStateChart_OnExit(TRON_TRONSTATECHART_READY_STATE, _instance);
_instance->Tron_TronStateChart_State = TRON_TRONSTATECHART_GAME_STATE;
Tron_send_TronPort_tronGo(_instance, _instance->Tron_nbID__var);
Tron_TronStateChart_OnEntry(TRON_TRONSTATECHART_GAME_STATE, _instance);
Tron_TronStateChart_State_event_consumed = 1;
}
}
}
void Tron_handle_arduino_timeout(struct Tron_Instance *_instance, uint8_t id) {
uint8_t Tron_TronStateChart_State_event_consumed = 0;
if (_instance->Tron_TronStateChart_State == TRON_TRONSTATECHART_DISCOVERY_STATE) {
if (Tron_TronStateChart_State_event_consumed == 0 && 1) {
Tron_TronStateChart_OnExit(TRON_TRONSTATECHART_DISCOVERY_STATE, _instance);
_instance->Tron_TronStateChart_State = TRON_TRONSTATECHART_IDOPTION_STATE;
_instance->Tron_myID__var = _instance->Tron_curID__var;
Tron_TronStateChart_OnEntry(TRON_TRONSTATECHART_IDOPTION_STATE, _instance);
Tron_TronStateChart_State_event_consumed = 1;
}
}
else if (_instance->Tron_TronStateChart_State == TRON_TRONSTATECHART_IDOPTION_STATE) {
if (Tron_TronStateChart_State_event_consumed == 0 && 1) {
Tron_TronStateChart_OnExit(TRON_TRONSTATECHART_IDOPTION_STATE, _instance);
_instance->Tron_TronStateChart_State = TRON_TRONSTATECHART_RENDEZVOUS_STATE;
Tron_TronStateChart_OnEntry(TRON_TRONSTATECHART_RENDEZVOUS_STATE, _instance);
Tron_TronStateChart_State_event_consumed = 1;
}
}
else if (_instance->Tron_TronStateChart_State == TRON_TRONSTATECHART_RANDOMWAIT_STATE) {
if (Tron_TronStateChart_State_event_consumed == 0 && 1) {
Tron_TronStateChart_OnExit(TRON_TRONSTATECHART_RANDOMWAIT_STATE, _instance);
_instance->Tron_TronStateChart_State = TRON_TRONSTATECHART_DISCOVERY_STATE;
Tron_TronStateChart_OnEntry(TRON_TRONSTATECHART_DISCOVERY_STATE, _instance);
Tron_TronStateChart_State_event_consumed = 1;
}
}
else if (_instance->Tron_TronStateChart_State == TRON_TRONSTATECHART_GAME_STATE) {
if (Tron_TronStateChart_State_event_consumed == 0 && _instance->Tron_lost__var) {
Tron_TronStateChart_OnExit(TRON_TRONSTATECHART_GAME_STATE, _instance);
_instance->Tron_TronStateChart_State = TRON_TRONSTATECHART_DEFEAT_STATE;
Tron_TronStateChart_OnEntry(TRON_TRONSTATECHART_DEFEAT_STATE, _instance);
Tron_TronStateChart_State_event_consumed = 1;
}
else if (Tron_TronStateChart_State_event_consumed == 0 && _instance->Tron_won__var) {
Tron_TronStateChart_OnExit(TRON_TRONSTATECHART_GAME_STATE, _instance);
_instance->Tron_TronStateChart_State = TRON_TRONSTATECHART_VICTORY_STATE;
Tron_TronStateChart_OnEntry(TRON_TRONSTATECHART_VICTORY_STATE, _instance);
Tron_TronStateChart_State_event_consumed = 1;
}
else if (Tron_TronStateChart_State_event_consumed == 0 &&  !((_instance->Tron_lost__var || _instance->Tron_won__var))) {
Tron_send_arduino_timer_start(_instance, _instance->Tron_timer__var, _instance->Tron_speed__var);
_instance->Tron_direction__var = _instance->Tron_dirBuff__var;
switch(_instance->Tron_direction__var) {
					case B00000000:
						
if(f_Tron_isInSnake(_instance, _instance->Tron_headX__var, _instance->Tron_headY__var - 1)) {
_instance->Tron_lost__var = 1;
Tron_send_TronPort_loose(_instance, _instance->Tron_myID__var);

}
f_Tron_addHeadDir(_instance, _instance->Tron_headX__var, _instance->Tron_headY__var - 1, _instance->Tron_direction__var);

					break;
					case B00000001:
						
if(f_Tron_isInSnake(_instance, _instance->Tron_headX__var + 1, _instance->Tron_headY__var)) {
_instance->Tron_lost__var = 1;
Tron_send_TronPort_loose(_instance, _instance->Tron_myID__var);

}
f_Tron_addHeadDir(_instance, _instance->Tron_headX__var + 1, _instance->Tron_headY__var, _instance->Tron_direction__var);

					break;
					case B00000010:
						
if(f_Tron_isInSnake(_instance, _instance->Tron_headX__var, _instance->Tron_headY__var + 1)) {
_instance->Tron_lost__var = 1;
Tron_send_TronPort_loose(_instance, _instance->Tron_myID__var);

}
f_Tron_addHeadDir(_instance, _instance->Tron_headX__var, _instance->Tron_headY__var + 1, _instance->Tron_direction__var);

					break;
					case B00000011:
						
if(f_Tron_isInSnake(_instance, _instance->Tron_headX__var - 1, _instance->Tron_headY__var)) {
_instance->Tron_lost__var = 1;
Tron_send_TronPort_loose(_instance, _instance->Tron_myID__var);

}
f_Tron_addHeadDir(_instance, _instance->Tron_headX__var - 1, _instance->Tron_headY__var, _instance->Tron_direction__var);

					break;
				}
if(f_Tron_outOfBound(_instance, _instance->Tron_headX__var, _instance->Tron_headY__var)) {
_instance->Tron_lost__var = 1;
Tron_send_TronPort_loose(_instance, _instance->Tron_myID__var);

}
Tron_TronStateChart_State_event_consumed = 1;
}
}
}
void Tron_handle_button_button_state_change(struct Tron_Instance *_instance, uint8_t bstate) {
uint8_t Tron_TronStateChart_State_event_consumed = 0;
if (_instance->Tron_TronStateChart_State == TRON_TRONSTATECHART_RENDEZVOUS_STATE) {
if (Tron_TronStateChart_State_event_consumed == 0 && 1) {
Tron_TronStateChart_OnExit(TRON_TRONSTATECHART_RENDEZVOUS_STATE, _instance);
_instance->Tron_TronStateChart_State = TRON_TRONSTATECHART_READY_STATE;
Tron_TronStateChart_OnEntry(TRON_TRONSTATECHART_READY_STATE, _instance);
Tron_TronStateChart_State_event_consumed = 1;
}
}
else if (_instance->Tron_TronStateChart_State == TRON_TRONSTATECHART_GAME_STATE) {
if (Tron_TronStateChart_State_event_consumed == 0 && ((bstate == 1) || (bstate == 2) || (bstate == 4) || (bstate == 5))) {
if(bstate == 4) {
_instance->Tron_dirBuff__var = B00000000;

}
if(bstate == 2) {
_instance->Tron_dirBuff__var = B00000001;

}
if(bstate == 1) {
_instance->Tron_dirBuff__var = B00000010;

}
if(bstate == 5) {
_instance->Tron_dirBuff__var = B00000011;

}
Tron_TronStateChart_State_event_consumed = 1;
}
}
}

// Observers for outgoing messages:
void (*external_Tron_send_TronPort_addHead_listener)(struct Tron_Instance *, uint8_t, uint8_t, uint8_t)= 0x0;
void register_external_Tron_send_TronPort_addHead_listener(void (*_listener)(struct Tron_Instance *, uint8_t, uint8_t, uint8_t)){
external_Tron_send_TronPort_addHead_listener = _listener;
}
void (*Tron_send_TronPort_addHead_listener)(struct Tron_Instance *, uint8_t, uint8_t, uint8_t)= 0x0;
void register_Tron_send_TronPort_addHead_listener(void (*_listener)(struct Tron_Instance *, uint8_t, uint8_t, uint8_t)){
Tron_send_TronPort_addHead_listener = _listener;
}
void Tron_send_TronPort_addHead(struct Tron_Instance *_instance, uint8_t x, uint8_t y, uint8_t id){
if (Tron_send_TronPort_addHead_listener != 0x0) Tron_send_TronPort_addHead_listener(_instance, x, y, id);
if (external_Tron_send_TronPort_addHead_listener != 0x0) external_Tron_send_TronPort_addHead_listener(_instance, x, y, id);
;
}
void (*external_Tron_send_TronPort_loose_listener)(struct Tron_Instance *, uint8_t)= 0x0;
void register_external_Tron_send_TronPort_loose_listener(void (*_listener)(struct Tron_Instance *, uint8_t)){
external_Tron_send_TronPort_loose_listener = _listener;
}
void (*Tron_send_TronPort_loose_listener)(struct Tron_Instance *, uint8_t)= 0x0;
void register_Tron_send_TronPort_loose_listener(void (*_listener)(struct Tron_Instance *, uint8_t)){
Tron_send_TronPort_loose_listener = _listener;
}
void Tron_send_TronPort_loose(struct Tron_Instance *_instance, uint8_t id){
if (Tron_send_TronPort_loose_listener != 0x0) Tron_send_TronPort_loose_listener(_instance, id);
if (external_Tron_send_TronPort_loose_listener != 0x0) external_Tron_send_TronPort_loose_listener(_instance, id);
;
}
void (*external_Tron_send_TronPort_tronReady_listener)(struct Tron_Instance *, uint8_t)= 0x0;
void register_external_Tron_send_TronPort_tronReady_listener(void (*_listener)(struct Tron_Instance *, uint8_t)){
external_Tron_send_TronPort_tronReady_listener = _listener;
}
void (*Tron_send_TronPort_tronReady_listener)(struct Tron_Instance *, uint8_t)= 0x0;
void register_Tron_send_TronPort_tronReady_listener(void (*_listener)(struct Tron_Instance *, uint8_t)){
Tron_send_TronPort_tronReady_listener = _listener;
}
void Tron_send_TronPort_tronReady(struct Tron_Instance *_instance, uint8_t id){
if (Tron_send_TronPort_tronReady_listener != 0x0) Tron_send_TronPort_tronReady_listener(_instance, id);
if (external_Tron_send_TronPort_tronReady_listener != 0x0) external_Tron_send_TronPort_tronReady_listener(_instance, id);
;
}
void (*external_Tron_send_TronPort_tronGo_listener)(struct Tron_Instance *, uint8_t)= 0x0;
void register_external_Tron_send_TronPort_tronGo_listener(void (*_listener)(struct Tron_Instance *, uint8_t)){
external_Tron_send_TronPort_tronGo_listener = _listener;
}
void (*Tron_send_TronPort_tronGo_listener)(struct Tron_Instance *, uint8_t)= 0x0;
void register_Tron_send_TronPort_tronGo_listener(void (*_listener)(struct Tron_Instance *, uint8_t)){
Tron_send_TronPort_tronGo_listener = _listener;
}
void Tron_send_TronPort_tronGo(struct Tron_Instance *_instance, uint8_t nbID){
if (Tron_send_TronPort_tronGo_listener != 0x0) Tron_send_TronPort_tronGo_listener(_instance, nbID);
if (external_Tron_send_TronPort_tronGo_listener != 0x0) external_Tron_send_TronPort_tronGo_listener(_instance, nbID);
;
}
void (*external_Tron_send_TronPort_hasID_listener)(struct Tron_Instance *, uint8_t)= 0x0;
void register_external_Tron_send_TronPort_hasID_listener(void (*_listener)(struct Tron_Instance *, uint8_t)){
external_Tron_send_TronPort_hasID_listener = _listener;
}
void (*Tron_send_TronPort_hasID_listener)(struct Tron_Instance *, uint8_t)= 0x0;
void register_Tron_send_TronPort_hasID_listener(void (*_listener)(struct Tron_Instance *, uint8_t)){
Tron_send_TronPort_hasID_listener = _listener;
}
void Tron_send_TronPort_hasID(struct Tron_Instance *_instance, uint8_t id){
if (Tron_send_TronPort_hasID_listener != 0x0) Tron_send_TronPort_hasID_listener(_instance, id);
if (external_Tron_send_TronPort_hasID_listener != 0x0) external_Tron_send_TronPort_hasID_listener(_instance, id);
;
}
void (*external_Tron_send_TronPort_iHaveID_listener)(struct Tron_Instance *, uint8_t)= 0x0;
void register_external_Tron_send_TronPort_iHaveID_listener(void (*_listener)(struct Tron_Instance *, uint8_t)){
external_Tron_send_TronPort_iHaveID_listener = _listener;
}
void (*Tron_send_TronPort_iHaveID_listener)(struct Tron_Instance *, uint8_t)= 0x0;
void register_Tron_send_TronPort_iHaveID_listener(void (*_listener)(struct Tron_Instance *, uint8_t)){
Tron_send_TronPort_iHaveID_listener = _listener;
}
void Tron_send_TronPort_iHaveID(struct Tron_Instance *_instance, uint8_t id){
if (Tron_send_TronPort_iHaveID_listener != 0x0) Tron_send_TronPort_iHaveID_listener(_instance, id);
if (external_Tron_send_TronPort_iHaveID_listener != 0x0) external_Tron_send_TronPort_iHaveID_listener(_instance, id);
;
}
void (*external_Tron_send_TronPort_mayIHaveID_listener)(struct Tron_Instance *, uint8_t)= 0x0;
void register_external_Tron_send_TronPort_mayIHaveID_listener(void (*_listener)(struct Tron_Instance *, uint8_t)){
external_Tron_send_TronPort_mayIHaveID_listener = _listener;
}
void (*Tron_send_TronPort_mayIHaveID_listener)(struct Tron_Instance *, uint8_t)= 0x0;
void register_Tron_send_TronPort_mayIHaveID_listener(void (*_listener)(struct Tron_Instance *, uint8_t)){
Tron_send_TronPort_mayIHaveID_listener = _listener;
}
void Tron_send_TronPort_mayIHaveID(struct Tron_Instance *_instance, uint8_t id){
if (Tron_send_TronPort_mayIHaveID_listener != 0x0) Tron_send_TronPort_mayIHaveID_listener(_instance, id);
if (external_Tron_send_TronPort_mayIHaveID_listener != 0x0) external_Tron_send_TronPort_mayIHaveID_listener(_instance, id);
;
}
void (*external_Tron_send_button_button_state_listener)(struct Tron_Instance *)= 0x0;
void register_external_Tron_send_button_button_state_listener(void (*_listener)(struct Tron_Instance *)){
external_Tron_send_button_button_state_listener = _listener;
}
void (*Tron_send_button_button_state_listener)(struct Tron_Instance *)= 0x0;
void register_Tron_send_button_button_state_listener(void (*_listener)(struct Tron_Instance *)){
Tron_send_button_button_state_listener = _listener;
}
void Tron_send_button_button_state(struct Tron_Instance *_instance){
if (Tron_send_button_button_state_listener != 0x0) Tron_send_button_button_state_listener(_instance);
if (external_Tron_send_button_button_state_listener != 0x0) external_Tron_send_button_button_state_listener(_instance);
;
}
void (*external_Tron_send_lcd_print_num_listener)(struct Tron_Instance *, int16_t)= 0x0;
void register_external_Tron_send_lcd_print_num_listener(void (*_listener)(struct Tron_Instance *, int16_t)){
external_Tron_send_lcd_print_num_listener = _listener;
}
void (*Tron_send_lcd_print_num_listener)(struct Tron_Instance *, int16_t)= 0x0;
void register_Tron_send_lcd_print_num_listener(void (*_listener)(struct Tron_Instance *, int16_t)){
Tron_send_lcd_print_num_listener = _listener;
}
void Tron_send_lcd_print_num(struct Tron_Instance *_instance, int16_t num){
if (Tron_send_lcd_print_num_listener != 0x0) Tron_send_lcd_print_num_listener(_instance, num);
if (external_Tron_send_lcd_print_num_listener != 0x0) external_Tron_send_lcd_print_num_listener(_instance, num);
;
}
void (*external_Tron_send_lcd_print_dec_listener)(struct Tron_Instance *, double)= 0x0;
void register_external_Tron_send_lcd_print_dec_listener(void (*_listener)(struct Tron_Instance *, double)){
external_Tron_send_lcd_print_dec_listener = _listener;
}
void (*Tron_send_lcd_print_dec_listener)(struct Tron_Instance *, double)= 0x0;
void register_Tron_send_lcd_print_dec_listener(void (*_listener)(struct Tron_Instance *, double)){
Tron_send_lcd_print_dec_listener = _listener;
}
void Tron_send_lcd_print_dec(struct Tron_Instance *_instance, double num){
if (Tron_send_lcd_print_dec_listener != 0x0) Tron_send_lcd_print_dec_listener(_instance, num);
if (external_Tron_send_lcd_print_dec_listener != 0x0) external_Tron_send_lcd_print_dec_listener(_instance, num);
;
}
void (*external_Tron_send_lcd_print_str_listener)(struct Tron_Instance *, char *)= 0x0;
void register_external_Tron_send_lcd_print_str_listener(void (*_listener)(struct Tron_Instance *, char *)){
external_Tron_send_lcd_print_str_listener = _listener;
}
void (*Tron_send_lcd_print_str_listener)(struct Tron_Instance *, char *)= 0x0;
void register_Tron_send_lcd_print_str_listener(void (*_listener)(struct Tron_Instance *, char *)){
Tron_send_lcd_print_str_listener = _listener;
}
void Tron_send_lcd_print_str(struct Tron_Instance *_instance, char * msg){
if (Tron_send_lcd_print_str_listener != 0x0) Tron_send_lcd_print_str_listener(_instance, msg);
if (external_Tron_send_lcd_print_str_listener != 0x0) external_Tron_send_lcd_print_str_listener(_instance, msg);
;
}
void (*external_Tron_send_lcd_clear_listener)(struct Tron_Instance *)= 0x0;
void register_external_Tron_send_lcd_clear_listener(void (*_listener)(struct Tron_Instance *)){
external_Tron_send_lcd_clear_listener = _listener;
}
void (*Tron_send_lcd_clear_listener)(struct Tron_Instance *)= 0x0;
void register_Tron_send_lcd_clear_listener(void (*_listener)(struct Tron_Instance *)){
Tron_send_lcd_clear_listener = _listener;
}
void Tron_send_lcd_clear(struct Tron_Instance *_instance){
if (Tron_send_lcd_clear_listener != 0x0) Tron_send_lcd_clear_listener(_instance);
if (external_Tron_send_lcd_clear_listener != 0x0) external_Tron_send_lcd_clear_listener(_instance);
;
}
void (*external_Tron_send_lcd_set_cursor_listener)(struct Tron_Instance *, uint8_t, uint8_t)= 0x0;
void register_external_Tron_send_lcd_set_cursor_listener(void (*_listener)(struct Tron_Instance *, uint8_t, uint8_t)){
external_Tron_send_lcd_set_cursor_listener = _listener;
}
void (*Tron_send_lcd_set_cursor_listener)(struct Tron_Instance *, uint8_t, uint8_t)= 0x0;
void register_Tron_send_lcd_set_cursor_listener(void (*_listener)(struct Tron_Instance *, uint8_t, uint8_t)){
Tron_send_lcd_set_cursor_listener = _listener;
}
void Tron_send_lcd_set_cursor(struct Tron_Instance *_instance, uint8_t c, uint8_t l){
if (Tron_send_lcd_set_cursor_listener != 0x0) Tron_send_lcd_set_cursor_listener(_instance, c, l);
if (external_Tron_send_lcd_set_cursor_listener != 0x0) external_Tron_send_lcd_set_cursor_listener(_instance, c, l);
;
}
void (*external_Tron_send_lcd_set_bgcolor_listener)(struct Tron_Instance *, uint8_t)= 0x0;
void register_external_Tron_send_lcd_set_bgcolor_listener(void (*_listener)(struct Tron_Instance *, uint8_t)){
external_Tron_send_lcd_set_bgcolor_listener = _listener;
}
void (*Tron_send_lcd_set_bgcolor_listener)(struct Tron_Instance *, uint8_t)= 0x0;
void register_Tron_send_lcd_set_bgcolor_listener(void (*_listener)(struct Tron_Instance *, uint8_t)){
Tron_send_lcd_set_bgcolor_listener = _listener;
}
void Tron_send_lcd_set_bgcolor(struct Tron_Instance *_instance, uint8_t color){
if (Tron_send_lcd_set_bgcolor_listener != 0x0) Tron_send_lcd_set_bgcolor_listener(_instance, color);
if (external_Tron_send_lcd_set_bgcolor_listener != 0x0) external_Tron_send_lcd_set_bgcolor_listener(_instance, color);
;
}
void (*external_Tron_send_lcd_fillRect_listener)(struct Tron_Instance *, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t)= 0x0;
void register_external_Tron_send_lcd_fillRect_listener(void (*_listener)(struct Tron_Instance *, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t)){
external_Tron_send_lcd_fillRect_listener = _listener;
}
void (*Tron_send_lcd_fillRect_listener)(struct Tron_Instance *, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t)= 0x0;
void register_Tron_send_lcd_fillRect_listener(void (*_listener)(struct Tron_Instance *, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t)){
Tron_send_lcd_fillRect_listener = _listener;
}
void Tron_send_lcd_fillRect(struct Tron_Instance *_instance, uint16_t x, uint16_t y, uint16_t w, uint16_t l, uint16_t col){
if (Tron_send_lcd_fillRect_listener != 0x0) Tron_send_lcd_fillRect_listener(_instance, x, y, w, l, col);
if (external_Tron_send_lcd_fillRect_listener != 0x0) external_Tron_send_lcd_fillRect_listener(_instance, x, y, w, l, col);
;
}
void (*external_Tron_send_lcd_drawRect_listener)(struct Tron_Instance *, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t)= 0x0;
void register_external_Tron_send_lcd_drawRect_listener(void (*_listener)(struct Tron_Instance *, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t)){
external_Tron_send_lcd_drawRect_listener = _listener;
}
void (*Tron_send_lcd_drawRect_listener)(struct Tron_Instance *, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t)= 0x0;
void register_Tron_send_lcd_drawRect_listener(void (*_listener)(struct Tron_Instance *, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t)){
Tron_send_lcd_drawRect_listener = _listener;
}
void Tron_send_lcd_drawRect(struct Tron_Instance *_instance, uint16_t x, uint16_t y, uint16_t w, uint16_t l, uint16_t col){
if (Tron_send_lcd_drawRect_listener != 0x0) Tron_send_lcd_drawRect_listener(_instance, x, y, w, l, col);
if (external_Tron_send_lcd_drawRect_listener != 0x0) external_Tron_send_lcd_drawRect_listener(_instance, x, y, w, l, col);
;
}
void (*external_Tron_send_arduino_timer_start_listener)(struct Tron_Instance *, uint8_t, int16_t)= 0x0;
void register_external_Tron_send_arduino_timer_start_listener(void (*_listener)(struct Tron_Instance *, uint8_t, int16_t)){
external_Tron_send_arduino_timer_start_listener = _listener;
}
void (*Tron_send_arduino_timer_start_listener)(struct Tron_Instance *, uint8_t, int16_t)= 0x0;
void register_Tron_send_arduino_timer_start_listener(void (*_listener)(struct Tron_Instance *, uint8_t, int16_t)){
Tron_send_arduino_timer_start_listener = _listener;
}
void Tron_send_arduino_timer_start(struct Tron_Instance *_instance, uint8_t id, int16_t time){
if (Tron_send_arduino_timer_start_listener != 0x0) Tron_send_arduino_timer_start_listener(_instance, id, time);
if (external_Tron_send_arduino_timer_start_listener != 0x0) external_Tron_send_arduino_timer_start_listener(_instance, id, time);
;
}
void (*external_Tron_send_arduino_timer_cancel_listener)(struct Tron_Instance *, uint8_t)= 0x0;
void register_external_Tron_send_arduino_timer_cancel_listener(void (*_listener)(struct Tron_Instance *, uint8_t)){
external_Tron_send_arduino_timer_cancel_listener = _listener;
}
void (*Tron_send_arduino_timer_cancel_listener)(struct Tron_Instance *, uint8_t)= 0x0;
void register_Tron_send_arduino_timer_cancel_listener(void (*_listener)(struct Tron_Instance *, uint8_t)){
Tron_send_arduino_timer_cancel_listener = _listener;
}
void Tron_send_arduino_timer_cancel(struct Tron_Instance *_instance, uint8_t id){
if (Tron_send_arduino_timer_cancel_listener != 0x0) Tron_send_arduino_timer_cancel_listener(_instance, id);
if (external_Tron_send_arduino_timer_cancel_listener != 0x0) external_Tron_send_arduino_timer_cancel_listener(_instance, id);
;
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
struct Msg_Handler ** TronPort_receiver_list_head;
struct Msg_Handler ** TronPort_receiver_list_tail;
// Handler Array
struct Msg_Handler * TronPort_handlers;

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
 * Implementation for type : Adafruit_1_8pLCDShieldShield
 *****************************************************************************/


// BEGIN: Code from the c_global annotation Adafruit_1_8pLCDShieldShield

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

// END: Code from the c_global annotation Adafruit_1_8pLCDShieldShield

// Declaration of prototypes:
void Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_OnExit(int state, struct Adafruit_1_8pLCDShieldShield_Instance *_instance);
void Adafruit_1_8pLCDShieldShield_send_button_button_state_response(struct Adafruit_1_8pLCDShieldShield_Instance *_instance, uint8_t bstate);
void Adafruit_1_8pLCDShieldShield_send_button_button_state_change(struct Adafruit_1_8pLCDShieldShield_Instance *_instance, uint8_t bstate);
void Adafruit_1_8pLCDShieldShield_send_lcd_LCDready(struct Adafruit_1_8pLCDShieldShield_Instance *_instance);
// Declaration of functions:

// On Entry Actions:
void Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_OnEntry(int state, struct Adafruit_1_8pLCDShieldShield_Instance *_instance) {
switch(state) {
case ADAFRUIT_1_8PLCDSHIELDSHIELD_RGBLCDSHIELDSM_STATE:
_instance->Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State = ADAFRUIT_1_8PLCDSHIELDSHIELD_RGBLCDSHIELDSM_EMPTY_STATE;
Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_OnEntry(_instance->Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State, _instance);
break;
case ADAFRUIT_1_8PLCDSHIELDSHIELD_RGBLCDSHIELDSM_EMPTY_STATE:
break;
case ADAFRUIT_1_8PLCDSHIELDSHIELD_RGBLCDSHIELDSM_SETUP_STATE:
pinMode(_instance->Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_bpin__var, INPUT);
tft.initR(INITR_BLACKTAB);
			tft.fillScreen(0x0000);
			tft.setTextColor(0xFFFF);
break;
case ADAFRUIT_1_8PLCDSHIELDSHIELD_RGBLCDSHIELDSM_IDLE_STATE:
break;
default: break;
}
}

// On Exit Actions:
void Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_OnExit(int state, struct Adafruit_1_8pLCDShieldShield_Instance *_instance) {
switch(state) {
case ADAFRUIT_1_8PLCDSHIELDSHIELD_RGBLCDSHIELDSM_STATE:
Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_OnExit(_instance->Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State, _instance);
break;
case ADAFRUIT_1_8PLCDSHIELDSHIELD_RGBLCDSHIELDSM_EMPTY_STATE:
break;
case ADAFRUIT_1_8PLCDSHIELDSHIELD_RGBLCDSHIELDSM_SETUP_STATE:
break;
case ADAFRUIT_1_8PLCDSHIELDSHIELD_RGBLCDSHIELDSM_IDLE_STATE:
break;
default: break;
}
}

// Event Handlers for incoming messages:
void Adafruit_1_8pLCDShieldShield_handle_button_button_state(struct Adafruit_1_8pLCDShieldShield_Instance *_instance) {
uint8_t Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State_event_consumed = 0;
if (_instance->Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State == ADAFRUIT_1_8PLCDSHIELDSHIELD_RGBLCDSHIELDSM_IDLE_STATE) {
if (Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State_event_consumed == 0 && 1) {
Adafruit_1_8pLCDShieldShield_send_button_button_state_response(_instance, _instance->Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_Idle_bstate__var);
Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State_event_consumed = 1;
}
}
}
void Adafruit_1_8pLCDShieldShield_handle_arduino_ready(struct Adafruit_1_8pLCDShieldShield_Instance *_instance) {
uint8_t Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State_event_consumed = 0;
if (_instance->Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State == ADAFRUIT_1_8PLCDSHIELDSHIELD_RGBLCDSHIELDSM_EMPTY_STATE) {
if (Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State_event_consumed == 0 && 1) {
Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_OnExit(ADAFRUIT_1_8PLCDSHIELDSHIELD_RGBLCDSHIELDSM_EMPTY_STATE, _instance);
_instance->Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State = ADAFRUIT_1_8PLCDSHIELDSHIELD_RGBLCDSHIELDSM_SETUP_STATE;
Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_OnEntry(ADAFRUIT_1_8PLCDSHIELDSHIELD_RGBLCDSHIELDSM_SETUP_STATE, _instance);
Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State_event_consumed = 1;
}
}
}
void Adafruit_1_8pLCDShieldShield_handle_arduino_100ms_interrupt(struct Adafruit_1_8pLCDShieldShield_Instance *_instance) {
uint8_t Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State_event_consumed = 0;
if (_instance->Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State == ADAFRUIT_1_8PLCDSHIELDSHIELD_RGBLCDSHIELDSM_IDLE_STATE) {
if (Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State_event_consumed == 0 && 1) {
int joystickState = analogRead(_instance->Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_bpin__var);
				int bstatetmp = 0;
			    if (joystickState < 50) {bstatetmp = 1;}
			    else if (joystickState < 150) {bstatetmp = 2;}
			    else if (joystickState < 250) {bstatetmp = 3;}
			    else if (joystickState < 500) {bstatetmp = 4;}
			    else if (joystickState < 650) {bstatetmp = 5;}
if( !((_instance->Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_Idle_bstate__var == bstatetmp))) {
_instance->Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_Idle_bstate__var = bstatetmp;
Adafruit_1_8pLCDShieldShield_send_button_button_state_change(_instance, _instance->Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_Idle_bstate__var);

}
_instance->Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_Idle_bstate__var = bstatetmp;
Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State_event_consumed = 1;
}
}
}
void Adafruit_1_8pLCDShieldShield_handle_lcd_print_dec(struct Adafruit_1_8pLCDShieldShield_Instance *_instance, double num) {
uint8_t Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State_event_consumed = 0;
if (_instance->Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State == ADAFRUIT_1_8PLCDSHIELDSHIELD_RGBLCDSHIELDSM_IDLE_STATE) {
if (Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State_event_consumed == 0 && 1) {
tft.print(num);
Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State_event_consumed = 1;
}
}
}
void Adafruit_1_8pLCDShieldShield_handle_lcd_drawRect(struct Adafruit_1_8pLCDShieldShield_Instance *_instance, uint16_t x, uint16_t y, uint16_t w, uint16_t l, uint16_t col) {
uint8_t Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State_event_consumed = 0;
if (_instance->Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State == ADAFRUIT_1_8PLCDSHIELDSHIELD_RGBLCDSHIELDSM_IDLE_STATE) {
if (Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State_event_consumed == 0 && 1) {
tft.drawRect(x,y,w,l,col);
Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State_event_consumed = 1;
}
}
}
void Adafruit_1_8pLCDShieldShield_handle_lcd_set_cursor(struct Adafruit_1_8pLCDShieldShield_Instance *_instance, uint8_t c, uint8_t l) {
uint8_t Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State_event_consumed = 0;
if (_instance->Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State == ADAFRUIT_1_8PLCDSHIELDSHIELD_RGBLCDSHIELDSM_IDLE_STATE) {
if (Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State_event_consumed == 0 && 1) {
tft.setCursor(c, l);
Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State_event_consumed = 1;
}
}
}
void Adafruit_1_8pLCDShieldShield_handle_lcd_fillRect(struct Adafruit_1_8pLCDShieldShield_Instance *_instance, uint16_t x, uint16_t y, uint16_t w, uint16_t l, uint16_t col) {
uint8_t Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State_event_consumed = 0;
if (_instance->Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State == ADAFRUIT_1_8PLCDSHIELDSHIELD_RGBLCDSHIELDSM_IDLE_STATE) {
if (Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State_event_consumed == 0 && 1) {
tft.fillRect(x,y,w,l,col);
Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State_event_consumed = 1;
}
}
}
void Adafruit_1_8pLCDShieldShield_handle_lcd_print_num(struct Adafruit_1_8pLCDShieldShield_Instance *_instance, int16_t num) {
uint8_t Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State_event_consumed = 0;
if (_instance->Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State == ADAFRUIT_1_8PLCDSHIELDSHIELD_RGBLCDSHIELDSM_IDLE_STATE) {
if (Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State_event_consumed == 0 && 1) {
tft.print(num);
Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State_event_consumed = 1;
}
}
}
void Adafruit_1_8pLCDShieldShield_handle_lcd_clear(struct Adafruit_1_8pLCDShieldShield_Instance *_instance) {
uint8_t Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State_event_consumed = 0;
if (_instance->Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State == ADAFRUIT_1_8PLCDSHIELDSHIELD_RGBLCDSHIELDSM_IDLE_STATE) {
if (Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State_event_consumed == 0 && 1) {
tft.fillScreen(0x0000);
Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State_event_consumed = 1;
}
}
}
void Adafruit_1_8pLCDShieldShield_handle_lcd_print_str(struct Adafruit_1_8pLCDShieldShield_Instance *_instance, char * msg) {
uint8_t Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State_event_consumed = 0;
if (_instance->Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State == ADAFRUIT_1_8PLCDSHIELDSHIELD_RGBLCDSHIELDSM_IDLE_STATE) {
if (Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State_event_consumed == 0 && 1) {
tft.print(msg);
Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State_event_consumed = 1;
}
}
}
void Adafruit_1_8pLCDShieldShield_handle_empty_event(struct Adafruit_1_8pLCDShieldShield_Instance *_instance) {
if (_instance->Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State == ADAFRUIT_1_8PLCDSHIELDSHIELD_RGBLCDSHIELDSM_SETUP_STATE) {
if (1) {
Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_OnExit(ADAFRUIT_1_8PLCDSHIELDSHIELD_RGBLCDSHIELDSM_SETUP_STATE, _instance);
_instance->Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State = ADAFRUIT_1_8PLCDSHIELDSHIELD_RGBLCDSHIELDSM_IDLE_STATE;
Adafruit_1_8pLCDShieldShield_send_lcd_LCDready(_instance);
Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_OnEntry(ADAFRUIT_1_8PLCDSHIELDSHIELD_RGBLCDSHIELDSM_IDLE_STATE, _instance);
}
}
}

// Observers for outgoing messages:
void (*external_Adafruit_1_8pLCDShieldShield_send_button_button_state_response_listener)(struct Adafruit_1_8pLCDShieldShield_Instance *, uint8_t)= 0x0;
void register_external_Adafruit_1_8pLCDShieldShield_send_button_button_state_response_listener(void (*_listener)(struct Adafruit_1_8pLCDShieldShield_Instance *, uint8_t)){
external_Adafruit_1_8pLCDShieldShield_send_button_button_state_response_listener = _listener;
}
void (*Adafruit_1_8pLCDShieldShield_send_button_button_state_response_listener)(struct Adafruit_1_8pLCDShieldShield_Instance *, uint8_t)= 0x0;
void register_Adafruit_1_8pLCDShieldShield_send_button_button_state_response_listener(void (*_listener)(struct Adafruit_1_8pLCDShieldShield_Instance *, uint8_t)){
Adafruit_1_8pLCDShieldShield_send_button_button_state_response_listener = _listener;
}
void Adafruit_1_8pLCDShieldShield_send_button_button_state_response(struct Adafruit_1_8pLCDShieldShield_Instance *_instance, uint8_t bstate){
if (Adafruit_1_8pLCDShieldShield_send_button_button_state_response_listener != 0x0) Adafruit_1_8pLCDShieldShield_send_button_button_state_response_listener(_instance, bstate);
if (external_Adafruit_1_8pLCDShieldShield_send_button_button_state_response_listener != 0x0) external_Adafruit_1_8pLCDShieldShield_send_button_button_state_response_listener(_instance, bstate);
;
}
void (*external_Adafruit_1_8pLCDShieldShield_send_button_button_state_change_listener)(struct Adafruit_1_8pLCDShieldShield_Instance *, uint8_t)= 0x0;
void register_external_Adafruit_1_8pLCDShieldShield_send_button_button_state_change_listener(void (*_listener)(struct Adafruit_1_8pLCDShieldShield_Instance *, uint8_t)){
external_Adafruit_1_8pLCDShieldShield_send_button_button_state_change_listener = _listener;
}
void (*Adafruit_1_8pLCDShieldShield_send_button_button_state_change_listener)(struct Adafruit_1_8pLCDShieldShield_Instance *, uint8_t)= 0x0;
void register_Adafruit_1_8pLCDShieldShield_send_button_button_state_change_listener(void (*_listener)(struct Adafruit_1_8pLCDShieldShield_Instance *, uint8_t)){
Adafruit_1_8pLCDShieldShield_send_button_button_state_change_listener = _listener;
}
void Adafruit_1_8pLCDShieldShield_send_button_button_state_change(struct Adafruit_1_8pLCDShieldShield_Instance *_instance, uint8_t bstate){
if (Adafruit_1_8pLCDShieldShield_send_button_button_state_change_listener != 0x0) Adafruit_1_8pLCDShieldShield_send_button_button_state_change_listener(_instance, bstate);
if (external_Adafruit_1_8pLCDShieldShield_send_button_button_state_change_listener != 0x0) external_Adafruit_1_8pLCDShieldShield_send_button_button_state_change_listener(_instance, bstate);
;
}
void (*external_Adafruit_1_8pLCDShieldShield_send_lcd_LCDready_listener)(struct Adafruit_1_8pLCDShieldShield_Instance *)= 0x0;
void register_external_Adafruit_1_8pLCDShieldShield_send_lcd_LCDready_listener(void (*_listener)(struct Adafruit_1_8pLCDShieldShield_Instance *)){
external_Adafruit_1_8pLCDShieldShield_send_lcd_LCDready_listener = _listener;
}
void (*Adafruit_1_8pLCDShieldShield_send_lcd_LCDready_listener)(struct Adafruit_1_8pLCDShieldShield_Instance *)= 0x0;
void register_Adafruit_1_8pLCDShieldShield_send_lcd_LCDready_listener(void (*_listener)(struct Adafruit_1_8pLCDShieldShield_Instance *)){
Adafruit_1_8pLCDShieldShield_send_lcd_LCDready_listener = _listener;
}
void Adafruit_1_8pLCDShieldShield_send_lcd_LCDready(struct Adafruit_1_8pLCDShieldShield_Instance *_instance){
if (Adafruit_1_8pLCDShieldShield_send_lcd_LCDready_listener != 0x0) Adafruit_1_8pLCDShieldShield_send_lcd_LCDready_listener(_instance);
if (external_Adafruit_1_8pLCDShieldShield_send_lcd_LCDready_listener != 0x0) external_Adafruit_1_8pLCDShieldShield_send_lcd_LCDready_listener(_instance);
;
}
void (*external_Adafruit_1_8pLCDShieldShield_send_arduino_timer_start_listener)(struct Adafruit_1_8pLCDShieldShield_Instance *, uint8_t, int16_t)= 0x0;
void register_external_Adafruit_1_8pLCDShieldShield_send_arduino_timer_start_listener(void (*_listener)(struct Adafruit_1_8pLCDShieldShield_Instance *, uint8_t, int16_t)){
external_Adafruit_1_8pLCDShieldShield_send_arduino_timer_start_listener = _listener;
}
void (*Adafruit_1_8pLCDShieldShield_send_arduino_timer_start_listener)(struct Adafruit_1_8pLCDShieldShield_Instance *, uint8_t, int16_t)= 0x0;
void register_Adafruit_1_8pLCDShieldShield_send_arduino_timer_start_listener(void (*_listener)(struct Adafruit_1_8pLCDShieldShield_Instance *, uint8_t, int16_t)){
Adafruit_1_8pLCDShieldShield_send_arduino_timer_start_listener = _listener;
}
void Adafruit_1_8pLCDShieldShield_send_arduino_timer_start(struct Adafruit_1_8pLCDShieldShield_Instance *_instance, uint8_t id, int16_t time){
if (Adafruit_1_8pLCDShieldShield_send_arduino_timer_start_listener != 0x0) Adafruit_1_8pLCDShieldShield_send_arduino_timer_start_listener(_instance, id, time);
if (external_Adafruit_1_8pLCDShieldShield_send_arduino_timer_start_listener != 0x0) external_Adafruit_1_8pLCDShieldShield_send_arduino_timer_start_listener(_instance, id, time);
;
}
void (*external_Adafruit_1_8pLCDShieldShield_send_arduino_timer_cancel_listener)(struct Adafruit_1_8pLCDShieldShield_Instance *, uint8_t)= 0x0;
void register_external_Adafruit_1_8pLCDShieldShield_send_arduino_timer_cancel_listener(void (*_listener)(struct Adafruit_1_8pLCDShieldShield_Instance *, uint8_t)){
external_Adafruit_1_8pLCDShieldShield_send_arduino_timer_cancel_listener = _listener;
}
void (*Adafruit_1_8pLCDShieldShield_send_arduino_timer_cancel_listener)(struct Adafruit_1_8pLCDShieldShield_Instance *, uint8_t)= 0x0;
void register_Adafruit_1_8pLCDShieldShield_send_arduino_timer_cancel_listener(void (*_listener)(struct Adafruit_1_8pLCDShieldShield_Instance *, uint8_t)){
Adafruit_1_8pLCDShieldShield_send_arduino_timer_cancel_listener = _listener;
}
void Adafruit_1_8pLCDShieldShield_send_arduino_timer_cancel(struct Adafruit_1_8pLCDShieldShield_Instance *_instance, uint8_t id){
if (Adafruit_1_8pLCDShieldShield_send_arduino_timer_cancel_listener != 0x0) Adafruit_1_8pLCDShieldShield_send_arduino_timer_cancel_listener(_instance, id);
if (external_Adafruit_1_8pLCDShieldShield_send_arduino_timer_cancel_listener != 0x0) external_Adafruit_1_8pLCDShieldShield_send_arduino_timer_cancel_listener(_instance, id);
;
}



/*****************************************************************************
 * Implementation for type : ArduinoScheduler
 *****************************************************************************/


// BEGIN: Code from the c_global annotation ArduinoScheduler

struct ArduinoScheduler_Instance *_ArduinoScheduler_instance;
uint8_t interrupt_counter = 0;

// END: Code from the c_global annotation ArduinoScheduler

// Declaration of prototypes:
void ArduinoScheduler_ArduinoSchedulerStateChart_OnExit(int state, struct ArduinoScheduler_Instance *_instance);
void ArduinoScheduler_send_arduino_ready(struct ArduinoScheduler_Instance *_instance);
void ArduinoScheduler_send_arduino_4ms_interrupt(struct ArduinoScheduler_Instance *_instance);
void ArduinoScheduler_send_arduino_100ms_interrupt(struct ArduinoScheduler_Instance *_instance);
void ArduinoScheduler_send_arduino_1s_poll(struct ArduinoScheduler_Instance *_instance);
void ArduinoScheduler_send_arduino_timeout(struct ArduinoScheduler_Instance *_instance, uint8_t id);
void f_ArduinoScheduler_initialize_timer2(struct ArduinoScheduler_Instance *_instance);
SIGNAL(TIMER2_OVF_vect);
// Declaration of functions:
// Definition of function initialize_timer2
void f_ArduinoScheduler_initialize_timer2(struct ArduinoScheduler_Instance *_instance) {
// Store the instance in a global variable so that the interrupt routine can use it
		_ArduinoScheduler_instance = _instance;
		
		// Run timer2 interrupt up counting at 250kHz 
		 TCCR2A = 0;
		 TCCR2B = 1<<CS22 | 0<<CS21 | 0<<CS20;
		
		 //Timer2 Overflow Interrupt Enable
		 TIMSK2 |= 1<<TOIE2;
		
}
// Definition of function timer2_overflow_interrupt
SIGNAL(TIMER2_OVF_vect) {
TCNT2 = 5; // Leave 250 tics until overflow (1 overflow every 1ms)
interrupt_counter++;
if(interrupt_counter >= 99) {
ArduinoScheduler_send_arduino_100ms_interrupt(_ArduinoScheduler_instance);
interrupt_counter=0;

}
if(interrupt_counter % 4) {
ArduinoScheduler_send_arduino_4ms_interrupt(_ArduinoScheduler_instance);

}
}

// On Entry Actions:
void ArduinoScheduler_ArduinoSchedulerStateChart_OnEntry(int state, struct ArduinoScheduler_Instance *_instance) {
switch(state) {
case ARDUINOSCHEDULER_ARDUINOSCHEDULERSTATECHART_STATE:
_instance->ArduinoScheduler_ArduinoSchedulerStateChart_State = ARDUINOSCHEDULER_ARDUINOSCHEDULERSTATECHART_ACTIVE_STATE;
ArduinoScheduler_ArduinoSchedulerStateChart_OnEntry(_instance->ArduinoScheduler_ArduinoSchedulerStateChart_State, _instance);
break;
case ARDUINOSCHEDULER_ARDUINOSCHEDULERSTATECHART_ACTIVE_STATE:
break;
default: break;
}
}

// On Exit Actions:
void ArduinoScheduler_ArduinoSchedulerStateChart_OnExit(int state, struct ArduinoScheduler_Instance *_instance) {
switch(state) {
case ARDUINOSCHEDULER_ARDUINOSCHEDULERSTATECHART_STATE:
ArduinoScheduler_ArduinoSchedulerStateChart_OnExit(_instance->ArduinoScheduler_ArduinoSchedulerStateChart_State, _instance);
break;
case ARDUINOSCHEDULER_ARDUINOSCHEDULERSTATECHART_ACTIVE_STATE:
break;
default: break;
}
}

// Event Handlers for incoming messages:
void ArduinoScheduler_handle_polling_poll(struct ArduinoScheduler_Instance *_instance) {
uint8_t ArduinoScheduler_ArduinoSchedulerStateChart_State_event_consumed = 0;
if (1) {
;long tms = millis();
;uint8_t t = 0;
while(t < NB_SOFT_TIMERS) {
if(_instance->ArduinoScheduler_ArduinoSchedulerStateChart_timers__var[t]
 > 0 && _instance->ArduinoScheduler_ArduinoSchedulerStateChart_timers__var[t]
 < tms) {
_instance->ArduinoScheduler_ArduinoSchedulerStateChart_timers__var[t] = 0;
ArduinoScheduler_send_arduino_timeout(_instance, t);

}
t = t + 1;

}
if(_instance->ArduinoScheduler_ArduinoSchedulerStateChart_prev_1sec__var < tms) {
_instance->ArduinoScheduler_ArduinoSchedulerStateChart_prev_1sec__var = _instance->ArduinoScheduler_ArduinoSchedulerStateChart_prev_1sec__var + 1000;
ArduinoScheduler_send_arduino_1s_poll(_instance);

}
ArduinoScheduler_ArduinoSchedulerStateChart_State_event_consumed = 1;
}
}
void ArduinoScheduler_handle_polling_setup(struct ArduinoScheduler_Instance *_instance) {
uint8_t ArduinoScheduler_ArduinoSchedulerStateChart_State_event_consumed = 0;
if (1) {
f_ArduinoScheduler_initialize_timer2(_instance);
_instance->ArduinoScheduler_ArduinoSchedulerStateChart_prev_1sec__var = millis() + 1000;
ArduinoScheduler_send_arduino_ready(_instance);
ArduinoScheduler_ArduinoSchedulerStateChart_State_event_consumed = 1;
}
}
void ArduinoScheduler_handle_arduino_timer_cancel(struct ArduinoScheduler_Instance *_instance, uint8_t id) {
uint8_t ArduinoScheduler_ArduinoSchedulerStateChart_State_event_consumed = 0;
if (1) {
if(id < NB_SOFT_TIMERS) {
_instance->ArduinoScheduler_ArduinoSchedulerStateChart_timers__var[id] = 0;

}
ArduinoScheduler_ArduinoSchedulerStateChart_State_event_consumed = 1;
}
}
void ArduinoScheduler_handle_arduino_timer_start(struct ArduinoScheduler_Instance *_instance, uint8_t id, int16_t time) {
uint8_t ArduinoScheduler_ArduinoSchedulerStateChart_State_event_consumed = 0;
if (1) {
if(id < NB_SOFT_TIMERS) {
_instance->ArduinoScheduler_ArduinoSchedulerStateChart_timers__var[id] = millis() + time - 1;

}
ArduinoScheduler_ArduinoSchedulerStateChart_State_event_consumed = 1;
}
}

// Observers for outgoing messages:
void (*external_ArduinoScheduler_send_arduino_ready_listener)(struct ArduinoScheduler_Instance *)= 0x0;
void register_external_ArduinoScheduler_send_arduino_ready_listener(void (*_listener)(struct ArduinoScheduler_Instance *)){
external_ArduinoScheduler_send_arduino_ready_listener = _listener;
}
void (*ArduinoScheduler_send_arduino_ready_listener)(struct ArduinoScheduler_Instance *)= 0x0;
void register_ArduinoScheduler_send_arduino_ready_listener(void (*_listener)(struct ArduinoScheduler_Instance *)){
ArduinoScheduler_send_arduino_ready_listener = _listener;
}
void ArduinoScheduler_send_arduino_ready(struct ArduinoScheduler_Instance *_instance){
if (ArduinoScheduler_send_arduino_ready_listener != 0x0) ArduinoScheduler_send_arduino_ready_listener(_instance);
if (external_ArduinoScheduler_send_arduino_ready_listener != 0x0) external_ArduinoScheduler_send_arduino_ready_listener(_instance);
;
}
void (*external_ArduinoScheduler_send_arduino_4ms_interrupt_listener)(struct ArduinoScheduler_Instance *)= 0x0;
void register_external_ArduinoScheduler_send_arduino_4ms_interrupt_listener(void (*_listener)(struct ArduinoScheduler_Instance *)){
external_ArduinoScheduler_send_arduino_4ms_interrupt_listener = _listener;
}
void (*ArduinoScheduler_send_arduino_4ms_interrupt_listener)(struct ArduinoScheduler_Instance *)= 0x0;
void register_ArduinoScheduler_send_arduino_4ms_interrupt_listener(void (*_listener)(struct ArduinoScheduler_Instance *)){
ArduinoScheduler_send_arduino_4ms_interrupt_listener = _listener;
}
void ArduinoScheduler_send_arduino_4ms_interrupt(struct ArduinoScheduler_Instance *_instance){
if (ArduinoScheduler_send_arduino_4ms_interrupt_listener != 0x0) ArduinoScheduler_send_arduino_4ms_interrupt_listener(_instance);
if (external_ArduinoScheduler_send_arduino_4ms_interrupt_listener != 0x0) external_ArduinoScheduler_send_arduino_4ms_interrupt_listener(_instance);
;
}
void (*external_ArduinoScheduler_send_arduino_100ms_interrupt_listener)(struct ArduinoScheduler_Instance *)= 0x0;
void register_external_ArduinoScheduler_send_arduino_100ms_interrupt_listener(void (*_listener)(struct ArduinoScheduler_Instance *)){
external_ArduinoScheduler_send_arduino_100ms_interrupt_listener = _listener;
}
void (*ArduinoScheduler_send_arduino_100ms_interrupt_listener)(struct ArduinoScheduler_Instance *)= 0x0;
void register_ArduinoScheduler_send_arduino_100ms_interrupt_listener(void (*_listener)(struct ArduinoScheduler_Instance *)){
ArduinoScheduler_send_arduino_100ms_interrupt_listener = _listener;
}
void ArduinoScheduler_send_arduino_100ms_interrupt(struct ArduinoScheduler_Instance *_instance){
if (ArduinoScheduler_send_arduino_100ms_interrupt_listener != 0x0) ArduinoScheduler_send_arduino_100ms_interrupt_listener(_instance);
if (external_ArduinoScheduler_send_arduino_100ms_interrupt_listener != 0x0) external_ArduinoScheduler_send_arduino_100ms_interrupt_listener(_instance);
;
}
void (*external_ArduinoScheduler_send_arduino_1s_poll_listener)(struct ArduinoScheduler_Instance *)= 0x0;
void register_external_ArduinoScheduler_send_arduino_1s_poll_listener(void (*_listener)(struct ArduinoScheduler_Instance *)){
external_ArduinoScheduler_send_arduino_1s_poll_listener = _listener;
}
void (*ArduinoScheduler_send_arduino_1s_poll_listener)(struct ArduinoScheduler_Instance *)= 0x0;
void register_ArduinoScheduler_send_arduino_1s_poll_listener(void (*_listener)(struct ArduinoScheduler_Instance *)){
ArduinoScheduler_send_arduino_1s_poll_listener = _listener;
}
void ArduinoScheduler_send_arduino_1s_poll(struct ArduinoScheduler_Instance *_instance){
if (ArduinoScheduler_send_arduino_1s_poll_listener != 0x0) ArduinoScheduler_send_arduino_1s_poll_listener(_instance);
if (external_ArduinoScheduler_send_arduino_1s_poll_listener != 0x0) external_ArduinoScheduler_send_arduino_1s_poll_listener(_instance);
;
}
void (*external_ArduinoScheduler_send_arduino_timeout_listener)(struct ArduinoScheduler_Instance *, uint8_t)= 0x0;
void register_external_ArduinoScheduler_send_arduino_timeout_listener(void (*_listener)(struct ArduinoScheduler_Instance *, uint8_t)){
external_ArduinoScheduler_send_arduino_timeout_listener = _listener;
}
void (*ArduinoScheduler_send_arduino_timeout_listener)(struct ArduinoScheduler_Instance *, uint8_t)= 0x0;
void register_ArduinoScheduler_send_arduino_timeout_listener(void (*_listener)(struct ArduinoScheduler_Instance *, uint8_t)){
ArduinoScheduler_send_arduino_timeout_listener = _listener;
}
void ArduinoScheduler_send_arduino_timeout(struct ArduinoScheduler_Instance *_instance, uint8_t id){
if (ArduinoScheduler_send_arduino_timeout_listener != 0x0) ArduinoScheduler_send_arduino_timeout_listener(_instance, id);
if (external_ArduinoScheduler_send_arduino_timeout_listener != 0x0) external_ArduinoScheduler_send_arduino_timeout_listener(_instance, id);
;
}






/*****************************************************************************
 * Definitions for configuration : TronCfg
 *****************************************************************************/

//Declaration of connexion array
#define NB_MAX_CONNEXION 9
struct Msg_Handler * TronCfg_receivers[NB_MAX_CONNEXION];

//Declaration of instance variables
//Instance TronCfg_arduinoScheduler
struct ArduinoScheduler_Instance TronCfg_arduinoScheduler_var;
struct Msg_Handler TronCfg_arduinoScheduler_arduino_handlers;
uint16_t TronCfg_arduinoScheduler_arduino_msgs[2];
void * TronCfg_arduinoScheduler_arduino_handlers_tab[2];

struct Msg_Handler TronCfg_arduinoScheduler_polling_handlers;
uint16_t TronCfg_arduinoScheduler_polling_msgs[2];
void * TronCfg_arduinoScheduler_polling_handlers_tab[2];

//Instance TronCfg_myLCD
struct Adafruit_1_8pLCDShieldShield_Instance TronCfg_myLCD_var;
struct Msg_Handler TronCfg_myLCD_button_handlers;
uint16_t TronCfg_myLCD_button_msgs[1];
void * TronCfg_myLCD_button_handlers_tab[1];

struct Msg_Handler TronCfg_myLCD_lcd_handlers;
uint16_t TronCfg_myLCD_lcd_msgs[8];
void * TronCfg_myLCD_lcd_handlers_tab[8];

struct Msg_Handler TronCfg_myLCD_arduino_handlers;
uint16_t TronCfg_myLCD_arduino_msgs[5];
void * TronCfg_myLCD_arduino_handlers_tab[5];

//Instance TronCfg_tron
struct Tron_Instance TronCfg_tron_var;
struct Msg_Handler TronCfg_tron_TronPort_handlers;
uint16_t TronCfg_tron_TronPort_msgs[7];
void * TronCfg_tron_TronPort_handlers_tab[7];

struct Msg_Handler TronCfg_tron_button_handlers;
uint16_t TronCfg_tron_button_msgs[2];
void * TronCfg_tron_button_handlers_tab[2];

struct Msg_Handler TronCfg_tron_lcd_handlers;
uint16_t TronCfg_tron_lcd_msgs[1];
void * TronCfg_tron_lcd_handlers_tab[1];

struct Msg_Handler TronCfg_tron_arduino_handlers;
uint16_t TronCfg_tron_arduino_msgs[5];
void * TronCfg_tron_arduino_handlers_tab[5];


// Enqueue of messages Adafruit_1_8pLCDShieldShield::button::button_state_change
void enqueue_Adafruit_1_8pLCDShieldShield_send_button_button_state_change(struct Adafruit_1_8pLCDShieldShield_Instance *_instance, uint8_t bstate){
if ( fifo_byte_available() > 5 ) {

_fifo_enqueue( (1 >> 8) & 0xFF );
_fifo_enqueue( 1 & 0xFF );

// ID of the source port of the instance
_fifo_enqueue( (_instance->id_button >> 8) & 0xFF );
_fifo_enqueue( _instance->id_button & 0xFF );

// parameter bstate
union u_bstate_t {
uint8_t p;
byte bytebuffer[1];
} u_bstate;
u_bstate.p = bstate;
_fifo_enqueue( u_bstate.bytebuffer[0] & 0xFF );
}
}
// Enqueue of messages Adafruit_1_8pLCDShieldShield::button::button_state_response
void enqueue_Adafruit_1_8pLCDShieldShield_send_button_button_state_response(struct Adafruit_1_8pLCDShieldShield_Instance *_instance, uint8_t bstate){
if ( fifo_byte_available() > 5 ) {

_fifo_enqueue( (2 >> 8) & 0xFF );
_fifo_enqueue( 2 & 0xFF );

// ID of the source port of the instance
_fifo_enqueue( (_instance->id_button >> 8) & 0xFF );
_fifo_enqueue( _instance->id_button & 0xFF );

// parameter bstate
union u_bstate_t {
uint8_t p;
byte bytebuffer[1];
} u_bstate;
u_bstate.p = bstate;
_fifo_enqueue( u_bstate.bytebuffer[0] & 0xFF );
}
}
// Enqueue of messages Tron::button::button_state
void enqueue_Tron_send_button_button_state(struct Tron_Instance *_instance){
if ( fifo_byte_available() > 4 ) {

_fifo_enqueue( (3 >> 8) & 0xFF );
_fifo_enqueue( 3 & 0xFF );

// ID of the source port of the instance
_fifo_enqueue( (_instance->id_button >> 8) & 0xFF );
_fifo_enqueue( _instance->id_button & 0xFF );
}
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
if (sender == TronCfg_tron_var.id_TronPort) {
executor::executor_dispatch_tronReady(TronCfg_tron_var.TronPort_receiver_list_head, TronCfg_tron_var.TronPort_receiver_list_tail, param_id);}
if (sender == Serial_instance.listener_id) {
executor::executor_dispatch_tronReady(Serial_instance.TronPort_receiver_list_head,Serial_instance.TronPort_receiver_list_tail, param_id);}
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
if (sender == TronCfg_tron_var.id_TronPort) {
executor::executor_dispatch_iHaveID(TronCfg_tron_var.TronPort_receiver_list_head, TronCfg_tron_var.TronPort_receiver_list_tail, param_id);}
if (sender == Serial_instance.listener_id) {
executor::executor_dispatch_iHaveID(Serial_instance.TronPort_receiver_list_head,Serial_instance.TronPort_receiver_list_tail, param_id);}
}

//Dynamic dispatcher for message ready
void dispatch_ready(uint16_t sender) {
struct executor {
static void executor_dispatch_ready(struct Msg_Handler ** head, struct Msg_Handler ** tail) {
struct Msg_Handler ** cur = head;
while (cur != NULL) {
   void (*handler)(void *) = NULL;
   int i;
   for(i = 0; i < (**cur).nb_msg; i++) {
       if((**cur).msg[i] == 4) {
           handler = (void (*) (void *)) (**cur).msg_handler[i];
           break;
       }
   }
   if(handler != NULL) {
       handler((**cur).instance);
}
   if(cur == tail){
       cur = NULL;}
   else {
   cur++;}
}
}
};
if (sender == TronCfg_arduinoScheduler_var.id_arduino) {
executor::executor_dispatch_ready(TronCfg_arduinoScheduler_var.arduino_receiver_list_head, TronCfg_arduinoScheduler_var.arduino_receiver_list_tail);}
}

//Dynamic dispatcher for message 1s_poll
void dispatch_1s_poll(uint16_t sender) {
struct executor {
static void executor_dispatch_1s_poll(struct Msg_Handler ** head, struct Msg_Handler ** tail) {
struct Msg_Handler ** cur = head;
while (cur != NULL) {
   void (*handler)(void *) = NULL;
   int i;
   for(i = 0; i < (**cur).nb_msg; i++) {
       if((**cur).msg[i] == 5) {
           handler = (void (*) (void *)) (**cur).msg_handler[i];
           break;
       }
   }
   if(handler != NULL) {
       handler((**cur).instance);
}
   if(cur == tail){
       cur = NULL;}
   else {
   cur++;}
}
}
};
if (sender == TronCfg_arduinoScheduler_var.id_arduino) {
executor::executor_dispatch_1s_poll(TronCfg_arduinoScheduler_var.arduino_receiver_list_head, TronCfg_arduinoScheduler_var.arduino_receiver_list_tail);}
}

//Dynamic dispatcher for message set_cursor
void dispatch_set_cursor(uint16_t sender, uint8_t param_c, uint8_t param_l) {
struct executor {
static void executor_dispatch_set_cursor(struct Msg_Handler ** head, struct Msg_Handler ** tail, uint8_t param_c, uint8_t param_l) {
struct Msg_Handler ** cur = head;
while (cur != NULL) {
   void (*handler)(void *, uint8_t param_c, uint8_t param_l) = NULL;
   int i;
   for(i = 0; i < (**cur).nb_msg; i++) {
       if((**cur).msg[i] == 6) {
           handler = (void (*) (void *, uint8_t, uint8_t)) (**cur).msg_handler[i];
           break;
       }
   }
   if(handler != NULL) {
       handler((**cur).instance, param_c, param_l);
}
   if(cur == tail){
       cur = NULL;}
   else {
   cur++;}
}
}
};
if (sender == TronCfg_tron_var.id_lcd) {
executor::executor_dispatch_set_cursor(TronCfg_tron_var.lcd_receiver_list_head, TronCfg_tron_var.lcd_receiver_list_tail, param_c, param_l);}
}

//Dynamic dispatcher for message fillRect
void dispatch_fillRect(uint16_t sender, uint16_t param_x, uint16_t param_y, uint16_t param_w, uint16_t param_l, uint16_t param_col) {
struct executor {
static void executor_dispatch_fillRect(struct Msg_Handler ** head, struct Msg_Handler ** tail, uint16_t param_x, uint16_t param_y, uint16_t param_w, uint16_t param_l, uint16_t param_col) {
struct Msg_Handler ** cur = head;
while (cur != NULL) {
   void (*handler)(void *, uint16_t param_x, uint16_t param_y, uint16_t param_w, uint16_t param_l, uint16_t param_col) = NULL;
   int i;
   for(i = 0; i < (**cur).nb_msg; i++) {
       if((**cur).msg[i] == 7) {
           handler = (void (*) (void *, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t)) (**cur).msg_handler[i];
           break;
       }
   }
   if(handler != NULL) {
       handler((**cur).instance, param_x, param_y, param_w, param_l, param_col);
}
   if(cur == tail){
       cur = NULL;}
   else {
   cur++;}
}
}
};
if (sender == TronCfg_tron_var.id_lcd) {
executor::executor_dispatch_fillRect(TronCfg_tron_var.lcd_receiver_list_head, TronCfg_tron_var.lcd_receiver_list_tail, param_x, param_y, param_w, param_l, param_col);}
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
if (sender == TronCfg_tron_var.id_TronPort) {
executor::executor_dispatch_loose(TronCfg_tron_var.TronPort_receiver_list_head, TronCfg_tron_var.TronPort_receiver_list_tail, param_id);}
if (sender == Serial_instance.listener_id) {
executor::executor_dispatch_loose(Serial_instance.TronPort_receiver_list_head,Serial_instance.TronPort_receiver_list_tail, param_id);}
}

//Dynamic dispatcher for message set_bgcolor
void dispatch_set_bgcolor(uint16_t sender, uint8_t param_color) {
struct executor {
static void executor_dispatch_set_bgcolor(struct Msg_Handler ** head, struct Msg_Handler ** tail, uint8_t param_color) {
struct Msg_Handler ** cur = head;
while (cur != NULL) {
   void (*handler)(void *, uint8_t param_color) = NULL;
   int i;
   for(i = 0; i < (**cur).nb_msg; i++) {
       if((**cur).msg[i] == 8) {
           handler = (void (*) (void *, uint8_t)) (**cur).msg_handler[i];
           break;
       }
   }
   if(handler != NULL) {
       handler((**cur).instance, param_color);
}
   if(cur == tail){
       cur = NULL;}
   else {
   cur++;}
}
}
};
if (sender == TronCfg_tron_var.id_lcd) {
executor::executor_dispatch_set_bgcolor(TronCfg_tron_var.lcd_receiver_list_head, TronCfg_tron_var.lcd_receiver_list_tail, param_color);}
}

//Dynamic dispatcher for message button_state_change
void dispatch_button_state_change(uint16_t sender, uint8_t param_bstate) {
struct executor {
static void executor_dispatch_button_state_change(struct Msg_Handler ** head, struct Msg_Handler ** tail, uint8_t param_bstate) {
struct Msg_Handler ** cur = head;
while (cur != NULL) {
   void (*handler)(void *, uint8_t param_bstate) = NULL;
   int i;
   for(i = 0; i < (**cur).nb_msg; i++) {
       if((**cur).msg[i] == 1) {
           handler = (void (*) (void *, uint8_t)) (**cur).msg_handler[i];
           break;
       }
   }
   if(handler != NULL) {
       handler((**cur).instance, param_bstate);
}
   if(cur == tail){
       cur = NULL;}
   else {
   cur++;}
}
}
};
if (sender == TronCfg_myLCD_var.id_button) {
executor::executor_dispatch_button_state_change(TronCfg_myLCD_var.button_receiver_list_head, TronCfg_myLCD_var.button_receiver_list_tail, param_bstate);}
}

//Dynamic dispatcher for message LCDready
void dispatch_LCDready(uint16_t sender) {
struct executor {
static void executor_dispatch_LCDready(struct Msg_Handler ** head, struct Msg_Handler ** tail) {
struct Msg_Handler ** cur = head;
while (cur != NULL) {
   void (*handler)(void *) = NULL;
   int i;
   for(i = 0; i < (**cur).nb_msg; i++) {
       if((**cur).msg[i] == 9) {
           handler = (void (*) (void *)) (**cur).msg_handler[i];
           break;
       }
   }
   if(handler != NULL) {
       handler((**cur).instance);
}
   if(cur == tail){
       cur = NULL;}
   else {
   cur++;}
}
}
};
if (sender == TronCfg_myLCD_var.id_lcd) {
executor::executor_dispatch_LCDready(TronCfg_myLCD_var.lcd_receiver_list_head, TronCfg_myLCD_var.lcd_receiver_list_tail);}
}

//Dynamic dispatcher for message timer_cancel
void dispatch_timer_cancel(uint16_t sender, uint8_t param_id) {
struct executor {
static void executor_dispatch_timer_cancel(struct Msg_Handler ** head, struct Msg_Handler ** tail, uint8_t param_id) {
struct Msg_Handler ** cur = head;
while (cur != NULL) {
   void (*handler)(void *, uint8_t param_id) = NULL;
   int i;
   for(i = 0; i < (**cur).nb_msg; i++) {
       if((**cur).msg[i] == 10) {
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
if (sender == TronCfg_myLCD_var.id_arduino) {
executor::executor_dispatch_timer_cancel(TronCfg_myLCD_var.arduino_receiver_list_head, TronCfg_myLCD_var.arduino_receiver_list_tail, param_id);}
if (sender == TronCfg_tron_var.id_arduino) {
executor::executor_dispatch_timer_cancel(TronCfg_tron_var.arduino_receiver_list_head, TronCfg_tron_var.arduino_receiver_list_tail, param_id);}
}

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
if (sender == TronCfg_tron_var.id_TronPort) {
executor::executor_dispatch_mayIHaveID(TronCfg_tron_var.TronPort_receiver_list_head, TronCfg_tron_var.TronPort_receiver_list_tail, param_id);}
if (sender == Serial_instance.listener_id) {
executor::executor_dispatch_mayIHaveID(Serial_instance.TronPort_receiver_list_head,Serial_instance.TronPort_receiver_list_tail, param_id);}
}

//Dynamic dispatcher for message button_state
void dispatch_button_state(uint16_t sender) {
struct executor {
static void executor_dispatch_button_state(struct Msg_Handler ** head, struct Msg_Handler ** tail) {
struct Msg_Handler ** cur = head;
while (cur != NULL) {
   void (*handler)(void *) = NULL;
   int i;
   for(i = 0; i < (**cur).nb_msg; i++) {
       if((**cur).msg[i] == 3) {
           handler = (void (*) (void *)) (**cur).msg_handler[i];
           break;
       }
   }
   if(handler != NULL) {
       handler((**cur).instance);
}
   if(cur == tail){
       cur = NULL;}
   else {
   cur++;}
}
}
};
if (sender == TronCfg_tron_var.id_button) {
executor::executor_dispatch_button_state(TronCfg_tron_var.button_receiver_list_head, TronCfg_tron_var.button_receiver_list_tail);}
}

//Dynamic dispatcher for message 100ms_interrupt
void dispatch_100ms_interrupt(uint16_t sender) {
struct executor {
static void executor_dispatch_100ms_interrupt(struct Msg_Handler ** head, struct Msg_Handler ** tail) {
struct Msg_Handler ** cur = head;
while (cur != NULL) {
   void (*handler)(void *) = NULL;
   int i;
   for(i = 0; i < (**cur).nb_msg; i++) {
       if((**cur).msg[i] == 11) {
           handler = (void (*) (void *)) (**cur).msg_handler[i];
           break;
       }
   }
   if(handler != NULL) {
       handler((**cur).instance);
}
   if(cur == tail){
       cur = NULL;}
   else {
   cur++;}
}
}
};
if (sender == TronCfg_arduinoScheduler_var.id_arduino) {
executor::executor_dispatch_100ms_interrupt(TronCfg_arduinoScheduler_var.arduino_receiver_list_head, TronCfg_arduinoScheduler_var.arduino_receiver_list_tail);}
}

//Dynamic dispatcher for message clear
void dispatch_clear(uint16_t sender) {
struct executor {
static void executor_dispatch_clear(struct Msg_Handler ** head, struct Msg_Handler ** tail) {
struct Msg_Handler ** cur = head;
while (cur != NULL) {
   void (*handler)(void *) = NULL;
   int i;
   for(i = 0; i < (**cur).nb_msg; i++) {
       if((**cur).msg[i] == 12) {
           handler = (void (*) (void *)) (**cur).msg_handler[i];
           break;
       }
   }
   if(handler != NULL) {
       handler((**cur).instance);
}
   if(cur == tail){
       cur = NULL;}
   else {
   cur++;}
}
}
};
if (sender == TronCfg_tron_var.id_lcd) {
executor::executor_dispatch_clear(TronCfg_tron_var.lcd_receiver_list_head, TronCfg_tron_var.lcd_receiver_list_tail);}
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
if (sender == TronCfg_tron_var.id_TronPort) {
executor::executor_dispatch_tronGo(TronCfg_tron_var.TronPort_receiver_list_head, TronCfg_tron_var.TronPort_receiver_list_tail, param_nbID);}
if (sender == Serial_instance.listener_id) {
executor::executor_dispatch_tronGo(Serial_instance.TronPort_receiver_list_head,Serial_instance.TronPort_receiver_list_tail, param_nbID);}
}

//Dynamic dispatcher for message print_dec
void dispatch_print_dec(uint16_t sender, double param_num) {
struct executor {
static void executor_dispatch_print_dec(struct Msg_Handler ** head, struct Msg_Handler ** tail, double param_num) {
struct Msg_Handler ** cur = head;
while (cur != NULL) {
   void (*handler)(void *, double param_num) = NULL;
   int i;
   for(i = 0; i < (**cur).nb_msg; i++) {
       if((**cur).msg[i] == 13) {
           handler = (void (*) (void *, double)) (**cur).msg_handler[i];
           break;
       }
   }
   if(handler != NULL) {
       handler((**cur).instance, param_num);
}
   if(cur == tail){
       cur = NULL;}
   else {
   cur++;}
}
}
};
if (sender == TronCfg_tron_var.id_lcd) {
executor::executor_dispatch_print_dec(TronCfg_tron_var.lcd_receiver_list_head, TronCfg_tron_var.lcd_receiver_list_tail, param_num);}
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
if (sender == TronCfg_tron_var.id_TronPort) {
executor::executor_dispatch_hasID(TronCfg_tron_var.TronPort_receiver_list_head, TronCfg_tron_var.TronPort_receiver_list_tail, param_id);}
if (sender == Serial_instance.listener_id) {
executor::executor_dispatch_hasID(Serial_instance.TronPort_receiver_list_head,Serial_instance.TronPort_receiver_list_tail, param_id);}
}

//Dynamic dispatcher for message timeout
void dispatch_timeout(uint16_t sender, uint8_t param_id) {
struct executor {
static void executor_dispatch_timeout(struct Msg_Handler ** head, struct Msg_Handler ** tail, uint8_t param_id) {
struct Msg_Handler ** cur = head;
while (cur != NULL) {
   void (*handler)(void *, uint8_t param_id) = NULL;
   int i;
   for(i = 0; i < (**cur).nb_msg; i++) {
       if((**cur).msg[i] == 14) {
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
if (sender == TronCfg_arduinoScheduler_var.id_arduino) {
executor::executor_dispatch_timeout(TronCfg_arduinoScheduler_var.arduino_receiver_list_head, TronCfg_arduinoScheduler_var.arduino_receiver_list_tail, param_id);}
}

//Dynamic dispatcher for message button_state_response
void dispatch_button_state_response(uint16_t sender, uint8_t param_bstate) {
struct executor {
static void executor_dispatch_button_state_response(struct Msg_Handler ** head, struct Msg_Handler ** tail, uint8_t param_bstate) {
struct Msg_Handler ** cur = head;
while (cur != NULL) {
   void (*handler)(void *, uint8_t param_bstate) = NULL;
   int i;
   for(i = 0; i < (**cur).nb_msg; i++) {
       if((**cur).msg[i] == 2) {
           handler = (void (*) (void *, uint8_t)) (**cur).msg_handler[i];
           break;
       }
   }
   if(handler != NULL) {
       handler((**cur).instance, param_bstate);
}
   if(cur == tail){
       cur = NULL;}
   else {
   cur++;}
}
}
};
if (sender == TronCfg_myLCD_var.id_button) {
executor::executor_dispatch_button_state_response(TronCfg_myLCD_var.button_receiver_list_head, TronCfg_myLCD_var.button_receiver_list_tail, param_bstate);}
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
if (sender == TronCfg_tron_var.id_TronPort) {
executor::executor_dispatch_addHead(TronCfg_tron_var.TronPort_receiver_list_head, TronCfg_tron_var.TronPort_receiver_list_tail, param_x, param_y, param_id);}
if (sender == Serial_instance.listener_id) {
executor::executor_dispatch_addHead(Serial_instance.TronPort_receiver_list_head,Serial_instance.TronPort_receiver_list_tail, param_x, param_y, param_id);}
}

//Dynamic dispatcher for message 4ms_interrupt
void dispatch_4ms_interrupt(uint16_t sender) {
struct executor {
static void executor_dispatch_4ms_interrupt(struct Msg_Handler ** head, struct Msg_Handler ** tail) {
struct Msg_Handler ** cur = head;
while (cur != NULL) {
   void (*handler)(void *) = NULL;
   int i;
   for(i = 0; i < (**cur).nb_msg; i++) {
       if((**cur).msg[i] == 15) {
           handler = (void (*) (void *)) (**cur).msg_handler[i];
           break;
       }
   }
   if(handler != NULL) {
       handler((**cur).instance);
}
   if(cur == tail){
       cur = NULL;}
   else {
   cur++;}
}
}
};
if (sender == TronCfg_arduinoScheduler_var.id_arduino) {
executor::executor_dispatch_4ms_interrupt(TronCfg_arduinoScheduler_var.arduino_receiver_list_head, TronCfg_arduinoScheduler_var.arduino_receiver_list_tail);}
}

//Dynamic dispatcher for message drawRect
void dispatch_drawRect(uint16_t sender, uint16_t param_x, uint16_t param_y, uint16_t param_w, uint16_t param_l, uint16_t param_col) {
struct executor {
static void executor_dispatch_drawRect(struct Msg_Handler ** head, struct Msg_Handler ** tail, uint16_t param_x, uint16_t param_y, uint16_t param_w, uint16_t param_l, uint16_t param_col) {
struct Msg_Handler ** cur = head;
while (cur != NULL) {
   void (*handler)(void *, uint16_t param_x, uint16_t param_y, uint16_t param_w, uint16_t param_l, uint16_t param_col) = NULL;
   int i;
   for(i = 0; i < (**cur).nb_msg; i++) {
       if((**cur).msg[i] == 16) {
           handler = (void (*) (void *, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t)) (**cur).msg_handler[i];
           break;
       }
   }
   if(handler != NULL) {
       handler((**cur).instance, param_x, param_y, param_w, param_l, param_col);
}
   if(cur == tail){
       cur = NULL;}
   else {
   cur++;}
}
}
};
if (sender == TronCfg_tron_var.id_lcd) {
executor::executor_dispatch_drawRect(TronCfg_tron_var.lcd_receiver_list_head, TronCfg_tron_var.lcd_receiver_list_tail, param_x, param_y, param_w, param_l, param_col);}
}

//Dynamic dispatcher for message print_num
void dispatch_print_num(uint16_t sender, int16_t param_num) {
struct executor {
static void executor_dispatch_print_num(struct Msg_Handler ** head, struct Msg_Handler ** tail, int16_t param_num) {
struct Msg_Handler ** cur = head;
while (cur != NULL) {
   void (*handler)(void *, int16_t param_num) = NULL;
   int i;
   for(i = 0; i < (**cur).nb_msg; i++) {
       if((**cur).msg[i] == 17) {
           handler = (void (*) (void *, int16_t)) (**cur).msg_handler[i];
           break;
       }
   }
   if(handler != NULL) {
       handler((**cur).instance, param_num);
}
   if(cur == tail){
       cur = NULL;}
   else {
   cur++;}
}
}
};
if (sender == TronCfg_tron_var.id_lcd) {
executor::executor_dispatch_print_num(TronCfg_tron_var.lcd_receiver_list_head, TronCfg_tron_var.lcd_receiver_list_tail, param_num);}
}

//Dynamic dispatcher for message timer_start
void dispatch_timer_start(uint16_t sender, uint8_t param_id, int16_t param_time) {
struct executor {
static void executor_dispatch_timer_start(struct Msg_Handler ** head, struct Msg_Handler ** tail, uint8_t param_id, int16_t param_time) {
struct Msg_Handler ** cur = head;
while (cur != NULL) {
   void (*handler)(void *, uint8_t param_id, int16_t param_time) = NULL;
   int i;
   for(i = 0; i < (**cur).nb_msg; i++) {
       if((**cur).msg[i] == 18) {
           handler = (void (*) (void *, uint8_t, int16_t)) (**cur).msg_handler[i];
           break;
       }
   }
   if(handler != NULL) {
       handler((**cur).instance, param_id, param_time);
}
   if(cur == tail){
       cur = NULL;}
   else {
   cur++;}
}
}
};
if (sender == TronCfg_tron_var.id_arduino) {
executor::executor_dispatch_timer_start(TronCfg_tron_var.arduino_receiver_list_head, TronCfg_tron_var.arduino_receiver_list_tail, param_id, param_time);}
if (sender == TronCfg_myLCD_var.id_arduino) {
executor::executor_dispatch_timer_start(TronCfg_myLCD_var.arduino_receiver_list_head, TronCfg_myLCD_var.arduino_receiver_list_tail, param_id, param_time);}
}

//Dynamic dispatcher for message print_str
void dispatch_print_str(uint16_t sender, char * param_msg) {
struct executor {
static void executor_dispatch_print_str(struct Msg_Handler ** head, struct Msg_Handler ** tail, char * param_msg) {
struct Msg_Handler ** cur = head;
while (cur != NULL) {
   void (*handler)(void *, char * param_msg) = NULL;
   int i;
   for(i = 0; i < (**cur).nb_msg; i++) {
       if((**cur).msg[i] == 19) {
           handler = (void (*) (void *, char *)) (**cur).msg_handler[i];
           break;
       }
   }
   if(handler != NULL) {
       handler((**cur).instance, param_msg);
}
   if(cur == tail){
       cur = NULL;}
   else {
   cur++;}
}
}
};
if (sender == TronCfg_tron_var.id_lcd) {
executor::executor_dispatch_print_str(TronCfg_tron_var.lcd_receiver_list_head, TronCfg_tron_var.lcd_receiver_list_tail, param_msg);}
}
void sync_dispatch_Tron_send_lcd_print_dec(struct Tron_Instance *_instance, double num){
dispatch_print_dec(_instance->id_lcd, num);
}
void sync_dispatch_ArduinoScheduler_send_arduino_ready(struct ArduinoScheduler_Instance *_instance){
dispatch_ready(_instance->id_arduino);
}
void sync_dispatch_ArduinoScheduler_send_arduino_1s_poll(struct ArduinoScheduler_Instance *_instance){
dispatch_1s_poll(_instance->id_arduino);
}
void sync_dispatch_Tron_send_lcd_set_cursor(struct Tron_Instance *_instance, uint8_t c, uint8_t l){
dispatch_set_cursor(_instance->id_lcd, c, l);
}
void sync_dispatch_Tron_send_lcd_fillRect(struct Tron_Instance *_instance, uint16_t x, uint16_t y, uint16_t w, uint16_t l, uint16_t col){
dispatch_fillRect(_instance->id_lcd, x, y, w, l, col);
}
void sync_dispatch_ArduinoScheduler_send_arduino_timeout(struct ArduinoScheduler_Instance *_instance, uint8_t id){
dispatch_timeout(_instance->id_arduino, id);
}
void sync_dispatch_Tron_send_lcd_set_bgcolor(struct Tron_Instance *_instance, uint8_t color){
dispatch_set_bgcolor(_instance->id_lcd, color);
}
void sync_dispatch_Adafruit_1_8pLCDShieldShield_send_button_button_state_response(struct Adafruit_1_8pLCDShieldShield_Instance *_instance, uint8_t bstate){
dispatch_button_state_response(_instance->id_button, bstate);
}
void sync_dispatch_Adafruit_1_8pLCDShieldShield_send_button_button_state_change(struct Adafruit_1_8pLCDShieldShield_Instance *_instance, uint8_t bstate){
dispatch_button_state_change(_instance->id_button, bstate);
}
void sync_dispatch_ArduinoScheduler_send_arduino_4ms_interrupt(struct ArduinoScheduler_Instance *_instance){
dispatch_4ms_interrupt(_instance->id_arduino);
}
void sync_dispatch_Tron_send_lcd_drawRect(struct Tron_Instance *_instance, uint16_t x, uint16_t y, uint16_t w, uint16_t l, uint16_t col){
dispatch_drawRect(_instance->id_lcd, x, y, w, l, col);
}
void sync_dispatch_Adafruit_1_8pLCDShieldShield_send_lcd_LCDready(struct Adafruit_1_8pLCDShieldShield_Instance *_instance){
dispatch_LCDready(_instance->id_lcd);
}
void sync_dispatch_Tron_send_arduino_timer_cancel(struct Tron_Instance *_instance, uint8_t id){
dispatch_timer_cancel(_instance->id_arduino, id);
}
void sync_dispatch_Adafruit_1_8pLCDShieldShield_send_arduino_timer_cancel(struct Adafruit_1_8pLCDShieldShield_Instance *_instance, uint8_t id){
dispatch_timer_cancel(_instance->id_arduino, id);
}
void sync_dispatch_Tron_send_lcd_print_num(struct Tron_Instance *_instance, int16_t num){
dispatch_print_num(_instance->id_lcd, num);
}
void sync_dispatch_Tron_send_arduino_timer_start(struct Tron_Instance *_instance, uint8_t id, int16_t time){
dispatch_timer_start(_instance->id_arduino, id, time);
}
void sync_dispatch_Adafruit_1_8pLCDShieldShield_send_arduino_timer_start(struct Adafruit_1_8pLCDShieldShield_Instance *_instance, uint8_t id, int16_t time){
dispatch_timer_start(_instance->id_arduino, id, time);
}
void sync_dispatch_ArduinoScheduler_send_arduino_100ms_interrupt(struct ArduinoScheduler_Instance *_instance){
dispatch_100ms_interrupt(_instance->id_arduino);
}
void sync_dispatch_Tron_send_lcd_clear(struct Tron_Instance *_instance){
dispatch_clear(_instance->id_lcd);
}
void sync_dispatch_Tron_send_lcd_print_str(struct Tron_Instance *_instance, char * msg){
dispatch_print_str(_instance->id_lcd, msg);
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
case 1:
while (mbufi < 3) mbuf[mbufi++] = fifo_dequeue();
union u_button_state_change_bstate_t {
uint8_t p;
byte bytebuffer[1];
} u_button_state_change_bstate;
u_button_state_change_bstate.bytebuffer[0] = mbuf[2];
dispatch_button_state_change((mbuf[0] << 8) + mbuf[1] /* instance port*/,
 u_button_state_change_bstate.p /* bstate */ );
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
case 3:
while (mbufi < 2) mbuf[mbufi++] = fifo_dequeue();
dispatch_button_state((mbuf[0] << 8) + mbuf[1] /* instance port*/);
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
case 2:
while (mbufi < 3) mbuf[mbufi++] = fifo_dequeue();
union u_button_state_response_bstate_t {
uint8_t p;
byte bytebuffer[1];
} u_button_state_response_bstate;
u_button_state_response_bstate.bytebuffer[0] = mbuf[2];
dispatch_button_state_response((mbuf[0] << 8) + mbuf[1] /* instance port*/,
 u_button_state_response_bstate.p /* bstate */ );
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
}
}

// Forwarding of messages Tron::TronPort::addHead
void forward_Tron_send_TronPort_addHead(struct Tron_Instance *_instance, uint8_t x, uint8_t y, uint8_t id){
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

// Forwarding of messages Tron::TronPort::loose
void forward_Tron_send_TronPort_loose(struct Tron_Instance *_instance, uint8_t id){
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

// Forwarding of messages Tron::TronPort::tronReady
void forward_Tron_send_TronPort_tronReady(struct Tron_Instance *_instance, uint8_t id){
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

// Forwarding of messages Tron::TronPort::tronGo
void forward_Tron_send_TronPort_tronGo(struct Tron_Instance *_instance, uint8_t nbID){
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

// Forwarding of messages Tron::TronPort::hasID
void forward_Tron_send_TronPort_hasID(struct Tron_Instance *_instance, uint8_t id){
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

// Forwarding of messages Tron::TronPort::iHaveID
void forward_Tron_send_TronPort_iHaveID(struct Tron_Instance *_instance, uint8_t id){
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

// Forwarding of messages Tron::TronPort::mayIHaveID
void forward_Tron_send_TronPort_mayIHaveID(struct Tron_Instance *_instance, uint8_t id){
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


//external Message enqueue
void externalMessageEnqueue(uint8_t * msg, uint8_t msgSize, uint16_t listener_id) {
if ((msgSize >= 2) && (msg != NULL)) {
uint8_t msgSizeOK = 0;
switch(msg[0] * 256 + msg[1]) {
case 53:
if(msgSize == 3) {
msgSizeOK = 1;}
break;
case 51:
if(msgSize == 3) {
msgSizeOK = 1;}
break;
case 50:
if(msgSize == 3) {
msgSizeOK = 1;}
break;
case 52:
if(msgSize == 3) {
msgSizeOK = 1;}
break;
case 56:
if(msgSize == 3) {
msgSizeOK = 1;}
break;
case 55:
if(msgSize == 5) {
msgSizeOK = 1;}
break;
case 54:
if(msgSize == 3) {
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

void initialize_configuration_TronCfg() {
// Initialize connectors
register_external_Tron_send_TronPort_addHead_listener(forward_Tron_send_TronPort_addHead);
register_external_Tron_send_TronPort_loose_listener(forward_Tron_send_TronPort_loose);
register_external_Tron_send_TronPort_tronReady_listener(forward_Tron_send_TronPort_tronReady);
register_external_Tron_send_TronPort_tronGo_listener(forward_Tron_send_TronPort_tronGo);
register_external_Tron_send_TronPort_hasID_listener(forward_Tron_send_TronPort_hasID);
register_external_Tron_send_TronPort_iHaveID_listener(forward_Tron_send_TronPort_iHaveID);
register_external_Tron_send_TronPort_mayIHaveID_listener(forward_Tron_send_TronPort_mayIHaveID);
register_Tron_send_button_button_state_listener(enqueue_Tron_send_button_button_state);
register_Tron_send_lcd_print_num_listener(sync_dispatch_Tron_send_lcd_print_num);
register_Tron_send_lcd_print_dec_listener(sync_dispatch_Tron_send_lcd_print_dec);
register_Tron_send_lcd_print_str_listener(sync_dispatch_Tron_send_lcd_print_str);
register_Tron_send_lcd_clear_listener(sync_dispatch_Tron_send_lcd_clear);
register_Tron_send_lcd_set_cursor_listener(sync_dispatch_Tron_send_lcd_set_cursor);
register_Tron_send_lcd_set_bgcolor_listener(sync_dispatch_Tron_send_lcd_set_bgcolor);
register_Tron_send_lcd_fillRect_listener(sync_dispatch_Tron_send_lcd_fillRect);
register_Tron_send_lcd_drawRect_listener(sync_dispatch_Tron_send_lcd_drawRect);
register_Tron_send_arduino_timer_start_listener(sync_dispatch_Tron_send_arduino_timer_start);
register_Tron_send_arduino_timer_cancel_listener(sync_dispatch_Tron_send_arduino_timer_cancel);
register_ArduinoScheduler_send_arduino_ready_listener(sync_dispatch_ArduinoScheduler_send_arduino_ready);
register_ArduinoScheduler_send_arduino_4ms_interrupt_listener(sync_dispatch_ArduinoScheduler_send_arduino_4ms_interrupt);
register_ArduinoScheduler_send_arduino_100ms_interrupt_listener(sync_dispatch_ArduinoScheduler_send_arduino_100ms_interrupt);
register_ArduinoScheduler_send_arduino_1s_poll_listener(sync_dispatch_ArduinoScheduler_send_arduino_1s_poll);
register_ArduinoScheduler_send_arduino_timeout_listener(sync_dispatch_ArduinoScheduler_send_arduino_timeout);
register_Adafruit_1_8pLCDShieldShield_send_button_button_state_response_listener(enqueue_Adafruit_1_8pLCDShieldShield_send_button_button_state_response);
register_Adafruit_1_8pLCDShieldShield_send_button_button_state_change_listener(enqueue_Adafruit_1_8pLCDShieldShield_send_button_button_state_change);
register_Adafruit_1_8pLCDShieldShield_send_lcd_LCDready_listener(sync_dispatch_Adafruit_1_8pLCDShieldShield_send_lcd_LCDready);
register_Adafruit_1_8pLCDShieldShield_send_arduino_timer_start_listener(sync_dispatch_Adafruit_1_8pLCDShieldShield_send_arduino_timer_start);
register_Adafruit_1_8pLCDShieldShield_send_arduino_timer_cancel_listener(sync_dispatch_Adafruit_1_8pLCDShieldShield_send_arduino_timer_cancel);

// Init the ID, state variables and properties for instance TronCfg_myLCD
TronCfg_myLCD_var.id_button = add_instance( (void*) &TronCfg_myLCD_var);
TronCfg_myLCD_button_msgs[0] = 3;
TronCfg_myLCD_button_handlers_tab[0] = (void*) &Adafruit_1_8pLCDShieldShield_handle_button_button_state;
TronCfg_myLCD_button_handlers.nb_msg = 1;
TronCfg_myLCD_button_handlers.msg = (uint16_t *) &TronCfg_myLCD_button_msgs;
TronCfg_myLCD_button_handlers.msg_handler = (void **) &TronCfg_myLCD_button_handlers_tab;
TronCfg_myLCD_button_handlers.instance = &TronCfg_myLCD_var;
TronCfg_myLCD_var.button_handlers = &TronCfg_myLCD_button_handlers;
TronCfg_receivers[0] = &TronCfg_tron_button_handlers;
TronCfg_myLCD_var.button_receiver_list_head = &TronCfg_receivers[0];
TronCfg_myLCD_var.button_receiver_list_tail = &TronCfg_receivers[0];
TronCfg_myLCD_var.id_lcd = add_instance( (void*) &TronCfg_myLCD_var);
TronCfg_myLCD_lcd_msgs[0] = 17;
TronCfg_myLCD_lcd_handlers_tab[0] = (void*) &Adafruit_1_8pLCDShieldShield_handle_lcd_print_num;
TronCfg_myLCD_lcd_msgs[1] = 13;
TronCfg_myLCD_lcd_handlers_tab[1] = (void*) &Adafruit_1_8pLCDShieldShield_handle_lcd_print_dec;
TronCfg_myLCD_lcd_msgs[2] = 19;
TronCfg_myLCD_lcd_handlers_tab[2] = (void*) &Adafruit_1_8pLCDShieldShield_handle_lcd_print_str;
TronCfg_myLCD_lcd_msgs[3] = 12;
TronCfg_myLCD_lcd_handlers_tab[3] = (void*) &Adafruit_1_8pLCDShieldShield_handle_lcd_clear;
TronCfg_myLCD_lcd_msgs[4] = 6;
TronCfg_myLCD_lcd_handlers_tab[4] = (void*) &Adafruit_1_8pLCDShieldShield_handle_lcd_set_cursor;
TronCfg_myLCD_lcd_msgs[5] = 8;
TronCfg_myLCD_lcd_handlers_tab[5] = NULL;
TronCfg_myLCD_lcd_msgs[6] = 7;
TronCfg_myLCD_lcd_handlers_tab[6] = (void*) &Adafruit_1_8pLCDShieldShield_handle_lcd_fillRect;
TronCfg_myLCD_lcd_msgs[7] = 16;
TronCfg_myLCD_lcd_handlers_tab[7] = (void*) &Adafruit_1_8pLCDShieldShield_handle_lcd_drawRect;
TronCfg_myLCD_lcd_handlers.nb_msg = 8;
TronCfg_myLCD_lcd_handlers.msg = (uint16_t *) &TronCfg_myLCD_lcd_msgs;
TronCfg_myLCD_lcd_handlers.msg_handler = (void **) &TronCfg_myLCD_lcd_handlers_tab;
TronCfg_myLCD_lcd_handlers.instance = &TronCfg_myLCD_var;
TronCfg_myLCD_var.lcd_handlers = &TronCfg_myLCD_lcd_handlers;
TronCfg_receivers[1] = &TronCfg_tron_lcd_handlers;
TronCfg_myLCD_var.lcd_receiver_list_head = &TronCfg_receivers[1];
TronCfg_myLCD_var.lcd_receiver_list_tail = &TronCfg_receivers[1];
TronCfg_myLCD_var.id_arduino = add_instance( (void*) &TronCfg_myLCD_var);
TronCfg_myLCD_arduino_msgs[0] = 4;
TronCfg_myLCD_arduino_handlers_tab[0] = (void*) &Adafruit_1_8pLCDShieldShield_handle_arduino_ready;
TronCfg_myLCD_arduino_msgs[1] = 15;
TronCfg_myLCD_arduino_handlers_tab[1] = NULL;
TronCfg_myLCD_arduino_msgs[2] = 11;
TronCfg_myLCD_arduino_handlers_tab[2] = (void*) &Adafruit_1_8pLCDShieldShield_handle_arduino_100ms_interrupt;
TronCfg_myLCD_arduino_msgs[3] = 5;
TronCfg_myLCD_arduino_handlers_tab[3] = NULL;
TronCfg_myLCD_arduino_msgs[4] = 14;
TronCfg_myLCD_arduino_handlers_tab[4] = NULL;
TronCfg_myLCD_arduino_handlers.nb_msg = 5;
TronCfg_myLCD_arduino_handlers.msg = (uint16_t *) &TronCfg_myLCD_arduino_msgs;
TronCfg_myLCD_arduino_handlers.msg_handler = (void **) &TronCfg_myLCD_arduino_handlers_tab;
TronCfg_myLCD_arduino_handlers.instance = &TronCfg_myLCD_var;
TronCfg_myLCD_var.arduino_handlers = &TronCfg_myLCD_arduino_handlers;
TronCfg_receivers[2] = &TronCfg_arduinoScheduler_arduino_handlers;
TronCfg_myLCD_var.arduino_receiver_list_head = &TronCfg_receivers[2];
TronCfg_myLCD_var.arduino_receiver_list_tail = &TronCfg_receivers[2];
TronCfg_myLCD_var.Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_State = ADAFRUIT_1_8PLCDSHIELDSHIELD_RGBLCDSHIELDSM_EMPTY_STATE;
TronCfg_myLCD_var.Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_bpin__var = 3;

// Init the ID, state variables and properties for instance TronCfg_tron
TronCfg_tron_var.id_TronPort = add_instance( (void*) &TronCfg_tron_var);
TronCfg_tron_TronPort_msgs[0] = 55;
TronCfg_tron_TronPort_handlers_tab[0] = (void*) &Tron_handle_TronPort_addHead;
TronCfg_tron_TronPort_msgs[1] = 56;
TronCfg_tron_TronPort_handlers_tab[1] = (void*) &Tron_handle_TronPort_loose;
TronCfg_tron_TronPort_msgs[2] = 53;
TronCfg_tron_TronPort_handlers_tab[2] = (void*) &Tron_handle_TronPort_tronReady;
TronCfg_tron_TronPort_msgs[3] = 54;
TronCfg_tron_TronPort_handlers_tab[3] = (void*) &Tron_handle_TronPort_tronGo;
TronCfg_tron_TronPort_msgs[4] = 50;
TronCfg_tron_TronPort_handlers_tab[4] = (void*) &Tron_handle_TronPort_hasID;
TronCfg_tron_TronPort_msgs[5] = 51;
TronCfg_tron_TronPort_handlers_tab[5] = (void*) &Tron_handle_TronPort_iHaveID;
TronCfg_tron_TronPort_msgs[6] = 52;
TronCfg_tron_TronPort_handlers_tab[6] = (void*) &Tron_handle_TronPort_mayIHaveID;
TronCfg_tron_TronPort_handlers.nb_msg = 7;
TronCfg_tron_TronPort_handlers.msg = (uint16_t *) &TronCfg_tron_TronPort_msgs;
TronCfg_tron_TronPort_handlers.msg_handler = (void **) &TronCfg_tron_TronPort_handlers_tab;
TronCfg_tron_TronPort_handlers.instance = &TronCfg_tron_var;
TronCfg_tron_var.TronPort_handlers = &TronCfg_tron_TronPort_handlers;
TronCfg_tron_var.TronPort_receiver_list_head = NULL;
TronCfg_tron_var.TronPort_receiver_list_tail = &TronCfg_receivers[3];
TronCfg_tron_var.id_button = add_instance( (void*) &TronCfg_tron_var);
TronCfg_tron_button_msgs[0] = 2;
TronCfg_tron_button_handlers_tab[0] = NULL;
TronCfg_tron_button_msgs[1] = 1;
TronCfg_tron_button_handlers_tab[1] = (void*) &Tron_handle_button_button_state_change;
TronCfg_tron_button_handlers.nb_msg = 2;
TronCfg_tron_button_handlers.msg = (uint16_t *) &TronCfg_tron_button_msgs;
TronCfg_tron_button_handlers.msg_handler = (void **) &TronCfg_tron_button_handlers_tab;
TronCfg_tron_button_handlers.instance = &TronCfg_tron_var;
TronCfg_tron_var.button_handlers = &TronCfg_tron_button_handlers;
TronCfg_receivers[3] = &TronCfg_myLCD_button_handlers;
TronCfg_tron_var.button_receiver_list_head = &TronCfg_receivers[3];
TronCfg_tron_var.button_receiver_list_tail = &TronCfg_receivers[3];
TronCfg_tron_var.id_lcd = add_instance( (void*) &TronCfg_tron_var);
TronCfg_tron_lcd_msgs[0] = 9;
TronCfg_tron_lcd_handlers_tab[0] = (void*) &Tron_handle_lcd_LCDready;
TronCfg_tron_lcd_handlers.nb_msg = 1;
TronCfg_tron_lcd_handlers.msg = (uint16_t *) &TronCfg_tron_lcd_msgs;
TronCfg_tron_lcd_handlers.msg_handler = (void **) &TronCfg_tron_lcd_handlers_tab;
TronCfg_tron_lcd_handlers.instance = &TronCfg_tron_var;
TronCfg_tron_var.lcd_handlers = &TronCfg_tron_lcd_handlers;
TronCfg_receivers[4] = &TronCfg_myLCD_lcd_handlers;
TronCfg_tron_var.lcd_receiver_list_head = &TronCfg_receivers[4];
TronCfg_tron_var.lcd_receiver_list_tail = &TronCfg_receivers[4];
TronCfg_tron_var.id_arduino = add_instance( (void*) &TronCfg_tron_var);
TronCfg_tron_arduino_msgs[0] = 4;
TronCfg_tron_arduino_handlers_tab[0] = NULL;
TronCfg_tron_arduino_msgs[1] = 15;
TronCfg_tron_arduino_handlers_tab[1] = NULL;
TronCfg_tron_arduino_msgs[2] = 11;
TronCfg_tron_arduino_handlers_tab[2] = (void*) &Tron_handle_arduino_100ms_interrupt;
TronCfg_tron_arduino_msgs[3] = 5;
TronCfg_tron_arduino_handlers_tab[3] = NULL;
TronCfg_tron_arduino_msgs[4] = 14;
TronCfg_tron_arduino_handlers_tab[4] = (void*) &Tron_handle_arduino_timeout;
TronCfg_tron_arduino_handlers.nb_msg = 5;
TronCfg_tron_arduino_handlers.msg = (uint16_t *) &TronCfg_tron_arduino_msgs;
TronCfg_tron_arduino_handlers.msg_handler = (void **) &TronCfg_tron_arduino_handlers_tab;
TronCfg_tron_arduino_handlers.instance = &TronCfg_tron_var;
TronCfg_tron_var.arduino_handlers = &TronCfg_tron_arduino_handlers;
TronCfg_receivers[5] = &TronCfg_arduinoScheduler_arduino_handlers;
TronCfg_tron_var.arduino_receiver_list_head = &TronCfg_receivers[5];
TronCfg_tron_var.arduino_receiver_list_tail = &TronCfg_receivers[5];
TronCfg_tron_var.Tron_TronStateChart_State = TRON_TRONSTATECHART_INIT_STATE;
TronCfg_tron_var.Tron_myID__var = 0;
TronCfg_tron_var.Tron_nbID__var = 1;
TronCfg_tron_var.Tron_curID__var = 0;
TronCfg_tron_var.Tron_nbReady__var = 0;
TronCfg_tron_var.Tron_headX__var = 10;
TronCfg_tron_var.Tron_headY__var = 10;
TronCfg_tron_var.Tron_tailX__var = 10;
TronCfg_tron_var.Tron_tailY__var = 10;
TronCfg_tron_var.Tron_headIndex__var = 0;
TronCfg_tron_var.Tron_headIndexQuarter__var = 0;
TronCfg_tron_var.Tron_lost__var = 0;
TronCfg_tron_var.Tron_won__var = 0;
TronCfg_tron_var.Tron_timer__var = 1;
TronCfg_tron_var.Tron_speed__var = 600;
TronCfg_tron_var.Tron_dirBuff__var = B00000001;

// Init the ID, state variables and properties for instance TronCfg_arduinoScheduler
TronCfg_arduinoScheduler_var.id_arduino = add_instance( (void*) &TronCfg_arduinoScheduler_var);
TronCfg_arduinoScheduler_arduino_msgs[0] = 18;
TronCfg_arduinoScheduler_arduino_handlers_tab[0] = (void*) &ArduinoScheduler_handle_arduino_timer_start;
TronCfg_arduinoScheduler_arduino_msgs[1] = 10;
TronCfg_arduinoScheduler_arduino_handlers_tab[1] = (void*) &ArduinoScheduler_handle_arduino_timer_cancel;
TronCfg_arduinoScheduler_arduino_handlers.nb_msg = 2;
TronCfg_arduinoScheduler_arduino_handlers.msg = (uint16_t *) &TronCfg_arduinoScheduler_arduino_msgs;
TronCfg_arduinoScheduler_arduino_handlers.msg_handler = (void **) &TronCfg_arduinoScheduler_arduino_handlers_tab;
TronCfg_arduinoScheduler_arduino_handlers.instance = &TronCfg_arduinoScheduler_var;
TronCfg_arduinoScheduler_var.arduino_handlers = &TronCfg_arduinoScheduler_arduino_handlers;
TronCfg_receivers[6] = &TronCfg_tron_arduino_handlers;
TronCfg_receivers[7] = &TronCfg_myLCD_arduino_handlers;
TronCfg_arduinoScheduler_var.arduino_receiver_list_head = &TronCfg_receivers[6];
TronCfg_arduinoScheduler_var.arduino_receiver_list_tail = &TronCfg_receivers[7];
TronCfg_arduinoScheduler_var.id_polling = add_instance( (void*) &TronCfg_arduinoScheduler_var);
TronCfg_arduinoScheduler_polling_msgs[0] = 20;
TronCfg_arduinoScheduler_polling_handlers_tab[0] = (void*) &ArduinoScheduler_handle_polling_setup;
TronCfg_arduinoScheduler_polling_msgs[1] = 21;
TronCfg_arduinoScheduler_polling_handlers_tab[1] = (void*) &ArduinoScheduler_handle_polling_poll;
TronCfg_arduinoScheduler_polling_handlers.nb_msg = 2;
TronCfg_arduinoScheduler_polling_handlers.msg = (uint16_t *) &TronCfg_arduinoScheduler_polling_msgs;
TronCfg_arduinoScheduler_polling_handlers.msg_handler = (void **) &TronCfg_arduinoScheduler_polling_handlers_tab;
TronCfg_arduinoScheduler_polling_handlers.instance = &TronCfg_arduinoScheduler_var;
TronCfg_arduinoScheduler_var.polling_handlers = &TronCfg_arduinoScheduler_polling_handlers;
TronCfg_arduinoScheduler_var.ArduinoScheduler_ArduinoSchedulerStateChart_State = ARDUINOSCHEDULER_ARDUINOSCHEDULERSTATECHART_ACTIVE_STATE;
TronCfg_arduinoScheduler_var.ArduinoScheduler_interrupt_counter__var = 0;

// Init the ID, state variables and properties for external connector Serial
Serial_instance.listener_id = add_instance( (void*) &Serial_instance);
TronCfg_receivers[8] = &TronCfg_tron_TronPort_handlers;
Serial_instance.TronPort_receiver_list_head = &TronCfg_receivers[8];
Serial_instance.TronPort_receiver_list_tail = &TronCfg_receivers[8];

// Network Initilization 
//Serial:
Serial_setup();


// End Network Initilization 

Adafruit_1_8pLCDShieldShield_RGBLCDShieldSM_OnEntry(ADAFRUIT_1_8PLCDSHIELDSHIELD_RGBLCDSHIELDSM_STATE, &TronCfg_myLCD_var);
Tron_TronStateChart_OnEntry(TRON_TRONSTATECHART_STATE, &TronCfg_tron_var);
ArduinoScheduler_ArduinoSchedulerStateChart_OnEntry(ARDUINOSCHEDULER_ARDUINOSCHEDULERSTATECHART_STATE, &TronCfg_arduinoScheduler_var);
}




void setup() {
initialize_configuration_TronCfg();
ArduinoScheduler_handle_polling_setup(&TronCfg_arduinoScheduler_var);

}

void loop() {
ArduinoScheduler_handle_polling_poll(&TronCfg_arduinoScheduler_var);

// Network Listener
Serial_read();
Adafruit_1_8pLCDShieldShield_handle_empty_event(&TronCfg_myLCD_var);

    processMessageQueue();
}
