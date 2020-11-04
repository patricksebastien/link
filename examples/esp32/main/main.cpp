#include <ableton/Link.hpp>
#include <driver/gpio.h>
#include <driver/timer.h>
#include <esp_event.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <nvs_flash.h>
#include <protocol_examples_common.h>
#include "driver/uart.h"
#include <stdio.h>

#define LED GPIO_NUM_2
#define PRINT_LINK_STATE false
#define USE_START_STOP
#define USE_I2C_DISPLAY 


// Serial midi
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_TXD  (GPIO_NUM_4)
#define ECHO_TEST_RXD  (GPIO_NUM_5)
#define BUF_SIZE (1024)
#define MIDI_TIMING_CLOCK 0xF8
#define MIDI_START 0xFA
#define MIDI_STOP 0xFC
#define MIDI_SONG_POSITION_POINTER 0xF2

#if defined USE_START_STOP
extern "C" {
#include "freertos/queue.h"
#include <string.h>
#include <stdlib.h>

#define GPIO_INPUT_IO_0     GPIO_NUM_4
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_INPUT_IO_0)  
#define ESP_INTR_FLAG_DEFAULT 0

static xQueueHandle gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level((gpio_num_t)io_num));
        }
    }
}
}
#endif

#if defined USE_I2C_DISPLAY
#define PRINT_LINK_STATE true  // je vais changer ceci pour afficher uniquement lorsqu'il y a un changement de BPM
extern "C" {
#include <stdbool.h>
#include "ssd1306.h"
#include "ssd1306_draw.h"
#include "ssd1306_font.h"
#include "ssd1306_default_if.h"


static const int I2CDisplayAddress = 0x3C;
static const int I2CDisplayWidth = 128;
static const int I2CDisplayHeight = 64;
static const int I2CResetPin = -1;

struct SSD1306_Device I2CDisplay;

bool DefaultBusInit( void ) {  
        assert( SSD1306_I2CMasterInitDefault( ) == true );
        assert( SSD1306_I2CMasterAttachDisplayDefault( &I2CDisplay, I2CDisplayWidth, I2CDisplayHeight, I2CDisplayAddress, I2CResetPin ) == true );
    return true;
}

void FontDisplayTask( void* Param ) {
    struct SSD1306_Device* Display = ( struct SSD1306_Device* ) Param;

    if ( Param != NULL ) {

        SSD1306_SetFont( Display, &Font_droid_sans_mono_13x24);
        SSD1306_Clear( Display, SSD_COLOR_BLACK );
        SSD1306_FontDrawAnchoredString( Display, TextAnchor_North, "BPM", SSD_COLOR_WHITE );
        SSD1306_FontDrawAnchoredString( Display, TextAnchor_Center, "66.6", SSD_COLOR_WHITE );
        SSD1306_Update( Display );
    }

    vTaskDelete( NULL );
}
    
void SetupDemo( struct SSD1306_Device* DisplayHandle, const struct SSD1306_FontDef* Font ) {
    SSD1306_Clear( DisplayHandle, SSD_COLOR_BLACK );
    SSD1306_SetFont( DisplayHandle, Font );
}

void SayHello( struct SSD1306_Device* DisplayHandle, const char* HelloText ) {
    SSD1306_FontDrawAnchoredString( DisplayHandle, TextAnchor_Center, HelloText, SSD_COLOR_WHITE );
    SSD1306_Update( DisplayHandle );
}
    
    
} 
#endif

unsigned int if_nametoindex(const char* ifName)
{
  return 0;
}

char* if_indextoname(unsigned int ifIndex, char* ifName)
{
  return nullptr;
}

void IRAM_ATTR timer_group0_isr(void* userParam)
{
  static BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  TIMERG0.int_clr_timers.t0 = 1;
  TIMERG0.hw_timer[0].config.alarm_en = 1;

  xSemaphoreGiveFromISR(userParam, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken)
  {
    portYIELD_FROM_ISR();
  }
}

void timerGroup0Init(int timerPeriodUS, void* userParam)
{
  timer_config_t config = {.alarm_en = TIMER_ALARM_EN,
    .counter_en = TIMER_PAUSE,
    .intr_type = TIMER_INTR_LEVEL,
    .counter_dir = TIMER_COUNT_UP,
    .auto_reload = TIMER_AUTORELOAD_EN,
    .divider = 80};

  timer_init(TIMER_GROUP_0, TIMER_0, &config);
  timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
  timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, timerPeriodUS);
  timer_enable_intr(TIMER_GROUP_0, TIMER_0);
  timer_isr_register(TIMER_GROUP_0, TIMER_0, &timer_group0_isr, userParam, 0, nullptr);

  timer_start(TIMER_GROUP_0, TIMER_0);
}

void printTask(void* userParam)
{
  auto link = static_cast<ableton::Link*>(userParam);
  const auto quantum = 4.0;
    
#if defined USE_I2C_DISPLAY   
  if ( DefaultBusInit( ) == true ) {
        printf( "BUS Init lookin good...\n" );
        printf( "Drawing.\n" );
        SetupDemo( &I2CDisplay, &Font_droid_sans_mono_13x24 );
        SayHello( &I2CDisplay, "Link!" );
        printf( "Done!\n" );
   }
#endif  

  while (true)
  {
    const auto sessionState = link->captureAppSessionState();
    const auto numPeers = link->numPeers();
    const auto time = link->clock().micros();
    const auto beats = sessionState.beatAtTime(time, quantum);
      
#if defined USE_I2C_DISPLAY
    char buf[10];
    snprintf(buf, 10 , "%i", (int) round( sessionState.tempo() ) );
#endif
      
    std::cout << std::defaultfloat << "| peers: " << numPeers << " | "
              << "tempo: " << sessionState.tempo() << " | " << std::fixed
              << "beats: " << beats << " |" << std::endl;
    vTaskDelay(800 / portTICK_PERIOD_MS);
    
#if defined USE_I2C_DISPLAY
      SSD1306_Clear( &I2CDisplay, SSD_COLOR_BLACK );
      SSD1306_SetFont( &I2CDisplay, &Font_droid_sans_mono_13x24);
      SSD1306_FontDrawAnchoredString( &I2CDisplay, TextAnchor_North, "BPM", SSD_COLOR_WHITE );
      SSD1306_FontDrawAnchoredString( &I2CDisplay, TextAnchor_Center, buf, SSD_COLOR_WHITE );
      SSD1306_Update( &I2CDisplay );   
#endif
      
  }
}

// callbacks
void tempoChanged(double tempo) {
  std::cout << std::defaultfloat << "| new tempo: " << tempo << std::endl;
}

void startStopChanged(bool isPlaying) {
  // received as soon as sent
  // need to wait for phase to be 0 (and deal with latency...)
  std::cout << std::defaultfloat << "| state: " << isPlaying << std::endl;
}

void tickTask(void* userParam)
{

  // serial
  uart_config_t uart_config = {
    .baud_rate = 31250, // midi speed
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 122,
  };
  uart_param_config(UART_NUM_1, &uart_config);
  uart_set_pin(UART_NUM_1, ECHO_TEST_TXD, ECHO_TEST_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);

  // connect link
  ableton::Link link(120.0f);
  link.enable(true);
  link.enableStartStopSync(true); // if not no callback for start/stop

  // callbacks
  link.setTempoCallback(tempoChanged);
  link.setStartStopCallback(startStopChanged);

  // debug
  if (PRINT_LINK_STATE)
  {
    xTaskCreate(printTask, "print", 8192, &link, 1, nullptr);
  }
  gpio_set_direction(LED, GPIO_MODE_OUTPUT);

  // phase
  while (true)
  { 
    xSemaphoreTake(userParam, portMAX_DELAY);

    const auto state = link.captureAudioSessionState();
    const auto phase = state.phaseAtTime(link.clock().micros(), 1.);
    gpio_set_level(LED, fmodf(phase, 1.) < 0.1);

    // MIDI CLOCK
    // Clock events are sent at a rate of 24 pulses per quarter note
    // (60000ms / BPM = quarter-note beat in ms) / 24 (pulses per quarter note for midi clock)
    // 100 bpm = 25 ms midi clock pulse
    char midiMsg[] = { MIDI_TIMING_CLOCK };
    uart_write_bytes(UART_NUM_1, midiMsg, 1);

    const TickType_t xDelay = 25 / portTICK_PERIOD_MS;
    vTaskDelay(xDelay); // NOT WORKING...

    portYIELD();
  }
}

extern "C" void app_main()
{
  ESP_ERROR_CHECK(nvs_flash_init());
  tcpip_adapter_init();
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  ESP_ERROR_CHECK(example_connect());

  SemaphoreHandle_t tickSemphr = xSemaphoreCreateBinary();
  timerGroup0Init(100, tickSemphr);

  xTaskCreate(tickTask, "tick", 8192, tickSemphr, configMAX_PRIORITIES - 1, nullptr);
  
  #if defined USE_START_STOP
  gpio_config_t io_conf;
  io_conf.intr_type = (gpio_int_type_t)GPIO_PIN_INTR_NEGEDGE; 
  io_conf.intr_type = (gpio_int_type_t)GPIO_PIN_INTR_DISABLE; //disable interrupt
  io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;  //bit mask of the pins, use GPIO4
  io_conf.mode = GPIO_MODE_INPUT;  //set as input mode   
  io_conf.pull_down_en = (gpio_pulldown_t)0; // 0 //disable pull-down mode
  io_conf.pull_up_en = (gpio_pullup_t)1; //enable pull-up mode
  gpio_config(&io_conf);

  gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_NEGEDGE); // GPIO_INTR_ANYEDGE //change gpio interrupt type for one pin
  gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t)); //create a queue to handle gpio event from isr
  xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL); //start gpio task

  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT); //install gpio isr service
  gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0); //hook isr handler for specific gpio pin
  gpio_isr_handler_remove(GPIO_INPUT_IO_0); //remove isr handler for gpio number. 
  gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);  //hook isr handler for specific gpio pin again
  #endif
}
