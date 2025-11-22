#include <stdio.h>
#include <pico/stdio.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <string.h>
#include <pico/cyw43_arch.h>
#include <math.h>

#include "tkjhat/sdk.h"
#include "lwip/tcp.h"
#include "lwip/pbuf.h"
#include "lwip/api.h"
#include "lwip/ip_addr.h"
#include "lwip/err.h"

#define TEST_TCP_SERVER_IP "51.20.8.40"
#if !defined(TEST_TCP_SERVER_IP)
#error TEST_TCP_SERVER_IP not defined
#endif

#define TCP_PORT 8080
#define DEBUG_printf printf
#define BUF_SIZE 502

#define TEST_ITERATIONS 10
#define POLL_TIME_S 5
#if 1
static void dump_bytes(const uint8_t *bptr, uint32_t len) {
    unsigned int i = 0;

    printf("__dump_bytes %d__\n", len);
    for (i = 0; i < len;) {
        printf("__%c__", bptr[i++]);
    }
    printf("\n");
}
#define DUMP_BYTES dump_bytes
#else
#define DUMP_BYTES(A,B)
#endif
// Program global state to manage the flow of the program
enum state {IDLE=1, WAITING_DATA, DATA_READY, SEND_DATA, SPACES_REQUIREMENTS_SATISFIED, DISPLAY, DISPLAY_FINISHED};
// Starting with IDLE state
enum state programState=IDLE;

struct MorseAlphabet {
    char morseCode[7];
    char letter;
    int currMorseCodeIndex;
};

typedef struct TCP_CLIENT_T_ {
    struct tcp_pcb *tcp_pcb;
    ip_addr_t remote_addr;
    uint8_t buffer[BUF_SIZE];
    int buffer_len;
    int sent_len;
    bool complete;
    int run_count;
    bool connected;
} TCP_CLIENT_T;

struct MorseAlphabet morseCodes[40] = {
    {".-", 'a'}, {"-...", 'b'}, {"-.-.", 'c'}, {"-..", 'd'}, {".", 'e'}, {"..-.", 'f'}, {"--.", 'g'}, {"....", 'h'}, {"..", 'i'}, {".---", 'j'}, 
    {"-.-", 'k'}, {".-..", 'l'}, {"--",'m'}, {"-.", 'n'}, {"---", 'o'}, {".--.", 'p'}, {"--.-", 'q'}, {".-.", 'r'}, {"...", 's'}, {"-", 't'}, 
    {"..-", 'u'}, {"...-", 'v'}, {".--", 'w'}, {"-..-", 'x'}, {"-.--", 'y'}, {"--..", 'z'}, {"-----", '0'}, {".----", '1'}, {"..---", '2'}, 
    {"...--", '3'}, {"....-", '4'}, {".....", '5'}, {"-....", '6'}, {"--...", '7'}, {"---..", '8'}, {"----.", '9'}, {".-.-.-", '.'}, {"--..--", ','}, {"..--..", '?'}, {"-.-.--", '!'},
};

// Data structure for morse messagees
struct Message {
   char lastReadCharacter; // Last read morse character
   int currentIndex; // Current index in the morse string
   char message[120]; // Strring to store the morse received.
};

struct InitialTemp {
    float temp;
    int isFirstGet;
};

TCP_CLIENT_T *clientState = NULL;
struct InitialTemp tempThreshold = {0};

// Initialize light sensor
uint32_t ambientLight;
static bool light_button_pressed = false; 
const uint32_t LIGHT_THRESHOLD = 5; // When pressing on light sensor, value <=5

// Initialize the variable with struct type above to be a default value = 0.
struct Message imuMorseMessage = {0};
struct Message serialReceivedMorseMessage = {0};
struct Message translatedMessage = {0};
volatile TickType_t lastButtonTick = 0;
// char serialReceivedMorseMessage[120] = ".... . .-.. .-.. ---  .-- --- .-. .-.. -.."; 

// Prototype for functions
// Function to receive daata from IMU sensor (Accelerometer and GyroScope)
void imu_task(void *pvParameters);
// Function to send the morse string to the serial monitor.
static void serial_send_task(void *arg);
// Function to be called when button interruption appear
static void btn_fxn(uint gpio, uint32_t eventMask);
// Function to convert data from IMU to morse character
void convert_to_morse_character(float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *temp);
// Serial receive function
void rgb_task(void *pvParameters);
void buzzer_task(void *pvParameters);
void translate_receives_morse_codes(void);
char find_letter_from_morse_code(char *morseCode);
void sending_feedback();
// static void buzzer_task(void *arg);
static void buzzer_music_play();
static void lcd_display_task(void *args);
static void serial_receive_task(void *arg);
static void display_controller_task(void *args);
void wirelessTask();
void run_tcp_client_test(void);
void send_data_tcp();
void add_character_to_string(struct Message *message,char character, int updatedIndex);
//prototype for light sensor
void light_sensor_task(void *pvParameters);


int main() {
    // Initialize the stdio to be able to send and receive data.
    stdio_init_all();
    while (!stdio_usb_connected()){
        // If the serial monitor is not monitoring, then continue to wait.
        sleep_ms(10);
    }
    wirelessTask();
    // Initialize the hat sdk to be able to turn off the RGB and initialize the I2C peripherals.
    init_hat_sdk();
    // Sleep 300ms to make sure hat sdk is initialized
    sleep_ms(300);
    // Initialize the  led to be able to use it
    init_led();
    // Initialize the button1,button2 to be able to use it
    init_button1();
    init_button2();
    init_rgb_led();
    rgb_led_write(0, 0, 0);
    
    init_buzzer();
    // Initialize light sensor
    init_veml6030();

    init_display();
    // Check the connection the ICM-42670 sensor (0=success, otherwise fail the connection).
    if (init_ICM42670() == 0){
        printf("__ICM42670 initialize successfully__\n");
        // If initialize the ICM-42670 sensor successfuly, configure the parameter inside to be default value
        // to be ready for measuring.
        if (ICM42670_start_with_default_values() != 0){
            printf("__ICM42670P cound not initialize the accelerometer or gyroscope__\n");
        }
    }else {
        printf("__ICM42670 could not be initialized__\n");
    }
    // Set the interruption for both button1, button2 using together btn_fxn function.
    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_RISE, true, btn_fxn);
    gpio_set_irq_enabled(BUTTON2, GPIO_IRQ_EDGE_RISE, true);
    // TaskHandle for serial_send_task function
    TaskHandle_t serialSendTask;
    // TaskHandle for imu_task function
    TaskHandle_t hIMUTask = NULL;
    TaskHandle_t hReceiveTask;
    
    TaskHandle_t hLightTask = NULL;

    

    TaskHandle_t buzzerTask = NULL;
    TaskHandle_t serialReceiveTask = NULL;
    TaskHandle_t lcdDisplay = NULL;
    TaskHandle_t displayControllerTask = NULL;
    // Create all task with priority 2, no args.
    BaseType_t result = xTaskCreate(serial_send_task, "serialSendTask", 1024, NULL, 2, &serialSendTask);
    xTaskCreate(display_controller_task, "displayControllerTask", 1024, NULL, 2, &displayControllerTask);
    xTaskCreate(rgb_task, "RGBTask", 256, (void*) displayControllerTask, 2, NULL);
    xTaskCreate(buzzer_task,"BuzzerTask", 1024, (void*) displayControllerTask, 2, NULL );
    
    xTaskCreate(imu_task, "IMUTask", 1024, NULL, 3, &hIMUTask);
    // xTaskCreate(buzzer_task, "buzzerTask", 1024, NULL, 2, &buzzerTask);

    xTaskCreate(light_sensor_task, "LightTask", 512, NULL, 2, &hLightTask);

    xTaskCreate(serial_receive_task, "serialReceiveTask", 1024, NULL, 2, &serialReceiveTask);
    xTaskCreate(lcd_display_task, "lcdTask", 1024, (void*) displayControllerTask, 2, &lcdDisplay);
    // xTaskCreate(wirelessTask, "WirelessTask", 1024, NULL, 2,NULL );
    // Start to run and schedule the task
    vTaskStartScheduler();

    return 0;
}

void imu_task(void *pvParameters){
    (void)pvParameters;

    // Variabble to store imu data received.
    float ax, ay, az, gx, gy, gz, temp;
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while(1){
        // Read the data received from Accelerometer and Gyroscope
            if (programState == WAITING_DATA){
                if (ICM42670_read_sensor_data(&ax, &ay, &az,&gx,&gy,&gz,&temp) == 0){
                    printf("__Accel: X=%f, Y=%f, Z=%f | Gyro: X=%f, Y=%f, Z=%f| Temp: %2.2f°C  threshold: %2.2f°C__\n", ax, ay, az, gx, gy, gz, temp, tempThreshold.temp);
                    // If the programState is WAITING_DATA -> convert the received data to morse code and store it in the morse string.
                    if (tempThreshold.isFirstGet == 0 ){
                        tempThreshold.isFirstGet=1;
                        tempThreshold.temp = temp;
                    }
                    
                    // Function to convert received data to morse character.
                    convert_to_morse_character(&ax, &ay, &az, &gx, &gy, &gz, &temp);
                    
                }else {
                    printf("__Failed to read data from IMU sensor__\n");
                }
            }
            
        // Stop 100ms to run other tasks.
       vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 50 ) );
    }
}
void add_character_to_string(struct Message *message,char character, int updatedIndex){
    message->message[message->currentIndex] = character;
    message->currentIndex = updatedIndex;
}

static void serial_send_task(void *arg)
{
    (void)arg;

    while (1)
    {
        // If the progrramState is SEND_DATA -> Send the morsee string to serial monitor.
        if (programState == SEND_DATA)
        {
            // Send the morse string
            printf("__Receive morse message %s __\n", imuMorseMessage.message);
            send_data_tcp();
            // Reset the morse string index = 0
            // Reset the morse string to be empty.
            add_character_to_string(&imuMorseMessage, '\0', 0);
            // Set  the programState to be equal
            programState = IDLE;
        }
        // Stop 100ms to run other tasks
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void btn_fxn(uint gpio, uint32_t eventMask)
{
    TickType_t now = xTaskGetTickCountFromISR();
    if ((now - lastButtonTick) < pdMS_TO_TICKS(100)) {
        return;
    }
    lastButtonTick = now;
    // Check if the button pressed is button1 or button2
    if (gpio == BUTTON1)
    {
        // If button1 -> set the state to WAITING_DATA to received IMU sensor data
        programState = WAITING_DATA;
        printf("__Start to read sensor data__\n");
    }
    else if (gpio == BUTTON2)
    {
        // If button2 -> first check if there are 2 consecutive spaces already.
        if (programState == SPACES_REQUIREMENTS_SATISFIED)
        {
            // If there are 2 consecutive spaces -> set the current position element in morse string to be \n.
            imuMorseMessage.message[imuMorseMessage.currentIndex] = '\n';
            // Increase the index by 1
            add_character_to_string(&imuMorseMessage, '\n', imuMorseMessage.currentIndex + 1);
            // Set the current position element in morse string to be \0 to stop the string
            add_character_to_string(&imuMorseMessage, '\0', 0);
            // Set the programState to be SEND_DATA to send it to serial monitor
            programState = SEND_DATA;
        }
        else if (programState == DATA_READY)
        {
            // If we just received a morse character from imu and there exists no 2 consecutive spaces
            // Then we put a space in the current position in morse string
            printf("__Send space__\n");
            // Check if the morse string before the current
            if (imuMorseMessage.message[imuMorseMessage.currentIndex - 1] == ' ')
            {
                printf("__2 space consecutively detected__\n");
                programState = SPACES_REQUIREMENTS_SATISFIED;
            }
            add_character_to_string(&imuMorseMessage, ' ', imuMorseMessage.currentIndex + 1);
        }
        else
        {
            printf("__Unavailble to send the space here__\n");
        }
    }
}

void convert_to_morse_character(float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *temp)
{
    if (fabs(*gx) > 200 && fabs(*gy) > 200 && fabs(*gz) > 200)
    {
        buzzer_music_play();
        programState = IDLE;
    }
    else if (*gx < -200 && fabs(*gz) < 110 && fabs(*gy) < 110)
    {
        // If the position of imu is place horizontally -> set the current position of the morse string to be a dot.
        // Increase the morse string index by 1
        add_character_to_string(&imuMorseMessage, '.', imuMorseMessage.currentIndex + 1);
        printf("__Received Morse character '.' with moving the head down to up and go back to DATA_READY__\n");
        // Set the programState to be DATA_READY to be able to send space
        programState = DATA_READY;
    }
    else if (fabs(*gx) < 70 && *gy > 200 && fabs(*gz) < 70)
    {
        // If the position of imu is place horizontally -> set the current position of the morse string to be a dash.
        // Increase the morse string index by 1
        add_character_to_string(&imuMorseMessage, '-', imuMorseMessage.currentIndex + 1);
        printf("__Received Morse character '-' with tilt left fast and go back to DATA_READY__\n");
        // Set the programState to be DATA_READY to be able to send space
        programState = DATA_READY;
    }
    else if ((*ax > -1.1 && *ax < -0.9) && (*ay > -0.1 && *ay < 0.1) && (*az > -0.1 && *az < 0.1) && *temp > tempThreshold.temp + 1)
    {
        // If the position of imu is place horizontally -> set the current position of the morse string to be a dot.
        // Increase the morse string index by 1
        add_character_to_string(&imuMorseMessage, '.', imuMorseMessage.currentIndex + 1);
        printf("__Received Morse character '.' with tilt left and increase the temperature and go back to DATA_READY__\n");
        // Set the programState to be DATA_READY to be able to send space
        programState = DATA_READY;
    }
    else if ((*ax > 0.9 && *ax < 1.1) && (*ay > -0.1 && *ay < 0.1) && (*az > -0.1 && *az < 0.1) && *temp > tempThreshold.temp + 1)
    {
        // If the position of imu is place horizontally -> set the current position of the morse string to be a dash.
        // Increase the morse string index by 1
        add_character_to_string(&imuMorseMessage, '-', imuMorseMessage.currentIndex + 1);
        printf("__Received Morse character '-' with tilt right and increase the temperature and go back to DATA_READY__\n");
        // Set the programState to be DATA_READY to be able to send space
        programState = DATA_READY;
    }
    // Check if the position of the IMU sensor match the condition
    if ((*ax > -0.1 && *ax < 0.1) && (*ay > -0.1 && *ay < 0.1) && (*az > 0.9 && *az < 1.1))
    {
        // If the position of imu is place horizontally -> set the current position of the morse string to be a dot.
        // Increase the morse string index by 1
        add_character_to_string(&imuMorseMessage, '.', imuMorseMessage.currentIndex + 1);
        printf("__Received Morse character '.' and go back to DATA_READY__\n");
        // Set the programState to be DATA_READY to be able to send space
        programState = DATA_READY;
    }
    else if ((*ax > -0.1 && *ax < 0.1) && (*az > -0.1 && *az < 0.1) && (*ay < -0.9 && *ay > -1.1))
    {
        // If the position of imu is place horizontally -> set the current position of the morse string to be a dash.
        // Increase the morse string index by 1
        add_character_to_string(&imuMorseMessage, '-', imuMorseMessage.currentIndex + 1);
        printf("__Received Morse character '-' and go back to DATA_READY__\n");
        // Set the programState to be DATA_READY to be able to send space
        programState = DATA_READY;
    }
}

void rgb_task(void *pvParameters)
{
    TaskHandle_t displayControllerTask = (TaskHandle_t)pvParameters;
    static int isGiveNotify = 0;
    while (1)
    {
        if (programState == DISPLAY && isGiveNotify == 0)
        {
            for (int i = 0; i < strlen(serialReceivedMorseMessage.message); i++)
            {
                printf("__Read morse character %c __\n", serialReceivedMorseMessage.message[i]);
                if (serialReceivedMorseMessage.message[i] == '.')
                {
                    rgb_led_write(0, 0, 255);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    rgb_led_write(0, 0, 0);
                    vTaskDelay(pdMS_TO_TICKS(300));
                }
                else if (serialReceivedMorseMessage.message[i] == '-')
                {
                    rgb_led_write(0, 255, 0);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    rgb_led_write(0, 0, 0);
                    vTaskDelay(pdMS_TO_TICKS(300));
                }
                else if (serialReceivedMorseMessage.message[i] == ' ')
                {
                    rgb_led_write(255, 0, 0);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    rgb_led_write(0, 0, 0);
                    vTaskDelay(pdMS_TO_TICKS(300));
                }
            }
            xTaskNotifyGive(displayControllerTask);
            isGiveNotify = 1;
        }
        else if (programState == DISPLAY_FINISHED)
        {
            isGiveNotify = 0;
        }
    }
}

void buzzer_task(void *pvParameters)
{
    TaskHandle_t displayControllerTask = (TaskHandle_t)pvParameters;
    static int isGiveNotify = 0;
    while (1)
    {
        if (programState == DISPLAY && isGiveNotify == 0)
        {
            for (int i = 0; i < strlen(serialReceivedMorseMessage.message); i++)
            {
                if (serialReceivedMorseMessage.message[i] == '.')
                {
                    buzzer_play_tone(440, 500);
                    vTaskDelay(pdMS_TO_TICKS(400));
                }
                else if (serialReceivedMorseMessage.message[i] == '-')
                {
                    buzzer_play_tone(440, 1000);
                    vTaskDelay(pdMS_TO_TICKS(400));
                }
                else if (serialReceivedMorseMessage.message[i] == ' ')
                {
                    vTaskDelay(pdMS_TO_TICKS(400));
                }
            }
            xTaskNotifyGive(displayControllerTask);
            isGiveNotify = 1;
        }
        else if (programState == DISPLAY_FINISHED)
        {
            isGiveNotify = 0;
        }
        vTaskDelay(100);
    }
}
void translate_receives_morse_codes()
{
    printf("__Start__\n");
    struct MorseAlphabet singleMorseCode = {0};
    for (int i = 0; i < 120; i++)
    {
        if (serialReceivedMorseMessage.message[i] != ' ')
        {
            singleMorseCode.morseCode[singleMorseCode.currMorseCodeIndex] = serialReceivedMorseMessage.message[i];
            singleMorseCode.currMorseCodeIndex += 1;
        }
        else
        {
            if (serialReceivedMorseMessage.message[i - 1] == ' ')
            {
                if (serialReceivedMorseMessage.message[i + 1] == '\0' || serialReceivedMorseMessage.message[i + 1] == '\n')
                {
                    add_character_to_string(&translatedMessage, '\n', translatedMessage.currentIndex + 1);
                    add_character_to_string(&translatedMessage, '\0', 0);
                    break;
                }
                else
                {
                    add_character_to_string(&translatedMessage, ' ', translatedMessage.currentIndex + 1);
                }
            }
            else
            {
                singleMorseCode.morseCode[singleMorseCode.currMorseCodeIndex] = '\0';
                singleMorseCode.currMorseCodeIndex = 0;
                char translatedLetter = find_letter_from_morse_code(singleMorseCode.morseCode);
                add_character_to_string(&translatedMessage, translatedLetter, translatedMessage.currentIndex + 1);
            }
        }
    }
    printf("__Translated string %s__\n", translatedMessage.message);
}

char find_letter_from_morse_code(char *morseCode){
    for (int i = 0; i < 40; i++){
        if (strcmp(morseCode, morseCodes[i].morseCode) == 0){
            return morseCodes[i].letter;
        }
    }
    return 'n';
}

static void buzzer_music_play(){
    int melody[25] = {
        330, 330, 330, 330, 330, 330, 330, 392, 262, 294, 330, 349, 349, 349, 330, 330,
        330, 330, 330, 330, 330, 392, 262, 294, 330
    };

    int durations[25] = {
        250, 250, 500, 250, 250, 500, 250, 250, 250, 250, 500, 250, 250, 500, 250, 250,
        250, 250, 500, 250, 250, 500, 250, 250, 500
    };

    int length =sizeof(melody) / sizeof(melody[0]);
    printf("__Music Play __\n");
    for (int i = 0 ; i < length; i++){
        buzzer_play_tone(melody[i], durations[i]);
        sleep_ms(50);
    }
}

static void serial_receive_task(void *arg)
{
    (void)arg;

    while (true)
    {
        int receivedChar = getchar_timeout_us(0);
        if (receivedChar != PICO_ERROR_TIMEOUT)
        {
            if (programState != DISPLAY)
            {
                if (receivedChar == '\r')
                    continue;
                if (serialReceivedMorseMessage.currentIndex >= 120 - 1)
                {
                    add_character_to_string(&serialReceivedMorseMessage, '\0', 0);
                    printf("__Overflow text warning__\n");
                    programState = DISPLAY;
                }
                else if (receivedChar == '\n')
                {
                    add_character_to_string(&serialReceivedMorseMessage, '\0', 0);
                    programState = DISPLAY;
                    printf("__Received String %s__\n", serialReceivedMorseMessage.message);
                }
                else
                {
                    add_character_to_string(&serialReceivedMorseMessage, receivedChar, serialReceivedMorseMessage.currentIndex + 1);
                    printf("__Received letter=%c__\n", receivedChar);
                }
            }
            else
            {
                printf("__Currently in display mode, cannot receive more data__\n");
            }
        }

        else
        {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}


static void lcd_display_task(void *args)
{
    TaskHandle_t displayControllerTask = (TaskHandle_t)args;
    static int isGiveNotify = 0;
    write_text("Waiting...");
    while (true)
    {

        if (programState == DISPLAY && isGiveNotify == 0)
        {
            translate_receives_morse_codes();
            int length = strlen(translatedMessage.message);
            char displayString[9];
            clear_display();
            if (length <= 9)
            {
                printf("__translated message: %s__\n", translatedMessage.message);
                write_text(translatedMessage.message);
                vTaskDelay(pdMS_TO_TICKS(500));
                clear_display();
            }
            else
            {
                for (int i = 0; i < length - 8; i++)
                {
                    memcpy(displayString, &translatedMessage.message[i], 8);
                    displayString[8] = '\0';
                    printf("__Display string %s__\n", displayString);           
                    write_text(displayString);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    clear_display();
                }
            }
            write_text("Waiting...");
            xTaskNotifyGive(displayControllerTask);
            isGiveNotify = 1;
        }
        else if (programState == DISPLAY_FINISHED)
        {
            isGiveNotify = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

static void display_controller_task(void *args){
    int finishedDisplayTask = 0;
    while(true){
        if (finishedDisplayTask < 3){
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            finishedDisplayTask++;
        }else {
            finishedDisplayTask = 0;
            programState = DISPLAY_FINISHED;
            printf("__3 task display finished__\n");
        }
    }


}

void sending_feedback() {
    buzzer_play_tone(440, 500);
}


void wirelessTask() {
    // Initializing CYW43439
    if (cyw43_arch_init()) {
        printf("__WiFi init failed!__\n");
        // có thể tiếp tục chạy không Wi-Fi hoặc for(;;);
    }
    printf("__Connecting to Wi-Fi__\n");
    printf("__Init succesffuly -> stable mode__\n");
    // Enabling "Station"-mode, where we can connect to wireless networks
        cyw43_arch_enable_sta_mode();
        // Connecting to the open panoulu-network (no password needed)
        // We try to connect for 30 seconds before printing an error message
        printf("__Set stable mode successfully -> connect to wifi__\n");
        if(cyw43_arch_wifi_connect_timeout_ms("phuc", "1231232312123", CYW43_AUTH_WPA2_AES_PSK, 30 * 1000)) {
            printf("__Failed to connect__\n");
        }
        else {
            printf("__Connected to wifi\n");
        }

    
    printf("__Run test__\n");
    run_tcp_client_test();
  
}

static err_t tcp_client_close(void *arg) {
    TCP_CLIENT_T *clientState = (TCP_CLIENT_T*)arg;
    err_t err = ERR_OK;
    if (clientState->tcp_pcb != NULL) {
        tcp_arg(clientState->tcp_pcb, NULL);
        tcp_poll(clientState->tcp_pcb, NULL, 0);
        tcp_sent(clientState->tcp_pcb, NULL);
        tcp_recv(clientState->tcp_pcb, NULL);
        tcp_err(clientState->tcp_pcb, NULL);
        err = tcp_close(clientState->tcp_pcb);
        if (err != ERR_OK) {
            DEBUG_printf("__close failed %d, calling abort__\n", err);
            tcp_abort(clientState->tcp_pcb);
            err = ERR_ABRT;
        }
        clientState->tcp_pcb = NULL;
    }
    return err;
}

static err_t tcp_result(void *arg, int status) {
    TCP_CLIENT_T *clientState = (TCP_CLIENT_T*)arg;
    if (status == 0) {
        DEBUG_printf("test success\n");
    } else {
        DEBUG_printf("test failed %d\n", status);
    }
    clientState->complete = true;
    return tcp_client_close(arg);
}

static err_t tcp_client_sent(void *arg, struct tcp_pcb *tpcb, u16_t len) {
    TCP_CLIENT_T *clientState = (TCP_CLIENT_T*)arg;
    DEBUG_printf("__tcp_client_sent %u__\n", len);
    clientState->sent_len += len;

    if (clientState->sent_len >= BUF_SIZE) {

        // We should receive a new buffer from the server
        clientState->buffer_len = 0;
        clientState->sent_len = 0;
        DEBUG_printf("__Waiting for buffer from server__\n");
    }

    return ERR_OK;
}

static err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err) {
    TCP_CLIENT_T *clientState = (TCP_CLIENT_T*)arg;
    if (err != ERR_OK) {
        printf("__connect failed %d__\n", err);
        return tcp_result(arg, err);
    }
    clientState->connected = true;
    const char *msg = "Hello Server";
    tcp_write(tpcb, msg, strlen(msg), TCP_WRITE_FLAG_COPY);

    DEBUG_printf("__Waiting for buffer from server___\n");
    return ERR_OK;
}

static err_t tcp_client_poll(void *arg, struct tcp_pcb *tpcb) {
    // DEBUG_printf("tcp_client_poll\n");
    return ERR_OK; // no response is an error?
}

static void tcp_client_err(void *arg, err_t err) {
    TCP_CLIENT_T *clientState = (TCP_CLIENT_T*)arg;
    if (err != ERR_ABRT) {
        DEBUG_printf("__tcp_client_err %d__\n", err);
        tcp_client_close(clientState);
    }else {
        free(clientState);
        printf("__Client state freed due to abort error__\n");
        return;
    }
}

err_t tcp_client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    TCP_CLIENT_T *clientState = (TCP_CLIENT_T*)arg;
    if (!p) {
        return tcp_result(arg, -1);
    }
    // this method is callback from lwIP, so cyw43_arch_lwip_begin is not required, however you
    // can use this method to cause an assertion in debug mode, if this method is called when
    // cyw43_arch_lwip_begin IS needed
    cyw43_arch_lwip_check();
    if (p->tot_len > 0) {
        DEBUG_printf("__recv %d err %d__\n", p->tot_len, err);
        for (struct pbuf *q = p; q != NULL; q = q->next) {
            DUMP_BYTES(q->payload, q->len);
        }
        // Receive the buffer
        const uint16_t buffer_left = BUF_SIZE - clientState->buffer_len;
        clientState->buffer_len += pbuf_copy_partial(p, clientState->buffer + clientState->buffer_len,
                                               p->tot_len > buffer_left ? buffer_left : p->tot_len, 0);
        tcp_recved(tpcb, p->tot_len);
    }
    pbuf_free(p);

    if (clientState->buffer_len == BUF_SIZE) {
        DUMP_BYTES(clientState->buffer, clientState->buffer_len);
        clientState->buffer[0] = '\0';
        clientState->buffer_len = 0;
    }
    return ERR_OK;
}

static bool tcp_client_open(void *arg) {
    TCP_CLIENT_T *clientState = (TCP_CLIENT_T*)arg;
    DEBUG_printf("__Connecting to %s port %u__\n", ip4addr_ntoa(&clientState->remote_addr), TCP_PORT);
    clientState->tcp_pcb = tcp_new_ip_type(IP_GET_TYPE(&clientState->remote_addr));
    if (!clientState->tcp_pcb) {
        DEBUG_printf("__failed to create pcb__\n");
        return false;
    }

    tcp_arg(clientState->tcp_pcb, clientState);
    tcp_poll(clientState->tcp_pcb, tcp_client_poll, POLL_TIME_S * 2);
    tcp_sent(clientState->tcp_pcb, tcp_client_sent);
    tcp_recv(clientState->tcp_pcb, tcp_client_recv);
    tcp_err(clientState->tcp_pcb, tcp_client_err);

    clientState->buffer_len = 0;

    // cyw43_arch_lwip_begin/end should be used around calls into lwIP to ensure correct locking.
    // You can omit them if you are in a callback from lwIP. Note that when using pico_cyw_arch_poll
    // these calls are a no-op and can be omitted, but it is a good practice to use them in
    // case you switch the cyw43_arch type later.
    cyw43_arch_lwip_begin();
    err_t err = tcp_connect(clientState->tcp_pcb, &clientState->remote_addr, TCP_PORT, tcp_client_connected);
    cyw43_arch_lwip_end();

    return err == ERR_OK;
}

// Perform initialisation
static TCP_CLIENT_T* tcp_client_init(void) {
    clientState = calloc(1, sizeof(TCP_CLIENT_T));
    if (!clientState) {
        DEBUG_printf("failed to allocate state\n");
        return NULL;
    }
    ip4addr_aton(TEST_TCP_SERVER_IP, &clientState->remote_addr);
    return clientState;
}
void run_tcp_client_test(void) {
    TCP_CLIENT_T *clientState = tcp_client_init();
    if (!clientState) {
        printf("__No state__\n");
        return;
    }
    if (!tcp_client_open(clientState)) {
        printf("__Cannot open__\n");
        tcp_result(clientState, -1);
        return;
    }

}


void send_data_tcp(){
    // Reset the morse string index = 0
    printf("__Sending condition connected=%d__\n", clientState->connected);
    if(clientState == NULL){
        printf("__Client state is null__\n");
    }
    if (clientState == NULL || !clientState->connected) return;
    printf("__Send data over TCP__\n");
    tcp_write(clientState->tcp_pcb, imuMorseMessage.message, strlen(imuMorseMessage.message), TCP_WRITE_FLAG_COPY);
  
}
void light_sensor_task(void *pvParameters) {
    (void)pvParameters;


    while (1) {
        if (programState == DATA_READY || programState == SPACES_REQUIREMENTS_SATISFIED) {
            
            ambientLight = veml6030_read_light(); 
            printf("light sensor %u\n", ambientLight);
            if (ambientLight < LIGHT_THRESHOLD) {

                    
                    if (programState == SPACES_REQUIREMENTS_SATISFIED) {
                        imuMorseMessage.message[imuMorseMessage.currentIndex] = '\n';
                        imuMorseMessage.currentIndex += 1;
                        imuMorseMessage.message[imuMorseMessage.currentIndex] = '\0';
                        programState = SEND_DATA; 
                    }
                    
                    else {
                        imuMorseMessage.message[imuMorseMessage.currentIndex] = ' ';
                        printf("Light Button: Send space\n");
                        
                        if ( imuMorseMessage.message[imuMorseMessage.currentIndex - 1] == ' '){
                            printf("Light Button: 2 spaces consecutively\n");
                            programState = SPACES_REQUIREMENTS_SATISFIED;
                            
                        } else {
                            programState = DATA_READY;
                        }
                        imuMorseMessage.currentIndex += 1;
                        vTaskDelay(pdMS_TO_TICKS(2000));
                        
                    }
                }
        
            
        } 
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}