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

#define INPUT_BUFFER_SIZE 502
#define MORSE_ALPHABET_SIZE 40
#define SONG_TONE_SIZE 25
#define WINDOW_SIZE 9
#define MORSE_CHARACTER_SIZE 7
#define LIGHT_THRESHOLD 3
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
static void dump_bytes(const uint8_t *bptr, uint32_t len)
{
    unsigned int i = 0;

    printf("__dump_bytes %d__\n", len);
    for (i = 0; i < len;)
    {
        printf("__%c__", bptr[i++]);
    }
    printf("\n");
}
#define DUMP_BYTES dump_bytes
#else
#define DUMP_BYTES(A, B)
#endif
// Program global state to manage the flow of the program
enum state
{
    IDLE = 1,
    WAITING_DATA,
    DATA_READY,
    SEND_DATA,
    SPACES_REQUIREMENTS_SATISFIED,
    DISPLAY,
    DISPLAY_FINISHED
};
// Starting with IDLE state
enum state programState = IDLE;

// Struct type for morse character alphabet
struct MorseAlphabet
{
    char morseCode[MORSE_CHARACTER_SIZE];
    char letter;
    int currMorseCodeIndex;
};

// Struct type for the tcp client
typedef struct TCP_CLIENT_T_
{
    struct tcp_pcb *tcp_pcb;
    ip_addr_t remote_addr;    // IP address of server
    uint8_t buffer[BUF_SIZE]; // Buffer of receive data
    int buffer_len;           // Number of element in the receive data
    int sent_len;             // Number of sent buffer to server
    bool complete;            // Boolean to check if the communication is complete or not
    int run_count;
    bool connected; // To check if the client is connected to server.
} TCP_CLIENT_T;

// Morse alphabet to be searched.
struct MorseAlphabet morseCodes[MORSE_ALPHABET_SIZE] = {
    {".-", 'a'},
    {"-...", 'b'},
    {"-.-.", 'c'},
    {"-..", 'd'},
    {".", 'e'},
    {"..-.", 'f'},
    {"--.", 'g'},
    {"....", 'h'},
    {"..", 'i'},
    {".---", 'j'},
    {"-.-", 'k'},
    {".-..", 'l'},
    {"--", 'm'},
    {"-.", 'n'},
    {"---", 'o'},
    {".--.", 'p'},
    {"--.-", 'q'},
    {".-.", 'r'},
    {"...", 's'},
    {"-", 't'},
    {"..-", 'u'},
    {"...-", 'v'},
    {".--", 'w'},
    {"-..-", 'x'},
    {"-.--", 'y'},
    {"--..", 'z'},
    {"-----", '0'},
    {".----", '1'},
    {"..---", '2'},
    {"...--", '3'},
    {"....-", '4'},
    {".....", '5'},
    {"-....", '6'},
    {"--...", '7'},
    {"---..", '8'},
    {"----.", '9'},
    {".-.-.-", '.'},
    {"--..--", ','},
    {"..--..", '?'},
    {"-.-.--", '!'},
};

// Data structure for morse messagees
struct Message
{
    char lastReadCharacter;          // Last read morse character
    int currentIndex;                // Current index in the morse string
    char message[INPUT_BUFFER_SIZE]; // Strring to store the morse received.
};

// Struct type for storing the temperature value, and to get the threshold
struct InitialTemp
{
    float temp;     // To store the first temperature value get
    int isFirstGet; // Boolean to check if the first value is stored or not
};

// Global pointer variable to manage the client.
TCP_CLIENT_T *clientState = NULL;
// Initialize the temperature threshold with the value 0.
struct InitialTemp tempThreshold = {0};

// Initialize the variable to store the morse code received from IMU with default value = 0.
struct Message imuMorseMessage = {0};
// Initialize the variable to store the serial received morse message with default value = 0
struct Message serialReceivedMorseMessage = {0};
// Initialize the variable to store the translated message of the serial received message.
struct Message translatedMessage = {0};
// Variable of tick type to check the last time the user click the button.
volatile TickType_t lastButtonTick = 0;

// Prototype for functions
// Function to receive daata from IMU sensor (Accelerometer and GyroScope) (Task)
void imu_task(void *pvParameters);
// Function to send the morse string to the serial monitor. (Task)
static void handle_send_task(void *arg);
// Function to be called when button interruption appear (Interruption)
static void btn_fxn(uint gpio, uint32_t eventMask);
// Function to convert data from IMU to morse character
void handle_imu_data(float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *temp);
// Function to display the morse code by the rgb task (Task)
void rgb_task(void *pvParameters);
// Function to display the morse code by the buzzer (Task)
void buzzer_task(void *pvParameters);
// Function to translate the serial received morse message to become normal alphabet.
void translate_receives_morse_codes(void);
// Function to find the correct letter from the morse code
char find_letter_from_morse_code(char *morseCode);
// Function to send the feedback by buzzer when the morse message is sent to serial client
void sending_feedback();
// Function to play the music.
static void buzzer_music_play();
// Function to display message to the lcd (Task).
static void lcd_display_task(void *pvParameters);
// Function to receive the string from the serial client.
static void serial_receive_task(void *arg);
// Function to control or to know if the display task from 3 thing: buzzer, rgb, lcd is displayed or not.
static void display_controller_task(void *args);
// Function to connect to the wifi.
void wirelessTask();
// Function to connect to the remote tcp server
void run_tcp_client_test(void);
// Function to send the data to the tcp server
void send_data_tcp();
// Function to add the character to the string and update the index.
void add_character_to_string(struct Message *message, char character, int updatedIndex);
// prototype for light sensor
void light_sensor_task(void *pvParameters);

int main()
{
    // Initialize the stdio to be able to send and receive data.
    stdio_init_all();
    while (!stdio_usb_connected())
    {
        // If the serial monitor is not monitoring, then continue to wait.
        sleep_ms(10);
    }
    // Connect to the wifi and the tcp server
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
    // Turn off the RGB at first
    rgb_led_write(0, 0, 0);
    // Initialize the buzzer
    init_buzzer();
    // Initialize light sensor
    init_veml6030();
    // Initialize the display
    init_display();
    // Check the connection the ICM-42670 sensor (0=success, otherwise fail the connection).
    if (init_ICM42670() == 0)
    {
        printf("__ICM42670 initialize successfully__\n");
        // If initialize the ICM-42670 sensor successfuly, configure the parameter inside to be default value
        // to be ready for measuring.
        if (ICM42670_start_with_default_values() != 0)
        {
            printf("__ICM42670P cound not initialize the accelerometer or gyroscope__\n");
        }
    }
    else
    {
        printf("__ICM42670 could not be initialized__\n");
    }
    // Set the interruption for both button1, button2 using together btn_fxn function.
    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_RISE, true, btn_fxn);
    gpio_set_irq_enabled(BUTTON2, GPIO_IRQ_EDGE_RISE, true);
    // TaskHandle for handle_send_task function
    TaskHandle_t serialSendTask;
    // TaskHandle for imu_task function
    TaskHandle_t hIMUTask = NULL;
    // TaskHandle for ambient_light function
    TaskHandle_t hLightTask = NULL;
    // TaskHandle for the buzzer display function
    TaskHandle_t buzzerTask = NULL;
    // Task handle for displaying by rgb.
    TaskHandle_t rgbTask = NULL;
    // TaskHandle for displaying on the lcd.
    TaskHandle_t lcdDisplay = NULL;
    // TaskHandle for the receiving morse message from the serial_client
    TaskHandle_t serialReceiveTask = NULL;
    // TaskHandle for controlling the 3 display task: buzzer, rgb, lcd.
    TaskHandle_t displayControllerTask = NULL;
    // Create and schedule all task above.
    xTaskCreate(handle_send_task, "serialSendTask", 1024, NULL, 2, &serialSendTask);
    xTaskCreate(display_controller_task, "displayControllerTask", 1024, NULL, 2, &displayControllerTask);
    xTaskCreate(rgb_task, "RGBTask", 256, (void *)displayControllerTask, 2, &rgbTask);
    xTaskCreate(buzzer_task, "BuzzerTask", 1024, (void *)displayControllerTask, 2, &buzzerTask);
    xTaskCreate(imu_task, "IMUTask", 1024, NULL, 3, &hIMUTask);
    xTaskCreate(light_sensor_task, "LightTask", 512, NULL, 2, &hLightTask);
    xTaskCreate(serial_receive_task, "serialReceiveTask", 1024, NULL, 2, &serialReceiveTask);
    xTaskCreate(lcd_display_task, "lcdTask", 1024, (void *)displayControllerTask, 2, &lcdDisplay);
    // xTaskCreate(wirelessTask, "WirelessTask", 1024, NULL, 2,NULL );
    // Start to run and schedule the task
    vTaskStartScheduler();

    return 0;
}

void imu_task(void *pvParameters)
{
    (void)pvParameters;

    // Variabble to store imu data received.
    float ax, ay, az, gx, gy, gz, temp;
    // Variable keeps track of when the task was last unblocked
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        // Read the data received from Accelerometer and Gyroscope
        // Only read if the programState equal WAITING_DATA
        if (programState == WAITING_DATA)
        {
            // Read the data from the sensor
            if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &temp) == 0)
            {
                printf("__Accel: X=%f, Y=%f, Z=%f | Gyro: X=%f, Y=%f, Z=%f| Temp: %2.2f°C  threshold: %2.2f°C__\n", ax, ay, az, gx, gy, gz, temp, tempThreshold.temp);
                // If this the first time reading the IMU sensor.
                if (tempThreshold.isFirstGet == 0)
                {
                    // Change the value isFirstGet to announce that we have get the first temperature value
                    tempThreshold.isFirstGet = 1;
                    // Store the first received temperature to be the threshold
                    tempThreshold.temp = temp;
                }

                // Function to handle imu data
                handle_imu_data(&ax, &ay, &az, &gx, &gy, &gz, &temp);
            }
            else
            {
                printf("__Failed to read data from IMU sensor__\n");
            }
        }

        // Stop 100ms to run other tasks.
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    }
}
void add_character_to_string(struct Message *message, char character, int updatedIndex)
{
    // Function to add the character to the current index
    message->message[message->currentIndex] = character;
    // Increase the value of the current index
    message->currentIndex = updatedIndex;
}

static void handle_send_task(void *arg)
{
    (void)arg;

    while (1)
    {
        // If the progrramState is SEND_DATA -> Send the morsee string to serial monitor.
        if (programState == SEND_DATA)
        {
            // Announce that we have send data with the buzzer sound
            sending_feedback();
            // Send the morse string to the serial client
            printf("__Receive morse message %s __\n", imuMorseMessage.message);
            // Send the morse string to the tcp server
            send_data_tcp();
            // Reset the morse string index = 0
            // Reset the morse string to be empty.
            imuMorseMessage.currentIndex = 0;
            add_character_to_string(&imuMorseMessage, '\0', 0);
            // Set the programState to be IDLE
            programState = IDLE;
        }
        // Stop 100ms to run other tasks
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void btn_fxn(uint gpio, uint32_t eventMask)
{
    // Keep track of the time when the interruption is call
    TickType_t now = xTaskGetTickCountFromISR();
    // Calculate the difference between the current and last tick time.
    if ((now - lastButtonTick) < pdMS_TO_TICKS(100))
    {
        // If less than 100ms -> return.
        return;
    }
    // Update the last tick time of the button
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
            // Increase the index by 1
            add_character_to_string(&imuMorseMessage, '\n', imuMorseMessage.currentIndex + 1);
            // Set the current position element in morse string to be \0 to stop the string
            add_character_to_string(&imuMorseMessage, '\0', 0);
            // Set the programState to be SEND_DATA to send it to serial monitor and tcp server
            programState = SEND_DATA;
        }
        else if (programState == DATA_READY)
        {
            printf("__Send space__\n");
            // Check if there is already 1 space before the current index.
            if (imuMorseMessage.message[imuMorseMessage.currentIndex - 1] == ' ')
            {
                // If it is, then set the programState to announce that we are in the state of 2 consecutive spaces.
                printf("__2 space consecutively detected__\n");
                programState = SPACES_REQUIREMENTS_SATISFIED;
            }
            // If we just received a morse character from imu and there exists no 2 consecutive spaces
            // Then we put a space in the current position in morse string
            add_character_to_string(&imuMorseMessage, ' ', imuMorseMessage.currentIndex + 1);
        }
        else
        {
            printf("__Unavailble to send the space here__\n");
        }
    }
}

void handle_imu_data(float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *temp)
{
    if (fabs(*gx) > 200 && fabs(*gy) > 200 && fabs(*gz) > 200)
    {
        // If we shake the device, the music will be played
        buzzer_music_play();
        // Set the programState back to IDLE
        programState = IDLE;
    }
    else if (*gx < -200 && fabs(*gz) < 110 && fabs(*gy) < 110)
    {
        // If the device's head is moved fast from down to up -> set the current position of the morse string to be a dot.
        // Increase the morse string index by 1
        add_character_to_string(&imuMorseMessage, '.', imuMorseMessage.currentIndex + 1);
        printf("__Received Morse character '.' with moving the head down to up and go back to DATA_READY__\n");
        // Set the programState to be DATA_READY to be able to send space
        programState = DATA_READY;
    }
    else if (fabs(*gx) < 70 && *gy > 200 && fabs(*gz) < 70)
    {
        // If the device is tilt left fast -> set the current position of the morse string to be a dash.
        // Increase the morse string index by 1
        add_character_to_string(&imuMorseMessage, '-', imuMorseMessage.currentIndex + 1);
        printf("__Received Morse character '-' with tilt left fast and go back to DATA_READY__\n");
        // Set the programState to be DATA_READY to be able to send space
        programState = DATA_READY;
    }
    else if ((*ax > -1.1 && *ax < -0.9) && (*ay > -0.1 && *ay < 0.1) && (*az > -0.1 && *az < 0.1) && *temp > tempThreshold.temp + 1)
    {
        // If the position of imu is place left tilt position and the temp > tempthreshold + 1 -> set the current position of the morse string to be a dot.
        // Increase the morse string index by 1
        add_character_to_string(&imuMorseMessage, '.', imuMorseMessage.currentIndex + 1);
        printf("__Received Morse character '.' with tilt left and increase the temperature and go back to DATA_READY__\n");
        // Set the programState to be DATA_READY to be able to send space
        programState = DATA_READY;
    }
    else if ((*ax > 0.9 && *ax < 1.1) && (*ay > -0.1 && *ay < 0.1) && (*az > -0.1 && *az < 0.1) && *temp > tempThreshold.temp + 1)
    {
        // If the position of imu is place right tilt position and the temp > tempthreshold + 1  -> set the current position of the morse string to be a dash.
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
    // Get the TaskHandle of the display controller from the parameters
    TaskHandle_t displayControllerTask = (TaskHandle_t)pvParameters;
    // Value to check if the task is finished for one cycle and send already notify the display controller
    // Value of 0 -> not give notification yet, value of 1 -> already give notification
    static int isGiveNotify = 0;
    while (1)
    {
        // If the programState is in DISPLAY mode and have not sent the notification to the display controller
        if (programState == DISPLAY && isGiveNotify == 0)
        {
            // Loops through the serial receive morse message to display it by rgb.
            for (int i = 0; i < strlen(serialReceivedMorseMessage.message); i++)
            {
                printf("__Read morse character %c __\n", serialReceivedMorseMessage.message[i]);
                if (serialReceivedMorseMessage.message[i] == '.')
                {
                    // If the read character is a dot -> show a blue color for 500ms
                    rgb_led_write(0, 0, 255);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    // Show none-color for 300ms
                    rgb_led_write(0, 0, 0);
                    vTaskDelay(pdMS_TO_TICKS(300));
                }
                else if (serialReceivedMorseMessage.message[i] == '-')
                {
                    // If the read character is a dash -> show a green color for 500ms
                    rgb_led_write(0, 255, 0);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    // Show none-color for 300ms
                    rgb_led_write(0, 0, 0);
                    vTaskDelay(pdMS_TO_TICKS(300));
                }
                else if (serialReceivedMorseMessage.message[i] == ' ')
                {
                    // If the read character is a space -> show a red color for 500ms
                    rgb_led_write(255, 0, 0);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    // Show none-color for 300ms
                    rgb_led_write(0, 0, 0);
                    vTaskDelay(pdMS_TO_TICKS(300));
                }
            }
            // After finishing, notify the display controller
            xTaskNotifyGive(displayControllerTask);
            // Set the isGiveNotify value to 1 so that it will not be displayed infinitely.
            isGiveNotify = 1;
        }
        else if (programState == DISPLAY_FINISHED)
        {
            // If the programState equal DISPLAY_FINISHED -> all 3 display finished
            // Now we can reset the isGiveNotify to be able to display another message
            isGiveNotify = 0;
        }
    }
}

void buzzer_task(void *pvParameters)
{
    // Get the TaskHandle of the display controller from the parameters
    TaskHandle_t displayControllerTask = (TaskHandle_t)pvParameters;
    // Value to check if the task is finished for one cycle and send already notify the display controller
    // Value of 0 -> not give notification yet, value of 1 -> already give notification
    static int isGiveNotify = 0;
    while (1)
    {
        // If the programState is in DISPLAY mode and have not sent the notification to the display controller
        if (programState == DISPLAY && isGiveNotify == 0)
        {
            // Loops through the serial receive morse message to display it by buzzer.
            for (int i = 0; i < strlen(serialReceivedMorseMessage.message); i++)
            {
                if (serialReceivedMorseMessage.message[i] == '.')
                {
                    // If read character is a dot -> play with tone 440 for 500ms
                    buzzer_play_tone(440, 500);
                    // interrupt the flow for 300ms
                    vTaskDelay(pdMS_TO_TICKS(300));
                }
                else if (serialReceivedMorseMessage.message[i] == '-')
                {
                    // If read character is a dash -> play with tone 440 for 1000ms
                    buzzer_play_tone(440, 1000);
                    // interrupt the flow for 300ms
                    vTaskDelay(pdMS_TO_TICKS(300));
                }
                else if (serialReceivedMorseMessage.message[i] == ' ')
                {
                    // If read character is a space -> interrupt the flow for 300ms
                    vTaskDelay(pdMS_TO_TICKS(300));
                }
            }
            // After finishing, notify the display controller
            xTaskNotifyGive(displayControllerTask);
            // Set the isGiveNotify value to 1 so that it will not be displayed infinitely.
            isGiveNotify = 1;
        }
        else if (programState == DISPLAY_FINISHED)
        {
            // If the programState equal DISPLAY_FINISHED -> all 3 display finished
            // Now we can reset the isGiveNotify to be able to display another message
            isGiveNotify = 0;
        }
        vTaskDelay(100);
    }
}
void translate_receives_morse_codes()
{
    // Define a struct type of morse alphabet character to store the recognized character from morse message.
    struct MorseAlphabet singleMorseCode = {0};
    for (int i = 0; i < INPUT_BUFFER_SIZE; i++)
    {
        // If the read character is not a space
        if (serialReceivedMorseMessage.message[i] != ' ')
        {
            // Store the read character (dot/dash) to the morse alphabet character to be waited to be translated.
            singleMorseCode.morseCode[singleMorseCode.currMorseCodeIndex] = serialReceivedMorseMessage.message[i];
            // Increase the current index by 1
            singleMorseCode.currMorseCodeIndex += 1;
        }
        else
        {
            // If it is a space, check if the previous position is a space or not
            if (serialReceivedMorseMessage.message[i - 1] == ' ')
            {
                // If there exists a space before it, we need to check if the next position is a '\0' or '\n' to stop the program.
                if (serialReceivedMorseMessage.message[i + 1] == '\0' || serialReceivedMorseMessage.message[i + 1] == '\n')
                {
                    add_character_to_string(&translatedMessage, '\n', translatedMessage.currentIndex + 1);
                    add_character_to_string(&translatedMessage, '\0', 0);
                    break;
                }
                else
                {
                    // If the next character is not '\0' or '\n' -> we put a space in the translated string to seperate 2 sentences.
                    add_character_to_string(&translatedMessage, ' ', translatedMessage.currentIndex + 1);
                }
            }
            else
            {
                // If it is the first time reading the space -> we have to search the current read morse code for a normal letter.
                // Terminate the read morse string
                singleMorseCode.morseCode[singleMorseCode.currMorseCodeIndex] = '\0';
                // Reset the current morse index.
                singleMorseCode.currMorseCodeIndex = 0;
                // Find the correct letter base on the read morse string above.
                char translatedLetter = find_letter_from_morse_code(singleMorseCode.morseCode);
                // Add it to the translated string and increase the index by 1
                add_character_to_string(&translatedMessage, translatedLetter, translatedMessage.currentIndex + 1);
            }
        }
    }
    printf("__Translated string %s__\n", translatedMessage.message);
}

char find_letter_from_morse_code(char *morseCode)
{
    // Loops through the alphabet to check for the matched letter of the morse code.
    for (int i = 0; i < MORSE_ALPHABET_SIZE; i++)
    {
        if (strcmp(morseCode, morseCodes[i].morseCode) == 0)
        {
            return morseCodes[i].letter;
        }
    }
    // If find nothing, return 'n'
    return 'n';
}

static void buzzer_music_play()
{
    // Function to play music
    // Array to store different tone of the song.
    int melody[SONG_TONE_SIZE] = {
        330, 330, 330, 330, 330, 330, 330, 392, 262, 294, 330, 349, 349, 349, 330, 330,
        330, 330, 330, 330, 330, 392, 262, 294, 330};
    // Array to store the duration of each tone above
    int durations[SONG_TONE_SIZE] = {
        250, 250, 500, 250, 250, 500, 250, 250, 250, 250, 500, 250, 250, 500, 250, 250,
        250, 250, 500, 250, 250, 500, 250, 250, 500};

    // Loops through 2 array above and play the song
    for (int i = 0; i < SONG_TONE_SIZE; i++)
    {
        // Play the note with the correct melody and duration
        buzzer_play_tone(melody[i], durations[i]);
        // Sleep 50ms for interruption between each note.
        sleep_ms(50);
    }
}

static void serial_receive_task(void *arg)
{
    // Function to receive the string from the serial client
    (void)arg;

    while (true)
    {
        // take one char per time and store it in line array, until reeceived the \n
        int receivedChar = getchar_timeout_us(0);
        if (receivedChar != PICO_ERROR_TIMEOUT)
        {
            // If the programState is not in the DISPLAY mode then we can update the message
            if (programState != DISPLAY)
            {
                // If it is character '\r' -> ignore.
                if (receivedChar == '\r')
                    continue;
                if (serialReceivedMorseMessage.currentIndex >= INPUT_BUFFER_SIZE)
                {
                    // If it reach the overflow -> update the last index to be '\0' and reset the currentindex
                    serialReceivedMorseMessage.currentIndex -= 1;
                    add_character_to_string(&serialReceivedMorseMessage, '\0', 0);
                    printf("__Overflow text warning__\n");
                    // Set the programState to display the string
                    programState = DISPLAY;
                }
                else if (receivedChar == '\n')
                {
                    // If the read character is '\n' -> we need to terminate the string and display it
                    // Send the announcement that we receive string with buzzer sound
                    sending_feedback();
                    sleep_ms(500);
                    add_character_to_string(&serialReceivedMorseMessage, '\0', 0);
                    programState = DISPLAY;
                    printf("__Received String %s__\n", serialReceivedMorseMessage.message);
                }
                else
                {
                    // If it is normal morse character -> add it to the current position and update the current position by 1
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

static void lcd_display_task(void *pvParameters)
{
    // Get the TaskHandle of the display controller from the parameters
    TaskHandle_t displayControllerTask = (TaskHandle_t)pvParameters;
    // Value to check if the task is finished for one cycle and send already notify the display controller
    // Value of 0 -> not give notification yet, value of 1 -> already give notification
    static int isGiveNotify = 0;
    write_text("Waiting...");
    while (true)
    {
        // If the programState is in DISPLAY mode and have not sent the notification to the display controller
        if (programState == DISPLAY && isGiveNotify == 0)
        {
            // Translate the serial morse code.
            translate_receives_morse_codes();
            // Get the length of the translated message
            int length = strlen(translatedMessage.message);
            // Initialize the display string with length 10
            char displayString[WINDOW_SIZE + 1];
            // Clear the display
            clear_display();
            // Create the sliding window with the size of 9 -> showing 9 character each loop.
            for (int i = 0; i <= length - WINDOW_SIZE; i++)
            {
                // Copy the from the current i to character at position i + WINDOWSIZE - 1 to the display string
                memcpy(displayString, &translatedMessage.message[i], WINDOW_SIZE);
                // Set the position at WINDOW_SIZE = '\0' to terminate the string.
                displayString[WINDOW_SIZE] = '\0';
                printf("__Display string %s__\n", displayString);
                // Write the display string to the lcd for 500ms
                write_text(displayString);
                vTaskDelay(pdMS_TO_TICKS(500));
                // Clear the display after that.
                clear_display();
            }
            // After finishing showing sliding text -> write back to Waiting... string
            write_text("Waiting...");
            // Notify the display controller task that lcd task is finish
            xTaskNotifyGive(displayControllerTask);
            // Set is notify to equal 1 -> not in an infinite cycle of showing it on the lcd
            isGiveNotify = 1;
        }
        else if (programState == DISPLAY_FINISHED)
        {
            // If the programState equal DISPLAY_FINISHED -> all 3 display finished
            // Now we can reset the isGiveNotify to be able to display another message
            isGiveNotify = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

static void display_controller_task(void *args)
{
    // Count the finished display task (lcd, buzzer, rgb)
    int finishedDisplayTask = 0;
    while (true)
    {
        // If counter < 3 -> waiting for notification from the 3 display method
        if (finishedDisplayTask < 3)
        {
            // NOte that this function is not CPU blocking, it only run when receive notification
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            // If receive notification -> increase the counter by 1
            finishedDisplayTask++;
        }
        else
        {
            // If counter is >= 3 -> Enough notification from 3 display
            // Reset the counter for another display cycle
            finishedDisplayTask = 0;
            // Set the programState to be DISPLAY_FINISHED so that 3 display task can reset their isGiveNotify
            programState = DISPLAY_FINISHED;
            printf("__3 task display finished__\n");
        }
    }
}

void sending_feedback()
{
    // Function to play a buzzer to announce.
    buzzer_play_tone(640, 500);
}

void wirelessTask()
{
    // Initializing CYW43439
    if (cyw43_arch_init())
    {
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
    if (cyw43_arch_wifi_connect_timeout_ms("phuc", "1231232312123", CYW43_AUTH_WPA2_AES_PSK, 30 * 1000))
    {
        printf("__Failed to connect__\n");
    }
    else
    {
        printf("__Connected to wifi\n");
    }

    printf("__Run test__\n");
    run_tcp_client_test();
}

static err_t tcp_client_close(void *arg)
{
    TCP_CLIENT_T *clientState = (TCP_CLIENT_T *)arg;
    err_t err = ERR_OK;
    if (clientState->tcp_pcb != NULL)
    {
        tcp_arg(clientState->tcp_pcb, NULL);
        tcp_poll(clientState->tcp_pcb, NULL, 0);
        tcp_sent(clientState->tcp_pcb, NULL);
        tcp_recv(clientState->tcp_pcb, NULL);
        tcp_err(clientState->tcp_pcb, NULL);
        err = tcp_close(clientState->tcp_pcb);
        if (err != ERR_OK)
        {
            DEBUG_printf("__close failed %d, calling abort__\n", err);
            tcp_abort(clientState->tcp_pcb);
            err = ERR_ABRT;
        }
        clientState->tcp_pcb = NULL;
    }
    return err;
}

static err_t tcp_result(void *arg, int status)
{
    TCP_CLIENT_T *clientState = (TCP_CLIENT_T *)arg;
    if (status == 0)
    {
        DEBUG_printf("test success\n");
    }
    else
    {
        DEBUG_printf("test failed %d\n", status);
    }
    clientState->complete = true;
    return tcp_client_close(arg);
}

static err_t tcp_client_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
    TCP_CLIENT_T *clientState = (TCP_CLIENT_T *)arg;
    DEBUG_printf("__tcp_client_sent %u__\n", len);
    clientState->sent_len += len;

    if (clientState->sent_len >= BUF_SIZE)
    {

        // We should receive a new buffer from the server
        clientState->buffer_len = 0;
        clientState->sent_len = 0;
        DEBUG_printf("__Waiting for buffer from server__\n");
    }

    return ERR_OK;
}

static err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err)
{
    TCP_CLIENT_T *clientState = (TCP_CLIENT_T *)arg;
    if (err != ERR_OK)
    {
        printf("__connect failed %d__\n", err);
        return tcp_result(arg, err);
    }
    clientState->connected = true;
    const char *msg = "Hello Server";
    tcp_write(tpcb, msg, strlen(msg), TCP_WRITE_FLAG_COPY);

    DEBUG_printf("__Waiting for buffer from server___\n");
    return ERR_OK;
}

static err_t tcp_client_poll(void *arg, struct tcp_pcb *tpcb)
{
    // DEBUG_printf("tcp_client_poll\n");
    return ERR_OK; // no response is an error?
}

static void tcp_client_err(void *arg, err_t err)
{
    TCP_CLIENT_T *clientState = (TCP_CLIENT_T *)arg;
    if (err != ERR_ABRT)
    {
        DEBUG_printf("__tcp_client_err %d__\n", err);
        tcp_client_close(clientState);
    }
    else
    {
        free(clientState);
        printf("__Client state freed due to abort error__\n");
        return;
    }
}

err_t tcp_client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    TCP_CLIENT_T *clientState = (TCP_CLIENT_T *)arg;
    if (!p)
    {
        return tcp_result(arg, -1);
    }
    if (programState == DISPLAY)
    {
        // If currently in display mode -> ignore the received data
        printf("__Currently in display mode, ignore received data__\n");
        pbuf_free(p);
        return ERR_OK;
    }
    // this method is callback from lwIP, so cyw43_arch_lwip_begin is not required, however you
    // can use this method to cause an assertion in debug mode, if this method is called when
    // cyw43_arch_lwip_begin IS needed
    cyw43_arch_lwip_check();
    if (p->tot_len > 0)
    {
        DEBUG_printf("__recv %d err %d__\n", p->tot_len, err);
        for (struct pbuf *q = p; q != NULL; q = q->next)
        {
            DUMP_BYTES(q->payload, q->len);
        }
        // Receive the buffer
        const uint16_t buffer_left = BUF_SIZE - clientState->buffer_len;
        clientState->buffer_len += pbuf_copy_partial(p, clientState->buffer + clientState->buffer_len,
                                                     p->tot_len > buffer_left ? buffer_left : p->tot_len, 0);
        tcp_recved(tpcb, p->tot_len);
    }
    pbuf_free(p);

    if (clientState->buffer_len == BUF_SIZE)
    {
        // Send the announcement that we receive string with buzzer sound
        sending_feedback();
        DUMP_BYTES(clientState->buffer, clientState->buffer_len);
        clientState->buffer[0] = '\0';
        clientState->buffer_len = 0;
    }
    return ERR_OK;
}

static bool tcp_client_open(void *arg)
{
    TCP_CLIENT_T *clientState = (TCP_CLIENT_T *)arg;
    DEBUG_printf("__Connecting to %s port %u__\n", ip4addr_ntoa(&clientState->remote_addr), TCP_PORT);
    clientState->tcp_pcb = tcp_new_ip_type(IP_GET_TYPE(&clientState->remote_addr));
    if (!clientState->tcp_pcb)
    {
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
static TCP_CLIENT_T *tcp_client_init(void)
{
    clientState = calloc(1, sizeof(TCP_CLIENT_T));
    if (!clientState)
    {
        DEBUG_printf("failed to allocate state\n");
        return NULL;
    }
    ip4addr_aton(TEST_TCP_SERVER_IP, &clientState->remote_addr);
    return clientState;
}
void run_tcp_client_test(void)
{
    TCP_CLIENT_T *clientState = tcp_client_init();
    if (!clientState)
    {
        printf("__No state__\n");
        return;
    }
    if (!tcp_client_open(clientState))
    {
        printf("__Cannot open__\n");
        tcp_result(clientState, -1);
        return;
    }
}

void send_data_tcp()
{

    printf("__Sending condition connected=%d__\n", clientState->connected);
    if (clientState == NULL)
    {
        printf("__Client state is null__\n");
    }
    // Check if the clientState is NULL or the connected boolean is false -> if it is -> return.
    if (clientState == NULL || !clientState->connected)
        return;
    // If clientState is not null and the connected boolean is true -> send the data over the tcp_server
    printf("__Send data over TCP__\n");
    tcp_write(clientState->tcp_pcb, imuMorseMessage.message, strlen(imuMorseMessage.message), TCP_WRITE_FLAG_COPY);
}
void light_sensor_task(void *pvParameters)
{
    // Function to send space with light sensor
    (void)pvParameters;
    uint32_t ambientLight;
    while (1)
    {
        // Only read the data from the ambientlight when programState is DATA_READY or SPACES_REQUIREMENTS_SATISFIED
        if (programState == DATA_READY || programState == SPACES_REQUIREMENTS_SATISFIED)
        {
            // Read the ambientlight
            ambientLight = veml6030_read_light();
            printf("light sensor %u\n", ambientLight);
            // Check is it below the ambient threshold
            if (ambientLight < LIGHT_THRESHOLD)
            {
                // If it is, then check if it is in the 2 space consecutive mode or not
                if (programState == SPACES_REQUIREMENTS_SATISFIED)
                {
                    // If it is then terminate the string and update the current index.
                    add_character_to_string(&imuMorseMessage, '\n', imuMorseMessage.currentIndex + 1);
                    add_character_to_string(&imuMorseMessage, '\0', 0);
                    // Set the programState to be in mode of send data
                    programState = SEND_DATA;
                }

                else
                {
                    // If it is not 2 space consecutively already
                    printf("Light Button: Send space\n");
                    // Check if the previous position is the space
                    if (imuMorseMessage.message[imuMorseMessage.currentIndex - 1] == ' ')
                    {
                        // If it is then we will update the programState
                        printf("Light Button: 2 spaces consecutively\n");
                        programState = SPACES_REQUIREMENTS_SATISFIED;
                    }
                    // Add the space to the message string and update the current index
                    add_character_to_string(&imuMorseMessage, ' ', imuMorseMessage.currentIndex + 1);
                    // Delay for 2000ms to interrupt the light sensor (if not, it will update space really fast because sensor read really fast)
                    vTaskDelay(pdMS_TO_TICKS(2000));
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}