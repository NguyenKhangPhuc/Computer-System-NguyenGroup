#include <stdio.h>
#include <string.h>
#include <pico/stdio.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <string.h>

#include "tkjhat/sdk.h"



// Program global state to manage the flow of the program
enum state {IDLE=1, WAITING_DATA, DATA_READY, SEND_DATA, SPACES_REQUIREMENTS_SATISFIED, DISPLAY, RECEIVED_DATA};
// Starting with IDLE state
enum state programState=IDLE;

struct MorseAlphabet {
    char morseCode[7];
    char letter;
    int currMorseCodeIndex;
};

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

// Global variable
struct MorseMessage message = {0};

// Initialize the variable with struct type above to be a default value = 0.
struct Message imuMorseMessage = {0};
struct Message serialReceivedMorseMessage = {0};
struct Message translatedMessage = {0};
// char serialReceivedMorseMessage[120] = ".... . .-.. .-.. ---  .-- --- .-. .-.. -.."; 

// Prototype for functions
// Function to receive daata from IMU sensor (Accelerometer and GyroScope)
void imu_task(void *pvParameters);
// Function to send the morse string to the serial monitor.
static void serial_send_task(void *arg);
// Function to be called when button interruption appear
static void btn_fxn(uint gpio, uint32_t eventMask);
// Function to convert data from IMU to morse character
void convert_to_morse_character(float *accleX, float *accelY, float *accelZ);
// Serial receive function
void rgb_task(void *pvParameters);
void buzzer_task(void *pvParameters);
void translate_receives_morse_codes(void);
char find_letter_from_morse_code(char *morseCode);
static void buzzer_task(void *arg);
static void lcd_display_task(void *args);
static void serial_receive_task(void *arg);


int main() {
    // Initialize the stdio to be able to send and receive data.
    stdio_init_all();
    while (!stdio_usb_connected()){
        // If the serial monitor is not monitoring, then continue to wait.
        sleep_ms(10);
    }
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
    init_buzzer();

    init_display();
    // Check the connection the ICM-42670 sensor (0=success, otherwise fail the connection).
    if (init_ICM42670() == 0){
        printf("ICM42670 initialize successfully\n");
        // If initialize the ICM-42670 sensor successfuly, configure the parameter inside to be default value
        // to be ready for measuring.
        if (ICM42670_start_with_default_values() != 0){
            printf("ICM42670P cound not initialize the accelerometer or gyroscope\n");
        }
    }else {
        printf("ICM42670 could not be initialized\n");
    }
    // Set the interruption for both button1, button2 using together btn_fxn function.
    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_RISE, true, btn_fxn);
    gpio_set_irq_enabled(BUTTON2, GPIO_IRQ_EDGE_RISE, true);

    // TaskHandle for serial_send_task function
    TaskHandle_t serialSendTask;
    // TaskHandle for imu_task function
    TaskHandle_t hIMUTask = NULL;
    TaskHandle_t hReceiveTask;
    

    TaskHandle_t buzzerTask = NULL;
    TaskHandle_t serialReceiveTask = NULL;
    TaskHandle_t lcdDisplay = NULL;
    // Create all task with priority 2, no args.
    BaseType_t result = xTaskCreate(serial_send_task, "serialSendTask", 2048, NULL, 2, &serialSendTask);
    //xTaskCreate(rgb_task, "RGBTask", 256, NULL, 2, NULL);
    xTaskCreate(buzzer_task,"BuzzerTask", 1024, NULL, 2, NULL );
    
    xTaskCreate(imu_task, "IMUTask", 1024, NULL, 3, &hIMUTask);
    // xTaskCreate(buzzer_task, "buzzerTask", 1024, NULL, 2, &buzzerTask);
    xTaskCreate(serial_receive_task, "serialReceiveTask", 1024, NULL, 2, &serialReceiveTask);
    xTaskCreate(lcd_display_task, "lcdTask", 1024, NULL, 2, &lcdDisplay);
    // Start to run and schedule the task
    vTaskStartScheduler();

    return 0;
}

void imu_task(void *pvParameters){
    (void)pvParameters;

    // Variabble to store imu data received.
    float ax, ay, az, gx, gy, gz, temp;

    while(1){
        // Read the data received from Accelerometer and Gyroscope
            if (programState == WAITING_DATA){
                if (ICM42670_read_sensor_data(&ax, &ay, &az,&gx,&gy,&gz,&temp) == 0){
                // If the programState is WAITING_DATA -> convert the received data to morse code and store it in the morse string.
                    printf("Accel: X=%f, Y=%f, Z=%f | Gyro: X=%f, Y=%f, Z=%f| Temp: %2.2f°C\n", ax, ay, az, gx, gy, gz, temp);
                    // Function to convert received data to morse character.
                    convert_to_morse_character(&ax, &ay, &az);       
                }else {
                    printf("Failed to read data from IMU sensor\n");
                }
            }
            
        // Stop 100ms to run other tasks.
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void serial_send_task(void *arg){
    (void)arg;

    while(1){
        // If the progrramState is SEND_DATA -> Send the morsee string to serial monitor.
        if (programState == SEND_DATA){
            // Send the morse string
            printf("Receive morse message %s\n", imuMorseMessage.message);
            // Reset the morse string index = 0
            imuMorseMessage.currentIndex = 0;
            // Reset the morse string to be empty.
            imuMorseMessage.message[imuMorseMessage.currentIndex] = '\0';
            // Set  the programState to be equal
            programState = IDLE;
        }
        // Stop 100ms to run other tasks
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


static void btn_fxn(uint gpio, uint32_t eventMask){
    // Check if the button pressed is button1 or button2
    if (gpio == BUTTON1){
        // If button1 -> set the state to WAITING_DATA to received IMU sensor data
        programState = WAITING_DATA;
        printf("Start to read sensor data\n");
    }else if (gpio == BUTTON2){
        // If button2 -> first check if there are 2 consecutive spaces already.
        if (programState == SPACES_REQUIREMENTS_SATISFIED){
            // If there are 2 consecutive spaces -> set the current position element in morse string to be \n.
            imuMorseMessage.message[imuMorseMessage.currentIndex] = '\n';
            // Increase the index by 1
            imuMorseMessage.currentIndex += 1;
            // Set the current position element in morse string to be \0 to stop the string
            imuMorseMessage.message[imuMorseMessage.currentIndex] = '\0';
            // Set the programState to be SEND_DATA to send it to serial monitor
            programState = SEND_DATA;
        }else if(programState == DATA_READY) {
            // If we just received a morse character from imu and there exists no 2 consecutive spaces
            // Then we put a space in the current position in morse string
            imuMorseMessage.message[imuMorseMessage.currentIndex] = ' ';
            printf("Send space\n");
            // Check if the morse string before the current 
            if (imuMorseMessage.message[imuMorseMessage.currentIndex - 1] == ' '){
                printf("2 space consecutively detected\n");
                programState = SPACES_REQUIREMENTS_SATISFIED;
            }
            // Increase the index bby 1
            imuMorseMessage.currentIndex += 1;
        }else {
            printf("Unavailble to send the space here\n");
        }

    }
}

void convert_to_morse_character(float *accelX, float *accelY, float *accelZ){
    // Check if the position of the IMU sensor match the condition
    if ((*accelX > -0.1 && *accelX < 0.1) && (*accelY > -0.1 && *accelY < 0.1) && (*accelZ > 0.9 && *accelZ < 1.1)){
        // If the position of imu is place horizontally -> set the current position of the morse string to be a dot.
        imuMorseMessage.message[imuMorseMessage.currentIndex] = '.';
        // Increase the morse string index by 1
        imuMorseMessage.currentIndex += 1;
        printf("Received Morse character '.' and go back to DATA_READY\n");
        // Set the programState to be DATA_READY to be able to send space
        programState = DATA_READY;
    }else if ( (*accelX > -0.1 && *accelX < 0.1) && (*accelZ > -0.1 && *accelZ < 0.1) && (*accelY < -0.9 && *accelY > -1.1)){
        // If the position of imu is place horizontally -> set the current position of the morse string to be a dash.
        imuMorseMessage.message[imuMorseMessage.currentIndex] = '-';
        // Increase the morse string index by 1
        imuMorseMessage.currentIndex += 1;
        printf("Received Morse character '-' and go back to DATA_READY\n");
        // Set the programState to be DATA_READY to be able to send space
        programState = DATA_READY;
    }
}

void rgb_task(void *pvParameters) {
    (void)pvParameters;

    while (1) {
    if (programState == RECEIVED_DATA) {
        for (int i=0; i < strlen(message.morseString); i++) {
            printf ("Read morse character %c", message.morseString[i]);
            if (message.morseString[i] == '.') {
                rgb_led_write(0, 0, 255);
                vTaskDelay(pdMS_TO_TICKS(1500));
                rgb_led_write(255, 255, 255);
                vTaskDelay(pdMS_TO_TICKS(300));
            }
            else if (message.morseString[i] == '-') {
                rgb_led_write(0, 255, 0);
                vTaskDelay(pdMS_TO_TICKS(1500));
                rgb_led_write(255, 255, 255);
                vTaskDelay(pdMS_TO_TICKS(300));
            }
            else if (message.morseString[i] == ' ') {
                rgb_led_write(255, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(1500));
                rgb_led_write(255, 255, 255);
                vTaskDelay(pdMS_TO_TICKS(300));
            }
        }
        programState = IDLE;
    }
        

        // rgb_led_write(20, 30, 255);
        // vTaskDelay(500);
        // rgb_led_write(255, 30, 10);
        // vTaskDelay(500);
        // rgb_led_write(50, 255, 10);
        // vTaskDelay(500);
    }
}

void buzzer_task(void *pvParameters) {
    (void)pvParameters;

    while (1) {
        if (programState == RECEIVED_DATA) {
            for(int i =0; i< strlen(message.morseString); i++){
                if (message.morseString[i] == '.') {
                    buzzer_play_tone(440, 500);
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
                else if (message.morseString[i] == '-') {
                    buzzer_play_tone(440, 1000);
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
                else if (message.morseString[i] == ' ') {
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }

            }
            programState = IDLE;
 
        }
        
        vTaskDelay(100);
    }
}
void translate_receives_morse_codes(){
    printf("Start\n");
    struct MorseAlphabet singleMorseCode = {0};
    for (int i = 0; i < 120; i++){
        printf("Letter %c\n",  serialReceivedMorseMessage.message[i]);
        if (serialReceivedMorseMessage.message[i] == '\0'){
            singleMorseCode.morseCode[singleMorseCode.currMorseCodeIndex] = '\0';
            singleMorseCode.currMorseCodeIndex = 0;
            char translatedLetter = find_letter_from_morse_code(singleMorseCode.morseCode);
            translatedMessage.message[translatedMessage.currentIndex] = translatedLetter;
            translatedMessage.currentIndex +=1;
            translatedMessage.message[translatedMessage.currentIndex] = '\n';
            translatedMessage.currentIndex +=1;
            translatedMessage.message[translatedMessage.currentIndex] = '\0';
            translatedMessage.currentIndex = 0;
            break;
        }
        if (serialReceivedMorseMessage.message[i] != ' '){
            
            singleMorseCode.morseCode[singleMorseCode.currMorseCodeIndex] = serialReceivedMorseMessage.message[i];
            singleMorseCode.currMorseCodeIndex+= 1;
        }else {
            if (serialReceivedMorseMessage.message[i - 1] == ' '){
                translatedMessage.message[translatedMessage.currentIndex] = ' ';
                translatedMessage.currentIndex +=1;
            }else{
                singleMorseCode.morseCode[singleMorseCode.currMorseCodeIndex] = '\0';
                singleMorseCode.currMorseCodeIndex = 0;
                char translatedLetter = find_letter_from_morse_code(singleMorseCode.morseCode);
                translatedMessage.message[translatedMessage.currentIndex] = translatedLetter;
                translatedMessage.currentIndex +=1;
            }
        }
    }
    printf("Translated string %s\n", translatedMessage.message);
}

char find_letter_from_morse_code(char *morseCode){
    for (int i = 0; i < 40; i++){
        if (strcmp(morseCode, morseCodes[i].morseCode) == 0){
            printf("Checked %c ", morseCodes[i].letter);
            return morseCodes[i].letter;
        }  
    }
    return 'n';
}

static void buzzer_task(void *arg){
    (void) arg;

    init_buzzer();
    printf("Initializing buzzer\n");
    int melody[25] = {
        330, 330, 330, 330, 330, 330, 330, 392, 262, 294, 330, 349, 349, 349, 330, 330,
        330, 330, 330, 330, 330, 392, 262, 294, 330
    };

    int durations[25] = {
        250, 250, 500, 250, 250, 500, 250, 250, 250, 250, 500, 250, 250, 500, 250, 250,
        250, 250, 500, 250, 250, 500, 250, 250, 500
    };

    int length = sizeof(melody)/ sizeof(melody[0]);
    while(1){
        printf("Music Play \n");
        for (int i = 0 ; i < length; i++){
            buzzer_play_tone(melody[i], durations[i]);
            sleep_ms(50);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

static void serial_receive_task(void *arg){
    (void) arg;

    while (true){
        int receivedChar = getchar_timeout_us(0);
        if (receivedChar != PICO_ERROR_TIMEOUT){
            if (receivedChar == '\r') continue;
            if (serialReceivedMorseMessage.currentIndex >= 120 - 1){
                serialReceivedMorseMessage.message[serialReceivedMorseMessage.currentIndex] = '\0';
                serialReceivedMorseMessage.currentIndex = 0;
                printf("Overflow text warning\n");
                programState = DISPLAY;
            }else if (receivedChar == '\n'){
                serialReceivedMorseMessage.message[serialReceivedMorseMessage.currentIndex] = '\0';
                serialReceivedMorseMessage.currentIndex = 0;
                programState = DISPLAY;
                printf("Received String %s\n",  serialReceivedMorseMessage.message);
            }else {
                serialReceivedMorseMessage.message[serialReceivedMorseMessage.currentIndex] = receivedChar;
                serialReceivedMorseMessage.currentIndex += 1;
                printf("Received letter=%c \n", receivedChar);
            }
        }else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

static void lcd_display_task(void *args){
    (void) args;
    write_text("Waiting...");
    while (true){
        
        if (programState == DISPLAY){
            translate_receives_morse_codes();
            int length = strlen(translatedMessage.message);
            char displayString[5];
            clear_display();
            for (int i = 0 ; i < length - 5; i++){
                sprintf(displayString, "%c%c%c%c%c", translatedMessage.message[i], translatedMessage.message[i + 1], translatedMessage.message[i + 2], translatedMessage.message[i + 3], translatedMessage.message[i + 4], translatedMessage.message[i + 4]);
                write_text(displayString);
                vTaskDelay(pdMS_TO_TICKS(50));
                clear_display();
            }
            write_text("Waiting...");
            programState = IDLE;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}