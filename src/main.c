#include <stdio.h>
#include <pico/stdio.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <string.h>

#include "tkjhat/sdk.h"
// Program global state to manage the flow of the program
enum state {IDLE=1, WAITING_DATA, DATA_READY, SEND_DATA, SEND_REQ_SATISFIED, RECEIVED_DATA};
// Starting with IDLE state
enum state programState=IDLE;

// Data structure for morse messagees
struct MorseMessage {
   char lastReadMorseCharacter; // Last read morse character
   int currentMorseStringPosition; // Current index in the morse string
   char morseString[120]; // Strring to store the morse received.
};

// Global variable
struct MorseMessage message = {0};

// Initialize the variable with struct type above to be a default value = 0.
struct MorseMessage imuMorseMessage = {0};

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
static void receive_task(void *arg);
void rgb_task(void *pvParameters);
void buzzer_task(void *pvParameters);

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
    // Set the interruption for both button1, button2 using together btn_fxn function.
    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_RISE, true, btn_fxn);
    gpio_set_irq_enabled(BUTTON2, GPIO_IRQ_EDGE_RISE, true);

    // TaskHandle for serial_send_task function
    TaskHandle_t serialSendTask;
    // TaskHandle for imu_task function
    TaskHandle_t hIMUTask = NULL;
    TaskHandle_t hReceiveTask;
    

    // Create all task with priority 2, no args.
    BaseType_t result = xTaskCreate(serial_send_task, "serialSendTask", 2048, NULL, 2, &serialSendTask);
    xTaskCreate(imu_task, "IMUTask", 1024, NULL, 2, &hIMUTask);
    xTaskCreate(receive_task, "receive", 1024, NULL, 2, &hReceiveTask);
    //xTaskCreate(rgb_task, "RGBTask", 256, NULL, 2, NULL);
    xTaskCreate(buzzer_task,"BuzzerTask", 1024, NULL, 2, NULL );
    
    // Start to run and schedule the task
    vTaskStartScheduler();

    return 0;
}

void imu_task(void *pvParameters){
    (void)pvParameters;

    // Variabble to store imu data received.
    float ax, ay, az, gx, gy, gz, temp;

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

    while(1){
        // Read the data received from Accelerometer and Gyroscope
            if (ICM42670_read_sensor_data(&ax, &ay, &az,&gx,&gy,&gz,&temp) == 0){
                // If the programState is WAITING_DATA -> convert the received data to morse code and store it in the morse string.
                if(programState == WAITING_DATA){
                    printf("Accel: X=%f, Y=%f, Z=%f | Gyro: X=%f, Y=%f, Z=%f| Temp: %2.2f°C\n", ax, ay, az, gx, gy, gz, temp);
                    // Function to convert received data to morse character.
                    convert_to_morse_character(&ax, &ay, &az);
                }
            }else {
                printf("Failed to read data from IMU sensor\n");
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
            printf("Receive morse message %s\n", imuMorseMessage.morseString);
            // Reset the morse string index = 0
            imuMorseMessage.currentMorseStringPosition = 0;
            // Reset the morse string to be empty.
            imuMorseMessage.morseString[imuMorseMessage.currentMorseStringPosition] = '\0';
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
        if (programState == SEND_REQ_SATISFIED){
            // If there are 2 consecutive spaces -> set the current position element in morse string to be \n.
            imuMorseMessage.morseString[imuMorseMessage.currentMorseStringPosition] = '\n';
            // Increase the index by 1
            imuMorseMessage.currentMorseStringPosition += 1;
            // Set the current position element in morse string to be \0 to stop the string
            imuMorseMessage.morseString[imuMorseMessage.currentMorseStringPosition] = '\0';
            // Set the programState to be SEND_DATA to send it to serial monitor
            programState = SEND_DATA;
        }else if(programState == DATA_READY) {
            // If we just received a morse character from imu and there exists no 2 consecutive spaces
            // Then we put a space in the current position in morse string
            imuMorseMessage.morseString[imuMorseMessage.currentMorseStringPosition] = ' ';
            printf("Send space\n");
            // Check if the morse string before the current 
            if (imuMorseMessage.morseString[imuMorseMessage.currentMorseStringPosition - 1] == ' '){
                printf("2 space consecutively detected\n");
                programState = SEND_REQ_SATISFIED;
            }
            // Increase the index bby 1
            imuMorseMessage.currentMorseStringPosition += 1;
        }else {
            printf("Unavailble to send the space here\n");
        }

    }
}

void convert_to_morse_character(float *accelX, float *accelY, float *accelZ){
    // Check if the position of the IMU sensor match the condition
    if ((*accelX > -0.1 && *accelX < 0.1) && (*accelY > -0.1 && *accelY < 0.1) && (*accelZ > 0.9 && *accelZ < 1.1)){
        // If the position of imu is place horizontally -> set the current position of the morse string to be a dot.
        imuMorseMessage.morseString[imuMorseMessage.currentMorseStringPosition] = '.';
        // Increase the morse string index by 1
        imuMorseMessage.currentMorseStringPosition += 1;
        printf("Received Morse character '.' and go back to DATA_READY\n");
        // Set the programState to be DATA_READY to be able to send space
        programState = DATA_READY;
    }else if ( (*accelX > -0.1 && *accelX < 0.1) && (*accelZ > -0.1 && *accelZ < 0.1) && (*accelY < -0.9 && *accelY > -1.1)){
        // If the position of imu is place horizontally -> set the current position of the morse string to be a dash.
        imuMorseMessage.morseString[imuMorseMessage.currentMorseStringPosition] = '-';
        // Increase the morse string index by 1
        imuMorseMessage.currentMorseStringPosition += 1;
        printf("Received Morse character '-' and go back to DATA_READY\n");
        // Set the programState to be DATA_READY to be able to send space
        programState = DATA_READY;
    }
}

static void receive_task(void *arg){
    (void)arg;
    size_t index = 0;
    
    while (1){
        //OPTION 1
        // Using getchar_timeout_us https://www.raspberrypi.com/documentation/pico-sdk/runtime.html#group_pico_stdio_1ga5d24f1a711eba3e0084b6310f6478c1a
        // take one char per time and store it in line array, until reeceived the \n
        // The application should instead play a sound, or blink a LED. 
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT){// I have received a character
            if (c == '\r') continue; // ignore CR, wait for LF if (ch == '\n') { line[len] = '\0';
            if (c == '\n'){
                // terminate and process the collected line
                message.morseString[message.currentMorseStringPosition] = '\0';
                message.currentMorseStringPosition = 0;
                programState = RECEIVED_DATA;
                printf("Received Morse String %s",message.morseString);
                
                vTaskDelay(pdMS_TO_TICKS(100)); // Wait for new message
                
            }
            else if(index < 174 - 1){
                message.morseString[message.currentMorseStringPosition] = (char)c;
                message.currentMorseStringPosition++;
            }
            else { //Overflow: print and restart the buffer with the new character. 
                message.morseString[message.currentMorseStringPosition] = '\0';
                printf("Receive morse code: %s\n", message.morseString);
                message.currentMorseStringPosition = 0; 
                message.morseString[message.currentMorseStringPosition] = (char)c; 
                message.currentMorseStringPosition++;
            }
        }
        else {
            vTaskDelay(pdMS_TO_TICKS(100)); // Wait for new message
        }
        //OPTION 2. Use the whole buffer. 
        /*absolute_time_t next = delayed_by_us(get_absolute_time,500);//Wait 500 us
        int read = stdio_get_until(line,INPUT_BUFFER_SIZE,next);
        if (read == PICO_ERROR_TIMEOUT){
            vTaskDelay(pdMS_TO_TICKS(100)); // Wait for new message
        }
        else {
            line[read] = '\0'; //Last character is 0
            printf("__[RX] \"%s\"\n__", line);
            vTaskDelay(pdMS_TO_TICKS(50));
        }*/
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