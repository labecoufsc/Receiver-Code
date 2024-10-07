#include <stdio.h>
#include "esp_log.h"
#include "mpu6050.h"
#include "SDcard.h"
#include "cppi2c.h"
#include <cstring>
#include "ublox7/ublox7.h"
#include "cppuart.h"
#include "SMS.h"
#include "ring_buffer.h"
#include <string.h>

static const char *TAG1 = "MPU6050";

//RTC_DATA_ATTR circbuf queue;

TaskHandle_t ACCELHandle;	
TaskHandle_t GPSHandle;
TaskHandle_t GSMHandle;
TaskHandle_t SDGPSHandle;
TaskHandle_t SDAccelHandle;
TaskHandle_t OnceHandle;

CPPUART::Uart uart {UART_NUM_1};
CPPI2C::I2c i2c {I2C_NUM_0};
//MPU6050 device(0x68);
UBLOX7 gps(UART_NUM_1);
SDcard sdcard;
Ringbuffer RingbufGPS(4096);
Ringbuffer RingbufAccel(24516);
SMS sms;
NAV_PVT nav_pvt;

using namespace std;
char buffer[EXAMPLE_MAX_CHAR_SIZE];
char GPSword[51];

char message[50] = "CCS these colours don't run";




static void Initialise_Task(void *arg)
{
	/*ESP_ERROR_CHECK(i2c.InitMaster(I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ, 1, 1, 0));
	ESP_LOGI(TAG1, "I2C initialized successfully");
	ESP_ERROR_CHECK(uart.uart_init(UB7_BAUD_9600, UB7_TXD, UB7_RXD, UB7_EN)); 
    ESP_ERROR_CHECK(gps.ub7_set_nav_mode(UB7_MODE_PEDESTRIAN));
    ESP_ERROR_CHECK(gps.ub7_set_message(UB7_MSG_RMC, false));
    ESP_ERROR_CHECK(gps.ub7_set_message(UB7_MSG_VTG, false));
    ESP_ERROR_CHECK(gps.ub7_set_message(UB7_MSG_GSA, false));
    ESP_ERROR_CHECK(gps.ub7_set_message(UB7_MSG_GLL, false));
    ESP_ERROR_CHECK(gps.ub7_set_message(UB7_MSG_GSV, false));
	ESP_ERROR_CHECK(gps.ub7_set_message(UB7_MSG_GGA, false));
    ESP_ERROR_CHECK(gps.ub7_set_output_rate(UB7_OUTPUT_1HZ));
	ESP_ERROR_CHECK(sdcard.SDcard_init());*/
	sms.GSM_init();
	runSingleGSMCommand(&deleteMSG1);
	vTaskDelay(5000/portTICK_PERIOD_MS); // wait for GPS initialisation to complete
	//sdcard.s_example_open_file(file_hello);
	// Get initial GPS reading for first Accel file time
	// ADD GPS READ HERE
	vTaskDelete(NULL);
}

// Create the task
void Once_Task(void *arg)
{
	TaskHandle_t InitialiseHandle;
	xTaskCreate(Initialise_Task, "Initialise_Task", 4096, NULL, 10, &InitialiseHandle);
	vTaskDelay(500/portTICK_PERIOD_MS);
	vTaskDelete(NULL);
}

void GSM_Task(void *arg) // number can be changed in sim800l/include/sim800l_cmds
{
	int length = 0;
	uint8_t CTRL_Z[]={0x1a};
	TickType_t xLastWakeTime;
 	const TickType_t xFrequency = 60;
    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

     for( ;; )
     {
        // Wait for the next cycle.
        vTaskDelayUntil( &xLastWakeTime, xFrequency);

		runSingleGSMCommand(&confTEXT);

		runSingleGSMCommand(&confSETUP);

		runSingleGSMCommand(&confPHONENUMBER);
		uart_write_bytes(UART_NUM_2, message, sizeof(message));
		uart_write_bytes(UART_NUM_2, CTRL_Z, sizeof(CTRL_Z));
		uart_wait_tx_done(UART_NUM_2, 200 / portTICK_PERIOD_MS);
	}
}

void GSM_Monitor_Task(void *arg) // number can be changed in sim800l/include/sim800l_cmds
{
	int length = 0;
	TickType_t xLastWakeTime;
 	const TickType_t xFrequency = 600;
	char buffer[256];

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

     for( ;; )
     {
        // Wait for the next cycle.
        vTaskDelayUntil( &xLastWakeTime, xFrequency);
		//runSingleGSMCommand(&listMSG); //RUN THIS COMMAND FIRST ALSO TRY TO DELETE ALL MESSAGES SEE COMMAND MANUAL
		//check_SMS_arrival(&listMSG, buffer, sizeof(buffer));
		//printf("buffer: %s\n", buffer);
		//check_SMS_arrival(&retrieveMSG1, buffer, sizeof(buffer));
		//printf("buffer: %s\n", buffer);
		
		check_SMS_arrival(&retrieveMSG1, buffer, sizeof(buffer));	
		printf("buffer: %s\n", buffer);	
		/*check_SMS_arrival(&retrieveMSG2, buffer, sizeof(buffer));
		check_SMS_arrival(&retrieveMSG3, buffer, sizeof(buffer));
		check_SMS_arrival(&retrieveMSG4, buffer, sizeof(buffer));
		check_SMS_arrival(&retrieveMSG5, buffer, sizeof(buffer));
		check_SMS_arrival(&retrieveMSG6, buffer, sizeof(buffer));
		check_SMS_arrival(&retrieveMSG7, buffer, sizeof(buffer));*/
		
		printf("Checking for messages\n");
		//printf("buffer: %s\n", buffer);
	 }
}


extern "C" void app_main(void)
{	
	TaskHandle_t OnceHandle1 = NULL;
	xTaskCreate(Once_Task, "Once_Task", 4096, NULL, 28, &OnceHandle1);
	vTaskDelay(20000/portTICK_PERIOD_MS); // wait for initialisation to complete
		
	//xTaskCreatePinnedToCore(ACCEL_Task, "ACCEL_Task", 4096, NULL, 10, &ACCELHandle, 0);
	//xTaskCreatePinnedToCore(GPS_Task, "GPS_Task", 4096, NULL, 10, &GPSHandle, 0);
	//xTaskCreatePinnedToCore(GSM_Task, "Gsm_Task", 4096, NULL, 15, &GSMHandle, 1);
	xTaskCreatePinnedToCore(GSM_Monitor_Task, "Gsm_Task", 4096, NULL, 15, &GSMHandle, 1);
	//xTaskCreatePinnedToCore(SD_TaskGPS, "SD_TaskGPS", 4096, NULL, 10, &SDGPSHandle, 1);
	//xTaskCreatePinnedToCore(SD_TaskAccel, "SD_TaskAccel", 4096, NULL, 10, &SDAccelHandle, 1);
}