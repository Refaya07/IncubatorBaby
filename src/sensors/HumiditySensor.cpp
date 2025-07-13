#include "HumiditySensor.h"
#include "HardwareConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define RETRY_GET_DATA 3
#define DHT_READ_DELAY_MS 2000  // Delay between readings
#define DHT_RETRY_DELAY_MS 100  // Delay between retries

static void dhtReadTask(void* param) {
    HumiditySensor* dht = static_cast<HumiditySensor*>(param);
    while (true) {
        dht->readTick();
        vTaskDelay(pdMS_TO_TICKS(DHT_READ_DELAY_MS));
    }
}

void HumiditySensor::init(uint8_t pin, uint8_t type) {
    dev = new DHT(pin, type);
    dev->begin();

    xTaskCreatePinnedToCore(dhtReadTask, "DHT read task", 1024 * 4, this, 5, NULL, 1);
}

float HumiditySensor::read() { return humidity; }

float HumiditySensor::readTemperature() { return temperature; }

void HumiditySensor::readTick() {
    // First attempt
    xSemaphoreTake(critMutex, portMAX_DELAY);
    humidity = dev->readHumidity();
    xSemaphoreGive(critMutex);

    // Retry with delays if reading failed
    for (uint8_t i = 0; (i < RETRY_GET_DATA) && isnan(humidity); i++) {
        vTaskDelay(pdMS_TO_TICKS(DHT_RETRY_DELAY_MS));  // Add delay between retries
        xSemaphoreTake(critMutex, portMAX_DELAY);
        humidity = dev->readHumidity();
        xSemaphoreGive(critMutex);
    }

    // If all retries failed, mark sensor as error
    isSensorError = isnan(humidity);

    // // If we got a valid reading, also read temperature
    // if (!isSensorError) {
    //     temperature = dev->readTemperature();
    // }
}