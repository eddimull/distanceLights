#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "led_strip.h"

#define TRIG_PIN 18
#define ECHO_PIN 23
#define SOUND_SPEED 0.034

#define MIN_DISTANCE 4
#define MAX_DISTANCE 70
#define LED_GPIO GPIO_NUM_2

#define WINDOW_SIZE 10 // Number of previous measurements to consider for smoothing

static led_strip_handle_t led_strip;
// distances to positions
static const uint8_t slide_positions[] = {5, 11, 21, 29, 39, 50, 60};

// Function to calculate the moving average
float calculateMovingAverage(float *values, int size, float newValue)
{
    static int index = 0;
    static float sum = 0;

    sum -= values[index];
    sum += newValue;
    values[index] = newValue;
    index = (index + 1) % size;

    return sum / size;
}

static void configure_led(void)
{
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = 1, // at least one LED on board
    };

    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
}

// Function to map a value from one range to another
int map(int value, int fromLow, int fromHigh, int toLow, int toHigh)
{
    return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}
// Function to change the color of the LED strip
static void change_color(uint8_t red, uint8_t green, uint8_t blue)
{
    printf("Changing color to: %d %d %d\n", red, green, blue);
    led_strip_set_pixel(led_strip, 0, red, green, blue);
    led_strip_refresh(led_strip);
}

// Function to calculate the RGB values based on distance and change the LED color
void calculateRGB(int distance, uint8_t *r, uint8_t *g, uint8_t *b)
{
    if (distance < slide_positions[0])
    {
        *r = 0;
        *g = 0;
        *b = 0;
    }
    else if (distance < slide_positions[1])
    {
        *r = 255;
        *g = 0;
        *b = 0;
    }
    else if (distance < slide_positions[2])
    {
        *r = 255;
        *g = map(distance, slide_positions[1], slide_positions[2], 0, 165);
        *b = 0;
    }
    else if (distance < slide_positions[3])
    {
        *r = 255;
        *g = 255;
        *b = 0;
    }
    else if (distance < slide_positions[4])
    {
        *r = 0;
        *g = 255;
        *b = 0;
    }
    else if (distance < slide_positions[5])
    {
        *r = 0;
        *g = 0;
        *b = 255;
    }
    else
    {
        *r = 238;
        *g = 130;
        *b = 238;
    }
}

void app_main()
{
    uint8_t r, g, b;
    configure_led();
    gpio_set_direction(TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);
    float distances[WINDOW_SIZE] = {0};
    while (1)
    {
        // clear the terminal output
        printf("\e[1;1H\e[2J");

        // Send a 10us pulse to TRIG_PIN
        gpio_set_level(TRIG_PIN, 0);
        vTaskDelay(2 / portTICK_PERIOD_MS);
        gpio_set_level(TRIG_PIN, 1);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        gpio_set_level(TRIG_PIN, 0);

        // Measure the length of the pulse on ECHO_PIN
        int64_t start_time = esp_timer_get_time();
        while (gpio_get_level(ECHO_PIN) == 0 && (esp_timer_get_time() - start_time) < 1000000)
            ;
        start_time = esp_timer_get_time();
        while (gpio_get_level(ECHO_PIN) == 1 && (esp_timer_get_time() - start_time) < 1000000)
            ;
        int64_t end_time = esp_timer_get_time();

        // Calculate the distance in cm
        float distance = ((end_time - start_time) * SOUND_SPEED) / 2;

        // Apply moving average smoothing
        float smoothedDistance = calculateMovingAverage(distances, WINDOW_SIZE, distance);

        printf("Raw Distance: %.2f cm\n", distance);
        printf("Smoothed Distance: %.2f cm\n", smoothedDistance);

        calculateRGB(smoothedDistance, &r, &g, &b);
        change_color(r, g, b);

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
