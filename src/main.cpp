// this is a simple clone of the Chrome Dino game for a 64x32 LED matrix display

#include <Arduino.h>
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>

#include "secret.h"

#if defined(WIFI_SSID) && defined(WIFI_PASS)
#define OTA
#endif
#undef OTA

#ifdef OTA
#include <WiFi.h>
#include <ArduinoOTA.h>
#endif

// Pins
#define BUTTON_PIN 32
#define LED_PIN 33

// globals
uint64_t lastUpdate = 0;
uint64_t lastRedraw = 0;

uint8_t plantVelocity = 3;

uint16_t score = 0;

bool previousButtonState = HIGH;
bool currentButtonState = HIGH;

#define UPDATE_FPS 15
#define REDRAW_FPS 60

#define UPDATE_MILLIS (1000 / UPDATE_FPS)
#define REDRAW_MILLIS (1000 / REDRAW_FPS)

// display
MatrixPanel_I2S_DMA *dma_display = nullptr;

class Sprite
{
public:
    Sprite(uint8_t x, uint8_t y, const uint8_t width, const uint8_t height, const uint8_t *data)
        : m_x(x), m_y(y), m_width(width), m_height(height), m_data(data) {}

    void draw(MatrixPanel_I2S_DMA *display, bool force = false)
    {
        if (!m_needsToRedraw && !force)
        {
            return;
        }

        for (uint8_t i = 0; i < m_width; i++)
        {
            for (uint8_t j = 0; j < m_height; j++)
            {
                uint8_t pixel = pgm_read_byte(m_data + i + j * m_width);
                if (pixel)
                {
                    display->drawPixelRGB888(m_x + i, m_y + j, pixel, pixel, pixel);
                }
            }
        }

        if (m_showBoundingBox)
        {
            display->drawRect(m_x, m_y, m_width, m_height, MatrixPanel_I2S_DMA::color565(255, 0, 0));
        }
    }

    void erase(MatrixPanel_I2S_DMA *display)
    {
        display->fillRect(m_x, m_y, m_width, m_height, MatrixPanel_I2S_DMA::color565(0, 0, 0));
    }

    void putAndDraw(MatrixPanel_I2S_DMA *display, int8_t newX, int8_t newY, bool disableCollision = false)
    {
        erase(display);

        if (!disableCollision)
        {
            // check if new position is within bounds
            if (newX < 0)
            {
                newX = 0;
            }
            else if (newX + m_width > display->width())
            {
                newX = display->width() - m_width;
            }

            if (newY < 0)
            {
                newY = 0;
            }
            else if (newY + m_height > display->height())
            {
                newY = display->height() - m_height;
            }
        }

        m_x = newX;
        m_y = newY;

        draw(display);
    }

    void moveAndDraw(MatrixPanel_I2S_DMA *display, int8_t dx, int8_t dy)
    {
        putAndDraw(display, m_x + dx, m_y + dy);
    }

    void setData(const uint8_t *data)
    {
        m_data = data;
    }

    uint8_t x() const { return m_x; }
    uint8_t y() const { return m_y; }
    uint8_t width() const { return m_width; }
    uint8_t height() const { return m_height; }

    void showBoundingBox(bool show)
    {
        m_showBoundingBox = show;
    }

protected:
    uint8_t m_x;
    uint8_t m_y;
    const uint8_t m_width;
    const uint8_t m_height;
    const uint8_t* m_data;

    bool m_needsToRedraw = true;

    bool m_showBoundingBox = false;
};

// define dinosaur sprite 22x22
const uint8_t dinosaur_data[] PROGMEM = {
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 255, 255, 255, 255, 255, 255, 255,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 255, 255, 255, 255, 255, 255, 255, 255, 255,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 255, 255,   0, 255, 255, 255, 255, 255, 255, 255,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 255, 255,   0, 255, 255, 255, 255, 255, 255, 255,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 255, 255, 255, 255, 255,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 255, 255, 255, 255, 255, 255, 255, 255,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 255, 255, 255, 255,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 255, 255, 255, 255, 255, 255,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 255, 255, 255, 255,   0,   0,   0,   0,   0,   0,   0,
  255,   0,   0,   0,   0,   0,   0,   0,   0,   0, 255, 255, 255, 255, 255,   0,   0,   0,   0,   0,   0,   0,
  255,   0,   0,   0,   0,   0, 255, 255, 255, 255, 255, 255, 255, 255, 255,   0,   0,   0,   0,   0,   0,   0,
  255, 255,   0,   0,   0, 255, 255, 255, 255, 255, 255, 255, 255, 255,   0,   0,   0,   0,   0,   0,   0,   0,
  255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0, 255, 255, 255, 255, 255, 255, 255, 255, 255,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0, 255, 255, 255,   0,   0, 255, 255,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0, 255, 255,   0,   0,   0, 255,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0, 255,   0,   0,   0,   0, 255,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0, 255,   0,   0,   0,   0, 255,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0, 255,   0,   0,   0,   0, 255,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0, 255, 255,   0,   0,   0, 255, 255,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
};

// define 4x4 cactus sprite
const uint8_t cactus_data[] PROGMEM = {
    0, 255, 255,   0,
    0, 255, 255, 255,
  255, 255, 255,   0,
    0, 255, 255,   0,
};

// define 2x2 plant sprite
const uint8_t plant_data[] PROGMEM = {
    0,   0,   0,   0,
    0,   0,   0,   0,
    0,  40, 255,   0,
    0, 255, 255,   0,
};

// define 2x4 plant sprite
const uint8_t plant2_data[] PROGMEM = {
    0, 100, 255,   0,
    0, 255, 255,   0,
    0, 100, 255,   0,
    0, 255, 255,   0,
};
    
class Dinosaur : public Sprite
{
    using Sprite::Sprite;
public:
    bool isOnGround(MatrixPanel_I2S_DMA *display)
    {
        // bottom coord is m_y + m_height - 1
        return m_y + m_height - 1 == display->height() - 1;
    }

    void update(MatrixPanel_I2S_DMA *display)
    {
        if (isOnGround(display) && !isJumping && abs(velocity) < 1)
        {
            return;
        }

        erase(display);

        // Serial.printf("isOnGround: %d, velocity: %d\n", isOnGround(display), velocity);

        if (isOnGround(display) && abs(velocity) <= 0)
        {
            // cancel all velocity
            velocity = 0;
            isJumping = false;
        } else {
            // apply gravity
            if (abs(velocity) < maxVelocity)
            {
                m_y -= velocity; // display is inverte
                velocity -= gravity;
            }
        }

        moveAndDraw(display, 0, 0);
    }

    void jump(MatrixPanel_I2S_DMA *display)
    {
        if (isOnGround(display))
        {
            velocity = jumpStrength;
        }
    }

protected:
    const uint8_t maxVelocity = 10;
    int8_t velocity = 0;
    const int8_t gravity = 1;
    const int8_t jumpStrength = 4;
    bool isJumping = false;
};

Dinosaur dinosaur(10, 0, 22, 22, dinosaur_data);

Sprite plant(64, 0, 4, 4, plant_data);

std::array<Sprite*, 4> sprites = {&dinosaur, &plant};

void setup()
{
    Serial.begin(9600);

#ifdef OTA
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.waitForConnectResult() != WL_CONNECTED)
    {
        Serial.println("Connection Failed! Rebooting...");
        delay(5000);
        ESP.restart();
    }

    ArduinoOTA.setHostname("esp32-dino");
    ArduinoOTA.setPassword("lonelybinary");
    ArduinoOTA.begin();
#endif

    HUB75_I2S_CFG mxconfig(/* width = */ 64, /* height = */ 32, /* chain = */ 1);
    dma_display = new MatrixPanel_I2S_DMA(mxconfig);

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);

    dma_display->begin();
    dma_display->setBrightness8(255);
    dma_display->clearScreen();

    // move all plant sprites to the bottom of the screen
    plant.putAndDraw(dma_display, 44, 32 - plant.height());
}

void updateDisplay()
{
    dinosaur.update(dma_display);

    // player should die if the plant is at the middle of the sprites bottom
    const auto xMiddleOfDino = dinosaur.x() + dinosaur.width() / 2;
    
    /*if (dinosaur.isOnGround(dma_display) && 
        plant.x() <= xMiddleOfDino && 
        xMiddleOfDino <= plant.x() + plant.width()
    )*/
   // improve above to allow for a range around the middle of the dinosaur
    constexpr uint8_t range = 4;
    if (dinosaur.isOnGround(dma_display) && 
        plant.x() <= xMiddleOfDino + range && 
        xMiddleOfDino - range <= plant.x() + plant.width()
    )
    {
        Serial.println("Game Over");
        dma_display->setCursor(0, 0);
        dma_display->clearScreen();
        dma_display->fillScreen(MatrixPanel_I2S_DMA::color565(60, 0, 0));
        dma_display->printf("Game Over: %d", score);
        while (digitalRead(BUTTON_PIN) == LOW) { delay(100); }
        while (digitalRead(BUTTON_PIN) == HIGH)
        {
            #ifdef OTA
            ArduinoOTA.handle();
            #endif
            delay(100);
        }
        ESP.restart();
    }

    if (millis() < 1000)
    {
        return;
    }

    // move the plant to the left. if x < 0, reset x to 64
    plant.moveAndDraw(dma_display, -plantVelocity, 0);
    if (plant.x() <= 0)
    {
        score++;
        uint8_t rndm = random(0, 3);
        switch (rndm)
        {
        case 0:
            plant.setData(plant_data);
            break;
        case 1:
            plant.setData(plant2_data);
            break;
        case 2:
            plant.setData(cactus_data);
            break;
        }
        plantVelocity = random(2, 4);
        plant.putAndDraw(dma_display, dma_display->width(), 32 - plant.height());
    }
}

void redrawDisplay()
{
    // dma_display->drawPixelRGB888(0, 0, 255, 0, 0);
    // dma_display->drawPixelRGB888(63, 0, 0, 255, 0);
    // dma_display->drawPixelRGB888(0, 31, 0, 0, 255);

    dma_display->fillRect(dma_display->width() - 12, 0, 12, 8, MatrixPanel_I2S_DMA::color565(0, 0, 0));
    dma_display->setCursor(dma_display->width() - 12, 0);
    dma_display->print(score);
}

void loop()
{
#ifdef OTA
    ArduinoOTA.handle();
#endif

    uint64_t now = millis();

    currentButtonState = digitalRead(BUTTON_PIN);

    if (currentButtonState == LOW && previousButtonState == HIGH)
    {
        dinosaur.jump(dma_display);
    }

    digitalWrite(LED_PIN, dinosaur.isOnGround(dma_display) ? HIGH : LOW);

    previousButtonState = currentButtonState;

    if (now - lastUpdate >= UPDATE_MILLIS)
    {
        lastUpdate = now;
        updateDisplay();
    }

    if (now - lastRedraw >= REDRAW_MILLIS)
    {
        lastRedraw = now;
        redrawDisplay();
    }
}