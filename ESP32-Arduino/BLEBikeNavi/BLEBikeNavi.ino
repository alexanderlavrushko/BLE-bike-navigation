#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <Adafruit_GFX.h> // available in Arduino libraries, or https://github.com/adafruit/Adafruit-GFX-Library
#include <SPI.h>
#include <Button2.h> // version 2.0.3, available in Arduino libraries, or https://github.com/LennartHennigs/Button2
#include "VoltageMeasurement.h"

// -----------------
// Display selection
// Uncomment wanted display with corresponding header
// -----------------

// OLED 128x128 RGB, Waveshare 14747, driver SSD1351
// Requires library Adafruit SSD1351, available in Arduino libraries, or https://github.com/adafruit/Adafruit-SSD1351-library
// Pins: DIN=23, CLK=18, CS=5, DC=17, RST=16, uses SPIClass(VSPI)
//#include "OLED_SSD1351_Adafruit.h"
//OLED_SSD1351_Adafruit selectedDisplay;
//constexpr bool ENABLE_VOLTAGE_MEASUREMENT = false;

// TTGO T-Display TFT 135x240
// Requires library TFT_eSPI from here: https://github.com/Xinyuan-LilyGO/TTGO-T-Display
// (copy TFT_eSPI to Arduino/libraries)
#include "TFT_TTGO.h"
TFT_TTGO selectedDisplay;
constexpr bool ENABLE_VOLTAGE_MEASUREMENT = true;

// --------
// Constants
// --------
#define SERVICE_UUID               "1E6387F0-BE8C-40DA-8F76-8ED84C42065D"
#define CHAR_READ_STATE_UUID       "1E6387F1-BE8C-40DA-8F76-8ED84C42065D"
#define CHAR_WRITE_DATA_UUID       "1E6387F2-BE8C-40DA-8F76-8ED84C42065D"
#define CHAR_INDICATE_REQUEST_UUID "1E6387F3-BE8C-40DA-8F76-8ED84C42065D"

#define COLOR_BLACK    0x0000
#define COLOR_BLUE     0x001F
#define COLOR_RED      0xF800
#define COLOR_GREEN    0x07E0
#define COLOR_CYAN     0x07FF
#define COLOR_MAGENTA  0xF81F
#define COLOR_YELLOW   0xFFE0
#define COLOR_WHITE    0xFFFF
#define COLOR_GRAY     0x2084

// ---------------------
// Variables for display
// ---------------------
IDisplay& g_display = selectedDisplay;
const int16_t CANVAS_WIDTH = g_display.GetWidth();
const int16_t CANVAS_HEIGHT = g_display.GetHeight();
static GFXcanvas16* g_pGfx = NULL;

// --------
// Variables for BLE
// --------
static BLEServer* g_pServer = nullptr;
static BLECharacteristic* g_pCharRead = nullptr;
static BLECharacteristic* g_pCharWrite = nullptr;
static BLECharacteristic* g_pCharIndicate = nullptr;

static bool g_centralConnected = false;
static bool g_needToHandleConnectionState = true;
static uint32_t g_lastActivityTime = 0;
static bool g_isWriteDataUpdated = false;
static std::string g_writeData;
static std::string g_timeText;

// --------
// Buttons
// --------
#define TTGO_LEFT_BUTTON 0
#define GPIO_NUM_TTGO_LEFT_BUTTON GPIO_NUM_0

#define TTGO_RIGHT_BUTTON 35
//#define GPIO_NUM_TTGO_RIGHT_BUTTON GPIO_NUM_35

#define BUTTON_DEEP_SLEEP TTGO_LEFT_BUTTON
#define GPIO_NUM_WAKEUP GPIO_NUM_TTGO_LEFT_BUTTON

Button2 g_btnDeepSleep(BUTTON_DEEP_SLEEP);
bool g_isDeepSleepRequested = false;

// --------
// Voltage measurement
// --------
#define VOLTAGE_ADC_ENABLE          14
#define VOLTAGE_ADC_PIN             34
static VoltageMeasurement g_voltage(VOLTAGE_ADC_PIN, VOLTAGE_ADC_ENABLE);
static bool g_showVoltage = false;
static bool g_showTime = false;
Button2 g_btn1(TTGO_RIGHT_BUTTON);

// --------
// Types
// --------
enum class ECommand
{
    NewFrameWithColor = 1,
    ShowCurrentFrame = 2,
    DrawLine = 3,
    DrawCircle = 4,
    FillCircle = 5,
    FillTriangle = 6,
    TimeText = 7
};

// --------
// Bluetooth event callbacks
// --------
class MyServerCallbacks: public BLEServerCallbacks
{
    void onConnect(BLEServer* pServer) override
    {
        Serial.println("onConnect");
        g_lastActivityTime = millis();
        g_centralConnected = true;
        g_needToHandleConnectionState = true;
    }

    void onDisconnect(BLEServer* pServer) override
    {
        Serial.println("onDisconnect, will start advertising");
        g_centralConnected = false;
        g_needToHandleConnectionState = true;
        BLEDevice::startAdvertising();
    }
};

class MyCharWriteDataCallbacks: public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
//        Serial.println("onWrite");
        g_lastActivityTime = millis();
        g_writeData = pCharacteristic->getValue();
        g_isWriteDataUpdated = true;
    }
};

class MyCharIndicateStatusCallbacks: public BLECharacteristicCallbacks
{
    void onStatus(BLECharacteristic* pCharacteristic, Status status, uint32_t code) override
    {
        std::string event("onStatus:");
        switch (status)
        {
        case SUCCESS_INDICATE: event += "SUCCESS_INDICATE"; break;
        case SUCCESS_NOTIFY: event += "SUCCESS_NOTIFY"; break;
        case ERROR_INDICATE_DISABLED: event += "ERROR_INDICATE_DISABLED"; break;
        case ERROR_NOTIFY_DISABLED: event += "ERROR_NOTIFY_DISABLED"; break;
        case ERROR_GATT: event += "ERROR_GATT"; break;
        case ERROR_NO_CLIENT: event += "ERROR_NO_CLIENT"; break;
        case ERROR_INDICATE_TIMEOUT: event += "ERROR_INDICATE_TIMEOUT"; break;
        case ERROR_INDICATE_FAILURE: event += "ERROR_INDICATE_FAILURE"; break;
        }
        event += ":";
        event += String(code).c_str();
        PrintEvent(event.c_str(), nullptr);
    }

    void PrintEvent(const char* event, const char* value)
    {
        Serial.print(event);
        Serial.print("(CharIndicateStatus)");
        if (value)
        {
            Serial.print(" value='");
            Serial.print(value);
            Serial.print("'");
        }
        Serial.println();
    }
};

// --------
// Application lifecycle: setup & loop
// --------
void setup()
{
    Serial.begin(115200);
    Serial.println("BLEBikeNavi setup() started");

    // init graphics
    g_pGfx = new GFXcanvas16(CANVAS_WIDTH, CANVAS_HEIGHT);
    g_display.Init();

    // init BLE
    BLEDevice::init("Bike");
    g_pServer = BLEDevice::createServer();
    g_pServer->setCallbacks(new MyServerCallbacks());
    BLEService* pService = g_pServer->createService(SERVICE_UUID);

    // characteristic for read
    {
        uint32_t propertyFlags = BLECharacteristic::PROPERTY_READ;
        BLECharacteristic* pCharRead = pService->createCharacteristic(CHAR_READ_STATE_UUID, propertyFlags);
        pCharRead->setValue(BuildStateData());
        g_pCharRead = pCharRead;
    }

    // characteristic for write
    {
        uint32_t propertyFlags = BLECharacteristic::PROPERTY_WRITE;
        BLECharacteristic* pCharWrite = pService->createCharacteristic(CHAR_WRITE_DATA_UUID, propertyFlags);
        pCharWrite->setCallbacks(new MyCharWriteDataCallbacks);
        pCharWrite->setValue("");
        g_pCharWrite = pCharWrite;
    }

    // characteristic for indicate
    {
        uint32_t propertyFlags = BLECharacteristic::PROPERTY_INDICATE;
        BLECharacteristic* pCharIndicate = pService->createCharacteristic(CHAR_INDICATE_REQUEST_UUID, propertyFlags);
        pCharIndicate->setCallbacks(new MyCharIndicateStatusCallbacks);
        pCharIndicate->addDescriptor(new BLE2902());
        pCharIndicate->setValue("");
        g_pCharIndicate = pCharIndicate;
    }

    pService->start();
    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    // this fixes iPhone connection issue (don't know how it works)
    {
        pAdvertising->setMinPreferred(0x06);
        pAdvertising->setMinPreferred(0x12);
    }
    BLEDevice::startAdvertising();

    // setup deep sleep button
    g_btnDeepSleep.setLongClickTime(500);
    g_btnDeepSleep.setLongClickDetectedHandler([](Button2& b) {
        g_isDeepSleepRequested = true;
    });
    g_btnDeepSleep.setLongClickHandler([](Button2& b) {
        g_display.EnterSleepMode();

        // without this the module won't wake up with button if powered from battery,
        // especially if entered deep sleep when powered from USB
        g_voltage.end();

        esp_sleep_enable_ext0_wakeup(GPIO_NUM_WAKEUP, 0);
        delay(200);
        esp_deep_sleep_start();
    });

    // setup voltage measurement
    if (ENABLE_VOLTAGE_MEASUREMENT)
    {
        g_voltage.begin();
        g_btn1.setReleasedHandler([](Button2& b) {
            if (!g_showTime && !g_showVoltage)
            {
                g_showTime = true;
            }
            else if (g_showTime)
            {
                g_showTime = false;
                g_showVoltage = true;
            }
            else
            {
                g_showTime = false;
                g_showVoltage = false;
            }
        });
    }

    Serial.println("BLEBikeNavi setup() finished");
    Serial.println("Advertising...");
}

void loop()
{
    // handle deep sleep button
    g_btnDeepSleep.loop();
    if (g_isDeepSleepRequested)
    {
        g_pGfx->fillRect(0, 0, CANVAS_WIDTH, 20, COLOR_BLACK);
        g_pGfx->setCursor(0, 0);
        g_pGfx->setTextColor(COLOR_WHITE);
        g_pGfx->setTextSize(2);
        g_pGfx->println("SLEEP");
        g_display.SendImage(0, 0, CANVAS_WIDTH, CANVAS_HEIGHT, g_pGfx->getBuffer());
    }

    // handle voltage button
    g_btn1.loop();
    if (g_showVoltage)
    {
        static uint64_t voltageTimeStamp = 0;
        static float voltage = 0;
        if (millis() - voltageTimeStamp > 500)
        {
            voltageTimeStamp = millis();
            voltage = g_voltage.measureVolts();
        }
        String voltageStr = String(voltage) + " V";
        const int16_t textHeight = 20;
        const int16_t yOffset = CANVAS_HEIGHT - textHeight;
        g_pGfx->fillRect(0, yOffset, CANVAS_WIDTH, textHeight, COLOR_BLACK);
        g_pGfx->setCursor(0, yOffset);
        g_pGfx->setTextColor(COLOR_WHITE);
        g_pGfx->setTextSize(2);
        g_pGfx->println(voltageStr);
        g_display.SendImage(0, 0, CANVAS_WIDTH, CANVAS_HEIGHT, g_pGfx->getBuffer());
    }

    // handle commands received over BLE
    if (g_centralConnected)
    {
        if (g_needToHandleConnectionState)
        {
            g_needToHandleConnectionState = false;

            g_pGfx->fillRect(0, 0, CANVAS_WIDTH, 10, COLOR_BLACK);
            g_pGfx->setCursor(0, 0);
            g_pGfx->setTextColor(COLOR_GREEN);
            g_pGfx->setTextSize(1);
            g_pGfx->println("Connected");
            g_display.SendImage(0, 0, CANVAS_WIDTH, CANVAS_HEIGHT, g_pGfx->getBuffer());
        }

        if (g_isWriteDataUpdated)
        {
            std::string currentData = g_writeData;
            g_isWriteDataUpdated = false;

            if (currentData.size() > 0)
            {
                const uint8_t* bytesStart = reinterpret_cast<const uint8_t*>(currentData.c_str());
                const size_t dataSize = currentData.size();
                size_t index = 0;
                while (index < dataSize)
                {
                    const uint8_t* bytes = bytesStart + index;
                    const size_t remainingSize = dataSize - index;

                    ECommand cmd = static_cast<ECommand>(bytes[0]);
                    if (cmd == ECommand::NewFrameWithColor)
                    {
                        const int OffsetColor = 1;
                        const int RequiredSize = 3;
                        if (remainingSize >= RequiredSize)
                        {
                            index += RequiredSize;
                            const uint16_t color = *(reinterpret_cast<const uint16_t*>(bytes + OffsetColor));
                            g_pGfx->fillScreen(color);
                        }
                        else
                        {
                            PrintError("FillCanvas insufficient bytes", currentData, index);
                        }
                    }
                    else if (cmd == ECommand::ShowCurrentFrame)
                    {
                        const int RequiredSize = 1;
                        index += RequiredSize;
                        g_display.SendImage(0, 0, CANVAS_WIDTH, CANVAS_HEIGHT, g_pGfx->getBuffer());
                    }
                    else if (cmd == ECommand::DrawLine)
                    {
                        const int OffsetXStart = 1;
                        const int OffsetYStart = 3;
                        const int OffsetXEnd = 5;
                        const int OffsetYEnd = 7;
                        const int OffsetColor = 9;
                        const int OffsetWidth = 11;
                        const int RequiredSize = 12;
                        if (remainingSize >= RequiredSize)
                        {
                            index += RequiredSize;
                            const int16_t xStart = *(reinterpret_cast<const int16_t*>(bytes + OffsetXStart));
                            const int16_t yStart = *(reinterpret_cast<const int16_t*>(bytes + OffsetYStart));
                            const int16_t xEnd = *(reinterpret_cast<const int16_t*>(bytes + OffsetXEnd));
                            const int16_t yEnd = *(reinterpret_cast<const int16_t*>(bytes + OffsetYEnd));
                            const uint16_t color = *(reinterpret_cast<const uint16_t*>(bytes + OffsetColor));
                            const uint16_t lineWidth = bytes[OffsetWidth];
                            g_pGfx->drawLine(xStart, yStart, xEnd, yEnd, color);
                            if (lineWidth > 1)
                            {
                                int16_t stepX = 1;
                                int16_t stepY = 0;
                                // if line is more like horizontal than vertical, iterate by vertical axis
                                if (abs(xEnd - xStart) > abs(yEnd - yStart))
                                {
                                    stepX = 0;
                                    stepY = 1;
                                }
                                // width 2:         0, 1
                                // width 3:     -1, 0, 1
                                // width 4:     -1, 0, 1, 2
                                // width 5: -2, -1, 0, 1, 2
                                // width 6: -2, -1, 0, 1, 2, 3
                                int16_t deltaStart = -lineWidth / 2 + (1 - lineWidth % 2);
                                int16_t deltaEnd = lineWidth / 2;
                                for (int16_t i = deltaStart; i <= deltaEnd; ++i)
                                {
                                    g_pGfx->drawLine(xStart + i * stepX,
                                                     yStart + i * stepY,
                                                     xEnd + i * stepX,
                                                     yEnd + i * stepY,
                                                     color);
                                }
                            }
                        }
                        else
                        {
                            PrintError("DrawLine insufficient bytes", currentData, index);
                            break;
                        }
                    }
                    else if (cmd == ECommand::DrawCircle)
                    {
                        const int OffsetX = 1;
                        const int OffsetY = 3;
                        const int OffsetRadius = 5;
                        const int OffsetColor = 6;
                        const int RequiredSize = 8;
                        if (remainingSize >= RequiredSize)
                        {
                            index += RequiredSize;
                            const int16_t x = *(reinterpret_cast<const int16_t*>(bytes + OffsetX));
                            const int16_t y = *(reinterpret_cast<const int16_t*>(bytes + OffsetY));
                            const int8_t radius = bytes[OffsetRadius];
                            const uint16_t color = *(reinterpret_cast<const uint16_t*>(bytes + OffsetColor));
                            g_pGfx->drawCircle(x, y, radius, color);
                        }
                        else
                        {
                            PrintError("DrawCircle insufficient bytes", currentData, index);
                            break;
                        }
                    }
                    else if (cmd == ECommand::FillCircle)
                    {
                        const int OffsetX = 1;
                        const int OffsetY = 3;
                        const int OffsetRadius = 5;
                        const int OffsetColor = 6;
                        const int RequiredSize = 8;
                        if (remainingSize >= RequiredSize)
                        {
                            index += RequiredSize;
                            const int16_t x = *(reinterpret_cast<const int16_t*>(bytes + OffsetX));
                            const int16_t y = *(reinterpret_cast<const int16_t*>(bytes + OffsetY));
                            const int8_t radius = bytes[OffsetRadius];
                            const uint16_t color = *(reinterpret_cast<const uint16_t*>(bytes + OffsetColor));
                            g_pGfx->fillCircle(x, y, radius, color);
                        }
                        else
                        {
                            PrintError("FillCircle insufficient bytes", currentData, index);
                            break;
                        }
                    }
                    else if (cmd == ECommand::FillTriangle)
                    {
                        const int OffsetX1 = 1;
                        const int OffsetY1 = 3;
                        const int OffsetX2 = 5;
                        const int OffsetY2 = 7;
                        const int OffsetX3 = 9;
                        const int OffsetY3 = 11;
                        const int OffsetColor = 13;
                        const int RequiredSize = 15;
                        if (remainingSize >= RequiredSize)
                        {
                            index += RequiredSize;
                            const int16_t x1 = *(reinterpret_cast<const int16_t*>(bytes + OffsetX1));
                            const int16_t y1 = *(reinterpret_cast<const int16_t*>(bytes + OffsetY1));
                            const int16_t x2 = *(reinterpret_cast<const int16_t*>(bytes + OffsetX2));
                            const int16_t y2 = *(reinterpret_cast<const int16_t*>(bytes + OffsetY2));
                            const int16_t x3 = *(reinterpret_cast<const int16_t*>(bytes + OffsetX3));
                            const int16_t y3 = *(reinterpret_cast<const int16_t*>(bytes + OffsetY3));
                            const uint16_t color = *(reinterpret_cast<const uint16_t*>(bytes + OffsetColor));
                            g_pGfx->fillTriangle(x1, y1, x2, y2, x3, y3, color);
                        }
                        else
                        {
                            PrintError("FillTriangle insufficient bytes", currentData, index);
                            break;
                        }
                    }
                    else if (cmd == ECommand::TimeText)
                    {
                        const int OffsetColor = 1;
                        const int OffsetText = 3;
                        const int MinimumSize = 4;
                        if (remainingSize >= MinimumSize)
                        {
                            const char* text = (const char*)bytes + OffsetText;
                            int textLength = 0;
                            for (; textLength < remainingSize; ++textLength)
                            {
                                if (text[textLength] == 0)
                                    break;
                            }
                            
                            if (textLength < remainingSize)
                            {
                                index += MinimumSize + textLength;
                                g_timeText = std::string(text);
                                if (g_showTime)
                                {
                                    const uint16_t color = *(reinterpret_cast<const uint16_t*>(bytes + OffsetColor));
                                    const int16_t textHeight = 20;
                                    const int16_t yOffset = CANVAS_HEIGHT - textHeight;
//                                    g_pGfx->fillRect(0, yOffset, CANVAS_WIDTH, textHeight, COLOR_BLACK);
                                    g_pGfx->setCursor(0, yOffset);
                                    g_pGfx->setTextColor(color);
                                    g_pGfx->setTextSize(2);
                                    g_pGfx->println(text);
                                }
                            }
                            else
                            {
                                PrintError("TimeText not null-terminated string", currentData, index);
                                break;
                            }
                        }
                        else
                        {
                            PrintError("TimeText insufficient bytes", currentData, index);
                            break;
                        }
                    }
                    else
                    {
                        PrintError("Unrecognized command", currentData, index);
                        break;
                    }
                }
            }
            else
            {
                Serial.println("ERROR: write data size == 0");
            }
        }
        else
        {
            uint32_t time = millis();
            if (time - g_lastActivityTime > 4000)
            {
                g_lastActivityTime = time;
                g_pCharIndicate->indicate();
            }
        }
    }
    else
    {
        if (g_needToHandleConnectionState && millis() > 3000)
        {
            g_needToHandleConnectionState = false;

            g_pGfx->fillRect(0, 0, CANVAS_WIDTH, 10, COLOR_BLACK);
            g_pGfx->setCursor(0, 0);
            g_pGfx->setTextColor(COLOR_YELLOW);
            g_pGfx->setTextSize(1);
            g_pGfx->println("Disconnected");
            g_display.SendImage(0, 0, CANVAS_WIDTH, CANVAS_HEIGHT, g_pGfx->getBuffer());
        }
    }
    delay(10);
}

std::string BuildStateData()
{
    const uint8_t screenWidth{ CANVAS_WIDTH };
    const uint8_t screenHeight{ CANVAS_HEIGHT };

    const uint8_t bytes[] = { screenWidth, screenHeight };

    const std::string strData(reinterpret_cast<const char*> (bytes), sizeof(bytes));
    return strData;
}

void PrintError(const char* context, const std::string& data, size_t index)
{
    Serial.print("ERROR: ");
    Serial.print(context);
    Serial.print("data: ");
    for (int i = 0; i < data.length(); ++i)
    {
        Serial.print(" ");
        if (i == index)
        {
            Serial.print("=>");
        }

        if (data[i] < 0x10)
        {
            Serial.print("0");
        }
        Serial.print(data[i], HEX);
    }
    Serial.print(" index=");
    Serial.println(index);
}
