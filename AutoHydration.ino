#define ENABLE_LOGGER
#include <Constants.h>
#include <MulticastOutput.h>
#include <TimeManager.h>
#include <Button.h>
#include <TimeLib.h>

#include <EEPROM.h>
#include "user_interface.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

//Display Type
//#define WIFIKIT
#define LCD

#if defined ARDUINO_ARCH_ESP8266 || defined ARDUINO_ESP8266_WEMOS_D1MINI
#include <ESP8266WiFi.h>
#endif

#include "_secret.h"

/* defined in _secret.h */
char ssid[] = WIFISSID;
char pass[] = WIFIPASS;


#define PUMP_PIN D7 //6  //GPIO 0 pin 15
#define LED_PIN D4
#define BUTTON_1_PIN D8
#define SELECTOR_PIN A0

#if defined WIFIKIT 
#include <U8x8lib.h>
U8X8_SSD1306_128X32_UNIVISION_HW_I2C display(16);
#endif

#define LITER_PER_MINUTE  2.3

#define AI_ENABLED

Adafruit_BME280 BME; // I2C

int workStartTime_Hours = 13;
int workTime_Min = 15;
int workFrequency_Hours = 24;
unsigned long nextWateringTime = 0;
unsigned long lastWateringTime = 0;

arduino::utils::Timer pumpTimer("[PUMP]");
arduino::utils::Timer idleTimer("[IDLE]");

enum Mode {
    PUMP_ON = 0,   // 00000
    IDLE    = 1,   // 00001
    SETUP   = 2,   // 00010
    SETUP_1 = 4,   // 00100
    SETUP_2 = 8,   // 01000
    LAST    = 256  // 10000   
};

Mode mode = LAST;

void switchMode(Mode);

struct SensorsData
{
    float m_temp = 0;
    float m_hum = 0;
    float m_pres = 0;
    unsigned long  m_time = 0;
};

SensorsData sensorsData[24];

void iregationAI()
{
#ifdef AI_ENABLED
    float tempSlope = 0, humSlope = 0, presSlope = 0, averageTemp = 0;

    //slope = ( n * sum ( x * y ) - sum ( x ) * sum(y) ) / (  n *  sum ( x^2 )  - sum(x) ^ 2  ) 

    short endindex = (1 <= TIME.hour()) ? 23 : TIME.hour() - 1;

    for ( short i = 0; i < endindex; i++ )
    {
        if (0 == sensorsData[i].m_temp)
            return;

        averageTemp += sensorsData[i].m_temp;
    }

    for (short i = 1; i < endindex; i++)
    {
        tempSlope += (float)((sensorsData[i - 1].m_temp - sensorsData[i].m_temp)); //divide by 1 hour step 
        humSlope += (float)((sensorsData[i - 1].m_hum - sensorsData[i].m_hum));
        presSlope += (float)((sensorsData[i - 1].m_pres - sensorsData[i].m_pres));
    }

    averageTemp /= endindex;
    tempSlope /= endindex;
    humSlope /= endindex;
    presSlope /= endindex;

    LOG_MSG("Average temp " << averageTemp << " tempSlope " << tempSlope << " humSlope " << humSlope << " presSlope " << presSlope);

    if (humSlope <= 0 && tempSlope > 0 && averageTemp > 25 && (nextWateringTime - lastWateringTime) > arduino::utils::Timer::DAY)
    {
        LOG_MSG("Force hydration required");
        unsigned long prevNextWateringTime = nextWateringTime;

        startPump( TIME.getEpochTime() );

        //update back the scheduled next watering time
        nextWateringTime = prevNextWateringTime;
    }
    else
    {
        LOG_MSG("No force hydration required");
    }
#endif
}

#if defined LCD
#include <LiquidCrystal_I2C.h>
#define SCREEN_WIDTH 16
#define SCREEN_HEIGHT 2

class   LiquidCrystal_Display : public  LiquidCrystal_I2C
{

 public :
    LiquidCrystal_Display(uint8_t lcd_addr, uint8_t lcd_cols, uint8_t lcd_rows) 
            : LiquidCrystal_I2C(lcd_addr,lcd_cols,lcd_rows)
    {
        m_lastScreenID = -1;
    }

    void begin()
    {
        LiquidCrystal_I2C::begin();

        uint8_t chars[][8] = { { 0b00111,  0b01111,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111 },
         { 0b11111,  0b11111,  0b11111,  0b00000,  0b00000,  0b00000,  0b00000,  0b00000 },
         { 0b11100,  0b11110,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111 },
         { 0b11111,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111,  0b01111,  0b00111 },
         { 0b00000,  0b00000,  0b00000,  0b00000,  0b00000,  0b11111,  0b11111,  0b11111 },
         { 0b11111,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111,  0b11110,  0b11100 },
         { 0b11111,  0b11111,  0b11111,  0b00000,  0b00000,  0b00000,  0b11111,  0b11111 },
         { 0b11111,  0b00000,  0b00000,  0b00000,  0b00000,  0b11111,  0b11111,  0b11111 } };
        
        for ( byte i = 0 ; i < sizeof(chars) / sizeof (chars[0] ) ; i++ )
        {
            createChar(i, chars[i]);
        }
        
    }

    void clear( unsigned char i_col = 0, unsigned char i_row = 0 )
    {       
        for ( unsigned char i = 0 ; i < SCREEN_HEIGHT ; i++ )
        {
            setCursor( 0, i );
            printf("%*c", SCREEN_WIDTH, ' ');
        }

        setCursor( i_col, i_row );
    }

    void setupScreen ( int i_value )
    {
        backlight();
        if (m_lastScreenID != 0)
        {
            m_lastScreenID = 0;
            clear(0, 0);
            printf("%s", "Water begin at:");
        }
        setCursor( 0 , 1);
        printf("%02d:00", i_value);
    }

    void setupScreen1 ( int i_value )
    {
        backlight();
        if ( m_lastScreenID != 1 )
        {
            m_lastScreenID = 1;
            clear(0, 0);
            print("Water for:");
            setCursor(0, 1);
            printf("%02d min", i_value);
        }
        
        setCursor(0, 1);
        printf("%02d", i_value);
    }

    void setupScreen2 (int i_value )
    {
        backlight();
        if ( m_lastScreenID != 2  )
        {
            m_lastScreenID = 2;
            clear(0, 0);
            print("Water every:");
        }

        setCursor(0, 1);
        printf("%03d hours", i_value);
    }

    void idleScreen ()
    {
        clear(0, 0);
        
        switch ( m_idleCounter++ % 5 )
        {
        case 0:
            printf("Next Water Time");
            setCursor(0, 1);
            printf("%s %d m",TIME.getShortTimeDateStr( nextWateringTime), workTime_Min );
            return;
        case 1:
          //  printf("%s", "Current time");
           // setCursor(0, 1);
           // print(TIME.getTimeStr());
            drawClock();
            return;
        case 2:
            if ( BME.readPressure() > 0 )
            {
                printf("   %s", "Weather");
                setCursor(0, 1);
                printf("%dC %d%% %.3fatm", (int)BME.readTemperature(), ( int )BME.readHumidity(), (float)( BME.readPressure() * 0.00000986F /** 0.00750062*/));
            }
            return;
        case 3:
            if (BME.readPressure() > 0)
            {
                printf("   %s", "Weather");
                setCursor(0, 1);
                printf("%dC %d%% %.2fmm", (int)BME.readTemperature(), (int)BME.readHumidity(), (float)(BME.readPressure() * 0.00750062F));
            }
            return;
        case 4:
            printf("Last Water Time");
            setCursor(0, 1);
            printf("%s %d m", TIME.getShortTimeDateStr( lastWateringTime ), workTime_Min );
            return;
        }
    }

    void initScreen()
    {
          setCursor( rand() % SCREEN_WIDTH, rand() % SCREEN_HEIGHT );
          print((char)255);
    }

    void drawDig( short i_dig, short x, short y)
    {
      //  LOG_MSG("Draw " << i_dig)
        switch (i_dig)
        {
        case 0:
            setCursor(x, y); // set cursor to column 0, line 0 (first row)
            write(0);  // call each segment to create
            write(1);  // top half of the number
            write(2);
            setCursor(x, y + 1); // set cursor to colum 0, line 1 (second row)
            write(3);  // call each segment to create
            write(4);  // bottom half of the number
            write(5);
            break;
        case 1:
            setCursor(x + 1, y);
            write(1);
            write(2);
            setCursor(x + 2, y + 1);
            write(5);
            break;
        case 2:
            setCursor(x, y);
            write(6);
            write(6);
            write(2);
            setCursor(x, y + 1);
            write(3);
            write(7);
            write(7);
            break;
        case 3:
            setCursor(x, y);
            write(6);
            write(6);
            write(2);
            setCursor(x, y + 1);
            write(7);
            write(7);
            write(5);
            break;
        case 4:
            setCursor(x, y);
            write(3);
            write(4);
            write(2);
            setCursor(x + 2, y + 1);
            write(5);
            break;
        case 5:
            setCursor(x, y);
            write(0);
            write(6);
            write(6);
            setCursor(x, y + 1);
            write(7);
            write(7);
            write(5);
            break;
        case 6:
            setCursor(x, y);
            write(0);
            write(6);
            write(6);
            setCursor(x, y + 1);
            write(3);
            write(7);
            write(5);
            break;
        case 7:
            setCursor(x, y);
            write(1);
            write(1);
            write(2);
            setCursor(x + 1, y + 1);
            write(0);
            break;
        case 8:
            setCursor(x, y);
            write(0);
            write(6);
            write(2);
            setCursor(x, y + 1);
            write(3);
            write(7);
            write(5);
            break;
        case 9:
            setCursor(x, y);
            write(0);
            write(6);
            write(2);
            setCursor(x + 1, y + 1);
            write(4);
            write(5);
            break;
        case 10:
            //Clean
            setCursor(x, y);
            write(32);
            write(32);
            write(32);
            setCursor(x, y + 1);
            write(32);
            write(32);
            write(32);
            break;
        }
    }

    void drawClock(  )
    {
        short x = 0;
        short y = 0;
        short hours = TIME.hour();
        short minutes = TIME.min();
        clear(0, 0);
        
        //draw hours
        drawDig( hours / 10, x, y);
        drawDig( hours % 10, x + 4, y);
      
        //draw dots
        setCursor( x + 7, y );
        write(165);
        setCursor( x + 7, y + 1 );
        write(165);

        //draw minutes
        drawDig(minutes / 10, x + 8, y);
        drawDig(minutes % 10, x + 12, y);
    }

private:
    unsigned short m_idleCounter;
    int m_lastScreenID;
};

LiquidCrystal_Display display( 0x27, SCREEN_WIDTH, SCREEN_HEIGHT );

/*
void print(short i_row, const char* i_string, short i_size, bool i_enableAnimation)
{
    char tempBuffer[SCREEN_WIDTH + 1];

    if (i_enableAnimation)
    {
        for (int i = 0; i < SCREEN_WIDTH; i++)
        {
            display.setCursor(i, i_row);

            display.print((char)255);
            delay(100);
        }
    }

    display.setCursor(0, i_row);

    if (i_size > SCREEN_WIDTH)
    {
        //memset ( tempBuffer,0x0,SCREEN_WIDTH);
        memcpy(tempBuffer, i_string, SCREEN_WIDTH);
        tempBuffer[SCREEN_WIDTH] = 0x0;
        i_string = tempBuffer;

        display.print(tempBuffer);
        LOG_MSG(tempBuffer);
    }
    else
    {
        display.print(i_string);
        LOG_MSG(i_string);
    }

    for (int i = i_size; i < SCREEN_WIDTH; i++)
    {
        display.setCursor(i, i_row);
        display.print(' ');
    }
}*/
#endif

//The function can get const parameter event though it is not defined as it because 
// std::function defined both constructors canst and non const :) template magic
void stopPump(const long&)
{
    using namespace arduino::utils;
    LOG_MSG("Stop pump "<< (TimeValue)TIME.getEpochTime());

    digitalWrite(PUMP_PIN, LOW);
    digitalWrite(LED_PIN, LED_OFF);
    
    switchMode( IDLE );
}

void startPump(const long&)
{
    using namespace arduino::utils;
    LOG_MSG("Start pump " << (TimeValue)TIME.getEpochTime() << (TimeValue)lastWateringTime) ;

    //prevent frequent work or any illegal execution
    if (0 != lastWateringTime && lastWateringTime > TIME.getEpochTime() - 30 /*sec*/ )
        return;

    digitalWrite( PUMP_PIN, HIGH );
    digitalWrite( LED_PIN, LED_ON );
    
    switchMode( PUMP_ON );

    if ( false == pumpTimer.addTask(TIME.getEpochTime() + workTime_Min * Timer::MINUTE, stopPump) )
    {
        LOG_MSG("Fatal exception " << " not able to schedule stop command , rebooting");
        stopPump( TIME.getEpochTime() );
        
        ESP.restart();
    }
}


void user_dump_rst_info()
{
    struct rst_info *rtc_info = system_get_rst_info();
    LOG_MSG("reset reason: " << (void*)(rtc_info->reason));

    if (rtc_info->reason == REASON_WDT_RST || rtc_info->reason == REASON_EXCEPTION_RST || rtc_info->reason == REASON_SOFT_WDT_RST)
    {
        if (rtc_info->reason == REASON_EXCEPTION_RST)
        {
            LOG_MSG("Fatal exception " << rtc_info->exccause);
        }

        char buffer[128];
        snprintf(buffer, sizeof(buffer), "epc1=0x%08x, epc2=0x%08x, epc3=0x%08x, excvaddr=0x%08x, depc=0x%08x", rtc_info->epc1, rtc_info->epc2, rtc_info->epc3, rtc_info->excvaddr, rtc_info->depc);

        LOG_MSG(buffer);
    }
}

void setup()
{
    using namespace arduino::utils;

    Serial.begin(115200);
    display.begin();
    EEPROM.begin(512);

   // display.setFont(u8x8_font_chroma48medium8_r);
    display.clear(0, 0);
    display.noCursor();
    display.noBlink();
    display.print("  initializing  ");
    display.backlight();

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);
 
    TIME.begin();

    BME.begin(0x76);

    pinMode( PUMP_PIN , OUTPUT );
    pinMode( SELECTOR_PIN, INPUT );

    digitalWrite( PUMP_PIN , LOW );

    idleTimer.addRecuringTask( TIME.getEpochDate(), 10 , [](long&) { if ( mode == IDLE ) { display.idleScreen(); } });
    
    if ( EEPROM.read(0) > 0 )
    {
        workStartTime_Hours = EEPROM.read(0);
        workFrequency_Hours = EEPROM.read(1);
        workTime_Min        = EEPROM.read(2);
    }  

    user_dump_rst_info();
     //else default values remain
 }

void switchMode ( Mode i_mode = IDLE )
{
     LOG_MSG("Switch from " << (short)mode << " to " << (short) i_mode);

     using namespace arduino::utils;
   
     static unsigned long pumpOnTime = 0;

     switch ( i_mode )
     {
        case SETUP:
        {
            pumpTimer.addTask(TIME.getEpochTime() + 5 * Timer::MINUTE, [](long&) { switchMode( LAST ); });//save
        }
        case SETUP_1:
        case SETUP_2:
        { 
          mode = i_mode;
        }
        break;
        case IDLE:
        {
           if ( mode == PUMP_ON )  // PUMP_OFF
            {    
                lastWateringTime = TIME.getEpochTime();
        
                display.clear(0, 0);
                display.printf("%s", "Watering done");
                display.setCursor(0, 1);
                display.printf( "Took %.1f litter", (float) ( ( TIME.getEpochTime() - pumpOnTime ) * LITER_PER_MINUTE / Timer::MINUTE ));
            }
            mode = IDLE;
        }
        break;
        case PUMP_ON:
        {
            // Pump started
            if ( mode != PUMP_ON )
            {
                //Pump ended              
                pumpOnTime = TIME.getEpochTime();

                nextWateringTime = TIME.getEpochTime() + workFrequency_Hours * Timer::HOUR;
                
                display.clear(0,0);
                display.printf("%s", "Watering till");
                display.setCursor(0, 1);
                display.print( TIME.getTimeStr ( TIME.getEpochTime() + workTime_Min * Timer::MINUTE ) );
            }
            mode = PUMP_ON;
        }
        break;
        default: // save
        {
            using namespace arduino::utils;
            pumpTimer = arduino::utils::Timer();

            short delayInHours = ( TIME.getEpochDate() + workStartTime_Hours * Timer::HOUR > TIME.getEpochTime() ) ? 0 : 24;
            long startWateringTime = TIME.getEpochDate() + (workStartTime_Hours + delayInHours) * Timer::HOUR;
            
            pumpTimer.addRecuringTask( startWateringTime , workFrequency_Hours * Timer::HOUR, startPump );

            nextWateringTime = startWateringTime;
			if (0 == lastWateringTime)
				lastWateringTime = TIME.getEpochDate();

            if ( mode & SETUP_2 )
            {
                display.clear(0, 0);
                display.printf("%s", "Save Water Time");
                display.setCursor(0, 1);
                display.printf("%s %d m", TIME.getShortTimeDateStr(nextWateringTime), workTime_Min);

                EEPROM.write(0, (uint8_t)workStartTime_Hours);
                EEPROM.write(1, (uint8_t)workFrequency_Hours);
                EEPROM.write(2, (uint8_t)workTime_Min);
                EEPROM.commit();
            }

            LOG_MSG("Going back to idle mode next execution time is " << TIME.getTimeDateStr ( nextWateringTime ) << " work time is " << workTime_Min);

            mode = IDLE;

            idleTimer.addTask( TIME.getEpochTime() + 30, [](long&) { display.noBacklight(); } );
        }
     }
     
}

Button_t botton( BUTTON_1_PIN, 
    []() { if ( false == display.getBacklight() ) { display.backlight(); idleTimer.addTask(TIME.getEpochTime() + 5 * arduino::utils::Timer::MINUTE, [](long&) { display.noBacklight(); }); }  },
    []() { if ( false == display.getBacklight() ) { display.backlight(); idleTimer.addTask(TIME.getEpochTime() + workTime_Min * arduino::utils::Timer::MINUTE + 5, [](long&) { display.noBacklight(); }); }  ( IDLE & mode ) ? startPump( TIME.getEpochTime() ) : stopPump( TIME.getEpochTime() );  },
    []() { display.backlight(); switchMode( static_cast<Mode>( static_cast<int>( mode ) << 1  ) ); });

void loop()
{
    using namespace arduino::utils;
    //static bool taskSet = false;
    
    if ( !TIME.synced() )
    {
        if ( millis() % 100 )
        {
            LOG_MSG("Time is not synced yet , going to wait");

            display.initScreen();
        }

        TIME.run();
        return;
    }
    else if ( LAST == mode  ) 
    {
        idleTimer.addRecuringTask( TIME.getEpochTime(), 30 * Timer::MINUTE, [](long&) {
            sensorsData[ TIME.hour() ].m_temp = BME.readTemperature();
            sensorsData[ TIME.hour() ].m_hum  = BME.readHumidity();
            sensorsData[ TIME.hour() ].m_pres = BME.readPressure() * 0.00750062F;
            sensorsData[ TIME.hour() ].m_time = TIME.getEpochTime();

            LOG_MSG("METRICS " << "TEMP:" << BME.readTemperature() << " HUM:" << BME.readHumidity() << " PRES:" << (float)( BME.readPressure() * 0.00750062 ));
        });

        short delayInHours = (TIME.getEpochDate() + 19 * Timer::HOUR > TIME.getEpochTime()) ? 0 : 24;
        //Run at 19:00
        idleTimer.addRecuringTask( TIME.getEpochDate() + 19 * Timer::HOUR + delayInHours , Timer::DAY , [](long&) { iregationAI(); });

        //simulate setup complete
        switchMode( static_cast<Mode>(static_cast<int>( SETUP_2 ) << 1) );
    }

    if ( mode == SETUP )
    {
        int selectorValue = map( analogRead( SELECTOR_PIN ), 0, 1023, 0, 23);
        
        if ( selectorValue != workStartTime_Hours )
        {
            LOG_MSG("New workStartTime_Hours " << workStartTime_Hours);
            workStartTime_Hours = selectorValue;
            display.setupScreen( workStartTime_Hours );
        }
    }

    if ( mode == SETUP_1 )
    {
        int selectorValue = map( analogRead(SELECTOR_PIN), 0, 1023, 1, 30 );

        if ( selectorValue != workTime_Min )
        {
            LOG_MSG("New workTime_Min " << workTime_Min);
            workTime_Min = selectorValue;
            display.setupScreen1(workTime_Min);
        }
    }

    if ( mode == SETUP_2 )
    {
        int selectorValue = map( analogRead(SELECTOR_PIN), 0, 1023, 1, 30 ); 

        if (selectorValue != workFrequency_Hours )
        {
            LOG_MSG("New workFrequency_Hours " << workFrequency_Hours);
            
            if ( selectorValue > 24 )
            {
                selectorValue = ( ( 27 > selectorValue ) ? ( 27 - selectorValue ) * Timer::DAY : Timer::WEEK ) / Timer::HOUR ;
            }

            workFrequency_Hours = selectorValue;
            display.setupScreen2( workFrequency_Hours );
        }
    }

    TIME.run();
    pumpTimer.run();
    botton.run();
    idleTimer.run();
}

