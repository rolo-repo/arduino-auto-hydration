#include <ArduinoOTA.h>
#include "Constants.h"

#include "buildinfo.h"
#include <LiquidCrystal.h>
#define ENABLE_LOGGER
#include <Constants.h>
#include <MulticastOutput.h>
#include <TimeManager.h>
#include <Button.h>
#include <TimeLib.h>

#include "PlaceHolder.h"


#include <EEPROM.h>
#include "user_interface.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include <ESP8266WebServer.h>
#include "BufferAndSize.h"

#include "OTA.h"
#include "resource.h"
#include "Pair.h"

#include "EEPROM_Adapter.h"

#include "Led.h"


//Display Type
//#define WIFIKIT
#define LCD

#if defined ARDUINO_ARCH_ESP8266 || defined ARDUINO_ESP8266_WEMOS_D1MINI
#include <ESP8266WiFi.h>
ESP8266WebServer  server(80);
#endif


using namespace arduino::utils;


const char* const HTML_open = "<!DOCTYPE html><html><head></head>";
const char* const HTML_body_open = "<body>";
const char* const HTML_body_close = "</body>";
const char* const HTML_new_line = "<p>";
const char* const HTML_close = "</html>";



#include "_secret.h"

/* defined in _secret.h */
//char ssid[] = WIFISSID;
//char pass[] = WIFIPASS;

String ssid;
String ssid_password;
constexpr char* hostname = "autoHydration_";

#define SCL_PIN D1
#define SDA_PIN D2

#define PUMP_PIN D3 //6  //GPIO 0 pin 15
#define LED_PIN D4
#define BUTTON_1_PIN D7
#define SELECTOR_PIN A0

#define  GREEN_LED D5
#define  BLUE_LED D0
#define  RED_LED D6

Led stsRedLed(LED_PIN);
RGBLed rgbLed(GREEN_LED, BLUE_LED, RED_LED, Led::Brightness::_20, true);


#if defined WIFIKIT 
#include <U8x8lib.h>
U8X8_SSD1306_128X32_UNIVISION_HW_I2C display(16);
#endif

#define LITER_PER_MINUTE  2.3

//#define AI_ENABLED

//Adafruit_BME280 BME; // I2C

int workStartTime_Hours = 13;
int workTime_Min = 15;
int workFrequency_Hours = 24;
unsigned long nextWateringTime = 0;
unsigned long lastWateringTime = 0;

arduino::utils::Timer pumpTimer("[PUMP]");
arduino::utils::Timer idleTimer("[IDLE]");

enum Mode {
	PUMP_ON = 0,   // 00000
	IDLE = 1,   // 00001
	SETUP = 2,   // 00010
	SETUP_1 = 4,   // 00100
	SETUP_2 = 8,   // 01000
	LAST = 256  // 10000   
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

struct {
	bool _on = false;

	bool on() const {
		return _on;
	}

	void stop(const long& = 0)
	{
		using namespace arduino::utils;
		LOG_MSG("Stop pump " << (TimeValue)TIME.getEpochTime());

		digitalWrite(PUMP_PIN, LOW);

		rgbLed.turn_off();

		_on = false;
	}

	void start( const long& = 0 )
	{
		using namespace arduino::utils;
		LOG_MSG("Start pump " << (TimeValue)TIME.getEpochTime() << (TimeValue)lastWateringTime);

		//prevent frequent work or any illegal execution
		if (0 != lastWateringTime && lastWateringTime > TIME.getEpochTime() - 30 /*sec*/)
			return;

		digitalWrite(PUMP_PIN, HIGH);
		rgbLed.turn_on( RGBLed::LedType::Blue );

		if (false == pumpTimer.addTask(TIME.getEpochTime() + 5 * Timer::MINUTE, [=](const long&) { stop(); }))
		{
			LOG_MSG("Fatal exception " << " not able to schedule stop command , rebooting");
			stop(TIME.getEpochTime());

			ESP.restart();
		}

		_on = true;
	}

} pump;


SensorsData sensorsData[24];

void iregationAI()
{
#ifdef AI_ENABLED
	float tempSlope = 0, humSlope = 0, presSlope = 0, averageTemp = 0;

	//slope = ( n * sum ( x * y ) - sum ( x ) * sum(y) ) / (  n *  sum ( x^2 )  - sum(x) ^ 2  ) 

	short endindex = (1 <= TIME.hour()) ? 23 : TIME.hour() - 1;

	for (short i = 0; i < endindex; i++)
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

		start(TIME.getEpochTime());

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

public:
	LiquidCrystal_Display(uint8_t lcd_addr, uint8_t lcd_cols, uint8_t lcd_rows)
		: LiquidCrystal_I2C(lcd_addr, lcd_cols, lcd_rows)
	{
		m_lastScreenID = -1;
	}

	bool begin()
	{
		LiquidCrystal_I2C::begin(SCREEN_WIDTH, SCREEN_HEIGHT);

		uint8_t chars[][8] = { { 0b00111,  0b01111,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111 },
		 { 0b11111,  0b11111,  0b11111,  0b00000,  0b00000,  0b00000,  0b00000,  0b00000 },
		 { 0b11100,  0b11110,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111 },
		 { 0b11111,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111,  0b01111,  0b00111 },
		 { 0b00000,  0b00000,  0b00000,  0b00000,  0b00000,  0b11111,  0b11111,  0b11111 },
		 { 0b11111,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111,  0b11110,  0b11100 },
		 { 0b11111,  0b11111,  0b11111,  0b00000,  0b00000,  0b00000,  0b11111,  0b11111 },
		 { 0b11111,  0b00000,  0b00000,  0b00000,  0b00000,  0b11111,  0b11111,  0b11111 } };

		for (byte i = 0; i < sizeof(chars) / sizeof(chars[0]); i++)
		{
			createChar(i, chars[i]);
		}

		return true;
	}

	void clear(unsigned char i_col = 0, unsigned char i_row = 0)
	{
		for (unsigned char i = 0; i < SCREEN_HEIGHT; i++)
		{
			setCursor(0, i);
			printf("%*c", SCREEN_WIDTH, ' ');
		}

		setCursor(i_col, i_row);
	}

	void setupScreen(int i_value)
	{
		backlight();
		if (m_lastScreenID != 0)
		{
			m_lastScreenID = 0;
			clear(0, 0);
			printf("%s", "Water begin at:");
		}
		setCursor(0, 1);
		printf("%02d:00", i_value);
	}

	void setupScreen1(int i_value)
	{
		backlight();
		if (m_lastScreenID != 1)
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

	void setupScreen2(int i_value)
	{
		backlight();
		if (m_lastScreenID != 2)
		{
			m_lastScreenID = 2;
			clear(0, 0);
			print("Water every:");
		}

		setCursor(0, 1);
		printf("%03d hours", i_value);
	}

	void idleScreen()
	{
		//clear(0, 0);
		//
		//switch ( m_idleCounter++ % 5 )
		//{
		//case 0:
		//    printf("Next Water Time");
		//    setCursor(0, 1);
		//    printf("%s %d m",TIME.getShortTimeDateStr( nextWateringTime), workTime_Min );
		//    return;
		//case 1:
		//  //  printf("%s", "Current time");
		//   // setCursor(0, 1);
		//   // print(TIME.getTimeStr());
		//    drawClock();
		//    return;
		//case 2:
		//    if ( BME.readPressure() > 0 )
		//    {
		//        printf("   %s", "Weather");
		//        setCursor(0, 1);
		//        printf("%dC %d%% %.3fatm", (int)BME.readTemperature(), ( int )BME.readHumidity(), (float)( BME.readPressure() * 0.00000986F /** 0.00750062*/));
		//    }
		//    return;
		//case 3:
		//    if (BME.readPressure() > 0)
		//    {
		//        printf("   %s", "Weather");
		//        setCursor(0, 1);
		//        printf("%dC %d%% %.2fmm", (int)BME.readTemperature(), (int)BME.readHumidity(), (float)(BME.readPressure() * 0.00750062F));
		//    }
		//    return;
		//case 4:
		//    printf("Last Water Time");
		//    setCursor(0, 1);
		//    printf("%s %d m", TIME.getShortTimeDateStr( lastWateringTime ), workTime_Min );
		//    return;
		//}
	}

	void initScreen()
	{
		setCursor(rand() % SCREEN_WIDTH, rand() % SCREEN_HEIGHT);
		print((char)255);
	}

	void drawDig(short i_dig, short x, short y)
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

	void drawClock()
	{
		short x = 0;
		short y = 0;
		short hours = TIME.hour();
		short minutes = TIME.min();
		clear(0, 0);

		//draw hours
		drawDig(hours / 10, x, y);
		drawDig(hours % 10, x + 4, y);

		//draw dots
		setCursor(x + 7, y);
		write(165);
		setCursor(x + 7, y + 1);
		write(165);

		//draw minutes
		drawDig(minutes / 10, x + 8, y);
		drawDig(minutes % 10, x + 12, y);
	}

private:
	unsigned short m_idleCounter;
	int m_lastScreenID;
};

//LiquidCrystal_Display display( 0x27, SCREEN_WIDTH, SCREEN_HEIGHT );

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

struct {
	bool enabled = false;
	LiquidCrystal_Display display = LiquidCrystal_Display(0x27, SCREEN_WIDTH, SCREEN_HEIGHT);
	LiquidCrystal_Display& operator*() { return display; }
	LiquidCrystal_Display& operator->() { return display; }
	LiquidCrystal_Display& operator()(LiquidCrystal_Display) { return display; }
} display;


struct
{
	bool enabled = false;
	Pair<float, float> data_t[24];
	Adafruit_BME280 bme;
	Adafruit_BME280& operator*() { return bme; }
} bme;

auto bmeReadPressure()
{
	LOG_MSG("About to read sensors data - preasure");
	rgbLed.blynk(RGBLed::LedType::Green);

	float pressure = (bme.enabled) ? (*bme).readPressure() : NAN;
	float pressureAt = (float)(pressure / 100) * 0.000986F;
	float pressureHpa = (float)(pressure / 100);

	return make_Pair(pressureAt, pressureHpa);
}

auto bmeReadTemperature()
{
	LOG_MSG("About to read sensors data - temperature");
	rgbLed.blynk(RGBLed::LedType::Green);

	float temperature = (bme.enabled) ? (*bme).readTemperature() : NAN;

	if ( temperature > 25 ) 
	temperature -= 10;

	return   ((temperature != NAN) ? temperature : 0.0F);
}

auto bmeReadHumidity()
{
	LOG_MSG("About to read sensors data - humidity");
	rgbLed.blynk(RGBLed::LedType::Green);

	float humidity = (bme.enabled) ? (*bme).readHumidity() : NAN;

	return   ((humidity != NAN) ? humidity : 0.0F);
}

class LandingPage_RequestHandler : public RequestHandler
{
private:
	struct TimerTask
	{
		uint8 id = 0;
		time_t date_time = 0;
		uint8 code = 0;
		bool recuring = false;
		bool enb = false;

		void schedule()
		{
			if (this->enb)
			{
				if (!recuring)
					pumpTimer.addTask(date_time, [this](long&) {  (0 == this->code) ? pump.start() : pump.stop(); this->enb = false; });
				else
					pumpTimer.addRecuringTask(date_time, arduino::utils::Timer::DAY, [this](long&) { (0 == this->code) ? pump.start() : pump.stop(); this->date_time += arduino::utils::Timer::DAY; });
			}
		}

		String toJson()
		{
			String data;
			if (this->enb) {
				data += '{';
				data += "\"id\":";
				data += id;
				data += ',';
				data += "\"date\":\"";
				data += TIME.getDateStrYYYY_MM_DD(date_time);
				data += "\",";
				data += "\"time\":\"";
				data += TIME.getTimeStr(date_time);
				data += "\",";
				data += "\"code\":";
				data += code;
				data += ',';
				data += "\"recuring\": \"";
				data += (recuring) ? "true" : "false";
				data += "\"}";
			}

			return data;
		}
	} tt[10];
public:
	bool canHandle(HTTPMethod method, String uri) override {
		LOG_MSG("Request : " << uri << " method " << method << " can handle : " << "dash or fput");

		if (method == HTTP_GET && (uri.startsWith("/dash") || uri.startsWith("/fput") || uri.startsWith("/schedule")))
			return true;
		else
			return false;
	}

	bool handle(ESP8266WebServer& server, HTTPMethod requestMethod, String requestUri) override {

		rgbLed.blynk(RGBLed::LedType::Green);

		using namespace arduino::utils;
		LOG_MSG("Request : " << requestUri << " " << requestMethod);

		if (requestUri.startsWith("/dash"))
		{
			PlaceHolder p("t", "");
			PlaceHolder v("version", TIME.getTimeDateStr(BUILD_ID));

			server.setContentLength(CONTENT_LENGTH_UNKNOWN);
			server.send(200, "text/html", " ");

			BufferAndSize_t<char, long> buff(new char[1024], 1024, true);

			for (uint16 i = 0; i < sizeof(dashboard_html); i += 1023) {

				short size = ((sizeof(dashboard_html) - i > 1023) ? 1023 : sizeof(dashboard_html) - i);
				memcpy_P(*buff, dashboard_html + i, size);
				buff[size] = 0x0;//null terminator
				//the mechanism of translate will not work in case the string will be slitted exactly by placeholder
				server.sendContent(PlaceHolder::tr(PlaceHolder::makeList(VA_LIST(&p, &v)), *buff));
			}

			return true;
		}

		if (requestUri.startsWith("/schedule"))
		{
			String jsonResp;
			jsonResp += '[';

			for (uint8 i = 0; i < SIZE_OF_ARR(tt); i++) {
				if (tt[i].enb) {
					jsonResp += tt[i].toJson();
					jsonResp += ',';
				}
			}
			jsonResp.remove(jsonResp.lastIndexOf(','));
			jsonResp += ']';

			server.send(200, "text/html", jsonResp);
			return true;
		}

		String date, action, time;

		static const char* date_p = "d_";
		static const char* action_p = "a_";
		static const char* time_p = "t_";

		pumpTimer = arduino::utils::Timer("[action]");

		for (uint8 i = 0; i < SIZE_OF_ARR(tt); i++) {
			tt[i].enb = false;
		}

		uint8 tt_index = 0;

		for (uint8 i = 0; i < server.args(); i++)
		{
			if (server.argName(i).startsWith(date_p)) {
				date = server.arg(i); continue;
			}
			if (server.argName(i).startsWith(time_p)) {
				time = server.arg(i); continue;
			}
			if (server.argName(i).startsWith(action_p))
			{
				if (time.isEmpty())
				{
					arduino::utils::PlaceHolder p("data", "error wrong format of input , expected date,time,action ");
					server.send(200, "text/html", arduino::utils::PlaceHolder::tr(&p, FPSTR(response_html)));

					LOG_MSG("Error time is empty " << time.isEmpty());
					return true;
				}

				action = server.arg(i);
				int32 hour, min, yyyy, mm, dd;

				sscanf(time.c_str(), "%d:%d", &hour, &min);

				time_t exeTime = 0;

				if (!date.isEmpty())
				{
					sscanf(date.c_str(), "%d-%d-%d", &yyyy, &mm, &dd);

					tmElements_t tm;

					tm.Hour = hour;
					tm.Minute = min;
					tm.Second = 0;
					tm.Year = yyyy - 1970;
					tm.Day = dd;
					tm.Month = mm;

					exeTime = ::makeTime(tm);

					if (exeTime < TIME.getEpochTime()) {
						LOG_MSG("scheduled time " << TIME.getShortTimeDateStr(exeTime) << " is less that current time " << TIME.getShortTimeDateStr());
						continue;
					}
					tt[tt_index].recuring = false;
				}
				else//recurring action
				{
					exeTime = TIME.getEpochDate() + hour * arduino::utils::Timer::HOUR + min * arduino::utils::Timer::MINUTE;

					if (exeTime < TIME.getEpochTime())
						exeTime += arduino::utils::Timer::DAY;

					tt[tt_index].recuring = true;
				}

				tt[tt_index].code = (action.equals("on")) ? 0 : 1;
				tt[tt_index].date_time = exeTime;
				tt[tt_index].enb = true;
				tt[tt_index].id = tt_index++;

				action.clear(); time.clear(); date.clear();
			}

			ESP.wdtFeed();
		}

		for (uint8 i = 0; i < SIZE_OF_ARR(tt); i++) {
			if (tt[i].enb) {
				LOG_MSG(tt[i].toJson());
				tt[i].schedule();
			}
		}

		PlaceHolder p("data", "schedule updated with " + String(tt_index) + " entities");
		server.send(200, "text/html", PlaceHolder::tr(&p, FPSTR(response_html)));
		return true;
	}
};

void notFound() {
	String message = "Not Found\n\n";
	message += "URI: ";
	message += server.uri();
	message += "\nMethod: ";
	message += (server.method() == HTTP_GET) ? "GET" : "POST";
	message += "\nArguments: ";
	message += server.args();
	message += "\n";
	for (uint8 i = 0; i < server.args(); i++) {
		message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
	}
	server.send(404, "text/plain", message);
}

//The function can get const parameter event though it is not defined as it because 
// std::function defined both constructors canst and non const :) template magic



void user_dump_rst_info()
{
	struct rst_info* rtc_info = system_get_rst_info();
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

class Hydration_RequestHander : public RequestHandler
{
public:

	Hydration_RequestHander(const char* i_action) : m_uri(i_action) {
	}

	bool canHandle(HTTPMethod method, String uri) {
		LOG_MSG("Request : " << uri << " , can handle : " << m_uri);

		if (method == HTTP_GET && uri.startsWith('/' + m_uri))
			return true;
		else
			return false;
	}

	bool handle(ESP8266WebServer& server, HTTPMethod requestMethod, String requestUri) {
		LOG_MSG("Request : " << requestUri << " " << requestMethod);
		using namespace arduino::utils;

		LOG_MSG("Going to invoke action " << server.argName(0));

		if (server.argName(0).equals("on")) { pump.start(); }
		if (server.argName(0).equals("off")) { pump.stop(); }
		server.send(200, "text/html", success_html);


		return true;
	}

private:
	String m_uri;
};


void setup()
{
	using namespace arduino::utils;

	pinMode(PUMP_PIN, OUTPUT);
	digitalWrite(PUMP_PIN, LOW);
	stsRedLed.turn_off();
	rgbLed.turn_off();

	//pinMode(SELECTOR_PIN, INPUT);

	Serial.begin(115200);

	WiFi.persistent(false);
	WiFi.disconnect();

	Wire.begin(SDA_PIN, SCL_PIN);
	Wire.beginTransmission(0x76);

	if (0 == Wire.endTransmission())
	{
		LOG_MSG("BME device found on channel " << 0x76);
		bme.enabled = (*bme).begin(0x76);
	}
	else {
		LOG_MSG("BME device not found on channel ");
	}

	Wire.beginTransmission(0x27);
	if (0 == Wire.endTransmission())
	{
		LOG_MSG("BME device found on channel " << 0x27);
		display.enabled = (*display).begin();
	}
	else {
		LOG_MSG("LCD device not found on channel ");
	}

	server.begin();

	EEPROM_Adapter_t::begin();
	TIME.begin(); //no need the time starts from 0

	bool sts = true;
	uint16 index = 0;
	sts &= EEPROM_Adapter_t::load(ssid, index);
	sts &= EEPROM_Adapter_t::load(ssid_password, index);

	if (!sts)
	{
		index = 0;
		ssid = WIFISSID;
		ssid_password = WIFIPASS;

		index = EEPROM_Adapter_t::save(ssid, index);
		index = EEPROM_Adapter_t::save(ssid_password, index);
	}

	LOG_MSG("Going to connect to " << ssid);

	WiFi.hostname(String(hostname) + arduino::utils::Constants::ID());
	WiFi.begin(ssid.c_str(), ssid_password.c_str());
	WiFi.setAutoReconnect(true);

	static WiFiEventHandler onGotIPhandler = WiFi.onStationModeGotIP([](const WiFiEventStationModeGotIP& event) { LOG_MSG("Local IP " << event.ip.toString()); rgbLed.turn_off(); stsRedLed.turn_on(); });
	static WiFiEventHandler onDisconnect = WiFi.onStationModeDisconnected([](const WiFiEventStationModeDisconnected& event) { LOG_MSG("Disconnected"); rgbLed.turn_on(RGBLed::LedType::Red); });

	idleTimer.addRecuringTask(TIME.getEpochTime(), 3, [](long& io_time) { if (!WiFi.isConnected()) { stsRedLed.turn_off(); rgbLed.turn_on(RGBLed::LedType::Red); }});

	idleTimer.addRecuringTask(TIME.getEpochTime(), 60, [](long& io_time) { if (WiFi.isConnected()) { LOG_MSG("MY IP " << WiFi.localIP().toString()); } });

	if (bme.enabled) {
		idleTimer.addRecuringTask(TIME.getEpochTime(), 2 * arduino::utils::Timer::DAY / SIZE_OF_ARR(bme.data_t), [](long& io_time) { static uint8 i = 0; bme.data_t[i++ % SIZE_OF_ARR(bme.data_t)] = make_Pair(bmeReadTemperature(), bmeReadHumidity()); });
	}

	// Send web page with input fields to client
	server.on("/", []() {
		String html;
		html += HTML_open;
		html += HTML_body_open;

		html += "Temperature :";
		html += bmeReadTemperature();
		html += HTML_new_line;
		html += "Humidity :";
		html += bmeReadHumidity();

		html += HTML_body_close;
		html += HTML_close;

		server.sendHeader("Access-Control-Allow-Origin", "*");
		server.send(200, "text/html", html.c_str());
	});


	server.on("/bme", []() {
		/*
		[
		{
			t: value,
			h: value
		}
		]
		*/
		static const char objOpen = '{';
		static const char objClose = '}';
		String jsonResp;
		jsonResp += '[';
		for (int8 i = 0; i < SIZE_OF_ARR(bme.data_t); i++)
		{
			jsonResp += objOpen;
			jsonResp += "\"t\":";
			jsonResp += String(bme.data_t[i].first);
			jsonResp += ',';
			jsonResp += "\"h\":";
			jsonResp += String(bme.data_t[i].second);
			jsonResp += objClose;
			if (i != SIZE_OF_ARR(bme.data_t) - 1)
				jsonResp += ',';
		}
		jsonResp += ']';
		server.sendHeader("Access-Control-Allow-Origin", "*");
		server.send(200, "text/html", jsonResp.c_str());
	});

	/*Setup SSDP*/

	server.on("/description.xml", HTTP_GET, []() {
		SSDP.schema(server.client());
	});



	SSDP.setDeviceType("upnp:rootdevice");
	SSDP.setSchemaURL("description.xml");
	SSDP.setHTTPPort(80);
	String _SSID(hostname);
	_SSID += arduino::utils::Constants::ID();
	SSDP.setName(_SSID);
	SSDP.setSerialNumber(arduino::utils::Constants::ID());
	SSDP.setURL("/");
	SSDP.setModelName(_SSID);
	SSDP.setModelNumber(arduino::utils::Constants::ID());
	SSDP.setModelURL("/");
	SSDP.setManufacturer("");
	SSDP.setManufacturerURL("/");
	SSDP.begin();

	arduino::utils::OTA_Init(BUILD_ID);

	server.addHandler(new LandingPage_RequestHandler());
	server.addHandler(new Hydration_RequestHander("run"));
	server.on("/test", []() { Led tst(LED_PIN); tst.rapid_blynk(2000); server.send(200, "text/html", success_html);  });
	server.on("/clean", []() {
		LOG_MSG("Cleaning eeprom");
		EEPROM_Adapter_t::clean();
		PlaceHolder p("delay", "5");
		server.send(200, "text/html", PlaceHolder::tr(&p, FPSTR(restart_in_process_html)));
		//restart in 5 sec
		idleTimer.addTask(TIME.getEpochTime() + 5, [](long&) { LOG_MSG("Going to restart");  RESET(); });
	});

	server.on("/restart", []() {
		PlaceHolder p("delay", "10");
		server.send(200, "text/html", PlaceHolder::tr(&p, FPSTR(restart_in_process_html)));
		//restart in 5 sec
		idleTimer.addTask(TIME.getEpochTime() + 10, [](long&) { LOG_MSG("Going to restart");  RESET(); });
	});

	server.onNotFound(notFound);

	// display.setFont(u8x8_font_chroma48medium8_r);
	if (display.enabled) {
		(*display).clear(0, 0);
		(*display).noCursor();
		(*display).noBlink();
		(*display).print("  initializing  ");
		(*display).backlight();
	}

	
	if (display.enabled) {
		idleTimer.addRecuringTask(TIME.getEpochDate(), 10, [](long&) { if (mode == IDLE) { (*display).idleScreen(); } });
	}

	stsRedLed.turn_off();
	rgbLed.turn_off();

	user_dump_rst_info();
	//else default values remain
}

void switchMode(Mode i_mode = IDLE)
{
	//  LOG_MSG("Switch from " << (short)mode << " to " << (short) i_mode);

	//  using namespace arduino::utils;
	//
	//  static unsigned long pumpOnTime = 0;

	//  switch ( i_mode )
	//  {
	//     case SETUP:
	//     {
	//         pumpTimer.addTask(TIME.getEpochTime() + 5 * Timer::MINUTE, [](long&) { switchMode( LAST ); });//save
	//     }
	//     case SETUP_1:
	//     case SETUP_2:
	//     { 
	//       mode = i_mode;
	//     }
	//     break;
	//     case IDLE:
	//     {
	//        if ( mode == PUMP_ON )  // PUMP_OFF
	//         {    
	//             lastWateringTime = TIME.getEpochTime();
	//     
	//             display.clear(0, 0);
	//             display.printf("%s", "Watering done");
	//             display.setCursor(0, 1);
	//             display.printf( "Took %.1f litter", (float) ( ( TIME.getEpochTime() - pumpOnTime ) * LITER_PER_MINUTE / Timer::MINUTE ));
	//         }
	//         mode = IDLE;
	//     }
	//     break;
	//     case PUMP_ON:
	//     {
	//         // Pump started
	//         if ( mode != PUMP_ON )
	//         {
	//             //Pump ended              
	//             pumpOnTime = TIME.getEpochTime();

	//             nextWateringTime = TIME.getEpochTime() + workFrequency_Hours * Timer::HOUR;
	//             
	//             display.clear(0,0);
	//             display.printf("%s", "Watering till");
	//             display.setCursor(0, 1);
	//             display.print( TIME.getTimeStr ( TIME.getEpochTime() + workTime_Min * Timer::MINUTE ) );
	//         }
	//         mode = PUMP_ON;
	//     }
	//     break;
	//     default: // save
	//     {
	//         using namespace arduino::utils;
	//         pumpTimer = arduino::utils::Timer();

	//         short delayInHours = ( TIME.getEpochDate() + workStartTime_Hours * Timer::HOUR > TIME.getEpochTime() ) ? 0 : 24;
	//         long startWateringTime = TIME.getEpochDate() + (workStartTime_Hours + delayInHours) * Timer::HOUR;
	//         
	//         pumpTimer.addRecuringTask( startWateringTime , workFrequency_Hours * Timer::HOUR, startPump );

	//         nextWateringTime = startWateringTime;
			 //if (0 == lastWateringTime)
			 //	lastWateringTime = TIME.getEpochDate();

	//         if ( mode & SETUP_2 )
	//         {
	//             display.clear(0, 0);
	//             display.printf("%s", "Save Water Time");
	//             display.setCursor(0, 1);
	//             display.printf("%s %d m", TIME.getShortTimeDateStr(nextWateringTime), workTime_Min);

	//             EEPROM.write(0, (uint8_t)workStartTime_Hours);
	//             EEPROM.write(1, (uint8_t)workFrequency_Hours);
	//             EEPROM.write(2, (uint8_t)workTime_Min);
	//             EEPROM.commit();
	//         }

	//         LOG_MSG("Going back to idle mode next execution time is " << TIME.getTimeDateStr ( nextWateringTime ) << " work time is " << workTime_Min);

	//         mode = IDLE;

	//         idleTimer.addTask( TIME.getEpochTime() + 30, [](long&) { display.noBacklight(); } );
	//     }
	//  }

}


void loop()
{
	using namespace arduino::utils;
	//static bool taskSet = false;

	static Button_t botton(BUTTON_1_PIN,
		[]() {},
		[]() { (pump.on()) ? pump.stop(TIME.getEpochTime()) : pump.start(TIME.getEpochTime()); },
		[]() { (pump.on()) ? pump.stop(TIME.getEpochTime()) : pump.start(TIME.getEpochTime()); });


	OTA_CHECK_AND_UPDATE;

	server.handleClient();

	if (!TIME.synced())
	{
		if (millis() % 100)
		{
			LOG_MSG("Time is not synced yet , going to wait");

			(*display).initScreen();
		}

		TIME.run();
		return;
	}

	TIME.run();
	pumpTimer.run();
	botton.run();
	idleTimer.run();
}

