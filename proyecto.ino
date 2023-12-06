//sudo chmod 666 /dev/ttyUSB0
#include <LiquidCrystal_I2C.h>

#define VRX_PIN 39
#define VRY_PIN 36
#define VRPUSH_PIN 17
#define MOISUREOUT_PIN 34 // ESP32 pin GPIO34 (ADC0) that connects to AOUT pin of moisture sensor
// El valor va entre 900 (Humedo) hasta 2700 (Seco)
#define POWER_RAIN_PIN 32  // ESP32's pin GPIO32 that provides the power to the rain sensor
#define AO_RAIN_PIN 35     // ESP32's pin GPIO36 connected to AO pin of the rain sensor
// 4095 seco -> 2400-2600 mojado y sube gradualmente hasta 3000. Diria que < 3100 es que esta lloviendo. > a eso es que dejó de llover.
#define LED_ON_PIN 18     // ESP32's pin GPIO36 connected to AO pin of the rain sensor
#define LIGHT_SENSOR_PIN 25 // ESP32 pin GIOP25 (ADC0)
// Con lampara aprox 1700-1800. Luz ambiente 1200 a 1400. A partir de <= 700 podria ser apto para riego
// A las 7:30 hay entre 3700 y 3800 (Sin sol directo) (< 3700 se podria regar)
// Maximo lightLimit con sol tenue directo esta entre 3900 y 4100
#define RELAY_PIN 16 // ESP32 pin GPIO27 connected to the IN pin of relay
#define ADC_VREF_mV    5500.0 // in millivolt
#define ADC_RESOLUTION 4096.0
#define PIN_LM35 27 // ESP23 pin GPIO36 (ADC0) connected to LM35

// Variables para joystick
#define UP    0
#define RIGHT 1
#define DOWN  2
#define LEFT  3
#define ENTER 4
#define NONE  5

int valueX = 0;
int valueY = 0;
int SwitchValue = 0;

// Variables pantalla LCD
#define LCD_COLUMNS 16
#define LCD_ROWS 2

// Variables sensores
float moisurePer;
int moisureMax = 2700; // Valor de humedad suelo maximo
int moisureLimit = 35; // Valor de humedad suelo inicial %

class Moisure {
   public:
      int maximo;
      int limit;
      double percentage;

      // Member functions declaration
      double getValue(void);
};

float rainPer;
int rainMax = 4095; // Valor de precipitaciones maximo
int rainLimit = 25; // Valor de precipitaciones inicial %

float tempPer;
int tempLimit = 5; // Valor de temperatura inicial °C

float lightPer;
int lightMax = 3600; // Valor de luminosidad maximo
int lightLimit = 40; // Valor de luminosidad inicial %

int modo = 2; // Inicializar en "Off"

// Variables para el menu
int currentMenu = 0; // Variable para el menú actual
int currentSubMenu = 0; // Variable para el submenú actual

struct MenuItem {
  String name;
  int* value;
  int minVal;
  int maxVal;
  float* perVal;
};

// Array de elementos de menú
MenuItem menuItems[] = {
  {"Humedad:", &moisureLimit, 0, 100, &moisurePer},
  {"Precip.:", &rainLimit, 0, 100, &rainPer},
  {"Temp.:", &tempLimit, 0, 50, &tempPer},
  {"Lumin.:", &lightLimit, 0, 100, &lightPer},
  {"Modo:", &modo, 0, 2}
};

String tercerMenuOptions[] = {"Auto", "Manual", "Off"};


LiquidCrystal_I2C lcd(0x27, LCD_COLUMNS, LCD_ROWS);

void setup() {
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  pinMode(VRX_PIN, INPUT);
  pinMode(VRY_PIN, INPUT);
  pinMode(VRPUSH_PIN, INPUT_PULLUP);
  pinMode(POWER_RAIN_PIN, OUTPUT);
  pinMode(LED_ON_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
}

void loop() {
  // Obtengo los valores de los sensores
  float tempC = getTemperatura();
  int moisureValue = getMoisureValue();
  int lightValue = getLightValue();
  int rainValue = getRainValue();

  // Obtengo los valores seteados en la configuracion
  tempPer = (float)tempC;
  moisurePer = getPorcent(moisureValue, moisureMax);
  lightPer = getPorcent(lightValue, lightMax);
  rainPer = getPorcent(rainValue, rainMax);
  
  int joystickAction = read_joystick(); // Lee la acción del joystick

  // Verificar la acción del joystick
  if (currentSubMenu == 0) {
    if (joystickAction == DOWN) {
      currentMenu = (currentMenu + 1) % 5;
    } else if (joystickAction == UP) {
      currentMenu = (currentMenu - 1 + 5) % 5;
    }
  }

  // Manejar el menú actual
  if (joystickAction == ENTER) {
    if (currentSubMenu == 0) {
      currentSubMenu = 1;
    } else if (currentSubMenu == 1) {
      if (currentMenu == 4) {
        // Lógica específica para el modo
        modo = (modo) % 3; // Ciclar entre 0, 1 y 2
      }
      currentSubMenu = 0;
    }
  } else if (currentSubMenu == 1) {
    if (joystickAction == LEFT) {
      updateValue(*menuItems[currentMenu].value, menuItems[currentMenu].minVal, menuItems[currentMenu].maxVal, -1);
    } else if (joystickAction == RIGHT) {
      updateValue(*menuItems[currentMenu].value, menuItems[currentMenu].minVal, menuItems[currentMenu].maxVal, 1);
    }
  }

  // Asegurarse de que los valores estén dentro de los límites
  *menuItems[currentMenu].value = constrain(*menuItems[currentMenu].value, menuItems[currentMenu].minVal, menuItems[currentMenu].maxVal);

  // Mostrar el menú en la pantalla
  displayMenu(currentMenu, currentSubMenu);
    
  // Switcheo on/off dependiendo modo y variables sensores
  switch (modo) {
    case 0:
      // Condicion para encendido automatico:
      // 0. modo debe ser automatico (0)
      // 1. El valor humedad del suelo debe ser menor o igual al seteado
      // 2. El valor luminosidad debe ser menor o igual al seteado
      // 3. El valor humedad por lluvia debe ser menor o igual al seteado
      // 4. El valor de temperatura debe ser mayor al seteado
      if(moisurePer <= moisureLimit && 
        lightPer <= lightLimit && 
        rainPer <= rainLimit && 
        tempC > tempLimit){        
        Serial.print("ENCENDIDO ");
        digitalWrite(LED_ON_PIN, HIGH);
        digitalWrite(RELAY_PIN, HIGH);
      }else{
        Serial.print("APAGADO ");
        digitalWrite(LED_ON_PIN, LOW);
        digitalWrite(RELAY_PIN, LOW);
      }
      break;
    case 1:
      Serial.print("ENCENDIDO MANUAL ");
      digitalWrite(LED_ON_PIN, HIGH);
      digitalWrite(RELAY_PIN, HIGH);
      break;
    default:
      Serial.print("APAGADO MANUAL");
      digitalWrite(LED_ON_PIN, LOW);
      digitalWrite(RELAY_PIN, LOW);
  }

  Serial.print(" | moisurePer: ");
  Serial.print(moisurePer);
  Serial.print(" | moisureLimit: ");
  Serial.print(moisureLimit);
  Serial.print(" | lightPer: ");
  Serial.print(lightPer);
  Serial.print(" | lightLimit: ");
  Serial.print(lightLimit);
  Serial.print(" | rainPer: ");
  Serial.print(rainPer);
  Serial.print(" | rainLimit: ");
  Serial.print(rainLimit);
  Serial.print(" | tempPer: ");
  Serial.print(String(tempPer) + "°C");
  Serial.print(" | tempLimit: ");
  Serial.println(String(tempLimit) + "°C");

  delay(100);  
}

// Lectura del joystick
int read_joystick() {
  int output = NONE;
  valueX = analogRead(VRX_PIN);
  valueY = analogRead(VRY_PIN);
  SwitchValue = digitalRead(VRPUSH_PIN);

  if (SwitchValue == 0) {
    output = ENTER;
  } else if (valueX >= 2200) {
    output = RIGHT;
  } else if (valueX <= 500) {
    output = LEFT;
  } else if (valueY >= 2200) {
    output = UP;
  } else if (valueY <= 500) {
    output = DOWN;
  }
  return output;
}

// Visualización del menu
void displayMenu(int currentMenu, int currentSubMenu) {
  lcd.clear();
  
  if(currentMenu == 0 || currentMenu == 1 || currentMenu == 3){
    lcd.print(menuItems[currentMenu].name + " " + String(*menuItems[currentMenu].perVal) + "%");
  }else if(currentMenu == 2){
    lcd.print(menuItems[currentMenu].name + " " + String(*menuItems[currentMenu].perVal));
    lcd.print("\xDF" "C"); // °C
  }else{
    lcd.print(menuItems[currentMenu].name);
  }
  
  lcd.setCursor(0, 1);
  if (currentMenu == 4) {
    lcd.print(tercerMenuOptions[modo]);
  } else if(currentMenu == 2) {
    lcd.print("" + String(*menuItems[currentMenu].value));
    lcd.print("\xDF" "C     "); // °C
  } else {
    lcd.print("" + String(*menuItems[currentMenu].value) + "%     ");
  }
  
  if (currentSubMenu == 1) {
    lcd.setCursor(0, 1);
    lcd.print("< ");
    if (currentMenu == 4) {
      lcd.print(tercerMenuOptions[modo]);
    } else if(currentMenu == 2) {
      lcd.print(String(*menuItems[currentMenu].value));
      lcd.print("\xDF" "C"); // °C
    } else {
      lcd.print(String(*menuItems[currentMenu].value) + "%");
    }
    lcd.print(" >   ");
  }
}

// Seteo del valor actualizado por menu
void updateValue(int& value, int minVal, int maxVal, int increment) {
  value = constrain(value + increment, minVal, maxVal);
}

// Obtengo valor humedad suelo del sensor
float getMoisureValue(){
  return -analogRead(MOISUREOUT_PIN) + moisureMax;
}

// Obtengo valor luminosidad del sensor
int getLightValue(){
  return analogRead(LIGHT_SENSOR_PIN);
}

// Obtengo y calculo temperatura del sensor
float getTemperatura(){
  // read the ADC value from the temperature sensor
  int adcVal = analogRead(PIN_LM35);
  // convert the ADC value to voltage in millivolt
  float milliVolt = adcVal * (ADC_VREF_mV / ADC_RESOLUTION);
  // convert the voltage to the temperature in °C
  float tempC = milliVolt / 10;
  return tempC;
}

// Obtengo valor lluvia del sensor
int getRainValue(){
  digitalWrite(POWER_RAIN_PIN, HIGH);
  delay(10);
  int value = analogRead(AO_RAIN_PIN);
  digitalWrite(POWER_RAIN_PIN, LOW);
  return (-value) + rainMax;
}

float getPorcent(float val, float maximo){
  float percent = (val / maximo) * 100;
  return (percent < 0) ? 0 : (percent > 100) ? 100 : percent;
}

float getValue(float val, float maximo){
  float percent = (val / maximo) * 100;
  return (percent < 0) ? 0 : (percent > 100) ? 100 : percent;
}
