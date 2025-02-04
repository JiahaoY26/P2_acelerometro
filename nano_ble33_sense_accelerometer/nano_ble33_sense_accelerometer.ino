#define LED_RED    LEDR
#define LED_GREEN  LEDG
#define LED_BLUE   LEDB

void setLEDColor(bool red, bool green, bool blue) {
    digitalWrite(LED_RED, red);
    digitalWrite(LED_GREEN, green);
    digitalWrite(LED_BLUE, blue);
}

void setup() {
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
}

void loop() {
    // Rojo
    setLEDColor(true, false, false);
    delay(1000);
    
    // Rosa (Rojo + Azul)
    setLEDColor(true, false, true);
    delay(1000);
    
    // Verde
    setLEDColor(false, true, false);
    delay(1000);
    
    // Amarillo (Rojo + Verde)
    setLEDColor(true, true, false);
    delay(1000);
    
    // Azul
    setLEDColor(false, false, true);
    delay(1000);
    
    // Morado (Azul + Rojo)
    setLEDColor(true, false, true);
    delay(1000);
}
