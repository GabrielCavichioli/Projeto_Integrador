/*
Título: Projeto Integrador I - SAVA
Autores: Gabriel Antonio Cavichioli e Letícia Rech.
Data: 07/09/2024.

Códigos adjuntos até o momento: 
Leitura de temperatura + acionamento do aquecedor;
Leitura de EC + acionamento da bomba peristáltica;
Timer + LCD;
Sensores de nível do reservatório + do reservatório de solução;
*/

/*__________________________________________________Bibliotecas__________________________________________________*/
#include <OneWire.h> // Biblioteca para o sensor de temperatura
#include <DallasTemperature.h> // Biblioteca para o sensor de temperatura
#include <LiquidCrystal_I2C.h> // Biblioteca do display LCD
#include <WiFi.h>                                           // Responsável pelo WiFi

//WiFi -----------------------------------------------------------------------------------------------
const char* ssid = "Teste";                    // Nome da rede WiFi
const char* password = "batatadoce";               // Senha do WiFi
WiFiServer server(80);                            // Declaração do servidor e porta

/*_____________________________________________________Timer_____________________________________________________*/
hw_timer_t *timer = NULL; // Criando o timer 
volatile uint32_t segs = 0; // Variável volátil para suportar modificações "bruscas" do timer nela

/*____________________________________________________Defines____________________________________________________*/
#define tempSensorPin 14 // Sensor de temperatura (GPIO14 - ADC16)
#define levelSensorOnTop 36 // Sensor de nível superior do reservatório
#define levelSensorBelow 39 // Sensor de nível inferior do reservatório
#define levelSensorNutri 34 // Sensor de nível inferior do reservatório de solução
#define tdsSensorPin 27 // Pino de dados do sensor TDS
#define heaterPin 19 // Pino do cabo de alimentação do aquecedor (comentado pois não temos o transistor para ele ainda para testes)
#define peristalticPumpPin 4 // Pino da bomba peristáltica
#define pumpPin 15 // Pino da bomba
#define valvPin 18 // Pino da válvula de água do reservatório
#define ONBOARD_LED 2 // LED OnBoard do ESP32
#define tempOn 15 // Tempo ligado da bomba
#define tempOff 10 // Tempo desligado da bomba
#define VREF 3.3 // Tensão de alimentação do sensor TDS (referência, valor do ADC)
#define SCOUNT 30 // Quantidade de amostras para somatório de leituras do sensor TDS

/*___________________________________________________Variáveis___________________________________________________*/
int analogBuffer[SCOUNT], analogBufferTemp[SCOUNT]; // Array para somatório de amostras do sensor TDS e array para somatório de amostras temporárias do sensor TDS
int analogBufferIndex = 0, copyIndex = 0; // Index do buffer que recebe a leitura e referência copiada do index sensor TDS 
unsigned char mins = 0, mins_lcd = 0, circular = 0; // mins da lógica de clico da bomba, mins do LCD e circular para controlar o ciclo da bomba
bool x = 1; // Controle da atualização do LCD

float averageVoltage = 0; // Leitura de tensão do ADC, sensor TDS
float tdsValue = 0, tdsValueEC = 0, plantEC = 1.3; // Valor do TDS em ppm, valor do TDS convertido para EC e valor de EC para uma planta
float temperature = 25, tempC = 0; // Temperatura ambiente para compensação e leitura do sensor de temperatura

/*_________________________________________________Instanciamentos_______________________________________________*/
OneWire oneWire(tempSensorPin); // Instanciando o pino de leitura para a biblioteca sensor de temperatura
DallasTemperature sensors(&oneWire); // Referência de temperatura para a bibliocteca da dallas
LiquidCrystal_I2C lcd(0x27, 16, 2); // Instaciamento para o dislay LCD, com enderço do LCD, número de caracteres e linhas

/*_____________________________________________________Funções___________________________________________________*/
// Função para ler a temperatura
float lerTemp() {
  sensors.requestTemperatures(); // Comando para obter temperatura
  float tempC = sensors.getTempCByIndex(0); // Como a biblioteca obtem a leitura para mais de um sensor no barramento, temos que selecionar o primeiro e único

  // Checando se a leitura ocorreu corretamente
  if(tempC != DEVICE_DISCONNECTED_C) 
  {
    return tempC;
  } else {
    // Serial.println("Erro: não foi possível ler a temperatura (index 0)");
  }
}

// Filtro digital de média móvel
int getMedianNum(int bArray[], int iFilterLen){
  int bTab[iFilterLen];
  for (byte i = 0; i<iFilterLen; i++)
  bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0){
    bTemp = bTab[(iFilterLen - 1) / 2];
  }
  else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}

// Funsão ISR (interrupção) do timer
void IRAM_ATTR timer_ISR() {
  digitalWrite(ONBOARD_LED, !digitalRead(ONBOARD_LED));
  segs++; // Contando segundos
}

// Função para atualizar display LCD
void att(){
  lcd.clear();
  if(mins_lcd <= 9){
    lcd.setCursor(1, 0);
    lcd.print("0");
    lcd.setCursor(2, 0);
    lcd.print(mins_lcd);
  } else {
    lcd.setCursor(1, 0);
    lcd.print(mins_lcd);
  }
  lcd.setCursor(3, 0);
  lcd.print(":");
   if(segs <= 9){
    lcd.setCursor(4, 0);
    lcd.print("0");
    lcd.setCursor(5, 0);
    lcd.print(segs);
  } else {
    lcd.setCursor(4, 0);
    lcd.print(segs);
  }
  lcd.setCursor(7, 0);
  lcd.print("Bomba:");
  if(circular == 1){
    lcd.setCursor(13, 0);
    lcd.print("On");
  } else {
    lcd.setCursor(13, 0);
    lcd.print("Off");
  }
  lcd.setCursor(0, 1);
  lcd.print("TT:");
  lcd.setCursor(3, 1);
  lcd.print(tempC);
  lcd.setCursor(9, 1);
  lcd.print("EC:");
  lcd.setCursor(12, 1);
  lcd.print(tdsValueEC);
  if(digitalRead(levelSensorNutri) == HIGH){
  delay(1000);
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("Sem Solucao de");
  lcd.setCursor(2, 1);
  lcd.print("Nutrientes!!");
  delay(1000);
  }
}

// SAVA no display LCD
void SAVA(){
  lcd.clear();
  lcd.setCursor(6, 0);
  lcd.print("SAVA");
  lcd.setCursor(5, 1);
  lcd.print("------");
}

/*____________________________________________________Main____________________________________________________*/
void setup(void)
{
  Serial.begin(115200); // Freq de comunicação serial

  // Definindo os GPIO's
  pinMode(heaterPin, OUTPUT); // Pino do aquecedor como saída
  pinMode(valvPin, OUTPUT); // Pino da válvula de água do reservatório como saída
  pinMode(peristalticPumpPin, OUTPUT); // Pino da bomba peristáltica como saída
  pinMode(pumpPin, OUTPUT); // Pino da bomba como saída
  pinMode(ONBOARD_LED, OUTPUT); // Definindo o LED OnBoard como saída
  pinMode(tdsSensorPin, INPUT); // Pino do sensor TDS como entrada
  pinMode(levelSensorOnTop, INPUT); // Pino do sensor de nível superior do reservatório como entrada
  pinMode(levelSensorBelow, INPUT); // Pino do sensor de nível inferior do reservatório como entrada
  pinMode(levelSensorNutri, INPUT); // Pino do sensor de nível inferior do reservatório de solução como entrada

  // Iniciando a biblioteca do sensor de temperatura
  sensors.begin();

  // Iniciando o timer e setando os valores de preescale defino, a cada 1 seg
  timer = timerBegin(0, 80, true); // Inicializando o timer 0, com um prescale de 80, e com contagem crescente
  timerAttachInterrupt(timer, &timer_ISR, true); // Atrelando o timer criando ao ISR para ela, com definição de ativação na borda
  timerAlarmWrite(timer, 1000000, true); // Definindo a quantidade de ticks para o timer, e também auto-reload
  timerAlarmEnable(timer); // Habilitando o timer

  // Inicialinado o display LCD
  lcd.init(); // Inicializando o display pela biblioteca
  lcd.backlight(); // Ativando a luz de fundo do display

  SAVA();
  delay(2000);

  //Configurações de WiFi
  Serial.print("Conectando no WiFi: ");                   // Imprime na Serial
  Serial.println(ssid);                                   // Imprime ssid na Serial
  WiFi.begin(ssid, password);                             // Tenta se conectar no WiFi
  
  while (WiFi.status() != WL_CONNECTED) {                 // Verifica se a Conexão foi realizada
      Serial.print(".");                                  // Se não imprime na Serial
      digitalWrite(ONBOARD_LED, !digitalRead(ONBOARD_LED));
      delay(500);                                         // Espera 0,5s
  }

  Serial.println("\nWifi Conectado");             // Imprime na Serial
  Serial.print("Endereço de IP: ");               // Imprime na Serial
  Serial.println(WiFi.localIP());                 // Imprime IP do ESP32 na Serial
  server.begin();                                 // Inicializa o Servidor

}

void loop(void)
{ 
  // Lendo temperatura
  tempC = lerTemp();

  // Testando o limite desejável de temperatura para operar o aquecedor
  if (tempC <= 25) {
    digitalWrite(heaterPin, HIGH); // Ligando o aquecedor caso a temperatura seja menor que 25 graus
  } else if (tempC > 26) {
    digitalWrite(heaterPin, LOW); // Desligando caso seja maior que 26 graus
  }
  
  // Lendo sensor TDS
  temperature = tempC;
  static unsigned long analogSampleTimepoint = millis();

  if(millis()-analogSampleTimepoint > 40U){     // Leitura do ADC a cada 40 milisegundos
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(tdsSensorPin);    // Leitura do valor analógico e armazenamento no buffer
    analogBufferIndex++;
    if(analogBufferIndex == SCOUNT){ 
      analogBufferIndex = 0;
    }
  }   
  
  static unsigned long printTimepoint = millis();

  if(millis()-printTimepoint > 800U){
    printTimepoint = millis();
    for(copyIndex=0; copyIndex<SCOUNT; copyIndex++){
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
      
      // Leitura do valor ajustada pelo filtro de média móvel e o range do ADC
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 4096.0;
      
      // Fóruma de compensação de temperatura ambiente: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0)); 
      float compensationCoefficient = 1.0+0.02*(temperature-25.0);
      float compensationVoltage=averageVoltage/compensationCoefficient;
      
      // Convertendo o valor de tensão do ADC para o valor do TDS em ppm
      tdsValue=(133.42*compensationVoltage*compensationVoltage*compensationVoltage - 255.86*compensationVoltage*compensationVoltage + 857.39*compensationVoltage)*0.5;
      tdsValueEC=tdsValue/500; // Convertendo para EC    
    }
  }

  // Testando se há necessidade de dosagem de solução
  if ((tdsValueEC < plantEC) && (digitalRead(levelSensorBelow) == LOW) && (digitalRead(levelSensorOnTop) == HIGH) && (digitalRead(levelSensorNutri) == LOW)) {
    digitalWrite(peristalticPumpPin, HIGH); // Ligando a dosagem da bomba peristáltica caso o EC seja menor que o desejado
  } else if (tdsValueEC > (plantEC+0.05) || (digitalRead(levelSensorOnTop) == LOW) || (digitalRead(levelSensorNutri) == HIGH)) {
    digitalWrite(peristalticPumpPin, LOW); // Ligando a dosagem da bomba peristáltica caso o EC seja maior que o desejado em um range de 0.05
  }

  //Verifica se o sensor baixo está ativo e o alto desativado, se verdadeiro começa a inserir água no sistema
  if ((digitalRead(levelSensorBelow) == HIGH) && (digitalRead(levelSensorOnTop) == LOW)){
    // digitalWrite(ONBOARD_LED, HIGH);
    digitalWrite(valvPin, HIGH);
  } 

  //Verifica se o sensor alto está ativo e o baixo desativado, se verdadeiro para de inserir água no sistema
  else if ((digitalRead(levelSensorBelow) == LOW) && (digitalRead(levelSensorOnTop) == HIGH)){
    // digitalWrite(ONBOARD_LED, LOW);
    digitalWrite(valvPin, LOW);
  }
  
  //Sensor do recipiente com nutriente ligado significa que acabou a solução e deve ser dado um aviso para reposição
  // if(digitalRead(levelSensorNutri) == HIGH){
  //   Serial.println("Recipiente sem solução nutritiva");
  // }

  // Atualização dos minutos para o ciclo da bomba, do display LCD e dos segundos conforme a lógica de tempo
  if(segs >= 60){
    mins++;
    mins_lcd++;
    segs = 0;
    x = 0;
  }
  if(mins_lcd == 60) mins_lcd = 0;

  // Ativando a bomba para iniciar o ciclo ligado após o tempo definido de desligamento tempOff
  if((mins >= tempOff)&&(circular == 0)){
    digitalWrite(pumpPin, !digitalRead(pumpPin)); // Invertendo estado da bomba
    circular++;
    mins = 0;
  }

  // Desativando a bomba para iniciar o ciclo desligado após o tempo definido ligado tempOn
  if((mins >= tempOn)&&(circular == 1)){
    digitalWrite(pumpPin, !digitalRead(pumpPin)); // Invertendo estado da bomba
    circular = 0;
    mins = 0;
  }

  // Lógica para atualização do display LCD
  if(segs >= x){
    x++;
    att();
  }

  // Comunicação entre cliente e servidor
  WiFiClient client = server.available();

  if (client) {
    Serial.println("Novo cliente conectado");
    String currentLine = ""; // Armazena a linha atual da solicitação

    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);

        // Verifica se a linha terminou
        if (c == '\n') {
          if (currentLine.length() == 0) {
            // Verifica requisição específica antes de enviar a resposta
            if (currentLine.endsWith("GET /status")) {
              client.println("HTTP/1.1 200 OK");
              client.println("Content-Type: application/json");
              client.println();
              client.print("{\"nutrient_level\": ");
              client.print(digitalRead(levelSensorNutri));
              client.print(", \"ec\": ");
              client.print(tdsValueEC);
              client.print(", \"temperature\": ");
              client.print(tempC);
              client.print(", \"pump_status\": ");
              client.print(circular); // 1 para bomba ligada, 0 para desligada
              client.println("}");
              return; // Evita continuar enviando HTML
            }

            // Envia página HTML padrão
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();
            client.println("<center>");
            client.println("<h1>Controle de Nutrientes</h1>");
            client.println("<a href=\"/bot1\"><button>Botão 1 - Alface</button></a><br>");
            client.println("<a href=\"/bot2\"><button>Botão 2 - Rúcula</button></a><br>");
            client.println("<a href=\"/bot3\"><button>Botão 3 - Agrião</button></a><br>");
            client.println("<a href=\"/bot4\"><button>Botão 4 - Salsa</button></a><br>");
            client.println("<a href=\"/bot5\"><button>Botão 5 - Cebolinha</button></a>");
            client.println("</center>");
            client.println();

            int infonutri = digitalRead(levelSensorNutri);
            if (infonutri == 1) {
              client.println("<p>Recipiente sem solução nutritiva</p>");
            }
            client.println();
            break;
          } else {
            currentLine = ""; // Limpa a linha atual
          }
        } else if (c != '\r') {
          currentLine += c; // Adiciona caractere à linha atual
        }
      }
    }

    // Ajusta valores baseados nos botões clicados (processa ao final da leitura)
    if (currentLine.endsWith("GET /bot1")) {
      plantEC = 1.3; // Alface
      Serial.println("EC definido para Alface: 1.3");
    } else if (currentLine.endsWith("GET /bot2")) {
      plantEC = 1.6; // Rúcula
      Serial.println("EC definido para Rúcula: 1.6");
    } else if (currentLine.endsWith("GET /bot3")) {
      plantEC = 1.9; // Agrião
      Serial.println("EC definido para Agrião: 1.9");
    } else if (currentLine.endsWith("GET /bot4")) {
      plantEC = 1.7; // Salsa
      Serial.println("EC definido para Salsa: 1.7");
    } else if (currentLine.endsWith("GET /bot5")) {
      plantEC = 1.6; // Cebolinha
      Serial.println("EC definido para Cebolinha: 1.6");
    }

    client.stop();
    Serial.println("Cliente desconectado");
  }



  // Apresentando informações
  // Serial.print("Temperatura do sensor (index 0): ");
  // Serial.print(tempC);
  // Serial.println("ºC");
  // Serial.print("Tensão:");
  // Serial.print(averageVoltage);
  // Serial.print("V");
  // Serial.print("  |  TDS Value:");
  // Serial.print(tdsValue);
  // Serial.print("ppm");
  // Serial.print("  |  TDS ValueEC:");
  // Serial.print(tdsValueEC);
  // Serial.println("EC (mS/cm)");
}
