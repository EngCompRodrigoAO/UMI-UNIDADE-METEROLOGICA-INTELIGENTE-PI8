/************************************************** INCLUDES **************************************************/
#include <Arduino.h>
#include <WiFi.h>
#include <MQUnifiedsensor.h>
#include <ThingsBoard.h>
#include <ArduinoUniqueID.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

/************************************************* DEFINIÇÕES *************************************************/
#define TOKEN "nuQ84KS86qXFBrhJ21zO"
#define THINGSBOARD_SERVER "thingsboard.cloud"

#define placa "ESP-32"         // DEFINIR MICROCONTROLADOR PARA A BIBLIOTECA MQUNIFIEDSENSOR.H
#define Voltage_Resolution 3.3 // TENSÃO DE REFERENCIA DO MICROCONTROLADOR USADO
#define pin 32                 // CASO USE ARDUINO DIGITAR A0
#define type "MQ-135"          // NOME DO SENSOR EM USO NA MQ135
#define ADC_Bit_Resolution 12  // 10 PARA UNO/MEGA/NANO E 12 PARA ESP32 WROOM
#define RatioMQ135CleanAir 3.6 // RS / R0 = 3.6 ppm

#define TENSAO_PLACA_SOLAR 39          // AQUISIÇÃO DE TENSÃO PLACA SOLAR
#define TENSAO_BATERIA 36              // AQUISIÇÃO DE TENSÃO BATERIA
#define STATUS_CARREGADO_CARREGADA 5   // STATUS CARREGADOR DE BATERIA CARREGANDO
#define STATUS_CARREGADO_CARREGANDO 23 // STATUS CARREGADOR DE BATERIA CARREGADA
#define LED_LIGADO 19                  // LED DE STATUS LIGADO
#define LED_VIDA 18                    // LED DE STATUS VIDA
#define LED_WIFI 17                    // LED DE STATUS WIFI
#define LED_ERRO 16                    // LED DE STATUS ERRO
#define SENSOR_VELOCIDADE_VENTO_DIGITAL 13
//#define SENSOR_VELOCIDADE_VENTO_ANALOGICO 12
#define SENSOR_ORIENTACAO_VENTO_ANALOGICO 35
#define SENSOR_PLUVIOMETRICO_DIGITAL 27
#define SENSOR_SISMICO_DIGITAL 14
#define SENSOR_SISMICO_ANALOGICO 25
#define SENSOR_ULTRAVIOLETA 33
#define SENSOR_UMIDADE_SOLO_ANALOGICO 34
#define SENSOR_UMIDADE_SOLO_DIGITAL 26

#define SEALEVELPRESSURE_HPA (1013.25)

/******************************************** VARIAVEIS CONSTANTES ********************************************/
const char *ssid = "FALANGE_SUPREMA";
const char *password = "#kinecs#";
const char *ntpServer = "pool.ntp.org";   // Servidor relogio mundial
const int daylightOffset_sec = -3600 * 3; // Servidor relogio mundial segundos constantes em um dia
const long gmtOffset_sec = 0;             // Sevidor relogio mundial GMT do Brasil

/********************************************** VARIAVEIS GLOBAIS **********************************************/
double CO2 = (0);
float CO = 00.00, NH3 = 00.00, CH3 = 00.00, C2H6O = 00.00, C3H6O = 00.00;
int ID_TEMPO = 0;
float TENSAO_CALCULADA = 00.00;
float vetTensao[1000];
float valor_medio;
int CONTADOR_ID = 1;
int TEMPO_EXECUCAO = 0;
float tensaoEntrada = 0.0; // VARIÁVEL PARA ARMAZENAR O VALOR DE TENSÃO DE ENTRADA DO SENSOR
float tensaoMedida = 0.0;  // VARIÁVEL PARA ARMAZENAR O VALOR DA TENSÃO MEDIDA PELO SENSOR
float valorR1 = 30000.0;   // VALOR DO RESISTOR 1 DO DIVISOR DE TENSÃO
float valorR2 = 7500.0;    // VALOR DO RESISTOR 2 DO DIVISOR DE TENSÃO
int leituraSensor = 0;     // VARIÁVEL PARA ARMAZENAR A LEITURA DO PINO ANALÓGICO

int rpm, CONTADOR_ATUALIZACAO_SERVER = 0, MES_ATUAL, MES_ANTERIOR, TEMPO_APRESENTA = 3000, interval = 1000, NIVEL_UV, BIRUTA, PLUVIOMETRICO, UMIDADE_SOLO, UMIDADE, PRESSAO, ALTITUDE, uvLevel;
float VELOCIDADE_VENTO, TEMPERATURA, SISMICO;
char DIRECAO_VENTO_NOMECLATURA[16][4] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"}; // DIREÇÃO DO VENTO
char COND_UV[5][9] = {"BAIXO", "MODERADO", "ALTO", "ELEVADO", "EXTREMO"};                                                                     // NIVEIS DE RADIAÇÃO UV
volatile byte pulsos;
unsigned long timeold;
unsigned int pulsos_por_volta = 20; // Altere o numero de acordo com o disco encoder
int refLevel = 0;
float outputVoltagem = 00.00;
String NUMERO_SERIE = "";

MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, type);
Adafruit_BME280 bme;
// INICIALIZANDO O CLIENTE ThingsBoard
WiFiClient espClient;
// INICIALIZAÇÃO DA INSTANCIA DO ThingsBoard
ThingsBoard tb(espClient);
// STATUS DO SINAL DE WIFI
int status = WL_IDLE_STATUS;

// FUNÇÃO PARA CONECTAR NA REDE WIFI
void CONECTAR_WIFI()
{
  digitalWrite(LED_WIFI, LOW);
  WiFi.begin(ssid, password);
  Serial.print("CONECTANDO");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    digitalWrite(LED_WIFI, HIGH);
    Serial.print(".");
    delay(500);
    digitalWrite(LED_WIFI, LOW);
  }
  Serial.println();
  digitalWrite(LED_WIFI, HIGH);
  Serial.print("CONECTADO, ENDEREÇO IP: ");
  Serial.println(WiFi.localIP());
}

// FUNÇÃO LÊR TENSÃO
void TENSAO(int TIPO)
{
  TENSAO_CALCULADA = 0;
  double maior_Valor = 0;

  float tensaoOFFSET = 1.1335567;

  for (int i = 0; i < 1000; i++)
  {
    vetTensao[i] = analogRead(TIPO);
    delayMicroseconds(10);
  }
  int somaTotalTensao = 0;

  for (int i = 0; i < 1000; i++)
  {
    somaTotalTensao += vetTensao[i];
  }
  TENSAO_CALCULADA = ((((analogRead(TIPO)) * 3.27) / 4096) / (valorR2 / (valorR1 + valorR2))) * tensaoOFFSET;
}

// FUNÇÃO LÊ SENSOR MQ135
void SENSOR_MQ135()
{
  MQ135.update(); // Atualiza os dados, o arduino faz a leitura da tensão no pino analógico.

  MQ135.setA(110.47);
  MQ135.setB(-2.862);
  CO = MQ135.readSensor(); // LÊ A CONCENTRAÇÃO DE CO2 MONOXIDO DE CARBONO EM PPM

  MQ135.setA(605.18);
  MQ135.setB(-3.937);
  CO2 = MQ135.readSensor(); // LÊ A CONCENTRAÇÃO DE CO DIOXIDO DE CARBONO EM PPM

  MQ135.setA(77.255);
  MQ135.setB(-3.18);
  C2H6O = MQ135.readSensor(); // LÊ A CONCENTRAÇÃO DE ALCOOL EM PPM

  MQ135.setA(44.947);
  MQ135.setB(-3.445);
  CH3 = MQ135.readSensor(); // LÊ A CONCENTRAÇÃO DE CH3 METIL TOLUENO EM PPM

  MQ135.setA(102.2);
  MQ135.setB(-2.473);
  NH3 = MQ135.readSensor(); // LÊ A CONCENTRAÇÃO DE NH4 AMONIO EM PPM

  MQ135.setA(34.668);
  MQ135.setB(-3.369);
  C3H6O = MQ135.readSensor(); // LÊ A CONCENTRAÇÃO DE ACETONA EM PPM
}


void contador()
{
  pulsos++; // Incrementa contador
}

// função que faz uma média de leituras em umA determinadA porta e retorna a média
int averageAnalogRead(int pinToRead, byte numberOfReadings) // pinToRead= pino analogico que quer obter a média, numberOfReadings= numero de vezes que deve fazer a leitura
{
  unsigned int runningValue = 0;

  for (int x = 0; x < numberOfReadings; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return (runningValue);
}

// FUÇÃO DE MAPA E RETORNA VALOR TRATADO
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) // x= valor a ser tatado, in_min= valor minimo de entrada, in_max= valor maximo a ser lido, out_mim= valor minimo da saida, out_max= valor maximo da saida
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// FUNÇÃO DIREÇÃO DO VENTO
int DIRECAO_VENTO()
{
  unsigned int POSICAO = 0;
  POSICAO = mapfloat(analogRead(SENSOR_ORIENTACAO_VENTO_ANALOGICO), 0, 4095, 0, 333);

  if (POSICAO >= 0 && POSICAO < 23)
  {
    POSICAO = 0;
  }
  else
  {
    if (POSICAO >= 22 && POSICAO < 46)
    {
      POSICAO = 1;
    }
    else
    {
      if (POSICAO >= 46 && POSICAO < 68)
      {
        POSICAO = 3;
      }
      else
      {
        if (POSICAO >= 68 && POSICAO < 91)
        {
          POSICAO = 4;
        }
        else
        {
          if (POSICAO >= 91 && POSICAO < 113)
          {
            POSICAO = 5;
          }
          else
          {
            if (POSICAO >= 113 && POSICAO < 135)
            {
              POSICAO = 6;
            }
            else
            {
              if (POSICAO >= 135 && POSICAO < 158)
              {
                POSICAO = 7;
              }
              else
              {
                if (POSICAO >= 158 && POSICAO < 180)
                {
                  POSICAO = 8;
                }
                else
                {
                  if (POSICAO >= 180 && POSICAO < 203)
                  {
                    POSICAO = 9;
                  }
                  else
                  {
                    if (POSICAO >= 203 && POSICAO < 226)
                    {
                      POSICAO = 10;
                    }
                    else
                    {
                      if (POSICAO >= 226 && POSICAO < 248)
                      {
                        POSICAO = 11;
                      }
                      else
                      {
                        if (POSICAO >= 248 && POSICAO < 270)
                        {
                          POSICAO = 12;
                        }
                        else
                        {
                          if (POSICAO >= 270 && POSICAO < 293)
                          {
                            POSICAO = 13;
                          }
                          else
                          {
                            if (POSICAO >= 293 && POSICAO < 315)
                            {
                              POSICAO = 14;
                            }
                            else
                            {
                              if (POSICAO >= 315 && POSICAO < 337)
                              {
                                POSICAO = 15;
                              }
                            }
                          }
                        }
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }

  return (POSICAO);
}

// Cria a  função para zerar os dados constantes no servidor
void RESETA_SERVER()
{
  // Carrega as informações para serem enviadas em um lote somente para o servidor
  /*ThingSpeak.setField(1, 0); // Fluxo corrente
  ThingSpeak.setField(2, 0); // Total em mililitros
  ThingSpeak.setField(3, 0); // Total em litros
  ThingSpeak.setField(4, 0); // Nivel reservatorio
  ThingSpeak.setField(5, 0); // Nivel acumulador
  ThingSpeak.setField(6, 0); // Status da Bomba
  ThingSpeak.setField(7, 0); // Status Solenoide Potavel
  ThingSpeak.setField(8, 0); // Status Solenoide Descarte
  ThingSpeak.setStatus("ZERA SERVIDOR");
  ThingSpeak.writeFields(1, apiWriteKey); // Envia o lote de informações para o servidor
  Serial.println("SERVIDOR ZERADO.");*/
}

// Cria a função para pegar a hora pelo Servidor GMT mundial.
void PEGAR_HORA()
{
  String MES_TEMP = "";
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("Falha ao obter hora");
    return;
  }
  Serial.println(&timeinfo, "%A, %d, %B, %Y, %H, %m, %S");
  Serial.println(&timeinfo, "%d/%m/%Y");
  MES_TEMP = (&timeinfo, "%m");
  MES_ATUAL = MES_TEMP.toInt();
  if (MES_ANTERIOR < MES_ATUAL)
  {
    Serial.println("RESETANDO SERVIDOR PARA O PROXIMO MES");
    // RESETA_SERVER();
    MES_ANTERIOR = MES_ATUAL;
    Serial.println("SERVIDOR FOI RESETADO.");
  }
}

// FUÇÃO ENVIA PARA SERVIDOR DADOS
void ENVIAR_PARA_SERVIDOR()
{
  Serial.println("ENVIANDO DADOS...");
  tb.sendAttributeInt("ID", ID_TEMPO); // ID DE CONTROLE DE DADOS

  tb.sendAttributeInt("TEMPO EXEC.", TEMPO_EXECUCAO); // TEMPO DE EXECUÇÃO DO MICROCONTROLADOR

  tb.sendAttributeInt("TEMPO", millis() / 1000); // TEMPO DE FUNCIONAMENTO.

  tb.sendTelemetryFloat("CO", CO); // MONOXIDO DE CARBONO

  tb.sendTelemetryFloat("CO2", CO2); // DIOXIDO DE CARBONO

  tb.sendTelemetryFloat("C2H6O", C2H6O); // ETANOL

  tb.sendTelemetryFloat("CH3", CH3); // METIL

  tb.sendTelemetryFloat("NH3", NH3); // AMONIA

  tb.sendTelemetryFloat("C3H6O", C3H6O); // ACETONA

  TENSAO(TENSAO_PLACA_SOLAR);

  tb.sendTelemetryFloat("PLACA_SOLAR", TENSAO_CALCULADA); // TENSÃO PLACA SOLAR

  TENSAO(TENSAO_BATERIA);

  tb.sendTelemetryFloat("BATERIA", TENSAO_CALCULADA); // TENSÃO PLACA SOLAR

  tb.sendTelemetryFloat("TEMPERATURA_AMBIENTE", TEMPERATURA); // TEMPERATURA AMBIENTE

  tb.sendTelemetryInt("UMIDADE_AMBIENTE", UMIDADE); // UMIDADE RELATIVA DO AR AMBIENTE

  tb.sendTelemetryFloat("PRESSÃO_AMBIENTE", PRESSAO); // PRESSÃO BAROMETRICA AMBIENTE

  tb.sendTelemetryFloat("ALTITUDE_AMBIENTE", ALTITUDE); // ALTITUDE QUE O EQUIPAMENTO ESTA

  tb.sendTelemetryInt("UV_AMBIENTE", NIVEL_UV); // INDICE UV AMBIENTE

  tb.sendTelemetryFloat("ANEMOMETRO", VELOCIDADE_VENTO); // ANEMOMETRO VELOCIDADE DOS VENTOS

  tb.sendTelemetryFloat("DIRECAO_VENTO", mapfloat(analogRead(SENSOR_ORIENTACAO_VENTO_ANALOGICO), 0, 4095, 0, 333)); // ANEMOMETRO VELOCIDADE DOS VENTOS

  tb.sendTelemetryFloat("SISNOGRAFO", SISMICO); // SISMOGRAFO INTENSIDADE DE TREMORES

  tb.sendTelemetryInt("UMIDADE_SOLO", UMIDADE_SOLO); // UMIDADE DO SOLO

  tb.sendTelemetryInt("PLUVIOMETRO", PLUVIOMETRICO); // PLUVIOMETRO

  tb.loop();
}

// FUNÇÃO ESCREVE OS DADOS NA SAIDA SERIAL
void ESCREVE_DADOS_SERIAL()
{
  Serial.print("ID: ");
  Serial.print(ID_TEMPO);
  Serial.print("  |  ");
  Serial.print("TEMPO EXEC.: ");
  Serial.print(TEMPO_EXECUCAO);
  Serial.print(" | ");
  Serial.print("TEMPO: ");
  Serial.print(millis() / 1000);
  Serial.print(" | ");
  Serial.print("PLACA SOLAR: ");
  TENSAO(TENSAO_PLACA_SOLAR);
  Serial.print(TENSAO_CALCULADA);
  Serial.print(" | ");
  Serial.print("BATERIA: ");
  TENSAO(TENSAO_BATERIA);
  Serial.print(TENSAO_CALCULADA);
  Serial.print(" | ");
  Serial.print("CO: ");
  Serial.print(CO);
  Serial.print("  |  ");
  Serial.print("CO2: ");
  Serial.print(CO2);
  Serial.print("  |  ");
  Serial.print("C2H6O: ");
  Serial.print(C2H6O);
  Serial.print("  |  ");
  Serial.print("CH3: ");
  Serial.print(CH3);
  Serial.print("  |  ");
  Serial.print("NH3: ");
  Serial.print(NH3);
  Serial.print("  |  ");
  Serial.print("C3H6O: ");
  Serial.print(C3H6O);
  Serial.print(" | ");
  Serial.print("TEMPERATURA: ");
  Serial.print(TEMPERATURA);
  Serial.print(" | ");
  Serial.print("UMIDADE: ");
  Serial.print(UMIDADE);
  Serial.print(" | ");
  Serial.print("PRESSAO: ");
  Serial.print(PRESSAO);
  Serial.print(" | ");
  Serial.print("ALTITUDE: ");
  Serial.print(ALTITUDE);
  Serial.print(" | ");
  Serial.print("NIVEL_UV: ");
  Serial.print(NIVEL_UV);
  Serial.print(" | ");
  Serial.print("ANEMOMETRO: ");
  Serial.print(VELOCIDADE_VENTO);
  Serial.print(" | ");
  Serial.print("DIRECAO_VENTO: ");
  Serial.print(mapfloat(analogRead(SENSOR_ORIENTACAO_VENTO_ANALOGICO), 0, 4095, 0, 333));
  Serial.print(" | ");
  Serial.print("SISMOGRAFO: ");
  Serial.print(SISMICO);
  Serial.print(" | ");
  Serial.print("UMIDADE_SOLO: ");
  Serial.print(UMIDADE_SOLO);
  Serial.print(" | ");
  Serial.print("PLUVIOMETRO: ");
  Serial.print(PLUVIOMETRICO);
  Serial.println(";");
}

void setup()
{
  Serial.begin(115200);
  if (Serial)
    Serial.println("SERIAL ABERTA!!!");

  CONECTAR_WIFI();

  // SETA PORTAS COMO ENTRADAS
  pinMode(TENSAO_PLACA_SOLAR, INPUT);                 // SENSOR TENSÃO PLACA SOLAR
  pinMode(TENSAO_BATERIA, INPUT);                     // SENSOR TENSÃO BATERIA
  pinMode(STATUS_CARREGADO_CARREGANDO, INPUT_PULLUP); // STATUS BATERIA CARREGANDO VINDO DA PLACA DE CARREGAMENTO
  pinMode(STATUS_CARREGADO_CARREGADA, INPUT_PULLUP);  // STATUS BATERIA CARREGADA VINDO DA PLACA DE CARREGAMENTO

  // SETA PORTAS COMO SAIDA
  pinMode(LED_LIGADO, OUTPUT); // STATUS LED LIGADO
  pinMode(LED_VIDA, OUTPUT);   // STATUS LED VIDA
  pinMode(LED_WIFI, OUTPUT);   // STATUS LED WIFI
  pinMode(LED_ERRO, OUTPUT);   // STATUS LED ERRO

  digitalWrite(LED_LIGADO, HIGH);

  pinMode(SENSOR_VELOCIDADE_VENTO_DIGITAL, INPUT); // Pino do sensor Velocidade do vento como entrada
  pinMode(SENSOR_ULTRAVIOLETA, INPUT);             // Pino do sensor Ultravioleta como entrada
  bme.begin(0x76);                                 // Endereço sensor BME280 0x77 ou 0x76
  // Aciona o contador a cada pulso
  attachInterrupt(SENSOR_VELOCIDADE_VENTO_DIGITAL, contador, RISING);
  pulsos = 0;
  rpm = 0;
  VELOCIDADE_VENTO = 0;
  timeold = 0;

  // UniqueIDdump(Serial);
  NUMERO_SERIE = "MCUDEVICE-";
  // Serial.print("UniqueID: ");
  for (size_t i = 0; i < UniqueIDsize; i++)
  {
    if (UniqueID[i] < 0x10)
      Serial.print("0");

    // Serial.print(UniqueID[i], HEX);
    NUMERO_SERIE += String(UniqueID[i], HEX);
    // Serial.print("");
  }
  Serial.println();

  NUMERO_SERIE.toUpperCase();
  Serial.println(NUMERO_SERIE);
  

  // Defina o modelo matemático para calcular a concentração de PPM e o valor das constantes.
  MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b

  /*****************************  MQ CALIBRAÇÃO ********************************************/
  MQ135.init();
  Serial.print("CALIBRANDO SENSOR MQ135 AGUARDE.");
  float calcR0 = 0;
  for (int i = 1; i <= 10; i++)
  {
    MQ135.update(); // Atualiza os dados, o arduino faz a leitura da tensão no pino analógico.
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0 / 10);
  Serial.println("  CALIBRADO!.");

  if (isinf(calcR0))
  {
    Serial.println("ATENÇÃO: CONEXÃO NÃO ENCOTRADA, R0 TENDENDO AO INFINITO. (CIRCUITO ABERDO DETECTADO)  CHECAR SE TEM ALIMENTAÇÃO 5V NO SENSOR");
    digitalWrite(LED_ERRO, HIGH);
    while (1)
      ;
  }
  if (calcR0 == 0)
  {
    Serial.println("ATENÇÃO: CONEXÃO NÃO ENCOTRADA, R0 É ZERO (PORTA ANALÓGICA EM CURTO COM GND) CHECAR SUAS LIGAÇÕES E ALIMENTAÇÃO.");
    digitalWrite(LED_ERRO, HIGH);
    while (1)
      ;
  }

  MQ135.serialDebug(false);
}

void loop()
{
  digitalWrite(LED_VIDA, HIGH);

  TEMPERATURA = bme.readTemperature();
  PRESSAO = bme.readPressure()/100;
  UMIDADE = bme.readHumidity();
  ALTITUDE = bme.readAltitude(SEALEVELPRESSURE_HPA);
  uvLevel = averageAnalogRead(SENSOR_ULTRAVIOLETA, 8);
  refLevel = 3.26;
  outputVoltagem = 3.3 / (refLevel * uvLevel);                 // Use os 3,3V de alimentação no pin referência para um melhor acuidade do valor de saida do sensor
  NIVEL_UV = mapfloat(outputVoltagem, 0.0, 3.3, 0.0, 15.0); // Convert the voltage to a UV intensity level
  // Atualiza contador a cada segundo
  if (millis() - timeold >= 1000)
  {
    detachInterrupt(SENSOR_VELOCIDADE_VENTO_DIGITAL);                     // Desabilita interrupcao durante o calculo
    rpm = (60 * 1000 / pulsos_por_volta) / (millis() - timeold) * pulsos; // RPM do sensor
    VELOCIDADE_VENTO = ((((2 * 3.6) * 3, 14) * 1.3) * rpm) / 60, 0;       // converte RPM em Km/h
    timeold = millis();
    pulsos = 0;
    attachInterrupt(SENSOR_VELOCIDADE_VENTO_DIGITAL, contador, RISING); // Habilita interrupcao
    if (WiFi.status() != WL_CONNECTED)
    {
      digitalWrite(LED_WIFI, LOW);
      CONECTAR_WIFI();
    }
    if (!tb.connected())
    {
      // CONECTANDO A SERVIDOR THINGSBOARD
      Serial.print("CONECTANDO A: ");
      Serial.print(THINGSBOARD_SERVER);
      Serial.print(" ENVIANDO TOKEN ");
      Serial.println(TOKEN);
      if (!tb.connect(THINGSBOARD_SERVER, TOKEN))
      {
        Serial.println("FALHA AO SE CONECTAR");
       digitalWrite(LED_ERRO, HIGH);
        return;
      }
      digitalWrite(LED_ERRO, LOW);
    }

    SENSOR_MQ135(); // LÊ OS GASES NO AMBIENTE

    int TEMP_EXEC = 0;
    TEMP_EXEC = millis();

    ESCREVE_DADOS_SERIAL(); // ESXREVE OS DADOS NA SAIDA SERIAL

    ENVIAR_PARA_SERVIDOR(); // ENVIA DADOS PARA O SERVIDOR

    digitalWrite(LED_LIGADO, LOW);

    delay(1000); // FREQUÊNCIA DE AMOSTRAGEM
    ID_TEMPO++;
    CONTADOR_ID++;
    TEMPO_EXECUCAO = millis() - TEMP_EXEC;
  }
}