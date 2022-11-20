/************************************************** INCLUDES **************************************************/
#include <Arduino.h>         //BIBLIOTECA COM FUNÇÕES IDE ARDUINO
#include <WiFi.h>            //BIBLIOTECA COM FUNÇÕES DO WIFI
#include <MQUnifiedsensor.h> //BIBLIOTECA COM FUNÇÕES PARA SENSORES DO TIPO MQ2, MQ3, MQ4, MQ5, MQ6, MQ7, MQ8, MQ9, MQ131, MQ135, MQ303A, MQ309A
#include <ThingsBoard.h>     //BIBLIOTECA COM FUNÇÕES SERVIDOR THINGSBOARD
#include <ArduinoUniqueID.h> //BIBLIOTECA COM FUNÇÕES PARA ADIQUIRIR NUMERO SERIAL DO MICROCONTROLADOR.
#include <Adafruit_Sensor.h> //BIBLIOTECA COM FUNÇÕES DE SENSORES DO TIPO AIDAFRUIT
#include <Adafruit_BME280.h> //BIBLIOTECA COM FUNÇÕES PARA O SENSOR BME280/BMP280

/************************************************* DEFINIÇÕES *************************************************/
#define TOKEN "nuQ84KS86qXFBrhJ21zO"           // TOKEN DO SERVIDOR THINGSBOARD
#define THINGSBOARD_SERVER "thingsboard.cloud" // ENDEREÇO DO SERVIDOR THINGSBOARD

#define LED_LIGADO 19 // LED DE STATUS LIGADO
#define LED_VIDA 18   // LED DE STATUS VIDA
#define LED_WIFI 17   // LED DE STATUS WIFI
#define LED_ERRO 16   // LED DE STATUS ERRO

#define STATUS_USANDO_BATERIA 5            // LED DE CONTROLE CIRCUITO DE CARREGAMENTO DE BATERIA
#define STATUS_PLACA_SOLAR 23              // LED DE CONTROLE CIRCUITO DA PLACA SOLAR
#define SENSOR_VELOCIDADE_VENTO_DIGITAL 13 // PORTA DE LIGAÇÃO DIGITAL ANEMOMETRO
#define SENSOR_PLUVIOMETRICO_DIGITAL 27    // PORTA DE LIGAÇÃO DIGITAL PLUVIOMETRO
#define SENSOR_SISMICO_DIGITAL 14          // PORTA DE LIGAÇÃO DIGITAL SISMOGRÁFO
#define SENSOR_UMIDADE_SOLO_DIGITAL 26     // PORTA DE LIGAÇÃO DIGITAL UMIDADE DO SOLO

#define SENSOR_SISMICO_ANALOGICO 25          // PORTA DE LIGAÇÃO ANALÓGICA SISMOGRÁFO
#define PORTA_SENSOR_MQ135 32                // PORTA ANALÓGICA ONDE O SENSOR SERÁ LIGADO AO MICROCONTROLADOR
#define SENSOR_ULTRAVIOLETA 33               // PORTA DE LIGAÇÃO ANALÓGICA ULTRAVIOLETA
#define SENSOR_UMIDADE_SOLO_ANALOGICO 34     // PORTA DE LIGAÇÃO ANALÓGICA UMIDADE SOLO
#define SENSOR_ORIENTACAO_VENTO_ANALOGICO 35 // PORTA DE LIGAÇÃO ANALÓGICA BIRUTA
#define TENSAO_BATERIA 36                    // AQUISIÇÃO DE TENSÃO BATERIA
// #define PORTA_SENSOR_MQ131 12                // PORTA ANALÓGICA ONDE O SENSOR SERÁ LIGADO AO MICROCONTROLADOR
#define TENSAO_PLACA_SOLAR 39 // AQUISIÇÃO DE TENSÃO PLACA SOLAR

#define PLACA "ESP-32"         // DEFINIR MICROCONTROLADOR PARA A BIBLIOTECA MQUNIFIEDSENSOR.H
#define RESOLUCAO_VOLTAGEM 3.3 // TENSÃO DE REFERENCIA DO MICROCONTROLADOR USADO
#define TIPO_1 "MQ-135"        // NOME DO SENSOR 1 EM USO NO CASO MQ135
// #define TIPO_2 "MQ-131"        // NOME DO SENSOR 2 EM USO NO CASO MQ131
#define ADC_RESOLUCAO_BIT 12   // 10 PARA UNO/MEGA/NANO E 12 PARA ESP32 WROOM
#define RatioMQ135CleanAir 3.6 // RS / R0 = 3.6 ppm
#define RatioMQ131CleanAir 15  // RS / R0 = 15 ppm

#define SEALEVELPRESSURE_HPA (1013.25) // DEFINIÇÃO DA PRESSÃO AO NIVEL DO MAR

/******************************************** VARIAVEIS CONSTANTES ********************************************/
const char *ssid = "FALANGE_SUPREMA";     // NOME DA REDE WIFI.
const char *password = "#kinecs#";        // SENHA DA REDE WIFI.
const char *ntpServer = "pool.ntp.org";   // SERVIDOR DE RELOGIO MUNDIAL.
const int daylightOffset_sec = -3600 * 3; // SERVIDOR DE RELOGIO MUNDIAL SEGUNDOS CONSTANTES EM 1 DIA.
const long gmtOffset_sec = 0;             // SERVIDOR DE RELOGIO MUNDIAL GMT DO BRASIL.

const int FUNDO_ESCALA_BME280 = 2147483647; // FUNDO DE ESCALA SENSOR BME-280

/********************************************** VARIAVEIS GLOBAIS **********************************************/
double CO2 = (0);                                                         // VARIAVEL PARA COLETAR CO DO SENSOR MQ135
float CO = 00.00, NH3 = 00.00, CH3 = 00.00, C2H6O = 00.00, C3H6O = 00.00; // VARIAVEL PARA COLETAR GASES DO SENSOR MQ135
// float NOX = 00.00, CL2 = 00.00, O3 = 00.00;// VARIAVEL PARA COLETAR GASES DO SENSOR MQ131
int ID_TEMPO = 0;
int DEBOUCE_SERVIDOR = 0;
float TENSAO_CALCULADA = 00.00;
float vetTensao[1000];
float valor_medio;
int CONTADOR_ID = 1;
int TEMPO_EXECUCAO = 0;
float tensaoEntrada = 0.0;     // VARIÁVEL PARA ARMAZENAR O VALOR DE TENSÃO DE ENTRADA DO SENSOR
float tensaoMedida = 0.0;      // VARIÁVEL PARA ARMAZENAR O VALOR DA TENSÃO MEDIDA PELO SENSOR
float valorR1 = 30000.0;       // VALOR DO RESISTOR 1 DO DIVISOR DE TENSÃO
float valorR2 = 7500.0;        // VALOR DO RESISTOR 2 DO DIVISOR DE TENSÃO
int leituraSensor = 0;         // VARIÁVEL PARA ARMAZENAR A LEITURA DO PINO ANALÓGICO
float MAGNITUDE_ABALO = 00.00; // VARIÁVEL PARA AMARZENAMENTO DA LEITURA DO ABALO SISMICO
float QUANTIDADE_DE_ML_POR_PULSO = 4.1, AREA_CAPTACAO_AGUA = 19855.64;
int CONTADOR_PLUVIOMETRICO = 0;
int STATUS_ENVIO = 0;
int RPM = 0,
    CONTADOR_ATUALIZACAO_SERVER = 0, MES_ATUAL, MES_ANTERIOR, TEMPO_APRESENTA = 3000, interval = 1000, NIVEL_UV = 0, BIRUTA = 0, PLUVIOMETRICO = 0, UMIDADE_SOLO = 0, UMIDADE = 0, PRESSAO = 0, ALTITUDE = 0, uvLevel;
float VELOCIDADE_VENTO = 00.00, TEMPERATURA = 00.00, FREQUENCIA_ABALO = 00.00;
char DIRECAO_VENTO_NOMECLATURA[16][4] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"}; // DIREÇÃO DO VENTO
char COND_UV[5][9] = {"BAIXO", "MODERADO", "ALTO", "ELEVADO", "EXTREMO"};
// NIVEIS DE RADIAÇÃO UV
volatile byte pulsos;
volatile byte FREQUENCIA_SISMICA;
volatile byte PULSO_PLUVIOMETRICO;
unsigned long timeold;
unsigned int pulsos_por_volta = 20; // Altere o numero de acordo com o disco encoder
int refLevel = 0;
float outputVoltagem = 00.00;
String NUMERO_SERIE = "";

MQUnifiedsensor MQ135(PLACA, RESOLUCAO_VOLTAGEM, ADC_RESOLUCAO_BIT, PORTA_SENSOR_MQ135, TIPO_1); // CONFIGURAÇÃO DO SENSOR MQ135
// MQUnifiedsensor MQ131(PLACA, RESOLUCAO_VOLTAGEM, ADC_RESOLUCAO_BIT, PORTA_SENSOR_MQ131, TIPO_2); // CONFIGURAÇÃO DO SENSOR MQ131

Adafruit_BME280 bme;
// INICIALIZANDO O CLIENTE ThingsBoard
WiFiClient espClient;
// INICIALIZAÇÃO DA INSTANCIA DO ThingsBoard
ThingsBoard tb(espClient);
// STATUS DO SINAL DE WIFI
int status = WL_IDLE_STATUS;

/****************************************************FUNÇÕES****************************************************/

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

// FUNÇÃO LER TENSÃO
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

// FUNÇÃO LE SENSOR MQ135
void SENSOR_MQ135()
{
  MQ135.update(); // ATUALZA OS DADOS, MICROCONTROLADOR FAZ A LEITURA DA TENSÃO NA PORTA ANALOGICA.

  /*
      REGRESSÃO EXPONENCIAL DOS GASES MQ-135
        GAS     |     A     |     B     |
        CO      |   110.47  |   -2.862  |
        CO2     |   605.18  |   -3.937  |
        C2H6O   |   77.225  |   -3.180  |
        CH3     |   44.947  |   -3.445  |
        NH3     |   102.20  |   -2.473  |
        C3H6O   |   34.668  |   -3.369  |
    */

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

// FUNÇÃO LE SENSOR MQ131
/*
void SENSOR_MQ131()
{
  MQ131.update(); // ATUALZA OS DADOS, MICROCONTROLADOR FAZ A LEITURA DA TENSÃO NA PORTA ANALOGICA.

   //                     REGRESSÃO EXPONENCIAL DOS GASES MQ-131
   //                       GAS   |     A     |     B     |
   //                       NOx   |  -462.43  |   -2.204  |
   //                       CL2   |   47.209  |   -1.186  |
   //                       O3    |   23.943  |   -1.11   |


  MQ131.setA(-462.43);
  MQ131.setB(-2.204);
  NOX = MQ131.readSensor(); // LÊ A CONCENTRAÇÃO DE NOX (NÚMERO DE OXIDAÇÃO) EM PPM

  MQ131.setA(-462.43);
  MQ131.setB(-2.204);
  CL2 = MQ131.readSensor(); // LÊ A CONCENTRAÇÃO DE Cl (CLORO) EM PPM

  MQ131.setA(-462.43);
  MQ131.setB(-2.204);
  O3 = MQ131.readSensor(); // LÊ A CONCENTRAÇÃO DE O3 (OZONIO) EM PPM
}
*/
void IRAM_ATTR contador()
{
  pulsos++; // Incrementa contador
}

void IRAM_ATTR CONTADOR_CHUVA()
{
  PULSO_PLUVIOMETRICO++;
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

// FUNÇÃO ENVIA PARA SERVIDOR DADOS
void ENVIAR_PARA_SERVIDOR()
{

  Serial.println("ENVIANDO DADOS...");

  tb.sendTelemetryFloat("TEMPERATURA_AMBIENTE", TEMPERATURA); // TEMPERATURA AMBIENTE

  tb.sendTelemetryInt("UMIDADE_AMBIENTE", UMIDADE); // UMIDADE RELATIVA DO AR AMBIENTE

  tb.sendTelemetryInt("PRESSÃO_AMBIENTE", PRESSAO); // PRESSÃO BAROMETRICA AMBIENTE

  tb.sendTelemetryInt("ALTITUDE_AMBIENTE", ALTITUDE); // ALTITUDE QUE O EQUIPAMENTO ESTA

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

  tb.sendTelemetryInt("UV_AMBIENTE", NIVEL_UV); // INDICE UV AMBIENTE

  tb.sendTelemetryFloat("ANEMOMETRO", VELOCIDADE_VENTO); // ANEMOMETRO VELOCIDADE DOS VENTOS

  tb.sendTelemetryFloat("DIRECAO_VENTO", mapfloat(analogRead(SENSOR_ORIENTACAO_VENTO_ANALOGICO), 0, 4095, 0, 359)); // ANEMOMETRO VELOCIDADE DOS VENTOS

  tb.sendTelemetryFloat("SISNOGRAFO", FREQUENCIA_SISMICA); // SISMOGRAFO INTENSIDADE DE TREMORES

  tb.sendTelemetryInt("UMIDADE_SOLO", UMIDADE_SOLO); // UMIDADE DO SOLO

  tb.sendTelemetryInt("PLUVIOMETRO", PLUVIOMETRICO); // PLUVIOMETRO

  tb.sendAttributeBool("LED_LIGADO", digitalRead(LED_LIGADO)); // LED STATUS

  tb.sendAttributeBool("LED_VIDA", digitalRead(LED_VIDA)); // LED STATUS

  tb.sendAttributeBool("LED_WIFI", digitalRead(LED_WIFI)); // LED STATUS

  tb.sendAttributeBool("LED_ERRO", digitalRead(LED_ERRO)); // LED STATUS

  tb.sendAttributeBool("LED_PLACA_SOLAR", digitalRead(STATUS_PLACA_SOLAR)); // LED STATUS

  tb.sendAttributeBool("LED_BATERIA", digitalRead(STATUS_USANDO_BATERIA)); // LED STATUS

  tb.loop();
  delay(50);
}

// FUNÇÃO ESCREVE OS DADOS NA SAIDA SERIAL
void ESCREVE_DADOS_SERIAL()
{
  Serial.print("ID_EQUIPAMENTO: ");
  Serial.print(NUMERO_SERIE);
  Serial.print("  |  ");
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
  // Serial.print(analogRead(TENSAO_BATERIA));
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
  Serial.print(mapfloat(analogRead(SENSOR_ORIENTACAO_VENTO_ANALOGICO), 0, 4095, 0, 359));
  Serial.print(" | ");
  Serial.print("SISMOGRAFO: ");
  Serial.print(FREQUENCIA_SISMICA);
  Serial.print(" | ");
  Serial.print("SISMOGRAFO ANALOGICO: ");
  Serial.print(MAGNITUDE_ABALO);
  Serial.print(" | ");
  Serial.print("UMIDADE_SOLO: ");
  Serial.print(UMIDADE_SOLO);
  Serial.print(" | ");
  Serial.print("PLUVIOMETRO: ");
  Serial.print(PLUVIOMETRICO);
  Serial.print(" | ");
  Serial.print("USO_PAINEL_SOLAR: ");
  Serial.print(digitalRead(STATUS_PLACA_SOLAR));
  Serial.print(" | ");
  Serial.print("USO_BATERIA: ");
  Serial.print(digitalRead(STATUS_USANDO_BATERIA));
  Serial.println(";");
}

// FUNÇÃO PARA TRATAMENTO DE ERROS
void TRATA_ERROS(int PULSOS_LED, int ESPACO_TEMPO)
{
  for (int x; x <= PULSOS_LED; x++)
  {
    digitalWrite(LED_ERRO, HIGH);
    delay(ESPACO_TEMPO);
  }
}

// FUNÇÃO PARA LEITURA E TRATAMENTO DO SENSOR PLUVIOMETRICO
void NIVEL_PLUVIOMETRICO()
{
  if (CONTADOR_PLUVIOMETRICO <= 3600)
  {
    // PLUVIOMETRICO += digitalRead(SENSOR_PLUVIOMETRICO_DIGITAL);
    PLUVIOMETRICO = PULSO_PLUVIOMETRICO * ((1000 / AREA_CAPTACAO_AGUA) / QUANTIDADE_DE_ML_POR_PULSO);
    CONTADOR_PLUVIOMETRICO++;
  }
  else
  {
    CONTADOR_PLUVIOMETRICO = 0;
  }
}

// CALIBRAÇÃO SENSORES MQ
void CALIBRACAO_SENSORES_MQ()
{
  /*****************************  MQ CALIBRAÇÃO ********************************************/
  MQ135.init();
  // MQ131.init();

  Serial.print("CALIBRANDO SENSOR MQ135 E MQ131 AGUARDE.");
  float calcR0_MQ135 = 0, calcR0_MQ131 = 0;

  for (int i = 1; i <= 10; i++)
  {
    MQ135.update(); // Atualiza os dados, o arduino faz a leitura da tensão no pino analógico.
    // MQ131.update();

    calcR0_MQ135 += MQ135.calibrate(RatioMQ135CleanAir);
    // calcR0_MQ131 += MQ131.calibrate(RatioMQ131CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0_MQ135 / 10);
  // MQ135.setR0(calcR0_MQ131 / 10);

  Serial.println("  CALIBRADO!.");

  if (isinf(calcR0_MQ135))
  {
    Serial.println("ATENÇÃO: CONEXÃO NÃO ENCOTRADA MQ135, R0 TENDENDO AO INFINITO. (CIRCUITO ABERDO DETECTADO)  CHECAR SE TEM ALIMENTAÇÃO 5V NO SENSOR");
    TRATA_ERROS(10, 500);
    delay(10000);
    ESP.restart();
    while (1)
      ;
  }
  /*
    if (calcR0_MQ131 == 0)
    {
      Serial.println("ATENÇÃO: CONEXÃO NÃO ENCOTRADA MQ131, R0 É ZERO (PORTA ANALÓGICA EM CURTO COM GND) CHECAR SUAS LIGAÇÕES E ALIMENTAÇÃO.");
      TRATA_ERROS(9, 500);
          delay(10000);
    ESP.restart();
      while (1)
        ;
    }
  */
  if (calcR0_MQ135 == 0)
  {
    Serial.println("ATENÇÃO: CONEXÃO NÃO ENCOTRADA MQ135, R0 É ZERO (PORTA ANALÓGICA EM CURTO COM GND) CHECAR SUAS LIGAÇÕES E ALIMENTAÇÃO.");
    TRATA_ERROS(8, 500);
    delay(10000);
    ESP.restart();
    while (1)
      ;
  }
  /*
    if (isinf(calcR0_MQ131))
    {
      Serial.println("ATENÇÃO: CONEXÃO NÃO ENCOTRADA MQ131, R0 TENDENDO AO INFINITO. (CIRCUITO ABERDO DETECTADO)  CHECAR SE TEM ALIMENTAÇÃO 5V NO SENSOR");
      TRATA_ERROS(7, 500);
      delay(10000);
      ESP.restart();
      while (1)
        ;
    }*/

  MQ135.serialDebug(false);
  // MQ131.serialDebug(false);
}

// FUNÇÃO LEITURA DOS DADOS MODULO 1
void MODULO_1_LEITURA()
{

  //*************BME280
  TEMPERATURA = bme.readTemperature();

  PRESSAO = bme.readPressure() / 100;

  UMIDADE = bme.readHumidity();

  ALTITUDE = bme.readAltitude(SEALEVELPRESSURE_HPA);

  if (PRESSAO == FUNDO_ESCALA_BME280 || UMIDADE == FUNDO_ESCALA_BME280 || ALTITUDE == FUNDO_ESCALA_BME280)
  {
    Serial.println("");
    Serial.println("!!!!! -----  *****  ATENCAO SENSOR BME280 MODULO 1 COM PROBLEMA!!! ***** ----- !!!!!");
    Serial.print("EQUIPAMENO SERA REINICIADO! EM 2 SEGUNDO");
    TRATA_ERROS(10, 200);
    ESP.restart();
  }
  //**************ULTRAVIOLETA
  uvLevel = analogRead(SENSOR_ULTRAVIOLETA); // averageAnalogRead(SENSOR_ULTRAVIOLETA, 8);

  Serial.println("");
  Serial.println(uvLevel);
  Serial.println("");
  
  NIVEL_UV = mapfloat(uvLevel, 0, 4096, 0, 15); // Convert the voltage to a UV intensity level

  //******************** GASES
  SENSOR_MQ135(); // LÊ OS GASES NO AMBIENTE MQ-135

  // SENSOR_MQ131(); //LÊ OS GASES NO AMBIENTE MQ-131
}

// FUNÇÃO PARA TESTAR INTERFACE
void TESTE_INTERFACE()
{
  digitalWrite(LED_LIGADO, HIGH);
  Serial.println("LED VERDE - LIGADO");
  delay(1000);
  digitalWrite(LED_LIGADO, LOW);
  delay(1000);

  digitalWrite(LED_VIDA, HIGH);
  Serial.println("LED LARANJA - VIDA");
  delay(1000);
  digitalWrite(LED_VIDA, LOW);
  delay(1000);

  digitalWrite(LED_WIFI, HIGH);
  Serial.println("LED AZUL - WIFI");
  delay(1000);
  digitalWrite(LED_WIFI, LOW);
  delay(1000);

  digitalWrite(LED_ERRO, HIGH);
  Serial.println("LED VERMELHO - ERRO");
  delay(1000);
  digitalWrite(LED_ERRO, LOW);
  delay(1000);

  digitalWrite(LED_LIGADO, HIGH);
  digitalWrite(LED_VIDA, HIGH);
  digitalWrite(LED_WIFI, HIGH);
  digitalWrite(LED_ERRO, HIGH);
  delay(5000);
  digitalWrite(LED_LIGADO, LOW);
  digitalWrite(LED_VIDA, LOW);
  digitalWrite(LED_WIFI, LOW);
  digitalWrite(LED_ERRO, LOW);
  delay(1000);
}

void setup()
{
  // SETA PORTAS COMO SAIDA
  pinMode(LED_LIGADO, OUTPUT); // STATUS LED LIGADO

  digitalWrite(LED_LIGADO, HIGH);

  pinMode(LED_VIDA, OUTPUT); // STATUS LED VIDA
  pinMode(LED_WIFI, OUTPUT); // STATUS LED WIFI
  pinMode(LED_ERRO, OUTPUT); // STATUS LED ERRO

  digitalWrite(LED_VIDA, LOW);
  digitalWrite(LED_WIFI, LOW);
  digitalWrite(LED_ERRO, LOW);

  // SETA PORTAS COMO ENTRADAS
  pinMode(TENSAO_PLACA_SOLAR, INPUT);           // SENSOR TENSÃO PLACA SOLAR
  pinMode(TENSAO_BATERIA, INPUT);               // SENSOR TENSÃO BATERIA
  pinMode(STATUS_PLACA_SOLAR, INPUT_PULLUP);    // STATUS BATERIA CARREGANDO VINDO DA PLACA DE CARREGAMENTO
  pinMode(STATUS_USANDO_BATERIA, INPUT_PULLUP); // STATUS BATERIA CARREGADA VINDO DA PLACA DE CARREGAMENTO

  pinMode(SENSOR_VELOCIDADE_VENTO_DIGITAL, INPUT); // Pino do sensor Velocidade do vento como entrada
  // pinMode(SENSOR_SISMICO_DIGITAL, INPUT);
  // pinMode(SENSOR_SISMICO_ANALOGICO,INPUT);
  pinMode(SENSOR_ULTRAVIOLETA, INPUT); // Pino do sensor Ultravioleta como entrada
  pinMode(SENSOR_PLUVIOMETRICO_DIGITAL, INPUT);

  Serial.begin(115200);
  if (Serial)
    Serial.println("SERIAL ABERTA!!!");

  TESTE_INTERFACE();

  // Defina o modelo matemático para calcular a concentração de PPM e o valor das constantes.
  MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b

  // MQ131.setRegressionMethod(1); //_PPM =  a*ratio^b

  CALIBRACAO_SENSORES_MQ();

  CONECTAR_WIFI();

  bme.begin(0x76); // Endereço sensor BME280 0x77 ou 0x76
  // Aciona o contador a cada pulso
  attachInterrupt(SENSOR_VELOCIDADE_VENTO_DIGITAL, contador, RISING);
  // attachInterrupt(SENSOR_SISMICO_DIGITAL, ABALO_SISMICO, CHANGE);
  attachInterrupt(SENSOR_PLUVIOMETRICO_DIGITAL, CONTADOR_CHUVA, CHANGE);
  pulsos = 0;
  RPM = 0;
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
}

void loop()
{
  digitalWrite(LED_LIGADO, HIGH);
  int TEMP_EXEC = 0;
  TEMP_EXEC = millis();
  digitalWrite(LED_VIDA, HIGH);

  // Atualiza contador a cada segundo
  if (millis() - timeold >= 1000)
  {
    detachInterrupt(SENSOR_VELOCIDADE_VENTO_DIGITAL);                     // Desabilita interrupcao durante o calculo
    RPM = (60 * 1000 / pulsos_por_volta) / (millis() - timeold) * pulsos; // RPM do sensor
    VELOCIDADE_VENTO = ((((2 * 3.6) * 3, 14) * 1.3) * RPM) / 60, 0;       // converte RPM em Km/h

    timeold = millis();
    pulsos = 0;
    attachInterrupt(SENSOR_VELOCIDADE_VENTO_DIGITAL, contador, RISING); // Habilita interrupcao anemometro
    if (WiFi.status() != WL_CONNECTED)
    {
      digitalWrite(LED_WIFI, LOW);
      CONECTAR_WIFI();
    }
    if (!tb.connected())
    {
      digitalWrite(LED_WIFI, HIGH);

      // CONECTANDO A SERVIDOR THINGSBOARD
      Serial.print("CONECTANDO A: ");
      Serial.print(THINGSBOARD_SERVER);
      Serial.print(" ENVIANDO TOKEN ");
      Serial.println(TOKEN);
      if (!tb.connect(THINGSBOARD_SERVER, TOKEN))
      {
        Serial.println("FALHA AO SE CONECTAR");
        TRATA_ERROS(20, 50);
        return;
      }
      digitalWrite(LED_ERRO, LOW);
    }

    MODULO_1_LEITURA();

    FREQUENCIA_ABALO += mapfloat(1.15, 0.0, 3.3, 0.0, 10.0);

    UMIDADE_SOLO = mapfloat(analogRead(SENSOR_UMIDADE_SOLO_ANALOGICO), 0, 4095, 100, 0);

    NIVEL_PLUVIOMETRICO();
    if (STATUS_ENVIO == 2)
    {
      ENVIAR_PARA_SERVIDOR(); // ENVIA DADOS PARA O SERVIDOR
      ESCREVE_DADOS_SERIAL(); // ESXREVE OS DADOS NA SAIDA SERIAL
      STATUS_ENVIO = 0;
    }
    else
    {
      ESCREVE_DADOS_SERIAL(); // ESXREVE OS DADOS NA SAIDA SERIAL
      STATUS_ENVIO++;
    }

    delay(1000); // FREQUÊNCIA DE AMOSTRAGEM

    ID_TEMPO++;

    CONTADOR_ID++;

    TEMPO_EXECUCAO = millis() - TEMP_EXEC;

    FREQUENCIA_ABALO = 0;
    digitalWrite(LED_VIDA, LOW);
    delay(1000);
  }
}