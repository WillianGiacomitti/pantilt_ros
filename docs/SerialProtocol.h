#pragma once
#include <Arduino.h>
#include <string.h>
#include <stdint.h>

// =====================================================================
// Protocolo de frame binário para comunicação ESP32 <-> Host (ROS 2)
//
// Formato do frame:
//   [SYNC1][SYNC2][TYPE][LEN][PAYLOAD...(LEN bytes)][CRC8]
//
//   SYNC1 = 0xA5, SYNC2 = 0x5A  -> permitem resincronizar após ruído/perda
//   TYPE  -> ver enum SerialMsgType
//   LEN   -> tamanho do payload em bytes (0-32)
//   CRC8  -> calculado sobre [TYPE][LEN][PAYLOAD]
//
// Todos os floats são little-endian (padrão em ESP32 e x86/ARM Linux).
// =====================================================================

#define SB_SYNC1 0xA5
#define SB_SYNC2 0x5A
#define SB_MAX_PAYLOAD 32

enum SerialMsgType : uint8_t {
  MSG_TELEMETRY    = 0x01, // ESP32 -> Host   (TelemetryPayload)
  MSG_CMD_POS      = 0x02, // Host  -> ESP32  (CmdPosPayload)
  MSG_CMD_VEL      = 0x03, // Host  -> ESP32  (CmdVelPayload)
  MSG_SET_ZERO_REQ = 0x04, // Host  -> ESP32  (sem payload)
  MSG_SET_ZERO_ACK = 0x05, // ESP32 -> Host   (SetZeroAckPayload)
  MSG_HEARTBEAT    = 0x06, // Bidirecional    (sem payload)
  MSG_ERROR        = 0x07, // ESP32 -> Host   (ErrorPayload)
  MSG_BOOT_INFO    = 0x08, // ESP32 -> Host   (BootInfoPayload) - enviado 1x ao ligar/reconectar
  MSG_HOME_REQ     = 0x09, // Host  -> ESP32  (HomeReqPayload) - auto home pelo batente negativo
  MSG_HOME_ACK     = 0x0A, // ESP32 -> Host   (HomeAckPayload) - enviado ao FIM do home (até ~1 min)
};

// Eixo do MSG_HOME_REQ / MSG_HOME_ACK. "Ambos" faz o PAN e depois o TILT (um eixo por vez).
#define HOME_EIXO_PAN   0
#define HOME_EIXO_TILT  1
#define HOME_EIXO_AMBOS 2

// ---------------------- Códigos de erro (ErrorPayload.code) ----------------------
// Cada número representa um tipo de problema diferente, para facilitar log/estatística
// no host sem precisar mandar strings pela serial.
#define ERR_I2C_TIMEOUT_PAN     1  // leitura do encoder PAN falhou (timeout/NACK no mux ou sensor)
#define ERR_I2C_TIMEOUT_TILT    2  // leitura do encoder TILT falhou
#define ERR_I2C_BUS_RECOVERED   3  // barramento I2C estava travado; rotina de recuperação foi acionada
#define ERR_FAILSAFE_TRIGGERED  4  // motores parados por perda de comunicação com o host
#define ERR_CMD_POS_INVALID     5  // frame MSG_CMD_POS recebido com payload de tamanho inválido
#define ERR_CMD_VEL_INVALID     6  // frame MSG_CMD_VEL recebido com payload de tamanho inválido
#define ERR_LOW_HEAP            7  // memória heap livre abaixo do limite de segurança
#define ERR_ENCODER_IMA_PAN     8  // AS5600 do PAN: ímã ausente (MD=0), fraco (ML) ou forte (MH)
#define ERR_ENCODER_IMA_TILT    9  // AS5600 do TILT: idem
#define ERR_PASSOS_PERDIDOS_PAN   10 // encoder e contagem de passos do PAN divergem (passo perdido)
#define ERR_PASSOS_PERDIDOS_TILT  11 // idem no TILT; persiste até o próximo set_zero
#define ERR_DRIVER_UART_PAN     12 // TMC2209 do PAN não responde pela UART ou foi resetado (config reaplicada)
#define ERR_DRIVER_UART_TILT    13 // idem no TILT
#define ERR_DRIVER_TEMP_PAN     14 // TMC2209 do PAN com pré-aviso (otpw) ou desligamento (ot) por temperatura
#define ERR_DRIVER_TEMP_TILT    15 // idem no TILT
#define ERR_LIMITE_PAN          16 // comando do PAN recortado/parado pelo limite de software do firmware
#define ERR_LIMITE_TILT         17 // idem no TILT
#define ERR_HOME_FALHOU         18 // auto home falhou (batente não encontrado, abortado ou ajuste incoerente)

// Erros "persistentes" (encoder, passos, driver, limite) são enviados na borda de subida
// e depois a cada ERRO_REPETICAO_MS enquanto a condição durar.
#define ERRO_REPETICAO_MS 2000

#pragma pack(push, 1)
struct TelemetryPayload {
  float pan_pos_rad;
  float pan_vel_rad;
  float tilt_pos_rad;
  float tilt_vel_rad;
};

struct CmdPosPayload {
  float pan_rad;
  float tilt_rad;
};

struct CmdVelPayload {
  float pan_rad_s;
  float tilt_rad_s;
};

struct SetZeroAckPayload {
  uint8_t success;
};

struct ErrorPayload {
  uint8_t code; // ver defines ERR_* acima
};

struct BootInfoPayload {
  uint8_t reset_reason;  // valor bruto de esp_reset_reason() (enum esp_reset_reason_t do ESP-IDF)
  uint32_t free_heap;    // bytes livres de heap no momento do boot
};

struct HomeReqPayload {
  uint8_t eixo;          // HOME_EIXO_PAN, HOME_EIXO_TILT ou HOME_EIXO_AMBOS
};

struct HomeAckPayload {
  uint8_t eixo;          // o mesmo do pedido
  uint8_t sucesso;       // 1 = todos os eixos pedidos referenciados; 0 = falhou (ver ERR_HOME_FALHOU)
};
#pragma pack(pop)

uint8_t sb_crc8(const uint8_t* data, size_t len);

class SerialFramer {
public:
  // Callback chamado quando um frame válido (CRC ok) é recebido.
  using FrameCallback = void (*)(uint8_t type, const uint8_t* payload, uint8_t len);

  explicit SerialFramer(FrameCallback cb) : onFrame(cb) {}

  // Alimenta o parser byte a byte. NÃO bloqueia — chamar em loop enquanto
  // Serial.available() > 0. Reconstrói o estado sozinho se receber lixo.
  void feed(uint8_t b);

  // Monta e escreve um frame completo diretamente em um Stream (ex: Serial).
  static bool sendFrame(Stream& out, uint8_t type, const uint8_t* payload, uint8_t len);

private:
  enum State { WAIT_SYNC1, WAIT_SYNC2, WAIT_TYPE, WAIT_LEN, WAIT_PAYLOAD, WAIT_CRC };

  State state = WAIT_SYNC1;
  uint8_t type = 0;
  uint8_t len = 0;
  uint8_t payload[SB_MAX_PAYLOAD];
  uint8_t payloadIdx = 0;
  FrameCallback onFrame;
};