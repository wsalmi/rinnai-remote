#include <unity.h>
#include <stdint.h>
#include <stddef.h>
#include "rinnai_uart_decoder.h"
#include "rinnai_rmt_decoder.h"
#include "rinnai_tristate_buttons.h"

void setUp(void) {}
void tearDown(void) {}

static void feedFrame(RinnaiUartDecoder &dec, const uint8_t *data, size_t len, uint32_t &tMs) {
    for (size_t i = 0; i < len; i++) {
        dec.feedByte(data[i], tMs);
        tMs += 3;
    }
}

void test_real_hardware_40_idle_and_flame_and_41_flame(void) {
    // 40°C sem chama (71 bytes)
    const uint8_t f40_idle[] = {
        0x15,0x45,0x15,0x51,0x14,0x55,0x11,0x45,0x44,0x51,0x55,0x44,0x45,0x44,0x15,0x51,0x35,0x00,
        0x45,0x15,0x55,0x55,0x15,0x51,0x11,0x45,0x55,0x51,0x54,0x51,0x44,0x55,0x44,0x11,0x00,0x00,
        0x51,0x15,0x55,0x55,0x55,0x55,0x55,0x45,0x54,0x55,0x45,0x44,0x51,0x15,0x14,0x00,0x00,
        0x55,0x45,0x15,0x55,0x14,0x55,0x11,0x54,0x51,0x14,0x11,0x45,0x44,0x54,0x55,0x54,0x03,0x00
    };
    // 40°C com chama acesa (73 bytes)
    const uint8_t f40_flame[] = {
        0x15,0x45,0x15,0x51,0x14,0x55,0x11,0x45,0x44,0x51,0x55,0x44,0x45,0x44,0x15,0x51,0x35,0x00,
        0x45,0x15,0x55,0x55,0x45,0x55,0x44,0x51,0x14,0x11,0x45,0x15,0x44,0x11,0x45,0x14,0x11,0x00,0x00,
        0x51,0x15,0x55,0x55,0x55,0x55,0x55,0x45,0x54,0x55,0x45,0x44,0x51,0x15,0x14,0x00,0x00,
        0x55,0x14,0x44,0x51,0x15,0x51,0x15,0x51,0x15,0x44,0x15,0x44,0x45,0x15,0x44,0x45,0x44,0x03,0x00
    };
    // 41°C com chama acesa (72 bytes)
    const uint8_t f41_flame[] = {
        0x15,0x45,0x15,0x51,0x14,0x55,0x11,0x45,0x44,0x51,0x55,0x44,0x45,0x44,0x15,0x51,0x35,0x00,
        0x45,0x15,0x55,0x55,0x45,0x51,0x11,0x45,0x51,0x11,0x45,0x15,0x14,0x11,0x51,0x14,0x11,0x00,0x00,
        0x51,0x15,0x55,0x55,0x55,0x55,0x55,0x45,0x54,0x55,0x55,0x15,0x45,0x44,0x34,0x00,
        0x55,0x14,0x44,0x51,0x15,0x51,0x54,0x51,0x54,0x44,0x15,0x44,0x54,0x44,0x44,0x44,0x55,0x03,0x00
    };

    TEST_ASSERT_FALSE(RinnaiFrameParser::detectFlame(f40_idle, sizeof(f40_idle)));
    TEST_ASSERT_EQUAL_INT(40, RinnaiFrameParser::calculateTemperatureVariable(f40_idle, sizeof(f40_idle)));

    TEST_ASSERT_TRUE(RinnaiFrameParser::detectFlame(f40_flame, sizeof(f40_flame)));
    TEST_ASSERT_EQUAL_INT(40, RinnaiFrameParser::calculateTemperatureVariable(f40_flame, sizeof(f40_flame)));

    TEST_ASSERT_TRUE(RinnaiFrameParser::detectFlame(f41_flame, sizeof(f41_flame)));
    TEST_ASSERT_EQUAL_INT(41, RinnaiFrameParser::calculateTemperatureVariable(f41_flame, sizeof(f41_flame)));

    // 37°C com chama acesa (72 bytes)
    const uint8_t f37_flame[] = {
        0x15,0x45,0x15,0x51,0x14,0x55,0x11,0x45,0x44,0x51,0x55,0x44,0x45,0x44,0x15,0x51,0x35,0x00,
        0x45,0x15,0x55,0x55,0x54,0x44,0x45,0x45,0x51,0x11,0x45,0x15,0x45,0x55,0x54,0xC5,0x00,
        0x51,0x15,0x55,0x55,0x45,0x51,0x55,0x55,0x44,0x55,0x55,0x14,0x54,0x55,0x14,0x00,0x00,
        0x55,0x14,0x44,0x51,0x15,0x45,0x54,0x45,0x54,0x44,0x44,0x44,0x14,0x45,0x44,0x14,0x11,0x51,0x00,0x00
    };
    TEST_ASSERT_TRUE(RinnaiFrameParser::detectFlame(f37_flame, sizeof(f37_flame)));
    TEST_ASSERT_EQUAL_INT(37, RinnaiFrameParser::calculateTemperatureVariable(f37_flame, sizeof(f37_flame)));

    // Simula stream contínuo no RinnaiUartDecoder (incluindo debounce)
    RinnaiUartDecoder dec;
    uint32_t t = 1000;
    const uint8_t pre[] = {0x15, 0x45, 0x15, 0x51};

    feedFrame(dec, f40_idle, sizeof(f40_idle), t);
    t += 30;
    feedFrame(dec, pre, 4, t); // dispara processamento do quadro anterior
    TEST_ASSERT_TRUE(dec.state().valid);
    TEST_ASSERT_FALSE(dec.state().flameActive);
    TEST_ASSERT_EQUAL_UINT8(40, dec.state().temperature);

    // Muda para 40°C com chama acesa
    feedFrame(dec, f40_flame + 4, sizeof(f40_flame) - 4, t);
    t += 30;
    feedFrame(dec, pre, 4, t);
    TEST_ASSERT_TRUE(dec.state().flameActive);
    TEST_ASSERT_EQUAL_UINT8(40, dec.state().temperature);

    // Muda para 41°C com chama acesa: precisa de 2 leituras consecutivas (debounce)
    feedFrame(dec, f41_flame + 4, sizeof(f41_flame) - 4, t);
    t += 30;
    feedFrame(dec, pre, 4, t);
    TEST_ASSERT_EQUAL_UINT8(40, dec.state().temperature); // 1a leitura ainda segura 40

    feedFrame(dec, f41_flame + 4, sizeof(f41_flame) - 4, t);
    t += 30;
    feedFrame(dec, pre, 4, t);
    TEST_ASSERT_TRUE(dec.state().flameActive);
    TEST_ASSERT_EQUAL_UINT8(41, dec.state().temperature); // 2a leitura confirma 41
}

void test_legacy_63_byte_frames(void) {
    // 35°C, 40°C, 46°C de captured_temps.json (63 bytes)
    const uint8_t f35[] = {
        0x15,0x45,0x15,0x51,0x14,0x55,0x11,0x45,0x44,0x51,0x55,0x44,0x45,0x44,0x15,0x51,
        0x35,0x45,0x15,0x55,0x55,0x55,0x55,0x54,0x55,0x55,0x54,0x51,0x11,0x15,0x44,0x14,
        0x0c,0x51,0x15,0x55,0x55,0x55,0x55,0x55,0x45,0x55,0x55,0x55,0x54,0x55,0x44,0x55,
        0x45,0x15,0x55,0x54,0x44,0x45,0x55,0x55,0x55,0x55,0x55,0x14,0x44,0x54,0x15
    };
    const uint8_t f40[] = {
        0x15,0x45,0x15,0x51,0x14,0x55,0x11,0x45,0x44,0x51,0x55,0x44,0x45,0x44,0x15,0x51,
        0x35,0x45,0x15,0x55,0x55,0x55,0x55,0x54,0x55,0x55,0x54,0x51,0x11,0x15,0x44,0x14,
        0x0c,0x51,0x15,0x55,0x55,0x55,0x55,0x55,0x45,0x55,0x55,0x55,0x54,0x55,0x44,0x55,
        0x45,0x15,0x55,0x14,0x55,0x11,0x55,0x55,0x55,0x55,0x44,0x54,0x54,0x14,0x0c
    };
    const uint8_t f46[] = {
        0x15,0x45,0x15,0x51,0x14,0x55,0x11,0x45,0x44,0x51,0x55,0x44,0x45,0x44,0x15,0x51,
        0x35,0x45,0x15,0x55,0x55,0x55,0x55,0x54,0x55,0x55,0x54,0x51,0x11,0x15,0x44,0x14,
        0x0c,0x51,0x15,0x55,0x55,0x55,0x55,0x55,0x45,0x55,0x55,0x55,0x54,0x55,0x44,0x55,
        0x45,0x15,0x55,0x14,0x44,0x15,0x54,0x55,0x55,0x55,0x15,0x44,0x45,0x45,0xc4
    };
    TEST_ASSERT_EQUAL_INT(35, RinnaiFrameParser::calculateTemperature(f35));
    TEST_ASSERT_EQUAL_INT(40, RinnaiFrameParser::calculateTemperature(f40));
    TEST_ASSERT_EQUAL_INT(46, RinnaiFrameParser::calculateTemperature(f46));
}

static void encodePktToPulses(const uint8_t pkt8[8], RinnaiRmtPulse out[129]) {
    size_t idx = 0;
    for (size_t byteIdx = 0; byteIdx < 8; byteIdx++) {
        for (int bit = 7; bit >= 0; bit--) {
            bool isOne = ((pkt8[byteIdx] >> bit) & 0x01) != 0;
            out[idx++] = {369, 1};
            out[idx++] = {(uint16_t)(isOne ? 1108 : 369), 0};
        }
    }
    out[idx++] = {550, 1};
}

void test_rmt_crc16_and_real_packets(void) {
    const uint8_t realPkts[8][8] = {
        {0x13, 0x14, 0xA1, 0x67, 0x90, 0x3C, 0xF0, 0xA8}, // B0
        {0x31, 0x00, 0x6E, 0x19, 0x01, 0x21, 0x84, 0x99}, // B1 idle
        {0x31, 0x00, 0x70, 0x19, 0x29, 0x21, 0x02, 0xEC}, // B1 flame
        {0x41, 0x00, 0x00, 0x00, 0x1C, 0x00, 0xBA, 0x8D}, // B2
        {0x09, 0x08, 0x28, 0x1C, 0x3A, 0x77, 0x9B, 0x49}, // B3 40°C idle
        {0x0D, 0xE8, 0x28, 0x28, 0x3C, 0x6F, 0x0F, 0x6E}, // B3 40°C flame
        {0x0D, 0xE8, 0x23, 0x23, 0x3C, 0x67, 0xC7, 0xC4}, // B3 35°C flame
        {0x09, 0x48, 0x26, 0x26, 0x3B, 0x67, 0x58, 0x34}, // B3 38°C flame
    };
    for (size_t i = 0; i < 8; i++) {
        uint16_t expected = ((uint16_t)realPkts[i][6] << 8) | realPkts[i][7];
        TEST_ASSERT_EQUAL_HEX16(expected, RinnaiRmtDecoder::crc16Rinnai(realPkts[i], 6));
    }

    RinnaiRmtDecoder rmt;
    RinnaiRmtPulse pulses[129];
    uint32_t nowMs = 1000;

    // B3 40°C idle
    encodePktToPulses(realPkts[4], pulses);
    TEST_ASSERT_TRUE(rmt.feedPulses(pulses, 129, nowMs));
    TEST_ASSERT_TRUE(rmt.state().valid);
    TEST_ASSERT_FALSE(rmt.state().flameActive);
    TEST_ASSERT_EQUAL_UINT8(40, rmt.state().temperature);
    TEST_ASSERT_EQUAL_UINT32(1, rmt.crcOkCount());
    TEST_ASSERT_EQUAL_UINT32(0, rmt.crcErrCount());

    // 1-bit corruption must fail CRC and NOT alter state
    uint8_t badPkt[8] = {0x0D, 0xE8, 0x29, 0x28, 0x3C, 0x6F, 0x0F, 0x6E};
    encodePktToPulses(badPkt, pulses);
    nowMs += 200;
    TEST_ASSERT_FALSE(rmt.feedPulses(pulses, 129, nowMs));
    TEST_ASSERT_FALSE(rmt.state().flameActive);
    TEST_ASSERT_EQUAL_UINT8(40, rmt.state().temperature);
    TEST_ASSERT_EQUAL_UINT32(1, rmt.crcErrCount());

    // B3 40°C flame
    encodePktToPulses(realPkts[5], pulses);
    nowMs += 200;
    TEST_ASSERT_TRUE(rmt.feedPulses(pulses, 129, nowMs));
    TEST_ASSERT_TRUE(rmt.state().flameActive);
    TEST_ASSERT_EQUAL_UINT8(40, rmt.state().temperature);

    // Real B3 35°C flame ON (2 readings for debounce)
    encodePktToPulses(realPkts[6], pulses);
    nowMs += 200;
    TEST_ASSERT_TRUE(rmt.feedPulses(pulses, 129, nowMs));
    nowMs += 200;
    TEST_ASSERT_TRUE(rmt.feedPulses(pulses, 129, nowMs));
    TEST_ASSERT_TRUE(rmt.state().flameActive);
    TEST_ASSERT_EQUAL_UINT8(35, rmt.state().temperature);

    // Real B3 38°C flame ON (09 48 variant, 2 readings for debounce)
    encodePktToPulses(realPkts[7], pulses);
    nowMs += 200;
    TEST_ASSERT_TRUE(rmt.feedPulses(pulses, 129, nowMs));
    nowMs += 200;
    TEST_ASSERT_TRUE(rmt.feedPulses(pulses, 129, nowMs));
    TEST_ASSERT_TRUE(rmt.state().flameActive);
    TEST_ASSERT_EQUAL_UINT8(38, rmt.state().temperature);

    // Every temperature 35..43°C (0x23..0x2B) in both IDLE and FLAME (2 readings each for debounce)
    for (uint8_t temp = 35; temp <= 43; temp++) {
        for (int fl = 0; fl <= 1; fl++) {
            uint8_t pkt[8] = {
                (uint8_t)(fl ? 0x0D : 0x09),
                (uint8_t)(fl ? 0xE8 : 0x08),
                temp,
                0x1C, 0x3A, 0x77, 0x00, 0x00
            };
            uint16_t crc = RinnaiRmtDecoder::crc16Rinnai(pkt, 6);
            pkt[6] = (uint8_t)(crc >> 8);
            pkt[7] = (uint8_t)(crc & 0xFF);
            encodePktToPulses(pkt, pulses);
            nowMs += 200;
            TEST_ASSERT_TRUE(rmt.feedPulses(pulses, 129, nowMs));
            nowMs += 200;
            TEST_ASSERT_TRUE(rmt.feedPulses(pulses, 129, nowMs));
            TEST_ASSERT_EQUAL_UINT8(temp, rmt.state().temperature);
            TEST_ASSERT_EQUAL(fl != 0, rmt.state().flameActive);
        }
    }

    // Clamping limits 35..43°C
    TEST_ASSERT_EQUAL_UINT8(35, RinnaiClosedLoopController::clampTargetTemp(30));
    TEST_ASSERT_EQUAL_UINT8(43, RinnaiClosedLoopController::clampTargetTemp(43));
    TEST_ASSERT_EQUAL_UINT8(43, RinnaiClosedLoopController::clampTargetTemp(48));
}

struct MockTriStateHardware : public ITriStateHardware {
    int releaseCount = 0;
    int upCount = 0;
    int downCount = 0;
    void release() override { releaseCount++; }
    void driveUp() override { upCount++; }
    void driveDown() override { downCount++; }
};

struct ControllerTestCtx {
    int syncCount = 0;
    uint8_t lastSynced = 0;
    int clickCount = 0;
    RinnaiClickDir lastDir = RinnaiClickDir::NONE;
};

static void testSyncCb(uint8_t syncedTemp, void *ctx) {
    ControllerTestCtx *c = static_cast<ControllerTestCtx *>(ctx);
    c->syncCount++;
    c->lastSynced = syncedTemp;
}

static void testClickCb(RinnaiClickDir dir, void *ctx) {
    ControllerTestCtx *c = static_cast<ControllerTestCtx *>(ctx);
    c->clickCount++;
    c->lastDir = dir;
}

static RinnaiTelemetryState makeTelemetry(uint8_t temp, bool flame, uint32_t pktCount) {
    RinnaiTelemetryState t;
    t.valid = true;
    t.powerOn = true;
    t.flameActive = flame;
    t.temperature = temp;
    t.packetCount = pktCount;
    t.tempPacketCount = pktCount;
    t.lastPacketMs = 1000;
    return t;
}

void test_closed_loop_controller_flame_sync_and_antiloop(void) {
    MockTriStateHardware hw;
    RinnaiClosedLoopController ctrl(hw);
    ControllerTestCtx ctx;
    ctrl.setCallbacks(testSyncCb, testClickCb, &ctx);

    // 1. Primeira leitura válida dispara sincronismo imediato com o setpoint real
    uint32_t nowMs = 1000;
    RinnaiTelemetryState tel = makeTelemetry(38, false, 1);
    tel.lastPacketMs = nowMs;
    ctrl.update(tel, nowMs);
    TEST_ASSERT_EQUAL_INT(1, ctx.syncCount);
    TEST_ASSERT_EQUAL_UINT8(38, ctx.lastSynced);
    TEST_ASSERT_EQUAL_UINT8(38, ctrl.targetTemp());

    // 2. Mudança física no painel durante chama acesa (35°C -> 38°C) sincroniza imediatamente sem cliques
    tel = makeTelemetry(35, true, 2);
    nowMs += 300;
    tel.lastPacketMs = nowMs;
    ctrl.update(tel, nowMs);
    TEST_ASSERT_EQUAL_INT(2, ctx.syncCount);
    TEST_ASSERT_EQUAL_UINT8(35, ctx.lastSynced);
    TEST_ASSERT_EQUAL_INT(0, hw.upCount);
    TEST_ASSERT_EQUAL_INT(0, hw.downCount);

    tel = makeTelemetry(38, true, 3);
    nowMs += 300;
    tel.lastPacketMs = nowMs;
    ctrl.update(tel, nowMs);
    TEST_ASSERT_EQUAL_INT(3, ctx.syncCount);
    TEST_ASSERT_EQUAL_UINT8(38, ctx.lastSynced);
    TEST_ASSERT_EQUAL_INT(0, hw.upCount);
    TEST_ASSERT_EQUAL_INT(0, hw.downCount);

    // 3. Comando remoto durante chama acesa (38°C -> 40°C) atua em malha fechada e fecha ao atingir 40°C
    ctrl.setTargetTemperature(40, tel, nowMs);
    TEST_ASSERT_TRUE(ctrl.hasPendingTarget());

    ctrl.update(tel, nowMs); // IDLE -> PRESSING UP #1
    TEST_ASSERT_EQUAL_INT(1, hw.upCount);
    nowMs += RinnaiClosedLoopController::PRESS_DURATION_MS + 5;
    ctrl.update(tel, nowMs); // PRESSING -> WAITING_SETTLE
    nowMs += RinnaiClosedLoopController::SETTLE_MIN_MS + 10;
    tel = makeTelemetry(39, true, 4);
    tel.lastPacketMs = nowMs;
    ctrl.update(tel, nowMs); // 39°C confirmado, volta para IDLE
    TEST_ASSERT_TRUE(ctrl.hasPendingTarget());

    nowMs += 10;
    ctrl.update(tel, nowMs); // IDLE -> PRESSING UP #2
    TEST_ASSERT_EQUAL_INT(2, hw.upCount);
    nowMs += RinnaiClosedLoopController::PRESS_DURATION_MS + 5;
    ctrl.update(tel, nowMs); // PRESSING -> WAITING_SETTLE
    nowMs += RinnaiClosedLoopController::SETTLE_MIN_MS + 10;
    tel = makeTelemetry(40, true, 5);
    tel.lastPacketMs = nowMs;
    ctrl.update(tel, nowMs); // 40°C confirmado -> alvo atingido
    TEST_ASSERT_FALSE(ctrl.hasPendingTarget());

    // 4. Trava anti-loop durante chama bloqueada desde o 1o clique (ex: 40°C -> 43°C):
    //    1o clique e tratado como wake-up reativo (painel pode estar Idle);
    //    2 cliques seguintes sem mudanca acionam a trava e revertem para 40°C.
    int syncBefore = ctx.syncCount;
    ctrl.setTargetTemperature(43, tel, nowMs);
    TEST_ASSERT_TRUE(ctrl.hasPendingTarget());

    // Tentativa 1: wake-up reativo
    nowMs += 10;
    ctrl.update(tel, nowMs); // PRESSING
    nowMs += RinnaiClosedLoopController::PRESS_DURATION_MS + 5;
    ctrl.update(tel, nowMs); // WAITING_SETTLE
    nowMs += RinnaiClosedLoopController::SETTLE_MAX_MS + 5;
    ctrl.update(tel, nowMs); // wakeClickUsed = true, stalledClicks = 0
    TEST_ASSERT_TRUE(ctrl.hasPendingTarget());
    TEST_ASSERT_TRUE(ctrl.wakeClickUsed());
    TEST_ASSERT_EQUAL_UINT8(0, ctrl.stalledClicks());

    // Tentativa 2: stall 1/2
    nowMs += 10;
    ctrl.update(tel, nowMs); // PRESSING
    nowMs += RinnaiClosedLoopController::PRESS_DURATION_MS + 5;
    ctrl.update(tel, nowMs); // WAITING_SETTLE
    nowMs += RinnaiClosedLoopController::SETTLE_MAX_MS + 5;
    ctrl.update(tel, nowMs); // stalledClicks = 1
    TEST_ASSERT_TRUE(ctrl.hasPendingTarget());
    TEST_ASSERT_EQUAL_UINT8(1, ctrl.stalledClicks());

    // Tentativa 3: stall 2/2 -> aborta e sincroniza de volta para 40°C
    nowMs += 10;
    ctrl.update(tel, nowMs); // PRESSING
    nowMs += RinnaiClosedLoopController::PRESS_DURATION_MS + 5;
    ctrl.update(tel, nowMs); // WAITING_SETTLE
    nowMs += RinnaiClosedLoopController::SETTLE_MAX_MS + 5;
    ctrl.update(tel, nowMs); // abortAndSync
    TEST_ASSERT_FALSE(ctrl.hasPendingTarget());
    TEST_ASSERT_EQUAL_INT(syncBefore + 1, ctx.syncCount);
    TEST_ASSERT_EQUAL_UINT8(40, ctx.lastSynced);
    TEST_ASSERT_EQUAL_UINT8(40, ctrl.targetTemp());
}

void test_large_jump_awake_with_stale_packets(void) {
    MockTriStateHardware hw;
    RinnaiClosedLoopController ctrl(hw);
    ControllerTestCtx ctx;
    ctrl.setCallbacks(testSyncCb, testClickCb, &ctx);

    uint32_t nowMs = 1000;
    RinnaiTelemetryState tel = makeTelemetry(35, false, 1);
    tel.lastPacketMs = nowMs;
    ctrl.update(tel, nowMs);
    TEST_ASSERT_EQUAL_INT(1, ctx.syncCount);

    // Salto grande para cima: 35°C -> 43°C (+8°C) com pacotes antigos em trânsito
    ctrl.setTargetTemperature(43, tel, nowMs);
    TEST_ASSERT_TRUE(ctrl.hasPendingTarget());

    for (uint8_t nextTemp = 36; nextTemp <= 43; nextTemp++) {
        nowMs += 10;
        ctrl.update(tel, nowMs); // IDLE -> PRESSING
        TEST_ASSERT_EQUAL(RinnaiClosedLoopController::State::PRESSING, ctrl.state());

        // Pacote B3 antigo chega durante o pressionamento
        nowMs += 40;
        tel.packetCount++;
        tel.tempPacketCount++;
        tel.lastPacketMs = nowMs;
        ctrl.update(tel, nowMs);

        nowMs += RinnaiClosedLoopController::PRESS_DURATION_MS;
        ctrl.update(tel, nowMs); // PRESSING -> WAITING_SETTLE
        TEST_ASSERT_EQUAL(RinnaiClosedLoopController::State::WAITING_SETTLE, ctrl.state());

        // Pacote B3 antigo aos 300ms do settle não pode encerrar antes da confirmação
        nowMs += 300;
        tel.packetCount++;
        tel.tempPacketCount++;
        tel.lastPacketMs = nowMs;
        ctrl.update(tel, nowMs);
        TEST_ASSERT_EQUAL(RinnaiClosedLoopController::State::WAITING_SETTLE, ctrl.state());
        TEST_ASSERT_EQUAL_UINT8(0, ctrl.stalledClicks());

        // Nova temperatura confirmada aos 700ms do settle
        nowMs += 400;
        tel.temperature = nextTemp;
        tel.packetCount++;
        tel.tempPacketCount++;
        tel.lastPacketMs = nowMs;
        ctrl.update(tel, nowMs);
        TEST_ASSERT_EQUAL(RinnaiClosedLoopController::State::IDLE, ctrl.state());
        TEST_ASSERT_EQUAL_UINT8(0, ctrl.stalledClicks());
    }

    TEST_ASSERT_FALSE(ctrl.hasPendingTarget());
    TEST_ASSERT_EQUAL_INT(8, hw.upCount);
    TEST_ASSERT_EQUAL_INT(0, hw.downCount);
    TEST_ASSERT_EQUAL_UINT8(43, ctrl.targetTemp());
    TEST_ASSERT_EQUAL_INT(1, ctx.syncCount); // nenhuma reversão

    // Salto grande para baixo: 43°C -> 35°C (-8°C) com pacotes antigos em trânsito
    ctrl.setTargetTemperature(35, tel, nowMs);
    TEST_ASSERT_TRUE(ctrl.hasPendingTarget());

    for (uint8_t nextTemp = 42; nextTemp >= 35; nextTemp--) {
        nowMs += 10;
        ctrl.update(tel, nowMs); // IDLE -> PRESSING
        TEST_ASSERT_EQUAL(RinnaiClosedLoopController::State::PRESSING, ctrl.state());

        nowMs += 40;
        tel.packetCount++;
        tel.tempPacketCount++;
        tel.lastPacketMs = nowMs;
        ctrl.update(tel, nowMs);

        nowMs += RinnaiClosedLoopController::PRESS_DURATION_MS;
        ctrl.update(tel, nowMs); // PRESSING -> WAITING_SETTLE

        nowMs += 300;
        tel.packetCount++;
        tel.tempPacketCount++;
        tel.lastPacketMs = nowMs;
        ctrl.update(tel, nowMs);
        TEST_ASSERT_EQUAL(RinnaiClosedLoopController::State::WAITING_SETTLE, ctrl.state());

        nowMs += 400;
        tel.temperature = nextTemp;
        tel.packetCount++;
        tel.tempPacketCount++;
        tel.lastPacketMs = nowMs;
        ctrl.update(tel, nowMs);
        TEST_ASSERT_EQUAL(RinnaiClosedLoopController::State::IDLE, ctrl.state());
    }

    TEST_ASSERT_FALSE(ctrl.hasPendingTarget());
    TEST_ASSERT_EQUAL_INT(8, hw.upCount);
    TEST_ASSERT_EQUAL_INT(8, hw.downCount);
    TEST_ASSERT_EQUAL_UINT8(35, ctrl.targetTemp());
    TEST_ASSERT_EQUAL_INT(1, ctx.syncCount);
}

void test_large_jump_from_idle_panel_wakeup(void) {
    MockTriStateHardware hw;
    RinnaiClosedLoopController ctrl(hw);
    ControllerTestCtx ctx;
    ctrl.setCallbacks(testSyncCb, testClickCb, &ctx);

    uint32_t nowMs = 1000;
    RinnaiTelemetryState tel = makeTelemetry(37, false, 1);
    tel.lastPacketMs = nowMs;
    ctrl.update(tel, nowMs);
    int syncBefore = ctx.syncCount;

    ctrl.setTargetTemperature(42, tel, nowMs);
    TEST_ASSERT_TRUE(ctrl.hasPendingTarget());

    // Clique #1: painel em Idle apenas acende o visor (temperatura permanece 37°C)
    nowMs += 10;
    ctrl.update(tel, nowMs); // PRESSING
    TEST_ASSERT_EQUAL_INT(1, hw.upCount);
    nowMs += RinnaiClosedLoopController::PRESS_DURATION_MS + 5;
    ctrl.update(tel, nowMs); // WAITING_SETTLE
    nowMs += 300;
    tel.packetCount++;
    tel.tempPacketCount++;
    ctrl.update(tel, nowMs);
    TEST_ASSERT_EQUAL(RinnaiClosedLoopController::State::WAITING_SETTLE, ctrl.state());
    nowMs += RinnaiClosedLoopController::SETTLE_MAX_MS;
    ctrl.update(tel, nowMs); // timeout do wake-up
    TEST_ASSERT_EQUAL(RinnaiClosedLoopController::State::IDLE, ctrl.state());
    TEST_ASSERT_TRUE(ctrl.hasPendingTarget());
    TEST_ASSERT_TRUE(ctrl.wakeClickUsed());
    TEST_ASSERT_EQUAL_UINT8(0, ctrl.stalledClicks());

    // Cliques #2..#6: painel aceso avança 37 -> 38 -> 39 -> 40 -> 41 -> 42
    for (uint8_t nextTemp = 38; nextTemp <= 42; nextTemp++) {
        nowMs += 10;
        ctrl.update(tel, nowMs); // PRESSING
        nowMs += RinnaiClosedLoopController::PRESS_DURATION_MS + 5;
        ctrl.update(tel, nowMs); // WAITING_SETTLE
        nowMs += 600;
        tel.temperature = nextTemp;
        tel.packetCount++;
        tel.tempPacketCount++;
        tel.lastPacketMs = nowMs;
        ctrl.update(tel, nowMs);
        TEST_ASSERT_EQUAL(RinnaiClosedLoopController::State::IDLE, ctrl.state());
        TEST_ASSERT_EQUAL_UINT8(0, ctrl.stalledClicks());
    }

    TEST_ASSERT_FALSE(ctrl.hasPendingTarget());
    TEST_ASSERT_EQUAL_INT(6, hw.upCount);
    TEST_ASSERT_EQUAL_INT(0, hw.downCount);
    TEST_ASSERT_EQUAL_UINT8(42, ctrl.targetTemp());
    TEST_ASSERT_EQUAL_INT(syncBefore, ctx.syncCount);
}

void test_single_degree_jump_idle_vs_awake(void) {
    MockTriStateHardware hw;
    RinnaiClosedLoopController ctrl(hw);
    ControllerTestCtx ctx;
    ctrl.setCallbacks(testSyncCb, testClickCb, &ctx);

    uint32_t nowMs = 1000;
    RinnaiTelemetryState tel = makeTelemetry(38, false, 1);
    tel.lastPacketMs = nowMs;
    ctrl.update(tel, nowMs);

    // 1) Idle: 38 -> 39 precisa de 1 wake + 1 clique de ajuste (total 2), sem overshoot
    ctrl.setTargetTemperature(39, tel, nowMs);

    // Wake click
    nowMs += 10;
    ctrl.update(tel, nowMs);
    nowMs += RinnaiClosedLoopController::PRESS_DURATION_MS + 5;
    ctrl.update(tel, nowMs);
    nowMs += RinnaiClosedLoopController::SETTLE_MAX_MS + 5;
    ctrl.update(tel, nowMs);
    TEST_ASSERT_TRUE(ctrl.hasPendingTarget());
    TEST_ASSERT_TRUE(ctrl.wakeClickUsed());

    // Adjust click -> 39°C
    nowMs += 10;
    ctrl.update(tel, nowMs);
    nowMs += RinnaiClosedLoopController::PRESS_DURATION_MS + 5;
    ctrl.update(tel, nowMs);
    nowMs += 550;
    tel.temperature = 39;
    tel.packetCount++;
    tel.tempPacketCount++;
    ctrl.update(tel, nowMs);

    // Ciclos extras não devem disparar nenhum clique adicional
    for (int i = 0; i < 3; i++) {
        nowMs += 200;
        ctrl.update(tel, nowMs);
    }
    TEST_ASSERT_FALSE(ctrl.hasPendingTarget());
    TEST_ASSERT_EQUAL_INT(2, hw.upCount);
    TEST_ASSERT_EQUAL_INT(0, hw.downCount);
    TEST_ASSERT_EQUAL_UINT8(39, ctrl.targetTemp());

    // 2) Aceso: 39 -> 40 precisa de exatamente 1 clique, sem overshoot
    ctrl.setTargetTemperature(40, tel, nowMs);
    nowMs += 10;
    ctrl.update(tel, nowMs);
    nowMs += RinnaiClosedLoopController::PRESS_DURATION_MS + 5;
    ctrl.update(tel, nowMs);
    nowMs += 550;
    tel.temperature = 40;
    tel.packetCount++;
    tel.tempPacketCount++;
    ctrl.update(tel, nowMs);

    for (int i = 0; i < 3; i++) {
        nowMs += 200;
        ctrl.update(tel, nowMs);
    }
    TEST_ASSERT_FALSE(ctrl.hasPendingTarget());
    TEST_ASSERT_EQUAL_INT(3, hw.upCount);
    TEST_ASSERT_EQUAL_INT(0, hw.downCount);
    TEST_ASSERT_EQUAL_UINT8(40, ctrl.targetTemp());
}

void test_antiloop_after_partial_progress_from_idle(void) {
    MockTriStateHardware hw;
    RinnaiClosedLoopController ctrl(hw);
    ControllerTestCtx ctx;
    ctrl.setCallbacks(testSyncCb, testClickCb, &ctx);

    uint32_t nowMs = 1000;
    RinnaiTelemetryState tel = makeTelemetry(39, true, 1);
    tel.lastPacketMs = nowMs;
    ctrl.update(tel, nowMs);
    int syncBefore = ctx.syncCount;

    // Painel Idle em 39°C pedindo 43°C com trava física em 42°C
    ctrl.setTargetTemperature(43, tel, nowMs);

    // Clique #1: wake-up (permanece em 39°C)
    nowMs += 10;
    ctrl.update(tel, nowMs);
    nowMs += RinnaiClosedLoopController::PRESS_DURATION_MS + 5;
    ctrl.update(tel, nowMs);
    nowMs += RinnaiClosedLoopController::SETTLE_MAX_MS + 5;
    ctrl.update(tel, nowMs);
    TEST_ASSERT_TRUE(ctrl.wakeClickUsed());
    TEST_ASSERT_EQUAL_UINT8(0, ctrl.stalledClicks());

    // Cliques #2..#4: avança 39 -> 40 -> 41 -> 42
    for (uint8_t nextTemp = 40; nextTemp <= 42; nextTemp++) {
        nowMs += 10;
        ctrl.update(tel, nowMs);
        nowMs += RinnaiClosedLoopController::PRESS_DURATION_MS + 5;
        ctrl.update(tel, nowMs);
        nowMs += 600;
        tel.temperature = nextTemp;
        tel.packetCount++;
        tel.tempPacketCount++;
        ctrl.update(tel, nowMs);
        TEST_ASSERT_TRUE(ctrl.wakeClickUsed());
        TEST_ASSERT_EQUAL_UINT8(0, ctrl.stalledClicks());
    }
    TEST_ASSERT_EQUAL_INT(4, hw.upCount);

    // Clique #5: trava em 42°C (stall 1/2)
    nowMs += 10;
    ctrl.update(tel, nowMs);
    nowMs += RinnaiClosedLoopController::PRESS_DURATION_MS + 5;
    ctrl.update(tel, nowMs);
    nowMs += RinnaiClosedLoopController::SETTLE_MAX_MS + 5;
    ctrl.update(tel, nowMs);
    TEST_ASSERT_TRUE(ctrl.hasPendingTarget());
    TEST_ASSERT_EQUAL_UINT8(1, ctrl.stalledClicks());
    TEST_ASSERT_EQUAL_INT(5, hw.upCount);

    // Clique #6: trava novamente em 42°C (stall 2/2) -> aborta imediatamente (NÃO espera 3)
    nowMs += 10;
    ctrl.update(tel, nowMs);
    nowMs += RinnaiClosedLoopController::PRESS_DURATION_MS + 5;
    ctrl.update(tel, nowMs);
    nowMs += RinnaiClosedLoopController::SETTLE_MAX_MS + 5;
    ctrl.update(tel, nowMs);

    TEST_ASSERT_FALSE(ctrl.hasPendingTarget());
    TEST_ASSERT_EQUAL_INT(syncBefore + 1, ctx.syncCount);
    TEST_ASSERT_EQUAL_UINT8(42, ctx.lastSynced);
    TEST_ASSERT_EQUAL_UINT8(42, ctrl.targetTemp());
    TEST_ASSERT_EQUAL_INT(6, hw.upCount);
}

void test_target_change_mid_flight(void) {
    // Subcaso A: Inversão de alvo durante PRESSING (38 -> 42, muda para 36)
    {
        MockTriStateHardware hw;
        RinnaiClosedLoopController ctrl(hw);
        ControllerTestCtx ctx;
        ctrl.setCallbacks(testSyncCb, testClickCb, &ctx);

        uint32_t nowMs = 1000;
        RinnaiTelemetryState tel = makeTelemetry(38, false, 1);
        ctrl.update(tel, nowMs);

        ctrl.setTargetTemperature(42, tel, nowMs);
        nowMs += 10;
        ctrl.update(tel, nowMs); // PRESSING UP #1
        TEST_ASSERT_EQUAL_INT(1, hw.upCount);

        // Durante PRESSING, usuário muda alvo para 36°C
        nowMs += 40;
        ctrl.setTargetTemperature(36, tel, nowMs);
        TEST_ASSERT_TRUE(ctrl.hasPendingTarget());
        TEST_ASSERT_EQUAL_UINT8(36, ctrl.targetTemp());

        nowMs += RinnaiClosedLoopController::PRESS_DURATION_MS;
        ctrl.update(tel, nowMs); // PRESSING -> WAITING_SETTLE

        // Painel reflete o clique UP para 39°C (movedCorrectly == true para activeDir=UP)
        nowMs += 600;
        tel.temperature = 39;
        tel.packetCount++;
        tel.tempPacketCount++;
        ctrl.update(tel, nowMs);
        TEST_ASSERT_EQUAL(RinnaiClosedLoopController::State::IDLE, ctrl.state());
        TEST_ASSERT_TRUE(ctrl.hasPendingTarget());
        TEST_ASSERT_TRUE(ctrl.wakeClickUsed());

        // Controlador inverte automaticamente para DOWN e desce 39 -> 38 -> 37 -> 36
        for (uint8_t nextTemp = 38; nextTemp >= 36; nextTemp--) {
            nowMs += 10;
            ctrl.update(tel, nowMs); // PRESSING DOWN
            nowMs += RinnaiClosedLoopController::PRESS_DURATION_MS + 5;
            ctrl.update(tel, nowMs); // WAITING_SETTLE
            nowMs += 600;
            tel.temperature = nextTemp;
            tel.packetCount++;
            tel.tempPacketCount++;
            ctrl.update(tel, nowMs);
        }

        TEST_ASSERT_FALSE(ctrl.hasPendingTarget());
        TEST_ASSERT_EQUAL_UINT8(36, ctrl.targetTemp());
        TEST_ASSERT_EQUAL_INT(1, hw.upCount);
        TEST_ASSERT_EQUAL_INT(3, hw.downCount);
        TEST_ASSERT_EQUAL_INT(1, ctx.syncCount);
    }

    // Subcaso B: Reversão do alvo para o valor inicial durante PRESSING quando clique #1 só acorda
    {
        MockTriStateHardware hw;
        RinnaiClosedLoopController ctrl(hw);
        ControllerTestCtx ctx;
        ctrl.setCallbacks(testSyncCb, testClickCb, &ctx);

        uint32_t nowMs = 2000;
        RinnaiTelemetryState tel = makeTelemetry(36, false, 1);
        ctrl.update(tel, nowMs);
        int syncBefore = ctx.syncCount;

        ctrl.setTargetTemperature(40, tel, nowMs);
        nowMs += 10;
        ctrl.update(tel, nowMs); // PRESSING UP #1
        TEST_ASSERT_EQUAL_INT(1, hw.upCount);

        nowMs += 40;
        ctrl.setTargetTemperature(36, tel, nowMs); // reverte para 36°C durante PRESSING

        nowMs += RinnaiClosedLoopController::PRESS_DURATION_MS;
        ctrl.update(tel, nowMs); // WAITING_SETTLE
        nowMs += RinnaiClosedLoopController::SETTLE_MAX_MS + 5;
        ctrl.update(tel, nowMs); // permanece em 36°C == _targetTemp

        TEST_ASSERT_FALSE(ctrl.hasPendingTarget());
        TEST_ASSERT_EQUAL_UINT8(36, ctrl.targetTemp());
        TEST_ASSERT_EQUAL_INT(1, hw.upCount);
        TEST_ASSERT_EQUAL_INT(0, hw.downCount);
        TEST_ASSERT_EQUAL_INT(syncBefore, ctx.syncCount);
    }

    // Subcaso C: Alteração de alvo durante WAITING_SETTLE (36 -> 38, estende para 39)
    {
        MockTriStateHardware hw;
        RinnaiClosedLoopController ctrl(hw);
        ControllerTestCtx ctx;
        ctrl.setCallbacks(testSyncCb, testClickCb, &ctx);

        uint32_t nowMs = 3000;
        RinnaiTelemetryState tel = makeTelemetry(36, false, 1);
        ctrl.update(tel, nowMs);

        ctrl.setTargetTemperature(38, tel, nowMs);
        nowMs += 10;
        ctrl.update(tel, nowMs); // PRESSING UP #1
        nowMs += RinnaiClosedLoopController::PRESS_DURATION_MS + 5;
        ctrl.update(tel, nowMs); // WAITING_SETTLE

        nowMs += 320;
        ctrl.setTargetTemperature(39, tel, nowMs); // estende para 39°C durante WAITING_SETTLE
        ctrl.update(tel, nowMs);
        TEST_ASSERT_EQUAL(RinnaiClosedLoopController::State::WAITING_SETTLE, ctrl.state());

        nowMs += 300;
        tel.temperature = 37;
        tel.packetCount++;
        tel.tempPacketCount++;
        ctrl.update(tel, nowMs); // 37°C confirmado -> IDLE

        for (uint8_t nextTemp = 38; nextTemp <= 39; nextTemp++) {
            nowMs += 10;
            ctrl.update(tel, nowMs);
            nowMs += RinnaiClosedLoopController::PRESS_DURATION_MS + 5;
            ctrl.update(tel, nowMs);
            nowMs += 600;
            tel.temperature = nextTemp;
            tel.packetCount++;
            tel.tempPacketCount++;
            ctrl.update(tel, nowMs);
        }

        TEST_ASSERT_FALSE(ctrl.hasPendingTarget());
        TEST_ASSERT_EQUAL_UINT8(39, ctrl.targetTemp());
        TEST_ASSERT_EQUAL_INT(3, hw.upCount);
        TEST_ASSERT_EQUAL_INT(0, hw.downCount);
    }
}

struct SimPanelHardware : public ITriStateHardware {
    bool panelAwake = false;
    uint8_t panelTemp = 36;
    uint8_t busTemp = 36;
    bool hasPendingBusUpdate = false;
    uint8_t pendingBusTemp = 36;
    uint32_t busUpdateAtMs = 0;
    uint32_t simNowMs = 0;
    int upCount = 0;
    int downCount = 0;

    void release() override {}

    void driveUp() override {
        upCount++;
        if (!panelAwake) {
            panelAwake = true;
            return;
        }
        if (panelTemp < 60) {
            panelTemp++;
            pendingBusTemp = panelTemp;
            busUpdateAtMs = simNowMs + 250;
            hasPendingBusUpdate = true;
        }
    }

    void driveDown() override {
        downCount++;
        if (!panelAwake) {
            panelAwake = true;
            return;
        }
        if (panelTemp > 35) {
            panelTemp--;
            pendingBusTemp = panelTemp;
            busUpdateAtMs = simNowMs + 250;
            hasPendingBusUpdate = true;
        }
    }

    void tickBus(uint32_t nowMs) {
        simNowMs = nowMs;
        if (hasPendingBusUpdate && nowMs >= busUpdateAtMs) {
            busTemp = pendingBusTemp;
            hasPendingBusUpdate = false;
        }
    }
};

static void emitSimB3Burst(RinnaiRmtDecoder &rmt, uint8_t temp, bool flame, uint32_t nowMs) {
    uint8_t pkt[8] = {
        (uint8_t)(flame ? 0x0D : 0x09),
        (uint8_t)(flame ? 0xE8 : 0x08),
        temp,
        0x1C, 0x3A, 0x77, 0x00, 0x00
    };
    uint16_t crc = RinnaiRmtDecoder::crc16Rinnai(pkt, 6);
    pkt[6] = (uint8_t)(crc >> 8);
    pkt[7] = (uint8_t)(crc & 0xFF);

    RinnaiRmtPulse pulses[129];
    encodePktToPulses(pkt, pulses);
    rmt.feedPulses(pulses, 129, nowMs);
}

void test_e2e_rmt_decoder_and_controller_large_jump_from_idle(void) {
    SimPanelHardware simHw;
    RinnaiRmtDecoder rmt;
    RinnaiClosedLoopController ctrl(simHw);
    ControllerTestCtx ctx;
    ctrl.setCallbacks(testSyncCb, testClickCb, &ctx);

    uint32_t nowMs = 1000;
    simHw.simNowMs = nowMs;
    emitSimB3Burst(rmt, simHw.busTemp, false, nowMs);
    ctrl.update(rmt.state(), nowMs);
    TEST_ASSERT_TRUE(rmt.state().valid);
    TEST_ASSERT_EQUAL_UINT8(36, rmt.state().temperature);
    TEST_ASSERT_EQUAL_INT(1, ctx.syncCount);

    // 1) Salto grande subindo a partir do painel em Idle: 36°C -> 42°C
    ctrl.setTargetTemperature(42, rmt.state(), nowMs);
    uint32_t nextBurstMs = nowMs + 250;

    for (int step = 0; step < 3000; step++) {
        nowMs += 10;
        simHw.tickBus(nowMs);
        if (nowMs >= nextBurstMs) {
            emitSimB3Burst(rmt, simHw.busTemp, false, nowMs);
            nextBurstMs += 250;
        }
        rmt.tick(nowMs);
        ctrl.update(rmt.state(), nowMs);
        if (!ctrl.hasPendingTarget() && ctrl.state() == RinnaiClosedLoopController::State::IDLE) {
            break;
        }
    }

    TEST_ASSERT_FALSE(ctrl.hasPendingTarget());
    TEST_ASSERT_EQUAL_UINT8(42, rmt.state().temperature);
    TEST_ASSERT_EQUAL_UINT8(42, ctrl.targetTemp());
    TEST_ASSERT_EQUAL_INT(7, simHw.upCount); // 1 wake-up + 6 passos (36->42)
    TEST_ASSERT_EQUAL_INT(0, simHw.downCount);
    TEST_ASSERT_EQUAL_INT(1, ctx.syncCount);

    // 2) Painel volta a Idle e recebe salto grande descendo: 42°C -> 36°C
    simHw.panelAwake = false;
    ctrl.setTargetTemperature(36, rmt.state(), nowMs);

    for (int step = 0; step < 3000; step++) {
        nowMs += 10;
        simHw.tickBus(nowMs);
        if (nowMs >= nextBurstMs) {
            emitSimB3Burst(rmt, simHw.busTemp, false, nowMs);
            nextBurstMs += 250;
        }
        rmt.tick(nowMs);
        ctrl.update(rmt.state(), nowMs);
        if (!ctrl.hasPendingTarget() && ctrl.state() == RinnaiClosedLoopController::State::IDLE) {
            break;
        }
    }

    TEST_ASSERT_FALSE(ctrl.hasPendingTarget());
    TEST_ASSERT_EQUAL_UINT8(36, rmt.state().temperature);
    TEST_ASSERT_EQUAL_UINT8(36, ctrl.targetTemp());
    TEST_ASSERT_EQUAL_INT(7, simHw.upCount);
    TEST_ASSERT_EQUAL_INT(7, simHw.downCount); // 1 wake-up + 6 passos (42->36)
    TEST_ASSERT_EQUAL_INT(1, ctx.syncCount);
}

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;
    UNITY_BEGIN();
    RUN_TEST(test_real_hardware_40_idle_and_flame_and_41_flame);
    RUN_TEST(test_legacy_63_byte_frames);
    RUN_TEST(test_rmt_crc16_and_real_packets);
    RUN_TEST(test_closed_loop_controller_flame_sync_and_antiloop);
    RUN_TEST(test_large_jump_awake_with_stale_packets);
    RUN_TEST(test_large_jump_from_idle_panel_wakeup);
    RUN_TEST(test_single_degree_jump_idle_vs_awake);
    RUN_TEST(test_antiloop_after_partial_progress_from_idle);
    RUN_TEST(test_target_change_mid_flight);
    RUN_TEST(test_e2e_rmt_decoder_and_controller_large_jump_from_idle);
    return UNITY_END();
}
