#pragma once

#ifndef ZIGBEE_MODE_ED
#error "Zigbee End Device mode is required (-DZIGBEE_MODE_ED)"
#endif

#include <Arduino.h>
#include "Zigbee.h"
#include "esp_zigbee_core.h"

// Versão do firmware (OTA): incrementar a cada build publicado. Formato 0xMMmmpppp.
#ifndef RINNAI_FW_VERSION
#define RINNAI_FW_VERSION 0x00010007
#endif
#define RINNAI_OTA_MANUFACTURER 0x1001
#define RINNAI_OTA_IMAGE_TYPE   0x1011

/**
 * @brief Endpoint Zigbee HA Thermostat (Cluster 0x0201 - Server Role) para aquecedor Rinnai.
 *
 * Atributos expostos e reportados ao Zigbee2MQTT / HomeKit:
 * - LocalTemperature (0x0000): Temperatura atual decodificada na linha TX da central.
 * - OccupiedHeatingSetpoint (0x0012): Alvo controlado pelos botões '+' / '-' (35°C a 43°C).
 * - SystemMode (0x001C): Espelho do estado da chama (Heat com chama acesa / Off caso contrário);
 *   comandos remotos de troca de modo são revertidos para refletir o estado real do aquecedor.
 * - ThermostatRunningState (0x0029): Bit 0 (0x0001 = Heat On) quando a chama está acesa.
 */
class RinnaiZigbeeThermostat : public ZigbeeEP {
public:
    explicit RinnaiZigbeeThermostat(uint8_t endpoint = 1) : ZigbeeEP(endpoint) {
        _device_id = ESP_ZB_HA_THERMOSTAT_DEVICE_ID;
        _cluster_list = esp_zb_zcl_cluster_list_create();

        esp_zb_basic_cluster_cfg_t basic_cfg = {
            .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
            .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_MAINS_SINGLE_PHASE,
        };
        esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&basic_cfg);
        esp_zb_cluster_list_add_basic_cluster(_cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        esp_zb_identify_cluster_cfg_t identify_cfg = {
            .identify_time = ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE,
        };
        esp_zb_attribute_list_t *identify_cluster = esp_zb_identify_cluster_create(&identify_cfg);
        esp_zb_cluster_list_add_identify_cluster(_cluster_list, identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        // Cria o cluster manualmente para garantir ESP_ZB_ZCL_ATTR_ACCESS_REPORTING em 0x0012 e 0x001C
        // (esp_zb_thermostat_cluster_create() omite ACCESS_REPORTING nesses dois atributos).
        esp_zb_attribute_list_t *thermostat_cluster =
            esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT);
        esp_zb_cluster_add_attr(
            thermostat_cluster,
            ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
            ESP_ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID,
            ESP_ZB_ZCL_ATTR_TYPE_S16,
            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
            &_localTempAttr
        );
        esp_zb_cluster_add_attr(
            thermostat_cluster,
            ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
            ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID,
            ESP_ZB_ZCL_ATTR_TYPE_S16,
            ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_SCENE,
            &_occCoolSetpointAttr
        );
        esp_zb_cluster_add_attr(
            thermostat_cluster,
            ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
            ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID,
            ESP_ZB_ZCL_ATTR_TYPE_S16,
            ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING | ESP_ZB_ZCL_ATTR_ACCESS_SCENE,
            &_occHeatSetpointAttr
        );
        esp_zb_cluster_add_attr(
            thermostat_cluster,
            ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
            ESP_ZB_ZCL_ATTR_THERMOSTAT_CONTROL_SEQUENCE_OF_OPERATION_ID,
            ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM,
            ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,
            &_ctrlSeqAttr
        );
        esp_zb_cluster_add_attr(
            thermostat_cluster,
            ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
            ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID,
            ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM,
            ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING | ESP_ZB_ZCL_ATTR_ACCESS_SCENE,
            &_systemModeAttr
        );

        struct LimitAttr {
            uint16_t id;
            int16_t *ptr;
        };
        const LimitAttr limits[] = {
            {ESP_ZB_ZCL_ATTR_THERMOSTAT_ABS_MIN_HEAT_SETPOINT_LIMIT_ID, &_absMinHeat},
            {ESP_ZB_ZCL_ATTR_THERMOSTAT_ABS_MAX_HEAT_SETPOINT_LIMIT_ID, &_absMaxHeat},
            {ESP_ZB_ZCL_ATTR_THERMOSTAT_MIN_HEAT_SETPOINT_LIMIT_ID,     &_minHeatLimit},
            {ESP_ZB_ZCL_ATTR_THERMOSTAT_MAX_HEAT_SETPOINT_LIMIT_ID,     &_maxHeatLimit},
            {ESP_ZB_ZCL_ATTR_THERMOSTAT_ABS_MIN_COOL_SETPOINT_LIMIT_ID, &_absMinCool},
            {ESP_ZB_ZCL_ATTR_THERMOSTAT_ABS_MAX_COOL_SETPOINT_LIMIT_ID, &_absMaxCool},
            {ESP_ZB_ZCL_ATTR_THERMOSTAT_MIN_COOL_SETPOINT_LIMIT_ID,     &_minCoolLimit},
            {ESP_ZB_ZCL_ATTR_THERMOSTAT_MAX_COOL_SETPOINT_LIMIT_ID,     &_maxCoolLimit},
        };
        for (const auto &attr : limits) {
            esp_zb_thermostat_cluster_add_attr(thermostat_cluster, attr.id, attr.ptr);
        }

        // Running State (0x0029): Read-Only + Reportável para indicar chama ativa no HomeKit
        esp_zb_cluster_add_attr(
            thermostat_cluster,
            ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
            ESP_ZB_ZCL_ATTR_THERMOSTAT_THERMOSTAT_RUNNING_STATE_ID,
            ESP_ZB_ZCL_ATTR_TYPE_16BITMAP,
            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
            &_runningStateAttr
        );

        esp_zb_cluster_list_add_thermostat_cluster(
            _cluster_list,
            thermostat_cluster,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE
        );

        _ep_config = {
            .endpoint = _endpoint,
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = _device_id,
            .app_device_version = 0,
        };

        // Cliente OTA (cluster 0x0019): permite atualizar via Zigbee2MQTT sem USB
        addOTAClient(RINNAI_FW_VERSION, RINNAI_FW_VERSION, 1,
                     RINNAI_OTA_MANUFACTURER, RINNAI_OTA_IMAGE_TYPE);
    }

    void markStarted() {
        _stackReady = true;
        applySetpointLimits();
    }

    void applySetpointLimits() {
        if (!_stackReady || !Zigbee.started() || _limitsApplied) return;

        struct AttrInit {
            uint16_t id;
            int16_t val;
        };
        AttrInit attrs[] = {
            {ESP_ZB_ZCL_ATTR_THERMOSTAT_ABS_MIN_HEAT_SETPOINT_LIMIT_ID, 3500},
            {ESP_ZB_ZCL_ATTR_THERMOSTAT_ABS_MAX_HEAT_SETPOINT_LIMIT_ID, 4300},
            {ESP_ZB_ZCL_ATTR_THERMOSTAT_MIN_HEAT_SETPOINT_LIMIT_ID,     3500},
            {ESP_ZB_ZCL_ATTR_THERMOSTAT_MAX_HEAT_SETPOINT_LIMIT_ID,     4300},
            {ESP_ZB_ZCL_ATTR_THERMOSTAT_ABS_MIN_COOL_SETPOINT_LIMIT_ID, 5000},
            {ESP_ZB_ZCL_ATTR_THERMOSTAT_ABS_MAX_COOL_SETPOINT_LIMIT_ID, 7000},
            {ESP_ZB_ZCL_ATTR_THERMOSTAT_MIN_COOL_SETPOINT_LIMIT_ID,     5000},
            {ESP_ZB_ZCL_ATTR_THERMOSTAT_MAX_COOL_SETPOINT_LIMIT_ID,     7000},
            {ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID,   6000},
        };

        static const uint16_t reportableAttrs[] = {
            ESP_ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID,
            ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID,
            ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID,
            ESP_ZB_ZCL_ATTR_THERMOSTAT_THERMOSTAT_RUNNING_STATE_ID,
        };
        uint16_t failedRepAttr = 0;

        esp_zb_lock_acquire(portMAX_DELAY);
        for (auto &entry : attrs) {
            esp_zb_zcl_set_attribute_val(
                _endpoint,
                ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                entry.id,
                &entry.val,
                false
            );
        }
        for (uint16_t attrId : reportableAttrs) {
            if (!ensureReportingInfoLocked(attrId) && failedRepAttr == 0) {
                failedRepAttr = attrId;
            }
        }
        esp_zb_lock_release();

        _limitsApplied = true;
        Serial.println("[ZIGBEE] Limites do Thermostat aplicados (Heat: 35-43°C, Cool: 60°C)");
        if (failedRepAttr != 0) {
            Serial.printf("[ZIGBEE] AVISO: falha ao registrar reporting_info para 0x%04X\n", failedRepAttr);
        }
    }

    void setPhysicalPowerState(bool powerOn) {
        portENTER_CRITICAL(&_mux);
        _physicalPowerOn = powerOn;
        portEXIT_CRITICAL(&_mux);
    }

    bool consumePendingSetpoint(int &outTempC) {
        bool has = false;
        portENTER_CRITICAL(&_mux);
        if (_hasPendingSetpoint) {
            outTempC = _pendingSetpointC;
            _hasPendingSetpoint = false;
            has = true;
        }
        portEXIT_CRITICAL(&_mux);
        return has;
    }

    bool consumePendingModeRevert() {
        bool has = false;
        portENTER_CRITICAL(&_mux);
        if (_pendingModeRevert) {
            _pendingModeRevert = false;
            has = true;
        }
        portEXIT_CRITICAL(&_mux);
        return has;
    }

    void updateLocalTemperature(uint8_t tempC, bool forceReport = false) {
        if (tempC == 0) return;
        int16_t zclTemp = (int16_t)tempC * 100;
        if (!forceReport && zclTemp == _lastLocalTemp) return;
        _lastLocalTemp = zclTemp;
        setAndReportAttr(ESP_ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID, &zclTemp, true);
    }

    void updateHeatingSetpoint(uint8_t tempC, bool forceReport = false) {
        int16_t zclSetpoint = (int16_t)tempC * 100;
        portENTER_CRITICAL(&_mux);
        bool unchanged = (!forceReport && zclSetpoint == _lastSetpoint);
        if (!unchanged) {
            _lastSetpoint = zclSetpoint;
        }
        portEXIT_CRITICAL(&_mux);
        if (unchanged) return;
        setAndReportAttr(ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID, &zclSetpoint, true);
    }

    void updateSystemMode(bool powerOn, bool forceReport = false) {
        setPhysicalPowerState(powerOn);
        uint8_t mode = powerOn ? ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_HEAT
                               : ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF;
        if (!forceReport && mode == _lastSystemMode) return;
        _lastSystemMode = mode;
        setAndReportAttr(ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID, &mode, true);
    }

    void updateRunningState(bool flameActive, bool forceReport = false) {
        uint16_t runState = flameActive ? 0x0001 : 0x0000;
        if (!forceReport && runState == _lastRunningState) return;
        _lastRunningState = runState;
        setAndReportAttr(ESP_ZB_ZCL_ATTR_THERMOSTAT_THERMOSTAT_RUNNING_STATE_ID, &runState, true);
    }

protected:
    void zbAttributeSet(const esp_zb_zcl_set_attr_value_message_t *message) override {
        if (!message || message->info.status != ESP_ZB_ZCL_STATUS_SUCCESS) {
            return;
        }
        if (message->info.cluster != ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT ||
            message->attribute.data.value == nullptr) {
            return;
        }

        if (message->attribute.id == ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID &&
            message->attribute.data.size >= sizeof(int16_t)) {
            int16_t raw = *(const int16_t *)message->attribute.data.value;
            int tempC = (raw >= 0) ? ((raw + 50) / 100) : ((raw - 50) / 100);
            portENTER_CRITICAL(&_mux);
            _lastSetpoint = raw;
            _pendingSetpointC = tempC;
            _hasPendingSetpoint = true;
            portEXIT_CRITICAL(&_mux);
            Serial.printf("[ZIGBEE] Write occupied_heating_setpoint: %d°C (raw=%d)\n", tempC, raw);
            log_i("Zigbee setpoint write: %d (raw=%d)", tempC, raw);
        } else if (message->attribute.id == ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID &&
                   message->attribute.data.size >= sizeof(uint8_t)) {
            uint8_t reqMode = *(const uint8_t *)message->attribute.data.value;
            bool revert = false;
            portENTER_CRITICAL(&_mux);
            uint8_t expectedMode = _physicalPowerOn
                ? ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_HEAT
                : ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF;
            if (reqMode != expectedMode) {
                _pendingModeRevert = true;
                revert = true;
            }
            portEXIT_CRITICAL(&_mux);
            Serial.printf("[ZIGBEE] Write system_mode=%u (fisico=%u%s)\n",
                          reqMode, _physicalPowerOn ? 1 : 0, revert ? " -> revert" : "");
            log_i("Zigbee system_mode write=%u (physical=%u)", reqMode, _physicalPowerOn ? 1 : 0);
        }
    }

private:
    bool ensureReportingInfoLocked(uint16_t attrId) {
        esp_zb_zcl_attr_location_info_t loc = {};
        loc.endpoint_id = _endpoint;
        loc.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT;
        loc.cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;
        loc.manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC;
        loc.attr_id = attrId;
        if (esp_zb_zcl_find_reporting_info(loc) != nullptr) {
            return true;
        }
        esp_zb_zcl_reporting_info_t rep = {};
        rep.direction = ESP_ZB_ZCL_REPORT_DIRECTION_SEND;
        rep.ep = _endpoint;
        rep.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT;
        rep.cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;
        rep.attr_id = attrId;
        rep.u.send_info.min_interval = 1;
        rep.u.send_info.max_interval = 0;
        rep.u.send_info.def_min_interval = 1;
        rep.u.send_info.def_max_interval = 0;
        rep.dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID;
        rep.manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC;
        esp_err_t err = esp_zb_zcl_update_reporting_info(&rep);
        return (err == ESP_OK) && (esp_zb_zcl_find_reporting_info(loc) != nullptr);
    }

    void setAndReportAttr(uint16_t attrId, void *valuePtr, bool sendReport) {
        if (!_stackReady || !Zigbee.started()) return;
        applySetpointLimits();

        bool didReport = false;
        esp_zb_zcl_status_t setStatus = ESP_ZB_ZCL_STATUS_SUCCESS;
        esp_err_t repErr = ESP_OK;

        esp_zb_lock_acquire(portMAX_DELAY);
        setStatus = esp_zb_zcl_set_attribute_val(
            _endpoint,
            ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            attrId,
            valuePtr,
            false
        );

        if (sendReport && Zigbee.connected()) {
            didReport = true;
            if (ensureReportingInfoLocked(attrId)) {
                esp_zb_zcl_report_attr_cmd_t reportCmd = {};
                reportCmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
                reportCmd.attributeID = attrId;
                reportCmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
                reportCmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT;
                reportCmd.zcl_basic_cmd.src_endpoint = _endpoint;
                reportCmd.manuf_specific = 0;
                repErr = esp_zb_zcl_report_attr_cmd_req(&reportCmd);
            } else {
                repErr = ESP_ERR_NOT_FOUND;
            }
        }
        esp_zb_lock_release();

        if (setStatus != ESP_ZB_ZCL_STATUS_SUCCESS) {
            Serial.printf("[ZIGBEE] Set attr 0x%04X falhou: 0x%02X\n", attrId, (unsigned)setStatus);
        }
        if (didReport) {
            if (repErr != ESP_OK) {
                Serial.printf("[ZIGBEE] Report attr 0x%04X falhou: 0x%x (%s)\n",
                              attrId, repErr, esp_err_to_name(repErr));
            } else if (attrId == ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID) {
                Serial.printf("[ZIGBEE] Report 0x%04X OK (val=%d)\n",
                              attrId, (int)(*(const int16_t *)valuePtr));
            } else if (attrId == ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID) {
                Serial.printf("[ZIGBEE] Report 0x%04X OK (val=%d)\n",
                              attrId, (int)(*(const uint8_t *)valuePtr));
            } else if (attrId == ESP_ZB_ZCL_ATTR_THERMOSTAT_THERMOSTAT_RUNNING_STATE_ID) {
                Serial.printf("[ZIGBEE] Report 0x%04X OK (val=%d)\n",
                              attrId, (int)(*(const uint16_t *)valuePtr));
            }
        }
    }

    int16_t _localTempAttr = 4000;
    int16_t _occCoolSetpointAttr = 6000;
    int16_t _occHeatSetpointAttr = 4000;
    uint8_t _ctrlSeqAttr = ESP_ZB_ZCL_THERMOSTAT_CONTROL_SEQ_OF_OPERATION_HEATING_ONLY;
    uint8_t _systemModeAttr = ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF;

    int16_t _absMinHeat = 3500;
    int16_t _absMaxHeat = 4300;
    int16_t _minHeatLimit = 3500;
    int16_t _maxHeatLimit = 4300;
    int16_t _absMinCool = 5000;
    int16_t _absMaxCool = 7000;
    int16_t _minCoolLimit = 5000;
    int16_t _maxCoolLimit = 7000;
    uint16_t _runningStateAttr = 0x0000;

    int16_t _lastLocalTemp = -32768;
    int16_t _lastSetpoint = -32768;
    uint8_t _lastSystemMode = 0xFF;
    uint16_t _lastRunningState = 0xFFFF;

    portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;
    bool _stackReady = false;
    bool _limitsApplied = false;
    volatile bool _physicalPowerOn = false;
    volatile bool _hasPendingSetpoint = false;
    volatile int _pendingSetpointC = 40;
    volatile bool _pendingModeRevert = false;
};
