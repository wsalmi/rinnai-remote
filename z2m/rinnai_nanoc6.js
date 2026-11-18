const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const tz = require('zigbee-herdsman-converters/converters/toZigbee');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const reporting = require('zigbee-herdsman-converters/lib/reporting');

const e = exposes.presets;
const ea = exposes.access;

/**
 * External Converter do Zigbee2MQTT para o Rinnai Smart Remote (M5NanoC6 + HY-M154).
 *
 * Instalação:
 * - Copie este arquivo para `data/external_converters/rinnai_nanoc6.js` no Zigbee2MQTT
 *   (ou adicione em `external_converters:` no `configuration.yaml` em versões antigas).
 *
 * Comportamento no HomeKit (via homebridge-z2m):
 * - Exposto como Termostato único (faixa segura de banho: 35°C a 48°C, passo 1°C).
 * - `running_state` = 'heat' exibe "Aquecendo" (laranja) no app Casa quando a chama acende.
 * - `system_mode` reflete o botão Power físico do painel ('heat' / 'off').
 */
const definition = {
    zigbeeModel: ['Rinnai-NanoC6'],
    model: 'Rinnai-NanoC6',
    vendor: 'RinnaiCustom',
    description: 'Controlador Zigbee para aquecedor Rinnai via M5NanoC6 + optoacopladores HY-M154 (35°C - 48°C)',
    fromZigbee: [fz.thermostat],
    toZigbee: [
        tz.thermostat_occupied_heating_setpoint,
        tz.thermostat_system_mode,
    ],
    exposes: [
        e.climate()
            .withLocalTemperature(ea.STATE)
            .withSetpoint('occupied_heating_setpoint', 35, 48, 1, ea.ALL)
            .withSystemMode(['off', 'heat'], ea.ALL)
            .withRunningState(['idle', 'heat'], ea.STATE),
    ],
    configure: async (device, coordinatorEndpoint, logger) => {
        const endpoint = device.getEndpoint(1);
        await reporting.bind(endpoint, coordinatorEndpoint, ['hvacThermostat']);

        const setupReport = async (fn, label) => {
            try {
                await fn(endpoint);
            } catch (err) {
                logger.warn(`Rinnai-NanoC6: configure reporting ${label} skipped: ${err.message}`);
            }
        };

        await setupReport(reporting.thermostatTemperature, 'localTemp');
        await setupReport(reporting.thermostatOccupiedHeatingSetpoint, 'occupiedHeatingSetpoint');
        await setupReport(reporting.thermostatSystemMode, 'systemMode');
        await setupReport(reporting.thermostatRunningState, 'runningState');

        try {
            await endpoint.read('hvacThermostat', [
                'localTemp',
                'occupiedHeatingSetpoint',
                'systemMode',
                'runningState',
            ]);
        } catch (err) {
            logger.warn(`Rinnai-NanoC6: initial attribute read skipped: ${err.message}`);
        }
    },
};

module.exports = definition;
