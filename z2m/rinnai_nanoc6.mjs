import * as fz from 'zigbee-herdsman-converters/converters/fromZigbee';
import * as tz from 'zigbee-herdsman-converters/converters/toZigbee';
import * as exposes from 'zigbee-herdsman-converters/lib/exposes';
import * as reporting from 'zigbee-herdsman-converters/lib/reporting';
import {logger} from 'zigbee-herdsman-converters/lib/logger';

const e = exposes.presets;
const ea = exposes.access;

/**
 * External Converter (Zigbee2MQTT 2.x, ESM) - Rinnai Smart Remote (M5NanoC6 + HY-M154).
 * Instalação: copiar para `data/external_converters/rinnai_nanoc6.mjs` e reiniciar o Z2M.
 *
 * Estado: system_mode = 'heat' quando a chama está acesa; 'off' em qualquer outro caso.
 * OTA: configuration.yaml ->
 *   ota:
 *     zigbee_ota_override_index_location: /opt/zigbee2mqtt/data/ota/ota_index.json
 */
const definition = {
    zigbeeModel: ['Rinnai-NanoC6'],
    model: 'Rinnai-NanoC6',
    vendor: 'RinnaiCustom',
    description: 'Controlador Zigbee para aquecedor Rinnai via M5NanoC6 + HY-M154 (35°C - 43°C)',
    fromZigbee: [fz.thermostat],
    toZigbee: [tz.thermostat_occupied_heating_setpoint, tz.thermostat_system_mode],
    ota: true,
    exposes: [
        e.climate()
            .withLocalTemperature(ea.STATE)
            .withSetpoint('occupied_heating_setpoint', 35, 43, 1, ea.ALL)
            .withSystemMode(['off', 'heat'], ea.ALL)
            .withRunningState(['idle', 'heat'], ea.STATE),
    ],
    configure: async (device, coordinatorEndpoint) => {
        const endpoint = device.getEndpoint(1);
        await reporting.bind(endpoint, coordinatorEndpoint, ['hvacThermostat']);
        const setup = async (fn, label) => {
            try {
                await fn(endpoint);
            } catch (err) {
                logger.warning(`Rinnai-NanoC6: configure reporting ${label} skipped: ${err.message}`, 'zhc:rinnai');
            }
        };
        await setup(reporting.thermostatTemperature, 'localTemp');
        await setup(reporting.thermostatOccupiedHeatingSetpoint, 'occupiedHeatingSetpoint');
        await setup(reporting.thermostatSystemMode, 'systemMode');
        await setup(reporting.thermostatRunningState, 'runningState');
        try {
            await endpoint.read('hvacThermostat', ['localTemp', 'occupiedHeatingSetpoint', 'systemMode', 'runningState']);
        } catch (err) {
            logger.warning(`Rinnai-NanoC6: initial read skipped: ${err.message}`, 'zhc:rinnai');
        }
    },
};

export default definition;
