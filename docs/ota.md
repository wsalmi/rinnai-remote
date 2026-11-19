# OTA via Zigbee2MQTT (2.8.0)

O firmware tem cliente OTA Zigbee (cluster 0x0019; fabricante `0x1001`, tipo `0x1011`).
A tabela de partições `zigbee.csv` já possui `ota_0`/`ota_1` (rollback automático em falha).

## Publicar uma versão
1. Aumente `RINNAI_FW_VERSION` (ex.: `0x00010002`) em `include/rinnai_zigbee_thermostat.h`.
2. `cd firmware_nanoc6 && ~/.platformio/penv/bin/pio run`
3. `python3 tools/make_ota.py --version 0x00010002` (mesmo valor do passo 1)
4. Copie `z2m/ota/*` para `data/ota/` do Z2M (Raspberry Pi) e `z2m/rinnai_nanoc6.mjs` para `data/external_converters/`.
5. `configuration.yaml`:
   ```yaml
   ota:
     zigbee_ota_override_index_location: /opt/zigbee2mqtt/data/ota/ota_index.json
   ```
6. Reinicie o Z2M → aba OTA do dispositivo → *Check for updates* → *Update*.

A primeira gravação do firmware com cliente OTA é por USB; as seguintes são remotas.
