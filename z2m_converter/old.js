import {occupancy} from "zigbee-herdsman-converters/converters/fromZigbee";
import {access, presets} from "zigbee-herdsman-converters/lib/exposes";
import * as reporting from "zigbee-herdsman-converters/lib/reporting";
import {bind} from "zigbee-herdsman-converters/lib/reporting";

const RADAR_CLUSTER = 0xff00;

/** @type{Record<string, import('zigbee-herdsman-converters/lib/types').Fz.Converter>} */
const fzLocal = {
    radar_config: {
        cluster: '65280',
        type: ["attributeReport", "readResponse"],
        convert: (model, msg, publish, options, meta) => {
            // Accept a few possible shapes for msg.data to be defensive
            const data = msg.data || {};
            const result = {};

            const extract = (d, k) => {
                if (d === undefined || d === null) return undefined;
                // value may be present directly as d[k]
                if (d[k] !== undefined) return d[k];
                // sometimes attribute reports return objects like { value: X, type: Y }
                if (d[k] && typeof d[k] === "object" && d[k].value !== undefined) return d[k].value;
                // string keys fallback
                if (d[String(k)] !== undefined) return d[String(k)];
                return undefined;
            };

            const v0 = extract(data, 0);
            const v1 = extract(data, 1);
            const v2 = extract(data, 2);
            const v3 = extract(data, 3);

            if (v0 !== undefined) result.range = Number(v0);
            if (v1 !== undefined) result.timeout = Number(v1);
            if (v2 !== undefined) result.update_rate = Number(v2);
            if (v3 !== undefined) result.angle = Number(v3);

            if (Object.keys(result).length === 0) return;

            return result;
        },
    },
    // Add analog value converter
    analog_value: {
        cluster: "genAnalogValue",
        type: ["attributeReport", "readResponse"],
        convert: (model, msg, publish, options, meta) => {
            const data = msg.data || {};
            const result = {};

            // presentValue is the main analog value (attribute 0x0055)
            if (data.presentValue !== undefined) {
                result.analog_value = Number(data.presentValue);
            }

            // Optional: description attribute (0x001C)
            if (data.description !== undefined) {
                result.analog_description = data.description;
            }

            if (Object.keys(result).length === 0) return;

            return result;
        },
    },
};

/** @type{Record<string, any>} */
const tzLocal = {
    radar_config: {
        key: ["range", "timeout", "update_rate", "angle"],
        convertSet: async (entity, key, value, meta) => {
            const attrMap = {
                range: {id: 0x0000, type: 0x21}, // U16
                timeout: {id: 0x0001, type: 0x21}, // U16
                update_rate: {id: 0x0002, type: 0x20}, // U8
                angle: {id: 0x0003, type: 0x21}, // U16
            };

            const payload = {};
            payload[attrMap[key]] = value;  // Just send the raw value
            const endpoint = meta.device.getEndpoint(10);
            await endpoint.write(0xff00, payload);
            return {state: {[key]: value}};
        },
        convertGet: async (entity, key, meta) => {
            const attrMap = {
                range: 0x0000,
                timeout: 0x0001,
                update_rate: 0x0002,
                angle: 0x0003,
            };

            const attrId = attrMap[key];
            if (attrId === undefined) throw new Error(`Unknown attribute key: ${key}`);

            try {
                const result = await entity.read(RADAR_CLUSTER, [attrId]);

                // Read returns something like { [attrId]: {value, type} }
                /** @type {any} */
                const entry = result?.[attrId];
                const val = entry && entry.value !== undefined ? entry.value : entry;

                // Log the read result and return state so callers (UI/API) get the value immediately.
                console.log(`[RADAR] Read ${key}:`, val);
                return {state: {[key]: val}};
            } catch (error) {
                console.error(`Failed to read attribute ${key} (${attrId}) on cluster 0x${RADAR_CLUSTER.toString(16)}:`, error);
            }
        },
    },
    // Add analog value write support (optional)
    analog_value: {
        key: ["analog_value"],
        convertSet: async (entity, key, value, meta) => {
            const numericValue = Number(value);
            // presentValue attribute ID is 0x0055
            await entity.write("genAnalogValue", {presentValue: numericValue});
            return {state: {analog_value: numericValue}};
        },
        convertGet: async (entity, key, meta) => {
            try {
                await entity.read("genAnalogValue", ["presentValue"]);
            } catch (error) {
                console.error("Failed to read analog value:", error);
            }
        },
    },
};

/** @type{import('zigbee-herdsman-converters/lib/types').DefinitionWithExtend} */
export default {
    zigbeeModel: ["JP0"],
    model: "JP0",
    vendor: "JUREK",
    description: "mmWave radar sensor with custom config cluster and analog value",
    fromZigbee: [occupancy, fzLocal.radar_config, fzLocal.analog_value],
    toZigbee: [tzLocal.radar_config, tzLocal.analog_value],
    exposes: [
        presets.occupancy(),
        presets.numeric("range", access.ALL).withValueMin(0).withValueMax(65535).withDescription("Detection range"),
        presets.numeric("timeout", access.ALL).withValueMin(0).withValueMax(65535).withDescription("Timeout setting"),
        presets.numeric("update_rate", access.ALL).withValueMin(0).withValueMax(255).withDescription("Update rate"),
        presets.numeric("angle", access.ALL).withValueMin(0).withValueMax(65535).withDescription("Detection angle"),
        // Add analog value expose
        presets.numeric("analog_value", access.ALL).withDescription("Analog input value"),
    ],
    configure: async (device, coordinatorEndpoint, definition) => {
        const endpoint = device.getEndpoint(10);

        if (endpoint) {
            // Bind occupancy cluster so the coordinator receives occupancy reports
            await bind(endpoint, coordinatorEndpoint, ["msOccupancySensing"]);

            // Bind the custom radar cluster so attribute reports arrive to coordinator
            // Use the helper bind to ensure correct parameters and compatibility.
            await bind(endpoint, coordinatorEndpoint, [RADAR_CLUSTER]);

            // Bind analog input cluster
            await bind(endpoint, coordinatorEndpoint, ["genAnalogValue"]);

            // Configure occupancy reporting
            await reporting.occupancy(endpoint);
        }
    },
};
