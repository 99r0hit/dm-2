#include <EEPROM.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "DriveMatrixProtocol.h"

// ============================================================
// nRF24
// ============================================================

#define NRF_CE_PIN  10
#define NRF_CSN_PIN 9

RF24 radio(
    NRF_CE_PIN,
    NRF_CSN_PIN
);

// ============================================================
// nRF24 ADDRESSES
// ============================================================

// Shared discovery address.
const byte DISCOVERY_ADDRESS[6] = "DISC1";

// Shared reply address.
const byte REPLY_ADDRESS[6] = "REPLY";

// ============================================================
// EEPROM RADIO ID
// ============================================================

#define EEPROM_MAGIC_ADDR 0
#define EEPROM_ID_ADDR    1
#define EEPROM_MAGIC      0xD7

struct RadioIdentity
{
    char id[10];
};

RadioIdentity identity;

// ============================================================
// TRANSMISSION STATE
// ============================================================

uint16_t txSequence = 0;

// ============================================================
// SELECTED RECEIVER
// ============================================================

char selectedReceiverID[13] = "";

bool targetSelected = false;

// ============================================================
// DISCOVERY
// ============================================================

#define MAX_DISCOVERED_RX 10
#define DISCOVERY_TIME_MS 1500

struct DiscoveredReceiver
{
    char receiverID[13];
    char vehicleName[19];
};

DiscoveredReceiver discoveredRX[MAX_DISCOVERED_RX];

uint8_t discoveredCount = 0;

// ============================================================
// FORWARD DECLARATIONS
// ============================================================

void sendControlPacket(
    int16_t steering,
    int16_t throttle
);

void handleCommand();

void discoverReceivers();

void printIdentity();

void printStatus();

bool isReceiverAlreadyDiscovered(
    const char *receiverID
);

bool buildControlAddress(
    const char *rxID,
    byte address[6]
);

void selectTarget(
    String command
);

void printTarget();

bool sendWiFiField(
    uint8_t field,
    const String &value
);

bool handleWiFiFieldCommand(
    uint8_t field,
    const String &command,
    const char *prefix
);

bool commitWiFiCredentials();

// ============================================================
// GENERATE RADIO ID
// ============================================================

void generateRadioID()
{
    unsigned long seed = 0;

    seed ^= analogRead(A0);
    seed ^= ((unsigned long)analogRead(A1) << 10);
    seed ^= ((unsigned long)analogRead(A2) << 20);
    seed ^= micros();

    randomSeed(seed);

    const char hex[] =
        "0123456789ABCDEF";

    identity.id[0] = 'R';
    identity.id[1] = 'F';
    identity.id[2] = '-';

    for (int i = 0; i < 6; i++)
    {
        identity.id[3 + i] =
            hex[random(0, 16)];
    }

    identity.id[9] = '\0';
}

// ============================================================
// LOAD RADIO ID
// ============================================================

void loadRadioID()
{
    uint8_t magic =
        EEPROM.read(
            EEPROM_MAGIC_ADDR
        );

    if (magic == EEPROM_MAGIC)
    {
        EEPROM.get(
            EEPROM_ID_ADDR,
            identity
        );

        if (
            identity.id[0] == 'R' &&
            identity.id[1] == 'F' &&
            identity.id[2] == '-'
        )
        {
            return;
        }
    }

    // No valid ID exists.
    generateRadioID();

    EEPROM.update(
        EEPROM_MAGIC_ADDR,
        EEPROM_MAGIC
    );

    EEPROM.put(
        EEPROM_ID_ADDR,
        identity
    );
}

// ============================================================
// PRINT RADIO ID
// ============================================================

void printIdentity()
{
    Serial.print(
        "RADIO_ID:"
    );

    Serial.println(
        identity.id
    );
}

// ============================================================
// PRINT RF ADDRESS
// ============================================================

void printAddress(
    const byte address[6]
)
{
    for (int i = 0; i < 5; i++)
    {
        if (address[i] < 0x10)
        {
            Serial.print("0");
        }

        Serial.print(
            address[i],
            HEX
        );

        if (i < 4)
        {
            Serial.print(":");
        }
    }
}

// ============================================================
// BUILD UNIQUE CONTROL ADDRESS FROM RX ID
// ============================================================

bool buildControlAddress(
    const char *rxID,
    byte address[6]
)
{
    // RX ID must contain exactly 12 hexadecimal characters.

    if (strlen(rxID) != 12)
    {
        return false;
    }

    for (int i = 0; i < 12; i++)
    {
        char c = rxID[i];

        bool valid =
            (c >= '0' && c <= '9') ||
            (c >= 'A' && c <= 'F') ||
            (c >= 'a' && c <= 'f');

        if (!valid)
        {
            return false;
        }
    }

    /*
     * nRF24 uses a 5-byte address.
     *
     * Address format:
     *
     * D3 + last four bytes of RX factory ID
     *
     * Example:
     *
     * 9C52F7020F3C
     *
     * becomes:
     *
     * D3:F7:02:0F:3C
     */

    address[0] = 0xD3;

    for (int i = 0; i < 4; i++)
    {
        char high =
            rxID[4 + (i * 2)];

        char low =
            rxID[5 + (i * 2)];

        byte highValue;

        byte lowValue;

        if (
            high >= '0' &&
            high <= '9'
        )
        {
            highValue =
                high - '0';
        }
        else if (
            high >= 'A' &&
            high <= 'F'
        )
        {
            highValue =
                high - 'A' + 10;
        }
        else
        {
            highValue =
                high - 'a' + 10;
        }

        if (
            low >= '0' &&
            low <= '9'
        )
        {
            lowValue =
                low - '0';
        }
        else if (
            low >= 'A' &&
            low <= 'F'
        )
        {
            lowValue =
                low - 'A' + 10;
        }
        else
        {
            lowValue =
                low - 'a' + 10;
        }

        address[i + 1] =
            (highValue << 4) |
            lowValue;
    }

    return true;
}

// ============================================================
// INITIALIZE nRF24
// ============================================================

bool initializeRadio()
{
    if (!radio.begin())
    {
        return false;
    }

    radio.setPALevel(
        RF24_PA_LOW
    );

    radio.setDataRate(
        RF24_250KBPS
    );

    radio.setChannel(
        76
    );

    radio.setAutoAck(
        true
    );

    radio.setRetries(
        5,
        15
    );

    radio.enableAckPayload();

    radio.enableDynamicPayloads();

    radio.setPayloadSize(
        32
    );

    /*
     * Initially there is no selected receiver.
     *
     * We therefore open the discovery address as the initial
     * writing pipe. Once TARGET is selected, this changes to
     * that RX's unique control address.
     */

    radio.openWritingPipe(
        DISCOVERY_ADDRESS
    );

    radio.stopListening();

    return true;
}

// ============================================================
// STATUS
// ============================================================

void printStatus()
{
    printIdentity();

    Serial.print(
        "NRF24:"
    );

    Serial.println(
        radio.isChipConnected()
        ? "OK"
        : "ERROR"
    );

    printTarget();
}

// ============================================================
// CHECK DUPLICATE RX
// ============================================================

bool isReceiverAlreadyDiscovered(
    const char *receiverID
)
{
    for (
        uint8_t i = 0;
        i < discoveredCount;
        i++
    )
    {
        if (
            strcmp(
                discoveredRX[i].receiverID,
                receiverID
            ) == 0
        )
        {
            return true;
        }
    }

    return false;
}

// ============================================================
// DISCOVER RECEIVERS
// ============================================================

void discoverReceivers()
{
    Serial.println();
    Serial.println(
        "DISCOVERY_START"
    );

    // --------------------------------------------------------
    // Clear previous discovery results
    // --------------------------------------------------------

    discoveredCount = 0;

    memset(
        discoveredRX,
        0,
        sizeof(discoveredRX)
    );

    // --------------------------------------------------------
    // Build discovery request
    // --------------------------------------------------------

    DiscoveryPacket request;

    memset(
        &request,
        0,
        sizeof(request)
    );

    memcpy(
        request.cmd,
        "DISCOVER",
        8
    );

    // --------------------------------------------------------
    // Switch to discovery TX
    // --------------------------------------------------------

    radio.stopListening();

    radio.openWritingPipe(
        DISCOVERY_ADDRESS
    );

    radio.write(
        &request,
        sizeof(request)
    );

    Serial.println(
        "DISCOVERY_TX:SENT"
    );

    // --------------------------------------------------------
    // Listen for all RX responses
    // --------------------------------------------------------

    radio.openReadingPipe(
        1,
        REPLY_ADDRESS
    );

    radio.startListening();

    unsigned long deadline =
        millis() + DISCOVERY_TIME_MS;

    while (millis() < deadline)
    {
        if (!radio.available())
        {
            continue;
        }

        DiscoveryResponse response;

        memset(
            &response,
            0,
            sizeof(response)
        );

        radio.read(
            &response,
            sizeof(response)
        );

        // ----------------------------------------------------
        // Force string termination
        // ----------------------------------------------------

        response.receiverID[
            sizeof(response.receiverID) - 1
        ] = '\0';

        response.vehicleName[
            sizeof(response.vehicleName) - 1
        ] = '\0';

        // ----------------------------------------------------
        // Validate ID
        // ----------------------------------------------------

        if (
            response.receiverID[0] == '\0'
        )
        {
            continue;
        }

        // ----------------------------------------------------
        // Ignore duplicate responses
        // ----------------------------------------------------

        if (
            isReceiverAlreadyDiscovered(
                response.receiverID
            )
        )
        {
            continue;
        }

        // ----------------------------------------------------
        // Store receiver
        // ----------------------------------------------------

        if (
            discoveredCount < MAX_DISCOVERED_RX
        )
        {
            strncpy(
                discoveredRX[discoveredCount].receiverID,
                response.receiverID,
                sizeof(
                    discoveredRX[discoveredCount]
                    .receiverID
                ) - 1
            );

            strncpy(
                discoveredRX[discoveredCount].vehicleName,
                response.vehicleName,
                sizeof(
                    discoveredRX[discoveredCount]
                    .vehicleName
                ) - 1
            );

            discoveredRX[discoveredCount]
                .receiverID[
                    sizeof(
                        discoveredRX[discoveredCount]
                        .receiverID
                    ) - 1
                ] = '\0';

            discoveredRX[discoveredCount]
                .vehicleName[
                    sizeof(
                        discoveredRX[discoveredCount]
                        .vehicleName
                    ) - 1
                ] = '\0';

            discoveredCount++;

            // ------------------------------------------------
            // Report immediately
            // ------------------------------------------------

            Serial.print(
                "RX_FOUND:"
            );

            Serial.println(
                response.receiverID
            );

            Serial.print(
                "VEHICLE_NAME:"
            );

            Serial.println(
                response.vehicleName
            );
        }
    }

    // --------------------------------------------------------
    // Restore TX mode
    // --------------------------------------------------------

    radio.stopListening();

    if (targetSelected)
    {
        byte controlAddress[6];

        if (
            buildControlAddress(
                selectedReceiverID,
                controlAddress
            )
        )
        {
            radio.openWritingPipe(
                controlAddress
            );
        }
    }
    else
    {
        radio.openWritingPipe(
            DISCOVERY_ADDRESS
        );
    }

    // --------------------------------------------------------
    // Discovery result
    // --------------------------------------------------------

    Serial.println();

    Serial.println(
        "DISCOVERY_COMPLETE"
    );

    Serial.print(
        "RX_COUNT:"
    );

    Serial.println(
        discoveredCount
    );

    if (discoveredCount == 0)
    {
        Serial.println(
            "DISCOVERY_RESULT:NONE"
        );
    }
    else
    {
        Serial.println(
            "DISCOVERY_RESULT:OK"
        );
    }

    Serial.println();
}

// ============================================================
// SELECT TARGET
// ============================================================

void selectTarget(
    String command
)
{
    int comma =
        command.indexOf(',');

    if (comma < 0)
    {
        Serial.println(
            "TARGET_INVALID"
        );

        return;
    }

    String targetID =
        command.substring(
            comma + 1
        );

    targetID.trim();

    // --------------------------------------------------------
    // Validate length
    // --------------------------------------------------------

    if (targetID.length() != 12)
    {
        Serial.println(
            "TARGET_INVALID"
        );

        return;
    }

    // --------------------------------------------------------
    // Check that target was discovered
    // --------------------------------------------------------

    bool found = false;

    for (
        uint8_t i = 0;
        i < discoveredCount;
        i++
    )
    {
        if (
            targetID.equals(
                discoveredRX[i].receiverID
            )
        )
        {
            found = true;
            break;
        }
    }

    if (!found)
    {
        Serial.print(
            "TARGET_NOT_FOUND:"
        );

        Serial.println(
            targetID
        );

        return;
    }

    // --------------------------------------------------------
    // Build unique control address
    // --------------------------------------------------------

    byte controlAddress[6];

    if (
        !buildControlAddress(
            targetID.c_str(),
            controlAddress
        )
    )
    {
        Serial.println(
            "TARGET_INVALID"
        );

        return;
    }

    // --------------------------------------------------------
    // Store selected RX
    // --------------------------------------------------------

    targetID.toCharArray(
        selectedReceiverID,
        sizeof(selectedReceiverID)
    );

    targetSelected = true;

    // --------------------------------------------------------
    // Switch Nano TX to selected RX
    // --------------------------------------------------------

    radio.stopListening();

    radio.openWritingPipe(
        controlAddress
    );

    // --------------------------------------------------------
    // Report target
    // --------------------------------------------------------

    Serial.print(
        "TARGET_SELECTED:"
    );

    Serial.println(
        selectedReceiverID
    );

    Serial.print(
        "CONTROL_ADDRESS:"
    );

    printAddress(
        controlAddress
    );

    Serial.println();
}

// ============================================================
// PRINT TARGET
// ============================================================

void printTarget()
{
    if (!targetSelected)
    {
        Serial.println(
            "TARGET:NONE"
        );

        return;
    }

    Serial.print(
        "TARGET:"
    );

    Serial.println(
        selectedReceiverID
    );

    byte controlAddress[6];

    if (
        buildControlAddress(
            selectedReceiverID,
            controlAddress
        )
    )
    {
        Serial.print(
            "CONTROL_ADDRESS:"
        );

        printAddress(
            controlAddress
        );

        Serial.println();
    }
}

// ============================================================
// SEND CONTROL PACKET
// ============================================================

void sendControlPacket(
    int16_t steering,
    int16_t throttle
)
{
    // --------------------------------------------------------
    // Safety: require selected target
    // --------------------------------------------------------

    if (!targetSelected)
    {
        Serial.println(
            "TX_BLOCKED:NO_TARGET"
        );

        return;
    }

    ControlPacket packet;

    packet.type =
        PACKET_CONTROL;

    packet.sequence =
        txSequence++;

    packet.steering =
        steering;

    packet.throttle =
        throttle;

    bool success =
        radio.write(
            &packet,
            sizeof(packet)
        );

    if (!success)
    {
        Serial.print(
            "TX_FAIL:"
        );

        Serial.println(
            packet.sequence
        );

        return;
    }

    Serial.print(
        "TX_OK:"
    );

    Serial.println(
        packet.sequence
    );

    // --------------------------------------------------------
    // ACK payload
    // --------------------------------------------------------

    if (
        radio.isAckPayloadAvailable()
    )
    {
        RadioAckPacket ack;

        memset(
            &ack,
            0,
            sizeof(ack)
        );

        radio.read(
            &ack,
            sizeof(ack)
        );

        Serial.print(
            "ACK_TYPE:"
        );

        Serial.println(
            ack.type
        );

        Serial.print(
            "ACK_SEQUENCE:"
        );

        Serial.println(
            ack.sequence
        );

        Serial.print(
            "ACK_STATUS:"
        );

        Serial.println(
            ack.status
        );

        Serial.print(
            "ACK_FAILSAFE:"
        );

        Serial.println(
            ack.failsafe
        );

        if (
            ack.type ==
            PACKET_RADIO_ACK
        )
        {
            Serial.println(
                "ACK_VALID"
            );
        }
        else
        {
            Serial.println(
                "ACK_INVALID"
            );
        }
    }
    else
    {
        Serial.println(
            "ACK_NONE"
        );
    }
}

// ============================================================
// SEND WI-FI PROVISIONING FIELD
// ============================================================

bool sendWiFiField(
    uint8_t field,
    const String &value
)
{
    if (!targetSelected)
    {
        Serial.println("WIFI_BLOCKED:NO_TARGET");
        return false;
    }

    const uint8_t CHUNK_SIZE = 28;
    uint16_t length = (uint16_t)value.length();

    if (length > 255)
    {
        Serial.println("WIFI_ERROR:TOO_LONG");
        return false;
    }

    uint8_t total =
        (uint8_t)((length + CHUNK_SIZE - 1) / CHUNK_SIZE);

    if (total == 0)
    {
        total = 1;
    }

    for (uint8_t index = 0; index < total; index++)
    {
        WiFiProvisionPacket packet;
        memset(&packet, 0, sizeof(packet));

        packet.type = PACKET_WIFI_PROVISION;
        packet.field = field;
        packet.chunkIndex = index;
        packet.chunkTotal = total;

        uint16_t offset =
            (uint16_t)index * CHUNK_SIZE;

        uint16_t remaining =
            length - offset;

        uint8_t count =
            (remaining > CHUNK_SIZE)
            ? CHUNK_SIZE
            : (uint8_t)remaining;

        if (count > 0)
        {
            memcpy(
                packet.data,
                value.c_str() + offset,
                count
            );
        }

        if (!radio.write(&packet, sizeof(packet)))
        {
            Serial.print("WIFI_TX_FAIL:");
            Serial.println(index);
            return false;
        }

        Serial.print("WIFI_TX:");
        Serial.print(field);
        Serial.print(":");
        Serial.print(index + 1);
        Serial.print("/");
        Serial.println(total);

        delay(5);
    }

    return true;
}

// ============================================================
// HANDLE WI-FI FIELD COMMAND
// ============================================================

bool handleWiFiFieldCommand(
    uint8_t field,
    const String &command,
    const char *prefix
)
{
    String prefixText = String(prefix);

    if (!command.startsWith(prefixText))
    {
        Serial.println("WIFI_INVALID");
        return false;
    }

    String value =
        command.substring(prefixText.length());

    value.trim();

    if (field == 1 || field == 4)
    {
        if (value.length() == 0 || value.length() > 32)
        {
            Serial.println("WIFI_INVALID:SSID");
            return false;
        }
    }
    else if (field == 2 || field == 5)
    {
        if (value.length() > 63)
        {
            Serial.println("WIFI_INVALID:PASSWORD");
            return false;
        }
    }
    else
    {
        Serial.println("WIFI_INVALID:FIELD");
        return false;
    }

    return sendWiFiField(field, value);
}

// ============================================================
// SEND WI-FI COMMIT
// ============================================================

bool commitWiFiCredentials()
{
    if (!targetSelected)
    {
        Serial.println("WIFI_BLOCKED:NO_TARGET");
        return false;
    }

    WiFiProvisionPacket packet;
    memset(&packet, 0, sizeof(packet));

    packet.type = PACKET_WIFI_PROVISION;
    packet.field = 3;
    packet.chunkIndex = 0;
    packet.chunkTotal = 1;

    if (!radio.write(&packet, sizeof(packet)))
    {
        Serial.println("WIFI_TX_FAIL:COMMIT");
        return false;
    }

    Serial.println("WIFI_COMMIT_SENT");
    return true;
}

// ============================================================
// SEND SESSION PACKET
// ============================================================

bool sendSessionPacket(bool active, uint32_t durationSeconds)
{
    if (!targetSelected)
    {
        Serial.println("SESSION_BLOCKED:NO_TARGET");
        return false;
    }

    SessionPacket packet;
    memset(&packet, 0, sizeof(packet));

    packet.type = PACKET_SESSION;
    packet.active = active ? 1 : 0;
    packet.durationSeconds = durationSeconds;

    if (!radio.write(&packet, sizeof(packet)))
    {
        Serial.println("SESSION_TX_FAIL");
        return false;
    }

    if (active)
    {
        Serial.print("SESSION_START_SENT:");
        Serial.println(durationSeconds);
    }
    else
    {
        Serial.println("SESSION_STOP_SENT");
    }

    return true;
}

// ============================================================
// HANDLE SERIAL COMMAND
// ============================================================

void handleCommand()
{
    if (!Serial.available())
    {
        return;
    }

    String command =
        Serial.readStringUntil(
            '\n'
        );

    command.trim();

    // --------------------------------------------------------
    // WHO
    // --------------------------------------------------------

    if (
        command == "WHO"
    )
    {
        printIdentity();

        return;
    }

    // --------------------------------------------------------
    // PING
    // --------------------------------------------------------

    if (
        command == "PING"
    )
    {
        Serial.println(
            "PONG"
        );

        return;
    }

    // --------------------------------------------------------
    // STATUS
    // --------------------------------------------------------

    if (
        command == "STATUS"
    )
    {
        printStatus();

        return;
    }

    // --------------------------------------------------------
    // DISCOVER
    // --------------------------------------------------------

    if (
        command == "DISCOVER"
    )
    {
        discoverReceivers();

        return;
    }

    // --------------------------------------------------------
    // TARGET
    //
    // TARGET,<RX_ID>
    //
    // Example:
    // TARGET,9C52F7020F3C
    // --------------------------------------------------------

    if (
        command.startsWith(
            "TARGET,"
        )
    )
    {
        selectTarget(
            command
        );

        return;
    }

    // --------------------------------------------------------
    // TARGET STATUS
    // --------------------------------------------------------

    if (
        command == "TARGET"
    )
    {
        printTarget();

        return;
    }

    // --------------------------------------------------------
    // CONTROL
    //
    // CONTROL,steering,throttle
    //
    // Example:
    // CONTROL,1000,2000
    // --------------------------------------------------------

    if (
        command.startsWith(
            "CONTROL,"
        )
    )
    {
        int firstComma =
            command.indexOf(',');

        int secondComma =
            command.indexOf(
                ',',
                firstComma + 1
            );

        if (
            firstComma < 0 ||
            secondComma < 0
        )
        {
            Serial.println(
                "CONTROL_INVALID"
            );

            return;
        }

        String steeringText =
            command.substring(
                firstComma + 1,
                secondComma
            );

        String throttleText =
            command.substring(
                secondComma + 1
            );

        steeringText.trim();
        throttleText.trim();

        long steeringValue =
            steeringText.toInt();

        long throttleValue =
            throttleText.toInt();

        // ----------------------------------------------------
        // Clamp to signed 16-bit
        // ----------------------------------------------------

        steeringValue =
            constrain(
                steeringValue,
                -32768L,
                32767L
            );

        throttleValue =
            constrain(
                throttleValue,
                -32768L,
                32767L
            );

        sendControlPacket(
            (int16_t)steeringValue,
            (int16_t)throttleValue
        );

        return;
    }

    // --------------------------------------------------------
    // SESSION_START,<seconds>
    // SESSION_STOP
    // --------------------------------------------------------

    if (command.startsWith("SESSION_START,"))
    {
        String durationText = command.substring(14);
        durationText.trim();

        unsigned long durationSeconds = durationText.toInt();

        if (durationSeconds == 0)
        {
            Serial.println("SESSION_INVALID");
            return;
        }

        sendSessionPacket(true, (uint32_t)durationSeconds);
        return;
    }

    if (command == "SESSION_STOP")
    {
        sendSessionPacket(false, 0);
        return;
    }

    // --------------------------------------------------------
    // WIFI_SSID,<SSID>
    // WIFI_PASSWORD,<PASSWORD>
    // WIFI_FALLBACK_SSID,<SSID>
    // WIFI_FALLBACK_PASSWORD,<PASSWORD>
    // WIFI_COMMIT
    // --------------------------------------------------------

    if (command.startsWith("WIFI_SSID,"))
    {
        handleWiFiFieldCommand(1, command, "WIFI_SSID,");
        return;
    }

    if (command.startsWith("WIFI_PASSWORD,"))
    {
        handleWiFiFieldCommand(2, command, "WIFI_PASSWORD,");
        return;
    }

    if (command.startsWith("WIFI_FALLBACK_SSID,"))
    {
        handleWiFiFieldCommand(4, command, "WIFI_FALLBACK_SSID,");
        return;
    }

    if (command.startsWith("WIFI_FALLBACK_PASSWORD,"))
    {
        handleWiFiFieldCommand(5, command, "WIFI_FALLBACK_PASSWORD,");
        return;
    }

    if (command == "WIFI_COMMIT")
    {
        commitWiFiCredentials();
        return;
    }

    // --------------------------------------------------------
    // TEST
    // --------------------------------------------------------

    if (
        command == "TEST"
    )
    {
        sendControlPacket(
            1234,
            5678
        );

        return;
    }

    // --------------------------------------------------------
    // Unknown
    // --------------------------------------------------------

    Serial.print(
        "UNKNOWN:"
    );

    Serial.println(
        command
    );
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(
        115200
    );

    delay(500);

    loadRadioID();

    Serial.println();
    Serial.println(
        "=============================="
    );

    Serial.println(
        "DriveMatrix Nano Radio"
    );

    Serial.println(
        "=============================="
    );

    printIdentity();

    Serial.print(
        "ControlPacket size: "
    );

    Serial.println(
        sizeof(ControlPacket)
    );

    Serial.print(
        "RadioAckPacket size: "
    );

    Serial.println(
        sizeof(RadioAckPacket)
    );

    Serial.print(
        "WiFiProvisionPacket size: "
    );

    Serial.println(
        sizeof(WiFiProvisionPacket)
    );

    Serial.print(
        "DiscoveryPacket size: "
    );

    Serial.println(
        sizeof(DiscoveryPacket)
    );

    Serial.print(
        "DiscoveryResponse size: "
    );

    Serial.println(
        sizeof(DiscoveryResponse)
    );

    Serial.println(
        "Initializing nRF24..."
    );

    if (
        initializeRadio()
    )
    {
        Serial.println(
            "NRF24:OK"
        );

        Serial.println(
            "READY"
        );
    }
    else
    {
        Serial.println(
            "NRF24:ERROR"
        );

        while (1)
        {
            delay(1000);
        }
    }
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
    handleCommand();
}