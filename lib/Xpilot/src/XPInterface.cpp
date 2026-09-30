#include <string.h>
#include "IMU.h"
#include "Radio.h"
#include "XPInterface.h"
#include "LEDnotifier.h"
#include "FlightConfigAccess.h"

XPInterface::XPInterface(HardwareSerial& serial)
    : _serial(serial)
    , _rxState(RxState::WAITING_FOR_START)
    , _rxBuffer{}
    , _rxIndex(0)
    , _radioStreaming(false)
{
}

void XPInterface::run()
{
    constexpr uint8_t MAX_BYTES_PER_RUN = PACKET_SIZE;

    uint8_t processed = 0;

    // Process one packet per run
    while (_serial.available() > 0 && processed < MAX_BYTES_PER_RUN)
    {
        processByte(static_cast<uint8_t>(_serial.read()));
        processed++;
    }

    // Stream one snapshot per run
    if (_radioStreaming)
    {
        sendRadioSnapshot();
    }
}

void XPInterface::processByte(uint8_t byte)
{
    switch (_rxState)
    {
        case RxState::WAITING_FOR_START:
        {
            if (byte == PACKET_START)
            {
                _rxIndex = 0;
                _rxBuffer[_rxIndex++] = byte;
                _rxState = RxState::RECEIVING_PACKET;
            }

            break;
        }

        case RxState::RECEIVING_PACKET:
        {
            _rxBuffer[_rxIndex++] = byte;

            if (_rxIndex >= PACKET_SIZE)
            {
                Packet packet;
                memcpy(&packet, _rxBuffer, PACKET_SIZE);

                const uint8_t calculated = calculateChecksum(_rxBuffer, PACKET_SIZE - 1);

                if (calculated == packet.checksum)
                    processPacket(packet);

                _rxState = RxState::WAITING_FOR_START;
            }

            break;
        }

        default:
            break;
    }
}

void XPInterface::processPacket(const Packet& packet)
{
    const Command command = static_cast<Command>(packet.command);

    switch (command)
    {
        case Command::GET:
            processGet(packet);
            break;

        case Command::SET:
            processSet(packet);
            break;

        case Command::SAVE:
        {
            sendAck(command, configManager.save() ? Command::ACK : Command::NACK);
            break;
        }

        case Command::LOAD:
        {
            sendAck(command, configManager.load() ? Command::ACK : Command::NACK);
            break;
        }

        case Command::DEFAULTS:
        {
            configManager.loadDefaults();
            sendAck(command);
            break;
        }

        case Command::CALIBRATE_IMU:
        {
            imu.calibrate();
            sendAck(command);
            break;
        }

        case Command::START_RADIO_STREAM:
            processStartRadioStream();
            break;

        case Command::STOP_RADIO_STREAM:
            processStopRadioStream();
            break;

        default:
            sendAck(command, Command::NACK);
            break;
    }

    LEDNotifier::blinkLED(SUCCESS_BLINK_COUNT, SUCCESS_BLINK_DURATION);
}

void XPInterface::processGet(const Packet& packet)
{
    if (packet.paramId >= static_cast<uint8_t>(ConfigID::COUNT))
    {
        sendAck(Command::GET, Command::NACK);
        return;
    }

    sendValue(static_cast<ConfigID>(packet.paramId));
}

void XPInterface::processSet(const Packet& packet)
{
    if (packet.paramId >= static_cast<uint8_t>(ConfigID::COUNT))
    {
        sendAck(Command::SET, Command::NACK);
        return;
    }

    const ConfigID id = static_cast<ConfigID>(packet.paramId);
    ConfigValue value{};
    memcpy(&value.raw, packet.value, sizeof(value.raw));

    sendAck(Command::SET, configManager.set(id, value) ? Command::ACK : Command::NACK);
}

void XPInterface::processStartRadioStream()
{
    _radioStreaming = true;
    sendAck(Command::START_RADIO_STREAM);
}

void XPInterface::processStopRadioStream()
{
    _radioStreaming = false;
    sendAck(Command::STOP_RADIO_STREAM);
}

void XPInterface::sendValue(ConfigID id)
{
    ConfigValue value{};
    ConfigValueType type;

    if (!configManager.get(id, value, type))
    {
        sendAck(Command::GET, Command::NACK);
        return;
    }

    Packet packet{};
    packet.start = PACKET_START;
    packet.command = static_cast<uint8_t>(Command::VALUE);
    packet.paramId = static_cast<uint8_t>(id);
    packet.type = static_cast<uint8_t>(type);
    memcpy(packet.value, &value.raw, sizeof(value.raw));

    sendPacket(packet);
}

void XPInterface::sendRadioSnapshot()
{
    // Request all 6 channels
    const uint8_t channel_count = 6;

    for (uint8_t channel = 0; channel < channel_count; channel++)
        sendRadioValue(channel, radio.getPWM(static_cast<Radio::CHANNEL>(channel)));
}

void XPInterface::sendRadioValue(uint8_t channel, uint16_t pwm)
{
    Packet packet{};
    packet.start = PACKET_START;
    packet.command = static_cast<uint8_t>(Command::RADIO_VALUE);
    packet.paramId = channel;
    packet.type = static_cast<uint8_t>(ConfigValueType::UINT16);
    memcpy(packet.value, &pwm, sizeof(pwm));

    sendPacket(packet);
}

void XPInterface::sendAck(Command originalCommand, Command ack)
{
    Packet packet{};
    packet.start = PACKET_START;
    packet.command = static_cast<uint8_t>(ack);
    packet.paramId = static_cast<uint8_t>(originalCommand);

    sendPacket(packet);
}

void XPInterface::sendPacket(Packet& packet)
{
    packet.checksum = calculateChecksum(reinterpret_cast<const uint8_t*>(&packet), PACKET_SIZE - 1);
    _serial.write(reinterpret_cast<const uint8_t*>(&packet), PACKET_SIZE);
}

uint8_t XPInterface::calculateChecksum(const uint8_t* data, uint8_t length)
{
    uint8_t checksum = 0;

    for (uint8_t i = 0; i < length; i++)
        checksum += data[i];

    return checksum;
}
