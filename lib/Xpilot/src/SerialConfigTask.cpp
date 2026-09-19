#include <string.h>
#include "IMU.h"
#include "Radio.h"
#include "ConfigManager.h"
#include "SerialConfigTask.h"
#include "FlightConfigAccess.h"

SerialConfigTask::SerialConfigTask(HardwareSerial& serial)
    : _serial(serial)
    , _rxState(RxState::WAITING_FOR_START)
    , _rxBuffer{}
    , _rxIndex(0)
    , _radioStreaming(false)
{
}

void SerialConfigTask::run()
{
    constexpr uint8_t MAX_BYTES_PER_RUN = 18;

    uint8_t processed = 0;

    while (_serial.available() > 0 && processed < MAX_BYTES_PER_RUN)
    {
        processByte(static_cast<uint8_t>(_serial.read()));
        processed++;
    }

    if (_radioStreaming)
    {
        sendRadioSnapshot();
    }
}

void SerialConfigTask::processByte(uint8_t byte)
{
    switch (_rxState)
    {
        case RxState::WAITING_FOR_START:
        {
            if (byte == SERIAL_PACKET_START)
            {
                _rxBuffer[0] = byte;
                _rxIndex = 1;
                _rxState = RxState::RECEIVING_PACKET;
            }

            break;
        }

        case RxState::RECEIVING_PACKET:
        {
            _rxBuffer[_rxIndex++] = byte;

            if (_rxIndex >= SERIAL_PACKET_SIZE)
            {
                SerialPacket packet;
                memcpy(&packet, _rxBuffer, SERIAL_PACKET_SIZE);

                const uint8_t calculated = calculateChecksum(_rxBuffer, SERIAL_PACKET_SIZE - 1);

                if (calculated == packet.checksum)
                    processPacket(packet);

                _rxIndex = 0;
                _rxState = RxState::WAITING_FOR_START;
            }

            break;
        }

        default:
            break;
    }
}

void SerialConfigTask::processPacket(const SerialPacket& packet)
{
    const SerialCommand command = static_cast<SerialCommand>(packet.command);

    switch (command)
    {
        case SerialCommand::GET:
            processGet(packet);
            break;

        case SerialCommand::SET:
            processSet(packet);
            break;

        case SerialCommand::SAVE:
        {
            sendAck(command, configManager.save() ? SerialCommand::ACK : SerialCommand::NACK);
            break;
        }

        case SerialCommand::LOAD:
        {
            sendAck(command, configManager.load() ? SerialCommand::ACK : SerialCommand::NACK);
            break;
        }

        case SerialCommand::DEFAULTS:
        {
            configManager.loadDefaults();
            sendAck(command);
            break;
        }

        case SerialCommand::CALIBRATE_IMU:
        {
            imu.calibrate();

            float accel[IMU::Axis::AXIS_COUNT], gyro[IMU::Axis::AXIS_COUNT];
            imu.getCalibration(accel, gyro);
            configManager.setIMUCalibration(accel, gyro);

            sendAck(command);
            break;
        }

        case SerialCommand::START_RADIO_STREAM:
            processStartRadioStream();
            break;

        case SerialCommand::STOP_RADIO_STREAM:
            processStopRadioStream();
            break;

        default:
            sendAck(command, SerialCommand::NACK);
            break;
    }
}

void SerialConfigTask::processGet(const SerialPacket& packet)
{
    if (packet.paramId >= static_cast<uint8_t>(ConfigID::COUNT))
    {
        sendAck(SerialCommand::GET, SerialCommand::NACK);
        return;
    }

    sendValue(static_cast<ConfigID>(packet.paramId));
}

void SerialConfigTask::processSet(const SerialPacket& packet)
{
    if (packet.paramId >= static_cast<uint8_t>(ConfigID::COUNT))
    {
        sendAck(SerialCommand::SET, SerialCommand::NACK);
        return;
    }

    const ConfigID id = static_cast<ConfigID>(packet.paramId);
    ConfigValue value{};
    memcpy(&value.raw, packet.value, sizeof(value.raw));

    sendAck(SerialCommand::SET, configManager.set(id, value) ? SerialCommand::ACK : SerialCommand::NACK);
}

void SerialConfigTask::processStartRadioStream()
{
    _radioStreaming = true;
    sendAck(SerialCommand::START_RADIO_STREAM);
}

void SerialConfigTask::processStopRadioStream()
{
    _radioStreaming = false;
    sendAck(SerialCommand::STOP_RADIO_STREAM);
}

void SerialConfigTask::sendValue(ConfigID id)
{
    ConfigValue value{};
    ConfigValueType type;

    if (!configManager.get(id, value, type))
    {
        sendAck(SerialCommand::GET, SerialCommand::NACK);
        return;
    }

    SerialPacket packet{};
    packet.start = SERIAL_PACKET_START;
    packet.command = static_cast<uint8_t>(SerialCommand::VALUE);
    packet.paramId = static_cast<uint8_t>(id);
    packet.type = static_cast<uint8_t>(type);
    memcpy(packet.value, &value.raw, sizeof(value.raw));

    sendPacket(packet);
}

void SerialConfigTask::sendRadioSnapshot()
{
    // Request 4 radio channels, i.e. throttle, roll, pitch and yaw
    const uint8_t channel_count = 4;
    uint16_t pwm[channel_count];

    if (!radio.getValidControlPWM(pwm, channel_count))
    {
        _radioStreaming = false;
        sendAck(SerialCommand::START_RADIO_STREAM, SerialCommand::NACK);
        return;
    }

    for (uint8_t channel = 0; channel < 4; ++channel)
        sendRadioValue(channel, pwm[channel]);
}

void SerialConfigTask::sendRadioValue(uint8_t channel, uint16_t pwm)
{
    SerialPacket packet{};
    packet.start = SERIAL_PACKET_START;
    packet.command = static_cast<uint8_t>(SerialCommand::RADIO_VALUE);
    packet.paramId = channel;
    packet.type = static_cast<uint8_t>(ConfigValueType::UINT16);
    memcpy(packet.value, &pwm, sizeof(pwm));

    sendPacket(packet);
}

void SerialConfigTask::sendAck(SerialCommand originalCommand, SerialCommand ack)
{
    SerialPacket packet{};
    packet.start = SERIAL_PACKET_START;
    packet.command = static_cast<uint8_t>(ack);
    packet.paramId = static_cast<uint8_t>(originalCommand);

    sendPacket(packet);
}

void SerialConfigTask::sendPacket(SerialPacket& packet)
{
    packet.checksum = calculateChecksum(reinterpret_cast<const uint8_t*>(&packet), SERIAL_PACKET_SIZE - 1);
    _serial.write(reinterpret_cast<const uint8_t*>(&packet), SERIAL_PACKET_SIZE);
}

uint8_t SerialConfigTask::calculateChecksum(const uint8_t* data, uint8_t length)
{
    uint8_t checksum = 0;

    for (uint8_t i = 0; i < length; i++)
        checksum += data[i];

    return checksum;
}
