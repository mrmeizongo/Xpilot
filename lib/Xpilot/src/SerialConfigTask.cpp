#include <string.h>
#include "IMU.h"
#include "SerialConfigTask.h"

SerialConfigTask::SerialConfigTask(
    HardwareSerial &serial,
    ConfigManager &configManager)
    : _serial(serial),
      _configManager(configManager),
      _rxState(RxState::WAITING_FOR_START),
      _rxBuffer{},
      _rxIndex(0)
{
}

void SerialConfigTask::run()
{
    constexpr uint8_t MAX_BYTES_PER_RUN = 18;

    uint8_t processed = 0;

    while (_serial.available() > 0 &&
           processed < MAX_BYTES_PER_RUN)
    {
        processByte(static_cast<uint8_t>(_serial.read()));
        processed++;
    }
}

void SerialConfigTask::processByte(
    uint8_t byte)
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

            memcpy(
                &packet,
                _rxBuffer,
                SERIAL_PACKET_SIZE);

            const uint8_t calculated =
                calculateChecksum(
                    _rxBuffer,
                    SERIAL_PACKET_SIZE - 1);

            if (calculated == packet.checksum)
            {
                processPacket(packet);
            }

            /*
             * Whether valid or invalid, return
             * to searching for the next start byte.
             */
            _rxIndex = 0;

            _rxState =
                RxState::WAITING_FOR_START;
        }

        break;
    }
    default:
        break;
    }
}

void SerialConfigTask::processPacket(
    const SerialPacket &packet)
{
    const SerialCommand command =
        static_cast<SerialCommand>(
            packet.command);

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
        if (_configManager.save())
        {
            sendAck(command);
        }
        else
        {
            sendAck(command, SerialCommand::NACK);
        }

        break;
    }

    case SerialCommand::LOAD:
    {
        if (_configManager.load())
        {
            sendAck(command);
        }
        else
        {
            sendAck(command, SerialCommand::NACK);
        }

        break;
    }

    case SerialCommand::DEFAULTS:
    {
        _configManager.loadDefaults();

        sendAck(command);

        break;
    }

    case SerialCommand::CALIBRATE_IMU:
    {
        imu.calibrate();

        float accel[3], gyro[3];
        imu.getCalibration(accel, gyro);

        _configManager.setIMUCalibration(accel, gyro);

        sendAck(command);
        break;
    }

    default:
        sendAck(command, SerialCommand::NACK);
        break;
    }
}

void SerialConfigTask::processGet(
    const SerialPacket &packet)
{
    if (packet.paramId >=
        static_cast<uint8_t>(ConfigID::COUNT))
    {
        sendAck(SerialCommand::GET, SerialCommand::NACK);
        return;
    }

    const ConfigID id =
        static_cast<ConfigID>(
            packet.paramId);

    sendValue(id);
}

void SerialConfigTask::processSet(
    const SerialPacket &packet)
{
    if (packet.paramId >=
        static_cast<uint8_t>(ConfigID::COUNT))
    {
        sendAck(SerialCommand::SET, SerialCommand::NACK);
        return;
    }

    const ConfigID id =
        static_cast<ConfigID>(
            packet.paramId);

    ConfigValue value{};

    memcpy(
        &value.raw,
        packet.value,
        sizeof(value.raw));

    if (_configManager.set(id, value))
    {
        sendAck(
            SerialCommand::SET);
    }
    else
    {
        sendAck(
            SerialCommand::SET, SerialCommand::NACK);
    }
}

void SerialConfigTask::sendValue(
    ConfigID id)
{
    ConfigValue value{};
    ConfigValueType type;

    if (!_configManager.get(
            id,
            value,
            type))
    {
        sendAck(
            SerialCommand::GET, SerialCommand::NACK);

        return;
    }

    SerialPacket packet{};

    packet.start =
        SERIAL_PACKET_START;

    packet.command =
        static_cast<uint8_t>(
            SerialCommand::VALUE);

    packet.paramId =
        static_cast<uint8_t>(id);

    packet.type =
        static_cast<uint8_t>(type);

    memcpy(
        packet.value,
        &value.raw,
        sizeof(value.raw));

    sendPacket(packet);
}

void SerialConfigTask::sendAck(
    SerialCommand originalCommand, SerialCommand ack)
{
    SerialPacket packet{};

    packet.start =
        SERIAL_PACKET_START;

    packet.command =
        static_cast<uint8_t>(
            ack);

    packet.paramId =
        static_cast<uint8_t>(
            originalCommand);

    sendPacket(packet);
}

void SerialConfigTask::sendPacket(
    SerialPacket &packet)
{
    packet.checksum =
        calculateChecksum(
            reinterpret_cast<const uint8_t *>(
                &packet),
            SERIAL_PACKET_SIZE - 1);

    _serial.write(
        reinterpret_cast<const uint8_t *>(
            &packet),
        SERIAL_PACKET_SIZE);
}

uint8_t SerialConfigTask::calculateChecksum(
    const uint8_t *data,
    uint8_t length)
{
    uint8_t checksum = 0;

    for (uint8_t i = 0; i < length; i++)
    {
        checksum += data[i];
    }

    return checksum;
}