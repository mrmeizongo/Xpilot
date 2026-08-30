/* ============================================
Flight stabilization software
    Copyright (C) 2024 Jamal Meizongo (mrmeizongo@outlook.com)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
===============================================
*/
#ifndef _SERIAL_CONFIG_TASK_H
#define _SERIAL_CONFIG_TASK_H
#include <Arduino.h>
#include "ConfigManager.h"
#include "SerialProtocol.h"

class SerialConfigTask
{
public:
    SerialConfigTask(
        HardwareSerial &serial,
        ConfigManager &configManager);

    void run();

private:
    enum class RxState : uint8_t
    {
        WAITING_FOR_START,
        RECEIVING_PACKET
    };

    HardwareSerial &_serial;
    ConfigManager &_configManager;

    RxState _rxState;

    uint8_t _rxBuffer[SERIAL_PACKET_SIZE];

    uint8_t _rxIndex;

    void processByte(uint8_t byte);

    void processPacket(
        const SerialPacket &packet);

    void processGet(
        const SerialPacket &packet);

    void processSet(
        const SerialPacket &packet);

    void sendValue(
        ConfigID id);

    void sendAck(
        SerialCommand originalCommand, SerialCommand ack = SerialCommand::ACK);

    void sendPacket(
        SerialPacket &packet);

    static uint8_t calculateChecksum(
        const uint8_t *data,
        uint8_t length);
};
#endif //_SERIAL_CONFIG_TASK_H