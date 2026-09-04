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

/**
 * Byte 0       START        0xAA
 * Byte 1       COMMAND
 * Byte 2       PARAM ID
 * Byte 3       TYPE
 * Byte 4-7     VALUE
 * Byte 8       CHECKSUM
 *
 * Every packet is exactly 9 bytes
 */

#ifndef _SERIAL_PROTOCOL_H
#define _SERIAL_PROTOCOL_H
#include <stdint.h>

enum class SerialCommand : uint8_t
{
    GET = 0x01,
    SET = 0x02,
    SAVE = 0x03,
    LOAD = 0x04,
    DEFAULTS = 0x05,
    CALIBRATE_IMU = 0x06,

    ACK = 0x80,
    NACK = 0x81,
    VALUE = 0x82
};

struct SerialPacket
{
    uint8_t start;
    uint8_t command;
    uint8_t paramId;
    uint8_t type;

    uint8_t value[4];

    uint8_t checksum;
};

constexpr uint8_t SERIAL_PACKET_START = 0xAA;

constexpr uint8_t SERIAL_PACKET_SIZE = sizeof(SerialPacket);
#endif //_SERIAL_PROTOCOL_H