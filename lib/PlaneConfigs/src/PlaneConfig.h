// 11/18/2024 by Jamal Meizongo (mrmeizongo@outlook.com)
// This and other library code in this repository
// are partial releases and work is still in progress.
// Please keep this in mind as you use this piece of software.

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
#ifndef _PLANE_CONFIG_H
#define _PLANE_CONFIG_H
#include "PlaneSelector.h"

/*
 * Use conditional compilation based on airplane definitions in PlaneSelector
 * Example
 * #if defined(USE_GLIDER) || defined(USE_CUB) || defined(USE_ACRO)
 * #error Only one airplane need to be defined at a time
 * #endif
 *
 * #if defined(USE_GLIDER)
 * #include "Glider.h"
 * #undef USE_GLIDER
 * #elif defined(USE_CUB)
 * #include "Cub.h"
 * #undef USE_CUB
 * #elif defined(USE_ACRO)
 * #include "Acro.h"
 * #undef USE_ACRO
 * #else
 * #include "DefaultConfig.h"
 * #endif
 */

/*
 * NOTE
 * If you intend to modify and use only the default settings in DefaultConfig.h you can leave this file as is
 */
#include "DefaultConfig.h"

#endif // PLANE_CONFIG_H