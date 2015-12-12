// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"


/*
  base class constructor.
  This incorporates initialisation as well.
*/
AP_BattMonitor_Backend::AP_BattMonitor_Backend(AP_BattMonitor &mon, uint8_t instance, AP_BattMonitor::BattMonitor_State &mon_state) :
        _mon(mon),
        _state(mon_state),
        _instance(instance)
{
}

/// capacity_remaining_pct - returns the % battery capacity remaining (0 ~ 100)
uint8_t AP_BattMonitor_Backend::capacity_remaining_pct() const
{
    float mah_remaining = _state.initial_capacity - _state.current_total_mah;
    if ( _mon._pack_capacity[_instance] > 10 ) { // a very very small battery
        return (100 * (mah_remaining) / _mon._pack_capacity[_instance]);
    } else {
        return 0;
    }
}

/// set capacity for this instance
void AP_BattMonitor_Backend::set_capacity(uint32_t capacity)
{
    _mon._pack_capacity[_instance] = capacity;
}

/*
*   This function is needed to check when is possible to use voltage to make a rough estimation of the initial battery capacity
*   Voltage is considered stable when it have fluctuated less than 0.2 volts for 2 seconds under no load (less than 1 amp draw)
*/
bool AP_BattMonitor_Backend::is_voltage_stable() {

    // get current time
    uint32_t tnow = AP_HAL::millis();

    if (_state.current_amps > 1) {
        // If drawing more than 1 amp we skip the proccess
        _state.vstable = 0;
        _state.tstable = tnow;
    }
    else if (_state.vstable > 0 && _state.vstable > _state.voltage - 0.2 && _state.vstable < _state.voltage + 0.2) {
        if (tnow - _state.tstable > 2000) {
            return true;
        }
    }
    else {
        _state.vstable = _state.voltage;
        _state.tstable = tnow;
    }

    return false;
}

/*
*   This function makes a rough estimation of initial battery capacity based on its voltage under no load. It should be called once voltage is stable
*   It is based on the empirical formula obtained from measures from 3.65v to 4v in a single cell LiPo.
*   Within this range the relation voltage/capacity_percentage is linear. Outside this range we assume battery is at 100% (full) or 0% (empty)
*/
void AP_BattMonitor_Backend::calculate_initial_capacity()
{
    if (_mon._capacity_estimation[_instance] == 0 )
    {
        // This shouldn't be call if estimation disabled. Just in case we set to pack_capacity
        _state.initial_capacity = _mon._pack_capacity[_instance];
    }
    else
    {
        uint8_t num_cells = _mon._capacity_estimation[_instance];	// If estimation enabled, this parameter tell the number of cells
        float capacity_percentage = _mon._est_gain[_instance]*_state.voltage/num_cells + _mon._est_offset[_instance];	// Empirical formula

        capacity_percentage = constrain_float(capacity_percentage, 0, 100);

        _state.initial_capacity = _mon._pack_capacity[_instance] * capacity_percentage/100.0f;
    }
}
