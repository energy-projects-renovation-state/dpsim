/** CIM Test
 *
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
 *
 * DPsim
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************************/

#include <iostream>
#include <list>

#include "DPsim_MNA.h"

using namespace DPsim;

int main(int argc, char *argv[]) {
	std::list<String> filenames;

	for (int i = 1; i < argc; i++) {
		std::cout << "Adding file: " << argv[i] << std::endl;
		filenames.push_back(String(argv[i]));
	}

	MnaSimulation sim("CIM", filenames, 50, 0.0001, 0.1, SimulationType::DP, Logger::Level::DEBUG);
	sim.run();

	return 0;
}