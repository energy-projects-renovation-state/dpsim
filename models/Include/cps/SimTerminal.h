/**
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
 *
 * CPowerSystems
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

#pragma once

#include <cps/TopologicalTerminal.h>
#include <cps/SimNode.h>

namespace CPS {

	template <typename VarType>
	class SimTerminal :
		public TopologicalTerminal,
		public SharedFactory<SimTerminal<VarType>> {
	protected:
		MatrixVar<VarType> mCurrent;
		std::weak_ptr<SimNode<VarType>> mNode;

	public:
		typedef std::shared_ptr<SimTerminal<VarType>> Ptr;
		typedef std::vector<Ptr> List;
		///
		SimTerminal(String name) : TopologicalTerminal(name, name) { }
		///
		SimTerminal(String uid, String name) : TopologicalTerminal(uid, name) { }
		///
		typename SimNode<VarType>::Ptr node() {
			return mNode.lock();
		}
		///
		void setNode(typename SimNode<VarType>::Ptr node) {
			mNode = node;
			setPhaseType(node->phaseType());
		}
		///
		TopologicalNode::Ptr topologicalNodes() { return node();	}
		///
		VarType singleVoltage() {
			if (node()->isGround())
				return 0.;
			else
				return node()->singleVoltage(mPhaseType);
		}
		///
		MatrixVar<VarType> voltage() {
			if (node()->isGround())
				return { 0., 0., 0. };
			else
				return node()->voltage();
		}
	};
}