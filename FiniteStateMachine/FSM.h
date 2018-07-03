/**
* \file
* \brief Header file with class description for finite-state machine.
*/
#ifndef FINITE_STATE_MACHINE_FANUC_TIMUR
#define FINITE_STATE_MACHINE_FANUC_TIMUR

#include <ArucoMarkers.h>
#include <CamCalibWI.h>
#include <fanucModel.h>
#include <Fanuc.h>
#include <TrajectoryMovement.h>

#include <array>
#include <string>
#include <vector>

namespace timur
{
	/**
	* \brief Class for working with finite-state machine.
	*/
	class StateMachine
	{
	private:



		/**
		* \brief Initializing a pointer to class methods(states).
		*/
		using States = void(timur::StateMachine::*)();

		/**
		* \brief A table containing pointers to class methods.
		*
		* The current state number and signal are used as table indexes.
		*/
		std::array<std::array<States, 3>, 6> _fsmTable;

		/**
		* \brief Current state number.
		*/
		int _currentState;

		/**
		* \brief Actions to the transition to the first state.
		* \param signal
		*/
		void toState1();

		/**
		* \brief Actions to the transition to the second state.
		* \param signal
		*/
		void toState2();

		/**
		* \brief Actions to the transition to the third state.
		* \param signal
		*/
		void toState3();

		/**
		* \brief Actions to the transition to the fourth state.
		* \param signal
		*/
		void toState4();

		/**
		* \brief Actions to the transition to the fifth state.
		* \param signal
		*/
		void toState5();

		/**
		* \brief Actions to the transition to the undefined state.
		* \param signal
		*/
		void toUndefinedState();

		/**
		* \brief Translating received symbols in indexes
		* \param[in] preSignal Character derived from string.
		* \return An integer corresponding to this character.
		*/
		static ptrdiff_t interpreter();
	public:

		StateMachine();
		/**
		* \brief Search in the input string of all substrings that satisfy the state machine.
		* \param inputString The string in which you want to find substrings.
		* \return Array containing all found substrings.
		*/
		void  startWorking();
	};
}

#endif // FINITE_STATE_MACHINE_FANUC_TIMUR