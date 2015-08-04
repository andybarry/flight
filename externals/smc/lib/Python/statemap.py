#
# The contents of this file are subject to the Mozilla Public
# License Version 1.1 (the "License"); you may not use this file
# except in compliance with the License. You may obtain a copy of
# the License at http://www.mozilla.org/MPL/
#
# Software distributed under the License is distributed on an "AS
# IS" basis, WITHOUT WARRANTY OF ANY KIND, either express or
# implied. See the License for the specific language governing
# rights and limitations under the License.
#
# The Original Code is State Machine Compiler (SMC).
#
# The Initial Developer of the Original Code is Charles W. Rapp.
# Portions created by Charles W. Rapp are
# Copyright (C) 2005. Charles W. Rapp.
# All Rights Reserved.
#
# Port to Python by Francois Perrad, francois.perrad@gadz.org
# Copyright 2004, Francois Perrad.
# All Rights Reserved.
#
# Contributor(s):
#
# RCS ID
# Id: statemap.py,v 1.8 2010/09/11 18:56:57 fperrad Exp
#
# See: http://smc.sourceforge.net/
#

import sys

class StateUndefinedException(Exception):
	"""A StateUndefinedException is thrown by
	an SMC-generated state machine whenever a transition is taken
	and there is no state currently set. This occurs when a
	transition is issued from within a transition action."""
	pass

class TransitionUndefinedException(Exception):
	"""A TransitionUndefinedException is thrown by
	an SMC-generated state machine whenever a transition is taken
	which:

	 - Is not explicitly defined in the current state.
	 - Is not explicitly defined in the current FSM's default state.
	 - There is no Default transition in the current state."""
	pass


class State(object):
	"""base State class"""

	def __init__(self, name, id):
		self._name = name
		self._id = id

	def getName(self):
		"""Returns the state's printable name."""
		return self._name

	def getId(self):
		"""Returns the state's unique identifier."""
		return self._id


class FSMContext(object):
	"""The user can derive FSM contexts from this class and interface
	to them with the methods of this class.

	The finite state machine needs to be initialized to the starting
	state of the FSM.  This must be done manually in the constructor
	of the derived class.
	"""

	def __init__(self, state):
		self._state = state
		self._previous_state = None
		self._state_stack = []
		self._transition = None
		self._debug_flag = False
		self._debug_stream = sys.stderr

	def getDebugFlag(self):
		"""Returns the debug flag's current setting."""
		return self._debug_flag

	def setDebugFlag(self, flag):
		"""Sets the debug flag.
		A true value means debugging is on and false means off."""
		self._debug_flag = flag

	def getDebugStream(self):
		"""Returns the stream to which debug output is written."""
		return self._debug_stream

	def setDebugStream(self, stream):
		"""Sets the debug output stream."""
		self._debug_stream = stream

	def getState(self):
		"""Returns the current state."""
		if self._state == None:
			raise StateUndefinedException
		return self._state

	def isInTransition(self):
		"""Is this state machine already inside a transition?
		True if state is undefined."""
		if self._state == None:
			return True
		else:
			return False

	def getTransition(self):
		"""Returns the current transition's name.
		Used only for debugging purposes."""
		return self._transition

	def clearState(self):
		"""Clears the current state."""
		self._previous_state = self._state
		self._state = None


	def getPreviousState(self):
		"""Returns the state which a transition left.
		May be None"""
		return self._previous_state


	def setState(self, state):
		"""Sets the current state to the specified state."""
		if not isinstance(state, State):
			raise ValueError("state should be a statemap.State")
		self._state = state
		if self._debug_flag:
			self._debug_stream.write("ENTER STATE     : %s\n" % self._state.getName())

	def isStateStackEmpty(self):
		"""Returns True if the state stack is empty and False otherwise."""
		return len(self._state_stack) == 0

	def getStateStackDepth(self):
		"""Returns the state stack's depth."""
		return len(self._state_stack)

	def pushState(self, state):
		"""Push the current state on top of the state stack
		and make the specified state the current state."""
		if not isinstance(state, State):
			raise ValueError("state should be a statemap.State")
		if self._state != None:
			self._state_stack.append(self._state)
		self._state = state
		if self._debug_flag:
			self._debug_stream.write("PUSH TO STATE   : %s\n" % self._state.getName())

	def popState(self):
		"""Make the state on top of the state stack the current state."""
		if len(self._state_stack) == 0:
			if self._debug_flag:
				self._debug_stream.write("POPPING ON EMPTY STATE STACK.\n")
			raise ValueError("empty state stack")
		else:
			self._state = self._state_stack.pop()
			if self._debug_flag:
				self._debug_stream.write("POP TO STATE    : %s\n" % self._state.getName())

	def emptyStateStack(self):
		"""Remove all states from the state stack."""
		self._state_stack = []
