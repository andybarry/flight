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
#
# Port to Ruby by Francois Perrad, francois.perrad@gadz.org
# Copyright 2004, Francois Perrad.
# All Rights Reserved.
#
# Contributor(s):
#
# RCS ID
# Id: statemap.rb,v 1.5 2010/09/11 18:55:55 fperrad Exp
#

# This namespace contains two class (and two exceptions) :
#* State
#    the base State class
#* FSMContext
#    the Finite State Machine Context class
#
#See: http://smc.sourceforge.net/
module Statemap

	# A StateUndefinedException is thrown by
	# an SMC-generated state machine whenever a transition is taken
	# and there is no state currently set. This occurs when a
	# transition is issued from with a transition action.
	class StateUndefinedException < Exception
	end

	# A TransitionUndefinedException is thrown by
	# an SMC-generated state machine whenever a transition is taken
	# which:
	#
	# * Is not explicitly defined in the current state.
	# * Is not explicitly defined in the current FSM's default state.
	# * There is no Default transition in the current state.
	class TransitionUndefinedException < Exception
	end

	# base State class
	class State

		def initialize(name, id)
			@_name = name
			@_id = id
		end

		# Returns the state's printable name.
		def getName()
			return @_name
		end

		# Returns the state's unique identifier.
		def getId()
			return @_id
		end

	end

	# The user can derive FSM contexts from this class and interface
	# to them with the methods of this class.
	#
	# The finite state machine needs to be initialized to the starting
	# state of the FSM.  This must be done manually in the constructor
	# of the derived class.
	class FSMContext

		def initialize(initState)
			@_state = initState
			@_previous_state = nil
			@_state_stack = []
			@_transition = nil
			@_debug_flag = nil
			@_debug_stream = $stderr
		end

		# Returns the debug flag's current setting.
		def getDebugFlag()
			return @_debug_flag
		end

		# Sets the debug flag.
		#
		# A true value means debugging is on and false means off.
		def setDebugFlag(flag)
			@_debug_flag = flag
		end

		# Returns the stream to which debug output is written.
		def getDebugStream()
			return @_debug_stream
		end

		# Sets the debug output stream.
		def setDebugStream(stream)
			@_debug_stream = stream
		end

		# Returns the current state.
		def getState()
			if @_state.nil? then
				raise Statemap::StateUndefinedException
			end
			return @_state
		end


		# Is this state machine already inside a transition?
		#
		# True if state is undefined.
		def isInTransition()
			return @_state == nil
		end

		# Returns the current transition's name.
		#
		# Used only for debugging purposes.
		def getTransition()
			return @_transition
		end

		# Clears the current state.
		def clearState()
			@_previous_state = @_state
			@_state = nil
		end

		# Returns the state which a transition left.
		#
		# May be nil.
		def getPreviousState()
			return @_previous_state
		end

		# Sets the current state to the specified state.
		def setState(state)
			raise "undefined state.\n" if state.nil?
			raise "#{state} is not a Statemap.State.\n" unless state.is_a?(Statemap::State)
			@_state = state
			if @_debug_flag then
				@_debug_stream.puts "ENTER STATE     : %s\n" % @_state.getName
			end
		end

		# Returns true if the state stack is empty and false otherwise.
		def isStateStackEmpty()
			return @_state_stack.empty?
		end

		# Returns the state stack's depth.
		def getStateStackDepth()
			return @_state_stack.size
		end

		# Push the current state on top of the state stack
		# and make the specified state the current state.
		def pushState(state)
			raise "undefined state.\n" if state.nil?
			raise "#{state} is not a Statemap.State.\n" unless state.is_a?(Statemap::State)
			unless @_state.nil?
				@_state_stack.push @_state
			end
			@_state = state
			if @_debug_flag then
				@_debug_stream.puts "PUSH TO STATE   : %s\n" % @_state.getName
			end
		end

		# Make the state on top of the state stack the current state.
		def popState()
			if @_state_stack.empty? then
				if @_debug_flag then
					@_debug_stream.puts "POPPING ON EMPTY STATE STACK.\n"
				end
				raise "empty state stack.\n"
			else
				@_state = @_state_stack.pop
				if @_debug_flag then
					@_debug_stream.puts "POP TO STATE    : %s\n" % @_state.getName
				end
			end
		end

		# Remove all states from the state stack.
		def emptyStateStack()
			@_state_stack = []
		end

	end

end
