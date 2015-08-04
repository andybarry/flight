--
-- The contents of this file are subject to the Mozilla Public
-- License Version 1.1 (the "License"); you may not use this file
-- except in compliance with the License. You may obtain a copy of
-- the License at http://www.mozilla.org/MPL/
--
-- Software distributed under the License is distributed on an "AS
-- IS" basis, WITHOUT WARRANTY OF ANY KIND, either express or
-- implied. See the License for the specific language governing
-- rights and limitations under the License.
--
-- The Original Code is State Machine Compiler (SMC).
--
-- The Initial Developer of the Original Code is Charles W. Rapp.
--
-- Port to Lua by Francois Perrad, francois.perrad@gadz.org
-- Copyright 2007, Francois Perrad.
-- All Rights Reserved.
--
-- Contributor(s):
--
-- RCS ID
-- Id: statemap.lua,v 1.6 2010/11/21 18:46:31 fperrad Exp
--
--
-- This module contains two class  :
-- * State
--    the base State class
-- * FSMContext
--    the Finite State Machine Context class
--
-- See: http://smc.sourceforge.net/
--

local assert = assert
local error = error
local pairs = pairs
local setmetatable = setmetatable
local type = type
local stderr = require 'io'.stderr

_ENV = nil

-- base State class
local State = {}

function State.class ()
    return setmetatable({}, {__index = State})
end

function State:new (name, id)
    local o = {
        name = name,
        id = id,
    }
    setmetatable(o, {__index = self})
    return o
end

--[[
 The user can derive FSM contexts from this class and interface
 to them with the methods of this class.

 The finite state machine needs to be initialized to the starting
 state of the FSM.  This must be done manually in the constructor
 of the derived class.
]]
local FSMContext = {}

function FSMContext.class ()
    return setmetatable({}, {__index = FSMContext})
end

function FSMContext:new (args)
    local o = {}
    for k, v in pairs(args) do
        o[k] = v
    end
    o._state_stack = {}
    o.debugStream = stderr
    setmetatable(o, {__index = self})
    o:_init()
    return o
end

function FSMContext:_init ()
    error "FSMContext can't be instantiated"
end

function FSMContext:getDebugFlag ()
    -- Returns the debug flag's current setting.
    return self.debugFlag
end

function FSMContext:setDebugFlag (flag)
    -- Sets the debug flag.
    --
    -- A true value means debugging is on and false means off.
    self.debugFlag = flag
end

function FSMContext:getDebugStream ()
    -- Returns the stream to which debug output is written.
    return self.debugStream
end

function FSMContext:setDebugStream (stream)
    -- Sets the debug output stream.
    self.debugStream = stream
end

function FSMContext:isInTransition ()
    -- Is this state machine already inside a transition?
    --
    -- True if state is undefined.
    return self._state == nil
end

function FSMContext:clearState ()
    -- Clears the current state.
    self.previousState = self._state
    self._state = nil
end

function FSMContext:getPreviousState ()
    -- Returns the state which a transition left.
    --
    -- May be nil.
    return self.previousState
end

function FSMContext:getState ()
    -- Gets the current state.
    if self._state == nil then
        error "State Undefined"
    end
    return self._state
end

function FSMContext:setState (state)
    -- Sets the current state to the specified state.
    assert(state ~= nil, "undefined state.")
    assert(type(state) == 'table') -- "state should be a State"
    self._state = state
    if self.debugFlag then
        self.debugStream:write("ENTER STATE     : ", self._state.name, "\n")
    end
end

function FSMContext:isStateStackEmpty ()
    -- Returns true if the state stack is empty and false otherwise.
    return #self._state_stack == 0
end

function FSMContext:getStackDepth ()
    -- Returns the state stack's depth.
    return #self._state_stack
end

function FSMContext:pushState (state)
    -- Push the current state on top of the state stack
    -- and make the specified state the current state.
    assert(state ~= nil, "undefined state.")
    assert(type(state) == 'table') -- "state should be a State"
    if self._state then
        local t = self._state_stack; t[#t+1] = self._state -- push
    end
    self._state = state
    if self.debugFlag then
        self.debugStream:write("PUSH TO STATE   : ", self._state.name, "\n")
    end
end

function FSMContext:popState ()
    -- Make the state on top of the state stack the current state.
    local t = self._state_stack
    if #t == 0 then
        if self.debugFlag then
            self.debugStream:write("POPPING ON EMPTY STATE STACK.\n")
        end
        error("empty state stack.")
    else
        self._state = t[#t]; t[#t] = nil -- pop
        if self.debugFlag then
            self.debugStream:write("POP TO STATE    : ", self._state.name, "\n")
        end
    end
end

function FSMContext:emptyStateStack ()
    -- Remove all states from the state stack.
    self._state_stack = {}
end

return {
    State       = State,
    FSMContext  = FSMContext,
}
