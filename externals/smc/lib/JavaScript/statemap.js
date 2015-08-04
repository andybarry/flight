/**
 * The contents of this file are subject to the Mozilla Public
 * License Version 1.1 (the "License"); you may not use this file
 * except in compliance with the License. You may obtain a copy of
 * the License at http://www.mozilla.org/MPL/
 *
 * Software distributed under the License is distributed on an "AS
 * IS" basis, WITHOUT WARRANTY OF ANY KIND, either express or
 * implied. See the License for the specific language governing
 * rights and limitations under the License.
 *
 * The Original Code is State Machine Compiler (SMC).
 *
 * The Initial Developer of the Original Code is Charles W. Rapp.
 *
 * Port to JavaScript by Francois Perrad, francois.perrad@gadz.org
 * Copyright 2013, Francois Perrad.
 * All Rights Reserved.
 *
 * Contributor(s):
 *
 * RCS ID
 * Id: statemap.js,v 1.3 2013/12/15 16:30:34 fperrad Exp
 *
 *
 * This module contains two class  :
 * * State
 *    the base State class
 * * FSMContext
 *    the Finite State Machine Context class
 *
 * See: http://smc.sourceforge.net/
 *
 */


// base State class
function State (name, id) {
    this.name = name;
    this.id = id;
};
try {
    global.State = State;
} catch (ex) {}

/*
 The user can derive FSM contexts from this class and interface
 to them with the methods of this class.

 The finite state machine needs to be initialized to the starting
 state of the FSM.  This must be done manually in the constructor
 of the derived class.
*/

function FSMContext (startState) {
    this.state = startState;
    this.previousState = null;
    this.stateStack = [];
    this.transition = '';
    this.debugFlag = false;
    try {
        this.debugStream = process.stderr;
    } catch (ex) {}
};
try {
    global.FSMContext = FSMContext;
} catch (ex) {}

// Returns the debug flag's current setting.
FSMContext.prototype.getDebugFlag = function () {
    return this.debugFlag;
}

// Sets the debug flag.
// A true value means debugging is on and false means off.
FSMContext.prototype.setDebugFlag = function (flag) {
    this.debugFlag = flag;
}

// Returns the stream to which debug output is written.
FSMContext.prototype.getDebugStream = function () {
    return this.debugStream;
}

// Sets the debug output stream.
FSMContext.prototype.setDebugStream = function (stream) {
    this.debugStream = stream;
}

// Is this state machine already inside a transition?
// True if state is undefined.
FSMContext.prototype.isInTransition = function () {
    return this.state == null;
}

// Clears the current state.
FSMContext.prototype.clearState = function () {
    this.previousState = this.state;
    this.state = null;
}

// Returns the state which a transition left.
// May be Null
FSMContext.prototype.getPreviousState = function () {
    return this.previousState;
}

// Returns the current state.
FSMContext.prototype.getState = function () {
    return this.state;
}

// Sets the current state to the specified state.
FSMContext.prototype.setState = function (state) {
    if (!(state instanceof State)) {
        throw new TypeError(state + ' should be of class State');
    }
    this.state = state;
    if (this.debugFlag) {
        this.debugStream.write("ENTER STATE     : " + this.state.name + "\n");
    }
}

// Returns True if the state stack is empty and False otherwise.
FSMContext.prototype.isStateStackEmpty = function () {
    return this.stateStack.length == 0;
}

// Returns the state stack's depth.
FSMContext.prototype.getStateStackDepth = function () {
    return this.stateStack.length;
}

// Push the current state on top of the state stack
// and make the specified state the current state.
FSMContext.prototype.pushState = function (state) {
    if (!(state instanceof State)) {
        throw new TypeError(state + ' should be of class State');
    }
    if (this.state) {
            this.stateStack.push(this.state);
    }
    this.state = state;
    if (this.debugFlag) {
        this.debugStream.write("PUSH TO STATE   : " + this.state.name + "\n");
    }
}

// Make the state on top of the state stack the current state.
FSMContext.prototype.popState = function () {
    if (this.stateStack.length == 0) {
        if (this.debugFlag) {
            this.debugStream.write("POPPING ON EMPTY STATE STACK.\n");
        }
        throw new Error("empty state stack.");
    }
    else {
        this.state = this.stateStack.pop();
        if (this.debugFlag) {
            this.debugStream.write("POP TO STATE    : " + this.state.name + "\n");
        }
    }
}

// Remove all states from the state stack.
FSMContext.prototype.emptyStateStack = function () {
    this.stateStack = [];
}

