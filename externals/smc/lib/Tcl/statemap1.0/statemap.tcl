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
# Copyright (C) 2000 - 2003 Charles W. Rapp.
# All Rights Reserved.
#
# Contributor(s):
#
# RCS ID
# Id: statemap.tcl,v 1.11 2011/11/20 14:58:33 cwrapp Exp
#
# statemap.tcl --
#
#  This package defines the fsmContext class which must be
#  inherited by any [incr Tcl] class wanting to use an smc
#  generated state machine.
#
# Change Log
# Log: statemap.tcl,v
# Revision 1.11  2011/11/20 14:58:33  cwrapp
# Check in for SMC v. 6.1.0
#
# Revision 1.10  2010/09/11 19:02:32  fperrad
# use the same message in all language
#
# Revision 1.9  2009/11/24 20:42:39  cwrapp
# v. 6.0.1 update
#
# Revision 1.8  2009/03/01 18:20:41  cwrapp
# Preliminary v. 6.0.0 commit.
#
# Revision 1.7  2005/05/28 18:47:13  cwrapp
# Updated C++, Java and Tcl libraries, added CSharp, Python and VB.
#
# Revision 1.0  2004/05/31 13:45:52  charlesr
# Initial revision
#

package provide statemap 0.1;

package require Itcl;

namespace eval ::statemap:: {
    namespace export FSMContext;
}

::itcl::class ::statemap::State {
# Member data.
    protected variable _name "NAME NOT SET";
    protected variable _id -1;

# Member functions.

    constructor {name id} {
        set _name $name;
        set _id $id;
    }

    public method getName {} {
        return -code ok $_name;
    }

    public method getId {} {
        return -code ok $_id;
    }
}

::itcl::class ::statemap::FSMContext {
# Member data.

    protected variable _state "";
    private variable _previous_state "";
    private variable _state_stack {};
    private variable _transition;
    private variable _debug_flag;
    private variable _debug_stream;

# Member functions.

    constructor {initState} {
        set _state $initState;
        set _previous_state "";
        set _state_stack {};
        set _transition "";
        set _debug_flag 0;
        set _debug_stream stderr;
    }

    public method enterStartState {} {
        ${_state} Entry $this;
        return -code ok;
    }

    public method getDebugFlag {} {
        return -code ok $_debug_flag;
    }

    public method setDebugFlag {flag} {
        if {$flag != 0} {
            set _debug_flag 1;
        } else {
            set _debug_flag 0;
        }

        return -code ok;
    }

    public method getDebugStream {} {
        return -code ok $_debug_stream;
    }

    public method setDebugStream {stream} {
        set _debug_stream $stream;
        return -code ok;
    }

    public method isInTransition {} {
        if {[string compare $_state ""] == 0} {
            set retval 1;
        } else {
            set retval 0;
        }

        return -code ok $retval;
    }

    public method getTransition {} {
        return -code ok $_transition;
    }

    # Save away the transition name only if debugging is
    # turned on.
    protected method setTransition {transition} {
        if {$_debug_flag == 1} {
            set _transition $transition;
        }
    }

    public method getState {} {
        if {[string compare $_state ""] == 0} {
            return -code error "The start state has not been set.";
        } else {
            return -code ok $_state;
        }
    }

    public method setState {state_name} {
        if {$_debug_flag == 1} {
            puts $_debug_stream "ENTER STATE     : [$state_name getName]";
        }

        # Set the previous state in case clearState was not
        # called.
        if {[string compare $_state ""] != 0} {
            set _previous_state $_state;
        }

        set _state $state_name;

        return -code ok;
    }

    public method clearState {} {
        set _previous_state $_state;
        set _state "";

        return -code ok;
    }

    public method getPreviousState {} {
        if {[string compare $_previous_state ""] == 0} {
            return -code error "The previous state has not been set.";
        } else {
            return -code ok $_previous_state;
        }
    }

    public method pushState {state_name} {
        if {$_debug_flag == 1} {
            puts $_debug_stream "PUSH TO STATE   : [$state_name getName]";
        }

        if {[string compare $_state ""] != 0} {
            lappend _state_stack $_state;
        }

        set _pervious_state $_state;
        set _state $state_name;

        return -code ok;
    }

    public method popState {} {
        if {[llength $_state_stack] > 0} {
            set _pervious_state $_state;
            set _state [lindex $_state_stack end];
            set _state_stack [lrange $_state_stack 0 [expr [llength $_state_stack] - 2]];

            set Retcode ok;
            set Retval "";

            if {$_debug_flag == 1} {
                puts $_debug_stream "POP TO STATE    : [$_state getName]";
            }
        } else {
            # Tried to pop on an empty state stack.
            if {$_debug_flag == 1} {
                puts $_debug_stream "POPPING ON EMPTY STATE STACK.";
            }

            set Retcode error;
            set Retval "Pop transition does not have a matching push transition (State: [$_state getName]).";
        }

        return -code $Retcode $Retval;
    }

    public method emptyStateStack {} {
        if {[llength $_state_stack] > 0} {
            set _state_stack {};
        }

        return -code ok;
    }
}
