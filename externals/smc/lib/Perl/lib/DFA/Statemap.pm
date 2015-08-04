#
# RCS ID
# Id: Statemap.pm,v 1.5 2009/11/24 20:42:39 cwrapp Exp
#

use strict;
use warnings;

package DFA::Statemap;

use vars qw($VERSION);
$VERSION = '1.01';

=head1 NAME

DFA::Statemap

=head1 DESCRIPTION

This is the SMC (State Machine Compiler) runtime for target language Perl.

See L<http://smc.sourceforge.net/>.

This namespace contains two class :

=over 8

=item State

the base State class

=item FSMContext

the Finite State Machine Context class

=back

=cut

package DFA::Statemap::State;

=head1 DFA::Statemap::State

=head2 new

Constructor.

=cut

sub new {
    my $proto = shift;
    my $class = ref($proto) || $proto;
    my ($name, $id) = @_;
    my $self = {};
    bless($self, $class);
    $self->{_name} = $name;
    $self->{_id} = $id;
    return $self
}

=head2 getName

Returns the state's printable name.

=cut

sub getName {
    my $self = shift;
    return $self->{_name};
}

=head2 getId

Returns the state's unique identifier.

=cut

sub getId {
    my $self = shift;
    return $self->{_id};
}

package DFA::Statemap::FSMContext;

use Carp;

=head1 DFA::Statemap::FSMContext

The user can derive FSM contexts from this class and interface
to them with the methods of this class.

The finite state machine needs to be initialized to the starting
state of the FSM.  This must be done manually in the constructor
of the derived class.

=head2 new ( $init_state )

Default constructor.

=cut

sub new {
    my $proto = shift;
    my $class = ref($proto) || $proto;
    my $self = {};
    bless($self, $class);
    my ($init_state) = @_;
    $self->{_state} = $init_state;
    $self->{_previous_state} = undef;
    $self->{_state_stack} = [];
    $self->{_transition} = undef;
    $self->{_debug_flag} = undef;
    $self->{_debug_stream} = \*STDERR;
    return $self
}

=head2 getDebugFlag

Returns the debug flag's current setting.

=cut

sub getDebugFlag {
    my $self = shift;
    return $self->{_debug_flag};
}

=head2 setDebugFlag

Sets the debug flag.
A true value means debugging is on and false means off.

=cut

sub setDebugFlag {
    my $self = shift;
    my ($flag) = @_;
    $self->{_debug_flag} = $flag;
}

=head2 getDebugStream

Returns the stream to which debug output is written.

=cut

sub getDebugStream {
    my $self = shift;
    return $self->{_debug_stream};
}

=head2 setDebugStream

Sets the debug output stream.

=cut

sub setDebugStream {
    my $self = shift;
    my ($stream) = @_;
    $self->{_debug_stream} = $stream;
}

=head2 getState

Returns the current state.

=cut

sub getState {
    my $self = shift;
    confess "StateUndefinedException\n"
            unless (defined $self->{_state});
    return $self->{_state};
}

=head2 isInTransition

Is this state machine already inside a transition?

True if state is undefined.

=cut

sub isInTransition {
    my $self = shift;
    return !defined($self->{_state});
}

=head2 getTransition

Returns the current transition's name.

Used only for debugging purposes.

=cut

sub getTransition {
    my $self = shift;
    return $self->{_transition};
}

=head2 clearState

Clears the current state.

=cut

sub clearState {
    my $self = shift;
    $self->{_previous_state} = $self->{_state};
    $self->{_state} = undef;
}

=head2 getPreviousState

Returns the state which a transition left.

May be B<undef>.

=cut

sub getPreviousState {
    my $self = shift;
    return $self->{_previous_state};
}

=head2 setState

Sets the current state to the specified state.

=cut

sub setState {
    my $self = shift;
    my ($state) = @_;
    if ($self->{_debug_flag}) {
        confess "undefined state.\n"
                unless (defined $state);
        confess "$state is not a Statemap::State.\n"
                unless (ref $state and $state->isa('DFA::Statemap::State'));
    }
    else {
        croak "undefined state.\n"
                unless (defined $state);
        croak "$state is not a Statemap::State.\n"
                unless (ref $state and $state->isa('DFA::Statemap::State'));
    }
    $self->{_state} = $state;
    if ($self->{_debug_flag}) {
        my $fh = $self->{_debug_stream};
        print $fh "ENTER STATE     : ", $self->{_state}->getName(), "\n";
    }
}

=head2 isStateStackEmpty

Returns true if the state stack is empty and false otherwise.

=cut

sub isStateStackEmpty {
    my $self = shift;
    return scalar(@{$self->{_state_stack}}) == 0;
}

=head2 getStateStackDepth

Returns the state stack's depth.

=cut

sub getStateStackDepth {
    my $self = shift;
    return scalar(@{$self->{_state_stack}});
}

=head2 pushState

Push the current state on top of the state stack
and make the specified state the current state.

=cut

sub pushState {
    my $self = shift;
    my ($state) = @_;
    if ($self->{_debug_flag}) {
        confess "undefined state\n"
                unless (defined $state);
        confess "$state is not a State\n"
                unless (ref $state and $state->isa('DFA::Statemap::State'));
    }
    else {
        croak "undefined state\n"
                unless (defined $state);
        croak "$state is not a State\n"
                unless (ref $state and $state->isa('DFA::Statemap::State'));
    }
    if (defined $self->{_state}) {
        push @{$self->{_state_stack}}, $self->{_state};
    }
    $self->{_state} = $state;
    if ($self->{_debug_flag}) {
        my $fh = $self->{_debug_stream};
        print $fh "PUSH TO STATE   : ", $self->{_state}->getName(), "\n";
    }
}

=head2 popState

Make the state on top of the state stack the current state.

=cut

sub popState {
    my $self = shift;
    if (scalar(@{$self->{_state_stack}}) == 0) {
        if ($self->{_debug_flag}) {
            my $fh = $self->{_debug_stream};
            print $fh "POPPING ON EMPTY STATE STACK.\n";
            confess "empty state stack.\n"
        }
        else {
            croak "empty state stack.\n"
        }
    }
    else {
        $self->{_state} = pop @{$self->{_state_stack}};
        if ($self->{_debug_flag}) {
            my $fh = $self->{_debug_stream};
            print $fh "POP TO STATE    : ", $self->{_state}->getName(), "\n";
        }
    }
}

=head2 emptyStateStack

Remove all states from the state stack.

=cut

sub emptyStateStack {
    my $self = shift;
    $self->{_state_stack} = [];
}

=head1 LICENSE

The contents of this file are subject to the Mozilla Public
License Version 1.1 (the "License"); you may not use this file
except in compliance with the License. You may obtain a copy of
the License at http://www.mozilla.org/MPL/

Software distributed under the License is distributed on an "AS
IS" basis, WITHOUT WARRANTY OF ANY KIND, either express or
implied. See the License for the specific language governing
rights and limitations under the License.

=head1 AUTHORS

The Original Code is State Machine Compiler (SMC).

The Initial Developer of the Original Code is Charles W. Rapp.

Port to Perl by Francois Perrad, francois.perrad@gadz.org

Copyright 2004-2009, Francois Perrad.
All Rights Reserved.

Contributor(s):

=head1 HISTORY

This module was previously named StateMachine::Statemap.

=head1 SEE ALSO

L<http://smc.sourceforge.net/>

=cut
