//
// The contents of this file are subject to the Mozilla Public
// License Version 1.1 (the "License"); you may not use this file
// except in compliance with the License. You may obtain a copy
// of the License at http://www.mozilla.org/MPL/
// 
// Software distributed under the License is distributed on an
// "AS IS" basis, WITHOUT WARRANTY OF ANY KIND, either express or
// implied. See the License for the specific language governing
// rights and limitations under the License.
// 
// The Original Code is State Machine Compiler (SMC).
// 
// The Initial Developer of the Original Code is Charles W. Rapp.
// Portions created by Charles W. Rapp are
// Copyright (C) 2007. Charles W. Rapp.
// All Rights Reserved.
//
// Contributor(s):
//    Chris Liscio
//
// Namespace
//	statemap
//
// Description
//  This namespace contains the finite state machine context
//  class. The user can derive FSM contexts from this class and
//  interface to them with the methods of this class.
//
// Notes
//  The finite state machine needs to be initialized to the
//  starting state of the FSM.  This must be done manually in
//  the constructor of the derived class. 
//
// Author
//	Chris Liscio
//
// RCS ID
// Id: statemap.h,v 1.3 2013/07/14 14:32:36 cwrapp Exp
//
// CHANGE LOG
// (See bottom of this file)
//

#import <Foundation/Foundation.h>

#define TRACE(msg...) NSLog(msg)

// Limit names to 100 ASCII characters.
// Why 100? Because it is a round number.
#define MAX_NAME_LEN 100

@interface SMCState : NSObject
{
    NSString *_name;
    int _stateId;
}
- (NSString*)name;
- (void)setName:(NSString*)aValue;

- (int)stateId;
- (void)setStateId:(int)newStateId;

- (id)initWithName:(NSString*)name stateId:(int)stateId;
@end

@interface SMCStateEntry : NSObject
{
    SMCState *_state;
    SMCStateEntry *_next;
}
+ (id)stateEntryWithState:(SMCState*)state next:(SMCStateEntry*)next;

- (SMCState*)state;
- (void)setState:(SMCState*)aValue;
- (SMCStateEntry*)next;
- (void)setNext:(SMCStateEntry*)aValue;
@end

@interface SMCFSMContext : NSObject
{
    // The current state of the finite state machine.
    SMCState *_state;

    // Remember which state a transition left.
    SMCState *_previousState;

    // The stack of pushed states.
    SMCStateEntry *_stateStack;

    NSString* _transition;
    BOOL _debugFlag;    
}
- (id)initWithState: (SMCState*)aState;
- (void)emptyStateStack;
- (BOOL)debugFlag;
- (void)setDebugFlag:(BOOL)newDebugFlag;
- (BOOL)isInTransition;
- (NSString*)transition;
- (void)setTransition:(NSString*)aValue;
- (void)clearState;
- (SMCState*)previousState;
- (void)setState:(SMCState*)state;
- (BOOL)isStateStackEmpty;
- (int)stateStackDepth;
- (void)pushState:(SMCState*)state;
- (void)popState;
@end
  
/*
 * Support for ARC and non-ARC compilation. 
 * Use S_RELEASE instead of release, if you want to support both ARC and non-ARC.
 *
 * See https://gist.github.com/2823399 for detail.
 */

#if __has_feature(objc_arc)
#define S_RETAIN self
#define S_AUTORELEASE self
#define S_RELEASE self
#define S_DEALLOC self
#else
#define S_RETAIN retain
#define S_AUTORELEASE autorelease
#define S_RELEASE release
#define S_DEALLOC dealloc
#endif

//
// CHANGE LOG
// Log: statemap.h,v
// Revision 1.3  2013/07/14 14:32:36  cwrapp
// check in for release 6.2.0
//
// Revision 1.2  2009/04/10 14:02:10  cwrapp
// Set initial state via initializer.
//
// Revision 1.1  2007/01/15 00:23:50  cwrapp
// Release 4.4.0 initial commit.
//
