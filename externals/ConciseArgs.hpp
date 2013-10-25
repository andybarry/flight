/*
 Copyright (c) <2012> <Abraham Bachrach>

 Permission is hereby granted, free of charge, to any person obtaining a copy of
 this software and associated documentation files (the "Software"), to deal in
 the Software without restriction, including without limitation the rights to
 use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 of the Software, and to permit persons to whom the Software is furnished to do
 so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */

#ifndef __CONCISE_ARGS__
#define __CONCISE_ARGS__

#include <stdint.h>
#include <stdlib.h>

#include <string>
#include <sstream>
#include <iostream>
#include <iomanip>

#include <list>

class ConciseArgs {
public:
  // setup the argument parser
  // extra_args and description are only used for printing the usage.
  // Extra args should have the "non flagged" arguments that are expected
  // Description is a short blurb about the program
  // addHelpOption automatically add a -h/--help option that prints the usage
  ConciseArgs(int _argc, char ** _argv, const std::string & _extra_args = "", const std::string & _description = "",
      bool addHelpOption = true);
  ~ConciseArgs();

  // Add an argument handler
  // var_ref stores a reference to the variable that will get set if this option is present
  // shortname is the character looked for with
  template<class T>
  void add(T & var_ref, const std::string & shortName, const std::string & longName = "",
      const std::string & description = "", bool mandatory = false);

  void addUsageSeperator(const std::string & sep_msg = "");

  // do the parsing of flags, expecting handle 0-3 mandatory arguments (IN ORDER) that don't have options flags
  // calls parseVarArg internally with the apropriate number of remaining arguments
  void parse();
  template<class T1>
  void parse(T1 & reqArg1);
  template<class T1, class T2>
  void parse(T1 & reqArg1, T2 & reqArg2);
  template<class T1, class T2, class T3>
  void parse(T1 & reqArg1, T2 & reqArg2, T3 & reqArg3);

  // do the parsing of flags, and return any unparsed arguments
  // numRequired specifies the expected number of arguments that don't need a flag
  // if numRequired >=0, usage will be printed if the number of remaining arguments returned doesn't match
  std::list<std::string> parseVarArg(int numRequired = -1);

  // Check whether an option was parsed so that you could print something/do something special if it is/isn't
  // Must be a valid option.
  // Takes in either the long or short option name
  bool wasParsed(const std::string & name);

  //print the usage
  void usage(bool exitAfterPrinting = false);

private:
  std::string extra_args;
  std::string description;
  std::string programName;
  std::list<std::string> argv;
  class OptBase;
  std::list<OptBase *> opts;
  bool showHelp;
  bool parsed;
};

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
////// WARNING: DIRTY IMPLIMENTATION DETAILS BELOW... SHEILD YOUR EYES! //////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

namespace conciseargs_helpers {

// templated to_string function that works for any class that overloads the << operator
template<class T>
inline const std::string to_string(const T& t)
{
  std::stringstream ss;
  ss << std::boolalpha << std::showpoint << t;
  return ss.str();
}
//overload std::string and char to put double quotes around them
template<>
inline const std::string to_string<std::string>(const std::string & t)
{
  std::stringstream ss;
  ss << "\"" << t << "\"";
  return ss.str();
}
//overload char and char to put single quotes around them
template<>
inline const std::string to_string<char>(const char & t)
{
  std::stringstream ss;
  ss << "'" << t << "'";
  return ss.str();
}

//Templated helper functions for getting the string representation
template<typename T>
inline const std::string typenameToStr()
{
  return std::string("unkown");
}
// macro to implement specializations for given types
#define OPT_PARSE_MAKE_TYPENAME_TO_STRING( type ) \
    template<> inline const std::string typenameToStr<type>() {return std::string(#type);}
OPT_PARSE_MAKE_TYPENAME_TO_STRING(double);
OPT_PARSE_MAKE_TYPENAME_TO_STRING(float);
OPT_PARSE_MAKE_TYPENAME_TO_STRING(int64_t);
OPT_PARSE_MAKE_TYPENAME_TO_STRING(int32_t);
OPT_PARSE_MAKE_TYPENAME_TO_STRING(int16_t);
OPT_PARSE_MAKE_TYPENAME_TO_STRING(int8_t);
OPT_PARSE_MAKE_TYPENAME_TO_STRING(uint64_t);
OPT_PARSE_MAKE_TYPENAME_TO_STRING(uint32_t);
OPT_PARSE_MAKE_TYPENAME_TO_STRING(uint16_t);
OPT_PARSE_MAKE_TYPENAME_TO_STRING(uint8_t);
OPT_PARSE_MAKE_TYPENAME_TO_STRING(char);
OPT_PARSE_MAKE_TYPENAME_TO_STRING(bool);
OPT_PARSE_MAKE_TYPENAME_TO_STRING(std::string);

}

// Declaration of the OptBase Interface for internal options.
// This interface allows all the templating to be done internally :-)
class ConciseArgs::OptBase {
public:
  OptBase(const std::string & _shortName, const std::string & _longName, const std::string & _description,
      bool _mandatory) :
      shortName(_shortName), longName(_longName), description(_description), mandatory(_mandatory), parsed(false)
  { //TODO: assert that shortname is a char?
  }
  std::string shortName;
  std::string longName;
  std::string description;
  bool mandatory;
  bool parsed;
  virtual bool parse(const std::string & next, bool &swallowed)=0;
  virtual void print(int longNameWidth, int longNameMsgMinWidth)=0;
  virtual std::string makeLongNameStr(int long_name_width, int min_width = 0)=0;
};

namespace conciseargs_helpers {

// The templated child class that does the actual argument parsing
template<typename T>
class OptType: public ConciseArgs::OptBase {
public:
  OptType(const std::string & _shortName, const std::string & _longName, const std::string & _description, T & _var_ref,
      bool _mandatory) :
      OptBase(_shortName, _longName, _description, _mandatory), var_ref(_var_ref), default_val(_var_ref)
  {
  }
  bool parse(const std::string & next, bool & swallowed)
  {
    using namespace conciseargs_helpers;
    swallowed = false;
    T tmp_var;
    std::istringstream ss(next);
    ss >> tmp_var;
    if (next.size() > 0 && !ss.fail() && (ss.peek() == std::istringstream::traits_type::eof())) {
      parsed = true;
      var_ref = tmp_var;
      swallowed = true;
      return true;
    }
    else {
      std::cerr << "ERROR: Could not parse '" << next << "' as <" << typenameToStr<T>() << "> value for option '"
          << longName << "'\n";
      return false;
    }
  }
  std::string makeLongNameStr(int longNameWidth, int min_width = 0)
  {
    using namespace std;
    using namespace conciseargs_helpers;
    stringstream var_str;
    if (longName.size() > 0) {
      var_str << ", --" << left << setw(longNameWidth) << longName;
    }
    else {
      var_str << left << setw(longNameWidth + 4) << " ";
    }

    if (mandatory)
      var_str << " = <" + typenameToStr<T>() + ">";
    else
      var_str << " = [" + to_string(default_val) + "]";

    stringstream msg_str;
    msg_str << left << setw(min_width) << var_str.str() << " ";
    string msg = msg_str.str();
    return msg;
  }
  void print(int longNameWidth, int longNameMsgMinWidth)
  {
    using namespace std;
    using namespace conciseargs_helpers;
    cerr << "  -" << shortName;
    cerr << makeLongNameStr(longNameWidth, longNameMsgMinWidth);
    cerr << " : " << description << "\n";
  }

  T & var_ref;
  T default_val;
}
;

//specialization for a boolean flag
template<>
bool OptType<bool>::parse(const std::string & next, bool & swallowed)
{
  parsed = true;
  swallowed = false;
  var_ref = !var_ref;
  return true;
}

}

ConciseArgs::ConciseArgs(int _argc, char ** _argv, const std::string & _extra_args,
    const std::string & _description,
    bool addHelpOption) :
    extra_args(_extra_args), description(_description), showHelp(false), parsed(false)
{
  programName = _argv[0];
  for (int i = 1; i < _argc; i++)
    argv.push_back(std::string(_argv[i]));
  if (addHelpOption)
    add(showHelp, "h", "help", "Display this help message");
}

ConciseArgs::~ConciseArgs()
{
  for (std::list<OptBase *>::iterator oit = opts.begin(); oit != opts.end(); oit++)
    delete *oit;
}

template<class T>
void ConciseArgs::add(T & var_ref, const std::string & shortName, const std::string & longName,
    const std::string & description, bool mandatory)
{
  using namespace std;
  for (list<OptBase *>::iterator oit = opts.begin(); oit != opts.end(); oit++) {
    OptBase * opt = *oit;
    if (shortName.size() > 0 && opt->shortName == shortName) {
      cerr << "ERROR: adding option (" << shortName << ", " << longName
          << "): conflicts with previous shortname in option:\n";
      opt->print(0, opt->longName.size());
      exit(1);
    }
    if (longName.size() > 0 && opt->longName == longName) {
      cerr << "ERROR: adding option (" << shortName << ", " << longName
          << "): conflicts with previous longName in option:\n";
      opt->print(0, opt->longName.size());
      exit(1);
    }
  }

  opts.push_back(new conciseargs_helpers::OptType<T>(shortName, longName, description, var_ref, mandatory));
}

void ConciseArgs::addUsageSeperator(const std::string & sep_msg)
{
  bool unused;
  add(unused, "", "", sep_msg);
}

std::list<std::string> ConciseArgs::parseVarArg(int numRequired)
{
  using namespace std;
  if (parsed) {
    cerr << "ERROR: ConciseArgs parsing was already done once!\n";
    exit(1);
  }
  parsed = true;
  size_t found;
  bool swallowed = false;
  for (list<OptBase *>::iterator oit = opts.begin(); oit != opts.end(); oit++) {
    OptBase * opt = *oit;
    if (opt->shortName.size() == 0 && opt->longName.size() == 0)
      continue;
    for (list<string>::iterator ait = argv.begin(); ait != argv.end(); ait++) {
      const string & str = *ait;

      //search for long opt
      found = str.find("--" + opt->longName);
      if (found == 0) {
        size_t
        eq_found = str.find("=");
        if (eq_found != string::npos && eq_found + 1 < str.size()) {
          if (!opt->parse(str.substr(eq_found + 1), swallowed))
            usage(true);
        }
        else {
          list<string>::iterator next_ait = ait;
          next_ait++;
          if (next_ait != argv.end()) {
            if (!opt->parse(*next_ait, swallowed))
              usage(true);
          }
          else {
            if (!opt->parse("", swallowed))
              usage(true);
          }
          if (swallowed)
            argv.erase(next_ait); //option was processed, so remove it
        }
        ait = argv.erase(ait); //option was processed, so remove it
        break; //option was found
      }

      //search for short opt
      found = str.find("-" + opt->shortName);
      if (found == 0) { //found at start
        unsigned int shortNameLength = opt->shortName.size() + 1;
        if (str.size() > shortNameLength) {
          if (!opt->parse(str.substr(shortNameLength), swallowed))
            usage(true);
        }
        else {
          list<string>::iterator next_ait = ait;
          next_ait++;
          if (next_ait != argv.end()) {
            if (!opt->parse(*next_ait, swallowed))
              usage(true);
          }
          else {
            if (!opt->parse("", swallowed))
              usage(true);
          }
          if (swallowed)
            argv.erase(next_ait); //option was processed, so remove it

        }
        ait = argv.erase(ait); //option was processed, so remove it

        break; //option was found, so stop searching
      }
    }
    if (opt->mandatory && !opt->parsed) {
      cerr << "ERROR: option '" << opt->longName << "' is mandatory!\n";
      usage(true);
    }
    if (showHelp)
      usage(true);
  }
  if (numRequired >= 0 && (int) argv.size() != numRequired) {
    cerr << "ERROR: there are " << argv.size() << " arguments without flags, but " << numRequired
        << "  required arguments\n";
    usage(true);
  }
  return argv;
}

void ConciseArgs::parse()
{
  parseVarArg(0);
}
template<class T1>
void ConciseArgs::parse(T1 & var_ref1)
{
  bool swallowed;
  std::list<std::string> req = parseVarArg(1);
  std::list<std::string>::iterator it = req.begin();
  if (!conciseargs_helpers::OptType<T1>("", "Required Argument 1", "", var_ref1, true).parse(*it++, swallowed))
    usage(true);
}
template<class T1, class T2>
void ConciseArgs::parse(T1 & var_ref1, T2 & var_ref2)
{
  bool swallowed;
  std::list<std::string> req = parseVarArg(2);
  std::list<std::string>::iterator it = req.begin();
  if (!conciseargs_helpers::OptType<T1>("", "Required Argument 1", "", var_ref1, true).parse(*it++, swallowed))
    usage(true);
  if (!conciseargs_helpers::OptType<T2>("", "Required Argument 2", "", var_ref2, true).parse(*it++, swallowed))
    usage(true);

}
template<class T1, class T2, class T3>
void ConciseArgs::parse(T1 & var_ref1, T2 & var_ref2, T3 & var_ref3)
{
  bool swallowed;
  std::list<std::string> req = parseVarArg(3);
  std::list<std::string>::iterator it = req.begin();
  if (!conciseargs_helpers::OptType<T1>("", "Required Argument 1", "", var_ref1, true).parse(*it++, swallowed))
    usage(true);
  if (!conciseargs_helpers::OptType<T2>("", "Required Argument 2", "", var_ref2, true).parse(*it++, swallowed))
    usage(true);
  if (!conciseargs_helpers::OptType<T3>("", "Required Argument 3", "", var_ref3, true).parse(*it++, swallowed))
    usage(true);
}

void ConciseArgs::usage(bool exitAfterPrinting)
{
  using namespace std;
  size_t found;
  found = programName.find_last_of("/");
  if (found != string::npos)
    programName = programName.substr(found + 1);

  int maxLongNameLen = 0;
  for (list<OptBase *>::iterator oit = opts.begin(); oit != opts.end(); oit++) {
    OptBase * opt = *oit;
    int longNameLen = opt->longName.size();
    if (longNameLen > maxLongNameLen)
      maxLongNameLen = longNameLen;
  }

  int maxLongNameStrLen = 0;
  for (list<OptBase *>::iterator oit = opts.begin(); oit != opts.end(); oit++) {
    OptBase * opt = *oit;
    int longNameStrLen = opt->makeLongNameStr(maxLongNameLen, 0).size();
    if (longNameStrLen > maxLongNameStrLen)
      maxLongNameStrLen = longNameStrLen;
  }

  cerr << "Usage:\n";
  cerr << "  " << programName << " [opts] " << extra_args << "\n";
  if (description.size() > 0)
    cerr << " " << description << "\n";
  cerr << "Options:\n";
  for (list<OptBase *>::iterator oit = opts.begin(); oit != opts.end(); oit++) {
    OptBase * opt = *oit;
    if (opt->shortName.size() == 0 && opt->longName.size() == 0)
      cerr << opt->description << "\n";
    else
      opt->print(maxLongNameLen, maxLongNameStrLen);
  }
  cerr << "\n";
  if (exitAfterPrinting) {
    exit(1);
  }

}

bool ConciseArgs::wasParsed(const std::string & name)
{
  using namespace std;
  if (!parsed) {
    cerr << "ERROR checking parse status of " << name << " : ConciseArgs object hasn't been parsed!\n";
    exit(1);
  }
  for (list<OptBase *>::iterator oit = opts.begin(); oit != opts.end(); oit++) {
    OptBase * opt = *oit;
    if (opt->shortName == name || opt->longName == name)
      return opt->parsed;
  }
  cerr << "ERROR checking whether '" << name << "' was parsed. Not a valid option\n";
  usage(true);
  return false;
}

#endif
