#!/usr/bin/python
#
#Converts a LCM log to a "matrix" format that is easier to work with in external
#tools such as Matlab. The set of messages on a given channel can be represented
#as a matrix, where the columns of this matrix are the the fields of the lcm type
#with one message per row

import os
import sys
import binascii
import types
import numpy
import re
import getopt

# check which version for mio location
if sys.version_info < (2, 6):
    import scipy.io.mio
else:
    import scipy.io.matlab.mio

from lcm import EventLog


def usage():
    pname, sname = os.path.split(sys.argv[0])
    sys.stderr.write("usage: % s %s < filename > \n" % (sname, str(longOpts)))
    print """
    -h --help                 print this message
    -p --print                Output log data to stdout instead of to .mat
    -f --format               print the data format to stderr
    -s --seperator=sep        print data with separator [sep] instead of default to ["" ""]
    -c --channelsToProcess=chan        Parse channelsToProcess that match Python regex [chan] defaults to [".*"]
    -i --ignore=chan          Ignore channelsToProcess that match Python regex [chan]
                              ignores take precedence over includes!
    -o --outfile=ofname       output data to [ofname] instead of default [filename.mat or stdout]
    -l --lcmtype_pkgs=pkgs    load python modules from comma seperated list of packages [pkgs] defaults to ["botlcm"]
    -v                        Verbose
    -m --no-m-file            Don't output .m files.

    """
    sys.exit()

class LCMTypeDatabase:
    def __init__(self, package_names, verbose):
        for p in lcm_packages:
            try:
                __import__(p)
            except:
                if verbose:
                    sys.stderr.write("couldn't load module %s\n" % p)
                    lcm_packages.remove(p)

        self.klasses = {}
        for pkg in [ sys.modules[n] for n in package_names ]:
            for mname in dir(pkg):
                try:
                    module = getattr(pkg, mname)
                    if type(module) != types.TypeType:
                        continue
                    self.klasses[module._get_packed_fingerprint()] = module
                except:
                    pass

    def find_type(self, packed_fingerprint):
        return self.klasses.get(packed_fingerprint, None)


flatteners = {}
data = {}

def make_simple_accessor(fieldname):
    return lambda lst, x: lst.append(getattr(x, fieldname))

def make_numpy_array_accessor(fieldname):
    return lambda lst, x: lst.extend(numpy.array(getattr(x, fieldname)).ravel())

def make_obj_accessor(fieldname, func):
    return lambda lst, x: func(lst, getattr(x, fieldname))

def make_obj_list_accessor(fieldname, func):
    return lambda lst, x: map(lambda item: func(lst, item), getattr(x, fieldname))
#    def list_accessor(lst, msg):
#        msg_lst = getattr(msg, fieldname)
#        for elem in msg_lst:
#            func(lst, elem)
#    return list_accessor
#


def make_lcmtype_accessor(msg):
    funcs = []

    for fieldname in getattr(msg, '__slots__'):
        m = getattr(msg, fieldname)

        if type(m) in [ types.IntType, types.LongType, types.FloatType,
                types.BooleanType ]:
            # scalar
            accessor = make_simple_accessor(fieldname)
            funcs.append(accessor)
        elif type(m) in [ types.ListType, types.TupleType ]:
            # convert to a numpy array
            arr = numpy.array(m)

            # check the data type of the array
            if arr.dtype.kind in "bif":
                # numeric data type
                funcs.append(make_numpy_array_accessor(fieldname))
            elif arr.dtype.kind == "O":
                # compound data type
                typeAccess = make_lcmtype_accessor(m[0])
                funcs.append(make_obj_list_accessor(fieldname, typeAccess))
                #pass
        elif type(m) in types.StringTypes:
            # ignore strings
            pass
        else:
            funcs.append(make_obj_accessor(fieldname, make_lcmtype_accessor(m)))

    def flatten(lst, m):
        for func in funcs:
            func(lst, m)
    return flatten

def make_flattener(msg):
    accessor = make_lcmtype_accessor(msg)
    def flattener(m):
        result = []
        accessor(result, m)
        return result
    return flattener




def make_lcmtype_string(msg, base=True):
    typeStr = []
    count = 0
    for fieldname in getattr(msg, '__slots__'):
        m = getattr(msg, fieldname)

        if type(m) in [ types.IntType, types.LongType, types.FloatType, types.BooleanType ]:
            count = count + 1
            if base:
                typeStr.append("%d- %s" % (count, fieldname))
            else:
                typeStr.append(fieldname)
        elif type(m) in [ types.ListType, types.TupleType ]:
            # convert to a numpy array
            arr = numpy.array(m)
            # check the data type of the array
            if arr.dtype.kind in "bif":
                # numeric data type

                if base:
                    typeStr.append("%d- %s(%d)" % (count + 1, fieldname, len(arr.ravel())))
                else:
                    typeStr.append("%s(%d)" % (fieldname, len(arr.ravel())))
                count = count + len(arr.ravel())
            elif arr.dtype.kind == "O":
                # compound data type
                subStr, subCount = make_lcmtype_string(m[0], False)
                numSub = len(m)
                if base:
                    subStr = "%d- %s<%s>(%d)" % (count + 1, fieldname, ", ".join(subStr), numSub)
                else:
                    subStr = "%s<%s>(%d)" % (fieldname, ", ".join(subStr), numSub)
                typeStr.append(subStr)
                count = count + numSub * subCount
                #pass
        elif type(m) in types.StringTypes:
            # ignore strings
            pass
        else:
            subStr, subCount = make_lcmtype_string(m, False);
            if base:
                for s in subStr:
                    count = count + 1
                    typeStr.append("%d- %s.%s" % (count, fieldname , s))
            else:
                count = count + subCount
                for s in subStr:
                    typeStr.append(fieldname + "." + s)

    return typeStr, count

def deleteStatusMsg(statMsg):
    if statMsg:
        sys.stderr.write("\r")
        sys.stderr.write(" " * (len(statMsg)))
        sys.stderr.write("\r")
    return ""

longOpts = ["help", "print", "format", "separator", "channelsToProcess", "ignore", "outfile", "lcm_packages"]

try:
    opts, args = getopt.gnu_getopt(sys.argv[1:], "hpvfms:c:i:o:l:", longOpts)
except getopt.GetoptError, err:
    # print help information and exit:
    print str(err) # will print something like "option -a not recognized"
    usage()
if len(args) != 1:
    usage()
#default options
fname = args[0]
lcm_packages = [ "botlcm"]
outFname = fname
outFname = outFname.replace(".", "_")

# only replace -'s with _'s in the filename
outDir = os.path.dirname(outFname)
outFile = os.path.basename(outFname)

outFname = os.path.join(outDir, outFile.replace("-", "_"))
#outFname = outFname.replace("-", "_")

outFname = outFname + ".mat"
printFname = "stdout"
printFile = sys.stdout
verbose = False
printOutput = False
printFormat = False
channelsToIgnore = ""
checkIgnore = False
channelsToProcess = ".*"
separator = ' '
output_m_file = True
for o, a in opts:
    if o == "-v":
        verbose = True
    elif o in ("-h", "--help"):
        usage()
    elif o in ("-p", "--print"):
        printOutput = True
    elif o in ("-f", "--format"):
        printFormat = True
    elif o in ("-s", "--separator="):
        separator = a
    elif o in ("-o", "--outfile="):
        outFname = a
        printFname = a
    elif o in ("-c", "--channelsToProcess="):
        channelsToProcess = a
    elif o in ("-i", "--ignore="):
        channelsToIgnore = a
        checkIgnore = True
    elif o in ("-l", "--lcm_packages="):
        lcm_packages = a.split(",")
    elif o in ("-m", "--no-m-file"):
        output_m_file = False
    else:
        assert False, "unhandled option"

fullPathName = os.path.abspath(outFname)
dirname = os.path.dirname(fullPathName)
outBaseName = os.path.basename(outFname).split(".")[0]
fullBaseName = dirname + "/" + outBaseName

type_db = LCMTypeDatabase(lcm_packages, verbose)

channelsToProcess = re.compile(channelsToProcess)
channelsToIgnore = re.compile(channelsToIgnore)
log = EventLog(fname, "r")

if printOutput:
    sys.stderr.write("opened % s, printing output to %s \n" % (fname, printFname))
    if printFname == "stdout":
        printFile = sys.stdout
    else:
        printFile = open(printFname, "w")
else:
    sys.stderr.write("opened % s, outputing to % s\n" % (fname, outFname))

ignored_channels = []
msgCount = 0
statusMsg = ""
startTime = 0

for e in log:
    if msgCount == 0:
        startTime = e.timestamp

    if e.channel in ignored_channels:
        continue
    if ((checkIgnore and channelsToIgnore.match(e.channel) and len(channelsToIgnore.match(e.channel).group())==len(e.channel)) \
         or (not channelsToProcess.match(e.channel))):
        if verbose:
            sys.stderr.write("ignoring channel %s\n" % e.channel)
        ignored_channels.append(e.channel)
        continue

    lcmtype = type_db.find_type(e.data[:8])
    if not lcmtype:
        if verbose:
            sys.stderr.write("ignoring channel %s\n" % e.channel)
        ignored_channels.append(e.channel)
        continue
    try:
        msg = lcmtype.decode(e.data)
    except:
        sys.stderr.write("error: couldn't decode msg on channel %s" % e.channel)
        continue

    msgCount = msgCount + 1
    if (msgCount % 5000) == 0:
        statusMsg = deleteStatusMsg(statusMsg)
        statusMsg = "read % d messages, % d %% done" % (msgCount, log.tell() / float(log.size())*100)
        sys.stderr.write(statusMsg)
        sys.stderr.flush()

    if e.channel in flatteners:
        flattener = flatteners[e.channel]
    else:
        flattener = make_flattener(msg)
        flatteners[e.channel] = flattener
        data[e.channel.replace("-","_")] = []
        if printFormat:
            statusMsg = deleteStatusMsg(statusMsg)
            typeStr, fieldCount = make_lcmtype_string(msg)
            typeStr.append("%d- log_timestamp" % (fieldCount + 1))

            typeStr = "\n#%s  %s :\n#[\n#%s\n#]\n" % (e.channel, lcmtype, "\n#".join(typeStr))
            sys.stderr.write(typeStr)



    a = flattener(msg)
    #in case the initial flattener didn't work for whatever reason :-/
    # convert to a numpy array
    arr = numpy.array(a)
    # check the data type of the array
    if not(arr.dtype.kind in "bif"):
        statusMsg = deleteStatusMsg(statusMsg)
        sys.stderr.write("WARNING: needed to create new flattener for channel %s\n" % (e.channel))
        flattener = make_flattener(msg)
        flatteners[e.channel] = flattener
        a = flattener(msg)

    a.append((e.timestamp - startTime) / 1e6)
    if printOutput:
        printFile.write("%s%s%s\n" % (e.channel, separator, separator.join([str(k) for k in a])))
    else:
        # change all "-" to "_" in the channel names, otherwise we'll have inaccessible matlab variables
        data[e.channel.replace("-","_")].append(a)




deleteStatusMsg(statusMsg)
if not printOutput:
    #need to pad variable length messages with zeros...
    for chan in data:
        lengths = map(len, data[chan])
        maxLen = max(lengths)
        minLen = min(lengths)
        if maxLen != minLen:
            sys.stderr.write("padding channel %s with zeros, messages ranged from %d to %d \n" % (chan, minLen, maxLen))
            count = 0
            for i in data[chan]:
                pad = numpy.zeros(maxLen - lengths[count])
                i.extend(pad)
                count = count + 1


    sys.stderr.write("loaded all %d messages, saving to % s\n" % (msgCount, outFname))

    if sys.version_info < (2, 6):
        scipy.io.mio.savemat(outFname, data)
    else:
        scipy.io.matlab.mio.savemat(outFname, data)

    if output_m_file:
        mfile = open(dirname + "/" + outBaseName + ".m", "w")
        loadFunc = """function [d imFnames]=%s()
full_fname = '%s';
fname = '%s';
if (exist(full_fname,'file'))
    filename = full_fname;
else
    filename = fname;
end
d = load(filename);
""" % (outBaseName, outFname, fullPathName)

        mfile.write(loadFunc);
        mfile.close()
