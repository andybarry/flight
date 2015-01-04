#!/bin/bash

make clean

# clean up java stuff
rm -r lcmtypes

rm LCMtypes.jar

# clean up python stuff

for FILE in `ls types/*.lcm`
do
    # replace "types/" in path with "" and ".lcm" with ".py"
    FILE2=${FILE/types\//}

    # % says "search from back"
    rm ${FILE2/%lcm/py}
    rm ${FILE2/%lcm/pyc}
done
