#!/bin/bash

find $CI_SOURCE_PATH/Workspace/src -name '*.cpp' -name '*.h' -name '*.hpp' | xargs -n1 clang-format -style=file -output-replacements-xml | grep "<replacement " > $CI_SOURCE_PATH/cppstyle.txt
if [ $? -ne 1 ]  # this should fail (should not find anything)
then
    echo "Incorrect formatting in some C++ files, run clang-format to fix"
    cat $CI_SOURCE_PATH/cppstyle.txt
    exit 1;
fi

sudo pip install -r $CI_SOURCE_PATH/ci/requirements.txt
find $CI_SOURCE_PATH/Workspace/src -path $CI_SOURCE_PATH/Workspace/src/dependencies -prune -o -name '*.py' -exec pycodestyle {} \; | grep ".*" > $CI_SOURCE_PATH/pystyle.txt
if [ $? -ne 1 ]  # this should fail (should not find anything)
then
    echo "Incorrect formatting in some Python files, check formatting using pycodestyle"
    cat $CI_SOURCE_PATH/pystyle.txt
    exit 1;
fi
