#!/bin/bash

echo "Installing googletest"
echo "======================================================"
echo "If this fails, follow instructions from https://github.com/google/googletest/blob/main/googletest/README.md"
echo "======================================================"

if test -d "/usr/local/include/gtest"; then
    echo "/usr/local/include/gtest exists, no need to reinstall"
    exit
fi

git clone https://github.com/google/googletest.git -b v1.14.0
cd googletest
mkdir build
cd build
cmake ..
make
sudo make install    # Install in /usr/local/ by default

echo "===================================="
echo "If installation is successfull, use"
echo "        rm -rf googletest"
echo "to delete the installation files"
echo "===================================="

echo "===================================="
echo " If failing un-expectedly after     "
echo "   modifying install-googletest.sh  "
echo "  review the comments at the bottom "
echo "===================================="

##Note, if you modify this file, you may need to re-enable execution
# https://stackoverflow.com/questions/73077753/git-action-showing-permission-denied-i-have-an-error-when-pushing-my-code-under
# https://github.com/orgs/community/discussions/26239