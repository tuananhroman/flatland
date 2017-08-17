#!/bin/bash -e

sudo apt-get install clang-3.8 clang-format-3.8 clang-tidy-3.8 -y 

# change to the file's directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $DIR/../

# check files are correctly formatted

echo "running clang format..."
# git ls-files | grep -E '\.[ch](pp)?$' | grep -v "thirdparty/" |  xargs clang-format-3.8 --style=file -i && git diff --exit-code || { git reset --hard; false; } 

echo "ci_prebuild.sh completed."