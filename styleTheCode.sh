#! /bin/sh
find . -not -wholename '*libcreate/*' -and '(' -name '*.h' -or -name '*.hpp' -or -name '*.cpp' ')' | xargs clang-format -i -style=file $1
